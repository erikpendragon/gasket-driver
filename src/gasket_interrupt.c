// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018 Google, Inc. */

#include "gasket_interrupt.h"

#include "gasket_constants.h"
#include "gasket_core.h"
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#ifdef GASKET_KERNEL_TRACE_SUPPORT
#define CREATE_TRACE_POINTS
#include <trace/events/gasket_interrupt.h>
#else
#define trace_gasket_interrupt_event(x, ...)
#endif
/* Retry attempts if the requested number of interrupts aren't available. */
/* The dev_id of one MSI-X vector's handler. */
struct gasket_msix_vector {
	struct gasket_interrupt_data *interrupt_data;
	int index;
};

/* Instance interrupt management data. */
struct gasket_interrupt_data {
	/* The name associated with this interrupt data. */
	const char *name;

	/* Interrupt type. See gasket_interrupt_type in gasket_core.h */
	int type;

	/* The PCI device [if any] associated with the owning device. */
	struct pci_dev *pci_dev;

	/* Set to 1 if MSI-X has successfully been configred, 0 otherwise. */
	int msix_configured;

	/* The number of interrupts requested by the owning device. */
	int num_interrupts;

	/* A pointer to the interrupt descriptor struct for this device. */
	const struct gasket_interrupt_desc *interrupts;

	/* The index of the bar into which interrupts should be mapped. */
	int interrupt_bar_index;

	/* The width of a single interrupt in a packed interrupt register. */
	int pack_width;

	/* The number of successfully configured interrupts. */
	int num_configured;

	/* Per-vector handler cookie: which interrupt a vector is. */
	struct gasket_msix_vector *vectors;

	/* The eventfd "callback" data for each interrupt. */
	struct eventfd_ctx **eventfd_ctxs;

	/* Spinlock to protect read/write races to eventfd_ctxs. */
	rwlock_t eventfd_ctx_lock;

	/* The number of times each interrupt has been called. */
	ulong *interrupt_counts;

	/* Linux IRQ number. */
	int irq;
};


/* Set up device registers for interrupt handling. */
static void gasket_interrupt_setup(struct gasket_dev *gasket_dev)
{
	int i;
	int pack_shift;
	u64 mask;
	u64 value;
	struct gasket_interrupt_data *interrupt_data =
		gasket_dev->interrupt_data;

	if (!interrupt_data) {
		dev_dbg(gasket_dev->dev, "Interrupt data is not initialized\n");
		return;
	}

	dev_dbg(gasket_dev->dev, "Running interrupt setup\n");

	if (interrupt_data->type == DEVICE_MANAGED)
		return; /* device driver handles setup */

	/* Setup the MSIX table. */

	for (i = 0; i < interrupt_data->num_interrupts; i++) {
		/*
		 * If the interrupt is not packed, we can write the index into
		 * the register directly. If not, we need to deal with a read-
		 * modify-write and shift based on the packing index.
		 */
		dev_dbg(gasket_dev->dev,
			"Setting up interrupt index %d with index 0x%llx and "
			"packing %d\n",
			interrupt_data->interrupts[i].index,
			interrupt_data->interrupts[i].reg,
			interrupt_data->interrupts[i].packing);
		if (interrupt_data->interrupts[i].packing == UNPACKED) {
			value = interrupt_data->interrupts[i].index;
		} else {
			switch (interrupt_data->interrupts[i].packing) {
			case PACK_0:
				pack_shift = 0;
				break;
			case PACK_1:
				pack_shift = interrupt_data->pack_width;
				break;
			case PACK_2:
				pack_shift = 2 * interrupt_data->pack_width;
				break;
			case PACK_3:
				pack_shift = 3 * interrupt_data->pack_width;
				break;
			default:
				dev_dbg(gasket_dev->dev,
					"Found interrupt description with "
					"unknown enum %d\n",
					interrupt_data->interrupts[i].packing);
				return;
			}

			mask = ~(0xFFFF << pack_shift);
			value = gasket_dev_read_64(gasket_dev,
						   interrupt_data->interrupt_bar_index,
						   interrupt_data->interrupts[i].reg);
			value &= mask;
			value |= interrupt_data->interrupts[i].index
				 << pack_shift;
		}
		gasket_dev_write_64(gasket_dev, value,
				    interrupt_data->interrupt_bar_index,
				    interrupt_data->interrupts[i].reg);
	}
}

void
gasket_handle_interrupt(struct gasket_interrupt_data *interrupt_data,
			int interrupt_index)
{
	struct eventfd_ctx *ctx;

	trace_gasket_interrupt_event(interrupt_data->name, interrupt_index);
	read_lock(&interrupt_data->eventfd_ctx_lock);
	ctx = interrupt_data->eventfd_ctxs[interrupt_index];
	if (ctx)
		eventfd_signal(ctx);
	read_unlock(&interrupt_data->eventfd_ctx_lock);

	++(interrupt_data->interrupt_counts[interrupt_index]);
}

static irqreturn_t gasket_msix_interrupt_handler(int irq, void *dev_id)
{
	struct gasket_msix_vector *vector = dev_id;

	gasket_handle_interrupt(vector->interrupt_data, vector->index);
	return IRQ_HANDLED;
}

/*
 * Allocate exactly num_interrupts MSI-X vectors and request each one.
 * Registered eventfds are kept, so this can re-run after resume.
 */
static int
gasket_interrupt_msix_init(struct gasket_interrupt_data *interrupt_data)
{
	int ret;
	int i;

	interrupt_data->vectors = kcalloc(interrupt_data->num_interrupts,
					  sizeof(*interrupt_data->vectors),
					  GFP_KERNEL);
	if (!interrupt_data->vectors)
		return -ENOMEM;

	ret = pci_alloc_irq_vectors(interrupt_data->pci_dev,
				    interrupt_data->num_interrupts,
				    interrupt_data->num_interrupts,
				    PCI_IRQ_MSIX);
	if (ret < 0)
		return ret;
	interrupt_data->msix_configured = 1;

	for (i = 0; i < interrupt_data->num_interrupts; i++) {
		int irq = pci_irq_vector(interrupt_data->pci_dev, i);

		interrupt_data->vectors[i].interrupt_data = interrupt_data;
		interrupt_data->vectors[i].index = i;
		ret = request_irq(irq, gasket_msix_interrupt_handler, 0,
				  interrupt_data->name,
				  &interrupt_data->vectors[i]);
		if (ret) {
			dev_err(&interrupt_data->pci_dev->dev,
				"Cannot get IRQ for interrupt %d, vector %d; %d\n",
				i, irq, ret);
			return ret;
		}

		interrupt_data->num_configured++;
	}

	return 0;
}

/*
 * On QCM DragonBoard, we exit gasket_interrupt_msix_init() and kernel interrupt
 * setup code with MSIX vectors masked. This is wrong because nothing else in
 * the driver will normally touch the MSIX vectors.
 *
 * As a temporary hack, force unmasking there.
 *
 * TODO: Figure out why QCM kernel doesn't unmask the MSIX vectors, after
 * gasket_interrupt_msix_init(), and remove this code.
 */
static void force_msix_interrupt_unmasking(struct gasket_dev *gasket_dev)
{
	int i;
#define MSIX_VECTOR_SIZE 16
#define MSIX_MASK_BIT_OFFSET 12
#define APEX_BAR2_REG_KERNEL_HIB_MSIX_TABLE 0x46800
	for (i = 0; i < gasket_dev->interrupt_data->num_configured; i++) {
		/* Check if the MSIX vector is unmasked */
		ulong location = APEX_BAR2_REG_KERNEL_HIB_MSIX_TABLE +
				 MSIX_MASK_BIT_OFFSET + i * MSIX_VECTOR_SIZE;
		u32 mask =
			gasket_dev_read_32(gasket_dev,
					   gasket_dev->interrupt_data->interrupt_bar_index,
					   location);
		if (!(mask & 1))
			continue;
		/* Unmask the msix vector (clear 32 bits) */
		gasket_dev_write_32(gasket_dev, 0,
				    gasket_dev->interrupt_data->interrupt_bar_index,
				    location);
	}
#undef MSIX_VECTOR_SIZE
#undef MSIX_MASK_BIT_OFFSET
#undef APEX_BAR2_REG_KERNEL_HIB_MSIX_TABLE
}

/* Show the per-interrupt counters (sysfs "interrupt_counts"). */
ssize_t gasket_interrupt_counts_show(struct gasket_dev *gasket_dev, char *buf)
{
	struct gasket_interrupt_data *interrupt_data = gasket_dev->interrupt_data;
	ssize_t ret = 0;
	int i;

	if (!interrupt_data)
		return -ENODEV;

	for (i = 0; i < interrupt_data->num_interrupts; ++i)
		ret += sysfs_emit_at(buf, ret, "0x%02x: %ld\n", i,
				     interrupt_data->interrupt_counts[i]);
	return ret;
}

int gasket_interrupt_init(struct gasket_dev *gasket_dev)
{
	int ret;
	struct gasket_interrupt_data *interrupt_data;
	const struct gasket_driver_desc *driver_desc =
		gasket_get_driver_desc(gasket_dev);

	interrupt_data = kzalloc(sizeof(struct gasket_interrupt_data),
				 GFP_KERNEL);
	if (!interrupt_data)
		return -ENOMEM;
	gasket_dev->interrupt_data = interrupt_data;
	interrupt_data->name = driver_desc->name;
	interrupt_data->type = driver_desc->interrupt_type;
	interrupt_data->pci_dev = gasket_dev->pci_dev;
	interrupt_data->num_interrupts = driver_desc->num_interrupts;
	interrupt_data->interrupts = driver_desc->interrupts;
	interrupt_data->interrupt_bar_index = driver_desc->interrupt_bar_index;
	interrupt_data->pack_width = driver_desc->interrupt_pack_width;

	interrupt_data->eventfd_ctxs = kcalloc(driver_desc->num_interrupts,
					       sizeof(struct eventfd_ctx *),
					       GFP_KERNEL);
	if (!interrupt_data->eventfd_ctxs) {
		kfree(interrupt_data);
		return -ENOMEM;
	}

	interrupt_data->interrupt_counts = kcalloc(driver_desc->num_interrupts,
						   sizeof(ulong),
						   GFP_KERNEL);
	if (!interrupt_data->interrupt_counts) {
		kfree(interrupt_data->eventfd_ctxs);
		kfree(interrupt_data);
		return -ENOMEM;
	}

	rwlock_init(&interrupt_data->eventfd_ctx_lock);

	switch (interrupt_data->type) {
	case PCI_MSIX:
		ret = gasket_interrupt_msix_init(interrupt_data);
		if (ret)
			break;
		force_msix_interrupt_unmasking(gasket_dev);
		break;

	case DEVICE_MANAGED:  /* Device driver manages IRQ init */
		interrupt_data->num_configured = interrupt_data->num_interrupts;
		ret = 0;
		break;

	default:
		ret = -EINVAL;
	}

	if (ret) {
		/* Failing to setup interrupts will cause the device to report
		 * GASKET_STATUS_LAMED. But it is not fatal.
		 */
		dev_warn(gasket_dev->dev,
			 "Couldn't initialize interrupts: %d\n", ret);
		return 0;
	}

	gasket_interrupt_setup(gasket_dev);

	return 0;
}
EXPORT_SYMBOL(gasket_interrupt_init);

void gasket_interrupt_msix_cleanup(struct gasket_interrupt_data *interrupt_data)
{
	int i;

	for (i = 0; i < interrupt_data->num_configured; i++)
		free_irq(pci_irq_vector(interrupt_data->pci_dev, i),
			 &interrupt_data->vectors[i]);
	interrupt_data->num_configured = 0;

	if (interrupt_data->msix_configured)
		pci_free_irq_vectors(interrupt_data->pci_dev);
	interrupt_data->msix_configured = 0;
	kfree(interrupt_data->vectors);
	interrupt_data->vectors = NULL;
}
EXPORT_SYMBOL(gasket_interrupt_msix_cleanup);

int gasket_interrupt_reinit(struct gasket_dev *gasket_dev)
{
	int ret;

	if (!gasket_dev->interrupt_data) {
		dev_dbg(gasket_dev->dev,
			"Attempted to reinit uninitialized interrupt data\n");
		return -EINVAL;
	}

	switch (gasket_dev->interrupt_data->type) {
	case PCI_MSIX:
		gasket_interrupt_msix_cleanup(gasket_dev->interrupt_data);
		ret = gasket_interrupt_msix_init(gasket_dev->interrupt_data);
		if (ret)
			break;
		force_msix_interrupt_unmasking(gasket_dev);
		break;

	case DEVICE_MANAGED: /* Device driver manages IRQ reinit */
		ret = 0;
		break;

	default:
		ret = -EINVAL;
	}

	if (ret) {
		/* Failing to setup interrupts will cause the device
		 * to report GASKET_STATUS_LAMED, but is not fatal.
		 */
		dev_warn(gasket_dev->dev, "Couldn't reinit interrupts: %d\n",
			 ret);
		return 0;
	}

	gasket_interrupt_setup(gasket_dev);

	return 0;
}
EXPORT_SYMBOL(gasket_interrupt_reinit);

/* See gasket_interrupt.h for description. */
int gasket_interrupt_reset_counts(struct gasket_dev *gasket_dev)
{
	dev_dbg(gasket_dev->dev, "Clearing interrupt counts\n");
	memset(gasket_dev->interrupt_data->interrupt_counts, 0,
	       gasket_dev->interrupt_data->num_interrupts *
			sizeof(*gasket_dev->interrupt_data->interrupt_counts));
	return 0;
}

/* See gasket_interrupt.h for description. */
void gasket_interrupt_cleanup(struct gasket_dev *gasket_dev)
{
	struct gasket_interrupt_data *interrupt_data =
		gasket_dev->interrupt_data;
	int i;

	/*
	 * It is possible to get an error code from gasket_interrupt_init
	 * before interrupt_data has been allocated, so check it.
	 */
	if (!interrupt_data)
		return;

	/* Readers of interrupt_data (sysfs) take the device mutex. */
	mutex_lock(&gasket_dev->mutex);
	gasket_dev->interrupt_data = NULL;
	mutex_unlock(&gasket_dev->mutex);

	switch (interrupt_data->type) {
	case PCI_MSIX:
		gasket_interrupt_msix_cleanup(interrupt_data);
		break;

	case DEVICE_MANAGED: /* Device driver manages IRQ cleanup */
		break;

	default:
		break;
	}

	/* Drop every eventfd reference, configured vector or not. */
	for (i = 0; i < interrupt_data->num_interrupts; i++)
		gasket_interrupt_clear_eventfd(interrupt_data, i);

	kfree(interrupt_data->interrupt_counts);
	kfree(interrupt_data->eventfd_ctxs);
	kfree(interrupt_data);
}

int gasket_interrupt_system_status(struct gasket_dev *gasket_dev)
{
	if (!gasket_dev->interrupt_data) {
		dev_dbg(gasket_dev->dev, "Interrupt data is null\n");
		return GASKET_STATUS_DEAD;
	}

	if (gasket_dev->interrupt_data->num_configured !=
		gasket_dev->interrupt_data->num_interrupts) {
		dev_dbg(gasket_dev->dev,
			"Not all interrupts were configured\n");
		return GASKET_STATUS_LAMED;
	}

	return GASKET_STATUS_ALIVE;
}

int gasket_interrupt_set_eventfd(struct gasket_interrupt_data *interrupt_data,
				 int interrupt, int event_fd)
{
	struct eventfd_ctx *ctx;
	ulong flags;

	if (interrupt < 0 || interrupt >= interrupt_data->num_interrupts)
		return -EINVAL;

	ctx = eventfd_ctx_fdget(event_fd);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	/* Put the old eventfd ctx before setting, else we leak the ref. */
	write_lock_irqsave(&interrupt_data->eventfd_ctx_lock, flags);
	if (interrupt_data->eventfd_ctxs[interrupt] != NULL)
		eventfd_ctx_put(interrupt_data->eventfd_ctxs[interrupt]);
	interrupt_data->eventfd_ctxs[interrupt] = ctx;
	write_unlock_irqrestore(&interrupt_data->eventfd_ctx_lock, flags);
	return 0;
}

/*
 * Signal every registered eventfd once. Used on removal: a process blocked
 * waiting for a completion interrupt that will now never come wakes up, finds
 * the device gone, and can exit instead of hanging.
 */
void gasket_interrupt_wake_all(struct gasket_dev *gasket_dev)
{
	struct gasket_interrupt_data *interrupt_data = gasket_dev->interrupt_data;
	ulong flags;
	int i;

	if (!interrupt_data)
		return;

	read_lock_irqsave(&interrupt_data->eventfd_ctx_lock, flags);
	for (i = 0; i < interrupt_data->num_interrupts; i++)
		if (interrupt_data->eventfd_ctxs[i])
			eventfd_signal(interrupt_data->eventfd_ctxs[i]);
	read_unlock_irqrestore(&interrupt_data->eventfd_ctx_lock, flags);
}

int gasket_interrupt_clear_eventfd(struct gasket_interrupt_data *interrupt_data,
				   int interrupt)
{
	ulong flags;

	if (interrupt < 0 || interrupt >= interrupt_data->num_interrupts)
		return -EINVAL;

	/* Put the old eventfd ctx before clearing, else we leak the ref. */
	write_lock_irqsave(&interrupt_data->eventfd_ctx_lock, flags);
	if (interrupt_data->eventfd_ctxs[interrupt] != NULL)
		eventfd_ctx_put(interrupt_data->eventfd_ctxs[interrupt]);
	interrupt_data->eventfd_ctxs[interrupt] = NULL;
	write_unlock_irqrestore(&interrupt_data->eventfd_ctx_lock, flags);
	return 0;
}
