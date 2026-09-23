// SPDX-License-Identifier: GPL-2.0
/*
 * Gasket generic driver framework. This file contains the implementation
 * for the Gasket generic driver framework - the functionality that is common
 * across Gasket devices.
 *
 * Copyright (C) 2018 Google, Inc.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "gasket_core.h"
#include "gasket_compat.h"

#include "gasket_interrupt.h"
#include "gasket_ioctl.h"
#include "gasket_page_table.h"

#include <linux/capability.h>
#include <linux/compat.h>
#include <linux/vmalloc.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/pid_namespace.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/sched.h>


#ifdef GASKET_KERNEL_TRACE_SUPPORT
#define CREATE_TRACE_POINTS
#include <trace/events/gasket_mmap.h>
#else
#define trace_gasket_mmap_exit(x)
#define trace_gasket_mmap_entry(x, ...)
#endif

/*
 * "Private" members of gasket_driver_desc.
 *
 * Contains internal per-device type tracking data, i.e., data not appropriate
 * as part of the public interface for the generic framework.
 */
struct gasket_internal_desc {
	/* Device-specific-driver-provided configuration information. */
	const struct gasket_driver_desc *driver_desc;

	/* Protects access to per-driver data (i.e. this structure). */
	struct mutex mutex;

	/* Kernel-internal device class. */
	struct class *class;

	/* Instantiated / present devices of this type. */
	struct gasket_dev *devs[GASKET_DEV_MAX];

	/* sysfs groups for the class devices: framework, driver, NULL. */
	const struct attribute_group *groups[3];
};

static void gasket_dev_release(struct device *dev);

/* One open file of a Gasket device (see gasket_dev.open_files). */
struct gasket_open_file {
	struct list_head node;
	struct file *filp;
};

/* do_map_region() needs be able to return more than just true/false. */
enum do_map_region_status {
	/* The region was successfully mapped. */
	DO_MAP_REGION_SUCCESS,

	/* Attempted to map region and failed. */
	DO_MAP_REGION_FAILURE,

	/* The requested region to map was not part of a mappable region. */
	DO_MAP_REGION_INVALID,
};

/* Global data definitions. */
/* Mutex - only for framework-wide data. Other data should be protected by
 * finer-grained locks.
 */
static DEFINE_MUTEX(g_mutex);

/* List of all registered device descriptions & their supporting data. */
static struct gasket_internal_desc g_descs[GASKET_FRAMEWORK_DESC_MAX];

/* Mapping of statuses to human-readable strings. Must end with {0,NULL}. */
static const struct gasket_num_name gasket_status_name_table[] = {
	{ GASKET_STATUS_DEAD, "DEAD" },
	{ GASKET_STATUS_ALIVE, "ALIVE" },
	{ GASKET_STATUS_LAMED, "LAMED" },
	{ GASKET_STATUS_DRIVER_EXIT, "DRIVER_EXITING" },
	{ 0, NULL },
};

/* Enumeration of the automatic Gasket framework sysfs nodes. */
enum gasket_sysfs_attribute_type {
	ATTR_BAR_OFFSETS,
	ATTR_BAR_SIZES,
	ATTR_DRIVER_VERSION,
	ATTR_FRAMEWORK_VERSION,
	ATTR_DEVICE_TYPE,
	ATTR_HARDWARE_REVISION,
	ATTR_PCI_ADDRESS,
	ATTR_STATUS,
	ATTR_IS_DEVICE_OWNED,
	ATTR_DEVICE_OWNER,
	ATTR_WRITE_OPEN_COUNT,
	ATTR_RESET_COUNT,
	ATTR_USER_MEM_RANGES,
	ATTR_INTERRUPT_COUNTS,
};

/* On some arm64 systems pcie dma controller can only access lower 4GB of
 * addresses. Unfortunately vendor BSP isn't providing any means of determining
 * this limitation and there're no errors reported if access to higher addresses
 * if being done. This parameter allows to workaround this issue by pretending
 * that our device only supports 32 bit addresses. This in turn will cause
 * dma driver to use shadow buffers located in low 32 bit address space.
 */
static int dma_bit_mask = 64;
module_param(dma_bit_mask, int, 0644);

/* Perform a standard Gasket callback. */
static inline int
check_and_invoke_callback(struct gasket_dev *gasket_dev,
			  int (*cb_function)(struct gasket_dev *))
{
	int ret = 0;

	if (cb_function) {
		mutex_lock(&gasket_dev->mutex);
		ret = cb_function(gasket_dev);
		mutex_unlock(&gasket_dev->mutex);
	}
	return ret;
}

/* Perform a standard Gasket callback without grabbing gasket_dev->mutex. */
static inline int
gasket_check_and_invoke_callback_nolock(struct gasket_dev *gasket_dev,
					int (*cb_function)(struct gasket_dev *))
{
	int ret = 0;

	if (cb_function)
		ret = cb_function(gasket_dev);
	return ret;
}

/*
 * Return nonzero if the gasket_cdev_info is owned by the current thread group
 * ID.
 */
static int gasket_owned_by_current_tgid(struct gasket_cdev_info *info)
{
	return (info->ownership.is_owned &&
		(info->ownership.owner == current->tgid));
}

/*
 * Find the next free gasket_internal_dev slot.
 *
 * Returns the located slot number on success or a negative number on failure.
 */
/* Called with internal_desc->mutex held; the caller publishes the slot. */
static int gasket_find_dev_slot(struct gasket_internal_desc *internal_desc,
				const char *kobj_name)
{
	int i;

	lockdep_assert_held(&internal_desc->mutex);

	/* Search for a previous instance of this device. */
	for (i = 0; i < GASKET_DEV_MAX; i++) {
		if (internal_desc->devs[i] &&
		    strcmp(internal_desc->devs[i]->kobj_name, kobj_name) == 0) {
			pr_err("Duplicate device %s\n", kobj_name);
			return -EBUSY;
		}
	}

	/* Find a free device slot. */
	for (i = 0; i < GASKET_DEV_MAX; i++) {
		if (!internal_desc->devs[i])
			break;
	}

	if (i == GASKET_DEV_MAX) {
		pr_err("Too many registered devices; max %d\n", GASKET_DEV_MAX);
		return -EBUSY;
	}

	return i;
}

/*
 * Allocate and initialize a Gasket device structure, add the device to the
 * device list.
 *
 * Returns 0 if successful, a negative error code otherwise.
 */
static int gasket_alloc_dev(struct gasket_internal_desc *internal_desc,
			    struct device *parent, struct gasket_dev **pdev)
{
	int dev_idx, ret;
	const struct gasket_driver_desc *driver_desc =
		internal_desc->driver_desc;
	struct gasket_dev *gasket_dev;
	struct gasket_cdev_info *dev_info;
	const char *parent_name = dev_name(parent);

	pr_debug("Allocating a Gasket device, parent %s.\n", parent_name);

	*pdev = NULL;

	gasket_dev = kzalloc(sizeof(*gasket_dev), GFP_KERNEL);
	if (!gasket_dev) {
		pr_err("no memory for device, parent %s\n", parent_name);
		return -ENOMEM;
	}

	/*
	 * Pick and claim the slot in one lock hold, so two parallel probes
	 * cannot get the same index. Lookups by pci_dev skip this entry until
	 * gasket_pci_add_device() sets pci_dev.
	 */
	snprintf(gasket_dev->kobj_name, GASKET_NAME_MAX, "%s", parent_name);
	mutex_lock(&internal_desc->mutex);
	dev_idx = gasket_find_dev_slot(internal_desc, parent_name);
	if (dev_idx >= 0)
		internal_desc->devs[dev_idx] = gasket_dev;
	mutex_unlock(&internal_desc->mutex);
	if (dev_idx < 0) {
		kfree(gasket_dev);
		return dev_idx;
	}

	mutex_init(&gasket_dev->mutex);
	init_rwsem(&gasket_dev->state_sem);
	INIT_LIST_HEAD(&gasket_dev->open_files);

	gasket_dev->internal_desc = internal_desc;
	gasket_dev->dev_idx = dev_idx;
	gasket_dev->dev = get_device(parent);
	gasket_dev->dma_dev = get_device(parent);
	/* gasket_bar_data is uninitialized. */
	gasket_dev->num_page_tables = driver_desc->num_page_tables;
	/* max_page_table_size and *page table are uninit'ed */
	/* interrupt_data is not initialized. */
	/* status is 0, or GASKET_STATUS_DEAD */

	dev_info = &gasket_dev->dev_info;
	snprintf(dev_info->name, GASKET_NAME_MAX, "%s_%u", driver_desc->name,
		 gasket_dev->dev_idx);
	dev_info->devt =
		MKDEV(driver_desc->major, driver_desc->minor +
		      gasket_dev->dev_idx);
	dev_info->gasket_dev_ptr = gasket_dev;

	/* From here on, put_device(&class_dev) frees gasket_dev. */
	device_initialize(&gasket_dev->class_dev);
	gasket_dev->class_dev.class = internal_desc->class;
	gasket_dev->class_dev.parent = parent;
	gasket_dev->class_dev.devt = dev_info->devt;
	gasket_dev->class_dev.groups = internal_desc->groups;
	gasket_dev->class_dev.release = gasket_dev_release;
	dev_set_drvdata(&gasket_dev->class_dev, gasket_dev);
	ret = dev_set_name(&gasket_dev->class_dev, "%s", dev_info->name);
	if (!ret)
		ret = device_add(&gasket_dev->class_dev);
	if (ret) {
		dev_err(parent, "cannot create %s device %s [ret = %d]\n",
			driver_desc->name, dev_info->name, ret);
		mutex_lock(&internal_desc->mutex);
		internal_desc->devs[dev_idx] = NULL;
		mutex_unlock(&internal_desc->mutex);
		put_device(&gasket_dev->class_dev);
		return ret;
	}
	dev_info->device = &gasket_dev->class_dev;
	*pdev = gasket_dev;

	/* cdev has not yet been added; cdev_added is 0 */
	/* ownership is all 0, indicating no owner or opens. */

	return 0;
}

/*
 * Release of the class device: the last reference is gone, which means the
 * device was removed and every file that had it open has been closed.
 */
static void gasket_dev_release(struct device *dev)
{
	struct gasket_dev *gasket_dev =
		container_of(dev, struct gasket_dev, class_dev);

	put_device(gasket_dev->dev);
	put_device(gasket_dev->dma_dev);
	kfree(gasket_dev);
}

/* Drop a Gasket device from the driver's table and release our reference. */
static void gasket_free_dev(struct gasket_dev *gasket_dev)
{
	struct gasket_internal_desc *internal_desc = gasket_dev->internal_desc;

	mutex_lock(&internal_desc->mutex);
	internal_desc->devs[gasket_dev->dev_idx] = NULL;
	mutex_unlock(&internal_desc->mutex);
	if (device_is_registered(&gasket_dev->class_dev))
		device_del(&gasket_dev->class_dev);
	put_device(&gasket_dev->class_dev);
}

/*
 * Maps the specified bar into kernel space.
 *
 * Returns 0 on success, a negative error code otherwise.
 * A zero-sized BAR will not be mapped, but is not an error.
 */
static int gasket_map_pci_bar(struct gasket_dev *gasket_dev, int bar_num)
{
	struct gasket_internal_desc *internal_desc = gasket_dev->internal_desc;
	const struct gasket_driver_desc *driver_desc =
		internal_desc->driver_desc;
	ulong desc_bytes = driver_desc->bar_descriptions[bar_num].size;
	void __iomem *virt_base;

	if (desc_bytes == 0)
		return 0;

	if (driver_desc->bar_descriptions[bar_num].type != PCI_BAR) {
		/* not PCI: skip this entry */
		return 0;
	}
	/*
	 * pci_resource_start and pci_resource_len return a "resource_size_t",
	 * which is safely castable to ulong (which itself is the arg to
	 * request_mem_region).
	 */
	gasket_dev->bar_data[bar_num].phys_base =
		(ulong)pci_resource_start(gasket_dev->pci_dev, bar_num);
	if (!gasket_dev->bar_data[bar_num].phys_base) {
		dev_err(gasket_dev->dev, "Cannot get BAR%u base address\n",
			bar_num);
		return -EINVAL;
	}

	gasket_dev->bar_data[bar_num].length_bytes =
		(ulong)pci_resource_len(gasket_dev->pci_dev, bar_num);
	if (gasket_dev->bar_data[bar_num].length_bytes < desc_bytes) {
		dev_err(gasket_dev->dev,
			"PCI BAR %u space is too small: %lu; expected >= %lu\n",
			bar_num, gasket_dev->bar_data[bar_num].length_bytes,
			desc_bytes);
		return -ENOMEM;
	}

	/*
	 * Request and map the BAR as a managed resource: it stays mapped
	 * until the driver is unbound, after remove() has stopped every user.
	 */
	/*
	 * The region name is kept by pointer until devres releases it, which
	 * can be after gasket_dev is freed: use the driver's static name.
	 */
	virt_base = pcim_iomap_region(gasket_dev->pci_dev, bar_num,
				      driver_desc->name);
	if (IS_ERR(virt_base)) {
		dev_err(gasket_dev->dev,
			"Cannot map BAR %d memory region %pR [ret=%ld]\n",
			bar_num, &gasket_dev->pci_dev->resource[bar_num],
			PTR_ERR(virt_base));
		return PTR_ERR(virt_base);
	}
	gasket_dev->bar_data[bar_num].virt_base = virt_base;

	return 0;
}

/*
 * Releases PCI BAR mapping.
 *
 * A zero-sized or not-mapped BAR will not be unmapped, but is not an error.
 */
static void gasket_unmap_pci_bar(struct gasket_dev *dev, int bar_num)
{
	struct gasket_internal_desc *internal_desc = dev->internal_desc;
	const struct gasket_driver_desc *driver_desc =
		internal_desc->driver_desc;

	if (driver_desc->bar_descriptions[bar_num].size == 0 ||
	    !dev->bar_data[bar_num].virt_base)
		return;

	if (driver_desc->bar_descriptions[bar_num].type != PCI_BAR)
		return;

	/* The mapping itself is released by devres when the driver unbinds. */
	dev->bar_data[bar_num].virt_base = NULL;
}

/*
 * Setup PCI memory mapping for the specified device.
 *
 * Reads the BAR registers and sets up pointers to the device's memory mapped
 * IO space.
 *
 * Returns 0 on success and a negative value otherwise.
 */
static int gasket_setup_pci(struct pci_dev *pci_dev,
			    struct gasket_dev *gasket_dev)
{
	int i, mapped_bars, ret;

	if (dma_bit_mask < 32 || dma_bit_mask > 64) {
		dev_err(&pci_dev->dev, "dma_bit_mask %d out of range 32..64\n",
			dma_bit_mask);
		return -EINVAL;
	}
	ret = dma_set_mask_and_coherent(&pci_dev->dev,
					DMA_BIT_MASK(dma_bit_mask));
	if (ret) {
		dev_err(&pci_dev->dev, "cannot set %d-bit DMA mask [ret=%d]\n",
			dma_bit_mask, ret);
		return ret;
	}

	for (i = 0; i < GASKET_NUM_BARS; i++) {
		ret = gasket_map_pci_bar(gasket_dev, i);
		if (ret) {
			mapped_bars = i;
			goto fail;
		}
	}

	return 0;

fail:
	for (i = 0; i < mapped_bars; i++)
		gasket_unmap_pci_bar(gasket_dev, i);

	return ret;
}

/* Unmaps memory for the specified device. */
static void gasket_cleanup_pci(struct gasket_dev *gasket_dev)
{
	int i;

	for (i = 0; i < GASKET_NUM_BARS; i++)
		gasket_unmap_pci_bar(gasket_dev, i);
}

/* Determine the health of the Gasket device. */
static int gasket_get_hw_status(struct gasket_dev *gasket_dev)
{
	int status;
	int i;
	const struct gasket_driver_desc *driver_desc =
		gasket_dev->internal_desc->driver_desc;

	status = gasket_check_and_invoke_callback_nolock(gasket_dev,
							 driver_desc->device_status_cb);
	if (status != GASKET_STATUS_ALIVE) {
		dev_dbg(gasket_dev->dev, "Hardware reported status %d.\n",
			status);
		return status;
	}

	status = gasket_interrupt_system_status(gasket_dev);
	if (status != GASKET_STATUS_ALIVE) {
		dev_dbg(gasket_dev->dev,
			"Interrupt system reported status %d.\n", status);
		return status;
	}

	for (i = 0; i < driver_desc->num_page_tables; ++i) {
		status = gasket_page_table_system_status(gasket_dev->page_table[i]);
		if (status != GASKET_STATUS_ALIVE) {
			dev_dbg(gasket_dev->dev,
				"Page table %d reported status %d.\n",
				i, status);
			return status;
		}
	}

	return GASKET_STATUS_ALIVE;
}

static ssize_t
gasket_write_mappable_regions(char *buf, ssize_t at,
			      const struct gasket_driver_desc *driver_desc,
			      int bar_index)
{
	int i;
	ssize_t total_written = 0;
	ulong min_addr, max_addr;
	const struct gasket_bar_desc *bar_desc =
		&driver_desc->bar_descriptions[bar_index];

	if (bar_desc->permissions == GASKET_NOMAP)
		return 0;
	for (i = 0; i < bar_desc->num_mappable_regions; i++) {
		min_addr = bar_desc->mappable_regions[i].start -
			   driver_desc->legacy_mmap_address_offset;
		max_addr = bar_desc->mappable_regions[i].start -
			   driver_desc->legacy_mmap_address_offset +
			   bar_desc->mappable_regions[i].length_bytes;
		total_written += sysfs_emit_at(buf, at + total_written,
					       "0x%08lx-0x%08lx\n",
					       min_addr, max_addr);
	}
	return total_written;
}

static ssize_t gasket_sysfs_data_show(struct device *device,
				      struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	struct gasket_dev *gasket_dev = dev_get_drvdata(device);
	const struct gasket_driver_desc *driver_desc;
	const struct gasket_bar_desc *bar_desc;

	if (!gasket_dev)
		return -ENODEV;

	driver_desc = gasket_dev->internal_desc->driver_desc;

	switch (gasket_attr_type(attr)) {
	case ATTR_BAR_OFFSETS:
		for (i = 0; i < GASKET_NUM_BARS; i++) {
			bar_desc = &driver_desc->bar_descriptions[i];
			if (bar_desc->size == 0)
				continue;
			ret += sysfs_emit_at(buf, ret, "%d: 0x%lx\n", i,
					     (ulong)bar_desc->base);
		}
		break;
	case ATTR_BAR_SIZES:
		for (i = 0; i < GASKET_NUM_BARS; i++) {
			bar_desc = &driver_desc->bar_descriptions[i];
			if (bar_desc->size == 0)
				continue;
			ret += sysfs_emit_at(buf, ret, "%d: 0x%lx\n", i,
					     (ulong)bar_desc->size);
		}
		break;
	case ATTR_DRIVER_VERSION:
		ret = sysfs_emit(buf, "%s\n", driver_desc->driver_version);
		break;
	case ATTR_FRAMEWORK_VERSION:
		ret = sysfs_emit(buf, "%s\n", GASKET_FRAMEWORK_VERSION);
		break;
	case ATTR_DEVICE_TYPE:
		ret = sysfs_emit(buf, "%s\n", driver_desc->name);
		break;
	case ATTR_HARDWARE_REVISION:
		ret = sysfs_emit(buf, "%d\n", gasket_dev->hardware_revision);
		break;
	case ATTR_PCI_ADDRESS:
		ret = sysfs_emit(buf, "%s\n", gasket_dev->kobj_name);
		break;
	case ATTR_STATUS:
		ret = sysfs_emit(buf, "%s\n",
				 gasket_num_name_lookup(gasket_dev->status,
							gasket_status_name_table));
		break;
	case ATTR_IS_DEVICE_OWNED:
		ret = sysfs_emit(buf, "%d\n",
				 gasket_dev->dev_info.ownership.is_owned);
		break;
	case ATTR_DEVICE_OWNER:
		ret = sysfs_emit(buf, "%d\n",
				 gasket_dev->dev_info.ownership.owner);
		break;
	case ATTR_WRITE_OPEN_COUNT:
		ret = sysfs_emit(buf, "%d\n",
				 gasket_dev->dev_info.ownership.write_open_count);
		break;
	case ATTR_RESET_COUNT:
		ret = sysfs_emit(buf, "%d\n", gasket_dev->reset_count);
		break;
	case ATTR_USER_MEM_RANGES:
		for (i = 0; i < GASKET_NUM_BARS; ++i)
			ret += gasket_write_mappable_regions(buf, ret,
							     driver_desc, i);
		break;
	case ATTR_INTERRUPT_COUNTS:
		mutex_lock(&gasket_dev->mutex);
		ret = gasket_interrupt_counts_show(gasket_dev, buf);
		mutex_unlock(&gasket_dev->mutex);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* These attributes apply to all Gasket driver instances. */
static GASKET_ATTR_RO(bar_offsets, gasket_sysfs_data_show, ATTR_BAR_OFFSETS);
static GASKET_ATTR_RO(bar_sizes, gasket_sysfs_data_show, ATTR_BAR_SIZES);
static GASKET_ATTR_RO(driver_version, gasket_sysfs_data_show,
		      ATTR_DRIVER_VERSION);
static GASKET_ATTR_RO(framework_version, gasket_sysfs_data_show,
		      ATTR_FRAMEWORK_VERSION);
static GASKET_ATTR_RO(device_type, gasket_sysfs_data_show, ATTR_DEVICE_TYPE);
static GASKET_ATTR_RO(revision, gasket_sysfs_data_show,
		      ATTR_HARDWARE_REVISION);
static GASKET_ATTR_RO(pci_address, gasket_sysfs_data_show, ATTR_PCI_ADDRESS);
static GASKET_ATTR_RO(status, gasket_sysfs_data_show, ATTR_STATUS);
static GASKET_ATTR_RO(is_device_owned, gasket_sysfs_data_show,
		      ATTR_IS_DEVICE_OWNED);
static GASKET_ATTR_RO(device_owner, gasket_sysfs_data_show, ATTR_DEVICE_OWNER);
static GASKET_ATTR_RO(write_open_count, gasket_sysfs_data_show,
		      ATTR_WRITE_OPEN_COUNT);
static GASKET_ATTR_RO(reset_count, gasket_sysfs_data_show, ATTR_RESET_COUNT);
static GASKET_ATTR_RO(user_mem_ranges, gasket_sysfs_data_show,
		      ATTR_USER_MEM_RANGES);
static GASKET_ATTR_RO(interrupt_counts, gasket_sysfs_data_show,
		      ATTR_INTERRUPT_COUNTS);

static struct attribute *gasket_generic_attrs[] = {
	&gasket_attr_bar_offsets.attr.attr,
	&gasket_attr_bar_sizes.attr.attr,
	&gasket_attr_driver_version.attr.attr,
	&gasket_attr_framework_version.attr.attr,
	&gasket_attr_device_type.attr.attr,
	&gasket_attr_revision.attr.attr,
	&gasket_attr_pci_address.attr.attr,
	&gasket_attr_status.attr.attr,
	&gasket_attr_is_device_owned.attr.attr,
	&gasket_attr_device_owner.attr.attr,
	&gasket_attr_write_open_count.attr.attr,
	&gasket_attr_reset_count.attr.attr,
	&gasket_attr_user_mem_ranges.attr.attr,
	&gasket_attr_interrupt_counts.attr.attr,
	NULL,
};

static const struct attribute_group gasket_generic_group = {
	.attrs = gasket_generic_attrs,
};

/* Add a char device and related info. */
static int gasket_add_cdev(struct gasket_cdev_info *dev_info,
			   const struct file_operations *file_ops,
			   struct module *owner)
{
	int ret;

	cdev_init(&dev_info->cdev, file_ops);
	dev_info->cdev.owner = owner;
	/* Open files keep the class device, and so gasket_dev, alive. */
	cdev_set_parent(&dev_info->cdev,
			&dev_info->gasket_dev_ptr->class_dev.kobj);
	ret = cdev_add(&dev_info->cdev, dev_info->devt, 1);
	if (ret) {
		dev_err(dev_info->gasket_dev_ptr->dev,
			"cannot add char device [ret=%d]\n", ret);
		return ret;
	}
	dev_info->cdev_added = 1;

	return 0;
}

/* Disable device operations. */
void gasket_disable_device(struct gasket_dev *gasket_dev)
{
	const struct gasket_driver_desc *driver_desc =
		gasket_dev->internal_desc->driver_desc;
	int i;

	struct gasket_open_file *of;

	dev_dbg(gasket_dev->dev, "disabling device\n");

	/* Wait for running file operations, then refuse new ones. */
	down_write(&gasket_dev->state_sem);
	WRITE_ONCE(gasket_dev->gone, true);
	up_write(&gasket_dev->state_sem);

	/* Remove sysfs (waits for active readers) and the device node. */
	if (device_is_registered(&gasket_dev->class_dev))
		device_del(&gasket_dev->class_dev);
	if (gasket_dev->dev_info.cdev_added)
		cdev_del(&gasket_dev->dev_info.cdev);

	/* User mappings of the BAR and coherent buffer must not outlive it. */
	mutex_lock(&gasket_dev->mutex);
	list_for_each_entry(of, &gasket_dev->open_files, node)
		unmap_mapping_range(of->filp->f_mapping, 0, 0, 1);
	mutex_unlock(&gasket_dev->mutex);

	/*
	 * Stop the chip before the memory it may be using goes away: reset it
	 * (stops the cores and pauses DMA) if it still answers, then turn off
	 * bus mastering so no DMA can reach host memory at all.
	 */
	if (gasket_dev->pci_dev) {
		if (driver_desc->device_reset_cb &&
		    pci_device_is_present(gasket_dev->pci_dev)) {
			mutex_lock(&gasket_dev->mutex);
			driver_desc->device_reset_cb(gasket_dev);
			/* Leave it in its low-power state, as after close. */
			gasket_check_and_invoke_callback_nolock(gasket_dev,
				driver_desc->device_close_cb);
			mutex_unlock(&gasket_dev->mutex);
		}
		pci_clear_master(gasket_dev->pci_dev);
	}

	gasket_dev->status = GASKET_STATUS_DEAD;

	/*
	 * Wake anyone blocked on a completion interrupt: with the mappings
	 * gone they get SIGBUS on the next register access instead of waiting
	 * forever (found by the VM remove-during-inference test).
	 */
	gasket_interrupt_wake_all(gasket_dev);
	gasket_interrupt_cleanup(gasket_dev);

	/*
	 * Free the coherent buffer while the device is still there. Every
	 * mapping was zapped above, and the fault handler inserts no page
	 * once "gone" is set, so no user PTE can point at it.
	 */
	mutex_lock(&gasket_dev->mutex);
	gasket_free_coherent_buffer(gasket_dev);
	mutex_unlock(&gasket_dev->mutex);

	for (i = 0; i < driver_desc->num_page_tables; ++i) {
		if (gasket_dev->page_table[i]) {
			struct gasket_page_table *pg_tbl;

			gasket_page_table_reset(gasket_dev->page_table[i]);

			mutex_lock(&gasket_dev->mutex);
			pg_tbl = gasket_dev->page_table[i];
			gasket_dev->page_table[i] = NULL;
			mutex_unlock(&gasket_dev->mutex);

			gasket_page_table_cleanup(pg_tbl);
		}
	}
}
EXPORT_SYMBOL(gasket_disable_device);

/*
 * Registered driver descriptor lookup for PCI devices.
 *
 * Precondition: Called with g_mutex held (to avoid a race on return).
 * Returns NULL if no matching device was found.
 */
static struct gasket_internal_desc *
lookup_pci_internal_desc(struct pci_dev *pci_dev)
{
	int i;

	__must_hold(&g_mutex);
	for (i = 0; i < GASKET_FRAMEWORK_DESC_MAX; i++) {
		if (g_descs[i].driver_desc &&
		    g_descs[i].driver_desc->pci_id_table &&
		    pci_match_id(g_descs[i].driver_desc->pci_id_table, pci_dev))
			return &g_descs[i];
	}

	return NULL;
}

/*
 * Registered driver descriptor lookup for platform devices.
 * Caller must hold g_mutex.
 */
static struct gasket_internal_desc *
lookup_platform_internal_desc(struct platform_device *pdev)
{
	int i;

	__must_hold(&g_mutex);
	for (i = 0; i < GASKET_FRAMEWORK_DESC_MAX; i++) {
		if (g_descs[i].driver_desc &&
		    strcmp(g_descs[i].driver_desc->name, pdev->name) == 0)
			return &g_descs[i];
	}

	return NULL;
}

/*
 * Verifies that the user has permissions to perform the requested mapping and
 * that the provided descriptor/range is of adequate size to hold the range to
 * be mapped.
 */
static bool gasket_mmap_has_permissions(struct gasket_dev *gasket_dev,
					struct vm_area_struct *vma,
					int bar_permissions)
{
	int requested_permissions;
	/* Always allow sysadmin to access. */
	if (capable(CAP_SYS_ADMIN))
		return true;

	/* Never allow non-sysadmins to access to a dead device. */
	if (gasket_dev->status != GASKET_STATUS_ALIVE) {
		dev_dbg(gasket_dev->dev, "Device is dead.\n");
		return false;
	}

	/* Make sure that no wrong flags are set. */
	requested_permissions =
		(vma->vm_flags & (VM_WRITE | VM_READ | VM_EXEC));
	if (requested_permissions & ~(bar_permissions)) {
		dev_dbg(gasket_dev->dev,
			"Attempting to map a region with requested permissions "
			"0x%x, but region has permissions 0x%x.\n",
			requested_permissions, bar_permissions);
		return false;
	}

	/* Do not allow a non-owner to write. */
	if ((vma->vm_flags & VM_WRITE) &&
	    !gasket_owned_by_current_tgid(&gasket_dev->dev_info)) {
		dev_dbg(gasket_dev->dev,
			"Attempting to mmap a region for write without owning "
			"device.\n");
		return false;
	}

	return true;
}

/*
 * Verifies that the input address is within the region allocated to coherent
 * buffer.
 */
static bool
gasket_is_coherent_region(const struct gasket_driver_desc *driver_desc,
			  ulong address)
{
	struct gasket_coherent_buffer_desc coh_buff_desc =
		driver_desc->coherent_buffer_description;

	if (coh_buff_desc.permissions != GASKET_NOMAP) {
		if ((address >= coh_buff_desc.base) &&
		    (address < coh_buff_desc.base + coh_buff_desc.size)) {
			return true;
		}
	}
	return false;
}

static int gasket_get_bar_index(const struct gasket_dev *gasket_dev,
				ulong phys_addr)
{
	int i;
	const struct gasket_driver_desc *driver_desc;

	driver_desc = gasket_dev->internal_desc->driver_desc;
	for (i = 0; i < GASKET_NUM_BARS; ++i) {
		struct gasket_bar_desc bar_desc =
			driver_desc->bar_descriptions[i];

		if (bar_desc.permissions != GASKET_NOMAP) {
			if (phys_addr >= bar_desc.base &&
			    phys_addr < (bar_desc.base + bar_desc.size)) {
				return i;
			}
		}
	}
	/* If we haven't found the address by now, it is invalid. */
	return -EINVAL;
}

/*
 * Sets the actual bounds to map, given the device's mappable region.
 *
 * Given the device's mappable region, along with the user-requested mapping
 * start offset and length of the user region, determine how much of this
 * mappable region can be mapped into the user's region (start/end offsets),
 * and the physical offset (phys_offset) into the BAR where the mapping should
 * begin (either the VMA's or region lower bound).
 *
 * In other words, this calculates the overlap between the VMA
 * (bar_offset, requested_length) and the given gasket_mappable_region.
 *
 * Returns true if there's anything to map, and false otherwise.
 */
static bool
gasket_mm_get_mapping_addrs(const struct gasket_mappable_region *region,
			    ulong bar_offset, ulong requested_length,
			    struct gasket_mappable_region *mappable_region,
			    ulong *virt_offset)
{
	ulong range_start = region->start;
	ulong range_length = region->length_bytes;
	ulong range_end = range_start + range_length;

	*virt_offset = 0;
	if (bar_offset + requested_length < range_start) {
		/*
		 * If the requested region is completely below the range,
		 * there is nothing to map.
		 */
		return false;
	} else if (bar_offset <= range_start) {
		/* If the bar offset is below this range's start
		 * but the requested length continues into it:
		 * 1) Only map starting from the beginning of this
		 *      range's phys. offset, so we don't map unmappable
		 *	memory.
		 * 2) The length of the virtual memory to not map is the
		 *	delta between the bar offset and the
		 *	mappable start (and since the mappable start is
		 *	bigger, start - req.)
		 * 3) The map length is the minimum of the mappable
		 *	requested length (requested_length - virt_offset)
		 *	and the actual mappable length of the range.
		 */
		mappable_region->start = range_start;
		*virt_offset = range_start - bar_offset;
		mappable_region->length_bytes =
			min(requested_length - *virt_offset, range_length);
		return true;
	} else if (bar_offset > range_start &&
		   bar_offset < range_end) {
		/*
		 * If the bar offset is within this range:
		 * 1) Map starting from the bar offset.
		 * 2) Because there is no forbidden memory between the
		 *	bar offset and the range start,
		 *	virt_offset is 0.
		 * 3) The map length is the minimum of the requested
		 *	length and the remaining length in the buffer
		 *	(range_end - bar_offset)
		 */
		mappable_region->start = bar_offset;
		*virt_offset = 0;
		mappable_region->length_bytes =
			min(requested_length, range_end - bar_offset);
		return true;
	}

	/*
	 * If the requested [start] offset is above range_end,
	 * there's nothing to map.
	 */
	return false;
}

/*
 * Calculates the offset where the VMA range begins in its containing BAR.
 * The offset is written into bar_offset on success.
 * Returns zero on success, anything else on error.
 */
static int gasket_mm_vma_bar_offset(const struct gasket_dev *gasket_dev,
				    const struct vm_area_struct *vma,
				    ulong *bar_offset)
{
	ulong raw_offset;
	int bar_index;
	const struct gasket_driver_desc *driver_desc =
		gasket_dev->internal_desc->driver_desc;

	raw_offset = (vma->vm_pgoff << PAGE_SHIFT) +
		driver_desc->legacy_mmap_address_offset;
	bar_index = gasket_get_bar_index(gasket_dev, raw_offset);
	if (bar_index < 0) {
		dev_err(gasket_dev->dev,
			"Unable to find matching bar for address 0x%lx\n",
			raw_offset);
		trace_gasket_mmap_exit(bar_index);
		return bar_index;
	}
	*bar_offset =
		raw_offset - driver_desc->bar_descriptions[bar_index].base;

	return 0;
}

int gasket_mm_unmap_region(const struct gasket_dev *gasket_dev,
			   struct vm_area_struct *vma,
			   const struct gasket_mappable_region *map_region)
{
	ulong bar_offset;
	ulong virt_offset;
	struct gasket_mappable_region mappable_region;
	int ret;

	if (map_region->length_bytes == 0)
		return 0;

	ret = gasket_mm_vma_bar_offset(gasket_dev, vma, &bar_offset);
	if (ret)
		return ret;

	if (!gasket_mm_get_mapping_addrs(map_region, bar_offset,
					 vma->vm_end - vma->vm_start,
					 &mappable_region, &virt_offset))
		return 1;

	/*
	 * The length passed to zap_vma_ptes MUST BE A MULTIPLE OF
	 * PAGE_SIZE! Trust me. I have the scars.
	 *
	 * Next multiple of y: ceil_div(x, y) * y
	 */
	gasket_zap_vma_range(vma, vma->vm_start + virt_offset,
			     DIV_ROUND_UP(mappable_region.length_bytes,
					  PAGE_SIZE) * PAGE_SIZE);
	return 0;
}
EXPORT_SYMBOL(gasket_mm_unmap_region);

/* Maps a virtual address + range to a physical offset of a BAR. */
static enum do_map_region_status
do_map_region(const struct gasket_dev *gasket_dev, struct vm_area_struct *vma,
	      struct gasket_mappable_region *mappable_region)
{
	/* Maximum size of a single call to io_remap_pfn_range. */
	/* I pulled this number out of thin air. */
	const ulong max_chunk_size = 64 * 1024 * 1024;
	ulong chunk_size, mapped_bytes = 0;

	const struct gasket_driver_desc *driver_desc =
		gasket_dev->internal_desc->driver_desc;

	ulong bar_offset, virt_offset;
	struct gasket_mappable_region region_to_map;
	ulong phys_offset, map_length;
	ulong virt_base, phys_base;
	int bar_index, ret;

	ret = gasket_mm_vma_bar_offset(gasket_dev, vma, &bar_offset);
	if (ret)
		return DO_MAP_REGION_INVALID;

	if (!gasket_mm_get_mapping_addrs(mappable_region, bar_offset,
					 vma->vm_end - vma->vm_start,
					 &region_to_map, &virt_offset))
		return DO_MAP_REGION_INVALID;
	phys_offset = region_to_map.start;
	map_length = region_to_map.length_bytes;

	virt_base = vma->vm_start + virt_offset;
	bar_index =
		gasket_get_bar_index(gasket_dev,
				     (vma->vm_pgoff << PAGE_SHIFT) +
				     driver_desc->legacy_mmap_address_offset);
	phys_base = gasket_dev->bar_data[bar_index].phys_base + phys_offset;
	while (mapped_bytes < map_length) {
		/*
		 * io_remap_pfn_range can take a while, so we chunk its
		 * calls and call cond_resched between each.
		 */
		chunk_size = min(max_chunk_size, map_length - mapped_bytes);

		cond_resched();
		ret = io_remap_pfn_range(vma, virt_base + mapped_bytes,
					 (phys_base + mapped_bytes) >>
					 PAGE_SHIFT, chunk_size,
					 vma->vm_page_prot);
		if (ret) {
			dev_err(gasket_dev->dev,
				"Error remapping PFN range.\n");
			goto fail;
		}
		mapped_bytes += chunk_size;
	}

	return DO_MAP_REGION_SUCCESS;

fail:
	/* Unmap the partial chunk we mapped. */
	mappable_region->length_bytes = mapped_bytes;
	if (gasket_mm_unmap_region(gasket_dev, vma, mappable_region))
		dev_err(gasket_dev->dev,
			"Error unmapping partial region 0x%lx (0x%lx bytes)\n",
			(ulong)virt_offset,
			(ulong)mapped_bytes);

	return DO_MAP_REGION_FAILURE;
}

/* File offset (pgoff space) and size of the coherent buffer window. */
static void gasket_coherent_window(const struct gasket_dev *gasket_dev,
				   loff_t *start, loff_t *len)
{
	const struct gasket_driver_desc *driver_desc =
		gasket_dev->internal_desc->driver_desc;

	*start = driver_desc->coherent_buffer_description.base -
		 driver_desc->legacy_mmap_address_offset;
	*len = driver_desc->coherent_buffer_description.size;
}

/*
 * Take the coherent buffer away from every user mapping of every open file.
 * Called with the device mutex held, before the buffer is freed; a later
 * access faults, and the fault handler finds no buffer (SIGBUS).
 */
void gasket_zap_coherent_mappings(struct gasket_dev *gasket_dev)
{
	struct gasket_open_file *of;
	loff_t start, len;

	lockdep_assert_held(&gasket_dev->mutex);
	gasket_coherent_window(gasket_dev, &start, &len);
	list_for_each_entry(of, &gasket_dev->open_files, node)
		unmap_mapping_range(of->filp->f_mapping, start, len, 1);
}
EXPORT_SYMBOL(gasket_zap_coherent_mappings);

/*
 * Page fault on a coherent buffer mapping. Pages are inserted one at a time
 * under the device mutex, and only while the buffer exists and the device is
 * present, so freeing the buffer (after gasket_zap_coherent_mappings()) never
 * leaves a user PTE pointing at freed memory.
 */
static vm_fault_t gasket_coherent_vma_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct gasket_dev *gasket_dev = vma->vm_private_data;
	vm_fault_t ret = VM_FAULT_SIGBUS;
	loff_t start, len, off;
	void *virt;

	gasket_coherent_window(gasket_dev, &start, &len);
	off = ((loff_t)vmf->pgoff << PAGE_SHIFT) - start;

	mutex_lock(&gasket_dev->mutex);
	if (READ_ONCE(gasket_dev->gone) ||
	    off < 0 || off >= gasket_dev->coherent_buffer.length_bytes)
		goto out;

	virt = gasket_dev->coherent_buffer.virt_base + off;
	ret = vmf_insert_pfn(vma, vmf->address,
			     is_vmalloc_addr(virt) ? vmalloc_to_pfn(virt) :
						     page_to_pfn(virt_to_page(virt)));
out:
	mutex_unlock(&gasket_dev->mutex);
	return ret;
}

static const struct vm_operations_struct gasket_coherent_vm_ops = {
	.fault = gasket_coherent_vma_fault,
};

/* Map a region of coherent memory. Called with the device mutex held. */
static int gasket_mmap_coherent(struct gasket_dev *gasket_dev,
				struct vm_area_struct *vma)
{
	const struct gasket_driver_desc *driver_desc =
		gasket_dev->internal_desc->driver_desc;
	const ulong requested_length = vma->vm_end - vma->vm_start;
	ulong permissions;
	loff_t start, len, off;

	gasket_coherent_window(gasket_dev, &start, &len);
	off = ((loff_t)vma->vm_pgoff << PAGE_SHIFT) - start;

	if (requested_length == 0 || off < 0 ||
	    off + requested_length > gasket_dev->coherent_buffer.length_bytes) {
		trace_gasket_mmap_exit(-EINVAL);
		return -EINVAL;
	}

	permissions = driver_desc->coherent_buffer_description.permissions;
	if (!gasket_mmap_has_permissions(gasket_dev, vma, permissions)) {
		dev_err(gasket_dev->dev, "Permission checking failed.\n");
		trace_gasket_mmap_exit(-EPERM);
		return -EPERM;
	}

	/*
	 * Pages are inserted on first touch (gasket_coherent_vma_fault()).
	 * vm_pgoff keeps the real file offset, so the buffer's mappings can be
	 * found and removed by offset range.
	 */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_ops = &gasket_coherent_vm_ops;

	/* Record the user virtual to dma_address mapping. */
	gasket_set_user_virt(gasket_dev, requested_length,
			     gasket_dev->coherent_buffer.phys_base,
			     vma->vm_start);
	return 0;
}

/*
 * Page fault on a BAR mapping: insert the one page, if it lies in a mappable
 * region and the device is still present. Runs under mmap_lock; takes the
 * device mutex, which removal holds while it zaps user mappings after
 * setting "gone", so no page can be inserted after the zap.
 */
static vm_fault_t gasket_bar_vma_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct gasket_dev *gasket_dev = vma->vm_private_data;
	const struct gasket_driver_desc *driver_desc =
		gasket_dev->internal_desc->driver_desc;
	const struct gasket_bar_desc *bar_desc;
	ulong raw_offset, bar_offset, start, end;
	vm_fault_t ret = VM_FAULT_SIGBUS;
	int bar_index, i;

	raw_offset = (vmf->pgoff << PAGE_SHIFT) +
		     driver_desc->legacy_mmap_address_offset;

	mutex_lock(&gasket_dev->mutex);
	if (READ_ONCE(gasket_dev->gone))
		goto out;

	bar_index = gasket_get_bar_index(gasket_dev, raw_offset);
	if (bar_index < 0)
		goto out;
	bar_desc = &driver_desc->bar_descriptions[bar_index];
	bar_offset = raw_offset - bar_desc->base;

	for (i = 0; i < bar_desc->num_mappable_regions; i++) {
		start = bar_desc->mappable_regions[i].start;
		end = start + bar_desc->mappable_regions[i].length_bytes;
		if (bar_offset >= start && bar_offset < end)
			break;
	}
	if (i == bar_desc->num_mappable_regions)
		goto out;

	ret = vmf_insert_pfn(vma, vmf->address,
			     (gasket_dev->bar_data[bar_index].phys_base +
			      bar_offset) >> PAGE_SHIFT);
out:
	mutex_unlock(&gasket_dev->mutex);
	return ret;
}

static const struct vm_operations_struct gasket_bar_vm_ops = {
	.fault = gasket_bar_vma_fault,
};

/* Map a device's BARs into user space. Called with gasket_dev->mutex held. */
static int __gasket_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int i, ret;
	int bar_index;
	int has_mapped_anything = 0;
	ulong permissions;
	ulong raw_offset;
	bool is_coherent_region;
	const struct gasket_driver_desc *driver_desc;
	struct gasket_dev *gasket_dev = (struct gasket_dev *)filp->private_data;
	const struct gasket_bar_desc *bar_desc;
	struct gasket_mappable_region *map_regions = NULL;
	int num_map_regions = 0;
	enum do_map_region_status map_status;

	driver_desc = gasket_dev->internal_desc->driver_desc;

	/*
	 * Device memory is never copy-on-write: a private writable mapping of
	 * a PFN range would trip a BUG in vmf_insert_pfn(). (A read-only
	 * file's MAP_SHARED mapping has neither VM_SHARED nor VM_MAYWRITE and
	 * is fine.)
	 */
	/* Same test as is_cow_mapping() (renamed vma_is_cow_mapping() in 7.3). */
	if ((vma->vm_flags & (VM_SHARED | VM_MAYWRITE)) == VM_MAYWRITE) {
		trace_gasket_mmap_exit(-EINVAL);
		return -EINVAL;
	}

	if (vma->vm_start & ~PAGE_MASK) {
		dev_err(gasket_dev->dev,
			"Base address not page-aligned: 0x%lx\n",
			vma->vm_start);
		trace_gasket_mmap_exit(-EINVAL);
		return -EINVAL;
	}

	/* Calculate the offset of this range into physical mem. */
	raw_offset = (vma->vm_pgoff << PAGE_SHIFT) +
		driver_desc->legacy_mmap_address_offset;
	trace_gasket_mmap_entry(gasket_dev->dev_info.name, raw_offset,
				vma->vm_end - vma->vm_start);

	/*
	 * Check if the raw offset is within a bar region. If not, check if it
	 * is a coherent region.
	 */
	bar_index = gasket_get_bar_index(gasket_dev, raw_offset);
	is_coherent_region = gasket_is_coherent_region(driver_desc, raw_offset);
	if (bar_index < 0 && !is_coherent_region) {
		dev_err(gasket_dev->dev,
			"Unable to find matching bar for address 0x%lx\n",
			raw_offset);
		trace_gasket_mmap_exit(bar_index);
		return bar_index;
	}
	if (bar_index > 0 && is_coherent_region) {
		dev_err(gasket_dev->dev,
			"double matching bar and coherent buffers for address "
			"0x%lx\n",
			raw_offset);
		trace_gasket_mmap_exit(bar_index);
		return -EINVAL;
	}

	vma->vm_private_data = gasket_dev;

	if (is_coherent_region)
		return gasket_mmap_coherent(gasket_dev, vma);

	/* Everything in the rest of this function is for normal BAR mapping. */

	/*
	 * Subtract the base of the bar from the raw offset to get the
	 * memory location within the bar to map.
	 */
	bar_desc = &driver_desc->bar_descriptions[bar_index];
	permissions = bar_desc->permissions;
	if (!gasket_mmap_has_permissions(gasket_dev, vma, permissions)) {
		dev_err(gasket_dev->dev, "Permission checking failed.\n");
		trace_gasket_mmap_exit(-EPERM);
		return -EPERM;
	}

	if (driver_desc->get_mappable_regions_cb) {
		ret = driver_desc->get_mappable_regions_cb(gasket_dev,
							   bar_index,
							   &map_regions,
							   &num_map_regions);
		if (ret)
			return ret;
	} else {
		if (!gasket_mmap_has_permissions(gasket_dev, vma,
						 bar_desc->permissions)) {
			dev_err(gasket_dev->dev,
				"Permission checking failed.\n");
			trace_gasket_mmap_exit(-EPERM);
			return -EPERM;
		}
		num_map_regions = bar_desc->num_mappable_regions;
		map_regions = kcalloc(num_map_regions,
				      sizeof(*bar_desc->mappable_regions),
				      GFP_KERNEL);
		if (map_regions) {
			memcpy(map_regions, bar_desc->mappable_regions,
			       num_map_regions *
					sizeof(*bar_desc->mappable_regions));
		}
	}

	if (!map_regions || num_map_regions == 0) {
		dev_err(gasket_dev->dev, "No mappable regions returned!\n");
		return -EINVAL;
	}

	/* Marks the VMA's pages as uncacheable. */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/*
	 * Static regions: map on first touch (see gasket_bar_vma_fault()), so
	 * removal can reliably take every page away again.
	 */
	if (!driver_desc->get_mappable_regions_cb) {
		ulong bar_offset, virt_offset;
		struct gasket_mappable_region unused;

		ret = gasket_mm_vma_bar_offset(gasket_dev, vma, &bar_offset);
		if (ret) {
			kfree(map_regions);
			return ret;
		}
		for (i = 0; i < num_map_regions; i++)
			if (gasket_mm_get_mapping_addrs(&map_regions[i],
							bar_offset,
							vma->vm_end -
							vma->vm_start,
							&unused, &virt_offset))
				has_mapped_anything = 1;
		kfree(map_regions);
		if (!has_mapped_anything) {
			dev_err(gasket_dev->dev,
				"Map request did not contain a valid region.\n");
			trace_gasket_mmap_exit(-EINVAL);
			return -EINVAL;
		}
		vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND |
				  VM_DONTDUMP);
		vma->vm_ops = &gasket_bar_vm_ops;
		trace_gasket_mmap_exit(0);
		return 0;
	}

	for (i = 0; i < num_map_regions; i++) {
		map_status = do_map_region(gasket_dev, vma, &map_regions[i]);
		/* Try the next region if this one was not mappable. */
		if (map_status == DO_MAP_REGION_INVALID)
			continue;
		if (map_status == DO_MAP_REGION_FAILURE) {
			ret = -ENOMEM;
			goto fail;
		}

		has_mapped_anything = 1;
	}

	kfree(map_regions);

	/* If we could not map any memory, the request was invalid. */
	if (!has_mapped_anything) {
		dev_err(gasket_dev->dev,
			"Map request did not contain a valid region.\n");
		trace_gasket_mmap_exit(-EINVAL);
		return -EINVAL;
	}

	trace_gasket_mmap_exit(0);
	return 0;

fail:
	/* Need to unmap any mapped ranges. */
	num_map_regions = i;
	for (i = 0; i < num_map_regions; i++)
		if (gasket_mm_unmap_region(gasket_dev, vma,
					   &bar_desc->mappable_regions[i]))
			dev_err(gasket_dev->dev, "Error unmapping range %d.\n",
				i);
	kfree(map_regions);

	return ret;
}

/*
 * Open the char device file.
 *
 * If the open is for writing, and the device is not owned, this process becomes
 * the owner.  If the open is for writing and the device is already owned by
 * some other process, it is an error.  If this process is the owner, increment
 * the open count.
 *
 * Returns 0 if successful, a negative error number otherwise.
 */
static int gasket_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct gasket_dev *gasket_dev;
	const struct gasket_driver_desc *driver_desc;
	struct gasket_ownership *ownership;
	char task_name[TASK_COMM_LEN];
	struct gasket_cdev_info *dev_info =
	    container_of(inode->i_cdev, struct gasket_cdev_info, cdev);
	struct pid_namespace *pid_ns = task_active_pid_ns(current);
	bool is_root = ns_capable(pid_ns->user_ns, CAP_SYS_ADMIN);

	struct gasket_open_file *of;

	gasket_dev = dev_info->gasket_dev_ptr;
	driver_desc = gasket_dev->internal_desc->driver_desc;
	ownership = &dev_info->ownership;
	get_task_comm(task_name, current);
	filp->private_data = gasket_dev;
	inode->i_size = 0;

	dev_dbg(gasket_dev->dev,
		"Attempting to open with tgid %u (%s) (f_mode: 0%03o, "
		"fmode_write: %d is_root: %u)\n",
		current->tgid, task_name, filp->f_mode,
		(filp->f_mode & FMODE_WRITE), is_root);

	of = kzalloc(sizeof(*of), GFP_KERNEL);
	if (!of)
		return -ENOMEM;
	of->filp = filp;

	ret = gasket_dev_enter(gasket_dev);
	if (ret) {
		kfree(of);
		return ret;
	}

	mutex_lock(&gasket_dev->mutex);

	/* Always allow non-writing accesses. */
	if (!(filp->f_mode & FMODE_WRITE)) {
		dev_dbg(gasket_dev->dev, "Allowing read-only opening.\n");
		list_add(&of->node, &gasket_dev->open_files);
		mutex_unlock(&gasket_dev->mutex);
		gasket_dev_exit(gasket_dev);
		return 0;
	}

	dev_dbg(gasket_dev->dev,
		"Current owner open count (owning tgid %u): %d.\n",
		ownership->owner, ownership->write_open_count);

	/* Opening a node owned by another TGID is an error (unless root) */
	if (ownership->is_owned && ownership->owner != current->tgid &&
	    !is_root) {
		dev_err(gasket_dev->dev,
			"Process %u is opening a node held by %u.\n",
			current->tgid, ownership->owner);
		mutex_unlock(&gasket_dev->mutex);
		gasket_dev_exit(gasket_dev);
		kfree(of);
		return -EPERM;
	}

	/* If the node is not owned, assign it to the current TGID. */
	if (!ownership->is_owned) {
		ret = gasket_check_and_invoke_callback_nolock(gasket_dev,
							      driver_desc->device_open_cb);
		if (ret) {
			dev_err(gasket_dev->dev,
				"Error in device open cb: %d\n", ret);
			mutex_unlock(&gasket_dev->mutex);
			gasket_dev_exit(gasket_dev);
			kfree(of);
			return ret;
		}
		ownership->is_owned = 1;
		ownership->owner = current->tgid;
		dev_dbg(gasket_dev->dev, "Device owner is now tgid %u\n",
			ownership->owner);
	}

	ownership->write_open_count++;
	list_add(&of->node, &gasket_dev->open_files);

	dev_dbg(gasket_dev->dev, "New open count (owning tgid %u): %d\n",
		ownership->owner, ownership->write_open_count);

	mutex_unlock(&gasket_dev->mutex);
	gasket_dev_exit(gasket_dev);
	return 0;
}

/*
 * Called on a close of the device file.  If this process is the owner,
 * decrement the open count.  On last close by the owner, free up buffers and
 * eventfd contexts, and release ownership.
 *
 * Returns 0 if successful, a negative error number otherwise.
 */
static int gasket_release(struct inode *inode, struct file *file)
{
	int i;
	struct gasket_dev *gasket_dev;
	struct gasket_ownership *ownership;
	const struct gasket_driver_desc *driver_desc;
	char task_name[TASK_COMM_LEN];
	struct gasket_cdev_info *dev_info =
		container_of(inode->i_cdev, struct gasket_cdev_info, cdev);
	struct pid_namespace *pid_ns = task_active_pid_ns(current);
	bool is_root = ns_capable(pid_ns->user_ns, CAP_SYS_ADMIN);

	struct gasket_open_file *of, *tmp;
	bool present;

	gasket_dev = dev_info->gasket_dev_ptr;
	driver_desc = gasket_dev->internal_desc->driver_desc;
	ownership = &dev_info->ownership;
	get_task_comm(task_name, current);

	/*
	 * Keep removal out while the hardware is torn down below. After
	 * removal only the bookkeeping is done: removal already reset the chip
	 * (if it still answered), turned off bus mastering and released every
	 * page table mapping.
	 */
	down_read(&gasket_dev->state_sem);
	present = !gasket_dev->gone;
	mutex_lock(&gasket_dev->mutex);

	list_for_each_entry_safe(of, tmp, &gasket_dev->open_files, node) {
		if (of->filp == file) {
			list_del(&of->node);
			kfree(of);
			break;
		}
	}

	dev_dbg(gasket_dev->dev,
		"Releasing device node. Call origin: tgid %u (%s) "
		"(f_mode: 0%03o, fmode_write: %d, is_root: %u)\n",
		current->tgid, task_name, file->f_mode,
		(file->f_mode & FMODE_WRITE), is_root);
	dev_dbg(gasket_dev->dev, "Current open count (owning tgid %u): %d\n",
		ownership->owner, ownership->write_open_count);

	if (file->f_mode & FMODE_WRITE) {
		ownership->write_open_count--;
		if (ownership->write_open_count == 0) {
			dev_dbg(gasket_dev->dev, "Device is now free\n");
			ownership->is_owned = 0;
			ownership->owner = 0;
		}
		if (ownership->write_open_count == 0 && present) {

			/* Forces chip reset before we unmap the page tables. */
			driver_desc->device_reset_cb(gasket_dev);

			for (i = 0; i < driver_desc->num_page_tables; ++i) {
				gasket_page_table_unmap_all(gasket_dev->page_table[i]);
				gasket_page_table_garbage_collect(gasket_dev->page_table[i]);
			}

			/*
			 * Freeing takes the buffer away from every mapping
			 * first, including a read-only opener's.
			 */
			for (i = 0; i < driver_desc->num_page_tables; ++i)
				gasket_free_coherent_memory_all(gasket_dev, i);

			/* Closes device, enters power save. */
			gasket_check_and_invoke_callback_nolock(gasket_dev,
								driver_desc->device_close_cb);
		}
	}

	dev_dbg(gasket_dev->dev, "New open count (owning tgid %u): %d\n",
		ownership->owner, ownership->write_open_count);
	mutex_unlock(&gasket_dev->mutex);
	up_read(&gasket_dev->state_sem);
	return 0;
}

/*
 * Gasket ioctl dispatch function.
 *
 * Check if the ioctl is a generic ioctl. If not, pass the ioctl to the
 * ioctl_handler_cb registered in the driver description.
 * If the ioctl is a generic ioctl, pass it to gasket_ioctl_handler.
 */
static long gasket_ioctl(struct file *filp, uint cmd, ulong arg)
{
	struct gasket_dev *gasket_dev;
	const struct gasket_driver_desc *driver_desc;
	void __user *argp = (void __user *)arg;
	char path[256];
	long ret;

	gasket_dev = (struct gasket_dev *)filp->private_data;
	driver_desc = gasket_dev->internal_desc->driver_desc;
	if (!driver_desc) {
		dev_dbg(gasket_dev->dev,
			"Unable to find device descriptor for file %s\n",
			d_path(&filp->f_path, path, 256));
		return -ENODEV;
	}

	ret = gasket_dev_enter(gasket_dev);
	if (ret)
		return ret;

	if (!gasket_is_supported_ioctl(cmd)) {
		/*
		 * The ioctl handler is not a standard Gasket callback, since
		 * it requires different arguments. This means we can't use
		 * check_and_invoke_callback.
		 */
		if (driver_desc->ioctl_handler_cb) {
			ret = driver_desc->ioctl_handler_cb(filp, cmd, argp);
		} else {
			dev_dbg(gasket_dev->dev,
				"Received unknown ioctl 0x%x\n", cmd);
			ret = -EINVAL;
		}
	} else {
		ret = gasket_handle_ioctl(filp, cmd, argp);
	}

	gasket_dev_exit(gasket_dev);
	return ret;
}

/*
 * mmap() entry point.
 *
 * This runs with the caller's mmap_lock held, so it must not wait on
 * state_sem (an ioctl holding state_sem may be waiting for that same
 * mmap_lock while removal waits to take state_sem for writing).
 *
 * The VMA is linked into the file's mapping only after this returns, so
 * removal's unmap walk can miss a VMA created at that moment. That is safe:
 * BAR pages are only inserted by the fault handler, which re-checks "gone"
 * under the mutex; and the coherent buffer is counted here before the VMA
 * is linked, so removal defers freeing it until that VMA is closed.
 */
static int gasket_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct gasket_dev *gasket_dev = filp->private_data;
	int ret;

	mutex_lock(&gasket_dev->mutex);
	if (READ_ONCE(gasket_dev->gone))
		ret = -ENODEV;
	else
		ret = __gasket_mmap(filp, vma);
	mutex_unlock(&gasket_dev->mutex);
	return ret;
}

/* File operations for all Gasket devices. */
static const struct file_operations gasket_file_ops = {
	.owner = THIS_MODULE,
	.mmap = gasket_mmap,
	.open = gasket_open,
	.release = gasket_release,
	.unlocked_ioctl = gasket_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

/* Perform final init and marks the device as active. */
int gasket_enable_device(struct gasket_dev *gasket_dev)
{
	int tbl_idx;
	int ret;
	const struct gasket_driver_desc *driver_desc =
		gasket_dev->internal_desc->driver_desc;

	dev_dbg(gasket_dev->dev, "enabling device\n");
	ret = gasket_interrupt_init(gasket_dev);
	if (ret) {
		dev_err(gasket_dev->dev,
			"Critical failure to allocate interrupts: %d\n", ret);
		goto undo;
	}

	for (tbl_idx = 0; tbl_idx < driver_desc->num_page_tables; tbl_idx++) {
		dev_dbg(gasket_dev->dev, "Initializing page table %d.\n",
			tbl_idx);
		ret = gasket_page_table_init(&gasket_dev->page_table[tbl_idx],
					     &gasket_dev->bar_data[driver_desc->page_table_bar_index],
					     &driver_desc->page_table_configs[tbl_idx],
					     gasket_dev->dev,
					     gasket_dev->pci_dev);
		if (ret) {
			dev_err(gasket_dev->dev,
				"Couldn't init page table %d: %d\n",
				tbl_idx, ret);
			goto undo;
		}
		/*
		 * Make sure that the page table is clear and set to simple
		 * addresses.
		 */
		gasket_page_table_reset(gasket_dev->page_table[tbl_idx]);
	}

	/*
	 * hardware_revision_cb returns a positive integer (the rev) if
	 * successful.)
	 */
	ret = check_and_invoke_callback(gasket_dev,
					driver_desc->hardware_revision_cb);
	if (ret < 0) {
		dev_err(gasket_dev->dev,
			"Error getting hardware revision: %d\n", ret);
		goto undo;
	}
	gasket_dev->hardware_revision = ret;

	/* device_status_cb returns a device status, not an error code. */
	gasket_dev->status = gasket_get_hw_status(gasket_dev);
	if (gasket_dev->status == GASKET_STATUS_DEAD)
		dev_err(gasket_dev->dev, "Device reported as unhealthy.\n");

	ret = gasket_add_cdev(&gasket_dev->dev_info, &gasket_file_ops,
			      driver_desc->module);
	if (ret)
		goto undo;

	return 0;

undo:
	/* Give back what was set up, so the caller can simply remove. */
	gasket_interrupt_cleanup(gasket_dev);
	for (tbl_idx = 0; tbl_idx < driver_desc->num_page_tables; tbl_idx++) {
		struct gasket_page_table *pg_tbl;

		/* sysfs readers of page_table[] take the device mutex. */
		mutex_lock(&gasket_dev->mutex);
		pg_tbl = gasket_dev->page_table[tbl_idx];
		gasket_dev->page_table[tbl_idx] = NULL;
		mutex_unlock(&gasket_dev->mutex);
		if (pg_tbl)
			gasket_page_table_cleanup(pg_tbl);
	}
	return ret;
}
EXPORT_SYMBOL(gasket_enable_device);

static int __gasket_add_device(struct device *parent_dev,
			       struct gasket_internal_desc *internal_desc,
			       struct gasket_dev **gasket_devp)
{
	int ret;
	struct gasket_dev *gasket_dev;

	ret = gasket_alloc_dev(internal_desc, parent_dev, &gasket_dev);
	if (ret)
		return ret;

	*gasket_devp = gasket_dev;
	return 0;
}

static void __gasket_remove_device(struct gasket_internal_desc *internal_desc,
				   struct gasket_dev *gasket_dev)
{
	gasket_free_dev(gasket_dev);
}

/*
 * Add PCI gasket device.
 *
 * Called by Gasket device probe function.
 * Allocates device metadata and maps device memory.  The device driver must
 * call gasket_enable_device after driver init is complete to place the device
 * in active use.
 */
int gasket_pci_add_device(struct pci_dev *pci_dev,
			  struct gasket_dev **gasket_devp)
{
	int ret;
	struct gasket_internal_desc *internal_desc;
	struct gasket_dev *gasket_dev;
	struct device *parent;

	dev_dbg(&pci_dev->dev, "add PCI gasket device\n");

	mutex_lock(&g_mutex);
	internal_desc = lookup_pci_internal_desc(pci_dev);
	mutex_unlock(&g_mutex);
	if (!internal_desc) {
		dev_err(&pci_dev->dev,
			"PCI add device called for unknown driver type\n");
		return -ENODEV;
	}

	parent = &pci_dev->dev;
	ret = __gasket_add_device(parent, internal_desc, &gasket_dev);
	if (ret)
		return ret;

	gasket_dev->pci_dev = pci_dev;
	ret = gasket_setup_pci(pci_dev, gasket_dev);
	if (ret)
		goto cleanup_pci;

	/*
	 * Once we've created the mapping structures successfully, attempt to
	 * create a symlink to the pci directory of this object.
	 */
	ret = sysfs_create_link(&gasket_dev->dev_info.device->kobj,
				&pci_dev->dev.kobj, dev_name(&pci_dev->dev));
	if (ret) {
		dev_err(gasket_dev->dev,
			"Cannot create sysfs pci link: %d\n", ret);
		goto cleanup_pci;
	}

	*gasket_devp = gasket_dev;
	return 0;

cleanup_pci:
	gasket_cleanup_pci(gasket_dev);
	__gasket_remove_device(internal_desc, gasket_dev);
	return ret;
}
EXPORT_SYMBOL(gasket_pci_add_device);

/* Remove a PCI gasket device. */
void gasket_pci_remove_device(struct pci_dev *pci_dev)
{
	int i;
	struct gasket_internal_desc *internal_desc;
	struct gasket_dev *gasket_dev = NULL;
	/* Find the device desc. */
	mutex_lock(&g_mutex);
	internal_desc = lookup_pci_internal_desc(pci_dev);
	if (!internal_desc) {
		mutex_unlock(&g_mutex);
		return;
	}
	mutex_unlock(&g_mutex);

	/* Now find the specific device */
	mutex_lock(&internal_desc->mutex);
	for (i = 0; i < GASKET_DEV_MAX; i++) {
		if (internal_desc->devs[i] &&
		    internal_desc->devs[i]->pci_dev == pci_dev) {
			gasket_dev = internal_desc->devs[i];
			break;
		}
	}
	mutex_unlock(&internal_desc->mutex);

	if (!gasket_dev)
		return;

	dev_dbg(gasket_dev->dev, "remove %s PCI gasket device\n",
		internal_desc->driver_desc->name);

	/* sysfs first (waits for readers), then the BARs they may touch. */
	if (device_is_registered(&gasket_dev->class_dev))
		device_del(&gasket_dev->class_dev);
	gasket_cleanup_pci(gasket_dev);
	__gasket_remove_device(internal_desc, gasket_dev);
}
EXPORT_SYMBOL(gasket_pci_remove_device);

/* Add platform gasket device. Called by Gasket device probe function. */
int gasket_platform_add_device(struct platform_device *pdev,
			       struct gasket_dev **gasket_devp)
{
	int ret;
	struct gasket_internal_desc *internal_desc;
	struct gasket_dev *gasket_dev;
	struct device *parent;

	dev_dbg(&pdev->dev, "add platform gasket device\n");

	mutex_lock(&g_mutex);
	internal_desc = lookup_platform_internal_desc(pdev);
	mutex_unlock(&g_mutex);
	if (!internal_desc) {
		dev_err(&pdev->dev,
			"%s called for unknown driver type\n", __func__);
		return -ENODEV;
	}

	parent = &pdev->dev;
	ret = __gasket_add_device(parent, internal_desc, &gasket_dev);
	if (ret)
		return ret;

	gasket_dev->platform_dev = pdev;
	*gasket_devp = gasket_dev;
	return 0;
}
EXPORT_SYMBOL(gasket_platform_add_device);

/* Remove a platform gasket device. */
void gasket_platform_remove_device(struct platform_device *pdev)
{
	int i;
	struct gasket_internal_desc *internal_desc;
	struct gasket_dev *gasket_dev = NULL;

	/* Find the device desc. */
	mutex_lock(&g_mutex);
	internal_desc = lookup_platform_internal_desc(pdev);
	mutex_unlock(&g_mutex);
	if (!internal_desc)
		return;

	/* Now find the specific device */
	mutex_lock(&internal_desc->mutex);
	for (i = 0; i < GASKET_DEV_MAX; i++) {
		if (internal_desc->devs[i] &&
		    internal_desc->devs[i]->platform_dev == pdev) {
			gasket_dev = internal_desc->devs[i];
			break;
		}
	}
	mutex_unlock(&internal_desc->mutex);

	if (!gasket_dev)
		return;

	dev_dbg(gasket_dev->dev, "remove %s platform gasket device\n",
		internal_desc->driver_desc->name);

	__gasket_remove_device(internal_desc, gasket_dev);
}
EXPORT_SYMBOL(gasket_platform_remove_device);

void gasket_set_dma_device(struct gasket_dev *gasket_dev,
			   struct device *dma_dev)
{
	put_device(gasket_dev->dma_dev);
	gasket_dev->dma_dev = get_device(dma_dev);
}
EXPORT_SYMBOL(gasket_set_dma_device);

/*
 * Lookup a name by number in a num_name table.
 * @num: Number to lookup.
 * @table: Array of num_name structures, the table for the lookup.
 *
 * Description: Searches for num in the table.  If found, the
 *		corresponding name is returned; otherwise NULL
 *		is returned.
 *
 *		The table must have a NULL name pointer at the end.
 */
const char *gasket_num_name_lookup(uint num,
				   const struct gasket_num_name *table)
{
	uint i = 0;

	while (table[i].snn_name) {
		if (num == table[i].snn_num)
			break;
		++i;
	}

	return table[i].snn_name;
}
EXPORT_SYMBOL(gasket_num_name_lookup);

int gasket_reset(struct gasket_dev *gasket_dev)
{
	int ret;

	mutex_lock(&gasket_dev->mutex);
	ret = gasket_reset_nolock(gasket_dev);
	mutex_unlock(&gasket_dev->mutex);
	return ret;
}
EXPORT_SYMBOL(gasket_reset);

int gasket_reset_nolock(struct gasket_dev *gasket_dev)
{
	int ret;
	int i;
	const struct gasket_driver_desc *driver_desc;

	driver_desc = gasket_dev->internal_desc->driver_desc;
	if (!driver_desc->device_reset_cb)
		return 0;

	ret = driver_desc->device_reset_cb(gasket_dev);
	if (ret) {
		dev_dbg(gasket_dev->dev, "Device reset cb returned %d.\n",
			ret);
		return ret;
	}

	/* Reinitialize the page tables and interrupt framework. */
	for (i = 0; i < driver_desc->num_page_tables; ++i)
		gasket_page_table_reset(gasket_dev->page_table[i]);

	ret = gasket_interrupt_reinit(gasket_dev);
	if (ret) {
		dev_dbg(gasket_dev->dev, "Unable to reinit interrupts: %d.\n",
			ret);
		return ret;
	}

	/* Get current device health. */
	gasket_dev->status = gasket_get_hw_status(gasket_dev);
	if (gasket_dev->status == GASKET_STATUS_DEAD) {
		dev_dbg(gasket_dev->dev, "Device reported as dead.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(gasket_reset_nolock);

gasket_ioctl_permissions_cb_t
gasket_get_ioctl_permissions_cb(struct gasket_dev *gasket_dev)
{
	return gasket_dev->internal_desc->driver_desc->ioctl_permissions_cb;
}
EXPORT_SYMBOL(gasket_get_ioctl_permissions_cb);

/* Get the driver structure for a given gasket_dev.
 * @dev: pointer to gasket_dev, implementing the requested driver.
 */
const struct gasket_driver_desc *gasket_get_driver_desc(struct gasket_dev *dev)
{
	return dev->internal_desc->driver_desc;
}

/* Get the device structure for a given gasket_dev.
 * @dev: pointer to gasket_dev, implementing the requested driver.
 */
struct device *gasket_get_device(struct gasket_dev *dev)
{
	return dev->dev;
}

/*
 * Asynchronously waits on device.
 * @gasket_dev: Device struct.
 * @bar: Bar
 * @offset: Register offset
 * @mask: Register mask
 * @val: Expected value
 * @max_retries: number of sleep periods
 * @delay_ms: Timeout in milliseconds
 *
 * Description: Busy waits for a specific combination of bits to be set on a
 * Gasket register.
 **/
int gasket_wait_with_reschedule(struct gasket_dev *gasket_dev, int bar,
				u64 offset, u64 mask, u64 val,
				uint max_retries, u64 delay_ms)
{
	uint retries = 0;
	u64 tmp;

	while (retries < max_retries) {
		tmp = gasket_dev_read_64(gasket_dev, bar, offset);
		if ((tmp & mask) == val)
			return 0;
		msleep(delay_ms);
		retries++;
	}
	dev_dbg(gasket_dev->dev, "%s timeout: reg %llx timeout (%llu ms)\n",
		__func__, offset, max_retries * delay_ms);
	return -ETIMEDOUT;
}
EXPORT_SYMBOL(gasket_wait_with_reschedule);

/* See gasket_core.h for description. */
int gasket_register_device(const struct gasket_driver_desc *driver_desc)
{
	int i, ret;
	int desc_idx = -1;
	struct gasket_internal_desc *internal;

	pr_debug("Loading %s driver version %s\n", driver_desc->name,
		 driver_desc->driver_version);
	/* Check for duplicates and find a free slot. */
	mutex_lock(&g_mutex);

	for (i = 0; i < GASKET_FRAMEWORK_DESC_MAX; i++) {
		if (g_descs[i].driver_desc == driver_desc) {
			pr_err("%s driver already loaded/registered\n",
			       driver_desc->name);
			mutex_unlock(&g_mutex);
			return -EBUSY;
		}
	}

	/* This and the above loop could be combined, but this reads easier. */
	for (i = 0; i < GASKET_FRAMEWORK_DESC_MAX; i++) {
		if (!g_descs[i].driver_desc) {
			g_descs[i].driver_desc = driver_desc;
			desc_idx = i;
			break;
		}
	}
	mutex_unlock(&g_mutex);

	if (desc_idx == -1) {
		pr_err("too many drivers loaded, max %d\n",
		       GASKET_FRAMEWORK_DESC_MAX);
		return -EBUSY;
	}

	internal = &g_descs[desc_idx];
	mutex_init(&internal->mutex);
	internal->groups[0] = &gasket_generic_group;
	internal->groups[1] = driver_desc->sysfs_group;
	internal->groups[2] = NULL;
	memset(internal->devs, 0, sizeof(struct gasket_dev *) * GASKET_DEV_MAX);

	internal->class = class_create(driver_desc->name);

	if (IS_ERR(internal->class)) {
		pr_err("Cannot register %s class [ret=%ld]\n",
		       driver_desc->name, PTR_ERR(internal->class));
		ret = PTR_ERR(internal->class);
		goto unregister_gasket_driver;
	}

	ret = register_chrdev_region(MKDEV(driver_desc->major,
					   driver_desc->minor), GASKET_DEV_MAX,
				     driver_desc->name);
	if (ret) {
		pr_err("cannot register %s char driver [ret=%d]\n",
		       driver_desc->name, ret);
		goto destroy_class;
	}

	return 0;

destroy_class:
	class_destroy(internal->class);

unregister_gasket_driver:
	mutex_lock(&g_mutex);
	g_descs[desc_idx].driver_desc = NULL;
	mutex_unlock(&g_mutex);
	return ret;
}
EXPORT_SYMBOL(gasket_register_device);

/* See gasket_core.h for description. */
void gasket_unregister_device(const struct gasket_driver_desc *driver_desc)
{
	int i, desc_idx;
	struct gasket_internal_desc *internal_desc = NULL;

	mutex_lock(&g_mutex);
	for (i = 0; i < GASKET_FRAMEWORK_DESC_MAX; i++) {
		if (g_descs[i].driver_desc == driver_desc) {
			internal_desc = &g_descs[i];
			desc_idx = i;
			break;
		}
	}

	if (!internal_desc) {
		mutex_unlock(&g_mutex);
		pr_err("request to unregister unknown desc: %s, %d:%d\n",
		       driver_desc->name, driver_desc->major,
		       driver_desc->minor);
		return;
	}

	unregister_chrdev_region(MKDEV(driver_desc->major, driver_desc->minor),
				 GASKET_DEV_MAX);

	class_destroy(internal_desc->class);

	/* Finally, effectively "remove" the driver. */
	g_descs[desc_idx].driver_desc = NULL;
	mutex_unlock(&g_mutex);

	pr_debug("removed %s driver\n", driver_desc->name);
}
EXPORT_SYMBOL(gasket_unregister_device);

static int __init gasket_init(void)
{
	int i;

	mutex_lock(&g_mutex);
	for (i = 0; i < GASKET_FRAMEWORK_DESC_MAX; i++) {
		g_descs[i].driver_desc = NULL;
		mutex_init(&g_descs[i].mutex);
	}

	mutex_unlock(&g_mutex);
	return 0;
}

/*
 * Nothing to tear down: gasket_init() only initialises static state, and
 * every driver that registered with the framework holds a module reference
 * on gasket until it unregisters. Without an exit hook the module could
 * never be unloaded.
 */
static void __exit gasket_exit(void)
{
}

MODULE_DESCRIPTION("Google Gasket driver framework");
MODULE_VERSION(GASKET_FRAMEWORK_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rob Springer <rspringer@google.com>");
module_init(gasket_init);
module_exit(gasket_exit);
