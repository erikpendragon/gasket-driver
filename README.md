# Coral Gasket Driver (modernized fork)

This is a maintained fork of [feranick/gasket-driver](https://github.com/feranick/gasket-driver),
which continues Google's archived [google/gasket-driver](https://github.com/google/gasket-driver).
It brings the PCIe Coral Edge TPU driver up to date with current Linux kernel APIs and fixes
bugs found in a full code review. It is a drop-in replacement: the userspace interface
(`/dev/apex_N`, char major 120, ioctls, sysfs file names) is unchanged, so the existing
`libedgetpu` works as before.

## What changed

- **Kernels:** builds on Linux 6.12 (minimum) to 7.3. All version checks live in `src/gasket_compat.h`.
- **Unload:** `gasket` and `apex` can be removed with `rmmod` (the old `gasket` had no exit function).
- **Safe removal:** unbind, hot-unplug or link loss while a program uses the device no longer frees memory
  that is still in use. User mappings are fault-based and are revoked on removal; waiting programs are woken.
- **Memory:** user pages are pinned with `pin_user_pages_fast(FOLL_LONGTERM)`; dma-buf uses the current
  locking API; several page-table range checks are fixed.
- **Plumbing:** managed PCI resources, `pci_alloc_irq_vectors`, standard PM ops, `.shutdown`, PCIe error
  handlers, sysfs on standard attribute groups (`sysfs_emit`).
- **Thermal:** the throttle poller and the performance ioctl no longer undo each other; 2 °C hysteresis.
- **hwmon:** the die temperature appears as a standard sensor (`sensors` shows `apex-pci-...`).

Each commit message explains its change in detail.

## Testing

- Built on 6.12, 6.17, 7.0 and 7.3-rc4 kernels with `W=1` and sparse.
- Hardware tests with a PCIe Coral passed through to a VM running a 6.12 kernel with KASAN, lockdep,
  kmemleak, UBSAN and DMA-API debugging: removal during inference (8 of 8 clean), unbind/bind with the
  device open, `rmmod`/`insmod` 20 times, no kernel reports, no leaks.
- Inference outputs are bit-identical to the previous driver, at the same speed.
- In daily use on a 7.0 kernel with Frigate NVR since September 2026.

## Known limits

- An ioctl holds a read lock while it pins user pages, so a pin stuck on a `userfaultfd` range delays
  device removal. It needs a hostile device owner.
- `.mmap_prepare` (6.17+) is not used yet, because the 6.12 minimum lacks the API it needs.

## Upgrading from the old driver

The old `gasket` module cannot be unloaded, so install the new package and reboot once.
Later updates only need the programs that use the Coral to stop.

---

## Original README

The Coral Gasket Driver allows usage of the [Coral EdgeTPU](https://coral.ai/) on Linux systems. The driver contains two modules:

* Gasket: Gasket (Google ASIC Software, Kernel Extensions, and Tools) is a top level driver for lightweight communication with Google ASICs.
* Apex: Apex refers to the [EdgeTPU v1](https://coral.ai/technology)

This repo contains both the source for direct integration into a kernel tree as well as the necessary files to generate a Debian DKMS package.

## Building Debian DKMS pacakge

From the top level directory, execute:

```
debuild -us -uc -tc -b
```
