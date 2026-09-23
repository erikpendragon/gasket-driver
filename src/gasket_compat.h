/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Kernel version shims for the out-of-tree Gasket/Apex build.
 *
 * Every LINUX_VERSION_CODE check in the driver lives here, so the rest of
 * the code is written against the newest kernel API only.
 *
 * Minimum supported kernel: 6.12 (LTS).
 */
#ifndef __GASKET_COMPAT_H__
#define __GASKET_COMPAT_H__

#include <linux/dma-buf.h>
#include <linux/eventfd.h>
#include <linux/mm.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
#error "gasket requires Linux 6.12 or newer"
#endif

/* 6.13: MODULE_IMPORT_NS() takes a string literal. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
#define GASKET_IMPORT_NS_DMA_BUF() MODULE_IMPORT_NS("DMA_BUF")
#else
#define GASKET_IMPORT_NS_DMA_BUF() MODULE_IMPORT_NS(DMA_BUF)
#endif

/* 7.1: zap_vma_ptes() was renamed zap_special_vma_range(). */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(7, 1, 0)
#define gasket_zap_vma_range(vma, addr, size) \
	zap_special_vma_range(vma, addr, size)
#else
#define gasket_zap_vma_range(vma, addr, size) zap_vma_ptes(vma, addr, size)
#endif

#endif /* __GASKET_COMPAT_H__ */
