/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_usbhost.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_USBHOST_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_USBHOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <nuttx/config.h>
#include "rx65n_definitions.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default, no-OHCI Case ****************************************************/

#define RX65N_USBHOST_HAVE_BANK         0x20004000
#define RX65N_USBHOST_MEMORYBASE        0x20004000
#define RX65N_USBHOST_MEMORYSIZE        0x4000
#undef RX65N_USBHOST_HEAPBASE
#undef RX65N_USBHOST_HEAPSIZE

#ifdef RX65N_USBHOST_HAVE_BANK
#  define RX65N_USBHOST_HEAPBASE        RX65N_USBHOST_MEMORYBASE
#  define RX65N_USBHOST_HEAPSIZE        RX65N_USBHOST_MEMORYSIZE
#endif

#if defined(CONFIG_USBHOST) && defined(RX65N_NUSBHOST) > 0

/* OHCI RAM Configuration ***************************************************/

#ifndef RX65N_USBHOST_HAVE_BANK
#  error "Phani needs to look into this AHB SRAM Bank1 is not available for OHCI RAM"
#endif

/* OHCI/Heap Memory Allocation **********************************************/

/* Configured Size of the region at the end of AHB SRAM BANK1 set set aside
 * for the OHCI. This size must fit within AHB SRAM Bank 1 and also be a
 * multiple of 256 bytes.
 */

#ifndef CONFIG_RX65N_USBHOST_RAM_SIZE
#  define  CONFIG_RX65N_USBHOST_RAM_SIZE   RX65N_USBHOST_MEMORYSIZE
#endif

#if CONFIG_RX65N_USBHOST_RAM_SIZE > RX65N_USBHOST_HEAPBASE
#  error "USB Host RAM size cannot exceed the size of AHB SRAM Bank 1"
#endif

#if (CONFIG_RX65N_USBHOST_RAM_SIZE & 0xff) != 0
#  error "RX65N RAM size must be in multiples of 256 bytes"
#endif

#define RX65N_USBHOST_RAM_END  (RX65N_USBHOST_HAVE_BANK + RX65N_USBHOST_MEMORYBASE)

/* Numbers and Sizes of Things **********************************************/

/* Fixed size of the OHCI control area */

#define RX65N_USBHOST_HCCA_SIZE 256

/* Fixed endpoint descriptor size.  The actual size required by the hardware
 * is only 16 bytes, however, we set aside an additional 16 bytes for for
 * internal use by the OHCI host driver.  16-bytes is set aside because the
 * EDs must still be aligned to 16-byte boundaries.
 */

#define RX65N_USBHOST_ED_SIZE   32

/* Configurable number of user endpoint descriptors (EDs).  This number
 * excludes the control endpoint that is always allocated.
 */

#ifndef CONFIG_RX65N_USBHOST_NEDS
#  define CONFIG_RX65N_USBHOST_NEDS     9

#endif

/* Derived size of user endpoint descriptor (ED) memory. */

#define RX65N_USBHOST_EDFREE_SIZE (CONFIG_RX65N_USBHOST_NEDS * RX65N_USBHOST_ED_SIZE)

/* Fixed transfer descriptor size.  The actual size required by the hardware
 * is only 16 bytes, however, we set aside an additional 16 bytes for for
 * internal use by the OHCI host driver.  16-bytes is set aside because the
 * TDs must still be aligned to 16-byte boundaries.
 */

#define RX65N_USBHOST_TD_SIZE   32

/* Configurable number of user transfer descriptors (TDs).  */

#ifndef CONFIG_RX65N_USBHOST_NTDS
#  define CONFIG_RX65N_USBHOST_NTDS 10
#endif

#if CONFIG_RX65N_USBHOST_NTDS < 2
#  error "Insufficient TDs"
#endif

/* Derived size of user transfer descriptor (TD) memory. */

#define RX65N_USBHOST_TDFREE_SIZE (CONFIG_RX65N_USBHOST_NTDS * RX65N_USBHOST_TD_SIZE)

/* Configurable number of request/descriptor buffers (TDBUFFER) */

#ifndef CONFIG_RX65N_USBHOST_TDBUFFERS

#  define CONFIG_RX65N_USBHOST_TDBUFFERS        10

#endif

#if CONFIG_RX65N_USBHOST_TDBUFFERS < 2

#  error "At least two TD buffers are required"
#endif

/* Configurable size of a TD buffer */

#if CONFIG_RX65N_USBHOST_TDBUFFERS > 0 && !defined(CONFIG_RX65N_USBHOST_TDBUFSIZE)
#  define CONFIG_RX65N_USBHOST_TDBUFSIZE        128

#endif

#if (CONFIG_RX65N_USBHOST_TDBUFSIZE & 3) != 0
#  error "TD buffer size must be an even number of 32-bit words"
#endif

#define RX65N_USBHOST_TBFREE_SIZE (CONFIG_RX65N_USBHOST_TDBUFFERS * CONFIG_RX65N_USBHOST_TDBUFSIZE)

/* Configurable size of an IO buffer.  The number of IO buffers will be
 * determined by what is left at the end of the BANK1 memory setup aside
 * of OHCI RAM.
 */

#ifndef CONFIG_RX65N_USBHOST_IOBUFSIZE
#  define CONFIG_RX65N_USBHOST_IOBUFSIZE 512
#endif

#if (CONFIG_RX65N_USBHOST_IOBUFSIZE & 3) != 0
#  error "IO buffer size must be an even number of 32-bit words"
#endif

#define RX65N_USBHOST_HCCA_BASE         1

#define RX65N_USBHOST_TDTAIL_ADDR       2

#define RX65N_USBHOST_EDCTRL_ADDR   3

#define RX65N_USBHOST_EDFREE_BASE   4

#define RX65N_USBHOST_TDFREE_BASE   5

#define RX65N_USBHOST_TBFREE_BASE   6

#define RX65N_USBHOST_IOFREE_BASE   7

/* Finally, use the remainder of the allocated OHCI for IO buffers */

#if CONFIG_RX65N_USBHOST_IOBUFSIZE > 0
#  define RX65N_USBHOST_IOBUFFERS   0
#else
#  define RX65N_USBHOST_IOBUFFERS 0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rx65n_usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being initialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
struct usbhost_connection_s;
struct usbhost_connection_s *rx65n_usbhost_initialize(int controller);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* #if defined(CONFIG_USBHOST) && defined(RX65N_NUSBHOST) > 0 */
#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_USBHOST_H */
