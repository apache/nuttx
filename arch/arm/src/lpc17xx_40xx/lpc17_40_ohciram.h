/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_ohciram.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_OHCIRAM_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_OHCIRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default, no-OHCI Case ****************************************************/

/* Assume that all of AHB SRAM will be available for heap.
 * If this is not true, then LPC17_40_BANK1_HEAPSIZE will be undefined but
 * redefined below.
 */

#undef LPC17_40_BANK1_HEAPBASE
#undef LPC17_40_BANK1_HEAPSIZE
#ifdef LPC17_40_HAVE_BANK1
#  define LPC17_40_BANK1_HEAPBASE LPC17_40_SRAM_BANK1
#  define LPC17_40_BANK1_HEAPSIZE LPC17_40_BANK1_SIZE
#endif

/* Is networking enabled?
 * Is the LPC17xx/LPC40xx Ethernet device enabled? Does this chip have
 * and Ethernet controlloer?
 * Yes... then we will replace the above default definitions.
 */

#if defined(CONFIG_USBHOST) && defined(CONFIG_LPC17_40_USBHOST) && LPC17_40_NUSBHOST > 0

/* OHCI RAM Configuration ***************************************************/

/* Is AHB SRAM available? */

#ifndef LPC17_40_HAVE_BANK1
#  error "AHB SRAM Bank1 is not available for OHCI RAM"
#endif

/* OHCI/Heap Memory Allocation **********************************************/

/* Configured Size of the region at the end of AHB SRAM BANK1 set set aside
 * for the OHCI. This size must fit within AHB SRAM Bank 1 and also be a
 * multiple of 256 bytes.
 */

#ifndef CONFIG_LPC17_40_OHCIRAM_SIZE
#  define CONFIG_LPC17_40_OHCIRAM_SIZE LPC17_40_BANK1_SIZE
#endif

#if CONFIG_LPC17_40_OHCIRAM_SIZE > LPC17_40_BANK1_SIZE
#  error "OHCI RAM size cannot exceed the size of AHB SRAM Bank 1"
#endif

#if (CONFIG_LPC17_40_OHCIRAM_SIZE & 0xff) != 0
#  error "OHCI RAM size must be in multiples of 256 bytes"
#endif

/* Then position the OHCI RAM at the end of AHB SRAM Bank 1 */

#define LPC17_40_OHCIRAM_END  (LPC17_40_SRAM_BANK1 + LPC17_40_BANK1_SIZE)
#define LPC17_40_OHCIRAM_BASE (LPC17_40_OHCIRAM_END - CONFIG_LPC17_40_OHCIRAM_SIZE)
#define LPC17_40_OHCIRAM_SIZE  CONFIG_LPC17_40_OHCIRAM_SIZE

/* Determine is there is any meaningful space left at the beginning of
 * AHB Bank 1 that could be added to the heap.
 */

#undef LPC17_40_BANK1_HEAPBASE
#undef LPC17_40_BANK1_HEAPSIZE
#if LPC17_40_OHCIRAM_SIZE < (LPC17_40_BANK1_SIZE-128)
#  define LPC17_40_BANK1_HEAPBASE LPC17_40_SRAM_BANK1
#  define LPC17_40_BANK1_HEAPSIZE (LPC17_40_BANK1_SIZE - LPC17_40_OHCIRAM_SIZE)
#endif

/* Numbers and Sizes of Things **********************************************/

/* Fixed size of the OHCI control area */

#define LPC17_40_HCCA_SIZE 256

/* Fixed endpoint descriptor size.
 * The actual size required by the hardware is only 16 bytes, however, we set
 * aside an additional 16 bytes for for internal use by the OHCI host driver.
 * 16-bytes is set aside because the EDs must still be aligned to 16-byte
 * boundaries.
 */

#define LPC17_40_ED_SIZE   32

/* Configurable number of user endpoint descriptors (EDs).
 *  This number excludes the control endpoint that is always allocated.
 */

#ifndef CONFIG_LP17_USBHOST_NEDS
#  define CONFIG_LP17_USBHOST_NEDS 2
#endif

/* Derived size of user endpoint descriptor (ED) memory. */

#define LPC17_40_EDFREE_SIZE (CONFIG_LP17_USBHOST_NEDS * LPC17_40_ED_SIZE)

/* Fixed transfer descriptor size.
 * The actual size required by the hardware is only 16 bytes, however, we set
 * aside an additional 16 bytes for for internal use bythe OHCI host driver.
 * 16-bytes is set aside because the TDs must still be aligned to 16-byte
 * boundaries.
 */

#define LPC17_40_TD_SIZE   32

/* Configurable number of user transfer descriptors (TDs).  */

#ifndef CONFIG_LP17_USBHOST_NTDS
#  define CONFIG_LP17_USBHOST_NTDS 3
#endif

#if CONFIG_LP17_USBHOST_NTDS < 2
#  error "Insufficient TDs"
#endif

/* Derived size of user transfer descriptor (TD) memory. */

#define LPC17_40_TDFREE_SIZE (CONFIG_LP17_USBHOST_NTDS * LPC17_40_TD_SIZE)

/* Configurable number of request/descriptor buffers (TDBUFFER) */

#ifndef CONFIG_LPC17_40_USBHOST_TDBUFFERS
#  define CONFIG_LPC17_40_USBHOST_TDBUFFERS 2
#endif

#if CONFIG_LPC17_40_USBHOST_TDBUFFERS < 2
#  error "At least two TD buffers are required"
#endif

/* Configurable size of a TD buffer */

#if CONFIG_LPC17_40_USBHOST_TDBUFFERS > 0 && !defined(CONFIG_LPC17_40_USBHOST_TDBUFSIZE)
#  define CONFIG_LPC17_40_USBHOST_TDBUFSIZE 128
#endif

#if (CONFIG_LPC17_40_USBHOST_TDBUFSIZE & 3) != 0
#  error "TD buffer size must be an even number of 32-bit words"
#endif

#define LPC17_40_TBFREE_SIZE (CONFIG_LPC17_40_USBHOST_TDBUFFERS * CONFIG_LPC17_40_USBHOST_TDBUFSIZE)

/* Configurable size of an IO buffer.
 * The number of IO buffers will be determined by what is left at the end
 * of the BANK1 memory setup aside of OHCI RAM.
 */

#ifndef CONFIG_LPC17_40_USBHOST_IOBUFSIZE
#  define CONFIG_LPC17_40_USBHOST_IOBUFSIZE 512
#endif

#if (CONFIG_LPC17_40_USBHOST_IOBUFSIZE & 3) != 0
#  error "IO buffer size must be an even number of 32-bit words"
#endif

/* OHCI Memory Layout *******************************************************/

/* Example:
 *  Hardware:
 *    LPC17_40_SRAM_BANK1            0x20008000
 *    LPC17_40_BANK1_SIZE            16384
 *
 *  Configuration:
 *    CONFIG_LPC17_40_OHCIRAM_SIZE   1536
 *    CONFIG_LP17_USBHOST_NEDS    2
 *    CONFIG_LP17_USBHOST_NTDS    3
 *    CONFIG_LPC17_40_USBHOST_TDBUFFERS  3
 *    CONFIG_LPC17_40_USBHOST_TDBUFSIZE  128
 *    CONFIG_LPC17_40_USBHOST_IOBUFSIZE  512
 *
 *  Sizes of things
 *    LPC17_40_EDFREE_SIZE      64   0x00000040
 *    LPC17_40_TDFREE_SIZE      96   0x00000060
 *    LPC17_40_TBFREE_SIZE     384  0x00000100
 *    LPC17_40_IOFREE_SIZE     512  0x00000200
 *
 *  Memory Layout
 *    LPC17_40_OHCIRAM_END     (0x20008000 + 16384) = 0x20084000
 *    LPC17_40_OHCIRAM_BASE    (0x2000c000 - 1536) = 0x2000ba00
 *    LPC17_40_OHCIRAM_SIZE    1280
 *    LPC17_40_BANK1_HEAPBASE  0x20008000
 *    LPC17_40_BANK1_HEAPSIZE       (16384 - 1280) = 15104
 *
 *    LPC17_40_HCCA_BASE       0x20083a00 -- Communications area
 *    LPC17_40_TDTAIL_ADDR     0x20083b00 -- Common. pre-allocated tail TD
 *    LPC17_40_EDCTRL_ADDR     0x20083b20 -- Pre-allocated ED for EP0
 *    LPC17_40_EDFREE_BASE     0x20083b40 -- Free EDs
 *    LPC17_40_TDFREE_BASE     0x20083b80 -- Free TDs
 *    LPC17_40_TBFREE_BASE     0x20083be0 -- Free request/descriptor buffers
 *    LPC17_40_IOFREE_BASE     0x20083d60 -- Free large I/O buffers
 *    LPC17_40_IOBUFFERS       (0x20084000 - 0x20083d60) / 512 = 672/512 = 1
 *
 *  Wasted memory:             672-512 = 160 bytes
 */

#define LPC17_40_HCCA_BASE     (LPC17_40_OHCIRAM_BASE)
#define LPC17_40_TDTAIL_ADDR   (LPC17_40_HCCA_BASE + LPC17_40_HCCA_SIZE)
#define LPC17_40_EDCTRL_ADDR   (LPC17_40_TDTAIL_ADDR + LPC17_40_TD_SIZE)
#define LPC17_40_EDFREE_BASE   (LPC17_40_EDCTRL_ADDR + LPC17_40_ED_SIZE)
#define LPC17_40_TDFREE_BASE   (LPC17_40_EDFREE_BASE + LPC17_40_EDFREE_SIZE)
#define LPC17_40_TBFREE_BASE   (LPC17_40_TDFREE_BASE + LPC17_40_TDFREE_SIZE)
#define LPC17_40_IOFREE_BASE   (LPC17_40_TBFREE_BASE + LPC17_40_TBFREE_SIZE)

#if LPC17_40_IOFREE_BASE > LPC17_40_OHCIRAM_END
#  error "Insufficient OHCI RAM allocated"
#endif

/* Finally, use the remainder of the allocated OHCI for IO buffers */

#if CONFIG_LPC17_40_USBHOST_IOBUFSIZE > 0
#  define LPC17_40_IOBUFFERS   ((LPC17_40_OHCIRAM_END - LPC17_40_IOFREE_BASE) / CONFIG_LPC17_40_USBHOST_IOBUFSIZE)
#else
#  define LPC17_40_IOBUFFERS 0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_USBHOST && CONFIG_LPC17_40_USBHOST && LPC17_40_NUSBHOST > 0*/
#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_OHCIRAM_H */
