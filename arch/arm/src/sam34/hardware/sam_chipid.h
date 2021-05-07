/****************************************************************************
 * arch/arm/src/sam34/hardware/sam_chipid.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_CHIPID_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_CHIPID_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CHIPID register offsets **************************************************/

#define SAM_CHIPID_CIDR                 0x00 /* Chip ID Register */
#define SAM_CHIPID_EXID                 0x04 /* Chip ID Extension Register */

/* CHIPID register addresses ************************************************/

#define SAM_CHIPID_CIDR                 (SAM_CHIPID_BASE+SAM_CHIPID_CIDR)
#define SAM_CHIPID_EXID                 (SAM_CHIPID_BASE+SAM_CHIPID_EXID)

/* CHIPID register bit definitions ******************************************/

#define CHIPID_CIDR_VERSION_SHIFT       (0)      /* Bits 0-4:  Version of the Device */
#define CHIPID_CIDR_VERSION_MASK        (0x1f << CHIPID_CIDR_VERSION_SHIFT)
#define CHIPID_CIDR_EPROC_SHIFT         (5)      /* Bits 5-7:  Embedded Processor */
#define CHIPID_CIDR_EPROC_MASK          (7 << CHIPID_CIDR_EPROC_SHIFT)
#  define CHIPID_CIDR_EPROC_ARM946ES    (1 << CHIPID_CIDR_EPROC_SHIFT) /* ARM946E-S */
#  define CHIPID_CIDR_EPROC_ARM7TDMI    (2 << CHIPID_CIDR_EPROC_SHIFT) /* ARM7TDMI */
#  define CHIPID_CIDR_EPROC_CORTEXM3    (3 << CHIPID_CIDR_EPROC_SHIFT) /* Cortex-M3 */
#  define CHIPID_CIDR_EPROC_ARM920T     (4 << CHIPID_CIDR_EPROC_SHIFT) /* ARM920T */
#  define CHIPID_CIDR_EPROC_ARM926EJS   (5 << CHIPID_CIDR_EPROC_SHIFT) /* ARM926EJ-S */
#  define CHIPID_CIDR_EPROC_CORTEXA5    (6 << CHIPID_CIDR_EPROC_SHIFT) /* Cortex-A5 */
#  define CHIPID_CIDR_EPROC_CORTEXM4    (7 << CHIPID_CIDR_EPROC_SHIFT) /* Cortex-M4 */

#define CHIPID_CIDR_NVPSIZ_SHIFT        (8)      /* Bits 8-11:  Nonvolatile Program Memory Size */
#define CHIPID_CIDR_NVPSIZ_MASK         (15 << CHIPID_CIDR_NVPSIZ_SHIFT)
#  define CHIPID_CIDR_NVPSIZ_NONE       (0  << CHIPID_CIDR_NVPSIZ_SHIFT) /* None */
#  define CHIPID_CIDR_NVPSIZ_8KB        (1  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 8K bytes */
#  define CHIPID_CIDR_NVPSIZ_16KB       (2  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 16K bytes */
#  define CHIPID_CIDR_NVPSIZ_32KB       (3  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 32K bytes */
#  define CHIPID_CIDR_NVPSIZ_64KB       (5  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 64K bytes */
#  define CHIPID_CIDR_NVPSIZ_128KB      (7  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 128K bytes */
#  define CHIPID_CIDR_NVPSIZ_256KB      (9  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 256K bytes */
#  define CHIPID_CIDR_NVPSIZ_512KB      (10 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 512K bytes */
#  define CHIPID_CIDR_NVPSIZ_1MB        (12 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 1024K bytes */
#  define CHIPID_CIDR_NVPSIZ_2MB        (14 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 2048K bytes */

#define CHIPID_CIDR_NVPSIZ2_SHIFT       (12)      /* Bits 12-15:  Nonvolatile Program Memory Size */
#define CHIPID_CIDR_NVPSIZ2_MASK        (15 << CHIPID_CIDR_NVPSIZ_SHIFT)
#  define CHIPID_CIDR_NVPSIZ2_NONE      (0  << CHIPID_CIDR_NVPSIZ_SHIFT) /* None */
#  define CHIPID_CIDR_NVPSIZ2_8KB       (1  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 8K bytes */
#  define CHIPID_CIDR_NVPSIZ2_16KB      (2  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 16K bytes */
#  define CHIPID_CIDR_NVPSIZ2_32KB      (3  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 32K bytes */
#  define CHIPID_CIDR_NVPSIZ2_64KB      (5  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 64K bytes */
#  define CHIPID_CIDR_NVPSIZ2_128KB     (7  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 128K bytes */
#  define CHIPID_CIDR_NVPSIZ2_256KB     (9  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 256K bytes */
#  define CHIPID_CIDR_NVPSIZ2_512KB     (10 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 512K bytes */
#  define CHIPID_CIDR_NVPSIZ2_1MB       (12 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 1024K bytes */
#  define CHIPID_CIDR_NVPSIZ2_2MB       (14 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 2048K bytes */

#define CHIPID_CIDR_SRAMSIZ_SHIFT       (16)      /* Bits 16-19:  Internal SRAM Size */
#define CHIPID_CIDR_SRAMSIZ_MASK        (15 << CHIPID_CIDR_SRAMSIZ_SHIFT)
#  define CHIPID_CIDR_SRAMSIZ_48KB      (0  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 48K bytes */
#  define CHIPID_CIDR_SRAMSIZ_1KB       (1  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 1K bytes */
#  define CHIPID_CIDR_SRAMSIZ_192KB     (1  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 192K bytes (SAM4S) */
#  define CHIPID_CIDR_SRAMSIZ_2KB       (2  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 2K bytes */
#  define CHIPID_CIDR_SRAMSIZ_6KB       (3  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 6K bytes */
#  define CHIPID_CIDR_SRAMSIZ_112KB     (4  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 112K bytes */
#  define CHIPID_CIDR_SRAMSIZ_24KB      (4  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 24K bytes (SAM4S, SAM4L) */
#  define CHIPID_CIDR_SRAMSIZ_4KB       (5  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 4K bytes */
#  define CHIPID_CIDR_SRAMSIZ_80KB      (6  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 80K bytes */
#  define CHIPID_CIDR_SRAMSIZ_160KB     (7  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 160K bytes */
#  define CHIPID_CIDR_SRAMSIZ_8KB       (8  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 8K bytes */
#  define CHIPID_CIDR_SRAMSIZ_16KB      (9  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 16K bytes */
#  define CHIPID_CIDR_SRAMSIZ_32KB      (10 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 32K bytes */
#  define CHIPID_CIDR_SRAMSIZ_64KB      (11 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 64K bytes */
#  define CHIPID_CIDR_SRAMSIZ_128KB     (12 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 128K bytes */
#  define CHIPID_CIDR_SRAMSIZ_256KB     (13 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 256K bytes */
#  define CHIPID_CIDR_SRAMSIZ_96KB      (14 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 96K bytes */
#  define CHIPID_CIDR_SRAMSIZ_512KB     (15 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 512K bytes */

#define CHIPID_CIDR_ARCH_SHIFT          (20)      /* Bits 20-27:  Architecture Identifier */
#define CHIPID_CIDR_ARCH_MASK           (0xff << CHIPID_CIDR_ARCH_SHIFT)
#  define CHIPID_CIDR_ARCH_AT91SAM9XX   (0x19 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM9xx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM9XEXX (0x29 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM9XExx Series */
#  define CHIPID_CIDR_ARCH_AT91X34      (0x34 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x34 Series */
#  define CHIPID_CIDR_ARCH_CAP7         (0x37 << CHIPID_CIDR_ARCH_SHIFT) /* CAP7 Series */
#  define CHIPID_CIDR_ARCH_CAP9         (0x39 << CHIPID_CIDR_ARCH_SHIFT) /* CAP9 Series */
#  define CHIPID_CIDR_ARCH_CAP11        (0x3b << CHIPID_CIDR_ARCH_SHIFT) /* CAP11 Series */
#  define CHIPID_CIDR_ARCH_AT91X40      (0x40 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x40 Series */
#  define CHIPID_CIDR_ARCH_AT91X42      (0x42 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x42 Series */
#  define CHIPID_CIDR_ARCH_AT91X55      (0x55 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x55 Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7AXX  (0x60 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7Axx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7AQXX (0x61 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7AQxx Series */
#  define CHIPID_CIDR_ARCH_AT91X63      (0x63 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x63 Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7SXX  (0x70 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7Sxx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7XCXX (0x71 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7XCxx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7SEXX (0x72 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7SExx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7LXX  (0x73 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7Lxx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7XXX  (0x75 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7Xxx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7SLXX (0x76 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7SLxx Series */
#  define CHIPID_CIDR_ARCH_SAM3UXC      (0x80 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3UxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3UXE      (0x81 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3UxE Series (144-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3AXC      (0x83 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3AxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3XXC      (0x84 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3XxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3XXE      (0x85 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3XxE Series (144-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3XXG      (0x86 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3XxG Series (208/217-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3SXA      (0x88 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3SxA Series (48-pin version) */
#  define CHIPID_CIDR_ARCH_SAM4SXA      (0x88 << CHIPID_CIDR_ARCH_SHIFT) /* SAM4SxA Series (48-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3SXB      (0x89 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3SxB Series (64-pin version) */
#  define CHIPID_CIDR_ARCH_SAM4SXB      (0x89 << CHIPID_CIDR_ARCH_SHIFT) /* SAM34xB Series (64-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3SXC      (0x8a << CHIPID_CIDR_ARCH_SHIFT) /* SAM3SxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_SAM4SXC      (0x8a << CHIPID_CIDR_ARCH_SHIFT) /* SAM4SxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_AT91X92      (0x92 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x92 Series */
#  define CHIPID_CIDR_ARCH_SAM3NXA      (0x93 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3NxA Series (48-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3NXB      (0x94 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3NxB Series (64-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3NXC      (0x95 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3NxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3DXB      (0x99 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3SDxB SAM3SDxB Series (64-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3SDXC     (0x9a << CHIPID_CIDR_ARCH_SHIFT) /* SAM3SDxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_SAM5A        (0xa5 << CHIPID_CIDR_ARCH_SHIFT) /* SAM5A */
#  define CHIPID_CIDR_ARCH_SAM4LA       (0xb0 << CHIPID_CIDR_ARCH_SHIFT) /* SAM4LxA Series */
#  define CHIPID_CIDR_ARCH_SAM4LB       (0xb1 << CHIPID_CIDR_ARCH_SHIFT) /* SAM4LxB Series */
#  define CHIPID_CIDR_ARCH_SAM4LC       (0xb2 << CHIPID_CIDR_ARCH_SHIFT) /* SAM4LxC Series */
#  define CHIPID_CIDR_ARCH_AT75CXX      (0xf0 << CHIPID_CIDR_ARCH_SHIFT) /* AT75Cxx Series */

#define CHIPID_CIDR_NVPTYP_SHIFT        (28)      /* Bits 28-30:  Nonvolatile Program Memory Type */
#define CHIPID_CIDR_NVPTYP_MASK         (7 << CHIPID_CIDR_NVPTYP_SHIFT)
#  define CHIPID_CIDR_NVPTYP_ROM        (0 << CHIPID_CIDR_NVPTYP_SHIFT) /* ROM */
#  define CHIPID_CIDR_NVPTYP_FLASH      (1 << CHIPID_CIDR_NVPTYP_SHIFT) /* ROMless or on-chip Flash */
#  define CHIPID_CIDR_NVPTYP_SRAM       (4 << CHIPID_CIDR_NVPTYP_SHIFT) /* SRAM emulating ROM */
#  define CHIPID_CIDR_NVPTYP_EFLASH     (2 << CHIPID_CIDR_NVPTYP_SHIFT) /* Embedded Flash Memory */
#  define CHIPID_CIDR_NVPTYP_REFLASH    (3 << CHIPID_CIDR_NVPTYP_SHIFT) /* ROM and Embedded Flash Memory */

#define CHIPID_CIDR_EXT                 (1 << 31) /* Bit 31: Extension Flag */

/* Chip ID Extension Register (32-bit value for SAM3U and SAM4S) */

#ifdef CONFIG_ARCH_CHIP_SAM4L
#  define CHIPID_EXID_AES               (1 << 0)  /* Bit 0:  AES Option */
#  define CHIPID_EXID_USB               (1 << 1)  /* Bit 1:  USB Configuration */
#  define CHIPID_EXID_USBFULL           (1 << 2)  /* Bit 2:  USB Option */
#  define CHIPID_EXID_LCD               (1 << 3)  /* Bit 3:  LCD Option */
#  define CHIPID_EXID_PACKAGE_SHIFT     (24)      /* Bits 24-26: Package Type */
#  define CHIPID_EXID_PACKAGE_MASK      (7 << CHIPID_EXID_PACKAGE_SHIFT)
#    define CHIPID_EXID_PACKAGE_24PIN   (0 << CHIPID_EXID_PACKAGE_SHIFT) /* 24-pin package */
#    define CHIPID_EXID_PACKAGE_32PIN   (1 << CHIPID_EXID_PACKAGE_SHIFT) /* 32-pin package */
#    define CHIPID_EXID_PACKAGE_48PIN   (2 << CHIPID_EXID_PACKAGE_SHIFT) /* 48-pin package */
#    define CHIPID_EXID_PACKAGE_64PIN   (3 << CHIPID_EXID_PACKAGE_SHIFT) /* 64-pin package */
#    define CHIPID_EXID_PACKAGE_100PIN  (4 << CHIPID_EXID_PACKAGE_SHIFT) /* 100-pin package */
#    define CHIPID_EXID_PACKAGE_144PIN  (5 << CHIPID_EXID_PACKAGE_SHIFT) /* 144-pin package */
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

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_CHIPID_H */
