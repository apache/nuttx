/****************************************************************************
 * arch/arm/src/rtl8730e/rtl8730e_memorymap.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RTL8730E_MEMORYMAP_H
#define __ARCH_ARM_SRC_RTL8730E_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTL8730E CA32 Physical Memory Map ***************************************
 *
 * Identity-mapped (V == P).  Values are taken from the vendor SDK
 * (hal_platform.h peripheral bases, atf platform_def.h GIC bases) and the
 * measured native boot log (PSRAM/DRAM runs 0x60000000-0x64000000, CA32
 * BL33 image is loaded at 0x60300000).
 *
 *   0x08000000-0x0fffffff  SPI NOR flash XIP window (128 MiB)
 *   0x23000000-0x23020200  KM0 SRAM (IPC shared memory at 0x2301fd00)
 *   0x40000000-0x43000000  Peripheral APB (HS_APB 0x41xxxxxx, LP_APB
 *                          0x42xxxxxx incl. LOGUART 0x4200c000)
 *   0x50000000-0x53000000  Secure alias of the peripheral APB (bit28 set)
 *   0x60000000-0x64000000  DDR/PSRAM, 64MB (NuttX runs at 0x60300000)
 *   0xa0000000-0xa0200000  MPCORE / GICv2 (GICD 0xa0101000, GICC 0xa0102000)
 */

#define VIRT_FLASH_PSECTION      0x08000000  /* SPI NOR XIP (128 MiB)   */
#define VIRT_KM4_SRAM_PSECTION   0x20000000  /* KM4 HP SRAM NS (SRAM_BASE) */
#define VIRT_KM0_PSECTION        0x23000000  /* KM0 SRAM: IPC shared mem */
#define VIRT_IO_PSECTION         0x40000000  /* 0x40000000-0x43000000   */
#define VIRT_SEC_IO_PSECTION     0x50000000  /* 0x50000000-0x53000000   */
#define VIRT_DDR_PSECTION        0x60000000  /* 0x60000000-0x64000000   */
#define VIRT_GIC_PSECTION        0xa0000000  /* 0xa0000000-0xa0200000   */

/* RTL8730E Virtual Memory Map **********************************************/

#define VIRT_FLASH_VSECTION      VIRT_FLASH_PSECTION
#define VIRT_KM0_VSECTION        VIRT_KM0_PSECTION
#define VIRT_IO_VSECTION         VIRT_IO_PSECTION
#define VIRT_SEC_IO_VSECTION     VIRT_SEC_IO_PSECTION
#define VIRT_DDR_VSECTION        VIRT_DDR_PSECTION
#define VIRT_GIC_VSECTION        VIRT_GIC_PSECTION

/* Sizes of memory regions in bytes. */

#define VIRT_FLASH_SECSIZE       (128*1024*1024)  /* 128 MiB XIP window */
#define VIRT_KM4_SRAM_SECSIZE    (1*1024*1024)    /* 1 MiB covers 0x2001c01c */
#define VIRT_KM0_SECSIZE         (128*1024)       /* 128 KB, rounds to 1 MMU section */
#define VIRT_IO_SECSIZE          (80*1024*1024)   /* 0x40000000-0x44FFFFFF: APB + SPIC ctrl */
#define VIRT_SEC_IO_SECSIZE      (48*1024*1024)
#define VIRT_DDR_SECSIZE         (64*1024*1024)
#define VIRT_GIC_SECSIZE         (2*1024*1024)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int rtl8730e_setupmappings(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RTL8730E_MEMORYMAP_H */
