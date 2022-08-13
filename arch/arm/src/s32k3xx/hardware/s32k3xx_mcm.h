/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_mcm.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MCM_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MCM Register Offsets *****************************************************/

#define S32K3XX_MCM_PLREV_OFFSET     (0x0000) /* SoC-defined Platform Revision Register (PLREV) */
#define S32K3XX_MCM_PCT_OFFSET       (0x0002) /* Processor Core Type Register (PCT) */
#define S32K3XX_MCM_CPCR_OFFSET      (0x000c) /* Core Platform Control Register (CPCR) */
#define S32K3XX_MCM_ISCR_OFFSET      (0x0010) /* Interrupt Status and Control Register (ISCR) */
#define S32K3XX_MCM_LMEMDESC0_OFFSET (0x0400) /* Local Memory Descriptor 0 Register (LMEM_DESC_0) */
#define S32K3XX_MCM_LMEMDESC1_OFFSET (0x0404) /* Local Memory Descriptor 1 Register (LMEM_DESC_1) */
#define S32K3XX_MCM_LMEMDESC2_OFFSET (0x0408) /* Local Memory Descriptor 2 Register (LMEM_DESC_2) */
#define S32K3XX_MCM_LMEMDESC3_OFFSET (0x040c) /* Local Memory Descriptor 3 Register (LMEM_DESC_3) */
#define S32K3XX_MCM_LMEMDESC4_OFFSET (0x0410) /* Local Memory Descriptor 4 Register (LMEM_DESC_4) */

/* MCM Register Addresses ***************************************************/

#define S32K3XX_MCM_PLREV            (S32K3XX_MCM_BASE + S32K3XX_MCM_PLREV_OFFSET)
#define S32K3XX_MCM_PCT              (S32K3XX_MCM_BASE + S32K3XX_MCM_PCT_OFFSET)
#define S32K3XX_MCM_CPCR             (S32K3XX_MCM_BASE + S32K3XX_MCM_CPCR_OFFSET)
#define S32K3XX_MCM_ISCR             (S32K3XX_MCM_BASE + S32K3XX_MCM_ISCR_OFFSET)
#define S32K3XX_MCM_LMEMDESC0        (S32K3XX_MCM_BASE + S32K3XX_MCM_LMEMDESC0_OFFSET)
#define S32K3XX_MCM_LMEMDESC1        (S32K3XX_MCM_BASE + S32K3XX_MCM_LMEMDESC1_OFFSET)
#define S32K3XX_MCM_LMEMDESC2        (S32K3XX_MCM_BASE + S32K3XX_MCM_LMEMDESC2_OFFSET)
#define S32K3XX_MCM_LMEMDESC3        (S32K3XX_MCM_BASE + S32K3XX_MCM_LMEMDESC3_OFFSET)
#define S32K3XX_MCM_LMEMDESC4        (S32K3XX_MCM_BASE + S32K3XX_MCM_LMEMDESC4_OFFSET)

/* MCM Register Bitfield Definitions ****************************************/

/* SoC-defined Platform Revision Register (PLREV) */

#define MCM_PLREV_SHIFT              (0)       /* Bits 0-15: SoC-defined Platform Revision (PLREV) */
#define MCM_PLREV_MASK               (0xffff << MCM_PLREV_SHIFT)

/* Processor Core Type Register (PCT) */

#define MCM_PCT_SHIFT                (0)       /* Bits 0-15: Processor Core Type (PCT) */
#define MCM_PCT_MASK                 (0xffff << MCM_PCT_SHIFT)
#  define MCM_PCT_CM7                (0xac70 << MCM_PCT_SHIFT) /* Cortex-M7 */

/* Core Platform Control Register (CPCR) */

                                               /* Bits 0-26: Reserved */
#define MCM_CPCR_CM7_AHBSPRI         (1 << 27) /* Bit 27: AHB Slave Priority (CM7_AHBSPRI) */
                                               /* Bits 28-31: Reserved */

/* Interrupt Status and Control Register (ISCR) */

                                               /* Bits 0-4: Reserved */
#define MCM_ISCR_WABS                (1 << 5)  /* Bit 5: Write Abort on Slave (WABS) */
#define MCM_ISCR_WABSO               (1 << 6)  /* Bit 6: Write Abort on Slave Overrun (WABSO) */
                                               /* Bit 7: Reserved */
#define MCM_ISCR_FIOC                (1 << 8)  /* Bit 8: FPU Invalid Operation Interrupt Status (FIOC) */
#define MCM_ISCR_FDZC                (1 << 9)  /* Bit 9: FPU Divide-by-Zero Interrupt Status (FDZC) */
#define MCM_ISCR_FOFC                (1 << 10) /* Bit 10: FPU Overflow Interrupt Status (FOFC) */
#define MCM_ISCR_FUFC                (1 << 11) /* Bit 11: FPU Underflow Interrupt Status (FUFC) */
#define MCM_ISCR_FIXC                (1 << 12) /* Bit 12: FPU Inexact Interrupt Status (FIXC) */
                                               /* Bits 13-14: Reserved */
#define MCM_ISCR_FIDC                (1 << 15) /* Bit 15: FPU Input Denormal Interrupt Status (FIDC) */
                                               /* Bits 16-20: Reserved */
#define MCM_ISCR_WABE                (1 << 21) /* Bit 21: TCM Write Abort Interrupt Enable (WABE) */
                                               /* Bits 22-23: Reserved */
#define MCM_ISCR_FIOCE               (1 << 24) /* Bit 24: FPU Invalid Operation Interrupt Enable (FIOCE) */
#define MCM_ISCR_FDZCE               (1 << 25) /* Bit 25: FPU Divide-by-Zero Interrupt Enable (FDZCE) */
#define MCM_ISCR_FOFCE               (1 << 26) /* Bit 26: FPU Overflow Interrupt Enable (FOFCE) */
#define MCM_ISCR_FUFCE               (1 << 27) /* Bit 27: FPU Underflow Interrupt Enable (FUFCE) */
#define MCM_ISCR_FIXCE               (1 << 28) /* Bit 28: FPU Inexact Interrupt Enable (FIXCE) */
                                               /* Bits 29-30: Reserved */
#define MCM_ISCR_FIDCE               (1 << 31) /* Bit 31: FPU Input Denormal Interrupt Enable (FIDCE) */

/* Local Memory Descriptor n Register (LMEM_DESC_n) */

                                               /* Bits 0-12: Reserved */
#define MCM_LMEMDESC_MT_SHIFT        (13)      /* Bits 13-15: Memory Type (MT) */
#define MCM_LMEMDESC_MT_MASK         (0x07 << MCM_LMEMDESC_MT_SHIFT)
#  define MCM_LMEMDESC_MT_ITCM       (0x00 << MCM_LMEMDESC_MT_SHIFT) /* ITCM */
#  define MCM_LMEMDESC_MT_DTCM       (0x01 << MCM_LMEMDESC_MT_SHIFT) /* DTCM */
#  define MCM_LMEMDESC_MT_ICACHE     (0x02 << MCM_LMEMDESC_MT_SHIFT) /* ICACHE */
#  define MCM_LMEMDESC_MT_DCACHE     (0x03 << MCM_LMEMDESC_MT_SHIFT) /* DCACHE */

                                               /* Bits 16: Reserved */
#define MCM_LMEMDESC_DPW_SHIFT       (17)      /* Bits 17-19: Data Path Width (DPW) */
#define MCM_LMEMDESC_DPW_MASK        (0x07 << MCM_LMEMDESC_DPW_SHIFT)
#  define MCM_LMEMDESC_DPW_32        (0x02 << MCM_LMEMDESC_DPW_SHIFT) /* LMEM0 is 32-bits wide */
#  define MCM_LMEMDESC_DPW_64        (0x03 << MCM_LMEMDESC_DPW_SHIFT) /* LMEM0 is 64-bits wide */

#define MCM_LMEMDESC_WY_SHIFT        (20)      /* Bits 20-23: Level 1 Cache Ways (WY) */
#define MCM_LMEMDESC_WY_MASK         (0x0f << MCM_LMEMDESC_WY_SHIFT)
#  define MCM_LMEMDESC_WY_NOCACHE    (0x00 << MCM_LMEMDESC_WY_SHIFT) /* No cache */
#  define MCM_LMEMDESC_WY_2WAYSA     (0x02 << MCM_LMEMDESC_WY_SHIFT) /* 2-way set associative */
#  define MCM_LMEMDESC_WY_4WAYSA     (0x04 << MCM_LMEMDESC_WY_SHIFT) /* 4-way set associative */

#define MCM_LMEMDESC_LMSZ_SHIFT      (24)      /* Bits 24-27: Local Memory Size (LMSZ) */
#define MCM_LMEMDESC_LMSZ_MASK       (0x0f << MCM_LMEMDESC_LMSZ_SHIFT)
#  define MCM_LMEMDESC_LMSZ_0K       (0x00 << MCM_LMEMDESC_LMSZ_SHIFT) /* 0 KB */
#  define MCM_LMEMDESC_LMSZ_1K       (0x01 << MCM_LMEMDESC_LMSZ_SHIFT) /* 1 KB */
#  define MCM_LMEMDESC_LMSZ_2K       (0x02 << MCM_LMEMDESC_LMSZ_SHIFT) /* 2 KB */
#  define MCM_LMEMDESC_LMSZ_4K       (0x03 << MCM_LMEMDESC_LMSZ_SHIFT) /* 4 KB */
#  define MCM_LMEMDESC_LMSZ_8K       (0x04 << MCM_LMEMDESC_LMSZ_SHIFT) /* 8 KB */
#  define MCM_LMEMDESC_LMSZ_16K      (0x05 << MCM_LMEMDESC_LMSZ_SHIFT) /* 16 KB */
#  define MCM_LMEMDESC_LMSZ_32K      (0x06 << MCM_LMEMDESC_LMSZ_SHIFT) /* 32 KB */
#  define MCM_LMEMDESC_LMSZ_64K      (0x07 << MCM_LMEMDESC_LMSZ_SHIFT) /* 64 KB */
#  define MCM_LMEMDESC_LMSZ_128K     (0x08 << MCM_LMEMDESC_LMSZ_SHIFT) /* 128 KB */
#  define MCM_LMEMDESC_LMSZ_256K     (0x09 << MCM_LMEMDESC_LMSZ_SHIFT) /* 256 KB */
#  define MCM_LMEMDESC_LMSZ_512K     (0x0a << MCM_LMEMDESC_LMSZ_SHIFT) /* 512 KB */
#  define MCM_LMEMDESC_LMSZ_1024K    (0x0b << MCM_LMEMDESC_LMSZ_SHIFT) /* 1024 KB */
#  define MCM_LMEMDESC_LMSZ_2048K    (0x0c << MCM_LMEMDESC_LMSZ_SHIFT) /* 2048 KB */
#  define MCM_LMEMDESC_LMSZ_4096K    (0x0d << MCM_LMEMDESC_LMSZ_SHIFT) /* 4096 KB */
#  define MCM_LMEMDESC_LMSZ_8192K    (0x0e << MCM_LMEMDESC_LMSZ_SHIFT) /* 8192 KB */
#  define MCM_LMEMDESC_LMSZ_16384K   (0x0f << MCM_LMEMDESC_LMSZ_SHIFT) /* 16384 KB */

#define MCM_LMEMDESC_LMSZH           (1 << 28) /* Bit 28: LMEM Size Hole (LMSZH) */
                                               /* Bits 29-30: Reserved */
#define MCM_LMEMDESC_LMV             (1 << 31) /* Bit 31: Local Memory Valid (LMV) */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MCM_H */
