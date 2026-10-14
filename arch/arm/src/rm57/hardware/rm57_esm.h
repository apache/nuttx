/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_esm.h
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

/* Register layout taken from TI's HALCoGen HL_reg_esm.h for
 * RM57L843 - matches TMS570's ESM address (0xfffff500). See
 * rm57_esm.c for the early-boot ESM quiesce sequence.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_ESM_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_ESM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RM57_ESM_EEPAPR1_OFFSET   0x0000 /* Error Enable Primary Register 1 */

/* Error Disable Primary Register 1 */
#define RM57_ESM_DEPAPR1_OFFSET 0x0004

/* Interrupt Enable Set/Status Register 1 */
#define RM57_ESM_IESR1_OFFSET 0x0008

/* Interrupt Enable Clear Register 1 */
#define RM57_ESM_IECR1_OFFSET 0x000c
#define RM57_ESM_ILSR1_OFFSET     0x0010 /* Interrupt Level Set Register 1 */

/* Interrupt Level Clear Register 1 */
#define RM57_ESM_ILCR1_OFFSET 0x0014

/* Status Register 1, n=0..2 */
#define RM57_ESM_SR1_OFFSET(n) (0x0018 + ((n) << 2))
#define RM57_ESM_EPSR_OFFSET      0x0024 /* Error Pin Status Register */
#define RM57_ESM_IOFFHR_OFFSET    0x0028 /* Interrupt Offset High Register */
#define RM57_ESM_IOFFLR_OFFSET    0x002c /* Interrupt Offset Low Register */
#define RM57_ESM_LTCR_OFFSET      0x0030 /* Low Time Counter Register */

/* Low Time Counter Preload Register */
#define RM57_ESM_LTCPR_OFFSET 0x0034
#define RM57_ESM_EKR_OFFSET       0x0038 /* Error Key Register */
#define RM57_ESM_SSR2_OFFSET      0x003c /* Status Shadow Register 2 */

/* Interrupt Enable Primary Status Register 4 */
#define RM57_ESM_IEPSR4_OFFSET 0x0040

/* Interrupt Enable Primary Clear Register 4 */
#define RM57_ESM_IEPCR4_OFFSET 0x0044

/* Interrupt Enable Set/Status Register 4 */
#define RM57_ESM_IESR4_OFFSET 0x0048

/* Interrupt Enable Clear Register 4 */
#define RM57_ESM_IECR4_OFFSET 0x004c
#define RM57_ESM_ILSR4_OFFSET     0x0050 /* Interrupt Level Set Register 4 */

/* Interrupt Level Clear Register 4 */
#define RM57_ESM_ILCR4_OFFSET 0x0054

/* Status Register 4, n=0..2 */
#define RM57_ESM_SR4_OFFSET(n) (0x0058 + ((n) << 2))

/* Interrupt Enable Primary Status Register 7 */
#define RM57_ESM_IEPSR7_OFFSET 0x0080

/* Interrupt Enable Primary Clear Register 7 */
#define RM57_ESM_IEPCR7_OFFSET 0x0084

/* Interrupt Enable Set/Status Register 7 */
#define RM57_ESM_IESR7_OFFSET 0x0088

/* Interrupt Enable Clear Register 7 */
#define RM57_ESM_IECR7_OFFSET 0x008c
#define RM57_ESM_ILSR7_OFFSET     0x0090 /* Interrupt Level Set Register 7 */

/* Interrupt Level Clear Register 7 */
#define RM57_ESM_ILCR7_OFFSET 0x0094

/* Status Register 7, n=0..2 */
#define RM57_ESM_SR7_OFFSET(n) (0x0098 + ((n) << 2))

/* Register Addresses *******************************************************/

#define RM57_ESM_EEPAPR1          (RM57_ESM_BASE + RM57_ESM_EEPAPR1_OFFSET)
#define RM57_ESM_DEPAPR1          (RM57_ESM_BASE + RM57_ESM_DEPAPR1_OFFSET)
#define RM57_ESM_IESR1            (RM57_ESM_BASE + RM57_ESM_IESR1_OFFSET)
#define RM57_ESM_IECR1            (RM57_ESM_BASE + RM57_ESM_IECR1_OFFSET)
#define RM57_ESM_ILSR1            (RM57_ESM_BASE + RM57_ESM_ILSR1_OFFSET)
#define RM57_ESM_ILCR1            (RM57_ESM_BASE + RM57_ESM_ILCR1_OFFSET)
#define RM57_ESM_EPSR             (RM57_ESM_BASE + RM57_ESM_EPSR_OFFSET)
#define RM57_ESM_IOFFHR           (RM57_ESM_BASE + RM57_ESM_IOFFHR_OFFSET)
#define RM57_ESM_IOFFLR           (RM57_ESM_BASE + RM57_ESM_IOFFLR_OFFSET)
#define RM57_ESM_LTCR             (RM57_ESM_BASE + RM57_ESM_LTCR_OFFSET)
#define RM57_ESM_LTCPR            (RM57_ESM_BASE + RM57_ESM_LTCPR_OFFSET)
#define RM57_ESM_EKR              (RM57_ESM_BASE + RM57_ESM_EKR_OFFSET)
#define RM57_ESM_SR1(n)           (RM57_ESM_BASE + RM57_ESM_SR1_OFFSET(n))
#define RM57_ESM_SR2              RM57_ESM_SR1(1)
#define RM57_ESM_SR3              RM57_ESM_SR1(2)
#define RM57_ESM_SSR2             (RM57_ESM_BASE + RM57_ESM_SSR2_OFFSET)
#define RM57_ESM_IEPSR4           (RM57_ESM_BASE + RM57_ESM_IEPSR4_OFFSET)
#define RM57_ESM_IEPCR4           (RM57_ESM_BASE + RM57_ESM_IEPCR4_OFFSET)
#define RM57_ESM_IESR4            (RM57_ESM_BASE + RM57_ESM_IESR4_OFFSET)
#define RM57_ESM_IECR4            (RM57_ESM_BASE + RM57_ESM_IECR4_OFFSET)
#define RM57_ESM_ILSR4            (RM57_ESM_BASE + RM57_ESM_ILSR4_OFFSET)
#define RM57_ESM_ILCR4            (RM57_ESM_BASE + RM57_ESM_ILCR4_OFFSET)
#define RM57_ESM_SR4(n)           (RM57_ESM_BASE + RM57_ESM_SR4_OFFSET(n))
#define RM57_ESM_IEPSR7           (RM57_ESM_BASE + RM57_ESM_IEPSR7_OFFSET)
#define RM57_ESM_IEPCR7           (RM57_ESM_BASE + RM57_ESM_IEPCR7_OFFSET)
#define RM57_ESM_IESR7            (RM57_ESM_BASE + RM57_ESM_IESR7_OFFSET)
#define RM57_ESM_IECR7            (RM57_ESM_BASE + RM57_ESM_IECR7_OFFSET)
#define RM57_ESM_ILSR7            (RM57_ESM_BASE + RM57_ESM_ILSR7_OFFSET)
#define RM57_ESM_ILCR7            (RM57_ESM_BASE + RM57_ESM_ILCR7_OFFSET)
#define RM57_ESM_SR7(n)           (RM57_ESM_BASE + RM57_ESM_SR7_OFFSET(n))

/* Register Bit-Field Definitions *******************************************/

/* Interrupt Offset High/Low Registers (IOFFHR/IOFFLR)
 *
 * The offset registers report the highest-priority pending channel on the
 * high-level (FIQ) and low-level (IRQ) lines respectively.  Zero means no
 * interrupt is pending; otherwise the value encodes both the error group
 * and the channel within it, in the banded layout below (SPNU562A
 * Table 16-13).  Group2 outranks group1, and within a group channel 0 has
 * the highest priority.
 *
 * Reading an offset register clears the corresponding flag in ESMSR2 only.
 * Group1 flags survive the read and must be cleared by writing a 1 to the
 * matching bit of SR1/SR4/SR7, or the source keeps re-asserting.
 */

#define ESM_IOFF_MASK             (0xff)  /* Bits 0-7: channel/group code */
#define ESM_IOFF_NONE             (0x00)  /* No interrupt pending */

#define ESM_IOFF_GRP1_LO_FIRST    (0x01)  /* 0x01-0x20: group1, ch 0-31 */
#define ESM_IOFF_GRP1_LO_LAST     (0x20)
#define ESM_IOFF_GRP2_FIRST       (0x21)  /* 0x21-0x40: group2, ch 0-31 */
#define ESM_IOFF_GRP2_LAST        (0x40)
#define ESM_IOFF_GRP1_MID_FIRST   (0x41)  /* 0x41-0x60: group1, ch 32-63 */
#define ESM_IOFF_GRP1_MID_LAST    (0x60)
#define ESM_IOFF_GRP1_HI_FIRST    (0x81)  /* 0x81-0xa0: group1, ch 64-95 */
#define ESM_IOFF_GRP1_HI_LAST     (0xa0)

/* Error Pin Status Register */

#define ESM_EPSR_EPSF             (1 << 0) /* Bit 0: 1 = nERROR pin inactive */

/* Error Key Register */

#define ESM_EKR_NORMAL            (0x00)  /* Normal mode */
#define ESM_EKR_RELEASE           (0x05)  /* Release nERROR once LTC expires */
#define ESM_EKR_FORCE             (0x0a)  /* Error forcing: drive nERROR low */

/* Channel assignments used by the start-up diagnostics (SPNS215C
 * Table 6-45).  Group1 channel n lives in SR1 (0-31), SR4 (32-63) or SR7
 * (64-95) at bit (n % 32); group2 is SR2/SSR2 and group3 is SR3.
 */

#define ESM_G1_CPU_SELFTEST       (27)    /* Cortex-R5F LBIST (STC1) error */
#define ESM_G1_CCM_SELFTEST       (31)    /* CCM-R5F self-test error */
#define ESM_G1_PD_COMPARE         (38)    /* Power domain compare error */
#define ESM_G1_PD_SELFTEST        (39)    /* Power domain self-test error */
#define ESM_G1_EFUSE_ERROR        (40)    /* eFuse farm EFC error */
#define ESM_G1_EFUSE_SELFTEST     (41)    /* eFuse farm self-test error */
#define ESM_G1_INTERCONNECT       (52)    /* CPU interconnect global error */
#define ESM_G1_N2HET_SELFTEST     (54)    /* N2HET LBIST (STC2) error */
#define ESM_G1_CCM_STATUS         (92)    /* CCM-R5F operating status */

#define ESM_G2_CCM_CPU            (2)     /* CCM-R5F CPU compare error */
#define ESM_G2_CPU_BUSERR         (3)     /* Cortex-R5F fatal bus error */
#define ESM_G2_L2RAMW_TYPEB       (7)     /* L2RAMW uncorrectable type B */
#define ESM_G2_CCM_VIM            (25)    /* CCM-R5F VIM compare error */
#define ESM_G2_CCM_INACTIVITY     (26)    /* CPU1 AXIM bus monitor failure */
#define ESM_G2_CCM_PD             (28)    /* CCM-R5F power domain monitor */

#define ESM_G3_EFUSE_AUTOLOAD     (1)     /* eFuse farm autoload error */

#define ESM_CHANBIT(n)            (1 << ((n) % 32))

/* Channels per status register, and the total the offset encoding spans */

#define ESM_CHANS_PER_REG         (32)
#define ESM_NCHANNELS             (128)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_ESM_H */
