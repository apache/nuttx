/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_vim.h
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

/* Register layout taken from TI's HALCoGen HL_reg_vim.h for
 * RM57L843. Note RM57's VIM register frame (0xfffffd00) is at a
 * different address than TMS570LS04x's VIM (0xfffffe00) - do not assume
 * parity with the sibling tms570_vim.h.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_VIM_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_VIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VIM has 128 total channel slots: one phantom + 127 real channels
 * (channel 127 has no dedicated vector, per the HALCoGen s_vim_init table)
 */

#define VIM_CHANNELS                    128

/* Helpers for indexing into the 4 groups of 32-bit-per-channel
 * registers (FIRQPRn/INTREQn/REQMASKSETn/REQMASKCLRn/WAKEMASKSETn/
 * WAKEMASKCLRn)
 */

#define VIM_REGNDX(ch)                   ((ch) >> 5)
#define VIM_REGBIT(ch)                   ((ch) & 31)

/* Register Offsets *********************************************************/

#define RM57_VIM_ECCSTAT_OFFSET         0x00ec /* ECC Status Register */
#define RM57_VIM_ECCCTL_OFFSET          0x00f0 /* ECC Control Register */

/* Uncorrectable Error Address Register */
#define RM57_VIM_UERRADDR_OFFSET 0x00f4

/* Fall-Back Address Register */
#define RM57_VIM_FBVECADDR_OFFSET 0x00f8

/* Single Bit Error Address Register */
#define RM57_VIM_SBERRADDR_OFFSET 0x00fc

/* IRQ Index Offset Vector Register */
#define RM57_VIM_IRQINDEX_OFFSET 0x0100

/* FIQ Index Offset Vector Register */
#define RM57_VIM_FIQINDEX_OFFSET 0x0104

/* FIQ/IRQ Program Register 0 */
#define RM57_VIM_FIRQPR0_OFFSET 0x0110

/* FIQ/IRQ Program Register 1 */
#define RM57_VIM_FIRQPR1_OFFSET 0x0114

/* FIQ/IRQ Program Register 2 */
#define RM57_VIM_FIRQPR2_OFFSET 0x0118

/* FIQ/IRQ Program Register 3 */
#define RM57_VIM_FIRQPR3_OFFSET 0x011c

/* Pending Interrupt Read Location Register 0 */
#define RM57_VIM_INTREQ0_OFFSET 0x0120

/* Pending Interrupt Read Location Register 1 */
#define RM57_VIM_INTREQ1_OFFSET 0x0124

/* Pending Interrupt Read Location Register 2 */
#define RM57_VIM_INTREQ2_OFFSET 0x0128

/* Pending Interrupt Read Location Register 3 */
#define RM57_VIM_INTREQ3_OFFSET 0x012c

/* Interrupt Enable Set Register 0 */
#define RM57_VIM_REQMASKSET0_OFFSET 0x0130

/* Interrupt Enable Set Register 1 */
#define RM57_VIM_REQMASKSET1_OFFSET 0x0134

/* Interrupt Enable Set Register 2 */
#define RM57_VIM_REQMASKSET2_OFFSET 0x0138

/* Interrupt Enable Set Register 3 */
#define RM57_VIM_REQMASKSET3_OFFSET 0x013c

/* Interrupt Enable Clear Register 0 */
#define RM57_VIM_REQMASKCLR0_OFFSET 0x0140

/* Interrupt Enable Clear Register 1 */
#define RM57_VIM_REQMASKCLR1_OFFSET 0x0144

/* Interrupt Enable Clear Register 2 */
#define RM57_VIM_REQMASKCLR2_OFFSET 0x0148

/* Interrupt Enable Clear Register 3 */
#define RM57_VIM_REQMASKCLR3_OFFSET 0x014c

/* Wake-up Enable Set Register 0 */
#define RM57_VIM_WAKEMASKSET0_OFFSET 0x0150

/* Wake-up Enable Set Register 1 */
#define RM57_VIM_WAKEMASKSET1_OFFSET 0x0154

/* Wake-up Enable Set Register 2 */
#define RM57_VIM_WAKEMASKSET2_OFFSET 0x0158

/* Wake-up Enable Set Register 3 */
#define RM57_VIM_WAKEMASKSET3_OFFSET 0x015c

/* Wake-up Enable Clear Register 0 */
#define RM57_VIM_WAKEMASKCLR0_OFFSET 0x0160

/* Wake-up Enable Clear Register 1 */
#define RM57_VIM_WAKEMASKCLR1_OFFSET 0x0164

/* Wake-up Enable Clear Register 2 */
#define RM57_VIM_WAKEMASKCLR2_OFFSET 0x0168

/* Wake-up Enable Clear Register 3 */
#define RM57_VIM_WAKEMASKCLR3_OFFSET 0x016c

/* IRQ Interrupt Vector Register */
#define RM57_VIM_IRQVECREG_OFFSET 0x0170

/* FIQ Interrupt Vector Register */
#define RM57_VIM_FIQVECREG_OFFSET 0x0174
#define RM57_VIM_CAPEVT_OFFSET          0x0178 /* Capture Event Register */

/* Channel Control Register n/4, n=0..31 */
#define RM57_VIM_CHANCTRL_OFFSET(n) (0x0180 + (((n) >> 2) << 2))

/* Register Addresses *******************************************************/

#define RM57_VIM_ECCSTAT       (RM57_VIM_BASE + RM57_VIM_ECCSTAT_OFFSET)
#define RM57_VIM_ECCCTL        (RM57_VIM_BASE + RM57_VIM_ECCCTL_OFFSET)
#define RM57_VIM_UERRADDR      (RM57_VIM_BASE + RM57_VIM_UERRADDR_OFFSET)
#define RM57_VIM_FBVECADDR     (RM57_VIM_BASE + RM57_VIM_FBVECADDR_OFFSET)
#define RM57_VIM_SBERRADDR     (RM57_VIM_BASE + RM57_VIM_SBERRADDR_OFFSET)
#define RM57_VIM_IRQINDEX      (RM57_VIM_BASE + RM57_VIM_IRQINDEX_OFFSET)
#define RM57_VIM_FIQINDEX      (RM57_VIM_BASE + RM57_VIM_FIQINDEX_OFFSET)
#define RM57_VIM_FIRQPR0       (RM57_VIM_BASE + RM57_VIM_FIRQPR0_OFFSET)
#define RM57_VIM_FIRQPR1       (RM57_VIM_BASE + RM57_VIM_FIRQPR1_OFFSET)
#define RM57_VIM_FIRQPR2       (RM57_VIM_BASE + RM57_VIM_FIRQPR2_OFFSET)
#define RM57_VIM_FIRQPR3       (RM57_VIM_BASE + RM57_VIM_FIRQPR3_OFFSET)
#define RM57_VIM_INTREQ0       (RM57_VIM_BASE + RM57_VIM_INTREQ0_OFFSET)
#define RM57_VIM_INTREQ1       (RM57_VIM_BASE + RM57_VIM_INTREQ1_OFFSET)
#define RM57_VIM_INTREQ2       (RM57_VIM_BASE + RM57_VIM_INTREQ2_OFFSET)
#define RM57_VIM_INTREQ3       (RM57_VIM_BASE + RM57_VIM_INTREQ3_OFFSET)
#define RM57_VIM_REQMASKSET0   (RM57_VIM_BASE + RM57_VIM_REQMASKSET0_OFFSET)
#define RM57_VIM_REQMASKSET1   (RM57_VIM_BASE + RM57_VIM_REQMASKSET1_OFFSET)
#define RM57_VIM_REQMASKSET2   (RM57_VIM_BASE + RM57_VIM_REQMASKSET2_OFFSET)
#define RM57_VIM_REQMASKSET3   (RM57_VIM_BASE + RM57_VIM_REQMASKSET3_OFFSET)
#define RM57_VIM_REQMASKCLR0   (RM57_VIM_BASE + RM57_VIM_REQMASKCLR0_OFFSET)
#define RM57_VIM_REQMASKCLR1   (RM57_VIM_BASE + RM57_VIM_REQMASKCLR1_OFFSET)
#define RM57_VIM_REQMASKCLR2   (RM57_VIM_BASE + RM57_VIM_REQMASKCLR2_OFFSET)
#define RM57_VIM_REQMASKCLR3   (RM57_VIM_BASE + RM57_VIM_REQMASKCLR3_OFFSET)
#define RM57_VIM_IRQVECREG     (RM57_VIM_BASE + RM57_VIM_IRQVECREG_OFFSET)
#define RM57_VIM_FIQVECREG     (RM57_VIM_BASE + RM57_VIM_FIQVECREG_OFFSET)
#define RM57_VIM_CAPEVT        (RM57_VIM_BASE + RM57_VIM_CAPEVT_OFFSET)
#define RM57_VIM_CHANCTRL(n)   (RM57_VIM_BASE + RM57_VIM_CHANCTRL_OFFSET(n))

/* Generic register-group accessors, indexed by VIM_REGNDX(channel) */

#define RM57_VIM_FIRQPR(n)     (RM57_VIM_BASE + 0x0110 + ((n) << 2))
#define RM57_VIM_REQMASKSET(n) (RM57_VIM_BASE + 0x0130 + ((n) << 2))
#define RM57_VIM_REQMASKCLR(n) (RM57_VIM_BASE + 0x0140 + ((n) << 2))

/* Register Bit-Field Definitions *******************************************/

/* ECC Control Register: default HALCoGen init value, Errata VIM#28
 * workaround (disable single-bit correction)
 */

#define VIM_ECCCTL_INIT (((uint32_t)0xa << 0) | ((uint32_t)0x5 << 16))

/* FIQ/IRQ Program Registers: 0 = IRQ, 1 = FIQ (per channel bit) */

#define VIM_FIRQPR_IRQ            0
#define VIM_FIRQPR_FIQ            1

/* IRQ/FIQ Index Offset Vector Registers */

#define VIM_IRQINDEX_MASK         (0x000000ff)
#define VIM_FIQINDEX_MASK         (0x000000ff)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_VIM_H */
