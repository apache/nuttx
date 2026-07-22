/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_sys.h
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

/* Register layout taken from TI's HALCoGen HL_reg_system.h for
 * RM57L843. System Frame 1 (systemREG1, 0xffffff00) and Frame 2
 * (systemREG2, 0xffffe100) match TMS570's addresses - this is common
 * Hercules "SYS" module infrastructure. Field-level PLL/clock values
 * (BOARD_PLL_* etc) are supplied per-board (arch/board/board.h), not
 * here.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_SYS_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_SYS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Frame 1 Register Offsets ******************************************/

#define RM57_SYS_SYSPC1_OFFSET      0x0000 /* SYS Pin Control Register 1 */
#define RM57_SYS_SYSPC2_OFFSET      0x0004 /* SYS Pin Control Register 2 */
#define RM57_SYS_SYSPC3_OFFSET      0x0008 /* SYS Pin Control Register 3 */
#define RM57_SYS_SYSPC4_OFFSET      0x000c /* SYS Pin Control Register 4 */
#define RM57_SYS_SYSPC5_OFFSET      0x0010 /* SYS Pin Control Register 5 */
#define RM57_SYS_SYSPC6_OFFSET      0x0014 /* SYS Pin Control Register 6 */
#define RM57_SYS_SYSPC7_OFFSET      0x0018 /* SYS Pin Control Register 7 */
#define RM57_SYS_SYSPC8_OFFSET      0x001c /* SYS Pin Control Register 8 */
#define RM57_SYS_SYSPC9_OFFSET      0x0020 /* SYS Pin Control Register 9 */
#define RM57_SYS_CSDIS_OFFSET       0x0030 /* Clock Source Disable Register */

/* Clock Source Disable Set Register */
#define RM57_SYS_CSDISSET_OFFSET 0x0034

/* Clock Source Disable Clear Register */
#define RM57_SYS_CSDISCLR_OFFSET 0x0038
#define RM57_SYS_CDDIS_OFFSET       0x003c /* Clock Domain Disable Register */

/* Clock Domain Disable Set Register */
#define RM57_SYS_CDDISSET_OFFSET 0x0040

/* Clock Domain Disable Clear Register */
#define RM57_SYS_CDDISCLR_OFFSET 0x0044

/* GCLK, HCLK, VCLK, VCLK2 Source Register */
#define RM57_SYS_GHVSRC_OFFSET 0x0048

/* Peripheral Asynchronous Clock Source Register */
#define RM57_SYS_VCLKASRC_OFFSET 0x004c
#define RM57_SYS_RCLKSRC_OFFSET     0x0050 /* RTI Clock Source Register */

/* Clock Source Valid Status Register */
#define RM57_SYS_CSVSTAT_OFFSET 0x0054

/* Memory Self-Test Global Control Register */
#define RM57_SYS_MSTGCR_OFFSET 0x0058

/* Memory Hardware Initialization Global Control Register */
#define RM57_SYS_MINITGCR_OFFSET 0x005c

/* Memory Self-Test/Initialization Enable Register */
#define RM57_SYS_MSINENA_OFFSET 0x0060

/* Memory Self-Test Fail Status Register */
#define RM57_SYS_MSTFAIL_OFFSET 0x0064
#define RM57_SYS_MSTCGSTAT_OFFSET   0x0068 /* MSTC Global Status Register */

/* Memory Hardware Initialization Status Register */
#define RM57_SYS_MINISTAT_OFFSET 0x006c
#define RM57_SYS_PLLCTL1_OFFSET     0x0070 /* PLL Control Register 1 */
#define RM57_SYS_PLLCTL2_OFFSET     0x0074 /* PLL Control Register 2 */
#define RM57_SYS_SYSPC10_OFFSET     0x0078 /* SYS Pin Control Register 10 */

/* Die Identification Register, Lower Word */
#define RM57_SYS_DIEIDL_OFFSET 0x007c

/* Die Identification Register, Upper Word */
#define RM57_SYS_DIEIDH_OFFSET 0x0080

/* LPO/Clock Monitor Control Register */
#define RM57_SYS_LPOMONCTL_OFFSET 0x0088
#define RM57_SYS_CLKTEST_OFFSET     0x008c /* Clock Test Register */
#define RM57_SYS_DFTCTRLREG1_OFFSET 0x0090 /* DFT Control Register 1 */
#define RM57_SYS_DFTCTRLREG2_OFFSET 0x0094 /* DFT Control Register 2 */
#define RM57_SYS_GPREG1_OFFSET      0x00a0 /* General Purpose Register */

/* System Software Interrupt Request 1 Register */
#define RM57_SYS_SSIR1_OFFSET 0x00b0

/* System Software Interrupt Request 2 Register */
#define RM57_SYS_SSIR2_OFFSET 0x00b4

/* System Software Interrupt Request 3 Register */
#define RM57_SYS_SSIR3_OFFSET 0x00b8

/* System Software Interrupt Request 4 Register */
#define RM57_SYS_SSIR4_OFFSET 0x00bc
#define RM57_SYS_RAMGCR_OFFSET      0x00c0 /* RAM Control Register */

/* Bus Matrix Module Control Register 1 */
#define RM57_SYS_BMMCR1_OFFSET 0x00c4
#define RM57_SYS_CPURSTCR_OFFSET    0x00cc /* CPU Reset Control Register */
#define RM57_SYS_CLKCNTL_OFFSET     0x00d0 /* Clock Control Register */
#define RM57_SYS_ECPCNTL_OFFSET     0x00d4 /* ECP Control Register */
#define RM57_SYS_DEVCR1_OFFSET      0x00dc /* DEV Parity Control Register 1 */

/* System Exception Control Register */
#define RM57_SYS_SYSECR_OFFSET 0x00e0

/* System Exception Status Register */
#define RM57_SYS_SYSESR_OFFSET 0x00e4

/* System Test Abort Status Register */
#define RM57_SYS_SYSTASR_OFFSET 0x00e8
#define RM57_SYS_GBLSTAT_OFFSET     0x00ec /* Global Status Register */

/* Device Identification Register */
#define RM57_SYS_DEVID_OFFSET 0x00f0

/* Software Interrupt Vector Register */
#define RM57_SYS_SSIVEC_OFFSET 0x00f4

/* System Software Interrupt Flag Register */
#define RM57_SYS_SSIF_OFFSET 0x00f8

/* System Frame 2 Register Offsets ******************************************/

#define RM57_SYS2_PLLCTL3_OFFSET    0x0000 /* PLL Control Register 3 */
#define RM57_SYS2_STCCLKDIV_OFFSET  0x0008 /* STC Clock Divider Register */
#define RM57_SYS2_ECPCNTL_OFFSET    0x0024 /* ECP Control Register */
#define RM57_SYS2_ECPCNTL1_OFFSET   0x0028 /* ECP Control Register 1 */

/* Peripheral Asynchronous Clock 2 Control Register */
#define RM57_SYS2_CLK2CNTRL_OFFSET 0x003c

/* Peripheral Asynchronous Clock Control Register */
#define RM57_SYS2_VCLKACON1_OFFSET 0x0040
#define RM57_SYS2_HCLKCNTL_OFFSET   0x0054 /* HCLK Control Register */
#define RM57_SYS2_CLKSLIP_OFFSET    0x0070 /* Clock Slip Register */
#define RM57_SYS2_EFC_CTLEN_OFFSET  0x00ec /* EFC Control Enable Register */

/* Register Addresses (System Frame 1) **************************************/

#define RM57_SYS_CSDIS              (RM57_SYS1_BASE + RM57_SYS_CSDIS_OFFSET)
#define RM57_SYS_CSDISSET (RM57_SYS1_BASE + RM57_SYS_CSDISSET_OFFSET)
#define RM57_SYS_CSDISCLR (RM57_SYS1_BASE + RM57_SYS_CSDISCLR_OFFSET)
#define RM57_SYS_CDDIS              (RM57_SYS1_BASE + RM57_SYS_CDDIS_OFFSET)
#define RM57_SYS_CDDISSET (RM57_SYS1_BASE + RM57_SYS_CDDISSET_OFFSET)
#define RM57_SYS_CDDISCLR (RM57_SYS1_BASE + RM57_SYS_CDDISCLR_OFFSET)
#define RM57_SYS_GHVSRC             (RM57_SYS1_BASE + RM57_SYS_GHVSRC_OFFSET)
#define RM57_SYS_VCLKASRC (RM57_SYS1_BASE + RM57_SYS_VCLKASRC_OFFSET)
#define RM57_SYS_RCLKSRC            (RM57_SYS1_BASE + RM57_SYS_RCLKSRC_OFFSET)
#define RM57_SYS_CSVSTAT            (RM57_SYS1_BASE + RM57_SYS_CSVSTAT_OFFSET)
#define RM57_SYS_MSTGCR             (RM57_SYS1_BASE + RM57_SYS_MSTGCR_OFFSET)
#define RM57_SYS_MINITGCR (RM57_SYS1_BASE + RM57_SYS_MINITGCR_OFFSET)
#define RM57_SYS_MSINENA            (RM57_SYS1_BASE + RM57_SYS_MSINENA_OFFSET)
#define RM57_SYS_MSTCGSTAT (RM57_SYS1_BASE + RM57_SYS_MSTCGSTAT_OFFSET)
#define RM57_SYS_MINISTAT (RM57_SYS1_BASE + RM57_SYS_MINISTAT_OFFSET)
#define RM57_SYS_PLLCTL1            (RM57_SYS1_BASE + RM57_SYS_PLLCTL1_OFFSET)
#define RM57_SYS_PLLCTL2            (RM57_SYS1_BASE + RM57_SYS_PLLCTL2_OFFSET)
#define RM57_SYS_GPREG1             (RM57_SYS1_BASE + RM57_SYS_GPREG1_OFFSET)
#define RM57_SYS_RAMGCR             (RM57_SYS1_BASE + RM57_SYS_RAMGCR_OFFSET)
#define RM57_SYS_CPURSTCR (RM57_SYS1_BASE + RM57_SYS_CPURSTCR_OFFSET)
#define RM57_SYS_CLKCNTL            (RM57_SYS1_BASE + RM57_SYS_CLKCNTL_OFFSET)
#define RM57_SYS_SYSECR             (RM57_SYS1_BASE + RM57_SYS_SYSECR_OFFSET)
#define RM57_SYS_SYSESR             (RM57_SYS1_BASE + RM57_SYS_SYSESR_OFFSET)
#define RM57_SYS_GBLSTAT            (RM57_SYS1_BASE + RM57_SYS_GBLSTAT_OFFSET)
#define RM57_SYS_DEVID              (RM57_SYS1_BASE + RM57_SYS_DEVID_OFFSET)

/* Register Addresses (System Frame 2) **************************************/

#define RM57_SYS2_PLLCTL3 (RM57_SYS2_BASE + RM57_SYS2_PLLCTL3_OFFSET)
#define RM57_SYS2_STCCLKDIV (RM57_SYS2_BASE + RM57_SYS2_STCCLKDIV_OFFSET)
#define RM57_SYS2_HCLKCNTL (RM57_SYS2_BASE + RM57_SYS2_HCLKCNTL_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Clock Source Disable / Domain Disable Registers */

#define SYS_CSDIS_CLKSR0OFF          (1 << 0)  /* Bit 0: Oscillator */
#define SYS_CSDIS_CLKSR1OFF          (1 << 1)  /* Bit 1: PLL1 */
#define SYS_CSDIS_CLKSR3OFF          (1 << 3)  /* Bit 3: External clock in */
#define SYS_CSDIS_CLKSR4OFF          (1 << 4)  /* Bit 4: Low-freq LPO */
#define SYS_CSDIS_CLKSR5OFF          (1 << 5)  /* Bit 5: High-freq LPO */
#define SYS_CSDIS_CLKSR6OFF          (1 << 6)  /* Bit 6: PLL2 */

/* Bit 7: External clock in 2 */
#define SYS_CSDIS_CLKSR7OFF (1 << 7)

#define SYS_CDDIS_GCLKOFF            (1 << 0)  /* Bit 0: GCLK domain off */

/* Bit 1: HCLK/VCLK_sys domains off */
#define SYS_CDDIS_HCLKOFF (1 << 1)

/* Bit 2: VCLK_periph domain off */
#define SYS_CDDIS_VCLKPOFF (1 << 2)
#define SYS_CDDIS_VCLK2OFF           (1 << 3)  /* Bit 3: VCLK2 domain off */
#define SYS_CDDIS_VCLKA1OFF          (1 << 4)  /* Bit 4: VCLKA1 domain off */
#define SYS_CDDIS_RTICLK1OFF         (1 << 6)  /* Bit 6: RTICLK1 domain off */

/* GCLK, HCLK, VCLK, VCLK2 Source Register clock source aliases */

#define SYS_CLKSRC_OSC                0
#define SYS_CLKSRC_PLL1                1
#define SYS_CLKSRC_EXTERNAL1           3
#define SYS_CLKSRC_LPOLOW              4
#define SYS_CLKSRC_LPOHIGH             5
#define SYS_CLKSRC_PLL2                6
#define SYS_CLKSRC_EXTERNAL2           7
#define SYS_CLKSRC_VCLK                9

/* Bits 0-3: GCLK/HCLK/VCLK/VCLK2 current source */
#define SYS_GHVSRC_GHVSRC_SHIFT (0)
#define SYS_GHVSRC_GHVSRC_MASK        (15 << SYS_GHVSRC_GHVSRC_SHIFT)
#  define SYS_GHVSRC_GHVSRC_SRC(n) ((uint32_t)(n) << SYS_GHVSRC_GHVSRC_SHIFT)

/* Bits 16-19: HCLK/VCLK/VCLK2 source on wakeup */
#define SYS_GHVSRC_HVLPM_SHIFT (16)
#define SYS_GHVSRC_HVLPM_MASK         (15 << SYS_GHVSRC_HVLPM_SHIFT)
#  define SYS_GHVSRC_HVLPM(n) ((uint32_t)(n) << SYS_GHVSRC_HVLPM_SHIFT)

/* Bits 24-27: GCLK/HCLK/VCLK/VCLK2 source on wakeup */
#define SYS_GHVSRC_GHVWAKE_SHIFT (24)
#define SYS_GHVSRC_GHVWAKE_MASK       (15 << SYS_GHVSRC_GHVWAKE_SHIFT)
#  define SYS_GHVSRC_GHVWAKE(n) ((uint32_t)(n) << SYS_GHVSRC_GHVWAKE_SHIFT)

/* RTI Clock Source Register */

/* Bits 0-3: RTI clock1 source */
#define SYS_RCLKSRC_RTI1SRC_SHIFT (0)
#define SYS_RCLKSRC_RTI1SRC_MASK      (15 << SYS_RCLKSRC_RTI1SRC_SHIFT)
#  define SYS_RCLKSRC_RTI1SRC(n) ((uint32_t)(n) << SYS_RCLKSRC_RTI1SRC_SHIFT)

/* Bits 8-9: RTI clock1 divider */
#define SYS_RCLKSRC_RTI1DIV_SHIFT (8)
#define SYS_RCLKSRC_RTI1DIV_MASK      (3 << SYS_RCLKSRC_RTI1DIV_SHIFT)
#  define SYS_RCLKSRC_RTI1DIV_DIV1    (0 << SYS_RCLKSRC_RTI1DIV_SHIFT)
#  define SYS_RCLKSRC_RTI1DIV_DIV2    (1 << SYS_RCLKSRC_RTI1DIV_SHIFT)
#  define SYS_RCLKSRC_RTI1DIV_DIV4    (2 << SYS_RCLKSRC_RTI1DIV_SHIFT)
#  define SYS_RCLKSRC_RTI1DIV_DIV8    (3 << SYS_RCLKSRC_RTI1DIV_SHIFT)

/* Memory Self-Test / Hardware Init Global Control Registers (ECC RAM init) */

/* Bits 0-3: Memory self-test global enable key */
#define SYS_MSTGCR_MSTGENA_SHIFT (0)
#define SYS_MSTGCR_MSTGENA_MASK       (15 << SYS_MSTGCR_MSTGENA_SHIFT)
#  define SYS_MSTGCR_MSTGENA_ENABLE   (10 << SYS_MSTGCR_MSTGENA_SHIFT)
#  define SYS_MSTGCR_MSTGENA_DISABLE  (5 << SYS_MSTGCR_MSTGENA_SHIFT)

/* Bits 0-7: Memory hardware init key */
#define SYS_MINITGCR_MASK (0xff)
#  define SYS_MINITGCR_ENABLE         (0x0a)
#  define SYS_MINITGCR_DISABLE        (0x05)

/* Bit 0: Memory self-test done */
#define SYS_MSTCGSTAT_MSTDONE (1 << 0)

/* Bit 8: Hardware init of all memory done */
#define SYS_MSTCGSTAT_MINIDONE (1 << 8)

/* PLL Control Register 1 */

/* Bits 0-15: PLL multiplication factor */
#define SYS_PLLCTL1_PLLMUL_SHIFT (0)
#define SYS_PLLCTL1_PLLMUL_MASK       (0xffff << SYS_PLLCTL1_PLLMUL_SHIFT)
#  define SYS_PLLCTL1_PLLMUL(n) ((uint32_t)(n) << SYS_PLLCTL1_PLLMUL_SHIFT)

/* Bits 16-21: Reference clock divider */
#define SYS_PLLCTL1_REFCLKDIV_SHIFT (16)
#define SYS_PLLCTL1_REFCLKDIV_MASK    (0x3f << SYS_PLLCTL1_REFCLKDIV_SHIFT)
#  define SYS_PLLCTL1_REFCLKDIV(n)    ((uint32_t)(n) << \
                                        SYS_PLLCTL1_REFCLKDIV_SHIFT)

/* Bit 23: Reset on oscillator fail */
#define SYS_PLLCTL1_ROF (1 << 23)

/* Bits 24-28: PLL output clock divider */
#define SYS_PLLCTL1_PLLDIV_SHIFT (24)
#define SYS_PLLCTL1_PLLDIV_MASK       (0x1f << SYS_PLLCTL1_PLLDIV_SHIFT)
#  define SYS_PLLCTL1_PLLDIV(n) ((uint32_t)(n) << SYS_PLLCTL1_PLLDIV_SHIFT)

/* Bit 31: Reset on PLL slip */
#define SYS_PLLCTL1_ROS (1 << 31)

/* PLL Control Register 2 */

/* Bits 0-8: Spreading amount */
#define SYS_PLLCTL2_SPRAMOUNT_SHIFT (0)
#define SYS_PLLCTL2_SPRAMOUNT_MASK     (0xff << SYS_PLLCTL2_SPRAMOUNT_SHIFT)
#  define SYS_PLLCTL2_SPRAMOUNT(n)     ((uint32_t)(n) << \
                                         SYS_PLLCTL2_SPRAMOUNT_SHIFT)

/* Bits 9-11: Internal PLL output divider */
#define SYS_PLLCTL2_ODPLL_SHIFT (9)
#define SYS_PLLCTL2_ODPLL_MASK         (7 << SYS_PLLCTL2_ODPLL_SHIFT)
#  define SYS_PLLCTL2_ODPLL(n) ((uint32_t)(n) << SYS_PLLCTL2_ODPLL_SHIFT)

/* Bits 12-20: Multiplier correction (freq modulation) */
#define SYS_PLLCTL2_MULMOD_SHIFT (12)
#define SYS_PLLCTL2_MULMOD_MASK        (0x1ff << SYS_PLLCTL2_MULMOD_SHIFT)
#  define SYS_PLLCTL2_MULMOD(n) ((uint32_t)(n) << SYS_PLLCTL2_MULMOD_SHIFT)

/* Bit 31: Frequency modulation enable */
#define SYS_PLLCTL2_FMENA (1 << 31)

/* PLL Control Register 3 (System Frame 2) */

/* Bits 0-4: PLL2 output clock divider */
#define SYS2_PLLCTL3_PLLDIV_SHIFT (0)
#define SYS2_PLLCTL3_PLLDIV_MASK       (0x1f << SYS2_PLLCTL3_PLLDIV_SHIFT)
#  define SYS2_PLLCTL3_PLLDIV(n) ((uint32_t)(n) << SYS2_PLLCTL3_PLLDIV_SHIFT)

/* Bits 9-11: PLL2 internal output divider */
#define SYS2_PLLCTL3_ODPLL2_SHIFT (9)
#define SYS2_PLLCTL3_ODPLL2_MASK       (7 << SYS2_PLLCTL3_ODPLL2_SHIFT)
#  define SYS2_PLLCTL3_ODPLL2(n) ((uint32_t)(n) << SYS2_PLLCTL3_ODPLL2_SHIFT)

/* Bits 16-31: PLL2 multiplication factor */
#define SYS2_PLLCTL3_PLLMUL2_SHIFT (16)
#define SYS2_PLLCTL3_PLLMUL2_MASK      (0xffff << SYS2_PLLCTL3_PLLMUL2_SHIFT)
#  define SYS2_PLLCTL3_PLLMUL2(n)      ((uint32_t)(n) << \
                                         SYS2_PLLCTL3_PLLMUL2_SHIFT)

/* System Exception Control Register (software reset) */

/* Bit 15: Software reset request */
#define SYS_SYSECR_RESET (1 << 15)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_SYS_H */
