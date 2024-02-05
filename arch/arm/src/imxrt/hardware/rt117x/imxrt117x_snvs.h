/*****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_snvs.h
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
 *****************************************************************************/

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_SNVS_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_SNVS_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

/*****************************************************************************
 * Preprocessor Definitions
 *****************************************************************************/

/* SNVS Register Offsets *****************************************************/
#define IMXRT_SNVS_HPLR_OFFSET                 (0x0000)
#define IMXRT_SNVS_HPCOMR_OFFSET               (0x0004)
#define IMXRT_SNVS_HPCR_OFFSET                 (0x0008)
#define IMXRT_SNVS_HPSICR_OFFSET               (0x000c)
#define IMXRT_SNVS_HPSVCR_OFFSET               (0x0010)
#define IMXRT_SNVS_HPSR_OFFSET                 (0x0014)
#define IMXRT_SNVS_HPSVSR_OFFSET               (0x0018)
#define IMXRT_SNVS_HPHACIVR_OFFSET             (0x001c)
#define IMXRT_SNVS_HPHACR_OFFSET               (0x0020)
#define IMXRT_SNVS_HPRTCMR_OFFSET              (0x0024)
#define IMXRT_SNVS_HPRTCLR_OFFSET              (0x0028)
#define IMXRT_SNVS_HPTAMR_OFFSET               (0x002c)
#define IMXRT_SNVS_HPTALR_OFFSET               (0x0030)
#define IMXRT_SNVS_LPLR_OFFSET                 (0x0034)
#define IMXRT_SNVS_LPCR_OFFSET                 (0x0038)
#define IMXRT_SNVS_LPMKCR_OFFSET               (0x003c)
#define IMXRT_SNVS_LPSVCR_OFFSET               (0x0040)
#define IMXRT_SNVS_LPTGFCR_OFFSET              (0x0044)
#define IMXRT_SNVS_LPTDCR_OFFSET               (0x0048)
#define IMXRT_SNVS_LPSR_OFFSET                 (0x004c)
#define IMXRT_SNVS_LPSRTCMR_OFFSET             (0x0050)
#define IMXRT_SNVS_LPSRTCLR_OFFSET             (0x0054)
#define IMXRT_SNVS_LPTAR_OFFSET                (0x0058)
#define IMXRT_SNVS_LPSMCMR_OFFSET              (0x005c)
#define IMXRT_SNVS_LPSMCLR_OFFSET              (0x0060)
#define IMXRT_SNVS_LPLVDR_OFFSET               (0x0064)
#define IMXRT_SNVS_LPGPR0_LEGACY_ALIAS_OFFSET  (0x0068)
#define IMXRT_SNVS_LPTDC2R_OFFSET              (0x00a0)
#define IMXRT_SNVS_LPTDSR_OFFSET               (0x00a4)
#define IMXRT_SNVS_LPTGF1CR_OFFSET             (0x00a8)
#define IMXRT_SNVS_LPTGF2CR_OFFSET             (0x00ac)
#define IMXRT_SNVS_LPAT1CR_OFFSET              (0x00c0)
#define IMXRT_SNVS_LPAT2CR_OFFSET              (0x00c4)
#define IMXRT_SNVS_LPAT3CR_OFFSET              (0x00c8)
#define IMXRT_SNVS_LPAT4CR_OFFSET              (0x00cc)
#define IMXRT_SNVS_LPAT5CR_OFFSET              (0x00d0)
#define IMXRT_SNVS_LPATCTLR_OFFSET             (0x00e0)
#define IMXRT_SNVS_LPATCLKR_OFFSET             (0x00e4)
#define IMXRT_SNVS_LPATRC1R_OFFSET             (0x00e8)
#define IMXRT_SNVS_LPATRC2R_OFFSET             (0x00ec)
#define IMXRT_SNVS_HPVIDR1_OFFSET              (0x0bf8)
#define IMXRT_SNVS_HPVIDR2_OFFSET              (0x0bfc)
#define IMXRT_SNVS_LPZMKR0_OFFSET              (0x006c)
#define IMXRT_SNVS_LPZMKR1_OFFSET              (0x0070)
#define IMXRT_SNVS_LPZMKR2_OFFSET              (0x0074)
#define IMXRT_SNVS_LPZMKR3_OFFSET              (0x0078)
#define IMXRT_SNVS_LPZMKR4_OFFSET              (0x007c)
#define IMXRT_SNVS_LPZMKR5_OFFSET              (0x0080)
#define IMXRT_SNVS_LPZMKR6_OFFSET              (0x0084)
#define IMXRT_SNVS_LPZMKR7_OFFSET              (0x0088)
#define IMXRT_SNVS_LPGPR_ALIAS0_OFFSET         (0x0090)
#define IMXRT_SNVS_LPGPR_ALIAS1_OFFSET         (0x0094)
#define IMXRT_SNVS_LPGPR_ALIAS2_OFFSET         (0x0098)
#define IMXRT_SNVS_LPGPR_ALIAS3_OFFSET         (0x009c)
#define IMXRT_SNVS_LPGPR0_OFFSET               (0x0100)
#define IMXRT_SNVS_LPGPR1_OFFSET               (0x0104)
#define IMXRT_SNVS_LPGPR2_OFFSET               (0x0108)
#define IMXRT_SNVS_LPGPR3_OFFSET               (0x010c)

/* SNVS Register Addresses ***************************************************/
#define IMXRT_SNVS_HPLR                 (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPLR_OFFSET)
#define IMXRT_SNVS_HPCOMR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPCOMR_OFFSET)
#define IMXRT_SNVS_HPCR                 (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPCR_OFFSET)
#define IMXRT_SNVS_HPSICR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPSICR_OFFSET)
#define IMXRT_SNVS_HPSVCR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPSVCR_OFFSET)
#define IMXRT_SNVS_HPSR                 (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPSR_OFFSET)
#define IMXRT_SNVS_HPSVSR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPSVSR_OFFSET)
#define IMXRT_SNVS_HPHACIVR             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPHACIVR_OFFSET)
#define IMXRT_SNVS_HPHACR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPHACR_OFFSET)
#define IMXRT_SNVS_HPRTCMR              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPRTCMR_OFFSET)
#define IMXRT_SNVS_HPRTCLR              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPRTCLR_OFFSET)
#define IMXRT_SNVS_HPTAMR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPTAMR_OFFSET)
#define IMXRT_SNVS_HPTALR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPTALR_OFFSET)
#define IMXRT_SNVS_LPLR                 (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPLR_OFFSET)
#define IMXRT_SNVS_LPCR                 (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPCR_OFFSET)
#define IMXRT_SNVS_LPMKCR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPMKCR_OFFSET)
#define IMXRT_SNVS_LPSVCR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPSVCR_OFFSET)
#define IMXRT_SNVS_LPTGFCR              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPTGFCR_OFFSET)
#define IMXRT_SNVS_LPTDCR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPTDCR_OFFSET)
#define IMXRT_SNVS_LPSR                 (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPSR_OFFSET)
#define IMXRT_SNVS_LPSRTCMR             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPSRTCMR_OFFSET)
#define IMXRT_SNVS_LPSRTCLR             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPSRTCLR_OFFSET)
#define IMXRT_SNVS_LPTAR                (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPTAR_OFFSET)
#define IMXRT_SNVS_LPSMCMR              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPSMCMR_OFFSET)
#define IMXRT_SNVS_LPSMCLR              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPSMCLR_OFFSET)
#define IMXRT_SNVS_LPLVDR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPLVDR_OFFSET)
#define IMXRT_SNVS_LPGPR0_LEGACY_ALIAS  (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR0_LEGACY_ALIAS_OFFSET)
#define IMXRT_SNVS_LPTDC2R              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPTDC2R_OFFSET)
#define IMXRT_SNVS_LPTDSR               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPTDSR_OFFSET)
#define IMXRT_SNVS_LPTGF1CR             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPTGF1CR_OFFSET)
#define IMXRT_SNVS_LPTGF2CR             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPTGF2CR_OFFSET)
#define IMXRT_SNVS_LPAT1CR              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPAT1CR_OFFSET)
#define IMXRT_SNVS_LPAT2CR              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPAT2CR_OFFSET)
#define IMXRT_SNVS_LPAT3CR              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPAT3CR_OFFSET)
#define IMXRT_SNVS_LPAT4CR              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPAT4CR_OFFSET)
#define IMXRT_SNVS_LPAT5CR              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPAT5CR_OFFSET)
#define IMXRT_SNVS_LPATCTLR             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPATCTLR_OFFSET)
#define IMXRT_SNVS_LPATCLKR             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPATCLKR_OFFSET)
#define IMXRT_SNVS_LPATRC1R             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPATRC1R_OFFSET)
#define IMXRT_SNVS_LPATRC2R             (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPATRC2R_OFFSET)
#define IMXRT_SNVS_HPVIDR1              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPVIDR1_OFFSET)
#define IMXRT_SNVS_HPVIDR2              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_HPVIDR2_OFFSET)
#define IMXRT_SNVS_LPZMKR0              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPZMKR0_OFFSET)
#define IMXRT_SNVS_LPZMKR1              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPZMKR1_OFFSET)
#define IMXRT_SNVS_LPZMKR2              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPZMKR2_OFFSET)
#define IMXRT_SNVS_LPZMKR3              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPZMKR3_OFFSET)
#define IMXRT_SNVS_LPZMKR4              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPZMKR4_OFFSET)
#define IMXRT_SNVS_LPZMKR5              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPZMKR5_OFFSET)
#define IMXRT_SNVS_LPZMKR6              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPZMKR6_OFFSET)
#define IMXRT_SNVS_LPZMKR7              (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPZMKR7_OFFSET)
#define IMXRT_SNVS_LPGPR_ALIAS0         (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR_ALIAS0_OFFSET)
#define IMXRT_SNVS_LPGPR_ALIAS1         (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR_ALIAS1_OFFSET)
#define IMXRT_SNVS_LPGPR_ALIAS2         (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR_ALIAS2_OFFSET)
#define IMXRT_SNVS_LPGPR_ALIAS3         (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR_ALIAS3_OFFSET)
#define IMXRT_SNVS_LPGPR0               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR0_OFFSET)
#define IMXRT_SNVS_LPGPR1               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR1_OFFSET)
#define IMXRT_SNVS_LPGPR2               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR2_OFFSET)
#define IMXRT_SNVS_LPGPR3               (IMXRT_SNVSHP_BASE + IMXRT_SNVS_LPGPR3_OFFSET)

/* SNVS_HP Lock Register (HPLR) */
#define SNVS_HPLR_ZMK_WSL     (1 << 0)   /* Bit 0: Zeroizable Master Key Write Soft Lock When set, prevents any writes (software and hardware) to the ZMK registers and the ZMK_HWP, ZMK_VAL, and ZMK_ECC_EN fields of the LPMKCR */
#define SNVS_HPLR_ZMK_RSL     (1 << 1)   /* Bit 1: Zeroizable Master Key Read Soft Lock When set, prevents any software reads to the ZMK Registers and ZMK_ECC_VALUE field of the LPMKCR */
#define SNVS_HPLR_SRTC_SL     (1 << 2)   /* Bit 2: Secure Real Time Counter Soft Lock When set, prevents any writes to the SRTC Registers, SRTC_ENV, and SRTC_INV_EN bits */
#define SNVS_HPLR_LPCALB_SL   (1 << 3)   /* Bit 3: LP Calibration Soft Lock When set, prevents any writes to the LP Calibration Value (LPCALB_VAL) and LP Calibration Enable (LPCALB_EN) */
#define SNVS_HPLR_MC_SL       (1 << 4)   /* Bit 4: Monotonic Counter Soft Lock When set, prevents any writes (increments) to the MC Registers and MC_ENV bit */
#define SNVS_HPLR_GPR_SL      (1 << 5)   /* Bit 5: General Purpose Register Soft Lock When set, prevents any writes to the GPR */
#define SNVS_HPLR_LPSVCR_SL   (1 << 6)   /* Bit 6: LP Security Violation Control Register Soft Lock When set, prevents any writes to the LPSVCR */
#define SNVS_HPLR_LPTGFCR_SL  (1 << 7)   /* Bit 7: LP Tamper Glitch Filter Configuration Register Soft Lock When set, prevents any writes to the LPTGFCR */
#define SNVS_HPLR_LPSECR_SL   (1 << 8)   /* Bit 8: LP Security Events Configuration Register Soft Lock When set, prevents any writes to the LPSECR */
#define SNVS_HPLR_MKS_SL      (1 << 9)   /* Bit 9: Master Key Select Soft Lock When set, prevents any writes to the MASTER_KEY_SEL field of the LPMKCR */
#define SNVS_HPLR_HPSVCR_L    (1 << 16)  /* Bit 16: HP Security Violation Control Register Lock When set, prevents any writes to the HPSVCR */
#define SNVS_HPLR_HPSICR_L    (1 << 17)  /* Bit 17: HP Security Interrupt Control Register Lock When set, prevents any writes to the HPSICR */
#define SNVS_HPLR_HAC_L       (1 << 18)  /* Bit 18: High Assurance Counter Lock When set, prevents any writes to HPHACIVR, HPHACR, and HAC_EN bit of HPCOMR */
#define SNVS_HPLR_AT1_SL      (1 << 24)  /* Bit 24: Active Tamper 1 Soft Lock When set, prevents any writes to the Active Tamper 1 registers */
#define SNVS_HPLR_AT2_SL      (1 << 25)  /* Bit 25: Active Tamper 2 Soft Lock When set, prevents any writes to the Active Tamper 2 registers */
#define SNVS_HPLR_AT3_SL      (1 << 26)  /* Bit 26: Active Tamper 3 Soft Lock When set, prevents any writes to the Active Tamper 3 registers */
#define SNVS_HPLR_AT4_SL      (1 << 27)  /* Bit 27: Active Tamper 4 Soft Lock When set, prevents any writes to the Active Tamper 4 registers */
#define SNVS_HPLR_AT5_SL      (1 << 28)  /* Bit 28: Active Tamper 5 Soft Lock When set, prevents any writes to the Active Tamper 5 registers */

/* SNVS_HP Command Register (HPCOMR) */
#define SNVS_HPCOMR_SSM_ST        (1 << 0)   /* Bit 0: SSM State Transition Transition state of the system security monitor */
#define SNVS_HPCOMR_SSM_ST_DIS    (1 << 1)   /* Bit 1: SSM Secure to Trusted State Transition Disable When set, disables the SSM transition from secure to trusted state */
#define SNVS_HPCOMR_SSM_SFNS_DIS  (1 << 2)   /* Bit 2: SSM Soft Fail to Non-Secure State Transition Disable When set, it disables the SSM transition from soft fail to non-secure state */
#define SNVS_HPCOMR_LP_SWR        (1 << 4)   /* Bit 4: LP Software Reset When set to 1, most registers in the SNVS_LP section are reset, but the following registers are not reset by an LP software reset: Monotonic Counter Secure Real Time Counter Time Alarm Register This bit cannot be set when the LP_SWR_DIS bit is set */
#define SNVS_HPCOMR_LP_SWR_DIS    (1 << 5)   /* Bit 5: LP Software Reset Disable When set, disables the LP software reset */
#define SNVS_HPCOMR_SW_SV         (1 << 8)   /* Bit 8: Software Security Violation When set, the system security monitor treats this bit as a non-fatal security violation */
#define SNVS_HPCOMR_SW_FSV        (1 << 9)   /* Bit 9: Software Fatal Security Violation When set, the system security monitor treats this bit as a fatal security violation */
#define SNVS_HPCOMR_SW_LPSV       (1 << 10)  /* Bit 10: LP Software Security Violation When set, SNVS_LP treats this bit as a security violation */
#define SNVS_HPCOMR_PROG_ZMK      (1 << 12)  /* Bit 12: Program Zeroizable Master Key This bit activates ZMK hardware programming mechanism */
#define SNVS_HPCOMR_MKS_EN        (1 << 13)  /* Bit 13: Master Key Select Enable When not set, the one time programmable (OTP) master key is selected by default */
#define SNVS_HPCOMR_HAC_EN        (1 << 16)  /* Bit 16: High Assurance Counter Enable This bit controls the SSM transition from the soft fail to the hard fail state */
#define SNVS_HPCOMR_HAC_LOAD      (1 << 17)  /* Bit 17: High Assurance Counter Load When set, it loads the High Assurance Counter Register with the value of the High Assurance Counter Load Register */
#define SNVS_HPCOMR_HAC_CLEAR     (1 << 18)  /* Bit 18: High Assurance Counter Clear When set, it clears the High Assurance Counter Register */
#define SNVS_HPCOMR_HAC_STOP      (1 << 19)  /* Bit 19: High Assurance Counter Stop This bit can be set only when SSM is in soft fail state */
#define SNVS_HPCOMR_NPSWA_EN      (1 << 31)  /* Bit 31: Non-Privileged Software Access Enable When set, allows non-privileged software to access all SNVS registers, including those that are privileged software read/write access only */

/* SNVS_HP Control Register (HPCR) */
#define SNVS_HPCR_RTC_EN            (1 << 0)   /* Bit 0: HP Real Time Counter Enable */
#define SNVS_HPCR_HPTA_EN           (1 << 1)   /* Bit 1: HP Time Alarm Enable When set, the time alarm interrupt is generated if the value in the HP Time Alarm Registers is equal to the value of the HP Real Time Counter */
#define SNVS_HPCR_DIS_PI            (1 << 2)   /* Bit 2: Disable periodic interrupt in the functional interrupt */
#define SNVS_HPCR_PI_EN             (1 << 3)   /* Bit 3: HP Periodic Interrupt Enable The periodic interrupt can be generated only if the HP Real Time Counter is enabled */
#define SNVS_HPCR_PI_FREQ_SHIFT     (4)        /* Bits 4-8: Periodic Interrupt Frequency Defines frequency of the periodic interrupt */
#define SNVS_HPCR_PI_FREQ_MASK      (0xF << SNVS_HPCR_PI_FREQ_SHIFT)
#define SNVS_HPCR_PI_FREQ(n)        (((n) << SNVS_HPCR_PI_FREQ_SHIFT) & SNVS_HPCR_PI_FREQ_MASK)
#define SNVS_HPCR_HPCALB_EN         (1 << 8)   /* Bit 8: HP Real Time Counter Calibration Enabled Indicates that the time calibration mechanism is enabled. */
#define SNVS_HPCR_HPCALB_VAL_SHIFT  (10)       /* Bits 10-15: HP Calibration Value Defines signed calibration value for the HP Real Time Counter */
#define SNVS_HPCR_HPCALB_VAL_MASK   (0x1F << SNVS_HPCR_HPCALB_VAL_SHIFT)
#define SNVS_HPCR_HPCALB_VAL(n)     (((n) << SNVS_HPCR_HPCALB_VAL_SHIFT) & SNVS_HPCR_HPCALB_VAL_MASK)
#define SNVS_HPCR_HP_TS             (1 << 16)  /* Bit 16: HP Time Synchronize */
#define SNVS_HPCR_BTN_CONFIG_SHIFT  (24)       /* Bits 24-27: Button Configuration */
#define SNVS_HPCR_BTN_CONFIG_MASK   (0x7 << SNVS_HPCR_BTN_CONFIG_SHIFT)
#define SNVS_HPCR_BTN_CONFIG(n)     (((n) << SNVS_HPCR_BTN_CONFIG_SHIFT) & SNVS_HPCR_BTN_CONFIG_MASK)
#define SNVS_HPCR_BTN_MASK          (1 << 27)  /* Bit 27: Button interrupt mask */

/* SNVS_HP Security Interrupt Control Register (HPSICR) */
#define SNVS_HPSICR_CAAM_EN   (1 << 0)   /* Bit 0: CAAM Security Violation Interrupt Enable Setting this bit to 1 enables generation of the security interrupt to the host processor upon detection of the CAAM Security Violation security violation */
#define SNVS_HPSICR_JTAGC_EN  (1 << 1)   /* Bit 1: JTAG Active Interrupt Enable Setting this bit to 1 enables generation of the security interrupt to the host processor upon detection of the JTAG Active security violation */
#define SNVS_HPSICR_WDOG2_EN  (1 << 2)   /* Bit 2: Watchdog 2 Reset Interrupt Enable Setting this bit to 1 enables generation of the security interrupt to the host processor upon detection of the Watchdog 2 Reset security violation */
#define SNVS_HPSICR_SRC_EN    (1 << 4)   /* Bit 4: Internal Boot Interrupt Enable Setting this bit to 1 enables generation of the security interrupt to the host processor upon detection of the Internal Boot security violation */
#define SNVS_HPSICR_OCOTP_EN  (1 << 5)   /* Bit 5: OCOTP attack error Interrupt Enable Setting this bit to 1 enables generation of the security interrupt to the host processor upon detection of the OCOTP attack error security violation */
#define SNVS_HPSICR_LPSVI_EN  (1 << 31)  /* Bit 31: LP Security Violation Interrupt Enable This bit enables generating of the security interrupt to the host processor upon security violation signal from the LP section */

/* SNVS_HP Security Violation Control Register (HPSVCR) */
#define SNVS_HPSVCR_CAAM_CFG         (1 << 0)  /* Bit 0: CAAM Security Violation Security Violation Configuration This field configures the CAAM Security Violation Security Violation Input */
#define SNVS_HPSVCR_JTAGC_CFG        (1 << 1)  /* Bit 1: JTAG Active Security Violation Configuration This field configures the JTAG Active Security Violation Input */
#define SNVS_HPSVCR_WDOG2_CFG        (1 << 2)  /* Bit 2: Watchdog 2 Reset Security Violation Configuration This field configures the Watchdog 2 Reset Security Violation Input */
#define SNVS_HPSVCR_SRC_CFG          (1 << 4)  /* Bit 4: Internal Boot Security Violation Configuration This field configures the Internal Boot Security Violation Input */
#define SNVS_HPSVCR_OCOTP_CFG_SHIFT  (5)       /* Bits 5-7: OCOTP attack error Security Violation Configuration This field configures the OCOTP attack error Security Violation Input */
#define SNVS_HPSVCR_OCOTP_CFG_MASK   (0x3 << SNVS_HPSVCR_OCOTP_CFG_SHIFT)
#define SNVS_HPSVCR_OCOTP_CFG(n)     (((n) << SNVS_HPSVCR_OCOTP_CFG_SHIFT) & SNVS_HPSVCR_OCOTP_CFG_MASK)
#define SNVS_HPSVCR_LPSV_CFG_SHIFT   (30)      /* Bits 30-32: LP Security Violation Configuration This field configures the LP security violation source. */
#define SNVS_HPSVCR_LPSV_CFG_MASK    (0x3 << SNVS_HPSVCR_LPSV_CFG_SHIFT)
#define SNVS_HPSVCR_LPSV_CFG(n)      (((n) << SNVS_HPSVCR_LPSV_CFG_SHIFT) & SNVS_HPSVCR_LPSV_CFG_MASK)

/* SNVS_HP Status Register (HPSR) */
#define SNVS_HPSR_HPTA                    (1 << 0)   /* Bit 0: HP Time Alarm Indicates that the HP Time Alarm has occurred since this bit was last cleared. */
#define SNVS_HPSR_PI                      (1 << 1)   /* Bit 1: Periodic Interrupt Indicates that periodic interrupt has occurred since this bit was last cleared. */
#define SNVS_HPSR_LPDIS                   (1 << 4)   /* Bit 4: Low Power Disable If 1, the low power section has been disabled by means of an input signal to SNVS */
#define SNVS_HPSR_BTN                     (1 << 6)   /* Bit 6: Button Value of the BTN input */
#define SNVS_HPSR_BI                      (1 << 7)   /* Bit 7: Button Interrupt Signal ipi_snvs_btn_int_b was asserted. */
#define SNVS_HPSR_SSM_STATE_SHIFT         (8)        /* Bits 8-12: System Security Monitor State This field contains the encoded state of the SSM's state machine */
#define SNVS_HPSR_SSM_STATE_MASK          (0xF << SNVS_HPSR_SSM_STATE_SHIFT)
#define SNVS_HPSR_SSM_STATE(n)            (((n) << SNVS_HPSR_SSM_STATE_SHIFT) & SNVS_HPSR_SSM_STATE_MASK)
#define SNVS_HPSR_SYS_SECURITY_CFG_SHIFT  (12)       /* Bits 12-15: System Security Configuration This field reflects the three security configuration inputs to SNVS */
#define SNVS_HPSR_SYS_SECURITY_CFG_MASK   (0x7 << SNVS_HPSR_SYS_SECURITY_CFG_SHIFT)
#define SNVS_HPSR_SYS_SECURITY_CFG(n)     (((n) << SNVS_HPSR_SYS_SECURITY_CFG_SHIFT) & SNVS_HPSR_SYS_SECURITY_CFG_MASK)
#define SNVS_HPSR_SYS_SECURE_BOOT         (1 << 15)  /* Bit 15: System Secure Boot If SYS_SECURE_BOOT is 1, the chip boots from internal ROM */
#define SNVS_HPSR_OTPMK_ZERO              (1 << 27)  /* Bit 27: One Time Programmable Master Key is Equal to Zero */
#define SNVS_HPSR_ZMK_ZERO                (1 << 31)  /* Bit 31: Zeroizable Master Key is Equal to Zero */

/* SNVS_HP Security Violation Status Register (HPSVSR) */
#define SNVS_HPSVSR_CAAM                (1 << 0)   /* Bit 0: CAAM Security Violation security violation was detected. */
#define SNVS_HPSVSR_JTAGC               (1 << 1)   /* Bit 1: JTAG Active security violation was detected. */
#define SNVS_HPSVSR_WDOG2               (1 << 2)   /* Bit 2: Watchdog 2 Reset security violation was detected. */
#define SNVS_HPSVSR_SRC                 (1 << 4)   /* Bit 4: Internal Boot security violation was detected. */
#define SNVS_HPSVSR_OCOTP               (1 << 5)   /* Bit 5: OCOTP attack error security violation was detected. */
#define SNVS_HPSVSR_SW_SV               (1 << 13)  /* Bit 13: Software Security Violation This bit is a read-only copy of the SW_SV bit in the HP Command Register */
#define SNVS_HPSVSR_SW_FSV              (1 << 14)  /* Bit 14: Software Fatal Security Violation This bit is a read-only copy of the SW_FSV bit in the HP Command Register */
#define SNVS_HPSVSR_SW_LPSV             (1 << 15)  /* Bit 15: LP Software Security Violation This bit is a read-only copy of the SW_LPSV bit in the HP Command Register */
#define SNVS_HPSVSR_ZMK_SYNDROME_SHIFT  (16)       /* Bits 16-25: Zeroizable Master Key Syndrome The ZMK syndrome indicates the single-bit error location and parity for the ZMK register */
#define SNVS_HPSVSR_ZMK_SYNDROME_MASK   (0x1FF << SNVS_HPSVSR_ZMK_SYNDROME_SHIFT)
#define SNVS_HPSVSR_ZMK_SYNDROME(n)     (((n) << SNVS_HPSVSR_ZMK_SYNDROME_SHIFT) & SNVS_HPSVSR_ZMK_SYNDROME_MASK)
#define SNVS_HPSVSR_ZMK_ECC_FAIL        (1 << 27)  /* Bit 27: Zeroizable Master Key Error Correcting Code Check Failure When set, this bit triggers a bad key violation to the SSM and a security violation to the SNVS_LP section, which clears security sensitive data */
#define SNVS_HPSVSR_LP_SEC_VIO          (1 << 31)  /* Bit 31: LP Security Violation A security volation was detected in the SNVS low power section */

/* SNVS_HP High Assurance Counter IV Register (HPHACIVR) */
#define SNVS_HPHACIVR_HAC_COUNTER_IV_SHIFT  (0)  /* Bits 0-32: High Assurance Counter Initial Value This register is used to set the starting count value to the high assurance counter */
#define SNVS_HPHACIVR_HAC_COUNTER_IV_MASK   (0xFFFFFFFF << SNVS_HPHACIVR_HAC_COUNTER_IV_SHIFT)
#define SNVS_HPHACIVR_HAC_COUNTER_IV(n)     (((n) << SNVS_HPHACIVR_HAC_COUNTER_IV_SHIFT) & SNVS_HPHACIVR_HAC_COUNTER_IV_MASK)

/* SNVS_HP High Assurance Counter Register (HPHACR) */
#define SNVS_HPHACR_HAC_COUNTER_SHIFT  (0)  /* Bits 0-32: High Assurance Counter When the HAC_EN bit is set and the SSM is in the soft fail state, this counter starts to count down with the system clock */
#define SNVS_HPHACR_HAC_COUNTER_MASK   (0xFFFFFFFF << SNVS_HPHACR_HAC_COUNTER_SHIFT)
#define SNVS_HPHACR_HAC_COUNTER(n)     (((n) << SNVS_HPHACR_HAC_COUNTER_SHIFT) & SNVS_HPHACR_HAC_COUNTER_MASK)

/* SNVS_HP Real Time Counter MSB Register (HPRTCMR) */
#define SNVS_HPRTCMR_RTC_SHIFT  (0)  /* Bits 0-15: HP Real Time Counter The most-significant 15 bits of the RTC */
#define SNVS_HPRTCMR_RTC_MASK   (0x7FFF << SNVS_HPRTCMR_RTC_SHIFT)
#define SNVS_HPRTCMR_RTC(n)     (((n) << SNVS_HPRTCMR_RTC_SHIFT) & SNVS_HPRTCMR_RTC_MASK)

/* SNVS_HP Real Time Counter LSB Register (HPRTCLR) */
#define SNVS_HPRTCLR_RTC_SHIFT  (0)  /* Bits 0-32: HP Real Time Counter least-significant 32 bits */
#define SNVS_HPRTCLR_RTC_MASK   (0xFFFFFFFF << SNVS_HPRTCLR_RTC_SHIFT)
#define SNVS_HPRTCLR_RTC(n)     (((n) << SNVS_HPRTCLR_RTC_SHIFT) & SNVS_HPRTCLR_RTC_MASK)

/* SNVS_HP Time Alarm MSB Register (HPTAMR) */
#define SNVS_HPTAMR_HPTA_MS_SHIFT  (0)  /* Bits 0-15: HP Time Alarm, most-significant 15 bits */
#define SNVS_HPTAMR_HPTA_MS_MASK   (0x7FFF << SNVS_HPTAMR_HPTA_MS_SHIFT)
#define SNVS_HPTAMR_HPTA_MS(n)     (((n) << SNVS_HPTAMR_HPTA_MS_SHIFT) & SNVS_HPTAMR_HPTA_MS_MASK)

/* SNVS_HP Time Alarm LSB Register (HPTALR) */
#define SNVS_HPTALR_HPTA_LS_SHIFT  (0)  /* Bits 0-32: HP Time Alarm, 32 least-significant bits */
#define SNVS_HPTALR_HPTA_LS_MASK   (0xFFFFFFFF << SNVS_HPTALR_HPTA_LS_SHIFT)
#define SNVS_HPTALR_HPTA_LS(n)     (((n) << SNVS_HPTALR_HPTA_LS_SHIFT) & SNVS_HPTALR_HPTA_LS_MASK)

/* SNVS_LP Lock Register (LPLR) */
#define SNVS_LPLR_ZMK_WHL     (1 << 0)   /* Bit 0: Zeroizable Master Key Write Hard Lock When set, prevents any writes (software and hardware) to the ZMK registers and ZMK_HWP, ZMK_VAL, and ZMK_ECC_EN fields of the LPMKCR */
#define SNVS_LPLR_ZMK_RHL     (1 << 1)   /* Bit 1: Zeroizable Master Key Read Hard Lock When set, prevents any software reads to the ZMK registers and ZMK_ECC_VALUE field of the LPMKCR */
#define SNVS_LPLR_SRTC_HL     (1 << 2)   /* Bit 2: Secure Real Time Counter Hard Lock When set, prevents any writes to the SRTC registers, SRTC_ENV, and SRTC_INV_EN bits */
#define SNVS_LPLR_LPCALB_HL   (1 << 3)   /* Bit 3: LP Calibration Hard Lock When set, prevents any writes to the LP Calibration Value (LPCALB_VAL) and LP Calibration Enable (LPCALB_EN) */
#define SNVS_LPLR_MC_HL       (1 << 4)   /* Bit 4: Monotonic Counter Hard Lock When set, prevents any writes (increments) to the MC Registers and MC_ENV bit */
#define SNVS_LPLR_GPR_HL      (1 << 5)   /* Bit 5: General Purpose Register Hard Lock When set, prevents any writes to the GPR */
#define SNVS_LPLR_LPSVCR_HL   (1 << 6)   /* Bit 6: LP Security Violation Control Register Hard Lock When set, prevents any writes to the LPSVCR */
#define SNVS_LPLR_LPTGFCR_HL  (1 << 7)   /* Bit 7: LP Tamper Glitch Filter Configuration Register Hard Lock When set, prevents any writes to the LPTGFCR */
#define SNVS_LPLR_LPSECR_HL   (1 << 8)   /* Bit 8: LP Security Events Configuration Register Hard Lock When set, prevents any writes to the LPSECR */
#define SNVS_LPLR_MKS_HL      (1 << 9)   /* Bit 9: Master Key Select Hard Lock When set, prevents any writes to the MASTER_KEY_SEL field of the LP Master Key Control Register */
#define SNVS_LPLR_AT1_HL      (1 << 24)  /* Bit 24: Active Tamper 1 Hard Lock When set, prevents any writes to the Active Tamper 1 registers */
#define SNVS_LPLR_AT2_HL      (1 << 25)  /* Bit 25: Active Tamper 2 Hard Lock When set, prevents any writes to the Active Tamper 2 registers */
#define SNVS_LPLR_AT3_HL      (1 << 26)  /* Bit 26: Active Tamper 3 Hard Lock When set, prevents any writes to the Active Tamper 3 registers */
#define SNVS_LPLR_AT4_HL      (1 << 27)  /* Bit 27: Active Tamper 4 Hard Lock When set, prevents any writes to the Active Tamper 4 registers */
#define SNVS_LPLR_AT5_HL      (1 << 28)  /* Bit 28: Active Tamper 5 Hard Lock When set, prevents any writes to the Active Tamper 5 registers */

/* SNVS_LP Control Register (LPCR) */
#define SNVS_LPCR_SRTC_ENV              (1 << 0)   /* Bit 0: Secure Real Time Counter Enabled and Valid When set, the SRTC becomes operational */
#define SNVS_LPCR_LPTA_EN               (1 << 1)   /* Bit 1: LP Time Alarm Enable When set, the SNVS functional interrupt is asserted if the LP Time Alarm Register is equal to the 32 MSBs of the secure real time counter */
#define SNVS_LPCR_MC_ENV                (1 << 2)   /* Bit 2: Monotonic Counter Enabled and Valid When set, the MC can be incremented (by write transaction to the LPSMCMR or LPSMCLR) */
#define SNVS_LPCR_LPWUI_EN              (1 << 3)   /* Bit 3: LP Wake-Up Interrupt Enable This interrupt line should be connected to the external pin and is intended to inform the external chip about an SNVS_LP event (tamper event, MC rollover, SRTC rollover, or time alarm ) */
#define SNVS_LPCR_SRTC_INV_EN           (1 << 4)   /* Bit 4: If this bit is 1, in the case of a security violation the SRTC stops counting and the SRTC is invalidated (SRTC_ENV bit is cleared) */
#define SNVS_LPCR_DP_EN                 (1 << 5)   /* Bit 5: Dumb PMIC Enabled When set, software can control the system power */
#define SNVS_LPCR_TOP                   (1 << 6)   /* Bit 6: Turn off System Power Asserting this bit causes a signal to be sent to the Power Management IC to turn off the system power */
#define SNVS_LPCR_LVD_EN                (1 << 7)   /* Bit 7: Digital Low-Voltage Event Enable By default the detection of a low-voltage event does not cause the pmic_en_b signal to be asserted */
#define SNVS_LPCR_LPCALB_EN             (1 << 8)   /* Bit 8: LP Calibration Enable When set, enables the SRTC calibration mechanism */
#define SNVS_LPCR_LPCALB_VAL_SHIFT      (10)       /* Bits 10-15: LP Calibration Value Defines signed calibration value for SRTC */
#define SNVS_LPCR_LPCALB_VAL_MASK       (0x1F << SNVS_LPCR_LPCALB_VAL_SHIFT)
#define SNVS_LPCR_LPCALB_VAL(n)         (((n) << SNVS_LPCR_LPCALB_VAL_SHIFT) & SNVS_LPCR_LPCALB_VAL_MASK)
#define SNVS_LPCR_BTN_PRESS_TIME_SHIFT  (16)       /* Bits 16-18: This field configures the button press time out values for the PMIC Logic */
#define SNVS_LPCR_BTN_PRESS_TIME_MASK   (0x3 << SNVS_LPCR_BTN_PRESS_TIME_SHIFT)
#define SNVS_LPCR_BTN_PRESS_TIME(n)     (((n) << SNVS_LPCR_BTN_PRESS_TIME_SHIFT) & SNVS_LPCR_BTN_PRESS_TIME_MASK)
#define SNVS_LPCR_DEBOUNCE_SHIFT        (18)       /* Bits 18-20: This field configures the amount of debounce time for the BTN input signal */
#define SNVS_LPCR_DEBOUNCE_MASK         (0x3 << SNVS_LPCR_DEBOUNCE_SHIFT)
#define SNVS_LPCR_DEBOUNCE(n)           (((n) << SNVS_LPCR_DEBOUNCE_SHIFT) & SNVS_LPCR_DEBOUNCE_MASK)
#define SNVS_LPCR_ON_TIME_SHIFT         (20)       /* Bits 20-22: The ON_TIME field is used to configure the period of time after BTN is asserted before pmic_en_b is asserted to turn on the SoC power */
#define SNVS_LPCR_ON_TIME_MASK          (0x3 << SNVS_LPCR_ON_TIME_SHIFT)
#define SNVS_LPCR_ON_TIME(n)            (((n) << SNVS_LPCR_ON_TIME_SHIFT) & SNVS_LPCR_ON_TIME_MASK)
#define SNVS_LPCR_PK_EN                 (1 << 22)  /* Bit 22: PMIC On Request Enable The value written to PK_EN will be asserted on output signal snvs_lp_pk_en */
#define SNVS_LPCR_PK_OVERRIDE           (1 << 23)  /* Bit 23: PMIC On Request Override The value written to PK_OVERRIDE will be asserted on output signal snvs_lp_pk_override */
#define SNVS_LPCR_GPR_Z_DIS             (1 << 24)  /* Bit 24: General Purpose Registers Zeroization Disable */

/* SNVS_LP Master Key Control Register (LPMKCR) */
#define SNVS_LPMKCR_MASTER_KEY_SEL_SHIFT  (0)       /* Bits 0-2: Master Key Select These bits select the SNVS Master Key output when Master Key Select bits are enabled by MKS_EN bit in the HPCOMR */
#define SNVS_LPMKCR_MASTER_KEY_SEL_MASK   (0x3 << SNVS_LPMKCR_MASTER_KEY_SEL_SHIFT)
#define SNVS_LPMKCR_MASTER_KEY_SEL(n)     (((n) << SNVS_LPMKCR_MASTER_KEY_SEL_SHIFT) & SNVS_LPMKCR_MASTER_KEY_SEL_MASK)
#define SNVS_LPMKCR_ZMK_HWP               (1 << 2)  /* Bit 2: Zeroizable Master Key hardware Programming mode When set, only the hardware key programming mechanism can set the ZMK and software cannot read it */
#define SNVS_LPMKCR_ZMK_VAL               (1 << 3)  /* Bit 3: Zeroizable Master Key Valid When set, the ZMK value can be selected by the master key control block for use by cryptographic modules */
#define SNVS_LPMKCR_ZMK_ECC_EN            (1 << 4)  /* Bit 4: Zeroizable Master Key Error Correcting Code Check Enable Writing one to this field automatically calculates and sets the ZMK ECC value in the ZMK_ECC_VALUE field of this register */
#define SNVS_LPMKCR_ZMK_ECC_VALUE_SHIFT   (7)       /* Bits 7-16: Zeroizable Master Key Error Correcting Code Value This field is automatically calculated and set when one is written into ZMK_ECC_EN bit of this register */
#define SNVS_LPMKCR_ZMK_ECC_VALUE_MASK    (0x1FF << SNVS_LPMKCR_ZMK_ECC_VALUE_SHIFT)
#define SNVS_LPMKCR_ZMK_ECC_VALUE(n)      (((n) << SNVS_LPMKCR_ZMK_ECC_VALUE_SHIFT) & SNVS_LPMKCR_ZMK_ECC_VALUE_MASK)

/* SNVS_LP Security Violation Control Register (LPSVCR) */
#define SNVS_LPSVCR_CAAM_EN   (1 << 0)  /* Bit 0: CAAM Security Violation Enable This bit enables CAAM Security Violation Input */
#define SNVS_LPSVCR_JTAGC_EN  (1 << 1)  /* Bit 1: JTAG Active Enable This bit enables JTAG Active Input */
#define SNVS_LPSVCR_WDOG2_EN  (1 << 2)  /* Bit 2: Watchdog 2 Reset Enable This bit enables Watchdog 2 Reset Input */
#define SNVS_LPSVCR_SRC_EN    (1 << 4)  /* Bit 4: Internal Boot Enable This bit enables Internal Boot Input */
#define SNVS_LPSVCR_OCOTP_EN  (1 << 5)  /* Bit 5: OCOTP attack error Enable This bit enables OCOTP attack error Input */

/* SNVS_LP Tamper Glitch Filters Configuration Register (LPTGFCR) */
#define SNVS_LPTGFCR_WMTGF_SHIFT  (0)        /* Bits 0-5: Wire-Mesh Tamper Glitch Filter Configures the length of the digital glitch filter for the wire-mesh tamper 1 and 2 pins between 1 and 63 SRTC clock cycles */
#define SNVS_LPTGFCR_WMTGF_MASK   (0x1F << SNVS_LPTGFCR_WMTGF_SHIFT)
#define SNVS_LPTGFCR_WMTGF(n)     (((n) << SNVS_LPTGFCR_WMTGF_SHIFT) & SNVS_LPTGFCR_WMTGF_MASK)
#define SNVS_LPTGFCR_WMTGF_EN     (1 << 7)   /* Bit 7: Wire-Mesh Tamper Glitch Filter Enable When set, enables the wire-mesh tamper glitch filter */
#define SNVS_LPTGFCR_ETGF1_SHIFT  (16)       /* Bits 16-23: External Tamper Glitch Filter 1 Configures the length of the digital glitch filter for the external tamper 1 pin between 128 and 32640 SRTC clock cycles */
#define SNVS_LPTGFCR_ETGF1_MASK   (0x7F << SNVS_LPTGFCR_ETGF1_SHIFT)
#define SNVS_LPTGFCR_ETGF1(n)     (((n) << SNVS_LPTGFCR_ETGF1_SHIFT) & SNVS_LPTGFCR_ETGF1_MASK)
#define SNVS_LPTGFCR_ETGF1_EN     (1 << 23)  /* Bit 23: External Tamper Glitch Filter 1 Enable When set, enables the external tamper glitch filter 1. */
#define SNVS_LPTGFCR_ETGF2_SHIFT  (24)       /* Bits 24-31: External Tamper Glitch Filter 2 Configures the length of the digital glitch filter for the external tamper 2 pin between 128 and 32640 SRTC clock cycles */
#define SNVS_LPTGFCR_ETGF2_MASK   (0x7F << SNVS_LPTGFCR_ETGF2_SHIFT)
#define SNVS_LPTGFCR_ETGF2(n)     (((n) << SNVS_LPTGFCR_ETGF2_SHIFT) & SNVS_LPTGFCR_ETGF2_MASK)
#define SNVS_LPTGFCR_ETGF2_EN     (1 << 31)  /* Bit 31: External Tamper Glitch Filter 2 Enable When set, enables the external tamper glitch filter 2. */

/* SNVS_LP Tamper Detect Configuration Register (LPTDCR) */
#define SNVS_LPTDCR_SRTCR_EN    (1 << 1)   /* Bit 1: SRTC Rollover Enable When set, an SRTC rollover event generates an LP security violation. */
#define SNVS_LPTDCR_MCR_EN      (1 << 2)   /* Bit 2: MC Rollover Enable When set, an MC Rollover event generates an LP security violation. */
#define SNVS_LPTDCR_CT_EN       (1 << 4)   /* Bit 4: Clock Tamper Enable When set, a clock monitor tamper generates an LP security violation. */
#define SNVS_LPTDCR_TT_EN       (1 << 5)   /* Bit 5: Temperature Tamper Enable When set, a temperature monitor tamper generates an LP security violation */
#define SNVS_LPTDCR_VT_EN       (1 << 6)   /* Bit 6: Voltage Tamper Enable Voltage Tamper Enable should be enabled 500 us after setting SCSC_SOSC_CTR [VOLT_TEMP_TAMPER_EN] */
#define SNVS_LPTDCR_WMT1_EN     (1 << 7)   /* Bit 7: Wire-Mesh Tampering 1 Enable When set, wire-mesh tampering 1 detection generates an LP security violation */
#define SNVS_LPTDCR_WMT2_EN     (1 << 8)   /* Bit 8: Wire-Mesh Tampering 2 Enable When set, wire-mesh tampering 2 detection generates an LP security violation */
#define SNVS_LPTDCR_ET1_EN      (1 << 9)   /* Bit 9: External Tampering 1 Enable When set, external tampering 1 detection generates an LP security violation */
#define SNVS_LPTDCR_ET2_EN      (1 << 10)  /* Bit 10: External Tampering 2 Enable When set, external tampering 2 detection generates an LP security violation */
#define SNVS_LPTDCR_ET1P        (1 << 11)  /* Bit 11: External Tampering 1 Polarity This bit is used to determine the polarity of external tamper 1. */
#define SNVS_LPTDCR_ET2P        (1 << 12)  /* Bit 12: External Tampering 2 Polarity This bit is used to determine the polarity of external tamper 2. */
#define SNVS_LPTDCR_PFD_OBSERV  (1 << 14)  /* Bit 14: System Power Fail Detector (PFD) Observability Flop The asynchronous reset input of this flop is connected directly to the inverted output of the PFD analog circuitry (external to the SNVS block) */
#define SNVS_LPTDCR_POR_OBSERV  (1 << 15)  /* Bit 15: Power On Reset (POR) Observability Flop The asynchronous reset input of this flop is connected directly to the output of the POR analog circuitry (external to the SNVS */
#define SNVS_LPTDCR_LTDC_SHIFT  (16)       /* Bits 16-19: Low Temp Detect Configuration These configuration bits are wired as an output of the module. */
#define SNVS_LPTDCR_LTDC_MASK   (0x7 << SNVS_LPTDCR_LTDC_SHIFT)
#define SNVS_LPTDCR_LTDC(n)     (((n) << SNVS_LPTDCR_LTDC_SHIFT) & SNVS_LPTDCR_LTDC_MASK)
#define SNVS_LPTDCR_HTDC_SHIFT  (20)       /* Bits 20-23: High Temperature Detect Configuration These configuration bits are wired as an output of the module */
#define SNVS_LPTDCR_HTDC_MASK   (0x7 << SNVS_LPTDCR_HTDC_SHIFT)
#define SNVS_LPTDCR_HTDC(n)     (((n) << SNVS_LPTDCR_HTDC_SHIFT) & SNVS_LPTDCR_HTDC_MASK)
#define SNVS_LPTDCR_VRC_SHIFT   (24)       /* Bits 24-27: Voltage Reference Configuration These configuration bits are wired as an output of the module. */
#define SNVS_LPTDCR_VRC_MASK    (0x7 << SNVS_LPTDCR_VRC_SHIFT)
#define SNVS_LPTDCR_VRC(n)      (((n) << SNVS_LPTDCR_VRC_SHIFT) & SNVS_LPTDCR_VRC_MASK)
#define SNVS_LPTDCR_OSCB        (1 << 28)  /* Bit 28: Oscillator Bypass When OSCB=1 the osc_bypass signal is asserted */

/* SNVS_LP Status Register (LPSR) */
#define SNVS_LPSR_LPTA   (1 << 0)   /* Bit 0: LP Time Alarm */
#define SNVS_LPSR_SRTCR  (1 << 1)   /* Bit 1: Secure Real Time Counter Rollover */
#define SNVS_LPSR_MCR    (1 << 2)   /* Bit 2: Monotonic Counter Rollover */
#define SNVS_LPSR_LVD    (1 << 3)   /* Bit 3: Digital Low Voltage Event Detected */
#define SNVS_LPSR_CTD    (1 << 4)   /* Bit 4: Clock Tampering Detected */
#define SNVS_LPSR_TTD    (1 << 5)   /* Bit 5: Temperature Tamper Detected */
#define SNVS_LPSR_VTD    (1 << 6)   /* Bit 6: Voltage Tampering Detected */
#define SNVS_LPSR_WMT1D  (1 << 7)   /* Bit 7: Wire-Mesh Tampering 1 Detected */
#define SNVS_LPSR_WMT2D  (1 << 8)   /* Bit 8: Wire-Mesh Tampering 2 Detected */
#define SNVS_LPSR_ET1D   (1 << 9)   /* Bit 9: External Tampering 1 Detected */
#define SNVS_LPSR_ET2D   (1 << 10)  /* Bit 10: External Tampering 2 Detected */
#define SNVS_LPSR_ESVD   (1 << 16)  /* Bit 16: External Security Violation Detected Indicates that a security violation is detected on one of the HP security violation ports */
#define SNVS_LPSR_EO     (1 << 17)  /* Bit 17: Emergency Off This bit is set when a power off is requested. */
#define SNVS_LPSR_SPOF   (1 << 18)  /* Bit 18: Set Power Off The SPO bit is set when the power button is pressed longer than the configured debounce time */
#define SNVS_LPSR_LPNS   (1 << 30)  /* Bit 30: LP Section is Non-Secured Indicates that LP section was provisioned/programmed in the non-secure state */
#define SNVS_LPSR_LPS    (1 << 31)  /* Bit 31: LP Section is Secured Indicates that the LP section is provisioned/programmed in the secure or trusted state */

/* SNVS_LP Secure Real Time Counter MSB Register (LPSRTCMR) */
#define SNVS_LPSRTCMR_SRTC_SHIFT  (0)  /* Bits 0-15: LP Secure Real Time Counter The most-significant 15 bits of the SRTC */
#define SNVS_LPSRTCMR_SRTC_MASK   (0x7FFF << SNVS_LPSRTCMR_SRTC_SHIFT)
#define SNVS_LPSRTCMR_SRTC(n)     (((n) << SNVS_LPSRTCMR_SRTC_SHIFT) & SNVS_LPSRTCMR_SRTC_MASK)

/* SNVS_LP Secure Real Time Counter LSB Register (LPSRTCLR) */
#define SNVS_LPSRTCLR_SRTC_SHIFT  (0)  /* Bits 0-32: LP Secure Real Time Counter least-significant 32 bits This register can be programmed only when SRTC is not active and not locked, meaning the SRTC_ENV, SRTC_SL, and SRTC_HL bits are not set */
#define SNVS_LPSRTCLR_SRTC_MASK   (0xFFFFFFFF << SNVS_LPSRTCLR_SRTC_SHIFT)
#define SNVS_LPSRTCLR_SRTC(n)     (((n) << SNVS_LPSRTCLR_SRTC_SHIFT) & SNVS_LPSRTCLR_SRTC_MASK)

/* SNVS_LP Time Alarm Register (LPTAR) */
#define SNVS_LPTAR_LPTA_SHIFT  (0)  /* Bits 0-32: LP Time Alarm This register can be programmed only when the LP time alarm is disabled (LPTA_EN bit is not set) */
#define SNVS_LPTAR_LPTA_MASK   (0xFFFFFFFF << SNVS_LPTAR_LPTA_SHIFT)
#define SNVS_LPTAR_LPTA(n)     (((n) << SNVS_LPTAR_LPTA_SHIFT) & SNVS_LPTAR_LPTA_MASK)

/* SNVS_LP Secure Monotonic Counter MSB Register (LPSMCMR) */
#define SNVS_LPSMCMR_MON_COUNTER_SHIFT  (0)   /* Bits 0-16: Monotonic Counter most-significant 16 Bits Note that writing to this register does not change the value of this field to the value that was written */
#define SNVS_LPSMCMR_MON_COUNTER_MASK   (0xFFFF << SNVS_LPSMCMR_MON_COUNTER_SHIFT)
#define SNVS_LPSMCMR_MON_COUNTER(n)     (((n) << SNVS_LPSMCMR_MON_COUNTER_SHIFT) & SNVS_LPSMCMR_MON_COUNTER_MASK)
#define SNVS_LPSMCMR_MC_ERA_BITS_SHIFT  (16)  /* Bits 16-32: Monotonic Counter Era Bits These bits are inputs to the module and typically connect to fuses */
#define SNVS_LPSMCMR_MC_ERA_BITS_MASK   (0xFFFF << SNVS_LPSMCMR_MC_ERA_BITS_SHIFT)
#define SNVS_LPSMCMR_MC_ERA_BITS(n)     (((n) << SNVS_LPSMCMR_MC_ERA_BITS_SHIFT) & SNVS_LPSMCMR_MC_ERA_BITS_MASK)

/* SNVS_LP Secure Monotonic Counter LSB Register (LPSMCLR) */
#define SNVS_LPSMCLR_MON_COUNTER_SHIFT  (0)  /* Bits 0-32: Monotonic Counter bits Note that writing to this register does not change the value of this field to the value that was written */
#define SNVS_LPSMCLR_MON_COUNTER_MASK   (0xFFFFFFFF << SNVS_LPSMCLR_MON_COUNTER_SHIFT)
#define SNVS_LPSMCLR_MON_COUNTER(n)     (((n) << SNVS_LPSMCLR_MON_COUNTER_SHIFT) & SNVS_LPSMCLR_MON_COUNTER_MASK)

/* SNVS_LP Digital Low-Voltage Detector Register (LPLVDR) */
#define SNVS_LPLVDR_LVD_SHIFT  (0)  /* Bits 0-32: Low-Voltage Detector Value */
#define SNVS_LPLVDR_LVD_MASK   (0xFFFFFFFF << SNVS_LPLVDR_LVD_SHIFT)
#define SNVS_LPLVDR_LVD(n)     (((n) << SNVS_LPLVDR_LVD_SHIFT) & SNVS_LPLVDR_LVD_MASK)

/* SNVS_LP General Purpose Register 0 (legacy alias) (LPGPR0_LEGACY_ALIAS) */
#define SNVS_LPGPR0_LEGACY_ALIAS_GPR_SHIFT  (0)  /* Bits 0-32: General Purpose Register When GPR_SL or GPR_HL bit is set, the register cannot be programmed. */
#define SNVS_LPGPR0_LEGACY_ALIAS_GPR_MASK   (0xFFFFFFFF << SNVS_LPGPR0_LEGACY_ALIAS_GPR_SHIFT)
#define SNVS_LPGPR0_LEGACY_ALIAS_GPR(n)     (((n) << SNVS_LPGPR0_LEGACY_ALIAS_GPR_SHIFT) & SNVS_LPGPR0_LEGACY_ALIAS_GPR_MASK)

/* SNVS_LP Tamper Detectors Config 2 Register (LPTDC2R) */
#define SNVS_LPTDC2R_ET3_EN   (1 << 0)   /* Bit 0: External Tampering 3 Enable When set, external tampering 3 detection generates an LP security violation */
#define SNVS_LPTDC2R_ET4_EN   (1 << 1)   /* Bit 1: External Tampering 4 Enable When set, external tampering 4 detection generates an LP security violation */
#define SNVS_LPTDC2R_ET5_EN   (1 << 2)   /* Bit 2: External Tampering 5 Enable When set, external tampering 5 detection generates an LP security violation */
#define SNVS_LPTDC2R_ET6_EN   (1 << 3)   /* Bit 3: External Tampering 6 Enable When set, external tampering 6 detection generates an LP security violation */
#define SNVS_LPTDC2R_ET7_EN   (1 << 4)   /* Bit 4: External Tampering 7 Enable When set, external tampering 7 detection generates an LP security violation */
#define SNVS_LPTDC2R_ET8_EN   (1 << 5)   /* Bit 5: External Tampering 8 Enable When set, external tampering 8 detection generates an LP security violation */
#define SNVS_LPTDC2R_ET9_EN   (1 << 6)   /* Bit 6: External Tampering 9 Enable When set, external tampering 9 detection generates an LP security violation */
#define SNVS_LPTDC2R_ET10_EN  (1 << 7)   /* Bit 7: External Tampering 10 Enable When set, external tampering 10 detection generates an LP security violation */
#define SNVS_LPTDC2R_ET3P     (1 << 16)  /* Bit 16: External Tampering 3 Polarity This bit is used to determine the polarity of external tamper 3. */
#define SNVS_LPTDC2R_ET4P     (1 << 17)  /* Bit 17: External Tampering 4 Polarity This bit is used to determine the polarity of external tamper 4. */
#define SNVS_LPTDC2R_ET5P     (1 << 18)  /* Bit 18: External Tampering 5 Polarity This bit is used to determine the polarity of external tamper 5. */
#define SNVS_LPTDC2R_ET6P     (1 << 19)  /* Bit 19: External Tampering 6 Polarity This bit is used to determine the polarity of external tamper 6. */
#define SNVS_LPTDC2R_ET7P     (1 << 20)  /* Bit 20: External Tampering 7 Polarity This bit is used to determine the polarity of external tamper 7. */
#define SNVS_LPTDC2R_ET8P     (1 << 21)  /* Bit 21: External Tampering 8 Polarity This bit is used to determine the polarity of external tamper 8. */
#define SNVS_LPTDC2R_ET9P     (1 << 22)  /* Bit 22: External Tampering 9 Polarity This bit is used to determine the polarity of external tamper 9. */
#define SNVS_LPTDC2R_ET10P    (1 << 23)  /* Bit 23: External Tampering 10 Polarity This bit is used to determine the polarity of external tamper 10. */

/* SNVS_LP Tamper Detectors Status Register (LPTDSR) */
#define SNVS_LPTDSR_ET3D   (1 << 0)  /* Bit 0: External Tampering 3 Detected */
#define SNVS_LPTDSR_ET4D   (1 << 1)  /* Bit 1: External Tampering 4 Detected */
#define SNVS_LPTDSR_ET5D   (1 << 2)  /* Bit 2: External Tampering 5 Detected */
#define SNVS_LPTDSR_ET6D   (1 << 3)  /* Bit 3: External Tampering 6 Detected */
#define SNVS_LPTDSR_ET7D   (1 << 4)  /* Bit 4: External Tampering 7 Detected */
#define SNVS_LPTDSR_ET8D   (1 << 5)  /* Bit 5: External Tampering 8 Detected */
#define SNVS_LPTDSR_ET9D   (1 << 6)  /* Bit 6: External Tampering 9 Enable When set, external tampering 9 detection generates an LP security violation */
#define SNVS_LPTDSR_ET10D  (1 << 7)  /* Bit 7: External Tampering 10 Detected */

/* SNVS_LP Tamper Glitch Filter 1 Configuration Register (LPTGF1CR) */
#define SNVS_LPTGF1CR_ETGF3_SHIFT  (0)        /* Bits 0-7: External Tamper Glitch Filter 3 Configures the length of the digital glitch filter for the external tamper 3 pin between 128 and 32640 SRTC clock cycles */
#define SNVS_LPTGF1CR_ETGF3_MASK   (0x7F << SNVS_LPTGF1CR_ETGF3_SHIFT)
#define SNVS_LPTGF1CR_ETGF3(n)     (((n) << SNVS_LPTGF1CR_ETGF3_SHIFT) & SNVS_LPTGF1CR_ETGF3_MASK)
#define SNVS_LPTGF1CR_ETGF3_EN     (1 << 7)   /* Bit 7: External Tamper Glitch Filter 3 Enable When set, enables the external tamper glitch filter 3. */
#define SNVS_LPTGF1CR_ETGF4_SHIFT  (8)        /* Bits 8-15: External Tamper Glitch Filter 4 Configures the length of the digital glitch filter for the external tamper 4 pin between 128 and 32640 SRTC clock cycles */
#define SNVS_LPTGF1CR_ETGF4_MASK   (0x7F << SNVS_LPTGF1CR_ETGF4_SHIFT)
#define SNVS_LPTGF1CR_ETGF4(n)     (((n) << SNVS_LPTGF1CR_ETGF4_SHIFT) & SNVS_LPTGF1CR_ETGF4_MASK)
#define SNVS_LPTGF1CR_ETGF4_EN     (1 << 15)  /* Bit 15: External Tamper Glitch Filter 4 Enable When set, enables the external tamper glitch filter 4. */
#define SNVS_LPTGF1CR_ETGF5_SHIFT  (16)       /* Bits 16-23: External Tamper Glitch Filter 5 Configures the length of the digital glitch filter for the external tamper 5 pin between 128 and 32640 SRTC clock cycles */
#define SNVS_LPTGF1CR_ETGF5_MASK   (0x7F << SNVS_LPTGF1CR_ETGF5_SHIFT)
#define SNVS_LPTGF1CR_ETGF5(n)     (((n) << SNVS_LPTGF1CR_ETGF5_SHIFT) & SNVS_LPTGF1CR_ETGF5_MASK)
#define SNVS_LPTGF1CR_ETGF5_EN     (1 << 23)  /* Bit 23: External Tamper Glitch Filter 5 Enable When set, enables the external tamper glitch filter 5. */
#define SNVS_LPTGF1CR_ETGF6_SHIFT  (24)       /* Bits 24-31: External Tamper Glitch Filter 6 Configures the length of the digital glitch filter for the external tamper 6 pin between 128 and 32640 SRTC clock cycles */
#define SNVS_LPTGF1CR_ETGF6_MASK   (0x7F << SNVS_LPTGF1CR_ETGF6_SHIFT)
#define SNVS_LPTGF1CR_ETGF6(n)     (((n) << SNVS_LPTGF1CR_ETGF6_SHIFT) & SNVS_LPTGF1CR_ETGF6_MASK)
#define SNVS_LPTGF1CR_ETGF6_EN     (1 << 31)  /* Bit 31: External Tamper Glitch Filter 6 Enable When set, enables the external tamper glitch filter 6. */

/* SNVS_LP Tamper Glitch Filter 2 Configuration Register (LPTGF2CR) */
#define SNVS_LPTGF2CR_ETGF7_SHIFT   (0)        /* Bits 0-7: External Tamper Glitch Filter 7 Configures the length of the digital glitch filter for the external tamper 7 pin between 128 and 32640 SRTC clock cycles */
#define SNVS_LPTGF2CR_ETGF7_MASK    (0x7F << SNVS_LPTGF2CR_ETGF7_SHIFT)
#define SNVS_LPTGF2CR_ETGF7(n)      (((n) << SNVS_LPTGF2CR_ETGF7_SHIFT) & SNVS_LPTGF2CR_ETGF7_MASK)
#define SNVS_LPTGF2CR_ETGF7_EN      (1 << 7)   /* Bit 7: External Tamper Glitch Filter 7 Enable When set, enables the external tamper glitch filter 7. */
#define SNVS_LPTGF2CR_ETGF8_SHIFT   (8)        /* Bits 8-15: External Tamper Glitch Filter 8 Configures the length of the digital glitch filter for the external tamper 8 pin between 128 and 32640 SRTC clock cycles */
#define SNVS_LPTGF2CR_ETGF8_MASK    (0x7F << SNVS_LPTGF2CR_ETGF8_SHIFT)
#define SNVS_LPTGF2CR_ETGF8(n)      (((n) << SNVS_LPTGF2CR_ETGF8_SHIFT) & SNVS_LPTGF2CR_ETGF8_MASK)
#define SNVS_LPTGF2CR_ETGF8_EN      (1 << 15)  /* Bit 15: External Tamper Glitch Filter 8 Enable When set, enables the external tamper glitch filter 8. */
#define SNVS_LPTGF2CR_ETGF9_SHIFT   (16)       /* Bits 16-23: External Tamper Glitch Filter 9 Configures the length of the digital glitch filter for the external tamper 9 pin between 128 and 32640 SRTC clock cycles */
#define SNVS_LPTGF2CR_ETGF9_MASK    (0x7F << SNVS_LPTGF2CR_ETGF9_SHIFT)
#define SNVS_LPTGF2CR_ETGF9(n)      (((n) << SNVS_LPTGF2CR_ETGF9_SHIFT) & SNVS_LPTGF2CR_ETGF9_MASK)
#define SNVS_LPTGF2CR_ETGF9_EN      (1 << 23)  /* Bit 23: External Tamper Glitch Filter 9 Enable When set, enables the external tamper glitch filter 9. */
#define SNVS_LPTGF2CR_ETGF10_SHIFT  (24)       /* Bits 24-31: External Tamper Glitch Filter 10 Configures the length of the digital glitch filter for the external tamper 10 pin between 128 and 32640 SRTC clock cycles */
#define SNVS_LPTGF2CR_ETGF10_MASK   (0x7F << SNVS_LPTGF2CR_ETGF10_SHIFT)
#define SNVS_LPTGF2CR_ETGF10(n)     (((n) << SNVS_LPTGF2CR_ETGF10_SHIFT) & SNVS_LPTGF2CR_ETGF10_MASK)
#define SNVS_LPTGF2CR_ETGF10_EN     (1 << 31)  /* Bit 31: External Tamper Glitch Filter 10 Enable When set, enables the external tamper glitch filter 10. */

/* SNVS_LP Active Tamper 1 Configuration Register (LPAT1CR) */
#define SNVS_LPAT1CR_SEED_SHIFT        (0)   /* Bits 0-16: Active Tamper 1 Initial Seed Default Seed is 1111h. */
#define SNVS_LPAT1CR_SEED_MASK         (0xFFFF << SNVS_LPAT1CR_SEED_SHIFT)
#define SNVS_LPAT1CR_SEED(n)           (((n) << SNVS_LPAT1CR_SEED_SHIFT) & SNVS_LPAT1CR_Seed_MASK)
#define SNVS_LPAT1CR_POLYNOMIAL_SHIFT  (16)  /* Bits 16-32: Active Tamper 1 Polynomial Default Polynomial is 8400h. */
#define SNVS_LPAT1CR_POLYNOMIAL_MASK   (0xFFFF << SNVS_LPAT1CR_POLYNOMIAL_SHIFT)
#define SNVS_LPAT1CR_POLYNOMIAL(n)     (((n) << SNVS_LPAT1CR_POLYNOMIAL_SHIFT) & SNVS_LPAT1CR_Polynomial_MASK)

/* SNVS_LP Active Tamper 2 Configuration Register (LPAT2CR) */
#define SNVS_LPAT2CR_SEED_SHIFT        (0)   /* Bits 0-16: Active Tamper 2 Initial Seed Default Seed is 2222h. */
#define SNVS_LPAT2CR_SEED_MASK         (0xFFFF << SNVS_LPAT2CR_SEED_SHIFT)
#define SNVS_LPAT2CR_SEED(n)           (((n) << SNVS_LPAT2CR_SEED_SHIFT) & SNVS_LPAT2CR_Seed_MASK)
#define SNVS_LPAT2CR_POLYNOMIAL_SHIFT  (16)  /* Bits 16-32: Active Tamper 2 Polynomial Default Polynomial is 9C00h. */
#define SNVS_LPAT2CR_POLYNOMIAL_MASK   (0xFFFF << SNVS_LPAT2CR_POLYNOMIAL_SHIFT)
#define SNVS_LPAT2CR_POLYNOMIAL(n)     (((n) << SNVS_LPAT2CR_POLYNOMIAL_SHIFT) & SNVS_LPAT2CR_Polynomial_MASK)

/* SNVS_LP Active Tamper 3 Configuration Register (LPAT3CR) */
#define SNVS_LPAT3CR_SEED_SHIFT        (0)   /* Bits 0-16: Active Tamper 3 Initial Seed Default Seed is 3333h. */
#define SNVS_LPAT3CR_SEED_MASK         (0xFFFF << SNVS_LPAT3CR_SEED_SHIFT)
#define SNVS_LPAT3CR_SEED(n)           (((n) << SNVS_LPAT3CR_SEED_SHIFT) & SNVS_LPAT3CR_Seed_MASK)
#define SNVS_LPAT3CR_POLYNOMIAL_SHIFT  (16)  /* Bits 16-32: Active Tamper 3 Polynomial Default Polynomial is CA00h. */
#define SNVS_LPAT3CR_POLYNOMIAL_MASK   (0xFFFF << SNVS_LPAT3CR_POLYNOMIAL_SHIFT)
#define SNVS_LPAT3CR_POLYNOMIAL(n)     (((n) << SNVS_LPAT3CR_POLYNOMIAL_SHIFT) & SNVS_LPAT3CR_Polynomial_MASK)

/* SNVS_LP Active Tamper 4 Configuration Register (LPAT4CR) */
#define SNVS_LPAT4CR_SEED_SHIFT        (0)   /* Bits 0-16: Active Tamper 4 Initial Seed Default Seed is 4444h. */
#define SNVS_LPAT4CR_SEED_MASK         (0xFFFF << SNVS_LPAT4CR_SEED_SHIFT)
#define SNVS_LPAT4CR_SEED(n)           (((n) << SNVS_LPAT4CR_SEED_SHIFT) & SNVS_LPAT4CR_Seed_MASK)
#define SNVS_LPAT4CR_POLYNOMIAL_SHIFT  (16)  /* Bits 16-32: Active Tamper 4 Polynomial Default Polynomial is 8580h. */
#define SNVS_LPAT4CR_POLYNOMIAL_MASK   (0xFFFF << SNVS_LPAT4CR_POLYNOMIAL_SHIFT)
#define SNVS_LPAT4CR_POLYNOMIAL(n)     (((n) << SNVS_LPAT4CR_POLYNOMIAL_SHIFT) & SNVS_LPAT4CR_Polynomial_MASK)

/* SNVS_LP Active Tamper 5 Configuration Register (LPAT5CR) */
#define SNVS_LPAT5CR_SEED_SHIFT        (0)   /* Bits 0-16: Active Tamper 5 Initial Seed Default Seed is 5555h. */
#define SNVS_LPAT5CR_SEED_MASK         (0xFFFF << SNVS_LPAT5CR_SEED_SHIFT)
#define SNVS_LPAT5CR_SEED(n)           (((n) << SNVS_LPAT5CR_SEED_SHIFT) & SNVS_LPAT5CR_Seed_MASK)
#define SNVS_LPAT5CR_POLYNOMIAL_SHIFT  (16)  /* Bits 16-32: Active Tamper 5 Polynomial Default Polynomial is A840h. */
#define SNVS_LPAT5CR_POLYNOMIAL_MASK   (0xFFFF << SNVS_LPAT5CR_POLYNOMIAL_SHIFT)
#define SNVS_LPAT5CR_POLYNOMIAL(n)     (((n) << SNVS_LPAT5CR_POLYNOMIAL_SHIFT) & SNVS_LPAT5CR_Polynomial_MASK)

/* SNVS_LP Active Tamper Control Register (LPATCTLR) */
#define SNVS_LPATCTLR_AT1_EN      (1 << 0)   /* Bit 0: Active Tamper 1 Enable When set, enables the Active Tamper 1 LFSR. */
#define SNVS_LPATCTLR_AT2_EN      (1 << 1)   /* Bit 1: Active Tamper 2 Enable When set, enables the Active Tamper 2 LFSR. */
#define SNVS_LPATCTLR_AT3_EN      (1 << 2)   /* Bit 2: Active Tamper 3 Enable When set, enables the Active Tamper 3 LFSR. */
#define SNVS_LPATCTLR_AT4_EN      (1 << 3)   /* Bit 3: Active Tamper 4 Enable When set, enables the Active Tamper 4 LFSR. */
#define SNVS_LPATCTLR_AT5_EN      (1 << 4)   /* Bit 4: Active Tamper 5 Enable When set, enables the Active Tamper 5 LFSR. */
#define SNVS_LPATCTLR_AT1_PAD_EN  (1 << 16)  /* Bit 16: Active Tamper 1 Pad Out Enable When set, enables the Active Tamper 1 external pad. */
#define SNVS_LPATCTLR_AT2_PAD_EN  (1 << 17)  /* Bit 17: Active Tamper 2 Pad Out Enable When set, enables the Active Tamper 2 external pad. */
#define SNVS_LPATCTLR_AT3_PAD_EN  (1 << 18)  /* Bit 18: Active Tamper 3 Pad Out Enable When set, enables the Active Tamper 3 external pad. */
#define SNVS_LPATCTLR_AT4_PAD_EN  (1 << 19)  /* Bit 19: Active Tamper 4 Pad Out Enable When set, enables the Active Tamper 4 external pad. */
#define SNVS_LPATCTLR_AT5_PAD_EN  (1 << 20)  /* Bit 20: Active Tamper 5 Pad Out Enable When set, enables the Active Tamper 5 external pad. */

/* SNVS_LP Active Tamper Clock Control Register (LPATCLKR) */
#define SNVS_LPATCLKR_AT1_CLK_CTL_SHIFT  (0)   /* Bits 0-2: Active Tamper 1 Clock Control 00: 16hz 01: 8hz 10: 4hz 11: 2hz */
#define SNVS_LPATCLKR_AT1_CLK_CTL_MASK   (0x3 << SNVS_LPATCLKR_AT1_CLK_CTL_SHIFT)
#define SNVS_LPATCLKR_AT1_CLK_CTL(n)     (((n) << SNVS_LPATCLKR_AT1_CLK_CTL_SHIFT) & SNVS_LPATCLKR_AT1_CLK_CTL_MASK)
#define SNVS_LPATCLKR_AT2_CLK_CTL_SHIFT  (4)   /* Bits 4-6: Active Tamper 2 Clock Control 00: 16hz 01: 8hz 10: 4hz 11: 2hz */
#define SNVS_LPATCLKR_AT2_CLK_CTL_MASK   (0x3 << SNVS_LPATCLKR_AT2_CLK_CTL_SHIFT)
#define SNVS_LPATCLKR_AT2_CLK_CTL(n)     (((n) << SNVS_LPATCLKR_AT2_CLK_CTL_SHIFT) & SNVS_LPATCLKR_AT2_CLK_CTL_MASK)
#define SNVS_LPATCLKR_AT3_CLK_CTL_SHIFT  (8)   /* Bits 8-10: Active Tamper 3 Clock Control 00: 16hz 01: 8hz 10: 4hz 11: 2hz */
#define SNVS_LPATCLKR_AT3_CLK_CTL_MASK   (0x3 << SNVS_LPATCLKR_AT3_CLK_CTL_SHIFT)
#define SNVS_LPATCLKR_AT3_CLK_CTL(n)     (((n) << SNVS_LPATCLKR_AT3_CLK_CTL_SHIFT) & SNVS_LPATCLKR_AT3_CLK_CTL_MASK)
#define SNVS_LPATCLKR_AT4_CLK_CTL_SHIFT  (12)  /* Bits 12-14: Active Tamper 4 Clock Control 00: 16hz 01: 8hz 10: 4hz 11: 2hz */
#define SNVS_LPATCLKR_AT4_CLK_CTL_MASK   (0x3 << SNVS_LPATCLKR_AT4_CLK_CTL_SHIFT)
#define SNVS_LPATCLKR_AT4_CLK_CTL(n)     (((n) << SNVS_LPATCLKR_AT4_CLK_CTL_SHIFT) & SNVS_LPATCLKR_AT4_CLK_CTL_MASK)
#define SNVS_LPATCLKR_AT5_CLK_CTL_SHIFT  (16)  /* Bits 16-18: Active Tamper 5 Clock Control 00: 16hz 01: 8hz 10: 4hz 11: 2hz */
#define SNVS_LPATCLKR_AT5_CLK_CTL_MASK   (0x3 << SNVS_LPATCLKR_AT5_CLK_CTL_SHIFT)
#define SNVS_LPATCLKR_AT5_CLK_CTL(n)     (((n) << SNVS_LPATCLKR_AT5_CLK_CTL_SHIFT) & SNVS_LPATCLKR_AT5_CLK_CTL_MASK)

/* SNVS_LP Active Tamper Routing Control 1 Register (LPATRC1R) */
#define SNVS_LPATRC1R_ET1RCTL_SHIFT  (0)   /* Bits 0-3: External Tamper 1 Routing Control Any undefined selection will be routed to passive */
#define SNVS_LPATRC1R_ET1RCTL_MASK   (0x7 << SNVS_LPATRC1R_ET1RCTL_SHIFT)
#define SNVS_LPATRC1R_ET1RCTL(n)     (((n) << SNVS_LPATRC1R_ET1RCTL_SHIFT) & SNVS_LPATRC1R_ET1RCTL_MASK)
#define SNVS_LPATRC1R_ET2RCTL_SHIFT  (4)   /* Bits 4-7: External Tamper 2 Routing Control Any undefined selection will be routed to passive */
#define SNVS_LPATRC1R_ET2RCTL_MASK   (0x7 << SNVS_LPATRC1R_ET2RCTL_SHIFT)
#define SNVS_LPATRC1R_ET2RCTL(n)     (((n) << SNVS_LPATRC1R_ET2RCTL_SHIFT) & SNVS_LPATRC1R_ET2RCTL_MASK)
#define SNVS_LPATRC1R_ET3RCTL_SHIFT  (8)   /* Bits 8-11: External Tamper 3 Routing Control Any undefined selection will be routed to passive */
#define SNVS_LPATRC1R_ET3RCTL_MASK   (0x7 << SNVS_LPATRC1R_ET3RCTL_SHIFT)
#define SNVS_LPATRC1R_ET3RCTL(n)     (((n) << SNVS_LPATRC1R_ET3RCTL_SHIFT) & SNVS_LPATRC1R_ET3RCTL_MASK)
#define SNVS_LPATRC1R_ET4RCTL_SHIFT  (12)  /* Bits 12-15: External Tamper 4 Routing Control Any undefined selection will be routed to passive */
#define SNVS_LPATRC1R_ET4RCTL_MASK   (0x7 << SNVS_LPATRC1R_ET4RCTL_SHIFT)
#define SNVS_LPATRC1R_ET4RCTL(n)     (((n) << SNVS_LPATRC1R_ET4RCTL_SHIFT) & SNVS_LPATRC1R_ET4RCTL_MASK)
#define SNVS_LPATRC1R_ET5RCTL_SHIFT  (16)  /* Bits 16-19: External Tamper 5 Routing Control Any undefined selection will be routed to passive */
#define SNVS_LPATRC1R_ET5RCTL_MASK   (0x7 << SNVS_LPATRC1R_ET5RCTL_SHIFT)
#define SNVS_LPATRC1R_ET5RCTL(n)     (((n) << SNVS_LPATRC1R_ET5RCTL_SHIFT) & SNVS_LPATRC1R_ET5RCTL_MASK)
#define SNVS_LPATRC1R_ET6RCTL_SHIFT  (20)  /* Bits 20-23: External Tamper 6 Routing Control Any undefined selection will be routed to passive */
#define SNVS_LPATRC1R_ET6RCTL_MASK   (0x7 << SNVS_LPATRC1R_ET6RCTL_SHIFT)
#define SNVS_LPATRC1R_ET6RCTL(n)     (((n) << SNVS_LPATRC1R_ET6RCTL_SHIFT) & SNVS_LPATRC1R_ET6RCTL_MASK)
#define SNVS_LPATRC1R_ET7RCTL_SHIFT  (24)  /* Bits 24-27: External Tamper 7 Routing Control Any undefined selection will be routed to passive */
#define SNVS_LPATRC1R_ET7RCTL_MASK   (0x7 << SNVS_LPATRC1R_ET7RCTL_SHIFT)
#define SNVS_LPATRC1R_ET7RCTL(n)     (((n) << SNVS_LPATRC1R_ET7RCTL_SHIFT) & SNVS_LPATRC1R_ET7RCTL_MASK)
#define SNVS_LPATRC1R_ET8RCTL_SHIFT  (28)  /* Bits 28-31: External Tamper 8 Routing Control Any undefined selection will be routed to passive */
#define SNVS_LPATRC1R_ET8RCTL_MASK   (0x7 << SNVS_LPATRC1R_ET8RCTL_SHIFT)
#define SNVS_LPATRC1R_ET8RCTL(n)     (((n) << SNVS_LPATRC1R_ET8RCTL_SHIFT) & SNVS_LPATRC1R_ET8RCTL_MASK)

/* SNVS_LP Active Tamper Routing Control 2 Register (LPATRC2R) */
#define SNVS_LPATRC2R_ET9RCTL_SHIFT   (0)  /* Bits 0-3: External Tamper 9 Routing Control Any undefined selection will be routed to passive */
#define SNVS_LPATRC2R_ET9RCTL_MASK    (0x7 << SNVS_LPATRC2R_ET9RCTL_SHIFT)
#define SNVS_LPATRC2R_ET9RCTL(n)      (((n) << SNVS_LPATRC2R_ET9RCTL_SHIFT) & SNVS_LPATRC2R_ET9RCTL_MASK)
#define SNVS_LPATRC2R_ET10RCTL_SHIFT  (4)  /* Bits 4-7: External Tamper 10 Routing Control Any undefined selection will be routed to passive */
#define SNVS_LPATRC2R_ET10RCTL_MASK   (0x7 << SNVS_LPATRC2R_ET10RCTL_SHIFT)
#define SNVS_LPATRC2R_ET10RCTL(n)     (((n) << SNVS_LPATRC2R_ET10RCTL_SHIFT) & SNVS_LPATRC2R_ET10RCTL_MASK)

/* SNVS_HP Version ID Register 1 (HPVIDR1) */
#define SNVS_HPVIDR1_MINOR_REV_SHIFT  (0)   /* Bits 0-8: SNVS block minor version number */
#define SNVS_HPVIDR1_MINOR_REV_MASK   (0xFF << SNVS_HPVIDR1_MINOR_REV_SHIFT)
#define SNVS_HPVIDR1_MINOR_REV(n)     (((n) << SNVS_HPVIDR1_MINOR_REV_SHIFT) & SNVS_HPVIDR1_MINOR_REV_MASK)
#define SNVS_HPVIDR1_MAJOR_REV_SHIFT  (8)   /* Bits 8-16: SNVS block major version number */
#define SNVS_HPVIDR1_MAJOR_REV_MASK   (0xFF << SNVS_HPVIDR1_MAJOR_REV_SHIFT)
#define SNVS_HPVIDR1_MAJOR_REV(n)     (((n) << SNVS_HPVIDR1_MAJOR_REV_SHIFT) & SNVS_HPVIDR1_MAJOR_REV_MASK)
#define SNVS_HPVIDR1_IP_ID_SHIFT      (16)  /* Bits 16-32: SNVS block ID */
#define SNVS_HPVIDR1_IP_ID_MASK       (0xFFFF << SNVS_HPVIDR1_IP_ID_SHIFT)
#define SNVS_HPVIDR1_IP_ID(n)         (((n) << SNVS_HPVIDR1_IP_ID_SHIFT) & SNVS_HPVIDR1_IP_ID_MASK)

/* SNVS_HP Version ID Register 2 (HPVIDR2) */
#define SNVS_HPVIDR2_ECO_REV_SHIFT  (8)   /* Bits 8-16: SNVS ECO Revision The engineering change order revision number for this release of SNVS. */
#define SNVS_HPVIDR2_ECO_REV_MASK   (0xFF << SNVS_HPVIDR2_ECO_REV_SHIFT)
#define SNVS_HPVIDR2_ECO_REV(n)     (((n) << SNVS_HPVIDR2_ECO_REV_SHIFT) & SNVS_HPVIDR2_ECO_REV_MASK)
#define SNVS_HPVIDR2_IP_ERA_SHIFT   (24)  /* Bits 24-32: IP Era 00h - Era 1 or 2 03h - Era 3 04h - Era 4 05h - Era 5 06h - Era 6 */
#define SNVS_HPVIDR2_IP_ERA_MASK    (0xFF << SNVS_HPVIDR2_IP_ERA_SHIFT)
#define SNVS_HPVIDR2_IP_ERA(n)      (((n) << SNVS_HPVIDR2_IP_ERA_SHIFT) & SNVS_HPVIDR2_IP_ERA_MASK)

/* SNVS_LP Zeroizable Master Key Register (LPZMKR0) */
#define SNVS_LPZMKR0_ZMK_SHIFT  (0)  /* Bits 0-32: Zeroizable Master Key Each of these registers contains 32 bits of the 256-bit ZMK value */
#define SNVS_LPZMKR0_ZMK_MASK   (0xFFFFFFFF << SNVS_LPZMKR0_ZMK_SHIFT)
#define SNVS_LPZMKR0_ZMK(n)     (((n) << SNVS_LPZMKR0_ZMK_SHIFT) & SNVS_LPZMKR0_ZMK_MASK)

/* SNVS_LP Zeroizable Master Key Register (LPZMKR1) */
#define SNVS_LPZMKR1_ZMK_SHIFT  (0)  /* Bits 0-32: Zeroizable Master Key Each of these registers contains 32 bits of the 256-bit ZMK value */
#define SNVS_LPZMKR1_ZMK_MASK   (0xFFFFFFFF << SNVS_LPZMKR1_ZMK_SHIFT)
#define SNVS_LPZMKR1_ZMK(n)     (((n) << SNVS_LPZMKR1_ZMK_SHIFT) & SNVS_LPZMKR1_ZMK_MASK)

/* SNVS_LP Zeroizable Master Key Register (LPZMKR2) */
#define SNVS_LPZMKR2_ZMK_SHIFT  (0)  /* Bits 0-32: Zeroizable Master Key Each of these registers contains 32 bits of the 256-bit ZMK value */
#define SNVS_LPZMKR2_ZMK_MASK   (0xFFFFFFFF << SNVS_LPZMKR2_ZMK_SHIFT)
#define SNVS_LPZMKR2_ZMK(n)     (((n) << SNVS_LPZMKR2_ZMK_SHIFT) & SNVS_LPZMKR2_ZMK_MASK)

/* SNVS_LP Zeroizable Master Key Register (LPZMKR3) */
#define SNVS_LPZMKR3_ZMK_SHIFT  (0)  /* Bits 0-32: Zeroizable Master Key Each of these registers contains 32 bits of the 256-bit ZMK value */
#define SNVS_LPZMKR3_ZMK_MASK   (0xFFFFFFFF << SNVS_LPZMKR3_ZMK_SHIFT)
#define SNVS_LPZMKR3_ZMK(n)     (((n) << SNVS_LPZMKR3_ZMK_SHIFT) & SNVS_LPZMKR3_ZMK_MASK)

/* SNVS_LP Zeroizable Master Key Register (LPZMKR4) */
#define SNVS_LPZMKR4_ZMK_SHIFT  (0)  /* Bits 0-32: Zeroizable Master Key Each of these registers contains 32 bits of the 256-bit ZMK value */
#define SNVS_LPZMKR4_ZMK_MASK   (0xFFFFFFFF << SNVS_LPZMKR4_ZMK_SHIFT)
#define SNVS_LPZMKR4_ZMK(n)     (((n) << SNVS_LPZMKR4_ZMK_SHIFT) & SNVS_LPZMKR4_ZMK_MASK)

/* SNVS_LP Zeroizable Master Key Register (LPZMKR5) */
#define SNVS_LPZMKR5_ZMK_SHIFT  (0)  /* Bits 0-32: Zeroizable Master Key Each of these registers contains 32 bits of the 256-bit ZMK value */
#define SNVS_LPZMKR5_ZMK_MASK   (0xFFFFFFFF << SNVS_LPZMKR5_ZMK_SHIFT)
#define SNVS_LPZMKR5_ZMK(n)     (((n) << SNVS_LPZMKR5_ZMK_SHIFT) & SNVS_LPZMKR5_ZMK_MASK)

/* SNVS_LP Zeroizable Master Key Register (LPZMKR6) */
#define SNVS_LPZMKR6_ZMK_SHIFT  (0)  /* Bits 0-32: Zeroizable Master Key Each of these registers contains 32 bits of the 256-bit ZMK value */
#define SNVS_LPZMKR6_ZMK_MASK   (0xFFFFFFFF << SNVS_LPZMKR6_ZMK_SHIFT)
#define SNVS_LPZMKR6_ZMK(n)     (((n) << SNVS_LPZMKR6_ZMK_SHIFT) & SNVS_LPZMKR6_ZMK_MASK)

/* SNVS_LP Zeroizable Master Key Register (LPZMKR7) */
#define SNVS_LPZMKR7_ZMK_SHIFT  (0)  /* Bits 0-32: Zeroizable Master Key Each of these registers contains 32 bits of the 256-bit ZMK value */
#define SNVS_LPZMKR7_ZMK_MASK   (0xFFFFFFFF << SNVS_LPZMKR7_ZMK_SHIFT)
#define SNVS_LPZMKR7_ZMK(n)     (((n) << SNVS_LPZMKR7_ZMK_SHIFT) & SNVS_LPZMKR7_ZMK_MASK)

/* SNVS_LP General Purpose Registers 0 .. 3 (LPGPR_ALIAS0) */
#define SNVS_LPGPR_ALIAS0_GPR_SHIFT  (0)  /* Bits 0-32: General Purpose Register When GPR_SL or GPR_HL bit is set, the register cannot be programmed. */
#define SNVS_LPGPR_ALIAS0_GPR_MASK   (0xFFFFFFFF << SNVS_LPGPR_ALIAS0_GPR_SHIFT)
#define SNVS_LPGPR_ALIAS0_GPR(n)     (((n) << SNVS_LPGPR_ALIAS0_GPR_SHIFT) & SNVS_LPGPR_ALIAS0_GPR_MASK)

/* SNVS_LP General Purpose Registers 0 .. 3 (LPGPR_ALIAS1) */
#define SNVS_LPGPR_ALIAS1_GPR_SHIFT  (0)  /* Bits 0-32: General Purpose Register When GPR_SL or GPR_HL bit is set, the register cannot be programmed. */
#define SNVS_LPGPR_ALIAS1_GPR_MASK   (0xFFFFFFFF << SNVS_LPGPR_ALIAS1_GPR_SHIFT)
#define SNVS_LPGPR_ALIAS1_GPR(n)     (((n) << SNVS_LPGPR_ALIAS1_GPR_SHIFT) & SNVS_LPGPR_ALIAS1_GPR_MASK)

/* SNVS_LP General Purpose Registers 0 .. 3 (LPGPR_ALIAS2) */
#define SNVS_LPGPR_ALIAS2_GPR_SHIFT  (0)  /* Bits 0-32: General Purpose Register When GPR_SL or GPR_HL bit is set, the register cannot be programmed. */
#define SNVS_LPGPR_ALIAS2_GPR_MASK   (0xFFFFFFFF << SNVS_LPGPR_ALIAS2_GPR_SHIFT)
#define SNVS_LPGPR_ALIAS2_GPR(n)     (((n) << SNVS_LPGPR_ALIAS2_GPR_SHIFT) & SNVS_LPGPR_ALIAS2_GPR_MASK)

/* SNVS_LP General Purpose Registers 0 .. 3 (LPGPR_ALIAS3) */
#define SNVS_LPGPR_ALIAS3_GPR_SHIFT  (0)  /* Bits 0-32: General Purpose Register When GPR_SL or GPR_HL bit is set, the register cannot be programmed. */
#define SNVS_LPGPR_ALIAS3_GPR_MASK   (0xFFFFFFFF << SNVS_LPGPR_ALIAS3_GPR_SHIFT)
#define SNVS_LPGPR_ALIAS3_GPR(n)     (((n) << SNVS_LPGPR_ALIAS3_GPR_SHIFT) & SNVS_LPGPR_ALIAS3_GPR_MASK)

/* SNVS_LP General Purpose Registers 0 .. 3 (LPGPR0) */
#define SNVS_LPGPR0_GPR_SHIFT  (0)  /* Bits 0-32: General Purpose Register When GPR_SL or GPR_HL bit is set, the register cannot be programmed. */
#define SNVS_LPGPR0_GPR_MASK   (0xFFFFFFFF << SNVS_LPGPR0_GPR_SHIFT)
#define SNVS_LPGPR0_GPR(n)     (((n) << SNVS_LPGPR0_GPR_SHIFT) & SNVS_LPGPR0_GPR_MASK)

/* SNVS_LP General Purpose Registers 0 .. 3 (LPGPR1) */
#define SNVS_LPGPR1_GPR_SHIFT  (0)  /* Bits 0-32: General Purpose Register When GPR_SL or GPR_HL bit is set, the register cannot be programmed. */
#define SNVS_LPGPR1_GPR_MASK   (0xFFFFFFFF << SNVS_LPGPR1_GPR_SHIFT)
#define SNVS_LPGPR1_GPR(n)     (((n) << SNVS_LPGPR1_GPR_SHIFT) & SNVS_LPGPR1_GPR_MASK)

/* SNVS_LP General Purpose Registers 0 .. 3 (LPGPR2) */
#define SNVS_LPGPR2_GPR_SHIFT  (0)  /* Bits 0-32: General Purpose Register When GPR_SL or GPR_HL bit is set, the register cannot be programmed. */
#define SNVS_LPGPR2_GPR_MASK   (0xFFFFFFFF << SNVS_LPGPR2_GPR_SHIFT)
#define SNVS_LPGPR2_GPR(n)     (((n) << SNVS_LPGPR2_GPR_SHIFT) & SNVS_LPGPR2_GPR_MASK)

/* SNVS_LP General Purpose Registers 0 .. 3 (LPGPR3) */
#define SNVS_LPGPR3_GPR_SHIFT  (0)  /* Bits 0-32: General Purpose Register When GPR_SL or GPR_HL bit is set, the register cannot be programmed. */
#define SNVS_LPGPR3_GPR_MASK   (0xFFFFFFFF << SNVS_LPGPR3_GPR_SHIFT)
#define SNVS_LPGPR3_GPR(n)     (((n) << SNVS_LPGPR3_GPR_SHIFT) & SNVS_LPGPR3_GPR_MASK)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_SNVS_H */
