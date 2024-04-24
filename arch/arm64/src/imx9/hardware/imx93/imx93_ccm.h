/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx93/imx93_ccm.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_CCM_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_CCM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define IMX9_CCM_CR_CTRL_OFFSET(n)          (0x0000 + ((n) << 7)) /* Clock root control (CLOCK_ROOTn_CONTROL, n=0..94) */
#define IMX9_CCM_CR_CTRL_SET_OFFSET(n)      (0x0004 + ((n) << 7)) /* Clock root control (CLOCK_ROOTn_CONTROL_SET, n=0..94) */
#define IMX9_CCM_CR_CTRL_CLR_OFFSET(n)      (0x0008 + ((n) << 7)) /* Clock root control (CLOCK_ROOTn_CONTROL_CLR, n=0..94) */
#define IMX9_CCM_CR_CTRL_TOG_OFFSET(n)      (0x000c + ((n) << 7)) /* Clock root control (CLOCK_ROOTn_CONTROL_TOG, n=0..94) */
#define IMX9_CCM_CR_STAT0_OFFSET(n)         (0x0020 + ((n) << 7)) /* Clock root working status (CLOCK_ROOTn_STATUS0, n=0..94) */
#define IMX9_CCM_CR_AUTH_OFFSET(n)          (0x0030 + ((n) << 7)) /* Clock root access control (CLOCK_ROOTn_AUTHEN, n=0..94) */
#define IMX9_CCM_CR_AUTH_SET_OFFSET(n)      (0x0034 + ((n) << 7)) /* Clock root access control (CLOCK_ROOTn_AUTHEN_SET, n=0..94) */
#define IMX9_CCM_CR_AUTH_CLR_OFFSET(n)      (0x0038 + ((n) << 7)) /* Clock root access control (CLOCK_ROOTn_AUTHEN_CLR, n=0..94) */
#define IMX9_CCM_CR_AUTH_TOG_OFFSET(n)      (0x003c + ((n) << 7)) /* Clock root access control (CLOCK_ROOTn_AUTHEN_TOG, n=0..94) */

#define IMX9_CCM_GPR_SH_OFFSET(n)           (0x4800 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn, n=0..7) */
#define IMX9_CCM_GPR_SH_SET_OFFSET(n)       (0x4804 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_SET, n=0..7) */
#define IMX9_CCM_GPR_SH_CLR_OFFSET(n)       (0x4808 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_CLR, n=0..7) */
#define IMX9_CCM_GPR_SH_TOG_OFFSET(n)       (0x480c + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_TOG, n=0..7) */
#define IMX9_CCM_GPR_SH_AUTH_OFFSET(n)      (0x4810 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_AUTHEN, n=0..7) */
#define IMX9_CCM_GPR_SH_AUTH_SET_OFFSET(n)  (0x4814 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_AUTHEN_SET, n=0..7) */
#define IMX9_CCM_GPR_SH_AUTH_CLR_OFFSET(n)  (0x4818 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_AUTHEN_CLR, n=0..7) */
#define IMX9_CCM_GPR_SH_AUTH_TOG_OFFSET(n)  (0x481c + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_AUTHEN_TOG, n=0..7) */

#define IMX9_CCM_GPR_PR_OFFSET(n)           (0x4c00 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn, n=1..7) */
#define IMX9_CCM_GPR_PR_SET_OFFSET(n)       (0x4c04 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_SET, n=1..7) */
#define IMX9_CCM_GPR_PR_CLR_OFFSET(n)       (0x4c08 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_CLR, n=1..7) */
#define IMX9_CCM_GPR_PR_TOG_OFFSET(n)       (0x4c0c + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_TOG, n=1..7) */
#define IMX9_CCM_GPR_PR_AUTH_OFFSET(n)      (0x4c10 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_AUTHEN, n=1..7) */
#define IMX9_CCM_GPR_PR_AUTH_SET_OFFSET(n)  (0x4c14 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_AUTHEN_SET, n=1..7) */
#define IMX9_CCM_GPR_PR_AUTH_CLR_OFFSET(n)  (0x4c18 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_AUTHEN_CLR, n=1..7) */
#define IMX9_CCM_GPR_PR_AUTH_TOG_OFFSET(n)  (0x4c1c + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_AUTHEN_TOG, n=1..7) */

#define IMX9_CCM_OSCPLL_DIR_OFFSET(n)       (0x5000 + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..18) */
#define IMX9_CCM_OSCPLL_LPM_STAT0_OFFSET(n) (0x5004 + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..18) */
#define IMX9_CCM_OSCPLL_LPM_STAT1_OFFSET(n) (0x5008 + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..18) */
#define IMX9_CCM_OSCPLL_LPM0_OFFSET(n)      (0x5010 + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..18) */
#define IMX9_CCM_OSCPLL_LPM1_OFFSET(n)      (0x5014 + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..18) */
#define IMX9_CCM_OSCPLL_LPM_CUR_OFFSET(n)   (0x501C + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..18) */
#define IMX9_CCM_OSCPLL_STAT0_OFFSET(n)     (0x5020 + ((n) << 6)) /* Clock source working status (OSCPLLn_STATUS0, n=0..18) */
#define IMX9_CCM_OSCPLL_STAT1_OFFSET(n)     (0x5024 + ((n) << 6)) /* Clock source low power status (OSCPLLn_STATUS1, n=0..18) */
#define IMX9_CCM_OSCPLL_AUTH_OFFSET(n)      (0x5030 + ((n) << 6)) /* Clock source access control (OSCPLLn_AUTHEN, n=0..18) */

#define IMX9_CCM_LPCG_DIR_OFFSET(n)         (0x8000 + ((n) << 6)) /* LPCG direct control (LPCGn_DIRECT, n=0..126) */
#define IMX9_CCM_LPCG_LPM_STAT0_OFFSET(n)   (0x8004 + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..126) */
#define IMX9_CCM_LPCG_LPM_STAT1_OFFSET(n)   (0x8008 + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..126) */
#define IMX9_CCM_LPCG_LPM0_OFFSET(n)        (0x8010 + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..126) */
#define IMX9_CCM_LPCG_LPM1_OFFSET(n)        (0x8014 + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..126) */
#define IMX9_CCM_LPCG_LPM_CUR_OFFSET(n)     (0x801C + ((n) << 6)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..126) */
#define IMX9_CCM_LPCG_STAT0_OFFSET(n)       (0x8020 + ((n) << 6)) /* LPCG working status (LPCGn_STATUS0, n=0..126) */
#define IMX9_CCM_LPCG_STAT1_OFFSET(n)       (0x8024 + ((n) << 6)) /* LPCG low power status (LPCGn_STATUS1, n=0..126) */
#define IMX9_CCM_LPCG_AUTH_OFFSET(n)        (0x8030 + ((n) << 6)) /* LPCG access control (LPCGn_AUTHEN, n=0..126) */

/* Register addresses *******************************************************/

#define IMX9_CCM_CR_CTRL(n)                 (IMX9_CCM_CTRL_BASE + IMX9_CCM_CR_CTRL_OFFSET(n))
#define IMX9_CCM_CR_CTRL_SET(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_CR_CTRL_SET_OFFSET(n))
#define IMX9_CCM_CR_CTRL_CLR(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_CR_CTRL_CLR_OFFSET(n))
#define IMX9_CCM_CR_CTRL_TOG(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_CR_CTRL_TOG_OFFSET(n))
#define IMX9_CCM_CR_STAT0(n)                (IMX9_CCM_CTRL_BASE + IMX9_CCM_CR_STAT0_OFFSET(n))
#define IMX9_CCM_CR_AUTH(n)                 (IMX9_CCM_CTRL_BASE + IMX9_CCM_CR_AUTH_OFFSET(n))
#define IMX9_CCM_CR_AUTH_SET(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_CR_AUTH_SET_OFFSET(n))
#define IMX9_CCM_CR_AUTH_CLR(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_CR_AUTH_CLR_OFFSET(n))
#define IMX9_CCM_CR_AUTH_TOG(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_CR_AUTH_TOG_OFFSET(n))

#define IMX9_CCM_GPR_SH(n)                  (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_SH_OFFSET(n))
#define IMX9_CCM_GPR_SH_SET(n)              (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_SH_SET_OFFSET(n))
#define IMX9_CCM_GPR_SH_CLR(n)              (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_SH_CLR_OFFSET(n))
#define IMX9_CCM_GPR_SH_TOG(n)              (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_SH_TOG_OFFSET(n))
#define IMX9_CCM_GPR_SH_AUTH(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_SH_AUTH_OFFSET(n))
#define IMX9_CCM_GPR_SH_AUTH_SET(n)         (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_SH_AUTH_SET_OFFSET(n))
#define IMX9_CCM_GPR_SH_AUTH_CLR(n)         (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_SH_AUTH_CLR_OFFSET(n))
#define IMX9_CCM_GPR_SH_AUTH_TOG(n)         (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_SH_AUTH_TOG_OFFSET(n))

#define IMX9_CCM_GPR_PR(n)                  (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_PR_OFFSET(n))
#define IMX9_CCM_GPR_PR_SET(n)              (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_PR_SET_OFFSET(n))
#define IMX9_CCM_GPR_PR_CLR(n)              (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_PR_CLR_OFFSET(n))
#define IMX9_CCM_GPR_PR_TOG(n)              (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_PR_TOG_OFFSET(n))
#define IMX9_CCM_GPR_PR_AUTH(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_PR_AUTH_OFFSET(n))
#define IMX9_CCM_GPR_PR_AUTH_SET(n)         (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_PR_AUTH_SET_OFFSET(n))
#define IMX9_CCM_GPR_PR_AUTH_CLR(n)         (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_PR_AUTH_CLR_OFFSET(n))
#define IMX9_CCM_GPR_PR_AUTH_TOG(n)         (IMX9_CCM_CTRL_BASE + IMX9_CCM_GPR_PR_AUTH_TOG_OFFSET(n))

#define IMX9_CCM_OSCPLL_DIR(n)              (IMX9_CCM_CTRL_BASE + IMX9_CCM_OSCPLL_DIR_OFFSET(n))
#define IMX9_CCM_OSCPLL_LPM_STAT0(n)        (IMX9_CCM_CTRL_BASE + IMX9_CCM_OSCPLL_LPM_STAT0_OFFSET(n))
#define IMX9_CCM_OSCPLL_LPM_STAT1(n)        (IMX9_CCM_CTRL_BASE + IMX9_CCM_OSCPLL_LPM_STAT1_OFFSET(n))
#define IMX9_CCM_OSCPLL_LPM0(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_OSCPLL_LPM0_OFFSET(n))
#define IMX9_CCM_OSCPLL_LPM1(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_OSCPLL_LPM1_OFFSET(n))
#define IMX9_CCM_OSCPLL_LPM_CUR(n)          (IMX9_CCM_CTRL_BASE + IMX9_CCM_OSCPLL_LPM1_OFFSET(n))
#define IMX9_CCM_OSCPLL_STAT0(n)            (IMX9_CCM_CTRL_BASE + IMX9_CCM_OSCPLL_STAT0_OFFSET(n))
#define IMX9_CCM_OSCPLL_STAT1(n)            (IMX9_CCM_CTRL_BASE + IMX9_CCM_OSCPLL_STAT1_OFFSET(n))
#define IMX9_CCM_OSCPLL_AUTH(n)             (IMX9_CCM_CTRL_BASE + IMX9_CCM_OSCPLL_AUTH_OFFSET(n))

#define IMX9_CCM_LPCG_DIR(n)                (IMX9_CCM_CTRL_BASE + IMX9_CCM_LPCG_DIR_OFFSET(n))
#define IMX9_CCM_LPCG_LPM_STAT0(n)          (IMX9_CCM_CTRL_BASE + IMX9_CCM_LPCG_LPM_STAT0_OFFSET(n))
#define IMX9_CCM_LPCG_LPM_STAT1(n)          (IMX9_CCM_CTRL_BASE + IMX9_CCM_LPCG_LPM_STAT1_OFFSET(n))
#define IMX9_CCM_LPCG_LPM0(n)               (IMX9_CCM_CTRL_BASE + IMX9_CCM_LPCG_LPM0_OFFSET(n))
#define IMX9_CCM_LPCG_LPM1(n)               (IMX9_CCM_CTRL_BASE + IMX9_CCM_LPCG_LPM1_OFFSET(n))
#define IMX9_CCM_LPCG_LPM_CUR(n)            (IMX9_CCM_CTRL_BASE + IMX9_CCM_LPCG_LPM1_OFFSET(n))
#define IMX9_CCM_LPCG_STAT0(n)              (IMX9_CCM_CTRL_BASE + IMX9_CCM_LPCG_STAT0_OFFSET(n))
#define IMX9_CCM_LPCG_STAT1(n)              (IMX9_CCM_CTRL_BASE + IMX9_CCM_LPCG_STAT1_OFFSET(n))
#define IMX9_CCM_LPCG_AUTH(n)               (IMX9_CCM_CTRL_BASE + IMX9_CCM_LPCG_AUTH_OFFSET(n))

/* Register bit definitions *************************************************/

/* Clock root control (CLOCK_ROOTn_CONTROL, n=0..94) */

#define CCM_CR_CTRL_DIV_SHIFT             (0)       /* Bits 0-7:   Divide selected clock by DIV+1 (DIV) */
#define CCM_CR_CTRL_DIV_MASK              (0xff << CCM_CR_CTRL_DIV_SHIFT)
#  define CCM_CR_CTRL_DIV(n)              (((n)-1) << CCM_CR_CTRL_DIV_SHIFT) /* Divide selected clock by n */

#define CCM_CR_CTRL_MUX_SHIFT             (8)       /* Bits 8-9:  Select clock from 8 clock sources (MUX) */
#define CCM_CR_CTRL_MUX_MASK              (0x03 << CCM_CR_CTRL_MUX_SHIFT)
#  define CCM_CR_CTRL_MUX_SRCSEL(n)       ((n) << CCM_CR_CTRL_MUX_SHIFT) /* Select clock source n */

                                                    /* Bits 11-23: Reserved */
#define CCM_CR_CTRL_OFF                   (1 << 24) /* Bit 24:     Shutdown clock root (OFF) */
                                                    /* Bits 25-31: Reserved */

/* Clock root working status (CLOCK_ROOTn_STATUS0, n=0..94) */

#define CCM_CR_STAT0_DIV_SHIFT            (0)       /* Bits 0-7:   Current clock root DIV setting (DIV) */
#define CCM_CR_STAT0_DIV_MASK             (0xff << CCM_CR_STAT0_DIV_SHIFT)
#define CCM_CR_STAT0_MUX_SHIFT            (8)       /* Bits 8-9:  Current clock root MUX setting (MUX) */
#define CCM_CR_STAT0_MUX_MASK             (0x03 << CCM_CR_STAT0_MUX_SHIFT)
                                                    /* Bits 11-23: Reserved */
#define CCM_CR_STAT0_OFF                  (1 << 24) /* Bit 24:     Current clock root OFF setting (OFF) */
                                                    /* Bits 25-27: Reserved */
#define CCM_CR_STAT0_SLICE_BUSY           (1 << 28) /* Bit 28:     Clock generation logic is applying the new setting (SLICE_BUSY) */
#define CCM_CR_STAT0_CHANGING             (1 << 31) /* Bit 31:     Clock generation logic is updating currently (CHANGING) */

/* Clock root access control (CLOCK_ROOTn_AUTHEN, n=0..94) */

#define CCM_CR_AUTH_TZ_USER               (1 << 8)  /* Bit 8:      Clock root can be changed in user mode (TZ_USER) */
#define CCM_CR_AUTH_TZ_NS                 (1 << 9)  /* Bit 9:      Clock root can be changed in non-secure mode (TZ_NS) */
                                                    /* Bit 10:     Reserved */
#define CCM_CR_AUTH_LOCK_TZ               (1 << 11) /* Bit 1:      Lock TrustZone settings (LOCK_TZ) */
                                                    /* Bits 12-14: Reserved */
#define CCM_CR_AUTH_LOCK_LIST             (1 << 12) /* Bit 15:     Lock whitelist settings (LOCK_LIST) */
#define CCM_CR_AUTH_WHITE_LIST_SHIFT      (16)      /* Bits 16-31:  Allow domains to change clock (WHITE_LIST) */
#define CCM_CR_AUTH_WHITE_LIST_MASK       (0xffff << CCM_CR_AUTH_WHITE_LIST_SHIFT)

/* General Purpose Register (GPR_SHAREDn, n=0..7) */

#define CCM_GPR_SH_GPR_SHIFT              (0)       /* Bits 0-31:  General purpose register, shared for all CPU domains (GPR) */
#define CCM_GPR_SH_GPR_MASK               (0xffffffff << CCM_GPR_SH_GPR_SHIFT)
#define CCM_GPR_A55_CLK_SEL_SHIFT         (0)
#define CCM_GPR_A55_CLK_SEL_MASK          (0x01 << CCM_GPR_A55_CLK_SEL_SHIFT)
#define CCM_GPR_A55_CLK_SEL_CCM           (0 << 0)
#define CCM_GPR_A55_CLK_SEL_PLL           (1 << 0)

/* General Purpose Register (GPR_SHAREDn_AUTHEN, n=0..7) */

#define CCM_GPR_SH_AUTH_TZ_USER           (1 << 8)  /* Bit 8:      Clock root can be changed in user mode (TZ_USER) */
#define CCM_GPR_SH_AUTH_TZ_NS             (1 << 9)  /* Bit 9:      Clock root can be changed in non-secure mode (TZ_NS) */
                                                    /* Bit 10:     Reserved */
#define CCM_GPR_SH_AUTH_LOCK_TZ           (1 << 11) /* Bit 1:      Lock TrustZone settings (LOCK_TZ) */
                                                    /* Bits 12-14: Reserved */
#define CCM_GPR_SH_AUTH_LOCK_LIST         (1 << 12) /* Bit 15:     Lock whitelist settings (LOCK_LIST) */
#define CCM_GPR_SH_AUTH_WHITE_LIST_SHIFT  (16)      /* Bits 16-31:  Allow domains to change clock (WHITE_LIST) */
#define CCM_GPR_SH_AUTH_WHITE_LIST_MASK   (0xffff << CCM_GPR_SH_AUTH_WHITE_LIST_SHIFT)

/* General Purpose Register (GPR_PRIVATEn, n=1..7) */

#define CCM_GPR_PR_GPR_SHIFT              (0)       /* Bits 0-31:  General purpose register, with dedicated bits for each domain (GPR) */
#define CCM_GPR_PR_GPR_MASK               (0xffffffff << CCM_GPR_PR_GPR_SHIFT)

/* General Purpose Register (GPR_PRIVATEn_AUTHEN, n=1..7) */

#define CCM_GPR_PR_AUTH_TZ_USER           (1 << 8)  /* Bit 8:      Clock root can be changed in user mode (TZ_USER) */
#define CCM_GPR_PR_AUTH_TZ_NS             (1 << 9)  /* Bit 9:      Clock root can be changed in non-secure mode (TZ_NS) */
                                                    /* Bit 10:     Reserved */
#define CCM_GPR_PR_AUTH_LOCK_TZ           (1 << 11) /* Bit 1:      Lock TrustZone settings (LOCK_TZ) */
                                                    /* Bits 12-14: Reserved */
#define CCM_GPR_PR_AUTH_LOCK_LIST         (1 << 12) /* Bit 15:     Lock whitelist settings (LOCK_LIST) */
#define CCM_GPR_PR_AUTH_WHITE_LIST_SHIFT  (16)      /* Bits 16-31: Allow domains to change clock (WHITE_LIST) */
#define CCM_GPR_PR_AUTH_WHITE_LIST_MASK   (0xffff << CCM_CR_AUTH_WHITE_LIST_SHIFT)

/* Clock source direct control (OSCPLLn_DIRECT, n=0..18) */

#define CCM_OSCPLL_DIR_ON                 (1 << 0)  /* Bit 0:      Turn on clock source (ON) */
                                                    /* Bits 1-31:  Reserved */

/* Clock source LPM status (OSCPLLn_LPM_STATUS0/1, n=0..18) */

#define CCM_OSCPLL_LPM_STAT_CPU_MODE_SHIFT      (0)  /* Bits 0-1:    Current mode of CPU */
#define CCM_OSCPLL_LPM_STAT_CPU_MODE_MASK       (0x03 << CCM_OSCPLL_LPM_STAT_CPU_MODE_SHIFT)
#define CCM_OSCPLL_LPM_STAT_CPU_TRANS_REQ_SHIFT (2)  /* Bit  2:      Domain request pending */
#define CCM_OSCPLL_LPM_STAT_CPU_TRANS_REQ_MASK  (0x01 << CCM_OSCPLL_LPM_STAT_CPU_TRANS_REQ_SHIFT)
#  define CCM_OSCPLL_LPM_STAT_ON                (0)  /* CPU is in RUN mode */
#  define CCM_OSCPLL_LPM_STAT_WAIT              (1)  /* CPU is in WAIT mode */
#  define CCM_OSCPLL_LPM_STAT_STOP              (2)  /* CPU is in STOP mode */
#  define CCM_OSCPLL_LPM_STAT_SUSPED            (3)  /* CPU is in SUSPEND mode */

/* CPU domain[n] from OSCPLLn_LPM_STATUS0/1 */

#define CCM_OSCPLL_LPM_STAT_CPU_DOMAIN_SHIFT    (4)
#define CCM_OSCPLL_LPM_STAT_CPU_DOMAIN(n)       ((n) * CCM_OSCPLL_LPM_STAT_CPU_DOMAIN_SHIFT)
#define CCM_OSCPLL_LPM_STAT_CPU_MODE_GET(n, v)  (((v) >> CCM_OSCPLL_LPM_STAT_CPU_DOMAIN(n)) & CCM_OSCPLL_LPM_STAT_CPU_MODE_MASK)

/* Clock source LPM mode (OSCPLLn_LPM_0/1 and _CUR, n=0..18) */

#define CCM_OSCPLL_LPM_MODE_SHIFT               (0)  /* Bits 0-2:    Current mode of CPU */
#define CCM_OSCPLL_LPM_MODE_MASK                (0x07 << CCM_OSCPLL_LPM_MODE_SHIFT)
#  define CCM_OSCPLL_LPM_MODE_OFF               (0)  /* Clock is off during all modes */
#  define CCM_OSCPLL_LPM_MODE_RUN               (1)  /* Clock is on in run mode, but off in WAIT and STOP modes */
#  define CCM_OSCPLL_LPM_MODE_RUNWAIT           (2)  /* Clock is on in run and wait modes, but off in STOP modes */
#  define CCM_OSCPLL_LPM_MODE_RUNWAITSTOP       (3)  /* Clock is on during all modes, except SUSPEND mode */
#  define CCM_OSCPLL_LPM_MODE_ALL               (4)  /* Clock is on during all modes */

/* CPU domain[n] from  OSCPLLn_LPM_0/1 */

#define CCM_OSCPLL_LPM_MODE_CPU_DOMAIN_SHIFT    (4)
#define CCM_OSCPLL_LPM_MODE_CPU_DOMAIN(n)       ((n) * CCM_OSCPLL_LPM_MODE_CPU_DOMAIN_SHIFT)
#define CCM_OSCPLL_LPM_MODE_CPU_MODE_SET(n, v)  ((v) & CCM_OSCPLL_LPM_MODE_MASK << CCM_LPCG_LPM_MODE_CPU_DOMAIN(n))
#define CCM_OSCPLL_LPM_MODE_CPU_MODE_GET(n, v)  (((v) >> CCM_LPCG_LPM_STAT_CPU_DOMAIN(n)) & CCM_OSCPLL_LPM_MODE_MASK)

/* Clock source working status (OSCPLLn_STATUS0, n=0..18) */

#define CCM_OSCPLL_STAT0_ON               (1 << 0)  /* Bit 0:      Clock source is turned on (ON) */
                                                    /* Bits 1-3:   Reserved */
#define CCM_OSCPLL_STAT0_STATUS_EARLY     (1 << 4)  /* Bit 4:      Clock source is active (STATUS_EARLY) */
#define CCM_OSCPLL_STAT0_STATUS_LATE      (1 << 5)  /* Bit 5:      Clock source is ready to use (STATUS_LATE) */
                                                    /* Bits 6-11:  Reserved */
#define CCM_OSCPLL_STAT0_IN_USE           (1 << 12) /* Bit 28:     Indicates whether the clock source is being used by active clock roots (IN_USE) */
                                                    /* Bits 16-31: Reserved */

/* Clock source low power status (OSCPLLn_STATUS1, n=0..18) */

#define CCM_OSCPLL_STAT1_DOM_ACTIVE_SHIFT (0)       /* Bits 0-15:   Domain active */
#define CCM_OSCPLL_STAT1_DOM_ACTIVE_MASK  (0xffff << CCM_OSCPLL_STAT1_DOM_ACTIVE_SHIFT)
#define CCM_OSCPLL_STAT1_DOM_ENABLE_SHIFT (16)      /* Bits 16-32:  Domain enabled */
#define CCM_OSCPLL_STAT1_DOM_ENABLE_MASK  (0xffff << CCM_OSCPLL_STAT1_DOM_ENABLE_SHIFT)

/* Clock source access control (OSCPLLn_AUTHEN, n=0..18) */

#define CCM_OSCPLL_AUTH_CPULPM            (1 << 2)  /* Bit 2:      CPU Low Power Mode (CPULPM) */
#define CCM_OSCPLL_AUTH_AUTO_CTRL         (1 << 3)  /* Bit 2:      Auto mode (AUTO_CTRL) */
#define CCM_OSCPLL_AUTH_LOCK_MODE         (1 << 7)  /* Bit 7:      Lock low power and access mode (LOCK_MODE) */
#define CCM_OSCPLL_AUTH_TZ_USER           (1 << 8)  /* Bit 8:      Clock source can be changed in user mode (TZ_USER) */
#define CCM_OSCPLL_AUTH_TZ_NS             (1 << 9)  /* Bit 9:      Clock source can be changed in non-secure mode (TZ_NS) */
                                                    /* Bit 10:     Reserved */
#define CCM_OSCPLL_AUTH_LOCK_TZ           (1 << 11) /* Bit 11:     Lock TrustZone settings (LOCK_TZ) */
                                                    /* Bits 12-14: Reserved */
#define CCM_OSCPLL_AUTH_LOCK_LIST         (1 << 15) /* Bit 15:     Lock whitelist settings (LOCK_LIST) */
#define CCM_OSCPLL_AUTH_WHITE_LIST_SHIFT  (16)      /* Bits 16-31: Allow domains to change clock (WHITE_LIST) */
#define CCM_OSCPLL_AUTH_WHITE_LIST_MASK   (0xffff << CCM_OSCPLL_AUTH_WHITE_LIST_SHIFT)

/* LPCG direct control (LPCGn_DIRECT, n=0..126) */

#define CCM_LPCG_DIR_ON                   (1 << 0)  /* Bit 0:      LPCG on (ON) */
                                                    /* Bit 1:      Reserved */
#define CCM_LPCG_ACK_TIMEOUT_EN           (1 << 2)  /* Bit 2:      Ack timeout enable */
                                                    /* Bits 3-31:  Reserved */

/* Clock source LPM status (LPCGn_LPM_STATUS0/1, n=0..18) */

#define CCM_LPCG_LPM_STAT_CPU_MODE_SHIFT        (0)  /* Bits 0-1:    Current mode of CPU */
#define CCM_LPCG_LPM_STAT_CPU_MODE_MASK         (0x03 << CCM_LPCG_LPM_STAT_CPU_MODE_SHIFT)
#define CCM_LPCG_LPM_STAT_CPU_TRANS_REQ_SHIFT   (2)  /* Bit  2:      Domain request pending */
#define CCM_LPCG_LPM_STAT_CPU_TRANS_REQ_MASK    (0x01 << CCM_LPCG_LPM_STAT_CPU_TRANS_REQ_SHIFT)
#  define CCM_LPCG_LPM_STAT_ON                  (0)  /* CPU is in RUN mode */
#  define CCM_LPCG_LPM_STAT_WAIT                (1)  /* CPU is in WAIT mode */
#  define CCM_LPCG_LPM_STAT_STOP                (2)  /* CPU is in STOP mode */
#  define CCM_LPCG_LPM_STAT_SUSPED              (3)  /* CPU is in SUSPEND mode */

/* CPU domain[n] from OSCPLLn_LPM_STATUS0/1 */

#define CCM_LPCG_LPM_STAT_CPU_DOMAIN_SHIFT      (4)
#define CCM_LPCG_LPM_STAT_CPU_DOMAIN(n)         ((n) * CCM_LPCG_LPM_STAT_CPU_DOMAIN_SHIFT)
#define CCM_LPCG_LPM_STAT_CPU_MODE_GET(n, v)    (((v) >> CCM_LPCG_LPM_STAT_CPU_DOMAIN(n)) & CCM_LPCG_LPM_STAT_CPU_MODE_MASK)

/* Clock source LPM mode (LPCGn_LPM_0/1 and _CUR, n=0..18) */

#define CCM_LPCG_LPM_MODE_SHIFT                 (0)  /* Bits 0-2:    Current mode of CPU */
#define CCM_LPCG_LPM_MODE_MASK                  (0x07 << CCM_LPCG_LPM_MODE_SHIFT)
#  define CCM_LPCG_LPM_MODE_OFF                 (0)  /* Clock is off during all modes */
#  define CCM_LPCG_LPM_MODE_RUN                 (1)  /* Clock is on in run mode, but off in WAIT and STOP modes */
#  define CCM_LPCG_LPM_MODE_RUNWAIT             (2)  /* Clock is on in run and wait modes, but off in STOP modes */
#  define CCM_LPCG_LPM_MODE_RUNWAITSTOP         (3)  /* Clock is on during all modes, except SUSPEND mode */
#  define CCM_LPCG_LPM_MODE_ALL                 (4)  /* Clock is on during all modes */

/* CPU domain[n] from  LPCGn_LPM_0/1 */

#define CCM_LPCG_LPM_MODE_CPU_DOMAIN_SHIFT      (4)
#define CCM_LPCG_LPM_MODE_CPU_DOMAIN(n)         ((n) * CCM_LPCG_LPM_MODE_CPU_DOMAIN_SHIFT)
#define CCM_LPCG_LPM_MODE_CPU_MODE_SET(n, v)    ((v) & CCM_LPCG_LPM_MODE_MASK << CCM_LPCG_LPM_MODE_CPU_DOMAIN(n))
#define CCM_LPCG_LPM_MODE_CPU_MODE_GET(n, v)    (((v) >> CCM_LPCG_LPM_MODE_CPU_DOMAIN(n)) & CCM_LPCG_LPM_MODE_MASK)

/* LPCG working status (LPCGn_STATUS0, n=0..126) */

#define CCM_LPCG_STAT0_ON                 (1 << 0)  /* Bit 0:      Clock source is turned on (ON) */
                                                    /* Bits 1-31   Reserved */

/* LPCG low power status (LPCGn_STATUS1, n=0..126) */

#define CCM_LPCG_STAT0_ACTIVE_DOMAIN_SHIFT (8)      /* Bits 8-11:  Domains that own this clock source according to whitelist (ACTIVE_DOMAIN) */
#define CCM_LPCG_STAT0_ACTIVE_DOMAIN_MASK  (0x0f << CCM_LPCG_STAT0_ACTIVE_DOMAIN_SHIFT)
#define CCM_LPCG_STAT0_DOMAIN_ENABLE_SHIFT (8)      /* Bits 12-15: Enable status from each domain (DOMAIN_ENABLE) */
#define CCM_LPCG_STAT0_DOMAIN_ENABLE_MASK  (0x0f << CCM_LPCG_STAT0_DOMAIN_ENABLE_SHIFT)
                                                    /* Bits 16-31: Reserved */

/* LPCG access control (LPCGn_AUTHEN, n=0..126) */

#define CCM_LPCG_AUTH_CPULPM            (1 << 2)  /* Bit 2:      CPU Low Power Mode (CPULPM) */
#define CCM_LPCG_AUTH_LOCK_MODE         (1 << 7)  /* Bit 7:      Lock low power and access mode (LOCK_MODE) */
#define CCM_LPCG_AUTH_TZ_USER           (1 << 8)  /* Bit 8:      Clock source can be changed in user mode (TZ_USER) */
#define CCM_LPCG_AUTH_TZ_NS             (1 << 9)  /* Bit 9:      Clock source can be changed in non-secure mode (TZ_NS) */
                                                  /* Bit 10:     Reserved */
#define CCM_LPCG_AUTH_LOCK_TZ           (1 << 11) /* Bit 11:     Lock TrustZone settings (LOCK_TZ) */
                                                  /* Bits 12-14: Reserved */
#define CCM_LPCG_AUTH_LOCK_LIST         (1 << 15) /* Bit 15:     Lock whitelist settings (LOCK_LIST) */
#define CCM_LPCG_AUTH_WHITE_LIST_SHIFT  (16)      /* Bits 16-31: Allow domains to change clock (WHITE_LIST) */
#define CCM_LPCG_AUTH_WHITE_LIST_MASK   (0xffff << CCM_LPCG_AUTH_WHITE_LIST_SHIFT)

/* Clock roots */

#define CCM_CR_A55PERIPH        0       /* CLOCK Root Arm A55 Periph. */
#define CCM_CR_A55MTRBUS        1       /* CLOCK Root Arm A55 MTR BUS. */
#define CCM_CR_A55              2       /* CLOCK Root Arm A55. */
#define CCM_CR_M33              3       /* CLOCK Root M33. */
#define CCM_CR_SENTINEL         4       /* CLOCK Root Sentinel. */
#define CCM_CR_BUSWAKEUP        5       /* CLOCK Root Bus Wakeup. */
#define CCM_CR_BUSAON           6       /* CLOCK Root Bus Aon. */
#define CCM_CR_WAKEUPAXI        7       /* CLOCK Root Wakeup Axi. */
#define CCM_CR_SWOTRACE         8       /* CLOCK Root Swo Trace. */
#define CCM_CR_M33SYSTICK       9       /* CLOCK Root M33 Systick. */
#define CCM_CR_FLEXIO1          10      /* CLOCK Root Flexio1. */
#define CCM_CR_FLEXIO2          11      /* CLOCK Root Flexio2. */
#define CCM_CR_LPIT1            12      /* CLOCK Root Lpit1. */
#define CCM_CR_LPIT2            13      /* CLOCK Root Lpit2. */
#define CCM_CR_LPTMR1           14      /* CLOCK Root Lptmr1. */
#define CCM_CR_LPTMR2           15      /* CLOCK Root Lptmr2. */
#define CCM_CR_TPM1             16      /* CLOCK Root Tpm1. */
#define CCM_CR_TPM2             17      /* CLOCK Root Tpm2. */
#define CCM_CR_TPM3             18      /* CLOCK Root Tpm3. */
#define CCM_CR_TPM4             19      /* CLOCK Root Tpm4. */
#define CCM_CR_TPM5             20      /* CLOCK Root Tpm5. */
#define CCM_CR_TPM6             21      /* CLOCK Root Tpm6. */
#define CCM_CR_FLEXSPI1         22      /* CLOCK Root Flexspi1. */
#define CCM_CR_CAN1             23      /* CLOCK Root Can1. */
#define CCM_CR_CAN2             24      /* CLOCK Root Can2. */
#define CCM_CR_LPUART1          25      /* CLOCK Root Lpuart1. */
#define CCM_CR_LPUART2          26      /* CLOCK Root Lpuart2. */
#define CCM_CR_LPUART3          27      /* CLOCK Root Lpuart3. */
#define CCM_CR_LPUART4          28      /* CLOCK Root Lpuart4. */
#define CCM_CR_LPUART5          29      /* CLOCK Root Lpuart5. */
#define CCM_CR_LPUART6          30      /* CLOCK Root Lpuart6. */
#define CCM_CR_LPUART7          31      /* CLOCK Root Lpuart7. */
#define CCM_CR_LPUART8          32      /* CLOCK Root Lpuart8. */
#define CCM_CR_LPI2C1           33      /* CLOCK Root Lpi2c1. */
#define CCM_CR_LPI2C2           34      /* CLOCK Root Lpi2c2. */
#define CCM_CR_LPI2C3           35      /* CLOCK Root Lpi2c3. */
#define CCM_CR_LPI2C4           36      /* CLOCK Root Lpi2c4. */
#define CCM_CR_LPI2C5           37      /* CLOCK Root Lpi2c5. */
#define CCM_CR_LPI2C6           38      /* CLOCK Root Lpi2c6. */
#define CCM_CR_LPI2C7           39      /* CLOCK Root Lpi2c7. */
#define CCM_CR_LPI2C8           40      /* CLOCK Root Lpi2c8. */
#define CCM_CR_LPSPI1           41      /* CLOCK Root Lpspi1. */
#define CCM_CR_LPSPI2           42      /* CLOCK Root Lpspi2. */
#define CCM_CR_LPSPI3           43      /* CLOCK Root Lpspi3. */
#define CCM_CR_LPSPI4           44      /* CLOCK Root Lpspi4. */
#define CCM_CR_LPSPI5           45      /* CLOCK Root Lpspi5. */
#define CCM_CR_LPSPI6           46      /* CLOCK Root Lpspi6. */
#define CCM_CR_LPSPI7           47      /* CLOCK Root Lpspi7. */
#define CCM_CR_LPSPI8           48      /* CLOCK Root Lpspi8. */
#define CCM_CR_I3C1             49      /* CLOCK Root I3c1. */
#define CCM_CR_I3C2             50      /* CLOCK Root I3c2. */
#define CCM_CR_USDHC1           51      /* CLOCK Root Usdhc1. */
#define CCM_CR_USDHC2           52      /* CLOCK Root Usdhc2. */
#define CCM_CR_USDHC3           53      /* CLOCK Root Usdhc3. */
#define CCM_CR_SAI1             54      /* CLOCK Root Sai1. */
#define CCM_CR_SAI2             55      /* CLOCK Root Sai2. */
#define CCM_CR_SAI3             56      /* CLOCK Root Sai3. */
#define CCM_CR_CCMCKO1          57      /* CLOCK Root Ccm Cko1. */
#define CCM_CR_CCMCKO2          58      /* CLOCK Root Ccm Cko2. */
#define CCM_CR_CCMCKO3          59      /* CLOCK Root Ccm Cko3. */
#define CCM_CR_CCMCKO4          60      /* CLOCK Root Ccm Cko4. */
#define CCM_CR_HSIO             61      /* CLOCK Root Hsio. */
#define CCM_CR_HSIOUSBTEST60M   62      /* CLOCK Root Hsio Usb Test 60M. */
#define CCM_CR_HSIOACSCAN80M    63      /* CLOCK Root Hsio Acscan 80M. */
#define CCM_CR_HSIOACSCAN480M   64      /* CLOCK Root Hsio Acscan 480M. */
#define CCM_CR_NIC              65      /* CLOCK Root Nic. */
#define CCM_CR_NICAPB           66      /* CLOCK Root Nic Apb. */
#define CCM_CR_MLAPB            67      /* CLOCK Root Ml Apb. */
#define CCM_CR_ML               68      /* CLOCK Root Ml. */
#define CCM_CR_MEDIAAXI         69      /* CLOCK Root Media Axi. */
#define CCM_CR_MEDIAAPB         70      /* CLOCK Root Media Apb. */
#define CCM_CR_MEDIALDB         71      /* CLOCK Root Media Ldb. */
#define CCM_CR_MEDIADISPPIX     72      /* CLOCK Root Media Disp Pix. */
#define CCM_CR_CAMPIX           73      /* CLOCK Root Cam Pix. */
#define CCM_CR_MIPITESTBYTE     74      /* CLOCK Root Mipi Test Byte. */
#define CCM_CR_MIPIPHYCFG       75      /* CLOCK Root Mipi Phy Cfg. */
#define CCM_CR_DRAMALT          76      /* CLOCK Root Dram Alt. */
#define CCM_CR_DRAMAPB          77      /* CLOCK Root Dram Apb. */
#define CCM_CR_ADC              78      /* CLOCK Root Adc. */
#define CCM_CR_PDM              79      /* CLOCK Root Pdm. */
#define CCM_CR_TSTMR1           80      /* CLOCK Root Tstmr1. */
#define CCM_CR_TSTMR2           81      /* CLOCK Root Tstmr2. */
#define CCM_CR_MQS1             82      /* CLOCK Root MQS1. */
#define CCM_CR_MQS2             83      /* CLOCK Root MQS2. */
#define CCM_CR_AUDIOXCVR        84      /* CLOCK Root Audio XCVR. */
#define CCM_CR_SPDIF            85      /* CLOCK Root Spdif. */
#define CCM_CR_ENET             86      /* CLOCK Root Enet. */
#define CCM_CR_ENETTIMER1       87      /* CLOCK Root Enet Timer1. */
#define CCM_CR_ENETTIMER2       88      /* CLOCK Root Enet Timer2. */
#define CCM_CR_ENETREF          89      /* CLOCK Root Enet Ref. */
#define CCM_CR_ENETREFPHY       90      /* CLOCK Root Enet Ref Phy. */
#define CCM_CR_I3C1SLOW         91      /* CLOCK Root I3c1Slow. */
#define CCM_CR_I3C2SLOW         92      /* CLOCK Root I3c2Slow. */
#define CCM_CR_USBPHYBURUNIN    93      /* CLOCK Root Usb Phy Burunin. */
#define CCM_CR_PALCAMESCAN      94      /* CLOCK Root Pal Came Scan. */

/* Clock gates */

#define CCM_LPCG_A55            0
#define CCM_LPCG_CM33           1
#define CCM_LPCG_ARM_TROUT      2
#define CCM_LPCG_SENTINEL       3
#define CCM_LPCG_SIM_WAKEUP     4
#define CCM_LPCG_SIM_AON        5
#define CCM_LPCG_SIM_MEGA       6
#define CCM_LPCG_ANADIG         7
#define CCM_LPCG_SRC            8
#define CCM_LPCG_CCM            9
#define CCM_LPCG_GPC            10
#define CCM_LPCG_ADC1           11
#define CCM_LPCG_WDOG1          12
#define CCM_LPCG_WDOG2          13
#define CCM_LPCG_WDOG3          14
#define CCM_LPCG_WDOG4          15
#define CCM_LPCG_WDOG5          16
#define CCM_LPCG_SEMA1          17
#define CCM_LPCG_SEMA2          18
#define CCM_LPCG_MU_A           19
#define CCM_LPCG_MU_B           20
#define CCM_LPCG_EDMA3          21
#define CCM_LPCG_EDMA4          22
#define CCM_LPCG_ROMCP_A55      23
#define CCM_LPCG_ROMCP_M33      24
#define CCM_LPCG_FLEXSPI1       25
#define CCM_LPCG_AON_TRDC       26
#define CCM_LPCG_WKUP_TRDC      27
#define CCM_LPCG_OCOTP          28
#define CCM_LPCG_BBSM_HP        29
#define CCM_LPCG_BBSM           30
#define CCM_LPCG_CSTRACE        31
#define CCM_LPCG_CSSWO          32
#define CCM_LPCG_IOMUXC         33
#define CCM_LPCG_GPIO1          34
#define CCM_LPCG_GPIO2          35
#define CCM_LPCG_GPIO3          36
#define CCM_LPCG_GPIO4          37
#define CCM_LPCG_FLEXIO1        38
#define CCM_LPCG_FLEXIO2        39
#define CCM_LPCG_LPIT1          40
#define CCM_LPCG_LPIT2          41
#define CCM_LPCG_LPTMR1         42
#define CCM_LPCG_LPTMR2         43
#define CCM_LPCG_TPM1           44
#define CCM_LPCG_TPM2           45
#define CCM_LPCG_TPM3           46
#define CCM_LPCG_TPM4           47
#define CCM_LPCG_TPM5           48
#define CCM_LPCG_TPM6           49
#define CCM_LPCG_CAN1           50
#define CCM_LPCG_CAN2           51
#define CCM_LPCG_LPUART1        52
#define CCM_LPCG_LPUART2        53
#define CCM_LPCG_LPUART3        54
#define CCM_LPCG_LPUART4        55
#define CCM_LPCG_LPUART5        56
#define CCM_LPCG_LPUART6        57
#define CCM_LPCG_LPUART7        58
#define CCM_LPCG_LPUART8        59
#define CCM_LPCG_LPI2C1         60
#define CCM_LPCG_LPI2C2         61
#define CCM_LPCG_LPI2C3         62
#define CCM_LPCG_LPI2C4         63
#define CCM_LPCG_LPI2C5         64
#define CCM_LPCG_LPI2C6         65
#define CCM_LPCG_LPI2C7         66
#define CCM_LPCG_LPI2C8         67
#define CCM_LPCG_LPSPI1         68
#define CCM_LPCG_LPSPI2         69
#define CCM_LPCG_LPSPI3         70
#define CCM_LPCG_LPSPI4         71
#define CCM_LPCG_LPSPI5         72
#define CCM_LPCG_LPSPI6         73
#define CCM_LPCG_LPSPI7         74
#define CCM_LPCG_LPSPI8         75
#define CCM_LPCG_I3C1           76
#define CCM_LPCG_I3C2           77
#define CCM_LPCG_USDHC1         78
#define CCM_LPCG_USDHC2         79
#define CCM_LPCG_USDHC3         80
#define CCM_LPCG_SAI1           81
#define CCM_LPCG_SAI2           82
#define CCM_LPCG_SAI3           83
#define CCM_LPCG_SSI_W2AO       84
#define CCM_LPCG_SSI_AO2W       85
#define CCM_LPCG_MIPI_CSI       86
#define CCM_LPCG_MIPI_DSI       87
#define CCM_LPCG_LVDS           88
#define CCM_LPCG_LCDIF          89
#define CCM_LPCG_PXP            90
#define CCM_LPCG_ISI            91
#define CCM_LPCG_NIC_MEDIA      92
#define CCM_LPCG_DDR_DFI        93
#define CCM_LPCG_DDR_CTL        94
#define CCM_LPCG_DDR_DFI_CTL    95
#define CCM_LPCG_DDR_SSI        96
#define CCM_LPCG_DDR_BYPASS     97
#define CCM_LPCG_DDR_APB        98
#define CCM_LPCG_DDR_DRAMPLL    99
#define CCM_LPCG_DDR_CLK_CTL    100
#define CCM_LPCG_NIC_CENTRAL    101
#define CCM_LPCG_GIC600         102
#define CCM_LPCG_NIC_APB        103
#define CCM_LPCG_USB_CONTROLLER 104
#define CCM_LPCG_USB_TEST_60M   105
#define CCM_LPCG_HSIO_TROUT_24M 106
#define CCM_LPCG_PDM            107
#define CCM_LPCG_MQS1           108
#define CCM_LPCG_MQS2           109
#define CCM_LPCG_AUD_XCVR       110
#define CCM_LPCG_NICMIX_MECC    111
#define CCM_LPCG_SPDIF          112
#define CCM_LPCG_SSI_ML2NIC     113
#define CCM_LPCG_SSI_MED2NIC    114
#define CCM_LPCG_SSI_HSIO2NIC   115
#define CCM_LPCG_SSI_W2NIC      116
#define CCM_LPCG_SSI_NIC2W      117
#define CCM_LPCG_SSI_NIC2DDR    118
#define CCM_LPCG_HSIO_32K       119
#define CCM_LPCG_ENET1          120
#define CCM_LPCG_ENET_QOS       121
#define CCM_LPCG_SYS_CNT        122
#define CCM_LPCG_TSTMR1         123
#define CCM_LPCG_TSTMR2         124
#define CCM_LPCG_TMC            125
#define CCM_LPCG_PMRO           126

/* Shared register indices */

#define CCM_SHARED_EXT_CLK      0
#define CCM_SHARED_A55_CLK      1
#define CCM_SHARED_DRAM_CLK     2

/* Other parameters */

#define ROOT_MUX_MAX            4       /* Count of root clock MUX options */
#define CCM_CR_COUNT            94      /* Count of clock roots */
#define CCM_LPCG_COUNT          126     /* Counte of clock gates */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* NOTE: The PLL input (IN) clocks are not available in clock tree */

enum ccm_clock_name_e
{
  OSC_24M          = 0,  /* 24MHZ OSCILLATOR. */
  ARM_PLL          = 1,  /* ARM PLL */
  ARM_PLLOUT       = 2,  /* ARM PLL OUT */
  SYS_PLL1_IN      = 3,  /* SYSTEM PLL1 IN */
  SYS_PLL1PFD0_IN  = 4,  /* SYSTEM PLL1 PFD0 IN */
  SYS_PLL1PFD0     = 5,  /* SYSTEM PLL1 PFD0 */
  SYS_PLL1PFD0DIV2 = 6,  /* SYSTEM PLL1 PFD0 DIV2  */
  SYS_PLL1PFD1_IN  = 7,  /* SYSTEM PLL1 PFD1 IN */
  SYS_PLL1PFD1     = 8,  /* SYSTEM PLL1 PFD1 */
  SYS_PLL1PFD1DIV2 = 9,  /* SYSTEM PLL1 PFD1 DIV2  */
  SYS_PLL1PFD2_IN  = 10, /* SYSTEM PLL1 PFD2 IN */
  SYS_PLL1PFD2     = 11, /* SYSTEM PLL1 PFD2 */
  SYS_PLL1PFD2DIV2 = 12, /* SYSTEM PLL1 PFD2 DIV2  */
  AUDIO_PLL1       = 13, /* AUDIO PLL1 */
  AUDIO_PLL1OUT    = 14, /* AUDIO PLL1 OUT */
  DRAM_PLL         = 15, /* DRAM PLL */
  DRAM_PLLOUT      = 16, /* DRAM PLL OUT */
  VIDEO_PLL1       = 17, /* VIDEO PLL1 */
  VIDEO_PLL1OUT    = 18, /* VIDEO PLL1 OUT */
  EXT              = 19, /* EXT */
};

/* This contains a simple LUT to find the corresponding MUX index per root */

static const int g_ccm_root_mux[][ROOT_MUX_MAX] =
{
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Arm A55 Periph */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Arm A55 MTR BUS */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Arm A55 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* M33 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Sentinel */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Bus Wakeup */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Bus Aon */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Wakeup Axi */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Swo Trace */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* M33 Systick */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Flexio1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Flexio2 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpit1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpit2 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lptmr1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lptmr2 */
  {OSC_24M, SYS_PLL1PFD0, AUDIO_PLL1OUT, EXT},                     /* Tpm1 */
  {OSC_24M, SYS_PLL1PFD0, AUDIO_PLL1OUT, EXT},                     /* Tpm2 */
  {OSC_24M, SYS_PLL1PFD0, AUDIO_PLL1OUT, EXT},                     /* Tpm3 */
  {OSC_24M, SYS_PLL1PFD0, AUDIO_PLL1OUT, EXT},                     /* Tpm4 */
  {OSC_24M, SYS_PLL1PFD0, AUDIO_PLL1OUT, EXT},                     /* Tpm5 */
  {OSC_24M, SYS_PLL1PFD0, AUDIO_PLL1OUT, EXT},                     /* Tpm6 */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Flexspi1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Can1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Can2 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpuart1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpuart2 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpuart3 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpuart4 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpuart5 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpuart6 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpuart7 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpuart8 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpi2c1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpi2c2 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpi2c3 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpi2c4 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpi2c5 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpi2c6 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpi2c7 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpi2c8 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpspi1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpspi2 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpspi3 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpspi4 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpspi5 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpspi6 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpspi7 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Lpspi8 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* I3c1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* I3c2 */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Usdhc1 */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Usdhc2 */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Usdhc3 */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, EXT},                    /* Sai1 */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, EXT},                    /* Sai2 */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, EXT},                    /* Sai3 */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, AUDIO_PLL1OUT},            /* Ccm Cko1 */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, VIDEO_PLL1OUT},            /* Ccm Cko2 */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, AUDIO_PLL1OUT},            /* Ccm Cko3 */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, VIDEO_PLL1OUT},            /* Ccm Cko4 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Hsio */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Hsio Usb Test 60M */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Hsio Acscan 80M */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, SYS_PLL1PFD2},           /* Hsio Acscan 480M */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Nic */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Nic Apb */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Ml Apb */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Ml */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Media Axi */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Media Apb */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, SYS_PLL1PFD0},           /* Media Ldb */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, SYS_PLL1PFD0},           /* Media Disp Pix */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, SYS_PLL1PFD0},           /* Cam Pix */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, SYS_PLL1PFD0},           /* Mipi Test Byte */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, SYS_PLL1PFD0},           /* Mipi Phy Cfg */
  {OSC_24M, SYS_PLL1PFD0, SYS_PLL1PFD1, SYS_PLL1PFD2},             /* Dram Alt */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, SYS_PLL1PFD2DIV2}, /* Dram Apb */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Adc */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, EXT},                    /* Pdm */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Tstmr1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Tstmr2 */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, EXT},                    /* Mqs1 */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, EXT},                    /* Mqs2 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, SYS_PLL1PFD2DIV2}, /* Audio XCVR */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, EXT},                    /* Spdif */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, SYS_PLL1PFD2DIV2}, /* Enet */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Enet Timer1 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Enet Timer2 */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, SYS_PLL1PFD2DIV2}, /* Enet Ref */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Enet Ref Phy */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* I3c1 Slow */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* I3c2 Slow */
  {OSC_24M, SYS_PLL1PFD0DIV2, SYS_PLL1PFD1DIV2, VIDEO_PLL1OUT},    /* Usb Phy Burunin */
  {OSC_24M, AUDIO_PLL1OUT, VIDEO_PLL1OUT, SYS_PLL1PFD2},           /* Pal Came Scan */
};

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX93_IMX93_CCM_H */
