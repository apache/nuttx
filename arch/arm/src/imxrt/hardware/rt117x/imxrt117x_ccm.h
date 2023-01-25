/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_ccm.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_CCM_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define IMXRT_CCM_CR_CTRL_OFFSET(n)         (0x0000 + ((n) << 7)) /* Clock root control (CLOCK_ROOTn_CONTROL, n=0..78) */
#define IMXRT_CCM_CR_CTRL_SET_OFFSET(n)     (0x0004 + ((n) << 7)) /* Clock root control (CLOCK_ROOTn_CONTROL_SET, n=0..78) */
#define IMXRT_CCM_CR_CTRL_CLR_OFFSET(n)     (0x0008 + ((n) << 7)) /* Clock root control (CLOCK_ROOTn_CONTROL_CLR, n=0..78) */
#define IMXRT_CCM_CR_CTRL_TOG_OFFSET(n)     (0x000c + ((n) << 7)) /* Clock root control (CLOCK_ROOTn_CONTROL_TOG, n=0..78) */
#define IMXRT_CCM_CR_STAT0_OFFSET(n)        (0x0020 + ((n) << 7)) /* Clock root working status (CLOCK_ROOTn_STATUS0, n=0..78) */
#define IMXRT_CCM_CR_STAT1_OFFSET(n)        (0x0024 + ((n) << 7)) /* Clock root low power status (CLOCK_ROOTn_STATUS1, n=0..78) */
#define IMXRT_CCM_CR_CFG_OFFSET(n)          (0x002c + ((n) << 7)) /* Clock root configuration (CLOCK_ROOTn_CONFIG, n=0..78) */
#define IMXRT_CCM_CR_AUTH_OFFSET(n)         (0x0030 + ((n) << 7)) /* Clock root access control (CLOCK_ROOTn_AUTHEN, n=0..78) */
#define IMXRT_CCM_CR_AUTH_SET_OFFSET(n)     (0x0034 + ((n) << 7)) /* Clock root access control (CLOCK_ROOTn_AUTHEN_SET, n=0..78) */
#define IMXRT_CCM_CR_AUTH_CLR_OFFSET(n)     (0x0038 + ((n) << 7)) /* Clock root access control (CLOCK_ROOTn_AUTHEN_CLR, n=0..78) */
#define IMXRT_CCM_CR_AUTH_TOG_OFFSET(n)     (0x003c + ((n) << 7)) /* Clock root access control (CLOCK_ROOTn_AUTHEN_TOG, n=0..78) */
#define IMXRT_CCM_CR_COUNT                  79                    /* Count of clock roots */

#define IMXRT_CCM_CR_SETP_OFFSET(n,m)       (0x0040 + ((n) << 7) + ((m) << 2)) /* Setpoint setting (CLOCK_ROOTn_SETPOINTm, n=0..78, m=0..15) */

#define IMXRT_CCM_CG_CTRL_OFFSET(n)         (0x4000 + ((n) << 7)) /* Clock group control (CLOCK_GROUPn_CONTROL, n=0..1) */
#define IMXRT_CCM_CG_CTRL_SET_OFFSET(n)     (0x4004 + ((n) << 7)) /* Clock group control (CLOCK_GROUPn_CONTROL_SET, n=0..1) */
#define IMXRT_CCM_CG_CTRL_CLR_OFFSET(n)     (0x4008 + ((n) << 7)) /* Clock group control (CLOCK_GROUPn_CONTROL_CLR, n=0..1) */
#define IMXRT_CCM_CG_CTRL_TOG_OFFSET(n)     (0x400c + ((n) << 7)) /* Clock group control (CLOCK_GROUPn_CONTROL_TOG, n=0..1) */
#define IMXRT_CCM_CG_STAT0_OFFSET(n)        (0x4020 + ((n) << 7)) /* Clock group working status (CLOCK_GROUPn_STATUS0, n=0..1) */
#define IMXRT_CCM_CG_STAT1_OFFSET(n)        (0x4024 + ((n) << 7)) /* Clock group low power/extended status (CLOCK_GROUPn_STATUS1, n=0..1) */
#define IMXRT_CCM_CG_CFG_OFFSET(n)          (0x402c + ((n) << 7)) /* Clock group configuration (CLOCK_GROUPn_CONFIG, n=0..1) */
#define IMXRT_CCM_CG_AUTH_OFFSET(n)         (0x4030 + ((n) << 7)) /* Clock group access control (CLOCK_GROUPn_AUTHEN, n=0..1) */
#define IMXRT_CCM_CG_AUTH_SET_OFFSET(n)     (0x4034 + ((n) << 7)) /* Clock group access control (CLOCK_GROUPn_AUTHEN_SET, n=0..1) */
#define IMXRT_CCM_CG_AUTH_CLR_OFFSET(n)     (0x4038 + ((n) << 7)) /* Clock group access control (CLOCK_GROUPn_AUTHEN_CLR, n=0..1) */
#define IMXRT_CCM_CG_AUTH_TOG_OFFSET(n)     (0x403c + ((n) << 7)) /* Clock group access control (CLOCK_GROUPn_AUTHEN_TOG, n=0..1) */

#define IMXRT_CCM_CG_SETP_OFFSET(n,m)       (0x4040 + ((n) << 7) + ((m) << 2)) /* Setpoint setting (CLOCK_GROUPn_SETPOINTm, n=0..1, m=0..15) */

#define IMXRT_CCM_GPR_SH_OFFSET(n)          (0x4800 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn, n=0..7) */
#define IMXRT_CCM_GPR_SH_SET_OFFSET(n)      (0x4804 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_SET, n=0..7) */
#define IMXRT_CCM_GPR_SH_CLR_OFFSET(n)      (0x4808 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_CLR, n=0..7) */
#define IMXRT_CCM_GPR_SH_TOG_OFFSET(n)      (0x480c + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_TOG, n=0..7) */
#define IMXRT_CCM_GPR_SH_AUTH_OFFSET(n)     (0x4810 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_AUTHEN, n=0..7) */
#define IMXRT_CCM_GPR_SH_AUTH_SET_OFFSET(n) (0x4814 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_AUTHEN_SET, n=0..7) */
#define IMXRT_CCM_GPR_SH_AUTH_CLR_OFFSET(n) (0x4818 + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_AUTHEN_CLR, n=0..7) */
#define IMXRT_CCM_GPR_SH_AUTH_TOG_OFFSET(n) (0x481c + ((n) << 5)) /* General Purpose Register (GPR_SHAREDn_AUTHEN_TOG, n=0..7) */

#define IMXRT_CCM_GPR_PR_OFFSET(n)          (0x4c20 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn, n=1..7) */
#define IMXRT_CCM_GPR_PR_SET_OFFSET(n)      (0x4c24 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_SET, n=1..7) */
#define IMXRT_CCM_GPR_PR_CLR_OFFSET(n)      (0x4c28 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_CLR, n=1..7) */
#define IMXRT_CCM_GPR_PR_TOG_OFFSET(n)      (0x4c2c + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_TOG, n=1..7) */
#define IMXRT_CCM_GPR_PR_AUTH_OFFSET(n)     (0x4c30 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_AUTHEN, n=1..7) */
#define IMXRT_CCM_GPR_PR_AUTH_SET_OFFSET(n) (0x4c34 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_AUTHEN_SET, n=1..7) */
#define IMXRT_CCM_GPR_PR_AUTH_CLR_OFFSET(n) (0x4c38 + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_AUTHEN_CLR, n=1..7) */
#define IMXRT_CCM_GPR_PR_AUTH_TOG_OFFSET(n) (0x4c3c + (((n)-1) << 5)) /* General Purpose Register (GPR_PRIVATEn_AUTHEN_TOG, n=1..7) */

#define IMXRT_CCM_OSCPLL_DIR_OFFSET(n)      (0x5000 + ((n) << 5)) /* Clock source direct control (OSCPLLn_DIRECT, n=0..28) */
#define IMXRT_CCM_OSCPLL_DOM_OFFSET(n)      (0x5004 + ((n) << 5)) /* Clock source domain control (OSCPLLn_DOMAIN, n=0..28) */
#define IMXRT_CCM_OSCPLL_SETP_OFFSET(n)     (0x5008 + ((n) << 5)) /* Clock source setpoint setting (OSCPLLn_SETPOINT, n=0..28) */
#define IMXRT_CCM_OSCPLL_STAT0_OFFSET(n)    (0x5010 + ((n) << 5)) /* Clock source working status (OSCPLLn_STATUS0, n=0..28) */
#define IMXRT_CCM_OSCPLL_STAT1_OFFSET(n)    (0x5014 + ((n) << 5)) /* Clock source low power status (OSCPLLn_STATUS1, n=0..28) */
#define IMXRT_CCM_OSCPLL_CFG_OFFSET(n)      (0x5018 + ((n) << 5)) /* Clock source configuration (OSCPLLn_CONFIG, n=0..28) */
#define IMXRT_CCM_OSCPLL_AUTH_OFFSET(n)     (0x501c + ((n) << 5)) /* Clock source access control (OSCPLLn_AUTHEN, n=0..28) */

#define IMXRT_CCM_LPCG_DIR_OFFSET(n)        (0x6000 + ((n) << 5)) /* LPCG direct control (LPCGn_DIRECT, n=0..137) */
#define IMXRT_CCM_LPCG_DOM_OFFSET(n)        (0x6004 + ((n) << 5)) /* LPCG domain control (LPCGn_DOMAIN, n=0..137) */
#define IMXRT_CCM_LPCG_STAT0_OFFSET(n)      (0x6010 + ((n) << 5)) /* LPCG working status (LPCGn_STATUS0, n=0..137) */
#define IMXRT_CCM_LPCG_STAT1_OFFSET(n)      (0x6014 + ((n) << 5)) /* LPCG low power status (LPCGn_STATUS1, n=0..137) */
#define IMXRT_CCM_LPCG_CFG_OFFSET(n)        (0x6018 + ((n) << 5)) /* LPCG configuration (LPCGn_CONFIG, n=0..137) */
#define IMXRT_CCM_LPCG_AUTH_OFFSET(n)       (0x601c + ((n) << 5)) /* LPCG access control (LPCGn_AUTHEN, n=0..137) */

#define IMXRT_CCM_LPCG_SETP_OFFSET(n)       (0x6048 + (((n)-2) << 5)) /* LPCG setpoint setting (LPCGn_SETPOINT, n=2..48) */

/* Register addresses *******************************************************/

#define IMXRT_CCM_CR_CTRL(n)                (IMXRT_CCM_BASE + IMXRT_CCM_CR_CTRL_OFFSET(n))
#define IMXRT_CCM_CR_CTRL_SET(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CR_CTRL_SET_OFFSET(n))
#define IMXRT_CCM_CR_CTRL_CLR(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CR_CTRL_CLR_OFFSET(n))
#define IMXRT_CCM_CR_CTRL_TOG(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CR_CTRL_TOG_OFFSET(n))
#define IMXRT_CCM_CR_STAT0(n)               (IMXRT_CCM_BASE + IMXRT_CCM_CR_STAT0_OFFSET(n))
#define IMXRT_CCM_CR_STAT1(n)               (IMXRT_CCM_BASE + IMXRT_CCM_CR_STAT1_OFFSET(n))
#define IMXRT_CCM_CR_CFG(n)                 (IMXRT_CCM_BASE + IMXRT_CCM_CR_CFG_OFFSET(n))
#define IMXRT_CCM_CR_AUTH(n)                (IMXRT_CCM_BASE + IMXRT_CCM_CR_AUTH_OFFSET(n))
#define IMXRT_CCM_CR_AUTH_SET(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CR_AUTH_SET_OFFSET(n))
#define IMXRT_CCM_CR_AUTH_CLR(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CR_AUTH_CLR_OFFSET(n))
#define IMXRT_CCM_CR_AUTH_TOG(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CR_AUTH_TOG_OFFSET(n))
#define IMXRT_CCM_CR_SETP(n,m)              (IMXRT_CCM_BASE + IMXRT_CCM_CR_SETP_OFFSET(n,m))

#define IMXRT_CCM_CG_CTRL(n)                (IMXRT_CCM_BASE + IMXRT_CCM_CG_CTRL_OFFSET(n))
#define IMXRT_CCM_CG_CTRL_SET(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CG_CTRL_SET_OFFSET(n))
#define IMXRT_CCM_CG_CTRL_CLR(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CG_CTRL_CLR_OFFSET(n))
#define IMXRT_CCM_CG_CTRL_TOG(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CG_CTRL_TOG_OFFSET(n))
#define IMXRT_CCM_CG_STAT0(n)               (IMXRT_CCM_BASE + IMXRT_CCM_CG_STAT0_OFFSET(n))
#define IMXRT_CCM_CG_STAT1(n)               (IMXRT_CCM_BASE + IMXRT_CCM_CG_STAT1_OFFSET(n))
#define IMXRT_CCM_CG_CFG(n)                 (IMXRT_CCM_BASE + IMXRT_CCM_CG_CFG_OFFSET(n))
#define IMXRT_CCM_CG_AUTH(n)                (IMXRT_CCM_BASE + IMXRT_CCM_CG_AUTH_OFFSET(n))
#define IMXRT_CCM_CG_AUTH_SET(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CG_AUTH_SET_OFFSET(n))
#define IMXRT_CCM_CG_AUTH_CLR(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CG_AUTH_CLR_OFFSET(n))
#define IMXRT_CCM_CG_AUTH_TOG(n)            (IMXRT_CCM_BASE + IMXRT_CCM_CG_AUTH_TOG_OFFSET(n))
#define IMXRT_CCM_CG_SETP(n,m)              (IMXRT_CCM_BASE + IMXRT_CCM_CG_SETP_OFFSET(n,m))

#define IMXRT_CCM_GPR_SH(n)                 (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SH_OFFSET(n))
#define IMXRT_CCM_GPR_SH_SET(n)             (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SH_SET_OFFSET(n))
#define IMXRT_CCM_GPR_SH_CLR(n)             (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SH_CLR_OFFSET(n))
#define IMXRT_CCM_GPR_SH_TOG(n)             (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SH_TOG_OFFSET(n))
#define IMXRT_CCM_GPR_SH_AUTH(n)            (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SH_AUTH_OFFSET(n))
#define IMXRT_CCM_GPR_SH_AUTH_SET(n)        (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SH_AUTH_SET_OFFSET(n))
#define IMXRT_CCM_GPR_SH_AUTH_CLR(n)        (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SH_AUTH_CLR_OFFSET(n))
#define IMXRT_CCM_GPR_SH_AUTH_TOG(n)        (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SH_AUTH_TOG_OFFSET(n))

#define IMXRT_CCM_GPR_PR(n)                 (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PR_OFFSET(n))
#define IMXRT_CCM_GPR_PR_SET(n)             (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PR_SET_OFFSET(n))
#define IMXRT_CCM_GPR_PR_CLR(n)             (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PR_CLR_OFFSET(n))
#define IMXRT_CCM_GPR_PR_TOG(n)             (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PR_TOG_OFFSET(n))
#define IMXRT_CCM_GPR_PR_AUTH(n)            (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PR_AUTH_OFFSET(n))
#define IMXRT_CCM_GPR_PR_AUTH_SET(n)        (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PR_AUTH_SET_OFFSET(n))
#define IMXRT_CCM_GPR_PR_AUTH_CLR(n)        (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PR_AUTH_CLR_OFFSET(n))
#define IMXRT_CCM_GPR_PR_AUTH_TOG(n)        (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PR_AUTH_TOG_OFFSET(n))

#define IMXRT_CCM_OSCPLL_DIR(n)             (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_DIR_OFFSET(n))
#define IMXRT_CCM_OSCPLL_DOM(n)             (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_DOM_OFFSET(n))
#define IMXRT_CCM_OSCPLL_SETP(n)            (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_SETP_OFFSET(n))
#define IMXRT_CCM_OSCPLL_STAT0(n)           (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_STAT0_OFFSET(n))
#define IMXRT_CCM_OSCPLL_STAT1(n)           (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_STAT1_OFFSET(n))
#define IMXRT_CCM_OSCPLL_CFG(n)             (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_CFG_OFFSET(n))
#define IMXRT_CCM_OSCPLL_AUTH(n)            (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_AUTH_OFFSET(n))

#define IMXRT_CCM_LPCG_DIR(n)               (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_DIR_OFFSET(n))
#define IMXRT_CCM_LPCG_DOM(n)               (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_DOM_OFFSET(n))
#define IMXRT_CCM_LPCG_STAT0(n)             (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_STAT0_OFFSET(n))
#define IMXRT_CCM_LPCG_STAT1(n)             (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_STAT1_OFFSET(n))
#define IMXRT_CCM_LPCG_CFG(n)               (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_CFG_OFFSET(n))
#define IMXRT_CCM_LPCG_AUTH(n)              (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_AUTH_OFFSET(n))
#define IMXRT_CCM_LPCG_SETP(n)              (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_SETP_OFFSET(n))

/* Register bit definitions *************************************************/

/* Clock root control (CLOCK_ROOTn_CONTROL, n=0..78) */

#define CCM_CR_CTRL_DIV_SHIFT             (0)       /* Bits 0-7:   Divide selected clock by DIV+1 (DIV) */
#define CCM_CR_CTRL_DIV_MASK              (0xff << CCM_CR_CTRL_DIV_SHIFT)
#  define CCM_CR_CTRL_DIV(n)              (((n)-1) << CCM_CR_CTRL_DIV_SHIFT) /* Divide selected clock by n */

#define CCM_CR_CTRL_MUX_SHIFT             (8)       /* Bits 8-10:  Select clock from 8 clock sources (MUX) */
#define CCM_CR_CTRL_MUX_MASK              (0x07 << CCM_CR_CTRL_MUX_SHIFT)
#  define CCM_CR_CTRL_MUX_SRCSEL(n)       ((n) << CCM_CR_CTRL_MUX_SHIFT) /* Select clock source n */

                                                    /* Bits 11-23: Reserved */
#define CCM_CR_CTRL_OFF                   (1 << 24) /* Bit 24:     Shutdown clock root (OFF) */
                                                    /* Bits 25-31: Reserved */

/* Clock root working status (CLOCK_ROOTn_STATUS0, n=0..78) */

#define CCM_CR_STAT0_DIV_SHIFT            (0)       /* Bits 0-7:   Current clock root DIV setting (DIV) */
#define CCM_CR_STAT0_DIV_MASK             (0xff << CCM_CR_STAT0_DIV_SHIFT)
#define CCM_CR_STAT0_MUX_SHIFT            (8)       /* Bits 8-10:  Current clock root MUX setting (MUX) */
#define CCM_CR_STAT0_MUX_MASK             (0x07 << CCM_CR_STAT0_MUX_SHIFT)
                                                    /* Bits 11-23: Reserved */
#define CCM_CR_STAT0_OFF                  (1 << 24) /* Bit 24:     Current clock root OFF setting (OFF) */
                                                    /* Bits 25-26: Reserved */
#define CCM_CR_STAT0_POWERDOWN            (1 << 27) /* Bit 27:     Current clock root POWERDOWN setting (POWERDOWN) */
#define CCM_CR_STAT0_SLICE_BUSY           (1 << 28) /* Bit 28:     Clock generation logic is applying the new setting (SLICE_BUSY) */
#define CCM_CR_STAT0_UPDATE_FORWARD       (1 << 29) /* Bit 29:     Status synchronization from clock generation logic is in progress (UPDATE_FORWARD) */
#define CCM_CR_STAT0_UPDATE_REVERSE       (1 << 30) /* Bit 30:     Status synchronization to clock generation logic is in progress (UPDATE_REVERSE) */
#define CCM_CR_STAT0_CHANGING             (1 << 31) /* Bit 31:     Clock generation logic is updating currently (CHANGING) */

/* Clock root low power status (CLOCK_ROOTn_STATUS1, n=0..78) */

                                                    /* Bits 0-15:  Reserved */
#define CCM_CR_STAT1_TARGET_SETPOINT_SHIFT  (16)    /* Bits 16-19: Setpoint value to which SoC will switch (TARGET_SETPOINT) */
#define CCM_CR_STAT1_TARGET_SETPOINT_MASK   (0x0f << CCM_CR_STAT1_TARGET_SETPOINT_SHIFT)
#define CCM_CR_STAT1_CURRENT_SETPOINT_SHIFT (20)    /* Bits 20-23: Setpoint value in which SoC is currently working (CURRENT_SETPOINT) */
#define CCM_CR_STAT1_CURRENT_SETPOINT_MASK  (0x0f << CCM_CR_STAT1_CURRENT_SETPOINT_SHIFT)

#define CCM_CR_STAT1_DOWN_REQUEST         (1 << 24) /* Bit 24:     Clock frequency decrease requested (DOWN_REQUEST) */
#define CCM_CR_STAT1_DOWN_DONE            (1 << 25) /* Bit 25:     Clock frequency decrease completed (DOWN_DONE) */
#define CCM_CR_STAT1_UP_REQUEST           (1 << 26) /* Bit 26:     Clock frequency increase requested (UP_REQUEST) */
#define CCM_CR_STAT1_UP_DONE              (1 << 27) /* Bit 27:     Clock frequency increase completed (UP_DONE) */
                                                    /* Bits 28-31: Reserved */

/* Clock root configuration (CLOCK_ROOTn_CONFIG, n=0..78) */

                                                    /* Bits 0-3:   Reserved */
#define CCM_CR_CFG_SETPOINT_PRESENT       (1 << 4)  /* Bit 4:      Setpoint is implemented (SETPOINT_PRESENT) */
                                                    /* Bits 5-31:  Reserved */

/* Clock root access control (CLOCK_ROOTn_AUTHEN, n=0..78) */

#define CCM_CR_AUTH_TZ_USER               (1 << 0)  /* Bit 0:      Clock root can be changed in user mode (TZ_USER) */
#define CCM_CR_AUTH_TZ_NS                 (1 << 1)  /* Bit 1:      Clock root can be changed in non-secure mode (TZ_NS) */
                                                    /* Bits 2-3:   Reserved */
#define CCM_CR_AUTH_LOCK_TZ               (1 << 4)  /* Bit 4:      Lock TrustZone settings (LOCK_TZ) */
                                                    /* Bits 5-7:   Reserved */
#define CCM_CR_AUTH_WHITE_LIST_SHIFT      (8)       /* Bits 8-11:  Allow domains to change clock (WHITE_LIST) */
#define CCM_CR_AUTH_WHITE_LIST_MASK       (0x0f << CCM_CR_AUTH_WHITE_LIST_SHIFT)
#  define CCM_CR_AUTH_WHITE_LIST_DOMAIN0  (0x01 << CCM_CR_AUTH_WHITE_LIST_SHIFT) /* Allow domain 0 to change clock */
#  define CCM_CR_AUTH_WHITE_LIST_DOMAIN1  (0x02 << CCM_CR_AUTH_WHITE_LIST_SHIFT) /* Allow domain 1 to change clock */
#  define CCM_CR_AUTH_WHITE_LIST_DOMAIN2  (0x04 << CCM_CR_AUTH_WHITE_LIST_SHIFT) /* Allow domain 2 to change clock */
#  define CCM_CR_AUTH_WHITE_LIST_DOMAIN3  (0x08 << CCM_CR_AUTH_WHITE_LIST_SHIFT) /* Allow domain 3 to change clock */

#define CCM_CR_AUTH_LOCK_LIST             (1 << 12) /* Bit 12:     Lock whitelist settings (LOCK_LIST) */
                                                    /* Bits 13-15: Reserved */
#define CCM_CR_AUTH_DOMAIN_MODE           (1 << 16) /* Bit 16:     Low power and access control by domain (DOMAIN_MODE) */
#define CCM_CR_AUTH_SETPOINT_MODE         (1 << 17) /* Bit 17:     Low power and access control by setpoint (SETPOINT_MODE) */
                                                    /* Bits 18-19: Reserved */
#define CCM_CR_AUTH_LOCK_MODE             (1 << 20) /* Bit 20:     Lock low power and access mode (LOCK_MODE) */
                                                    /* Bits 21-31: Reserved */

/* Setpoint setting (CLOCK_ROOTn_SETPOINTm, m=0..15, n=0..78) */

#define CCM_CR_SETP_DIV_SHIFT             (0)       /* Bits 0-7:   Clock divider value in setpoint (DIV) */
#define CCM_CR_SETP_DIV_MASK              (0xff << CCM_CR_CTRL_DIV_SHIFT)
#define CCM_CR_SETP_MUX_SHIFT             (8)       /* Bits 8-10:  Clock multiplexer value in setpoint (MUX) */
#define CCM_CR_SETP_MUX_MASK              (0x07 << CCM_CR_CTRL_MUX_SHIFT)
                                                    /* Bits 11-23: Reserved */
#define CCM_CR_SETP_OFF                   (1 << 24) /* Bit 24:     OFF value in setpoint (OFF) */
                                                    /* Bits 25-27: Reserved */
#define CCM_CR_SETP_GRADE_SHIFT           (28)      /* Bits 28-31: Indicate speed grade for each setpoint setting (GRADE) */
#define CCM_CR_SETP_GRADE_MASK            (0x0f << CCM_CR_SETP_GRADE_SHIFT)

/* Clock group control (CLOCK_GROUPn_CONTROL, n=0..1) */

#define CCM_CG_CTRL_DIV0_SHIFT            (0)       /* Bits 0-3:   Divide clock root by DIV+1 (DIV0) */
#define CCM_CG_CTRL_DIV0_MASK             (0x0f << CCM_CG_CTRL_DIV0_SHIFT)
#  define CCM_CG_CTRL_DIV0(n)             (((n)-1) << CCM_CG_CTRL_DIV0_SHIFT) /* Divide selected clock by n */

                                                    /* Bits 4-15:  Reserved */
#define CCM_CG_CTRL_RSTDIV_SHIFT          (16)      /* Bits 16-23: Clock group global restart count (RSTDIV) */
#define CCM_CG_CTRL_RSTDIV_MASK           (0xff << CCM_CG_CTRL_RSTDIV_SHIFT)
#  define CCM_CG_CTRL_RSTDIV(n)           (((n)-1) << CCM_CG_CTRL_RSTDIV_SHIFT) /* Divide selected clock by n */
#define CCM_CG_CTRL_OFF                   (1 << 24)  
                                                    /* Bit 24: Shutdown all clocks in clock group (OFF) */
                                                    /* Bits 25-31: Reserved */

/* Clock group working status (CLOCK_GROUPn_STATUS0, n=0..1) */

#define CCM_CG_STAT0_DIV0_SHIFT           (0)       /* Bits 0-3:   Current clock group DIV0 setting (DIV0) */
#define CCM_CG_STAT0_DIV0_MASK            (0x0f << CCM_CG_STAT0_DIV0_SHIFT)
                                                    /* Bits 4-15:  Reserved */
#define CCM_CG_STAT0_RSTDIV_SHIFT         (16)      /* Bits 16-23: Current clock group RSTDIV setting (RSTDIV) */
#define CCM_CG_STAT0_RSTDIV_MASK          (0xff << CCM_CG_STAT0_RSTDIV_SHIFT)
#define CCM_CG_STAT0_OFF                  (1 << 24) /* Bit 24:     Current clock group OFF setting (OFF) */
                                                    /* Bits 25-26: Reserved */
#define CCM_CG_STAT0_POWERDOWN            (1 << 27) /* Bit 27:     Current clock group POWERDOWN setting (POWERDOWN) */
#define CCM_CG_STAT0_SLICE_BUSY           (1 << 28) /* Bit 28:     Clock generation logic is applying the new setting (SLICE_BUSY) */
#define CCM_CG_STAT0_UPDATE_FORWARD       (1 << 29) /* Bit 29:     Status synchronization from clock generation logic is in progress (UPDATE_FORWARD) */
#define CCM_CG_STAT0_UPDATE_REVERSE       (1 << 30) /* Bit 30:     Status synchronization to clock generation logic is in progress (UPDATE_REVERSE) */
#define CCM_CG_STAT0_CHANGING             (1 << 31) /* Bit 31:     Clock generation logic is updating currently (CHANGING) */

/* Clock group low power/extended status (CLOCK_GROUPn_STATUS1, n=0..1) */

                                                    /* Bits 0-15:  Reserved */
#define CCM_CG_STAT1_TARGET_SETPOINT_SHIFT  (16)    /* Bits 16-19: Setpoint value to which SoC will switch (TARGET_SETPOINT) */
#define CCM_CG_STAT1_TARGET_SETPOINT_MASK   (0x0f << CCM_CG_STAT1_TARGET_SETPOINT_SHIFT)
#define CCM_CG_STAT1_CURRENT_SETPOINT_SHIFT (20)    /* Bits 20-23:  Setpoint value in which SoC is currently working (CURRENT_SETPOINT) */
#define CCM_CG_STAT1_CURRENT_SETPOINT_MASK  (0x0f << CCM_CG_STAT1_CURRENT_SETPOINT_SHIFT)

#define CCM_CG_STAT1_DOWN_REQUEST         (1 << 24) /* Bit 24:     Clock frequency decrease requested (DOWN_REQUEST) */
#define CCM_CG_STAT1_DOWN_DONE            (1 << 25) /* Bit 25:     Clock frequency decrease completed (DOWN_DONE) */
#define CCM_CG_STAT1_UP_REQUEST           (1 << 26) /* Bit 26:     Clock frequency increase requested (UP_REQUEST) */
#define CCM_CG_STAT1_UP_DONE              (1 << 27) /* Bit 27:     Clock frequency increase completed (UP_DONE) */
                                                    /* Bits 28-31: Reserved */

/* Clock group configuration (CLOCK_GROUPn_CONFIG, n=0..1) */

                                                    /* Bits 0-3:   Reserved */
#define CCM_CG_CFG_SETPOINT_PRESENT       (1 << 4)  /* Bit 4:      Setpoint is implemented (SETPOINT_PRESENT) */
                                                    /* Bits 5-31:  Reserved */

/* Clock group access control (CLOCK_GROUPn_AUTHEN, n=0..1) */

#define CCM_CG_AUTH_TZ_USER               (1 << 0)  /* Bit 0:      Clock group can be changed in user mode (TZ_USER) */
#define CCM_CG_AUTH_TZ_NS                 (1 << 1)  /* Bit 1:      Clock group can be changed in non-secure mode (TZ_NS) */
                                                    /* Bits 2-3:   Reserved */
#define CCM_CG_AUTH_LOCK_TZ               (1 << 4)  /* Bit 4:      Lock TrustZone settings (LOCK_TZ) */
                                                    /* Bits 5-7:   Reserved */
#define CCM_CG_AUTH_WHITE_LIST_SHIFT      (8)       /* Bits 8-11:  Allow domains to change clock (WHITE_LIST) */
#define CCM_CG_AUTH_WHITE_LIST_MASK       (0x0f << CCM_CG_AUTH_WHITE_LIST_SHIFT)
#  define CCM_CG_AUTH_WHITE_LIST_DOMAIN0  (0x01 << CCM_CG_AUTH_WHITE_LIST_SHIFT) /* Allow domain 0 to change clock */
#  define CCM_CG_AUTH_WHITE_LIST_DOMAIN1  (0x02 << CCM_CG_AUTH_WHITE_LIST_SHIFT) /* Allow domain 1 to change clock */
#  define CCM_CG_AUTH_WHITE_LIST_DOMAIN2  (0x04 << CCM_CG_AUTH_WHITE_LIST_SHIFT) /* Allow domain 2 to change clock */
#  define CCM_CG_AUTH_WHITE_LIST_DOMAIN3  (0x08 << CCM_CG_AUTH_WHITE_LIST_SHIFT) /* Allow domain 3 to change clock */

#define CCM_CG_AUTH_LOCK_LIST             (1 << 12) /* Bit 12:     Lock whitelist setting (LOCK_LIST) */
                                                    /* Bits 13-15: Reserved */
#define CCM_CG_AUTH_DOMAIN_MODE           (1 << 16) /* Bit 16:     Low power and access control by domain (DOMAIN_MODE) */
#define CCM_CG_AUTH_SETPOINT_MODE         (1 << 17) /* Bit 17:     Low power and access control by setpoint (SETPOINT_MODE) */
                                                    /* Bits 18-19: Reserved */
#define CCM_CG_AUTH_LOCK_MODE             (1 << 20) /* Bit 20:     Lock low power and access mode (LOCK_MODE) */
                                                    /* Bits 21-31: Reserved */

/* Setpoint setting (CLOCK_GROUPn_SETPOINTm, m=0..15, n=0..1) */

#define CCM_CG_SETP_DIV0_SHIFT            (0)       /* Bits 0-3:   Clock divider value in setpoint (DIV0) */
#define CCM_CG_SETP_DIV0_MASK             (0x0f << CCM_CG_SETP_DIV0_SHIFT)
                                                    /* Bits 4-15:  Reserved */
#define CCM_CG_SETP_RSTDIV_SHIFT          (16)      /* Bits 16-23: Clock group global restart count (RSTDIV) */
#define CCM_CG_SETP_RSTDIV_MASK           (0xff << CCM_CG_SETP_RSTDIV_SHIFT)
#define CCM_CG_SETP_OFF                   (1 << 24) /* Bit 24:     Shutdown all locks in clock group (OFF) */
                                                    /* Bits 25-27: Reserved */
#define CCM_CG_SETP_GRADE_SHIFT           (28)      /* Bits 28-31: Indicate speed grade for each setpoint setting (GRADE) */
#define CCM_CG_SETP_GRADE_MASK            (0x0f << CCM_CG_SETP_GRADE_SHIFT)

/* General Purpose Register (GPR_SHAREDn, n=0..7) */

#define CCM_GPR_SH_GPR_SHIFT              (0)       /* Bits 0-31:  General purpose register, shared for all CPU domains (GPR) */
#define CCM_GPR_SH_GPR_MASK               (0xffffffff << CCM_GPR_SH_GPR_SHIFT)

/* General Purpose Register (GPR_SHAREDn_AUTHEN, n=0..7) */

#define CCM_GPR_SH_AUTH_TZ_USER           (1 << 0)  /* Bit 0:      Clock can be changed in user mode (TZ_USER) */
#define CCM_GPR_SH_AUTH_TZ_NS             (1 << 1)  /* Bit 1:      Clock can be changed in non-secure mode (TZ_NS) */
                                                    /* Bits 2-3:   Reserved */
#define CCM_GPR_SH_AUTH_LOCK_TZ           (1 << 4)  /* Bit 4:      Lock TrustZone settings (LOCK_TZ) */
                                                    /* Bits 5-7:   Reserved */

#define CCM_GPR_SH_AUTH_WHITE_LIST_SHIFT     (8)    /* Bits 8-11:  Allow domains to change clock (WHITE_LIST) */
#define CCM_GPR_SH_AUTH_WHITE_LIST_MASK      (0x0f << CCM_GPR_SH_AUTH_WHITE_LIST_SHIFT)
#  define CCM_GPR_SH_AUTH_WHITE_LIST_DOMAIN0 (0x01 << CCM_GPR_SH_AUTH_WHITE_LIST_SHIFT) /* Allow domain 0 to change clock */
#  define CCM_GPR_SH_AUTH_WHITE_LIST_DOMAIN1 (0x02 << CCM_GPR_SH_AUTH_WHITE_LIST_SHIFT) /* Allow domain 1 to change clock */
#  define CCM_GPR_SH_AUTH_WHITE_LIST_DOMAIN2 (0x04 << CCM_GPR_SH_AUTH_WHITE_LIST_SHIFT) /* Allow domain 2 to change clock */
#  define CCM_GPR_SH_AUTH_WHITE_LIST_DOMAIN3 (0x08 << CCM_GPR_SH_AUTH_WHITE_LIST_SHIFT) /* Allow domain 3 to change clock */

#define CCM_GPR_SH_AUTH_LOCK_LIST         (1 << 12) /* Bit 12:     Lock whitelist setting (LOCK_LIST) */
                                                    /* Bits 13-15: Reserved */
#define CCM_GPR_SH_AUTH_DOMAIN_MODE       (1 << 16) /* Bit 16:     Low power and access control by domain (DOMAIN_MODE) */
                                                    /* Bits 17-19: Reserved */
#define CCM_GPR_SH_AUTH_LOCK_MODE         (1 << 20) /* Bit 20:     Lock low power and access mode (LOCK_MODE) */
                                                    /* Bits 21-31: Reserved */

/* General Purpose Register (GPR_PRIVATEn, n=1..7) */

#define CCM_GPR_PR_GPR_SHIFT              (0)       /* Bits 0-31:  General purpose register, with dedicated bits for each domain (GPR) */
#define CCM_GPR_PR_GPR_MASK               (0xffffffff << CCM_GPR_PR_GPR_SHIFT)

/* General Purpose Register (GPR_PRIVATEn_AUTHEN, n=1..7) */

#define CCM_GPR_PR_AUTH_TZ_USER           (1 << 0)  /* Bit 0:      Clock can be changed in user mode (TZ_USER) */
#define CCM_GPR_PR_AUTH_TZ_NS             (1 << 1)  /* Bit 1:      Clock can be changed in non-secure mode (TZ_NS) */
                                                    /* Bits 2-3:   Reserved */
#define CCM_GPR_PR_AUTH_LOCK_TZ           (1 << 4)  /* Bit 4:      Lock TrustZone settings (LOCK_TZ) */
                                                    /* Bits 5-7:   Reserved */

#define CCM_GPR_PR_AUTH_WHITE_LIST_SHIFT     (8)    /* Bits 8-11:  Allow domains to change clock (WHITE_LIST) */
#define CCM_GPR_PR_AUTH_WHITE_LIST_MASK      (0x0f << CCM_GPR_PR_AUTH_WHITE_LIST_SHIFT)
#  define CCM_GPR_PR_AUTH_WHITE_LIST_DOMAIN0 (0x01 << CCM_GPR_PR_AUTH_WHITE_LIST_SHIFT) /* Allow domain 0 to change clock */
#  define CCM_GPR_PR_AUTH_WHITE_LIST_DOMAIN1 (0x02 << CCM_GPR_PR_AUTH_WHITE_LIST_SHIFT) /* Allow domain 1 to change clock */
#  define CCM_GPR_PR_AUTH_WHITE_LIST_DOMAIN2 (0x04 << CCM_GPR_PR_AUTH_WHITE_LIST_SHIFT) /* Allow domain 2 to change clock */
#  define CCM_GPR_PR_AUTH_WHITE_LIST_DOMAIN3 (0x08 << CCM_GPR_PR_AUTH_WHITE_LIST_SHIFT) /* Allow domain 3 to change clock */

#define CCM_GPR_PR_AUTH_LOCK_LIST         (1 << 12) /* Bit 12:     Lock whitelist setting (LOCK_LIST) */
                                                    /* Bits 13-15: Reserved */
#define CCM_GPR_PR_AUTH_DOMAIN_MODE       (1 << 16) /* Bit 16:     Low power and access control by domain (DOMAIN_MODE) */
                                                    /* Bits 17-19: Reserved */
#define CCM_GPR_PR_AUTH_LOCK_MODE         (1 << 20) /* Bit 20:     Lock low power and access mode (LOCK_MODE) */
                                                    /* Bits 21-31: Reserved */

/* Clock source direct control (OSCPLLn_DIRECT, n=0..28) */

#define CCM_OSCPLL_DIR_ON                 (1 << 0)  /* Bit 0:      Turn on clock source (ON) */
                                                    /* Bits 1-31:  Reserved */

/* Clock source domain control (OSCPLLn_DOMAIN, n=0..28) */

#define CCM_OSCPLL_DOM_LEVEL_SHIFT        (0)       /* Bits 0-2:   Current dependence level of this clock source for the current accessing domain (LEVEL) */
#define CCM_OSCPLL_DOM_LEVEL_MASK         (0x07 << CCM_OSCPLL_DOM_LEVEL_SHIFT)
                                                    /* Bits 3-15:  Reserved */
#define CCM_OSCPLL_DOM_LEVEL0_SHIFT       (16)      /* Bits 16-18: Dependence level of this clock source for DOMAIN0 (LEVEL0) */
#define CCM_OSCPLL_DOM_LEVEL0_MASK        (0x07 << CCM_OSCPLL_DOM_LEVEL0_SHIFT)
                                                    /* Bit 31:     Reserved */
#define CCM_OSCPLL_DOM_LEVEL1_SHIFT       (20)      /* Bits 20-22: Dependence level of this clock source for DOMAIN1 (LEVEL1) */
#define CCM_OSCPLL_DOM_LEVEL1_MASK        (0x07 << CCM_OSCPLL_DOM_LEVEL1_SHIFT)
                                                    /* Bit 25:     Reserved */
#define CCM_OSCPLL_DOM_LEVEL2_SHIFT       (24)      /* Bits 24-26: Dependence level of this clock source for DOMAIN2 (LEVEL2) */
#define CCM_OSCPLL_DOM_LEVEL2_MASK        (0x07 << CCM_OSCPLL_DOM_LEVEL2_SHIFT)
                                                    /* Bit 27:     Reserved */
#define CCM_OSCPLL_DOM_LEVEL3_SHIFT       (28)      /* Bits 28-30: Dependence level of this clock source for DOMAIN3 (LEVEL3) */
#define CCM_OSCPLL_DOM_LEVEL3_MASK        (0x07 << CCM_OSCPLL_DOM_LEVEL3_SHIFT)
                                                    /* Bit 31:     Reserved */

/* Clock source setpoint setting (OSCPLLn_SETPOINT, n=0..28) */

#define CCM_OSCPLL_SETP_SETPOINT_SHIFT    (0)       /* Bits 0-15:  Defines 16 setpoint values (SETPOINT) */
#define CCM_OSCPLL_SETP_SETPOINT_MASK     (0xffff << CCM_OSCPLL_SETP_SETPOINT_SHIFT)
#define CCM_OSCPLL_SETP_STANDBY_SHIFT     (16)      /* Bits 16-31: Defines 16 setpoint standby values (STANDBY) */
#define CCM_OSCPLL_SETP_STANDBY_MASK      (0xffff << CCM_OSCPLL_SETP_STANDBY_SHIFT)

/* Clock source working status (OSCPLLn_STATUS0, n=0..28) */

#define CCM_OSCPLL_STAT0_ON               (1 << 0)  /* Bit 0:      Clock source is turned on (ON) */
                                                    /* Bits 1-3:   Reserved */
#define CCM_OSCPLL_STAT0_STATUS_EARLY     (1 << 4)  /* Bit 4:      Clock source is active (STATUS_EARLY) */
#define CCM_OSCPLL_STAT0_STATUS_LATE      (1 << 5)  /* Bit 5:      Clock source is ready to use (STATUS_LATE) */
                                                    /* Bits 6-7:   Reserved */

#define CCM_OSCPLL_STAT0_ACTIVE_DOMAIN_SHIFT (8)    /* Bits 8-11:  Domains that own this clock source according to whitelist (ACTIVE_DOMAIN) */
#define CCM_OSCPLL_STAT0_ACTIVE_DOMAIN_MASK  (0x0f << CCM_OSCPLL_STAT0_ACTIVE_DOMAIN_SHIFT)
#define CCM_OSCPLL_STAT0_DOMAIN_ENABLE_SHIFT (8)    /* Bits 12-15: Enable status from each domain (DOMAIN_ENABLE) */
#define CCM_OSCPLL_STAT0_DOMAIN_ENABLE_MASK  (0x0f << CCM_OSCPLL_STAT0_DOMAIN_ENABLE_SHIFT)

                                                    /* Bits 16-27: Reserved */
#define CCM_OSCPLL_STAT0_IN_USE           (1 << 28) /* Bit 28:     Indicates whether the clock source is being used by active clock roots (IN_USE) */
                                                    /* Bits 29-31: Reserved */

/* Clock source low power status (OSCPLLn_STATUS1, n=0..28) */

#define CCM_OSCPLL_STAT1_CPU0_MODE_SHIFT  (0)       /* Bits 0-1:   Domain 0 Low Power Mode (CPU0_MODE) */
#define CCM_OSCPLL_STAT1_CPU0_MODE_MASK   (0x03 << CCM_OSCPLL_STAT1_CPU0_MODE_SHIFT)
#  define CCM_OSCPLL_STAT1_CPU0_MODE_RUN  (0x00 << CCM_OSCPLL_STAT1_CPU0_MODE_SHIFT) /* Run */
#  define CCM_OSCPLL_STAT1_CPU0_MODE_WAIT (0x01 << CCM_OSCPLL_STAT1_CPU0_MODE_SHIFT) /* Wait */
#  define CCM_OSCPLL_STAT1_CPU0_MODE_STOP (0x02 << CCM_OSCPLL_STAT1_CPU0_MODE_SHIFT) /* Stop */
#  define CCM_OSCPLL_STAT1_CPU0_MODE_SUSP (0x03 << CCM_OSCPLL_STAT1_CPU0_MODE_SHIFT) /* Suspend */

#define CCM_OSCPLL_STAT1_CPU0_MODE_REQUEST (1 << 2) /* Bit 2:      Domain 0 request to enter Low Power Mode (CPU0_MODE_REQUEST) */

#define CCM_OSCPLL_STAT1_CPU0_MODE_DONE   (1 << 3)  /* Bit 3:      Domain 0 Low Power Mode task done (CPU0_MODE_DONE) */
#define CCM_OSCPLL_STAT1_CPU1_MODE_SHIFT  (4)       /* Bits 4-5:   Domain 1 Low Power Mode (CPU1_MODE) */
#define CCM_OSCPLL_STAT1_CPU1_MODE_MASK   (0x03 << CCM_OSCPLL_STAT1_CPU1_MODE_SHIFT)
#  define CCM_OSCPLL_STAT1_CPU1_MODE_RUN  (0x00 << CCM_OSCPLL_STAT1_CPU1_MODE_SHIFT) /* Run */
#  define CCM_OSCPLL_STAT1_CPU1_MODE_WAIT (0x01 << CCM_OSCPLL_STAT1_CPU1_MODE_SHIFT) /* Wait */
#  define CCM_OSCPLL_STAT1_CPU1_MODE_STOP (0x02 << CCM_OSCPLL_STAT1_CPU1_MODE_SHIFT) /* Stop */
#  define CCM_OSCPLL_STAT1_CPU1_MODE_SUSP (0x03 << CCM_OSCPLL_STAT1_CPU1_MODE_SHIFT) /* Suspend */

#define CCM_OSCPLL_STAT1_CPU1_MODE_REQUEST (1 << 6) /* Bit 6:      Domain 1 request to enter Low Power Mode (CPU1_MODE_REQUEST) */

#define CCM_OSCPLL_STAT1_CPU1_MODE_DONE   (1 << 7)  /* Bit 7:      Domain 1 Low Power Mode task done (CPU1_MODE_DONE) */
#define CCM_OSCPLL_STAT1_CPU2_MODE_SHIFT  (8)       /* Bits 8-9:   Domain 2 Low Power Mode (CPU2_MODE) */
#define CCM_OSCPLL_STAT1_CPU2_MODE_MASK   (0x03 << CCM_OSCPLL_STAT1_CPU2_MODE_SHIFT)
#  define CCM_OSCPLL_STAT1_CPU2_MODE_RUN  (0x00 << CCM_OSCPLL_STAT1_CPU2_MODE_SHIFT) /* Run */
#  define CCM_OSCPLL_STAT1_CPU2_MODE_WAIT (0x01 << CCM_OSCPLL_STAT1_CPU2_MODE_SHIFT) /* Wait */
#  define CCM_OSCPLL_STAT1_CPU2_MODE_STOP (0x02 << CCM_OSCPLL_STAT1_CPU2_MODE_SHIFT) /* Stop */
#  define CCM_OSCPLL_STAT1_CPU2_MODE_SUSP (0x03 << CCM_OSCPLL_STAT1_CPU2_MODE_SHIFT) /* Suspend */

#define CCM_OSCPLL_STAT1_CPU2_MODE_REQUEST (1 << 10) /* Bit 10:    Domain 2 request to enter Low Power Mode (CPU2_MODE_REQUEST) */

#define CCM_OSCPLL_STAT1_CPU2_MODE_DONE   (1 << 11) /* Bit 11:     Domain 2 Low Power Mode task done (CPU2_MODE_DONE) */
#define CCM_OSCPLL_STAT1_CPU3_MODE_SHIFT  (12)      /* Bits 12-13: Domain 3 Low Power Mode (CPU3_MODE) */
#define CCM_OSCPLL_STAT1_CPU3_MODE_MASK   (0x03 << CCM_OSCPLL_STAT1_CPU3_MODE_SHIFT)
#  define CCM_OSCPLL_STAT1_CPU3_MODE_RUN  (0x00 << CCM_OSCPLL_STAT1_CPU3_MODE_SHIFT) /* Run */
#  define CCM_OSCPLL_STAT1_CPU3_MODE_WAIT (0x01 << CCM_OSCPLL_STAT1_CPU3_MODE_SHIFT) /* Wait */
#  define CCM_OSCPLL_STAT1_CPU3_MODE_STOP (0x02 << CCM_OSCPLL_STAT1_CPU3_MODE_SHIFT) /* Stop */
#  define CCM_OSCPLL_STAT1_CPU3_MODE_SUSP (0x03 << CCM_OSCPLL_STAT1_CPU3_MODE_SHIFT) /* Suspend */

#define CCM_OSCPLL_STAT1_CPU3_MODE_REQUEST (1 << 14) /* Bit 14:    Domain 3 request to enter Low Power Mode (CPU3_MODE_REQUEST) */

#define CCM_OSCPLL_STAT1_CPU3_MODE_DONE   (1 << 15) /* Bit 15:     Domain 3 Low Power Mode task done (CPU3_MODE_DONE) */

#define CCM_OSCPLL_STAT1_TARGET_SETPOINT_SHIFT  (16) /* Bits 16-19: Setpoint value to which SoC will switch (TARGET_SETPOINT) */
#define CCM_OSCPLL_STAT1_TARGET_SETPOINT_MASK   (0x0f << CCM_OSCPLL_STAT1_TARGET_SETPOINT_SHIFT)
#define CCM_OSCPLL_STAT1_CURRENT_SETPOINT_SHIFT (20) /* Bits 20-23: Setpoint value in which SoC is currently working (CURRENT_SETPOINT) */
#define CCM_OSCPLL_STAT1_CURRENT_SETPOINT_MASK  (0x0f << CCM_OSCPLL_STAT1_CURRENT_SETPOINT_SHIFT)

#define CCM_OSCPLL_STAT1_SETPOINT_OFF_REQUEST (1 << 24) /* Bit 24: Clock gate turn off request from GPC setpoint (SETPOINT_OFF_REQUEST) */
#define CCM_OSCPLL_STAT1_SETPOINT_OFF_DONE    (1 << 25) /* Bit 25: Clock gate turn off completed (SETPOINT_OFF_DONE) */
#define CCM_OSCPLL_STAT1_SETPOINT_ON_REQUEST  (1 << 26) /* Bit 26: Clock gate turn on request from GPC setpoint (SETPOINT_ON_REQUEST) */
#define CCM_OSCPLL_STAT1_SETPOINT_ON_DONE     (1 << 27) /* Bit 27: Clock gate turn on completed (SETPOINT_ON_DONE) */
#define CCM_OSCPLL_STAT1_STANDBY_IN_REQUEST   (1 << 28) /* Bit 28: Clock gate turn off request from GPC standby (STANDBY_IN_REQUEST) */
#define CCM_OSCPLL_STAT1_STANDBY_IN_DONE      (1 << 29) /* Bit 29: Clock gate turn off completed (STANDBY_IN_DONE) */
#define CCM_OSCPLL_STAT1_STANDBY_OUT_DONE     (1 << 30) /* Bit 30: Clock gate turn on completed (STANDBY_OUT_DONE) */
#define CCM_OSCPLL_STAT1_STANDBY_OUT_REQUEST  (1 << 31) /* Bit 31: Clock gate turn on request from GPC standby (STANDBY_OUT_REQUEST) */

/* Clock source configuration (OSCPLLn_CONFIG, n=0..28) */

                                                    /* Bit 0:      Reserved */
#define CCM_OSCPLL_CFG_AUTOMODE_PRESENT   (1 << 1)  /* Bit 1:      Automode is implemented (AUTOMODE_PRESENT) */
                                                    /* Bits 2-3:   Reserved */
#define CCM_OSCPLL_CFG_SETPOINT_PRESENT   (1 << 4)  /* Bit 4:      Setpoint is implemented (SETPOINT_PRESENT) */
                                                    /* Bits 5-31:  Reserved */

/* Clock source access control (OSCPLLn_AUTHEN, n=0..28) */

#define CCM_OSCPLL_AUTH_TZ_USER           (1 << 0)  /* Bit 0:      Clock source can be changed in user mode (TZ_USER) */
#define CCM_OSCPLL_AUTH_TZ_NS             (1 << 1)  /* Bit 1:      Clock source can be changed in non-secure mode (TZ_NS) */
                                                    /* Bits 2-3:   Reserved */
#define CCM_OSCPLL_AUTH_LOCK_TZ           (1 << 4)  /* Bit 4:      Lock TrustZone settings (LOCK_TZ) */
                                                    /* Bits 5-7:   Reserved */

#define CCM_OSCPLL_AUTH_WHITE_LIST_SHIFT     (8)    /* Bits 8-11:  Allow domains to change clock (WHITE_LIST) */
#define CCM_OSCPLL_AUTH_WHITE_LIST_MASK      (0x0f << CCM_OSCPLL_AUTH_WHITE_LIST_SHIFT)
#  define CCM_OSCPLL_AUTH_WHITE_LIST_DOMAIN0 (0x01 << CCM_OSCPLL_AUTH_WHITE_LIST_SHIFT) /* Allow domain 0 to change clock */
#  define CCM_OSCPLL_AUTH_WHITE_LIST_DOMAIN1 (0x02 << CCM_OSCPLL_AUTH_WHITE_LIST_SHIFT) /* Allow domain 1 to change clock */
#  define CCM_OSCPLL_AUTH_WHITE_LIST_DOMAIN2 (0x04 << CCM_OSCPLL_AUTH_WHITE_LIST_SHIFT) /* Allow domain 2 to change clock */
#  define CCM_OSCPLL_AUTH_WHITE_LIST_DOMAIN3 (0x08 << CCM_OSCPLL_AUTH_WHITE_LIST_SHIFT) /* Allow domain 3 to change clock */

#define CCM_OSCPLL_AUTH_LOCK_LIST         (1 << 12) /* Bit 12:     Lock whitelist settings (LOCK_LIST) */
                                                    /* Bits 13-15: Reserved */
#define CCM_OSCPLL_AUTH_DOMAIN_MODE       (1 << 16) /* Bit 16:     Low power and access control by domain (DOMAIN_MODE) */
#define CCM_OSCPLL_AUTH_SETPOINT_MODE     (1 << 17) /* Bit 17:     LPCG works in setpoint controlled mode (SETPOINT_MODE) */
#define CCM_OSCPLL_AUTH_CPULPM            (1 << 18) /* Bit 18:     CPU Low Power Mode (CPULPM) */
                                                    /* Bit 19:     Reserved */
#define CCM_OSCPLL_AUTH_LOCK_MODE         (1 << 20) /* Bit 20:     Lock low power and access mode (LOCK_MODE) */
                                                    /* Bits 21-31: Reserved */

/* LPCG direct control (LPCGn_DIRECT, n=0..137) */

#define CCM_LPCG_DIR_ON                   (1 << 0)  /* Bit 0:      LPCG on (ON) */
                                                    /* Bits 1-31:  Reserved */

/* LPCG domain control (LPCGn_DOMAIN, n=0..137) */

#define CCM_LPCG_DOM_LEVEL_SHIFT          (0)       /* Bits 0-2:   Current dependence level of this clock source for the current accessing domain (LEVEL) */
#define CCM_LPCG_DOM_LEVEL_MASK           (0x07 << CCM_LPCG_DOM_LEVEL_SHIFT)
                                                    /* Bits 3-15:  Reserved */
#define CCM_LPCG_DOM_LEVEL0_SHIFT         (16)      /* Bits 16-18: Dependence level of this LPCG for DOMAIN0 (LEVEL0) */
#define CCM_LPCG_DOM_LEVEL0_MASK          (0x07 << CCM_LPCG_DOM_LEVEL0_SHIFT)
                                                    /* Bit 31:     Reserved */
#define CCM_LPCG_DOM_LEVEL1_SHIFT         (20)      /* Bits 20-22: Dependence level of this LPCG for DOMAIN1 (LEVEL1) */
#define CCM_LPCG_DOM_LEVEL1_MASK          (0x07 << CCM_LPCG_DOM_LEVEL1_SHIFT)
                                                    /* Bit 25:     Reserved */
#define CCM_LPCG_DOM_LEVEL2_SHIFT         (24)      /* Bits 24-26: Dependence level of this LPCG for DOMAIN2 (LEVEL2) */
#define CCM_LPCG_DOM_LEVEL2_MASK          (0x07 << CCM_LPCG_DOM_LEVEL2_SHIFT)
                                                    /* Bit 27:     Reserved */
#define CCM_LPCG_DOM_LEVEL3_SHIFT         (28)      /* Bits 28-30: Dependence level of this LPCG for DOMAIN3 (LEVEL3) */
#define CCM_LPCG_DOM_LEVEL3_MASK          (0x07 << CCM_LPCG_DOM_LEVEL3_SHIFT)
                                                    /* Bit 31:     Reserved */

/* LPCG working status (LPCGn_STATUS0, n=0..137) */

#define CCM_LPCG_STAT0_ON                 (1 << 0)  /* Bit 0:      Clock source is turned on (ON) */
                                                    /* Bits 1-7:   Reserved */

#define CCM_LPCG_STAT0_ACTIVE_DOMAIN_SHIFT (8)      /* Bits 8-11:  Domains that own this clock source according to whitelist (ACTIVE_DOMAIN) */
#define CCM_LPCG_STAT0_ACTIVE_DOMAIN_MASK  (0x0f << CCM_LPCG_STAT0_ACTIVE_DOMAIN_SHIFT)
#define CCM_LPCG_STAT0_DOMAIN_ENABLE_SHIFT (8)      /* Bits 12-15: Enable status from each domain (DOMAIN_ENABLE) */
#define CCM_LPCG_STAT0_DOMAIN_ENABLE_MASK  (0x0f << CCM_LPCG_STAT0_DOMAIN_ENABLE_SHIFT)
                                                    /* Bits 16-31: Reserved */

/* LPCG low power status (LPCGn_STATUS1, n=0..137) */

#define CCM_LPCG_STAT1_CPU0_MODE_SHIFT    (0)       /* Bits 0-1:   Domain 0 Low Power Mode (CPU0_MODE) */
#define CCM_LPCG_STAT1_CPU0_MODE_MASK     (0x03 << CCM_LPCG_STAT1_CPU0_MODE_SHIFT)
#  define CCM_LPCG_STAT1_CPU0_MODE_RUN    (0x00 << CCM_LPCG_STAT1_CPU0_MODE_SHIFT) /* Run */
#  define CCM_LPCG_STAT1_CPU0_MODE_WAIT   (0x01 << CCM_LPCG_STAT1_CPU0_MODE_SHIFT) /* Wait */
#  define CCM_LPCG_STAT1_CPU0_MODE_STOP   (0x02 << CCM_LPCG_STAT1_CPU0_MODE_SHIFT) /* Stop */
#  define CCM_LPCG_STAT1_CPU0_MODE_SUSP   (0x03 << CCM_LPCG_STAT1_CPU0_MODE_SHIFT) /* Suspend */

#define CCM_LPCG_STAT1_CPU0_MODE_REQUEST  (1 << 2)  /* Bit 2:      Domain 0 request to enter Low Power Mode (CPU0_MODE_REQUEST) */
#define CCM_LPCG_STAT1_CPU0_MODE_DONE     (1 << 3)  /* Bit 3:      Domain 0 Low Power Mode task done (CPU0_MODE_DONE) */
#define CCM_LPCG_STAT1_CPU1_MODE_SHIFT    (4)       /* Bits 4-5:   Domain 1 Low Power Mode (CPU1_MODE) */
#define CCM_LPCG_STAT1_CPU1_MODE_MASK     (0x03 << CCM_LPCG_STAT1_CPU1_MODE_SHIFT)
#  define CCM_LPCG_STAT1_CPU1_MODE_RUN    (0x00 << CCM_LPCG_STAT1_CPU1_MODE_SHIFT) /* Run */
#  define CCM_LPCG_STAT1_CPU1_MODE_WAIT   (0x01 << CCM_LPCG_STAT1_CPU1_MODE_SHIFT) /* Wait */
#  define CCM_LPCG_STAT1_CPU1_MODE_STOP   (0x02 << CCM_LPCG_STAT1_CPU1_MODE_SHIFT) /* Stop */
#  define CCM_LPCG_STAT1_CPU1_MODE_SUSP   (0x03 << CCM_LPCG_STAT1_CPU1_MODE_SHIFT) /* Suspend */

#define CCM_LPCG_STAT1_CPU1_MODE_REQUEST (1 << 6)  /* Bit 6:      Domain 1 request to enter Low Power Mode (CPU1_MODE_REQUEST) */
#define CCM_LPCG_STAT1_CPU1_MODE_DONE    (1 << 7)  /* Bit 7:      Domain 1 Low Power Mode task done (CPU1_MODE_DONE) */
#define CCM_LPCG_STAT1_CPU2_MODE_SHIFT   (8)       /* Bits 8-9:   Domain 2 Low Power Mode (CPU2_MODE) */
#define CCM_LPCG_STAT1_CPU2_MODE_MASK    (0x03 << CCM_LPCG_STAT1_CPU2_MODE_SHIFT)
#  define CCM_LPCG_STAT1_CPU2_MODE_RUN   (0x00 << CCM_LPCG_STAT1_CPU2_MODE_SHIFT) /* Run */
#  define CCM_LPCG_STAT1_CPU2_MODE_WAIT  (0x01 << CCM_LPCG_STAT1_CPU2_MODE_SHIFT) /* Wait */
#  define CCM_LPCG_STAT1_CPU2_MODE_STOP  (0x02 << CCM_LPCG_STAT1_CPU2_MODE_SHIFT) /* Stop */
#  define CCM_LPCG_STAT1_CPU2_MODE_SUSP  (0x03 << CCM_LPCG_STAT1_CPU2_MODE_SHIFT) /* Suspend */

#define CCM_LPCG_STAT1_CPU2_MODE_REQUEST (1 << 10) /* Bit 10:     Domain 2 request to enter Low Power Mode (CPU2_MODE_REQUEST) */
#define CCM_LPCG_STAT1_CPU2_MODE_DONE    (1 << 11) /* Bit 11:     Domain 2 Low Power Mode task done (CPU2_MODE_DONE) */
#define CCM_LPCG_STAT1_CPU3_MODE_SHIFT   (12)      /* Bits 12-13: Domain 3 Low Power Mode (CPU3_MODE) */
#define CCM_LPCG_STAT1_CPU3_MODE_MASK    (0x03 << CCM_LPCG_STAT1_CPU3_MODE_SHIFT)
#  define CCM_LPCG_STAT1_CPU3_MODE_RUN   (0x00 << CCM_LPCG_STAT1_CPU3_MODE_SHIFT) /* Run */
#  define CCM_LPCG_STAT1_CPU3_MODE_WAIT  (0x01 << CCM_LPCG_STAT1_CPU3_MODE_SHIFT) /* Wait */
#  define CCM_LPCG_STAT1_CPU3_MODE_STOP  (0x02 << CCM_LPCG_STAT1_CPU3_MODE_SHIFT) /* Stop */
#  define CCM_LPCG_STAT1_CPU3_MODE_SUSP  (0x03 << CCM_LPCG_STAT1_CPU3_MODE_SHIFT) /* Suspend */

#define CCM_LPCG_STAT1_CPU3_MODE_REQUEST (1 << 14) /* Bit 14:     Domain 3 request to enter Low Power Mode (CPU3_MODE_REQUEST) */
#define CCM_LPCG_STAT1_CPU3_MODE_DONE    (1 << 15) /* Bit 15:     Domain 3 Low Power Mode task done (CPU3_MODE_DONE) */

#define CCM_LPCG_STAT1_TARGET_SETPOINT_SHIFT  (16) /* Bits 16-19: Setpoint value the SoC will switch to (TARGET_SETPOINT) */
#define CCM_LPCG_STAT1_TARGET_SETPOINT_MASK   (0x0f << CCM_LPCG_STAT1_TARGET_SETPOINT_SHIFT)
#define CCM_LPCG_STAT1_CURRENT_SETPOINT_SHIFT (20) /* Bits 20-23: Setpoint value the SoC is currently working in (CURRENT_SETPOINT) */
#define CCM_LPCG_STAT1_CURRENT_SETPOINT_MASK  (0x0f << CCM_LPCG_STAT1_CURRENT_SETPOINT_SHIFT)

#define CCM_LPCG_STAT1_SETPOINT_OFF_REQUEST (1 << 24) /* Bit 24: Clock gate turn off request from GPC setpoint (SETPOINT_OFF_REQUEST) */
#define CCM_LPCG_STAT1_SETPOINT_OFF_DONE    (1 << 25) /* Bit 25: Clock gate turn off completed (SETPOINT_OFF_DONE) */
#define CCM_LPCG_STAT1_SETPOINT_ON_REQUEST  (1 << 26) /* Bit 26: Clock gate turn on request from GPC setpoint (SETPOINT_ON_REQUEST) */
#define CCM_LPCG_STAT1_SETPOINT_ON_DONE     (1 << 27) /* Bit 27: Clock gate turn on completed (SETPOINT_ON_DONE) */
                                                      /* Bits 28-31: Reserved */

/* LPCG configuration (LPCGn_CONFIG, n=0..137) */

                                                    /* Bits 0-3:   Reserved */
#define CCM_LPCG_CFG_SETPOINT_PRESENT     (1 << 4)  /* Bit 4:      Setpoint is implemented (SETPOINT_PRESENT) */
                                                    /* Bits 5-31:  Reserved */

/* LPCG access control (LPCGn_AUTHEN, n=0..137) */

#define CCM_LPCG_AUTH_TZ_USER             (1 << 0)  /* Bit 0:      LPCG can be changed in user mode (TZ_USER) */
#define CCM_LPCG_AUTH_TZ_NS               (1 << 1)  /* Bit 1:      LPCG can be changed in non-secure mode (TZ_NS) */
                                                    /* Bits 2-3:   Reserved */
#define CCM_LPCG_AUTH_LOCK_TZ             (1 << 4)  /* Bit 4:      Lock TrustZone settings (LOCK_TZ) */
                                                    /* Bits 5-7:   Reserved */

#define CCM_LPCG_AUTH_WHITE_LIST_SHIFT     (8)      /* Bits 8-11:  Allow domains to change clock root (WHITE_LIST) */
#define CCM_LPCG_AUTH_WHITE_LIST_MASK      (0x0f << CCM_LPCG_AUTH_WHITE_LIST_SHIFT)
#  define CCM_LPCG_AUTH_WHITE_LIST_DOMAIN0 (0x01 << CCM_LPCG_AUTH_WHITE_LIST_SHIFT) /* Allow domain 0 to change clock root */
#  define CCM_LPCG_AUTH_WHITE_LIST_DOMAIN1 (0x02 << CCM_LPCG_AUTH_WHITE_LIST_SHIFT) /* Allow domain 1 to change clock root */
#  define CCM_LPCG_AUTH_WHITE_LIST_DOMAIN2 (0x04 << CCM_LPCG_AUTH_WHITE_LIST_SHIFT) /* Allow domain 2 to change clock root */
#  define CCM_LPCG_AUTH_WHITE_LIST_DOMAIN3 (0x08 << CCM_LPCG_AUTH_WHITE_LIST_SHIFT) /* Allow domain 3 to change clock root */

#define CCM_LPCG_AUTH_LOCK_LIST           (1 << 12) /* Bit 12:     Lock whitelist settings (LOCK_LIST) */
                                                    /* Bits 13-15: Reserved */
#define CCM_LPCG_AUTH_DOMAIN_MODE         (1 << 16) /* Bit 16:     Low power and access control by domain (DOMAIN_MODE) */
#define CCM_LPCG_AUTH_SETPOINT_MODE       (1 << 17) /* Bit 17:     LPCG works in setpoint controlled mode (SETPOINT_MODE) */
#define CCM_LPCG_AUTH_CPULPM              (1 << 18) /* Bit 18:     LPCG works in CPU Low Power Mode (CPULPM) */
                                                    /* Bit 19:     Reserved */
#define CCM_LPCG_AUTH_LOCK_MODE           (1 << 20) /* Bit 20:     Lock low power and access mode (LOCK_MODE) */
                                                    /* Bits 21-31: Reserved */

/* LPCG setpoint setting (LPCGn_SETPOINT, n=2..48) */

#define CCM_LPCG_SETP_SETPOINT_SHIFT      (0)       /* Bits 0-15: Defines 16 setpoint values (SETPOINT) */
#define CCM_LPCG_SETP_SETPOINT_MASK       (0xffff << CCM_LPCG_SETP_SETPOINT_SHIFT)
#define CCM_LPCG_SETP_STANDBY_SHIFT       (16)      /* Bits 16-31: Defines 16 setpoint standby values (STANDBY) */
#define CCM_LPCG_SETP_STANDBY_MASK        (0xffff << CCM_LPCG_SETP_STANDBY_SHIFT)

/* Clock Roots */

#define CCM_CR_M7          0  /* CLOCK ROOT M7. */
#define CCM_CR_M4          1  /* CLOCK ROOT M4. */
#define CCM_CR_BUS         2  /* CLOCK ROOT BUS. */
#define CCM_CR_BUS_LPSR    3  /* CLOCK ROOT BUS LPSR. */
#define CCM_CR_SEMC        4  /* CLOCK ROOT SEMC. */
#define CCM_CR_CSSYS       5  /* CLOCK ROOT CSSYS. */
#define CCM_CR_CSTRACE     6  /* CLOCK ROOT CSTRACE. */
#define CCM_CR_M4_SYSTICK  7  /* CLOCK ROOT M4 SYSTICK. */
#define CCM_CR_M7_SYSTICK  8  /* CLOCK ROOT M7 SYSTICK. */
#define CCM_CR_ADC1        9  /* CLOCK ROOT ADC1. */
#define CCM_CR_ADC2        10 /* CLOCK ROOT ADC2. */
#define CCM_CR_ACMP        11 /* CLOCK ROOT ACMP. */
#define CCM_CR_FLEXIO1     12 /* CLOCK ROOT FLEXIO1. */
#define CCM_CR_FLEXIO2     13 /* CLOCK ROOT FLEXIO2. */
#define CCM_CR_GPT1        14 /* CLOCK ROOT GPT1. */
#define CCM_CR_GPT2        15 /* CLOCK ROOT GPT2. */
#define CCM_CR_GPT3        16 /* CLOCK ROOT GPT3. */
#define CCM_CR_GPT4        17 /* CLOCK ROOT GPT4. */
#define CCM_CR_GPT5        18 /* CLOCK ROOT GPT5. */
#define CCM_CR_GPT6        19 /* CLOCK ROOT GPT6. */
#define CCM_CR_FLEXSPI1    20 /* CLOCK ROOT FLEXSPI1. */
#define CCM_CR_FLEXSPI2    21 /* CLOCK ROOT FLEXSPI2. */
#define CCM_CR_CAN1        22 /* CLOCK ROOT CAN1. */
#define CCM_CR_CAN2        23 /* CLOCK ROOT CAN2. */
#define CCM_CR_CAN3        24 /* CLOCK ROOT CAN3. */
#define CCM_CR_LPUART1     25 /* CLOCK ROOT LPUART1. */
#define CCM_CR_LPUART2     26 /* CLOCK ROOT LPUART2. */
#define CCM_CR_LPUART3     27 /* CLOCK ROOT LPUART3. */
#define CCM_CR_LPUART4     28 /* CLOCK ROOT LPUART4. */
#define CCM_CR_LPUART5     29 /* CLOCK ROOT LPUART5. */
#define CCM_CR_LPUART6     30 /* CLOCK ROOT LPUART6. */
#define CCM_CR_LPUART7     31 /* CLOCK ROOT LPUART7. */
#define CCM_CR_LPUART8     32 /* CLOCK ROOT LPUART8. */
#define CCM_CR_LPUART9     33 /* CLOCK ROOT LPUART9. */
#define CCM_CR_LPUART10    34 /* CLOCK ROOT LPUART10. */
#define CCM_CR_LPUART11    35 /* CLOCK ROOT LPUART11. */
#define CCM_CR_LPUART12    36 /* CLOCK ROOT LPUART12. */
#define CCM_CR_LPI2C1      37 /* CLOCK ROOT LPI2C1. */
#define CCM_CR_LPI2C2      38 /* CLOCK ROOT LPI2C2. */
#define CCM_CR_LPI2C3      39 /* CLOCK ROOT LPI2C3. */
#define CCM_CR_LPI2C4      40 /* CLOCK ROOT LPI2C4. */
#define CCM_CR_LPI2C5      41 /* CLOCK ROOT LPI2C5. */
#define CCM_CR_LPI2C6      42 /* CLOCK ROOT LPI2C6. */
#define CCM_CR_LPSPI1      43 /* CLOCK ROOT LPSPI1. */
#define CCM_CR_LPSPI2      44 /* CLOCK ROOT LPSPI2. */
#define CCM_CR_LPSPI3      45 /* CLOCK ROOT LPSPI3. */
#define CCM_CR_LPSPI4      46 /* CLOCK ROOT LPSPI4. */
#define CCM_CR_LPSPI5      47 /* CLOCK ROOT LPSPI5. */
#define CCM_CR_LPSPI6      48 /* CLOCK ROOT LPSPI6. */
#define CCM_CR_EMV1        49 /* CLOCK ROOT EMV1. */
#define CCM_CR_EMV2        50 /* CLOCK ROOT EMV2. */
#define CCM_CR_ENET1       51 /* CLOCK ROOT ENET1. */
#define CCM_CR_ENET2       52 /* CLOCK ROOT ENET2. */
#define CCM_CR_ENET_QOS    53 /* CLOCK ROOT ENET QOS. */
#define CCM_CR_ENET_25M    54 /* CLOCK ROOT ENET 25M. */
#define CCM_CR_ENET_TIMER1 55 /* CLOCK ROOT ENET TIMER1. */
#define CCM_CR_ENET_TIMER2 56 /* CLOCK ROOT ENET TIMER2. */
#define CCM_CR_ENET_TIMER3 57 /* CLOCK ROOT ENET TIMER3. */
#define CCM_CR_USDHC1      58 /* CLOCK ROOT USDHC1. */
#define CCM_CR_USDHC2      59 /* CLOCK ROOT USDHC2. */
#define CCM_CR_ASRC        60 /* CLOCK ROOT ASRC. */
#define CCM_CR_MQS         61 /* CLOCK ROOT MQS. */
#define CCM_CR_MIC         62 /* CLOCK ROOT MIC. */
#define CCM_CR_SPDIF       63 /* CLOCK ROOT SPDIF */
#define CCM_CR_SAI1        64 /* CLOCK ROOT SAI1. */
#define CCM_CR_SAI2        65 /* CLOCK ROOT SAI2. */
#define CCM_CR_SAI3        66 /* CLOCK ROOT SAI3. */
#define CCM_CR_SAI4        67 /* CLOCK ROOT SAI4. */
#define CCM_CR_GC355       68 /* CLOCK ROOT GC355. */
#define CCM_CR_LCDIF       69 /* CLOCK ROOT LCDIF. */
#define CCM_CR_LCDIFV2     70 /* CLOCK ROOT LCDIFV2. */
#define CCM_CR_MIPI_REF    71 /* CLOCK ROOT MIPI REF. */
#define CCM_CR_MIPI_ESC    72 /* CLOCK ROOT MIPI ESC. */
#define CCM_CR_CSI2        73 /* CLOCK ROOT CSI2. */
#define CCM_CR_CSI2_ESC    74 /* CLOCK ROOT CSI2 ESC. */
#define CCM_CR_CSI2_UI     75 /* CLOCK ROOT CSI2 UI. */
#define CCM_CR_CSI         76 /* CLOCK ROOT CSI. */
#define CCM_CR_CKO1        77 /* CLOCK ROOT CKO1. */
#define CCM_CR_CKO2        78 /* CLOCK ROOT CKO2. */

/* Note IMXRT7 uses the definition LPCG instead of CCGR as the clock gate
 * register but for compatiblity we define them as LPCG
 */

#define CCM_CG_OFF                               (0)  /* Clock is off during all modes */
#define CCM_CG_RUN                               (1)  /* Clock is on in run mode, but off in WAIT and STOP modes */
#define CCM_CG_ALL                               (3)  /* Clock is on during all modes, except STOP mode. */

#define CCM_CCGR_M7                           0
#define CCM_CCGR_M4                           1
#define CCM_CCGR_SIM_M7                       2
#define CCM_CCGR_SIM_M                        3
#define CCM_CCGR_SIM_DISP                     4
#define CCM_CCGR_SIM_PER                      5
#define CCM_CCGR_SIM_LPSR                     6
#define CCM_CCGR_ANADIG                       7
#define CCM_CCGR_DCDC                         8
#define CCM_CCGR_SRC                          9
#define CCM_CCGR_CCM                         10
#define CCM_CCGR_GPC                         11
#define CCM_CCGR_SSARC                       12
#define CCM_CCGR_SIM_R                       13
#define CCM_CCGR_WDOG1                       14
#define CCM_CCGR_WDOG2                       15
#define CCM_CCGR_WDOG3                       16
#define CCM_CCGR_WDOG4                       17
#define CCM_CCGR_EWM0                        18
#define CCM_CCGR_SEMA                        19
#define CCM_CCGR_MU_A                        20
#define CCM_CCGR_MU_B                        21
#define CCM_CCGR_EDMA                        22
#define CCM_CCGR_DMA                         22 //Note Added CTRL for compatiblity
#define CCM_CCGR_EDMA_LPSR                   23
#define CCM_CCGR_ROMCP                       24
#define CCM_CCGR_OCRAM                       25
#define CCM_CCGR_FLEXRAM                     26
#define CCM_CCGR_LMEM                        27
#define CCM_CCGR_FLEXSPI1                    28
#define CCM_CCGR_FLEXSPI2                    29
#define CCM_CCGR_RDC                         30
#define CCM_CCGR_M7_XRDC                     31
#define CCM_CCGR_M4_XRDC                     32
#define CCM_CCGR_SEMC                        33
#define CCM_CCGR_XECC                        34
#define CCM_CCGR_IEE                         35
#define CCM_CCGR_KEY_MANAGER                 36
#define CCM_CCGR_PUF                         36
#define CCM_CCGR_OCOTP_CTRL                  37 //Note Added CTRL for compatiblity
#define CCM_CCGR_SNVS_HP                     38
#define CCM_CCGR_SNVS                        39
#define CCM_CCGR_SNVS_LP                     39 //Note Added CTRL for compatiblity
#define CCM_CCGR_CAAM                        40
#define CCM_CCGR_JTAG_MUX                    41
#define CCM_CCGR_CSTRACE                     42
#define CCM_CCGR_XBAR1                       43
#define CCM_CCGR_XBAR2                       44
#define CCM_CCGR_XBAR3                       45
#define CCM_CCGR_AOI1                        46
#define CCM_CCGR_AOI2                        47
#define CCM_CCGR_ADC_ETC                     48
#define CCM_CCGR_IOMUXC                      49
#define CCM_CCGR_IOMUXC_LPSR                 50
#define CCM_CCGR_GPIO                        51
#define CCM_CCGR_KPP                         52
#define CCM_CCGR_FLEXIO1                     53
#define CCM_CCGR_FLEXIO2                     54
#define CCM_CCGR_ADC1                        55
#define CCM_CCGR_ADC2                        56
#define CCM_CCGR_DAC                         57
#define CCM_CCGR_ACMP1                       58
#define CCM_CCGR_ACMP2                       59
#define CCM_CCGR_ACMP3                       60
#define CCM_CCGR_ACMP4                       61
#define CCM_CCGR_PIT                         62 // Renamed from PIT1 to PIT for compatibility
#define CCM_CCGR_PIT2                        63
#define CCM_CCGR_GPT1                        64
#define CCM_CCGR_GPT2                        65
#define CCM_CCGR_GPT3                        66
#define CCM_CCGR_GPT4                        67
#define CCM_CCGR_GPT5                        68
#define CCM_CCGR_GPT6                        69
#define CCM_CCGR_TIMER1                      70
#define CCM_CCGR_TIMER2                      71
#define CCM_CCGR_TIMER3                      72
#define CCM_CCGR_QTIMER4                     73
#define CCM_CCGR_ENC1                        74
#define CCM_CCGR_ENC2                        75
#define CCM_CCGR_ENC3                        76
#define CCM_CCGR_ENC4                        77
#define CCM_CCGR_HRTIMER                     78
#define CCM_CCGR_PWM1                        79
#define CCM_CCGR_PWM2                        80
#define CCM_CCGR_PWM3                        81
#define CCM_CCGR_PWM4                        82
#define CCM_CCGR_CAN1                        83
#define CCM_CCGR_CAN2                        84
#define CCM_CCGR_CAN3                        85
#define CCM_CCGR_LPUART1                     86
#define CCM_CCGR_LPUART2                     87
#define CCM_CCGR_LPUART3                     88
#define CCM_CCGR_LPUART4                     89
#define CCM_CCGR_LPUART5                     90
#define CCM_CCGR_LPUART6                     91
#define CCM_CCGR_LPUART7                     92
#define CCM_CCGR_LPUART8                     93
#define CCM_CCGR_LPUART9                     94
#define CCM_CCGR_LPUART10                    95
#define CCM_CCGR_LPUART11                    96
#define CCM_CCGR_LPUART12                    97
#define CCM_CCGR_LPI2C1                      98
#define CCM_CCGR_LPI2C2                      99
#define CCM_CCGR_LPI2C3                     100
#define CCM_CCGR_LPI2C4                     101
#define CCM_CCGR_LPI2C5                     102
#define CCM_CCGR_LPI2C6                     103
#define CCM_CCGR_LPSPI1                     104
#define CCM_CCGR_LPSPI2                     105
#define CCM_CCGR_LPSPI3                     106
#define CCM_CCGR_LPSPI4                     107
#define CCM_CCGR_LPSPI5                     108
#define CCM_CCGR_LPSPI6                     109
#define CCM_CCGR_SIM1                       110
#define CCM_CCGR_SIM2                       111
#define CCM_CCGR_ENET                       112
#define CCM_CCGR_ENET_1G                    113
#define CCM_CCGR_ENET_QOS                   114
#define CCM_CCGR_USB                        115
#define CCM_CCGR_CDOG                       116
#define CCM_CCGR_USDHC1                     117
#define CCM_CCGR_USDHC2                     118
#define CCM_CCGR_ASRC                       119
#define CCM_CCGR_MQS                        120
#define CCM_CCGR_PDM                        121
#define CCM_CCGR_SPDIF                      122
#define CCM_CCGR_SAI1                       123
#define CCM_CCGR_SAI2                       124
#define CCM_CCGR_SAI3                       125
#define CCM_CCGR_SAI4                       126
#define CCM_CCGR_PXP                        127
#define CCM_CCGR_GPU2D                      128
#define CCM_CCGR_LCDIF                      129
#define CCM_CCGR_LCDIFV2                    130
#define CCM_CCGR_MIPI_DSI                   131
#define CCM_CCGR_MIPI_CSI                   132
#define CCM_CCGR_CSI                        133
#define CCM_CCGR_DCIC_MIPI                  134
#define CCM_CCGR_DCIC_LCD                   135
#define CCM_CCGR_VIDEO_MUX                  136
#define CCM_CCGR_UNIQ_EDT_I                 137

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_CCM_H */
