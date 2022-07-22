/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_mu.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MU_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MU Register Offsets ******************************************************/

#define S32K3XX_MU_VER_OFFSET   (0x0000) /* Version ID Register (VER) */
#define S32K3XX_MU_PAR_OFFSET   (0x0004) /* Parameter Register (PAR) */
#define S32K3XX_MU_CR_OFFSET    (0x0008) /* Control Register (CR) */
#define S32K3XX_MU_SR_OFFSET    (0x000c) /* Status Register (SR) */
#define S32K3XX_MU_CCR0_OFFSET  (0x0010) /* Core Control Register 0 (CCR0) */
#define S32K3XX_MU_CSSR0_OFFSET (0x0018) /* Core Sticky Status Register 0 (CSSR0) */
#define S32K3XX_MU_FCR_OFFSET   (0x0100) /* Flag Control Register (FCR) */
#define S32K3XX_MU_FSR_OFFSET   (0x0104) /* Flag Status Register (FSR) */
#define S32K3XX_MU_GIER_OFFSET  (0x0110) /* General Interrupt Enable Register (GIER) */
#define S32K3XX_MU_GCR_OFFSET   (0x0114) /* General Control Register (GCR) */
#define S32K3XX_MU_GSR_OFFSET   (0x0118) /* General Status Register (GSR) */
#define S32K3XX_MU_TCR_OFFSET   (0x0120) /* Transmit Control Register (TCR) */
#define S32K3XX_MU_TSR_OFFSET   (0x0124) /* Transmit Status Register (TSR) */
#define S32K3XX_MU_RCR_OFFSET   (0x0128) /* Receive Control Register (RCR) */
#define S32K3XX_MU_RSR_OFFSET   (0x012c) /* Receive Status Register (RSR) */
#define S32K3XX_MU_TR0_OFFSET   (0x0200) /* Transmit Register 0 (TR0) */
#define S32K3XX_MU_TR1_OFFSET   (0x0204) /* Transmit Register 1 (TR1) */
#define S32K3XX_MU_TR2_OFFSET   (0x0208) /* Transmit Register 2 (TR2) */
#define S32K3XX_MU_TR3_OFFSET   (0x020c) /* Transmit Register 3 (TR3) */
#define S32K3XX_MU_RR0_OFFSET   (0x0280) /* Receive Register 0 (RR0) */
#define S32K3XX_MU_RR1_OFFSET   (0x0284) /* Receive Register 1 (RR1) */
#define S32K3XX_MU_RR2_OFFSET   (0x0288) /* Receive Register 2 (RR2) */
#define S32K3XX_MU_RR3_OFFSET   (0x028c) /* Receive Register 3 (RR3) */

/* MU Register Addresses ****************************************************/

#define S32K3XX_MU0_MUB_VER     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_VER_OFFSET)
#define S32K3XX_MU0_MUB_PAR     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_PAR_OFFSET)
#define S32K3XX_MU0_MUB_CR      (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_CR_OFFSET)
#define S32K3XX_MU0_MUB_SR      (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_SR_OFFSET)
#define S32K3XX_MU0_MUB_CCR0    (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_CCR0_OFFSET)
#define S32K3XX_MU0_MUB_CSSR0   (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_CSSR0_OFFSET)
#define S32K3XX_MU0_MUB_FCR     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_FCR_OFFSET)
#define S32K3XX_MU0_MUB_FSR     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_FSR_OFFSET)
#define S32K3XX_MU0_MUB_GIER    (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_GIER_OFFSET)
#define S32K3XX_MU0_MUB_GCR     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_GCR_OFFSET)
#define S32K3XX_MU0_MUB_GSR     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_GSR_OFFSET)
#define S32K3XX_MU0_MUB_TCR     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_TCR_OFFSET)
#define S32K3XX_MU0_MUB_TSR     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_TSR_OFFSET)
#define S32K3XX_MU0_MUB_RCR     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_RCR_OFFSET)
#define S32K3XX_MU0_MUB_RSR     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_RSR_OFFSET)
#define S32K3XX_MU0_MUB_TR0     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_TR0_OFFSET)
#define S32K3XX_MU0_MUB_TR1     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_TR1_OFFSET)
#define S32K3XX_MU0_MUB_TR2     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_TR2_OFFSET)
#define S32K3XX_MU0_MUB_TR3     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_TR3_OFFSET)
#define S32K3XX_MU0_MUB_RR0     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_RR0_OFFSET)
#define S32K3XX_MU0_MUB_RR1     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_RR1_OFFSET)
#define S32K3XX_MU0_MUB_RR2     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_RR2_OFFSET)
#define S32K3XX_MU0_MUB_RR3     (S32K3XX_MU0_MUB_BASE + S32K3XX_MU_RR3_OFFSET)

#define S32K3XX_MU1_MUB_VER     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_VER_OFFSET)
#define S32K3XX_MU1_MUB_PAR     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_PAR_OFFSET)
#define S32K3XX_MU1_MUB_CR      (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_CR_OFFSET)
#define S32K3XX_MU1_MUB_SR      (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_SR_OFFSET)
#define S32K3XX_MU1_MUB_CCR0    (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_CCR0_OFFSET)
#define S32K3XX_MU1_MUB_CSSR0   (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_CSSR0_OFFSET)
#define S32K3XX_MU1_MUB_FCR     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_FCR_OFFSET)
#define S32K3XX_MU1_MUB_FSR     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_FSR_OFFSET)
#define S32K3XX_MU1_MUB_GIER    (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_GIER_OFFSET)
#define S32K3XX_MU1_MUB_GCR     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_GCR_OFFSET)
#define S32K3XX_MU1_MUB_GSR     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_GSR_OFFSET)
#define S32K3XX_MU1_MUB_TCR     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_TCR_OFFSET)
#define S32K3XX_MU1_MUB_TSR     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_TSR_OFFSET)
#define S32K3XX_MU1_MUB_RCR     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_RCR_OFFSET)
#define S32K3XX_MU1_MUB_RSR     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_RSR_OFFSET)
#define S32K3XX_MU1_MUB_TR0     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_TR0_OFFSET)
#define S32K3XX_MU1_MUB_TR1     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_TR1_OFFSET)
#define S32K3XX_MU1_MUB_TR2     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_TR2_OFFSET)
#define S32K3XX_MU1_MUB_TR3     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_TR3_OFFSET)
#define S32K3XX_MU1_MUB_RR0     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_RR0_OFFSET)
#define S32K3XX_MU1_MUB_RR1     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_RR1_OFFSET)
#define S32K3XX_MU1_MUB_RR2     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_RR2_OFFSET)
#define S32K3XX_MU1_MUB_RR3     (S32K3XX_MU1_MUB_BASE + S32K3XX_MU_RR3_OFFSET)

#define S32K3XX_MU2_MUA_VER     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_VER_OFFSET)
#define S32K3XX_MU2_MUA_PAR     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_PAR_OFFSET)
#define S32K3XX_MU2_MUA_CR      (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_CR_OFFSET)
#define S32K3XX_MU2_MUA_SR      (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_SR_OFFSET)
#define S32K3XX_MU2_MUA_FCR     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_FCR_OFFSET)
#define S32K3XX_MU2_MUA_FSR     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_FSR_OFFSET)
#define S32K3XX_MU2_MUA_GIER    (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_GIER_OFFSET)
#define S32K3XX_MU2_MUA_GCR     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_GCR_OFFSET)
#define S32K3XX_MU2_MUA_GSR     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_GSR_OFFSET)
#define S32K3XX_MU2_MUA_TCR     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_TCR_OFFSET)
#define S32K3XX_MU2_MUA_TSR     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_TSR_OFFSET)
#define S32K3XX_MU2_MUA_RCR     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_RCR_OFFSET)
#define S32K3XX_MU2_MUA_RSR     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_RSR_OFFSET)
#define S32K3XX_MU2_MUA_TR0     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_TR0_OFFSET)
#define S32K3XX_MU2_MUA_TR1     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_TR1_OFFSET)
#define S32K3XX_MU2_MUA_TR2     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_TR2_OFFSET)
#define S32K3XX_MU2_MUA_TR3     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_TR3_OFFSET)
#define S32K3XX_MU2_MUA_RR0     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_RR0_OFFSET)
#define S32K3XX_MU2_MUA_RR1     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_RR1_OFFSET)
#define S32K3XX_MU2_MUA_RR2     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_RR2_OFFSET)
#define S32K3XX_MU2_MUA_RR3     (S32K3XX_MU2_MUA_BASE + S32K3XX_MU_RR3_OFFSET)

#define S32K3XX_MU2_MUB_VER     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_VER_OFFSET)
#define S32K3XX_MU2_MUB_PAR     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_PAR_OFFSET)
#define S32K3XX_MU2_MUB_CR      (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_CR_OFFSET)
#define S32K3XX_MU2_MUB_SR      (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_SR_OFFSET)
#define S32K3XX_MU2_MUB_CCR0    (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_CCR0_OFFSET)
#define S32K3XX_MU2_MUB_CSSR0   (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_CSSR0_OFFSET)
#define S32K3XX_MU2_MUB_FCR     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_FCR_OFFSET)
#define S32K3XX_MU2_MUB_FSR     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_FSR_OFFSET)
#define S32K3XX_MU2_MUB_GIER    (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_GIER_OFFSET)
#define S32K3XX_MU2_MUB_GCR     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_GCR_OFFSET)
#define S32K3XX_MU2_MUB_GSR     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_GSR_OFFSET)
#define S32K3XX_MU2_MUB_TCR     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_TCR_OFFSET)
#define S32K3XX_MU2_MUB_TSR     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_TSR_OFFSET)
#define S32K3XX_MU2_MUB_RCR     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_RCR_OFFSET)
#define S32K3XX_MU2_MUB_RSR     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_RSR_OFFSET)
#define S32K3XX_MU2_MUB_TR0     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_TR0_OFFSET)
#define S32K3XX_MU2_MUB_TR1     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_TR1_OFFSET)
#define S32K3XX_MU2_MUB_TR2     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_TR2_OFFSET)
#define S32K3XX_MU2_MUB_TR3     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_TR3_OFFSET)
#define S32K3XX_MU2_MUB_RR0     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_RR0_OFFSET)
#define S32K3XX_MU2_MUB_RR1     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_RR1_OFFSET)
#define S32K3XX_MU2_MUB_RR2     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_RR2_OFFSET)
#define S32K3XX_MU2_MUB_RR3     (S32K3XX_MU2_MUB_BASE + S32K3XX_MU_RR3_OFFSET)

/* MU Register Bitfield Definitions *****************************************/

/* Version ID Register (VER) */

#define MU_FEATURE_SHIFT        (0)        /* Bits 0-15: Feature Set Number (FEATURE) */
#define MU_FEATURE_MASK         (0xffff << MU_FEATURE_SHIFT)
#  define MU_FEATURE_STANDARD   (1 << 0)   /* Bit 0: Standard features are implemented */
#  define MU_FEATURE_RAIP_RAIE  (1 << 1)   /* Bit 1: RAIP/RAIE register bits are implemented */
#  define MU_FEATURE_CCR_CSSR   (1 << 2)   /* Bit 2: Core Control and Status Registers are implemented in both MUA and MUB */
#  define MU_FEATURE_EXPAND     (1 << 3)   /* Bit 3: Expand TRn/RRn registers number */

#define MU_MINOR_SHIFT          (16)       /* Bits 16-23: Minor Version Number (MINOR) */
#define MU_MINOR_MASK           (0xff << MU_MINOR_SHIFT)
#define MU_MAJOR_SHIFT          (24)       /* Bits 24-31: Major Version Number (MAJOR) */
#define MU_MAJOR_MASK           (0xff << MU_MAJOR_SHIFT)

/* Parameter Register (PAR) */

#define MU_PAR_TR_NUM_SHIFT     (0)        /* Bits 0-7: Transmit Register Number (TR_NUM) */
#define MU_PAR_TR_NUM_MASK      (0xff << MU_PAR_TR_NUM_SHIFT)
#define MU_PAR_RR_NUM_SHIFT     (8)        /* Bits 8-15: Receive Register Number (RR_NUM) */
#define MU_PAR_RR_NUM_MASK      (0xff << MU_PAR_RR_NUM_SHIFT)
#define MU_PAR_GIR_NUM_SHIFT    (16)       /* Bits 16-23: General Interrupt Request Number (GIR_NUM) */
#define MU_PAR_GIR_NUM_MASK     (0xff << MU_PAR_GIR_NUM_SHIFT)
#define MU_PAR_FLAG_WIDTH_SHIFT (24)       /* Bits 24-31: Flag Width (FLAG_WIDTH) */
#define MU_PAR_FLAG_WIDTH_MASK  (0xff << MU_PAR_FLAG_WIDTH_SHIFT)

/* Control Register (CR) */

#define MU_CR_MUR               (1 << 0)   /* Bit 0: MUA and MUB Reset (MUR) */
#define MU_CR_MURIE             (1 << 1)   /* Bit 1: MU Reset Interrupt Enable (MURIE) */
                                           /* Bits 2-31: Reserved */

/* Status Register (SR) */

#define MU_SR_MURS              (1 << 0)   /* Bit 0: MUA and MUB Reset State (MURS) */
#define MU_SR_MURIP             (1 << 1)   /* Bit 1: MU Reset Interrupt Pending (MURIP) */
#define MU_SR_EP                (1 << 2)   /* Bit 2: MU Side Event Pending (EP) */
#define MU_SR_FUP               (1 << 3)   /* Bit 3: MU Flags Update Pending (FUP) */
#define MU_SR_GIRP              (1 << 4)   /* Bit 4: MU General Interrupt Pending (GIRP) */
#define MU_SR_TEP               (1 << 5)   /* Bit 5: MU Transmit Empty Pending (TEP) */
#define MU_SR_RFP               (1 << 6)   /* Bit 6: MU Receive Full Pending Flag (RFP) */
                                           /* Bits 7-31: Reserved */

/* Core Control Register 0 (CCR0) */

#define MU_CCR0_NMI             (1 << 0)   /* Bit 0: MUA Non-maskable Interrupt Request */
                                           /* Bits 1-31: Reserved */

/* Core Sticky Status Register 0 (CSSR0) */

#define MU_CSSR0_NMIC           (1 << 0)   /* Bit 0: Processor B Non-Maskable-Interrupt Clear */
                                           /* Bits 1-31: Reserved */

/* Flag Control Register (FCR) */

#define MU_FCR_F(n)             (1 << (n)) /* Bit n: MUA/MUB to MUB/MUA Flag n (Fn) */
#define MU_FCR_F0               (1 << 0)   /* Bit 0: MUA/MUB to MUB/MUA Flag 0 (F0) */
#define MU_FCR_F1               (1 << 1)   /* Bit 1: MUA/MUB to MUB/MUA Flag 1 (F1) */
#define MU_FCR_F2               (1 << 2)   /* Bit 2: MUA/MUB to MUB/MUA Flag 2 (F2) */
#define MU_FCR_F3               (1 << 3)   /* Bit 3: MUB to MUA Flag 3 (F3) */
#define MU_FCR_F4               (1 << 4)   /* Bit 4: MUB to MUA Flag 4 (F4) */
#define MU_FCR_F5               (1 << 5)   /* Bit 5: MUB to MUA Flag 5 (F5) */
#define MU_FCR_F6               (1 << 6)   /* Bit 6: MUB to MUA Flag 6 (F6) */
#define MU_FCR_F7               (1 << 7)   /* Bit 7: MUB to MUA Flag 7 (F7) */
#define MU_FCR_F8               (1 << 8)   /* Bit 8: MUB to MUA Flag 8 (F8) */
#define MU_FCR_F9               (1 << 9)   /* Bit 9: MUB to MUA Flag 9 (F9) */
#define MU_FCR_F10              (1 << 10)  /* Bit 10: MUB to MUA Flag 10 (F10) */
#define MU_FCR_F11              (1 << 11)  /* Bit 11: MUB to MUA Flag 11 (F11) */
#define MU_FCR_F12              (1 << 12)  /* Bit 12: MUB to MUA Flag 12 (F12) */
#define MU_FCR_F13              (1 << 13)  /* Bit 13: MUB to MUA Flag 13 (F13) */
#define MU_FCR_F14              (1 << 14)  /* Bit 14: MUB to MUA Flag 14 (F14) */
#define MU_FCR_F15              (1 << 15)  /* Bit 15: MUB to MUA Flag 15 (F15) */
#define MU_FCR_F16              (1 << 16)  /* Bit 16: MUB to MUA Flag 16 (F16) */
#define MU_FCR_F17              (1 << 17)  /* Bit 17: MUB to MUA Flag 17 (F17) */
#define MU_FCR_F18              (1 << 18)  /* Bit 18: MUB to MUA Flag 18 (F18) */
#define MU_FCR_F19              (1 << 19)  /* Bit 19: MUB to MUA Flag 19 (F19) */
#define MU_FCR_F20              (1 << 20)  /* Bit 20: MUB to MUA Flag 20 (F20) */
#define MU_FCR_F21              (1 << 21)  /* Bit 21: MUB to MUA Flag 21 (F21) */
#define MU_FCR_F22              (1 << 22)  /* Bit 22: MUB to MUA Flag 22 (F22) */
#define MU_FCR_F23              (1 << 23)  /* Bit 23: MUB to MUA Flag 23 (F23) */
#define MU_FCR_F24              (1 << 24)  /* Bit 24: MUB to MUA Flag 24 (F24) */
#define MU_FCR_F25              (1 << 25)  /* Bit 25: MUB to MUA Flag 25 (F25) */
#define MU_FCR_F26              (1 << 26)  /* Bit 26: MUB to MUA Flag 26 (F26) */
#define MU_FCR_F27              (1 << 27)  /* Bit 27: MUB to MUA Flag 27 (F27) */
#define MU_FCR_F28              (1 << 28)  /* Bit 28: MUB to MUA Flag 28 (F28) */
#define MU_FCR_F29              (1 << 29)  /* Bit 29: MUB to MUA Flag 29 (F29) */
#define MU_FCR_F30              (1 << 30)  /* Bit 30: MUB to MUA Flag 30 (F30) */
#define MU_FCR_F31              (1 << 31)  /* Bit 31: MUB to MUA Flag 31 (F31) */

/* Flag Status Register (FSR) */

#define MU_FSR_F(n)             (1 << (n)) /* Bit n: MUB/MUA to MUA/MUB Side Flag n (Fn) */
#define MU_FSR_F0               (1 << 0)   /* Bit 0: MUB/MUA to MUA/MUB Side Flag 0 (F0) */
#define MU_FSR_F1               (1 << 1)   /* Bit 1: MUB/MUA to MUA/MUB Side Flag 1 (F1) */
#define MU_FSR_F2               (1 << 2)   /* Bit 2: MUB/MUA to MUA/MUB Side Flag 2 (F2) */
#define MU_FSR_F3               (1 << 3)   /* Bit 3: MUA to MUB Side Flag 3 (F3) */
#define MU_FSR_F4               (1 << 4)   /* Bit 4: MUA to MUB Side Flag 4 (F4) */
#define MU_FSR_F5               (1 << 5)   /* Bit 5: MUA to MUB Side Flag 5 (F5) */
#define MU_FSR_F6               (1 << 6)   /* Bit 6: MUA to MUB Side Flag 6 (F6) */
#define MU_FSR_F7               (1 << 7)   /* Bit 7: MUA to MUB Side Flag 7 (F7) */
#define MU_FSR_F8               (1 << 8)   /* Bit 8: MUA to MUB Side Flag 8 (F8) */
#define MU_FSR_F9               (1 << 9)   /* Bit 9: MUA to MUB Side Flag 9 (F9) */
#define MU_FSR_F10              (1 << 10)  /* Bit 10: MUA to MUB Side Flag 10 (F10) */
#define MU_FSR_F11              (1 << 11)  /* Bit 11: MUA to MUB Side Flag 11 (F11) */
#define MU_FSR_F12              (1 << 12)  /* Bit 12: MUA to MUB Side Flag 12 (F12) */
#define MU_FSR_F13              (1 << 13)  /* Bit 13: MUA to MUB Side Flag 13 (F13) */
#define MU_FSR_F14              (1 << 14)  /* Bit 14: MUA to MUB Side Flag 14 (F14) */
#define MU_FSR_F15              (1 << 15)  /* Bit 15: MUA to MUB Side Flag 15 (F15) */
#define MU_FSR_F16              (1 << 16)  /* Bit 16: MUA to MUB Side Flag 16 (F16) */
#define MU_FSR_F17              (1 << 17)  /* Bit 17: MUA to MUB Side Flag 17 (F17) */
#define MU_FSR_F18              (1 << 18)  /* Bit 18: MUA to MUB Side Flag 18 (F18) */
#define MU_FSR_F19              (1 << 19)  /* Bit 19: MUA to MUB Side Flag 19 (F19) */
#define MU_FSR_F20              (1 << 20)  /* Bit 20: MUA to MUB Side Flag 20 (F20) */
#define MU_FSR_F21              (1 << 21)  /* Bit 21: MUA to MUB Side Flag 21 (F21) */
#define MU_FSR_F22              (1 << 22)  /* Bit 22: MUA to MUB Side Flag 22 (F22) */
#define MU_FSR_F23              (1 << 23)  /* Bit 23: MUA to MUB Side Flag 23 (F23) */
#define MU_FSR_F24              (1 << 24)  /* Bit 24: MUA to MUB Side Flag 24 (F24) */
#define MU_FSR_F25              (1 << 25)  /* Bit 25: MUA to MUB Side Flag 25 (F25) */
#define MU_FSR_F26              (1 << 26)  /* Bit 26: MUA to MUB Side Flag 26 (F26) */
#define MU_FSR_F27              (1 << 27)  /* Bit 27: MUA to MUB Side Flag 27 (F27) */
#define MU_FSR_F28              (1 << 28)  /* Bit 28: MUA to MUB Side Flag 28 (F28) */
#define MU_FSR_F29              (1 << 29)  /* Bit 29: MUA to MUB Side Flag 29 (F29) */
#define MU_FSR_F30              (1 << 30)  /* Bit 30: MUA to MUB Side Flag 30 (F30) */
#define MU_FSR_F31              (1 << 31)  /* Bit 31: MUA to MUB Side Flag 31 (F31) */

/* General Interrupt Enable Register (GIER) */

#define MU_GIER_GIE(n)          (1 << (n)) /* Bit n: MUA/MUB General Purpose Interrupt Enable n (GIEn) */
#define MU_GIER_GIE0            (1 << 0)   /* Bit 0: MUA/MUB General Purpose Interrupt Enable 0 (GIE0) */
#define MU_GIER_GIE1            (1 << 1)   /* Bit 1: MUB General Purpose Interrupt Enable 1 (GIE1) */
#define MU_GIER_GIE2            (1 << 2)   /* Bit 2: MUB General Purpose Interrupt Enable 2 (GIE2) */
#define MU_GIER_GIE3            (1 << 3)   /* Bit 3: MUB General Purpose Interrupt Enable 3 (GIE3) */
#define MU_GIER_GIE4            (1 << 4)   /* Bit 4: MUB General Purpose Interrupt Enable 4 (GIE4) */
#define MU_GIER_GIE5            (1 << 5)   /* Bit 5: MUB General Purpose Interrupt Enable 5 (GIE5) */
#define MU_GIER_GIE6            (1 << 6)   /* Bit 6: MUB General Purpose Interrupt Enable 6 (GIE6) */
#define MU_GIER_GIE7            (1 << 7)   /* Bit 7: MUB General Purpose Interrupt Enable 7 (GIE7) */
#define MU_GIER_GIE8            (1 << 8)   /* Bit 8: MUB General Purpose Interrupt Enable 8 (GIE8) */
#define MU_GIER_GIE9            (1 << 9)   /* Bit 9: MUB General Purpose Interrupt Enable 9 (GIE9) */
#define MU_GIER_GIE10           (1 << 10)  /* Bit 10: MUB General Purpose Interrupt Enable 10 (GIE10) */
#define MU_GIER_GIE11           (1 << 11)  /* Bit 11: MUB General Purpose Interrupt Enable 11 (GIE11) */
#define MU_GIER_GIE12           (1 << 12)  /* Bit 12: MUB General Purpose Interrupt Enable 12 (GIE12) */
#define MU_GIER_GIE13           (1 << 13)  /* Bit 13: MUB General Purpose Interrupt Enable 13 (GIE13) */
#define MU_GIER_GIE14           (1 << 14)  /* Bit 14: MUB General Purpose Interrupt Enable 14 (GIE14) */
#define MU_GIER_GIE15           (1 << 15)  /* Bit 15: MUB General Purpose Interrupt Enable 15 (GIE15) */
#define MU_GIER_GIE16           (1 << 16)  /* Bit 16: MUB General Purpose Interrupt Enable 16 (GIE16) */
#define MU_GIER_GIE17           (1 << 17)  /* Bit 17: MUB General Purpose Interrupt Enable 17 (GIE17) */
#define MU_GIER_GIE18           (1 << 18)  /* Bit 18: MUB General Purpose Interrupt Enable 18 (GIE18) */
#define MU_GIER_GIE19           (1 << 19)  /* Bit 19: MUB General Purpose Interrupt Enable 19 (GIE19) */
#define MU_GIER_GIE20           (1 << 20)  /* Bit 20: MUB General Purpose Interrupt Enable 20 (GIE20) */
#define MU_GIER_GIE21           (1 << 21)  /* Bit 21: MUB General Purpose Interrupt Enable 21 (GIE21) */
#define MU_GIER_GIE22           (1 << 22)  /* Bit 22: MUB General Purpose Interrupt Enable 22 (GIE22) */
#define MU_GIER_GIE23           (1 << 23)  /* Bit 23: MUB General Purpose Interrupt Enable 23 (GIE23) */
#define MU_GIER_GIE24           (1 << 24)  /* Bit 24: MUB General Purpose Interrupt Enable 24 (GIE24) */
#define MU_GIER_GIE25           (1 << 25)  /* Bit 25: MUB General Purpose Interrupt Enable 25 (GIE25) */
#define MU_GIER_GIE26           (1 << 26)  /* Bit 26: MUB General Purpose Interrupt Enable 26 (GIE26) */
#define MU_GIER_GIE27           (1 << 27)  /* Bit 27: MUB General Purpose Interrupt Enable 27 (GIE27) */
#define MU_GIER_GIE28           (1 << 28)  /* Bit 28: MUB General Purpose Interrupt Enable 28 (GIE28) */
#define MU_GIER_GIE29           (1 << 29)  /* Bit 29: MUB General Purpose Interrupt Enable 29 (GIE29) */
#define MU_GIER_GIE30           (1 << 30)  /* Bit 30: MUB General Purpose Interrupt Enable 30 (GIE30) */
#define MU_GIER_GIE31           (1 << 31)  /* Bit 31: MUB General Purpose Interrupt Enable 31 (GIE31) */

/* General Control Register (GCR) */

#define MU_GCR_GIR(n)           (1 << (n)) /* Bit n: MUA/MUB General Purpose Interrupt Request n (GIRn) */
#define MU_GCR_GIR0             (1 << 0)   /* Bit 0: MUA/MUB General Purpose Interrupt Request 0 (GIR0) */
#define MU_GCR_GIR1             (1 << 1)   /* Bit 1: MUB General Purpose Interrupt Request 1 (GIR1) */
#define MU_GCR_GIR2             (1 << 2)   /* Bit 2: MUB General Purpose Interrupt Request 2 (GIR2) */
#define MU_GCR_GIR3             (1 << 3)   /* Bit 3: MUB General Purpose Interrupt Request 3 (GIR3) */
#define MU_GCR_GIR4             (1 << 4)   /* Bit 4: MUB General Purpose Interrupt Request 4 (GIR4) */
#define MU_GCR_GIR5             (1 << 5)   /* Bit 5: MUB General Purpose Interrupt Request 5 (GIR5) */
#define MU_GCR_GIR6             (1 << 6)   /* Bit 6: MUB General Purpose Interrupt Request 6 (GIR6) */
#define MU_GCR_GIR7             (1 << 7)   /* Bit 7: MUB General Purpose Interrupt Request 7 (GIR7) */
#define MU_GCR_GIR8             (1 << 8)   /* Bit 8: MUB General Purpose Interrupt Request 8 (GIR8) */
#define MU_GCR_GIR9             (1 << 9)   /* Bit 9: MUB General Purpose Interrupt Request 9 (GIR9) */
#define MU_GCR_GIR10            (1 << 10)  /* Bit 10: MUB General Purpose Interrupt Request 10 (GIR10) */
#define MU_GCR_GIR11            (1 << 11)  /* Bit 11: MUB General Purpose Interrupt Request 11 (GIR11) */
#define MU_GCR_GIR12            (1 << 12)  /* Bit 12: MUB General Purpose Interrupt Request 12 (GIR12) */
#define MU_GCR_GIR13            (1 << 13)  /* Bit 13: MUB General Purpose Interrupt Request 13 (GIR13) */
#define MU_GCR_GIR14            (1 << 14)  /* Bit 14: MUB General Purpose Interrupt Request 14 (GIR14) */
#define MU_GCR_GIR15            (1 << 15)  /* Bit 15: MUB General Purpose Interrupt Request 15 (GIR15) */
#define MU_GCR_GIR16            (1 << 16)  /* Bit 16: MUB General Purpose Interrupt Request 16 (GIR16) */
#define MU_GCR_GIR17            (1 << 17)  /* Bit 17: MUB General Purpose Interrupt Request 17 (GIR17) */
#define MU_GCR_GIR18            (1 << 18)  /* Bit 18: MUB General Purpose Interrupt Request 18 (GIR18) */
#define MU_GCR_GIR19            (1 << 19)  /* Bit 19: MUB General Purpose Interrupt Request 19 (GIR19) */
#define MU_GCR_GIR20            (1 << 20)  /* Bit 20: MUB General Purpose Interrupt Request 20 (GIR20) */
#define MU_GCR_GIR21            (1 << 21)  /* Bit 21: MUB General Purpose Interrupt Request 21 (GIR21) */
#define MU_GCR_GIR22            (1 << 22)  /* Bit 22: MUB General Purpose Interrupt Request 22 (GIR22) */
#define MU_GCR_GIR23            (1 << 23)  /* Bit 23: MUB General Purpose Interrupt Request 23 (GIR23) */
#define MU_GCR_GIR24            (1 << 24)  /* Bit 24: MUB General Purpose Interrupt Request 24 (GIR24) */
#define MU_GCR_GIR25            (1 << 25)  /* Bit 25: MUB General Purpose Interrupt Request 25 (GIR25) */
#define MU_GCR_GIR26            (1 << 26)  /* Bit 26: MUB General Purpose Interrupt Request 26 (GIR26) */
#define MU_GCR_GIR27            (1 << 27)  /* Bit 27: MUB General Purpose Interrupt Request 27 (GIR27) */
#define MU_GCR_GIR28            (1 << 28)  /* Bit 28: MUB General Purpose Interrupt Request 28 (GIR28) */
#define MU_GCR_GIR29            (1 << 29)  /* Bit 29: MUB General Purpose Interrupt Request 29 (GIR29) */
#define MU_GCR_GIR30            (1 << 30)  /* Bit 30: MUB General Purpose Interrupt Request 30 (GIR30) */
#define MU_GCR_GIR31            (1 << 31)  /* Bit 31: MUB General Purpose Interrupt Request 31 (GIR31) */

/* General Status Register (GSR) */

#define MU_GSR_GIP(n)           (1 << (n)) /* Bit n: MUA/MUB General Interrupt Request Pending n (GIPn) */
#define MU_GSR_GIP0             (1 << 0)   /* Bit 0: MUA/MUB General Interrupt Request Pending 0 (GIP0) */
#define MU_GSR_GIP1             (1 << 1)   /* Bit 1: MUB General Interrupt Request Pending 1 (GIP1) */
#define MU_GSR_GIP2             (1 << 2)   /* Bit 2: MUB General Interrupt Request Pending 2 (GIP2) */
#define MU_GSR_GIP3             (1 << 3)   /* Bit 3: MUB General Interrupt Request Pending 3 (GIP3) */
#define MU_GSR_GIP4             (1 << 4)   /* Bit 4: MUB General Interrupt Request Pending 4 (GIP4) */
#define MU_GSR_GIP5             (1 << 5)   /* Bit 5: MUB General Interrupt Request Pending 5 (GIP5) */
#define MU_GSR_GIP6             (1 << 6)   /* Bit 6: MUB General Interrupt Request Pending 6 (GIP6) */
#define MU_GSR_GIP7             (1 << 7)   /* Bit 7: MUB General Interrupt Request Pending 7 (GIP7) */
#define MU_GSR_GIP8             (1 << 8)   /* Bit 8: MUB General Interrupt Request Pending 8 (GIP8) */
#define MU_GSR_GIP9             (1 << 9)   /* Bit 9: MUB General Interrupt Request Pending 9 (GIP9) */
#define MU_GSR_GIP10            (1 << 10)  /* Bit 10: MUB General Interrupt Request Pending 10 (GIP10) */
#define MU_GSR_GIP11            (1 << 11)  /* Bit 11: MUB General Interrupt Request Pending 11 (GIP11) */
#define MU_GSR_GIP12            (1 << 12)  /* Bit 12: MUB General Interrupt Request Pending 12 (GIP12) */
#define MU_GSR_GIP13            (1 << 13)  /* Bit 13: MUB General Interrupt Request Pending 13 (GIP13) */
#define MU_GSR_GIP14            (1 << 14)  /* Bit 14: MUB General Interrupt Request Pending 14 (GIP14) */
#define MU_GSR_GIP15            (1 << 15)  /* Bit 15: MUB General Interrupt Request Pending 15 (GIP15) */
#define MU_GSR_GIP16            (1 << 16)  /* Bit 16: MUB General Interrupt Request Pending 16 (GIP16) */
#define MU_GSR_GIP17            (1 << 17)  /* Bit 17: MUB General Interrupt Request Pending 17 (GIP17) */
#define MU_GSR_GIP18            (1 << 18)  /* Bit 18: MUB General Interrupt Request Pending 18 (GIP18) */
#define MU_GSR_GIP19            (1 << 19)  /* Bit 19: MUB General Interrupt Request Pending 19 (GIP19) */
#define MU_GSR_GIP20            (1 << 20)  /* Bit 20: MUB General Interrupt Request Pending 20 (GIP20) */
#define MU_GSR_GIP21            (1 << 21)  /* Bit 21: MUB General Interrupt Request Pending 21 (GIP21) */
#define MU_GSR_GIP22            (1 << 22)  /* Bit 22: MUB General Interrupt Request Pending 22 (GIP22) */
#define MU_GSR_GIP23            (1 << 23)  /* Bit 23: MUB General Interrupt Request Pending 23 (GIP23) */
#define MU_GSR_GIP24            (1 << 24)  /* Bit 24: MUB General Interrupt Request Pending 24 (GIP24) */
#define MU_GSR_GIP25            (1 << 25)  /* Bit 25: MUB General Interrupt Request Pending 25 (GIP25) */
#define MU_GSR_GIP26            (1 << 26)  /* Bit 26: MUB General Interrupt Request Pending 26 (GIP26) */
#define MU_GSR_GIP27            (1 << 27)  /* Bit 27: MUB General Interrupt Request Pending 27 (GIP27) */
#define MU_GSR_GIP28            (1 << 28)  /* Bit 28: MUB General Interrupt Request Pending 28 (GIP28) */
#define MU_GSR_GIP29            (1 << 29)  /* Bit 29: MUB General Interrupt Request Pending 29 (GIP29) */
#define MU_GSR_GIP30            (1 << 30)  /* Bit 30: MUB General Interrupt Request Pending 30 (GIP30) */
#define MU_GSR_GIP31            (1 << 31)  /* Bit 31: MUB General Interrupt Request Pending 31 (GIP31) */

/* Transmit Control Register (TCR) */

#define MU_TCR_TIE_SHIFT        (0)        /* Bits 0-3: MU Transmit Interrupt Enable n (TIEn) */
#define MU_TCR_TIE_MASK         (0x0f << MU_TCR_TIE_SHIFT)
#  define MU_TCR_TIE0           (1 << 0)   /* Bit 0: MU Transmit Interrupt Enable 0 (TIE0) */
#  define MU_TCR_TIE1           (1 << 1)   /* Bit 1: MU Transmit Interrupt Enable 1 (TIE1) */
#  define MU_TCR_TIE2           (1 << 2)   /* Bit 2: MU Transmit Interrupt Enable 2 (TIE2) */
#  define MU_TCR_TIE3           (1 << 3)   /* Bit 3: MU Transmit Interrupt Enable 3 (TIE3) */
                                           /* Bits 4-31: Reserved */

/* Transmit Status Register (TSR) */

#define MU_TSR_TE_SHIFT         (0)        /* Bits 0-3: MU Transmit Register n Empty (TEn) */
#define MU_TSR_TE_MASK          (0x0f << MU_TSR_TE_SHIFT)
#  define MU_TSR_TE0            (1 << 0)   /* Bit 0: MU Transmit Register 0 Empty (TE0) */
#  define MU_TSR_TE1            (1 << 1)   /* Bit 1: MU Transmit Register 1 Empty (TE1) */
#  define MU_TSR_TE2            (1 << 2)   /* Bit 2: MU Transmit Register 2 Empty (TE2) */
#  define MU_TSR_TE3            (1 << 3)   /* Bit 3: MU Transmit Register 3 Empty (TE3) */
                                           /* Bits 4-31: Reserved */

/* Receive Control Register (RCR) */

#define MU_RCR_RIE_SHIFT        (0)        /* Bits 0-3: MU Receive Interrupt Enable n (RIEn) */
#define MU_RCR_RIE_MASK         (0x0f << MU_RCR_RIE_SHIFT)
#  define MU_RCR_RIE0           (1 << 0)   /* Bit 0: MUA Receive Interrupt Enable 0 (RIE0) */
#  define MU_RCR_RIE1           (1 << 1)   /* Bit 1: MUA Receive Interrupt Enable 1 (RIE1) */
#  define MU_RCR_RIE2           (1 << 2)   /* Bit 2: MUA Receive Interrupt Enable 2 (RIE2) */
#  define MU_RCR_RIE3           (1 << 3)   /* Bit 3: MUA Receive Interrupt Enable 3 (RIE3) */
                                           /* Bits 4-31: Reserved */

/* Receive Status Register (RSR) */

#define MU_RSR_RF_SHIFT         (0)        /* Bits 0-3: MU Receive Register n Full (RFn) */
#define MU_RSR_RF_MASK          (0x0f << MU_RSR_RF_SHIFT)
#  define MU_RSR_RF0            (1 << 0)   /* Bit 0: MUA Receive Register 0 Full (RF0) */
#  define MU_RSR_RF1            (1 << 1)   /* Bit 1: MUA Receive Register 1 Full (RF1) */
#  define MU_RSR_RF2            (1 << 2)   /* Bit 2: MUA Receive Register 2 Full (RF2) */
#  define MU_RSR_RF3            (1 << 3)   /* Bit 3: MUA Receive Register 3 Full (RF3) */
                                           /* Bits 4-31: Reserved */

/* Transmit Register (TRn) */

#define MU_TR_DATA_SHIFT        (0)        /* Bits 0-31: MU Transmit Data (TR_DATA) */
#define MU_TR_DATA_MASK         (0xffffffff << MU_TR_DATA_SHIFT)

/* Receive Register (RRn) */

#define MU_RR_DATA_SHIFT        (0)        /* Bits 0-31: MU Receive Data (RR_DATA) */
#define MU_RR_DATA_MASK         (0xffffffff << MU_RR_DATA_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MU_H */
