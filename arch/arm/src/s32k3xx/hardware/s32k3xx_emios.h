/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_emios.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_EMIOS_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_EMIOS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* eMIOS Register Offsets ***************************************************/

#define S32K3XX_EMIOS_MCR_OFFSET     (0x0000) /* Module Configuration Register (MCR) */
#define S32K3XX_EMIOS_GFLAG_OFFSET   (0x0004) /* Global Flag Register (GFLAG) */
#define S32K3XX_EMIOS_OUDIS_OFFSET   (0x0008) /* Output Update Disable Register (OUDIS) */
#define S32K3XX_EMIOS_UCDIS_OFFSET   (0x000c) /* Disable Channel Register (UCDIS) */

#define S32K3XX_EMIOS_A_OFFSET(n)    ((n) * 0x0020 + 0x0020) /* UC A n (An) */
#define S32K3XX_EMIOS_B_OFFSET(n)    ((n) * 0x0020 + 0x0024) /* UC B n (Bn) */
#define S32K3XX_EMIOS_CNT_OFFSET(n)  ((n) * 0x0020 + 0x0028) /* UC Counter n (CNTn) */
#define S32K3XX_EMIOS_C_OFFSET(n)    ((n) * 0x0020 + 0x002c) /* UC Control n (Cn) */
#define S32K3XX_EMIOS_S_OFFSET(n)    ((n) * 0x0020 + 0x0030) /* UC Status n (Sn) */
#define S32K3XX_EMIOS_ALTA_OFFSET(n) ((n) * 0x0020 + 0x0034) /* Alternate Address n (ALTAn) */
#define S32K3XX_EMIOS_C2_OFFSET(n)   ((n) * 0x0020 + 0x0038) /* UC Control 2 n (C2_n) */

/* eMIOS Register Addresses *************************************************/

#define S32K3XX_EMIOS0_MCR           (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_MCR_OFFSET)
#define S32K3XX_EMIOS0_GFLAG         (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_GFLAG_OFFSET)
#define S32K3XX_EMIOS0_OUDIS         (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_OUDIS_OFFSET)
#define S32K3XX_EMIOS0_UCDIS         (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_UCDIS_OFFSET)
#define S32K3XX_EMIOS0_A(n)          (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_A_OFFSET(n))
#define S32K3XX_EMIOS0_B(n)          (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_B_OFFSET(n))
#define S32K3XX_EMIOS0_CNT(n)        (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_CNT_OFFSET(n))
#define S32K3XX_EMIOS0_C(n)          (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_C_OFFSET(n))
#define S32K3XX_EMIOS0_S(n)          (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_S_OFFSET(n))
#define S32K3XX_EMIOS0_ALTA(n)       (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_ALTA_OFFSET(n))
#define S32K3XX_EMIOS0_C2(n)         (S32K3XX_EMIOS0_BASE + S32K3XX_EMIOS_C2_OFFSET(n))

#define S32K3XX_EMIOS1_MCR           (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_MCR_OFFSET)
#define S32K3XX_EMIOS1_GFLAG         (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_GFLAG_OFFSET)
#define S32K3XX_EMIOS1_OUDIS         (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_OUDIS_OFFSET)
#define S32K3XX_EMIOS1_UCDIS         (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_UCDIS_OFFSET)
#define S32K3XX_EMIOS1_A(n)          (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_A_OFFSET(n))
#define S32K3XX_EMIOS1_B(n)          (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_B_OFFSET(n))
#define S32K3XX_EMIOS1_CNT(n)        (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_CNT_OFFSET(n))
#define S32K3XX_EMIOS1_C(n)          (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_C_OFFSET(n))
#define S32K3XX_EMIOS1_S(n)          (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_S_OFFSET(n))
#define S32K3XX_EMIOS1_ALTA(n)       (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_ALTA_OFFSET(n))
#define S32K3XX_EMIOS1_C2(n)         (S32K3XX_EMIOS1_BASE + S32K3XX_EMIOS_C2_OFFSET(n))

#define S32K3XX_EMIOS2_MCR           (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_MCR_OFFSET)
#define S32K3XX_EMIOS2_GFLAG         (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_GFLAG_OFFSET)
#define S32K3XX_EMIOS2_OUDIS         (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_OUDIS_OFFSET)
#define S32K3XX_EMIOS2_UCDIS         (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_UCDIS_OFFSET)
#define S32K3XX_EMIOS2_A(n)          (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_A_OFFSET(n))
#define S32K3XX_EMIOS2_B(n)          (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_B_OFFSET(n))
#define S32K3XX_EMIOS2_CNT(n)        (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_CNT_OFFSET(n))
#define S32K3XX_EMIOS2_C(n)          (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_C_OFFSET(n))
#define S32K3XX_EMIOS2_S(n)          (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_S_OFFSET(n))
#define S32K3XX_EMIOS2_ALTA(n)       (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_ALTA_OFFSET(n))
#define S32K3XX_EMIOS2_C2(n)         (S32K3XX_EMIOS2_BASE + S32K3XX_EMIOS_C2_OFFSET(n))

/* eMIOS Register Bitfield Definitions **************************************/

/* Module Configuration Register (MCR) */

                                               /* Bits 0-7: Reserved */
#define EMIOS_MCR_GPRE_SHIFT         (8)       /* Bits 8-15: Global Prescaler (GPRE) */
#define EMIOS_MCR_GPRE_MASK          (0xff << EMIOS_MCR_GPRE_SHIFT)
#define EMIOS_MCR_GPRE(n)            (((n) << EMIOS_MCR_GPRE_SHIFT) & EMIOS_MCR_GPRE_MASK)
                                               /* Bits 16-25: Reserved */
#define EMIOS_MCR_GPREN              (1 << 26) /* Bit 26: Global Prescaler Enable (GPREN) */
                                               /* Bit 27: Reserved */
#define EMIOS_MCR_GTBE               (1 << 28) /* Bit 28: Global Timebase Enable (GTBE) */
#define EMIOS_MCR_FRZ                (1 << 29) /* Bit 29: Freeze (FRZ) */
#define EMIOS_MCR_MDIS               (1 << 30) /* Bit 30: Module Disable (MDIS) */
                                               /* Bit 31: Reserved */

/* Global Flag Register (GFLAG) */

#define EMIOS_GFLAG_F0               (1 << 0)  /* Bit 0: Mirror of UC 0 FLAG (F0) */
#define EMIOS_GFLAG_F1               (1 << 1)  /* Bit 1: Mirror of UC 1 FLAG (F1) */
#define EMIOS_GFLAG_F2               (1 << 2)  /* Bit 2: Mirror of UC 2 FLAG (F2) */
#define EMIOS_GFLAG_F3               (1 << 3)  /* Bit 3: Mirror of UC 3 FLAG (F3) */
#define EMIOS_GFLAG_F4               (1 << 4)  /* Bit 4: Mirror of UC 4 FLAG (F4) */
#define EMIOS_GFLAG_F5               (1 << 5)  /* Bit 5: Mirror of UC 5 FLAG (F5) */
#define EMIOS_GFLAG_F6               (1 << 6)  /* Bit 6: Mirror of UC 6 FLAG (F6) */
#define EMIOS_GFLAG_F7               (1 << 7)  /* Bit 7: Mirror of UC 7 FLAG (F7) */
#define EMIOS_GFLAG_F8               (1 << 8)  /* Bit 8: Mirror of UC 8 FLAG (F8) */
#define EMIOS_GFLAG_F9               (1 << 9)  /* Bit 9: Mirror of UC 9 FLAG (F9) */
#define EMIOS_GFLAG_F10              (1 << 10) /* Bit 10: Mirror of UC 10 FLAG (F10) */
#define EMIOS_GFLAG_F11              (1 << 11) /* Bit 11: Mirror of UC 11 FLAG (F11) */
#define EMIOS_GFLAG_F12              (1 << 12) /* Bit 12: Mirror of UC 12 FLAG (F12) */
#define EMIOS_GFLAG_F13              (1 << 13) /* Bit 13: Mirror of UC 13 FLAG (F13) */
#define EMIOS_GFLAG_F14              (1 << 14) /* Bit 14: Mirror of UC 14 FLAG (F14) */
#define EMIOS_GFLAG_F15              (1 << 15) /* Bit 15: Mirror of UC 15 FLAG (F15) */
#define EMIOS_GFLAG_F16              (1 << 16) /* Bit 16: Mirror of UC 16 FLAG (F16) */
#define EMIOS_GFLAG_F17              (1 << 17) /* Bit 17: Mirror of UC 17 FLAG (F17) */
#define EMIOS_GFLAG_F18              (1 << 18) /* Bit 18: Mirror of UC 18 FLAG (F18) */
#define EMIOS_GFLAG_F19              (1 << 19) /* Bit 19: Mirror of UC 19 FLAG (F19) */
#define EMIOS_GFLAG_F20              (1 << 20) /* Bit 20: Mirror of UC 20 FLAG (F20) */
#define EMIOS_GFLAG_F21              (1 << 21) /* Bit 21: Mirror of UC 21 FLAG (F21) */
#define EMIOS_GFLAG_F22              (1 << 22) /* Bit 22: Mirror of UC 22 FLAG (F22) */
#define EMIOS_GFLAG_F23              (1 << 23) /* Bit 23: Mirror of UC 23 FLAG (F23) */
                                               /* Bits 24-31: Reserved */

/* Output Update Disable Register (OUDIS) */

#define EMIOS_OUDIS_OU0              (1 << 0)   /* Bit 0: Channel 0 Output Update Disable (OU0) */
#define EMIOS_OUDIS_OU1              (1 << 1)   /* Bit 1: Channel 1 Output Update Disable (OU1) */
#define EMIOS_OUDIS_OU2              (1 << 2)   /* Bit 2: Channel 2 Output Update Disable (OU2) */
#define EMIOS_OUDIS_OU3              (1 << 3)   /* Bit 3: Channel 3 Output Update Disable (OU3) */
#define EMIOS_OUDIS_OU4              (1 << 4)   /* Bit 4: Channel 4 Output Update Disable (OU4) */
#define EMIOS_OUDIS_OU5              (1 << 5)   /* Bit 5: Channel 5 Output Update Disable (OU5) */
#define EMIOS_OUDIS_OU6              (1 << 6)   /* Bit 6: Channel 6 Output Update Disable (OU6) */
#define EMIOS_OUDIS_OU7              (1 << 7)   /* Bit 7: Channel 7 Output Update Disable (OU7) */
#define EMIOS_OUDIS_OU8              (1 << 8)   /* Bit 8: Channel 8 Output Update Disable (OU8) */
#define EMIOS_OUDIS_OU9              (1 << 9)   /* Bit 9: Channel 9 Output Update Disable (OU9) */
#define EMIOS_OUDIS_OU10             (1 << 10)  /* Bit 10: Channel 10 Output Update Disable (OU10) */
#define EMIOS_OUDIS_OU11             (1 << 11)  /* Bit 11: Channel 11 Output Update Disable (OU11) */
#define EMIOS_OUDIS_OU12             (1 << 12)  /* Bit 12: Channel 12 Output Update Disable (OU12) */
#define EMIOS_OUDIS_OU13             (1 << 13)  /* Bit 13: Channel 13 Output Update Disable (OU13) */
#define EMIOS_OUDIS_OU14             (1 << 14)  /* Bit 14: Channel 14 Output Update Disable (OU14) */
#define EMIOS_OUDIS_OU15             (1 << 15)  /* Bit 15: Channel 15 Output Update Disable (OU15) */
#define EMIOS_OUDIS_OU16             (1 << 16)  /* Bit 16: Channel 16 Output Update Disable (OU16) */
#define EMIOS_OUDIS_OU17             (1 << 17)  /* Bit 17: Channel 17 Output Update Disable (OU17) */
#define EMIOS_OUDIS_OU18             (1 << 18)  /* Bit 18: Channel 18 Output Update Disable (OU18) */
#define EMIOS_OUDIS_OU19             (1 << 19)  /* Bit 19: Channel 19 Output Update Disable (OU19) */
#define EMIOS_OUDIS_OU20             (1 << 20)  /* Bit 20: Channel 20 Output Update Disable (OU20) */
#define EMIOS_OUDIS_OU21             (1 << 21)  /* Bit 21: Channel 21 Output Update Disable (OU21) */
#define EMIOS_OUDIS_OU22             (1 << 22)  /* Bit 22: Channel 22 Output Update Disable (OU22) */
#define EMIOS_OUDIS_OU23             (1 << 23)  /* Bit 23: Channel 23 Output Update Disable (OU23) */
#define EMIOS_OUDIS_OU(n)            (1 << (n)) /* Bit n: Channel n Output Update Disable (OU23) */
                                                /* Bits 24-31: Reserved */

/* Disable Channel Register (UCDIS) */

#define EMIOS_UCDIS_OU0              (1 << 0)  /* Bit 0: Disable UC 0 (UCDIS0) */
#define EMIOS_UCDIS_OU1              (1 << 1)  /* Bit 1: Disable UC 1 (UCDIS1) */
#define EMIOS_UCDIS_OU2              (1 << 2)  /* Bit 2: Disable UC 2 (UCDIS2) */
#define EMIOS_UCDIS_OU3              (1 << 3)  /* Bit 3: Disable UC 3 (UCDIS3) */
#define EMIOS_UCDIS_OU4              (1 << 4)  /* Bit 4: Disable UC 4 (UCDIS4) */
#define EMIOS_UCDIS_OU5              (1 << 5)  /* Bit 5: Disable UC 5 (UCDIS5) */
#define EMIOS_UCDIS_OU6              (1 << 6)  /* Bit 6: Disable UC 6 (UCDIS6) */
#define EMIOS_UCDIS_OU7              (1 << 7)  /* Bit 7: Disable UC 7 (UCDIS7) */
#define EMIOS_UCDIS_OU8              (1 << 8)  /* Bit 8: Disable UC 8 (UCDIS8) */
#define EMIOS_UCDIS_OU9              (1 << 9)  /* Bit 9: Disable UC 9 (UCDIS9) */
#define EMIOS_UCDIS_OU10             (1 << 10) /* Bit 10: Disable UC 10 (UCDIS10) */
#define EMIOS_UCDIS_OU11             (1 << 11) /* Bit 11: Disable UC 11 (UCDIS11) */
#define EMIOS_UCDIS_OU12             (1 << 12) /* Bit 12: Disable UC 12 (UCDIS12) */
#define EMIOS_UCDIS_OU13             (1 << 13) /* Bit 13: Disable UC 13 (UCDIS13) */
#define EMIOS_UCDIS_OU14             (1 << 14) /* Bit 14: Disable UC 14 (UCDIS14) */
#define EMIOS_UCDIS_OU15             (1 << 15) /* Bit 15: Disable UC 15 (UCDIS15) */
#define EMIOS_UCDIS_OU16             (1 << 16) /* Bit 16: Disable UC 16 (UCDIS16) */
#define EMIOS_UCDIS_OU17             (1 << 17) /* Bit 17: Disable UC 17 (UCDIS17) */
#define EMIOS_UCDIS_OU18             (1 << 18) /* Bit 18: Disable UC 18 (UCDIS18) */
#define EMIOS_UCDIS_OU19             (1 << 19) /* Bit 19: Disable UC 19 (UCDIS19) */
#define EMIOS_UCDIS_OU20             (1 << 20) /* Bit 20: Disable UC 20 (UCDIS20) */
#define EMIOS_UCDIS_OU21             (1 << 21) /* Bit 21: Disable UC 21 (UCDIS21) */
#define EMIOS_UCDIS_OU22             (1 << 22) /* Bit 22: Disable UC 22 (UCDIS22) */
#define EMIOS_UCDIS_OU23             (1 << 23) /* Bit 23: Disable UC 23 (UCDIS23) */
                                               /* Bits 24-31: Reserved */

/* UC A n (An) */

#define EMIOS_A_SHIFT                (0)       /* Bits 0-15: A */
#define EMIOS_A_MASK                 (0xffff << EMIOS_A_SHIFT)
#define EMIOS_A(n)                   (((n) << EMIOS_A_SHIFT) & EMIOS_A_MASK)
                                               /* Bits 16-31: Reserved */

/* UC B n (Bn) */

#define EMIOS_B_SHIFT                (0)       /* Bits 0-15: B */
#define EMIOS_B_MASK                 (0xffff << EMIOS_B_SHIFT)
#define EMIOS_B(n)                   (((n) << EMIOS_B_SHIFT) & EMIOS_B_MASK)
                                               /* Bits 16-31: Reserved */

/* UC Counter n (CNTn) */

#define EMIOS_CNT_C_SHIFT            (0)       /* Bits 0-15: Internal Counter Value (C) */
#define EMIOS_CNT_C_MASK             (0xffff << EMIOS_CNT_C_SHIFT)
                                               /* Bits 16-31: Reserved */

/* UC Control n (Cn) */

#define EMIOS_C_MODE_SHIFT                         (0) /* Bits 0-6: Mode Selection (MODE) - NOTE: See S32K3XX Reference Manual for all options! */
#define EMIOS_C_MODE_MASK                          (0x7f << EMIOS_C_MODE_SHIFT)
#  define EMIOS_C_MODE_GPIN                        (0x00 << EMIOS_C_MODE_SHIFT) /* 000_0000: General-Purpose Input mode */
#  define EMIOS_C_MODE_GPOUT                       (0x01 << EMIOS_C_MODE_SHIFT) /* 000_0001: General-Purpose Output mode */
#  define EMIOS_C_MODE_SAIC                        (0x02 << EMIOS_C_MODE_SHIFT) /* 000_0010: Single Action Input Capture mode */
#  define EMIOS_C_MODE_SAIC_EDGE                   (0x42 << EMIOS_C_MODE_SHIFT) /* 100_0010: Single Action Input Capture mode (with edge capturing) */
#  define EMIOS_C_MODE_SAOC                        (0x03 << EMIOS_C_MODE_SHIFT) /* 000_0011: Single Action Output Capture mode */
#  define EMIOS_C_MODE_IPWM                        (0x04 << EMIOS_C_MODE_SHIFT) /* 000_0100: Input Pulse Width Measurement mode */
#  define EMIOS_C_MODE_IPM                         (0x05 << EMIOS_C_MODE_SHIFT) /* 000_0101: Input Period Measurement mode */
#  define EMIOS_C_MODE_DAOC_BMATCH                 (0x06 << EMIOS_C_MODE_SHIFT) /* 000_0110: Double Action Output Compare mode (with FLAG = 1 on B match) */
#  define EMIOS_C_MODE_DAOC_ABMATCH                (0x07 << EMIOS_C_MODE_SHIFT) /* 000_0111: Double Action Output Compare mode (with FLAG = 1 on A and B match) */
#  define EMIOS_C_MODE_PEC_CONT                    (0x0a << EMIOS_C_MODE_SHIFT) /* 000_1010: Pulse Edge Counting mode (continuous) */
#  define EMIOS_C_MODE_PEC_SINGLE                  (0x0b << EMIOS_C_MODE_SHIFT) /* 000_1011: Pulse Edge Counting mode (single-shot) */
#  define EMIOS_C_MODE_MC_UPCNT_CLRSTRT_INTCLK     (0x10 << EMIOS_C_MODE_SHIFT) /* 001_0000: Modulus Counter mode (up counter with clear on match start, internal clock source) */
#  define EMIOS_C_MODE_MC_UPCNT_CLRSTRT_EXTCLK     (0x11 << EMIOS_C_MODE_SHIFT) /* 001_0001: Modulus Counter mode (up counter with clear on match start, external clock source) */
#  define EMIOS_C_MODE_MC_UPCNT_CLREND_INTCLK      (0x12 << EMIOS_C_MODE_SHIFT) /* 001_0010: Modulus Counter mode (up counter with clear on match end, internal clock source) */
#  define EMIOS_C_MODE_MC_UPCNT_CLREND_EXTCLK      (0x13 << EMIOS_C_MODE_SHIFT) /* 001_0011: Modulus Counter mode (up counter with clear on match end, external clock source) */
#  define EMIOS_C_MODE_MC_UPDOWNCNT_CLRSTRT_INTCLK (0x14 << EMIOS_C_MODE_SHIFT) /* 001_0100: Modulus Counter mode (up/down counter with clear on match start, internal clock source) */
#  define EMIOS_C_MODE_MC_UPDOWNCNT_CLRSTRT_EXTCLK (0x15 << EMIOS_C_MODE_SHIFT) /* 001_0101: Modulus Counter mode (up/down counter with clear on match start, external clock source) */
#  define EMIOS_C_MODE_MC_UPDOWNCNT_CLREND_INTCLK  (0x16 << EMIOS_C_MODE_SHIFT) /* 001_0110: Modulus Counter mode (up/down counter with clear on match end, internal clock source) */
#  define EMIOS_C_MODE_MC_UPDOWNCNT_CLREND_EXTCLK  (0x17 << EMIOS_C_MODE_SHIFT) /* 001_0111: Modulus Counter mode (up/down counter with clear on match end, external clock source) */
#  define EMIOS_C_MODE_OPWMT                       (0x26 << EMIOS_C_MODE_SHIFT) /* 010_0110: Output PWM with Trigger mode */
#  define EMIOS_C_MODE_MCB_UPCNT_INTCLK            (0x50 << EMIOS_C_MODE_SHIFT) /* 101_0000: Modulus Counter Buffered mode (up counter, internal clock source) */
#  define EMIOS_C_MODE_MCB_UPCNT_EXTCLK            (0x51 << EMIOS_C_MODE_SHIFT) /* 101_0001: Modulus Counter Buffered mode (up counter, external clock source) */
#  define EMIOS_C_MODE_MCB_UPDOWNCNT_FSTRT_INTCLK  (0x54 << EMIOS_C_MODE_SHIFT) /* 101_0100: Modulus Counter Buffered mode (up/down counter with flag set on match start, internal clock source) */
#  define EMIOS_C_MODE_MCB_UPDOWNCNT_FSTRT_EXTCLK  (0x55 << EMIOS_C_MODE_SHIFT) /* 101_0101: Modulus Counter Buffered mode (up/down counter with flag set on match start, external clock source) */
#  define EMIOS_C_MODE_MCB_UPDOWNCNT_FBND_INTCLK   (0x56 << EMIOS_C_MODE_SHIFT) /* 101_0110: Modulus Counter Buffered mode (up/down counter with flag set on period boundary, internal clock source) */
#  define EMIOS_C_MODE_MCB_UPDOWNCNT_FBND_EXTCLK   (0x57 << EMIOS_C_MODE_SHIFT) /* 101_0111: Modulus Counter Buffered mode (up/down counter with flag set on period boundary, external clock source) */
#  define EMIOS_C_MODE_OPWFMB_BMATCH               (0x58 << EMIOS_C_MODE_SHIFT) /* 101_1000: Output Pulse Width and Frequency Modulation Buffered mode (BS1 match) */
#  define EMIOS_C_MODE_OPWFMB_ABMATCH              (0x5a << EMIOS_C_MODE_SHIFT) /* 101_1010: Output Pulse Width and Frequency Modulation Buffered mode (AS1 or BS1 match) */
#  define EMIOS_C_MODE_OPWMCB_TRAIL_FTRAIL         (0x5c << EMIOS_C_MODE_SHIFT) /* 101_1100: Center Aligned Output PWM with Dead Time Insertion Buffered mode (with trailing edge dead time, input capture flag asserted on trailing edge) */
#  define EMIOS_C_MODE_OPWMCB_TRAIL_FBOTH          (0x5e << EMIOS_C_MODE_SHIFT) /* 101_1110: Center Aligned Output PWM with Dead Time Insertion Buffered mode (with trailing edge dead time, input capture flag asserted on both edges) */
#  define EMIOS_C_MODE_OPWMCB_LEAD_FTRAIL          (0x5d << EMIOS_C_MODE_SHIFT) /* 101_1101: Center Aligned Output PWM with Dead Time Insertion Buffered mode (with leading edge dead time, input capture flag asserted on trailing edge) */
#  define EMIOS_C_MODE_OPWMCB_LEAD_FBOTH           (0x5f << EMIOS_C_MODE_SHIFT) /* 101_1111: Center Aligned Output PWM with Dead Time Insertion Buffered mode (with leading edge dead time, input capture flag asserted on both edges) */
#  define EMIOS_C_MODE_OPWMB_BMATCH                (0x60 << EMIOS_C_MODE_SHIFT) /* 110_0000: Output PWM Buffered mode (BS1 match) */
#  define EMIOS_C_MODE_OPWMB_ABMATCH               (0x62 << EMIOS_C_MODE_SHIFT) /* 110_0010: Output PWM Buffered mode (AS1 or BS1 match) */

#define EMIOS_C_EDPOL                (1 << 7)  /* Bit 7: Edge Polarity (EDPOL) */
#define EMIOS_C_EDSEL                (1 << 8)  /* Bit 8: Edge Selection (EDSEL) */
#define EMIOS_C_BSL_SHIFT            (9)       /* Bits 9-10: Bus Select (BSL) */
#define EMIOS_C_BSL_MASK             (0x03 << EMIOS_C_BSL_SHIFT)
#  define EMIOS_C_BSL_BUSA           (0x00 << EMIOS_C_BSL_SHIFT) /* Counter bus A for all channels */
#  define EMIOS_C_BSL_BUSBCD         (0x01 << EMIOS_C_BSL_SHIFT) /* Counter bus B for channels 0-7, C for 8-15, D for 16-23, E for 24-31 */
#  define EMIOS_C_BSL_BUSF           (0x02 << EMIOS_C_BSL_SHIFT) /* Counter bus F */
#  define EMIOS_C_BSL_INTCNT         (0x03 << EMIOS_C_BSL_SHIFT) /* Internal counter for all channels */

                                               /* Bit 11: Reserved */
#define EMIOS_C_FORCMB               (1 << 12) /* Bit 12: Force Match B (FORCMB) */
#define EMIOS_C_FORCMA               (1 << 13) /* Bit 13: Force Match A (FORCMA) */
                                               /* Bits 14-16: Reserved */
#define EMIOS_C_FEN                  (1 << 17) /* Bit 17: Flag Enable (FEN) */
#define EMIOS_C_FCK                  (1 << 18) /* Bit 18: Filter Clock Select (FCK) */
#define EMIOS_C_IF_SHIFT             (19)      /* Bits 19-22: Input Filter (IF) */
#define EMIOS_C_IF_MASK              (0x0f << EMIOS_C_IF_SHIFT)
#  define EMIOS_C_IF_BYPASS          (0x00 << EMIOS_C_IF_SHIFT) /* Bypassed. Input signal is synchronized before arriving at the digital filter. */
#  define EMIOS_C_IF_2CYCLES         (0x01 << EMIOS_C_IF_SHIFT) /* 2 Filter Clock Cycles */
#  define EMIOS_C_IF_4CYCLES         (0x02 << EMIOS_C_IF_SHIFT) /* 4 Filter Clock Cycles */
#  define EMIOS_C_IF_8CYCLES         (0x04 << EMIOS_C_IF_SHIFT) /* 8 Filter Clock Cycles */
#  define EMIOS_C_IF_16CYCLES        (0x08 << EMIOS_C_IF_SHIFT) /* 16 Filter Clock Cycles */

                                               /* Bit 23: Reserved */
#define EMIOS_C_DMA                  (1 << 24) /* Bit 24: Direct Memory Access (DMA) */
#define EMIOS_C_UCPREN               (1 << 25) /* Bit 25: Prescaler Enable (UCPREN) */
#define EMIOS_C_UCPRE_SHIFT          (26)      /* Bits 26-27: Prescaler (UCPRE) */
#define EMIOS_C_UCPRE_MASK           (0x03 << EMIOS_C_UCPRE_SHIFT)
#  define EMIOS_C_UCPRE_DIV1         (0x00 << EMIOS_C_UCPRE_SHIFT) /* Divide by 1 */
#  define EMIOS_C_UCPRE_DIV2         (0x01 << EMIOS_C_UCPRE_SHIFT) /* Divide by 2 */
#  define EMIOS_C_UCPRE_DIV3         (0x02 << EMIOS_C_UCPRE_SHIFT) /* Divide by 3 */
#  define EMIOS_C_UCPRE_DIV4         (0x03 << EMIOS_C_UCPRE_SHIFT) /* Divide by 4 */

#define EMIOS_C_ODISSL_SHIFT         (28)      /* Bits 28-29: Output Disable Select (ODISSL) */
#define EMIOS_C_ODISSL_MASK          (0x03 << EMIOS_C_ODISSL_SHIFT)
#  define EMIOS_C_ODISSL_IN0         (0x00 << EMIOS_C_ODISSL_SHIFT) /* Output Disable Input 0 */
#  define EMIOS_C_ODISSL_IN1         (0x01 << EMIOS_C_ODISSL_SHIFT) /* Output Disable Input 1 */
#  define EMIOS_C_ODISSL_IN2         (0x02 << EMIOS_C_ODISSL_SHIFT) /* Output Disable Input 2 */
#  define EMIOS_C_ODISSL_IN3         (0x03 << EMIOS_C_ODISSL_SHIFT) /* Output Disable Input 3 */

#define EMIOS_C_ODIS                 (1 << 30) /* Bit 30: Output Disable (ODIS) */
#define EMIOS_C_FREN                 (1 << 31) /* Bit 31: Freeze Enable (FREN) */

/* UC Status n (Sn) */

#define EMIOS_S_FLAG                 (1 << 0)  /* Bit 0: Flag indicating input capture or match event (FLAG) */
#define EMIOS_S_UCOUT                (1 << 1)  /* Bit 1: UC Output Pin state (UCOUT) */
#define EMIOS_S_UCIN                 (1 << 2)  /* Bit 2: UC Input Pin state (UCIN) */
                                               /* Bits 3-14: Reserved */
#define EMIOS_S_OVFL                 (1 << 15) /* Bit 15: Overflow (OVFL) */
                                               /* Bits 16-30: Reserved */
#define EMIOS_S_OVR                  (1 << 31) /* Bit 31: Overrun (OVR) */

/* Alternate Address n (ALTAn) */

#define EMIOS_ALTA_SHIFT             (0)       /* Bits 0-15: Alternate Address */
#define EMIOS_ALTA_MASK              (0xffff << EMIOS_ALTA_SHIFT)
                                               /* Bits 16-31: Reserved */

/* UC Control 2 n (C2_n) */

#define EMIOS_C2_UCRELDEL_INT_SHIFT  (0)       /* Bits 0-4: Reload Signal Output Delay Interval */
#define EMIOS_C2_UCRELDEL_INT_MASK   (0x1f << EMIOS_C2_UCRELDEL_INT_SHIFT)
                                               /* Bits 5-13: Reserved */
#define EMIOS_C2_UCPRECLK            (1 << 14) /* Bit 14: Prescaler Clock Source (UCPRECLK) */
                                               /* Bit 15: Reserved */
#define EMIOS_C2_UCEXTPRE_SHIFT      (16)      /* Bits 16-19: Extended Prescaler (UCEXTPRE) */ 
#define EMIOS_C2_UCEXTPRE_MASK       (0x0f << EMIOS_C2_UCEXTPRE_SHIFT)
#define EMIOS_C2_UCEXTPRE(n)         (((n) << EMIOS_C2_UCEXTPRE_SHIFT) & EMIOS_C2_UCEXTPRE_MASK)
                                               /* Bits 20-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_EMIOS_H */
