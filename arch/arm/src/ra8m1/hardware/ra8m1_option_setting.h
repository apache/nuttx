/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_option_setting.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_OPTION_SETTING_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_OPTION_SETTING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Option-setting memory registers (RA8M1 User's Manual, chapter 6).
 *
 * These are not peripheral registers: they are words in the code-flash
 * configuration area that the MCU samples at reset, programmed together
 * with the image (FACI command / debugger), and read-only at run time.
 * The linker script places each one by section name; there are no run-time
 * addresses here.
 *
 * This is a flat (no TrustZone) image, so only the secure-region registers
 * are used.  OFS1 below is the register at 0x0300_A200 (called OFS1_SEC in
 * the manual); the non-secure OFS1 at 0x1300_A180 is not used.
 *
 * Bits documented as "The program value is read from this bit" are
 * reserved: they must be programmed as 1 (note 2 of each register).
 */

/* Option Function Select Register 0 (OFS0) -- address 0x0300_A100 **********/

#define R_OFS0_RESERVED                     0xa001a001  /* Bits 31, 29, 16, 15, 13, 0 */

/* Independent Watchdog Timer (IWDT) ****************************************/

#define R_OFS0_IWDTSTRT                     (1 <<  1)   /* 0: Auto start after reset, 1: Register start (stopped) */

#define R_OFS0_IWDTTOPS_SHIFT               (2)         /* Bits 3-2: Timeout period select */
#define R_OFS0_IWDTTOPS_MASK                (0x3)
#  define R_OFS0_IWDTTOPS_128               (0 << R_OFS0_IWDTTOPS_SHIFT)   /* 128 cycles */
#  define R_OFS0_IWDTTOPS_512               (1 << R_OFS0_IWDTTOPS_SHIFT)   /* 512 cycles */
#  define R_OFS0_IWDTTOPS_1024              (2 << R_OFS0_IWDTTOPS_SHIFT)   /* 1024 cycles */
#  define R_OFS0_IWDTTOPS_2048              (3 << R_OFS0_IWDTTOPS_SHIFT)   /* 2048 cycles */

#define R_OFS0_IWDTCKS_SHIFT                (4)         /* Bits 7-4: Clock division ratio select */
#define R_OFS0_IWDTCKS_MASK                 (0xf)
#  define R_OFS0_IWDTCKS_DIV1               (0x0 << R_OFS0_IWDTCKS_SHIFT)  /* x 1 */
#  define R_OFS0_IWDTCKS_DIV16              (0x2 << R_OFS0_IWDTCKS_SHIFT)  /* x 1/16 */
#  define R_OFS0_IWDTCKS_DIV32              (0x3 << R_OFS0_IWDTCKS_SHIFT)  /* x 1/32 */
#  define R_OFS0_IWDTCKS_DIV64              (0x4 << R_OFS0_IWDTCKS_SHIFT)  /* x 1/64 */
#  define R_OFS0_IWDTCKS_DIV128             (0xf << R_OFS0_IWDTCKS_SHIFT)  /* x 1/128 */
#  define R_OFS0_IWDTCKS_DIV256             (0x5 << R_OFS0_IWDTCKS_SHIFT)  /* x 1/256 */

#define R_OFS0_IWDTRPES_SHIFT               (8)         /* Bits 9-8: Window end position select */
#define R_OFS0_IWDTRPES_MASK                (0x3)
#  define R_OFS0_IWDTRPES_75                (0 << R_OFS0_IWDTRPES_SHIFT)   /* 75% */
#  define R_OFS0_IWDTRPES_50                (1 << R_OFS0_IWDTRPES_SHIFT)   /* 50% */
#  define R_OFS0_IWDTRPES_25                (2 << R_OFS0_IWDTRPES_SHIFT)   /* 25% */
#  define R_OFS0_IWDTRPES_0                 (3 << R_OFS0_IWDTRPES_SHIFT)   /* 0% (no window end position) */

#define R_OFS0_IWDTRPSS_SHIFT               (10)        /* Bits 11-10: Window start position select */
#define R_OFS0_IWDTRPSS_MASK                (0x3)
#  define R_OFS0_IWDTRPSS_25                (0 << R_OFS0_IWDTRPSS_SHIFT)   /* 25% */
#  define R_OFS0_IWDTRPSS_50                (1 << R_OFS0_IWDTRPSS_SHIFT)   /* 50% */
#  define R_OFS0_IWDTRPSS_75                (2 << R_OFS0_IWDTRPSS_SHIFT)   /* 75% */
#  define R_OFS0_IWDTRPSS_100               (3 << R_OFS0_IWDTRPSS_SHIFT)   /* 100% (no window start position) */

#define R_OFS0_IWDTRSTIRQS                  (1 << 12)   /* 0: Interrupt, 1: Reset on underflow/refresh error */
#define R_OFS0_IWDTSTPCTL                   (1 << 14)   /* 0: Continue counting, 1: Stop counting in low power modes */

/* Watchdog Timer (WDT0) ****************************************************/

#define R_OFS0_WDT0STRT                     (1 << 17)   /* 0: Auto start after reset, 1: Register start (stopped) */

#define R_OFS0_WDT0TOPS_SHIFT               (18)        /* Bits 19-18: Timeout period select */
#define R_OFS0_WDT0TOPS_MASK                (0x3)
#  define R_OFS0_WDT0TOPS_1024              (0 << R_OFS0_WDT0TOPS_SHIFT)   /* 1024 cycles */
#  define R_OFS0_WDT0TOPS_4096              (1 << R_OFS0_WDT0TOPS_SHIFT)   /* 4096 cycles */
#  define R_OFS0_WDT0TOPS_8192              (2 << R_OFS0_WDT0TOPS_SHIFT)   /* 8192 cycles */
#  define R_OFS0_WDT0TOPS_16384             (3 << R_OFS0_WDT0TOPS_SHIFT)   /* 16384 cycles */

#define R_OFS0_WDT0CKS_SHIFT                (20)        /* Bits 23-20: Clock division ratio select */
#define R_OFS0_WDT0CKS_MASK                 (0xf)
#  define R_OFS0_WDT0CKS_DIV4               (0x1 << R_OFS0_WDT0CKS_SHIFT)  /* PCLKB / 4 */
#  define R_OFS0_WDT0CKS_DIV64              (0x4 << R_OFS0_WDT0CKS_SHIFT)  /* PCLKB / 64 */
#  define R_OFS0_WDT0CKS_DIV128             (0xf << R_OFS0_WDT0CKS_SHIFT)  /* PCLKB / 128 */
#  define R_OFS0_WDT0CKS_DIV512             (0x6 << R_OFS0_WDT0CKS_SHIFT)  /* PCLKB / 512 */
#  define R_OFS0_WDT0CKS_DIV2048            (0x7 << R_OFS0_WDT0CKS_SHIFT)  /* PCLKB / 2048 */
#  define R_OFS0_WDT0CKS_DIV8192            (0x8 << R_OFS0_WDT0CKS_SHIFT)  /* PCLKB / 8192 */

#define R_OFS0_WDT0RPES_SHIFT               (24)        /* Bits 25-24: Window end position select */
#define R_OFS0_WDT0RPES_MASK                (0x3)
#  define R_OFS0_WDT0RPES_75                (0 << R_OFS0_WDT0RPES_SHIFT)   /* 75% */
#  define R_OFS0_WDT0RPES_50                (1 << R_OFS0_WDT0RPES_SHIFT)   /* 50% */
#  define R_OFS0_WDT0RPES_25                (2 << R_OFS0_WDT0RPES_SHIFT)   /* 25% */
#  define R_OFS0_WDT0RPES_0                 (3 << R_OFS0_WDT0RPES_SHIFT)   /* 0% (no window end position) */

#define R_OFS0_WDT0RPSS_SHIFT               (26)        /* Bits 27-26: Window start position select */
#define R_OFS0_WDT0RPSS_MASK                (0x3)
#  define R_OFS0_WDT0RPSS_25                (0 << R_OFS0_WDT0RPSS_SHIFT)   /* 25% */
#  define R_OFS0_WDT0RPSS_50                (1 << R_OFS0_WDT0RPSS_SHIFT)   /* 50% */
#  define R_OFS0_WDT0RPSS_75                (2 << R_OFS0_WDT0RPSS_SHIFT)   /* 75% */
#  define R_OFS0_WDT0RPSS_100               (3 << R_OFS0_WDT0RPSS_SHIFT)   /* 100% (no window start position) */

#define R_OFS0_WDT0RSTIRQS                  (1 << 28)   /* 0: Interrupt, 1: Reset on underflow/refresh error */
#define R_OFS0_WDT0STPCTL                   (1 << 30)   /* 0: Continue counting, 1: Stop counting in CPU Sleep/Deep Sleep */

/* Option Function Select Register 1 (OFS1) -- address 0x0300_A200 **********/

/* Called OFS1_SEC in the manual. *******************************************/

#define R_OFS1_RESERVED                     0xfcfff0d0  /* Bits 31-26, 23-12, 7-6, 4 */

#define R_OFS1_VDSEL_SHIFT                  (0)         /* Bits 2-0: Voltage detection 0 level select */
#define R_OFS1_VDSEL_MASK                   (0x7)
#  define R_OFS1_VDSEL_2_85V                (0 << R_OFS1_VDSEL_SHIFT)      /* 2.85 V */
#  define R_OFS1_VDSEL_2_58V                (1 << R_OFS1_VDSEL_SHIFT)      /* 2.58 V */
#  define R_OFS1_VDSEL_2_15V                (2 << R_OFS1_VDSEL_SHIFT)      /* 2.15 V */
#  define R_OFS1_VDSEL_2_00V                (3 << R_OFS1_VDSEL_SHIFT)      /* 2.00 V */
#  define R_OFS1_VDSEL_1_90V                (4 << R_OFS1_VDSEL_SHIFT)      /* 1.90 V */
#  define R_OFS1_VDSEL_1_80V                (5 << R_OFS1_VDSEL_SHIFT)      /* 1.80 V */
#  define R_OFS1_VDSEL_1_70V                (6 << R_OFS1_VDSEL_SHIFT)      /* 1.70 V */
#  define R_OFS1_VDSEL_1_60V                (7 << R_OFS1_VDSEL_SHIFT)      /* 1.60 V */

#define R_OFS1_PVDAS                        (1 <<  3)   /* 0: Enable voltage monitor 0 reset after reset, 1: Disable */
#define R_OFS1_PVDLPSEL                     (1 <<  5)   /* 0: Enable PVD0 low power function in DSTBY1/2, 1: Disable */
#define R_OFS1_HOCOEN                       (1 <<  8)   /* 0: Enable HOCO oscillation after reset, 1: Disable */

#define R_OFS1_HOCOFRQ0_SHIFT               (9)         /* Bits 11-9: HOCO frequency setting 0 */
#define R_OFS1_HOCOFRQ0_MASK                (0x7)
#  define R_OFS1_HOCOFRQ0_16MHZ             (0 << R_OFS1_HOCOFRQ0_SHIFT)   /* 16 MHz */
#  define R_OFS1_HOCOFRQ0_18MHZ             (1 << R_OFS1_HOCOFRQ0_SHIFT)   /* 18 MHz */
#  define R_OFS1_HOCOFRQ0_20MHZ             (2 << R_OFS1_HOCOFRQ0_SHIFT)   /* 20 MHz */
#  define R_OFS1_HOCOFRQ0_32MHZ             (4 << R_OFS1_HOCOFRQ0_SHIFT)   /* 32 MHz */
#  define R_OFS1_HOCOFRQ0_48MHZ             (7 << R_OFS1_HOCOFRQ0_SHIFT)   /* 48 MHz */

#define R_OFS1_SWDBG                        (1 << 24)   /* 0: Enable software debug control, 1: Disable */
#define R_OFS1_INITECCEN                    (1 << 25)   /* 0: Disable TCM/cache ECC, 1: Enable */

/* Option Function Select Register 2 (OFS2) -- address 0x0300_A104 **********/

#define R_OFS2_RESERVED                     0xfffffffe  /* Bits 31-1 */

#define R_OFS2_DCDCEN                       (1 <<  0)   /* 0: Disable DCDC, 1: Enable DCDC */

/* OFS1 Security Attribution Register (OFS1_SEL) -- address 0x0300_A280 *****/

/* Each bit selects whether the matching OFS1 field comes from OFS1_SEC (0)
 * or from the non-secure OFS1 (1).  A flat image uses only OFS1 above, so
 * the whole register is programmed as 0.
 */

#define R_OFS1_SEL_ALL_SECURE               0x00000000

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_OPTION_SETTING_H */
