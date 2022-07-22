/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_pit.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PIT_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PIT Register Offsets *****************************************************/

#define S32K3XX_PIT_MCR_OFFSET            (0x0000) /* PIT Module Control Register (MCR) */
#define S32K3XX_PIT_LTMR64H_OFFSET        (0x00e0) /* PIT Upper Lifetime Timer Register (LTMR64H) */
#define S32K3XX_PIT_LTMR64L_OFFSET        (0x00e4) /* PIT Lower Lifetime Timer Register (LTMR64L) */
#define S32K3XX_PIT_RTI_LDVAL_STAT_OFFSET (0x00ec) /* RTI Timer Load Value Sync Status Register (RTI_LDVAL_STAT) */
#define S32K3XX_PIT_RTI_LDVAL_OFFSET      (0x00f0) /* Timer Load Value Register (RTI_LDVAL) */
#define S32K3XX_PIT_RTI_CVAL_OFFSET       (0x00f4) /* Current Timer Value Register (RTI_CVAL) */
#define S32K3XX_PIT_RTI_TCTRL_OFFSET      (0x00f8) /* Timer Control Register (RTI_TCTRL) */
#define S32K3XX_PIT_RTI_TFLG_OFFSET       (0x00fc) /* Timer Flag Register (RTI_TFLG) */
#define S32K3XX_PIT_LDVAL0_OFFSET         (0x0100) /* Timer Load Value Register 0 (LDVAL0) */
#define S32K3XX_PIT_CVAL0_OFFSET          (0x0104) /* Current Timer Value Register 0 (CVAL0) */
#define S32K3XX_PIT_TCTRL0_OFFSET         (0x0108) /* Timer Control Register 0 (TCTRL0) */
#define S32K3XX_PIT_TFLG0_OFFSET          (0x010c) /* Timer Flag Register 0 (TFLG0) */
#define S32K3XX_PIT_LDVAL1_OFFSET         (0x0110) /* Timer Load Value Register 1 (LDVAL1) */
#define S32K3XX_PIT_CVAL1_OFFSET          (0x0114) /* Current Timer Value Register 1 (CVAL1) */
#define S32K3XX_PIT_TCTRL1_OFFSET         (0x0118) /* Timer Control Register 1 (TCTRL1) */
#define S32K3XX_PIT_TFLG1_OFFSET          (0x011c) /* Timer Flag Register 1 (TFLG1) */
#define S32K3XX_PIT_LDVAL2_OFFSET         (0x0120) /* Timer Load Value Register 2 (LDVAL2) */
#define S32K3XX_PIT_CVAL2_OFFSET          (0x0124) /* Current Timer Value Register 2 (CVAL2) */
#define S32K3XX_PIT_TCTRL2_OFFSET         (0x0128) /* Timer Control Register 2 (TCTRL2) */
#define S32K3XX_PIT_TFLG2_OFFSET          (0x012c) /* Timer Flag Register 2 (TFLG2) */
#define S32K3XX_PIT_LDVAL3_OFFSET         (0x0130) /* Timer Load Value Register 3 (LDVAL3) */
#define S32K3XX_PIT_CVAL3_OFFSET          (0x0134) /* Current Timer Value Register 3 (CVAL3) */
#define S32K3XX_PIT_TCTRL3_OFFSET         (0x0138) /* Timer Control Register 3 (TCTRL3) */
#define S32K3XX_PIT_TFLG3_OFFSET          (0x013c) /* Timer Flag Register 3 (TFLG3) */

/* PIT Register Addresses ***************************************************/

#define S32K3XX_PIT0_MCR                  (S32K3XX_PIT0_BASE + S32K3XX_PIT_MCR_OFFSET)
#define S32K3XX_PIT0_LTMR64H              (S32K3XX_PIT0_BASE + S32K3XX_PIT_LTMR64H_OFFSET)
#define S32K3XX_PIT0_LTMR64L              (S32K3XX_PIT0_BASE + S32K3XX_PIT_LTMR64L_OFFSET)
#define S32K3XX_PIT0_RTI_LDVAL_STAT       (S32K3XX_PIT0_BASE + S32K3XX_PIT_RTI_LDVAL_STAT_OFFSET)
#define S32K3XX_PIT0_RTI_LDVAL            (S32K3XX_PIT0_BASE + S32K3XX_PIT_RTI_LDVAL_OFFSET)
#define S32K3XX_PIT0_RTI_CVAL             (S32K3XX_PIT0_BASE + S32K3XX_PIT_RTI_CVAL_OFFSET)
#define S32K3XX_PIT0_RTI_TCTRL            (S32K3XX_PIT0_BASE + S32K3XX_PIT_RTI_TCTRL_OFFSET)
#define S32K3XX_PIT0_RTI_TFLG             (S32K3XX_PIT0_BASE + S32K3XX_PIT_RTI_TFLG_OFFSET)
#define S32K3XX_PIT0_LDVAL0               (S32K3XX_PIT0_BASE + S32K3XX_PIT_LDVAL0_OFFSET)
#define S32K3XX_PIT0_CVAL0                (S32K3XX_PIT0_BASE + S32K3XX_PIT_CVAL0_OFFSET)
#define S32K3XX_PIT0_TCTRL0               (S32K3XX_PIT0_BASE + S32K3XX_PIT_TCTRL0_OFFSET)
#define S32K3XX_PIT0_TFLG0                (S32K3XX_PIT0_BASE + S32K3XX_PIT_TFLG0_OFFSET)
#define S32K3XX_PIT0_LDVAL1               (S32K3XX_PIT0_BASE + S32K3XX_PIT_LDVAL1_OFFSET)
#define S32K3XX_PIT0_CVAL1                (S32K3XX_PIT0_BASE + S32K3XX_PIT_CVAL1_OFFSET)
#define S32K3XX_PIT0_TCTRL1               (S32K3XX_PIT0_BASE + S32K3XX_PIT_TCTRL1_OFFSET)
#define S32K3XX_PIT0_TFLG1                (S32K3XX_PIT0_BASE + S32K3XX_PIT_TFLG1_OFFSET)
#define S32K3XX_PIT0_LDVAL2               (S32K3XX_PIT0_BASE + S32K3XX_PIT_LDVAL2_OFFSET)
#define S32K3XX_PIT0_CVAL2                (S32K3XX_PIT0_BASE + S32K3XX_PIT_CVAL2_OFFSET)
#define S32K3XX_PIT0_TCTRL2               (S32K3XX_PIT0_BASE + S32K3XX_PIT_TCTRL2_OFFSET)
#define S32K3XX_PIT0_TFLG2                (S32K3XX_PIT0_BASE + S32K3XX_PIT_TFLG2_OFFSET)
#define S32K3XX_PIT0_LDVAL3               (S32K3XX_PIT0_BASE + S32K3XX_PIT_LDVAL3_OFFSET)
#define S32K3XX_PIT0_CVAL3                (S32K3XX_PIT0_BASE + S32K3XX_PIT_CVAL3_OFFSET)
#define S32K3XX_PIT0_TCTRL3               (S32K3XX_PIT0_BASE + S32K3XX_PIT_TCTRL3_OFFSET)
#define S32K3XX_PIT0_TFLG3                (S32K3XX_PIT0_BASE + S32K3XX_PIT_TFLG3_OFFSET)

#define S32K3XX_PIT1_MCR                  (S32K3XX_PIT1_BASE + S32K3XX_PIT_MCR_OFFSET)
#define S32K3XX_PIT1_LTMR64H              (S32K3XX_PIT1_BASE + S32K3XX_PIT_LTMR64H_OFFSET)
#define S32K3XX_PIT1_LTMR64L              (S32K3XX_PIT1_BASE + S32K3XX_PIT_LTMR64L_OFFSET)
#define S32K3XX_PIT1_RTI_LDVAL_STAT       (S32K3XX_PIT1_BASE + S32K3XX_PIT_RTI_LDVAL_STAT_OFFSET)
#define S32K3XX_PIT1_RTI_LDVAL            (S32K3XX_PIT1_BASE + S32K3XX_PIT_RTI_LDVAL_OFFSET)
#define S32K3XX_PIT1_RTI_CVAL             (S32K3XX_PIT1_BASE + S32K3XX_PIT_RTI_CVAL_OFFSET)
#define S32K3XX_PIT1_RTI_TCTRL            (S32K3XX_PIT1_BASE + S32K3XX_PIT_RTI_TCTRL_OFFSET)
#define S32K3XX_PIT1_RTI_TFLG             (S32K3XX_PIT1_BASE + S32K3XX_PIT_RTI_TFLG_OFFSET)
#define S32K3XX_PIT1_LDVAL0               (S32K3XX_PIT1_BASE + S32K3XX_PIT_LDVAL0_OFFSET)
#define S32K3XX_PIT1_CVAL0                (S32K3XX_PIT1_BASE + S32K3XX_PIT_CVAL0_OFFSET)
#define S32K3XX_PIT1_TCTRL0               (S32K3XX_PIT1_BASE + S32K3XX_PIT_TCTRL0_OFFSET)
#define S32K3XX_PIT1_TFLG0                (S32K3XX_PIT1_BASE + S32K3XX_PIT_TFLG0_OFFSET)
#define S32K3XX_PIT1_LDVAL1               (S32K3XX_PIT1_BASE + S32K3XX_PIT_LDVAL1_OFFSET)
#define S32K3XX_PIT1_CVAL1                (S32K3XX_PIT1_BASE + S32K3XX_PIT_CVAL1_OFFSET)
#define S32K3XX_PIT1_TCTRL1               (S32K3XX_PIT1_BASE + S32K3XX_PIT_TCTRL1_OFFSET)
#define S32K3XX_PIT1_TFLG1                (S32K3XX_PIT1_BASE + S32K3XX_PIT_TFLG1_OFFSET)
#define S32K3XX_PIT1_LDVAL2               (S32K3XX_PIT1_BASE + S32K3XX_PIT_LDVAL2_OFFSET)
#define S32K3XX_PIT1_CVAL2                (S32K3XX_PIT1_BASE + S32K3XX_PIT_CVAL2_OFFSET)
#define S32K3XX_PIT1_TCTRL2               (S32K3XX_PIT1_BASE + S32K3XX_PIT_TCTRL2_OFFSET)
#define S32K3XX_PIT1_TFLG2                (S32K3XX_PIT1_BASE + S32K3XX_PIT_TFLG2_OFFSET)
#define S32K3XX_PIT1_LDVAL3               (S32K3XX_PIT1_BASE + S32K3XX_PIT_LDVAL3_OFFSET)
#define S32K3XX_PIT1_CVAL3                (S32K3XX_PIT1_BASE + S32K3XX_PIT_CVAL3_OFFSET)
#define S32K3XX_PIT1_TCTRL3               (S32K3XX_PIT1_BASE + S32K3XX_PIT_TCTRL3_OFFSET)
#define S32K3XX_PIT1_TFLG3                (S32K3XX_PIT1_BASE + S32K3XX_PIT_TFLG3_OFFSET)

#define S32K3XX_PIT2_MCR                  (S32K3XX_PIT2_BASE + S32K3XX_PIT_MCR_OFFSET)
#define S32K3XX_PIT2_LTMR64H              (S32K3XX_PIT2_BASE + S32K3XX_PIT_LTMR64H_OFFSET)
#define S32K3XX_PIT2_LTMR64L              (S32K3XX_PIT2_BASE + S32K3XX_PIT_LTMR64L_OFFSET)
#define S32K3XX_PIT2_RTI_LDVAL_STAT       (S32K3XX_PIT2_BASE + S32K3XX_PIT_RTI_LDVAL_STAT_OFFSET)
#define S32K3XX_PIT2_RTI_LDVAL            (S32K3XX_PIT2_BASE + S32K3XX_PIT_RTI_LDVAL_OFFSET)
#define S32K3XX_PIT2_RTI_CVAL             (S32K3XX_PIT2_BASE + S32K3XX_PIT_RTI_CVAL_OFFSET)
#define S32K3XX_PIT2_RTI_TCTRL            (S32K3XX_PIT2_BASE + S32K3XX_PIT_RTI_TCTRL_OFFSET)
#define S32K3XX_PIT2_RTI_TFLG             (S32K3XX_PIT2_BASE + S32K3XX_PIT_RTI_TFLG_OFFSET)
#define S32K3XX_PIT2_LDVAL0               (S32K3XX_PIT2_BASE + S32K3XX_PIT_LDVAL0_OFFSET)
#define S32K3XX_PIT2_CVAL0                (S32K3XX_PIT2_BASE + S32K3XX_PIT_CVAL0_OFFSET)
#define S32K3XX_PIT2_TCTRL0               (S32K3XX_PIT2_BASE + S32K3XX_PIT_TCTRL0_OFFSET)
#define S32K3XX_PIT2_TFLG0                (S32K3XX_PIT2_BASE + S32K3XX_PIT_TFLG0_OFFSET)
#define S32K3XX_PIT2_LDVAL1               (S32K3XX_PIT2_BASE + S32K3XX_PIT_LDVAL1_OFFSET)
#define S32K3XX_PIT2_CVAL1                (S32K3XX_PIT2_BASE + S32K3XX_PIT_CVAL1_OFFSET)
#define S32K3XX_PIT2_TCTRL1               (S32K3XX_PIT2_BASE + S32K3XX_PIT_TCTRL1_OFFSET)
#define S32K3XX_PIT2_TFLG1                (S32K3XX_PIT2_BASE + S32K3XX_PIT_TFLG1_OFFSET)
#define S32K3XX_PIT2_LDVAL2               (S32K3XX_PIT2_BASE + S32K3XX_PIT_LDVAL2_OFFSET)
#define S32K3XX_PIT2_CVAL2                (S32K3XX_PIT2_BASE + S32K3XX_PIT_CVAL2_OFFSET)
#define S32K3XX_PIT2_TCTRL2               (S32K3XX_PIT2_BASE + S32K3XX_PIT_TCTRL2_OFFSET)
#define S32K3XX_PIT2_TFLG2                (S32K3XX_PIT2_BASE + S32K3XX_PIT_TFLG2_OFFSET)
#define S32K3XX_PIT2_LDVAL3               (S32K3XX_PIT2_BASE + S32K3XX_PIT_LDVAL3_OFFSET)
#define S32K3XX_PIT2_CVAL3                (S32K3XX_PIT2_BASE + S32K3XX_PIT_CVAL3_OFFSET)
#define S32K3XX_PIT2_TCTRL3               (S32K3XX_PIT2_BASE + S32K3XX_PIT_TCTRL3_OFFSET)
#define S32K3XX_PIT2_TFLG3                (S32K3XX_PIT2_BASE + S32K3XX_PIT_TFLG3_OFFSET)

/* PIT Register Bitfield Definitions ****************************************/

/* PIT Module Control Register (MCR) */

#define PIT_MCR_FRZ                       (1 << 0)  /* Bit 0: Freeze (FRZ) */
#define PIT_MCR_MDIS                      (1 << 1)  /* Bit 1: Module Disable for PIT (MDIS) */
#define PIT_MCR_MDIS_RTI                  (1 << 2)  /* Bit 2: Module Disable for RTI (MDIS_RTI) */
                                                    /* Bits 3-31: Reserved */

/* PIT Upper Lifetime Timer Register (LTMR64H) */

#define PIT_LTMR64H_LTH_SHIFT             (0)       /* Bits 0-31: Life Timer Value (LTH) */
#define PIT_LTMR64H_LTH_MASK              (0xffffffff << PIT_LTMR64H_LTH_SHIFT)

/* PIT Lower Lifetime Timer Register (LTMR64L) */

#define PIT_LTMR64L_LTL_SHIFT             (0)       /* Bits 0-31: Life Timer Value (LTL) */
#define PIT_LTMR64L_LTL_MASK              (0xffffffff << PIT_LTMR64L_LTL_SHIFT)

/* RTI Timer Load Value Sync Status Register (RTI_LDVAL_STAT) */

#define PIT_RTI_LDVAL_STAT_RT_STAT        (1 << 0)  /* Bit 0: RTI Timer Load Value Sync Status (RT_STAT) */
                                                    /* Bits 1-31: Reserved */

/* Timer Load Value Register (RTI_LDVAL / LDVALn) */

#define PIT_LDVAL_TSV_SHIFT               (0)       /* Bits 0-31: Timer Start Value (TSV) */
#define PIT_LDVAL_TSV_MASK                (0xffffffff << PIT_LDVAL_TSV_SHIFT)

/* Current Timer Value Register (RTI_CVAL / CVALn) */

#define PIT_CVAL_TVL_SHIFT                (0)       /* Bits 0-31: Current Timer Value (TVL) */
#define PIT_CVAL_TVL_MASK                 (0xffffffff << PIT_CVAL_TVL_SHIFT)

/* Timer Control Register (RTI_TCTRL / TCTRLn) */

#define PIT_TCTRL_TEN                     (1 << 0)  /* Bit 0: Timer Enable (TEN) */
#define PIT_TCTRL_TIE                     (1 << 1)  /* Bit 1: Timer Interrupt Enable (TIE) */
#define PIT_TCTRL_CHN                     (1 << 2)  /* Bit 2: Chain Mode (CHN) */
                                                    /* Bits 3-31: Reserved */

/* Timer Flag Register (RTI_TFLG / TFLGn) */

#define PIT_TFLG_TIF                      (1 << 0)  /* Bit 0: Timer Interrupt Flag (TIF) */
                                                    /* Bits 1-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PIT_H */
