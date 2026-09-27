/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h7_tim.h
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32_TIM_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/* The N32H7 timer family includes:
 *   - 4x Advanced Timers (ATIM1-4)
 *   - 7x General-purpose Timers A (GTIMA1-7)
 *   - 3x General-purpose Timers B (GTIMB1-3)
 *   - 4x Basic Timers (BTIM1-4)
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ==========================================================================
 *                          ADVANCED TIMERS (ATIM1-4)
 * ==========================================================================
 */

/* Register Offsets - ATIM (common for all ATIM1-4) *************************/

#define N32_ATIM_CTRL1_OFFSET      0x0000  /* Control register 1 */
#define N32_ATIM_CTRL2_OFFSET      0x0004  /* Control register 2 */
#define N32_ATIM_STS_OFFSET        0x0008  /* Status register */
#define N32_ATIM_ETGEN_OFFSET      0x000c  /* Event generation register */
#define N32_ATIM_SMCTRL_OFFSET     0x0010  /* Slave mode control register */
#define N32_ATIM_DINTEN_OFFSET     0x0014  /* DMA/Interrupt enable register */
#define N32_ATIM_CCMOD1_OFFSET     0x0018  /* Capture/compare mode register 1 */
#define N32_ATIM_CCMOD2_OFFSET     0x001c  /* Capture/compare mode register 2 */
#define N32_ATIM_CCMOD3_OFFSET     0x0020  /* Capture/compare mode register 3 */
#define N32_ATIM_CCEN_OFFSET       0x0024  /* Capture/compare enable register */
#define N32_ATIM_CCDAT1_OFFSET     0x0028  /* Capture/compare register 1 */
#define N32_ATIM_CCDAT2_OFFSET     0x002c  /* Capture/compare register 2 */
#define N32_ATIM_CCDAT3_OFFSET     0x0030  /* Capture/compare register 3 */
#define N32_ATIM_CCDAT4_OFFSET     0x0034  /* Capture/compare register 4 */
#define N32_ATIM_CCDAT5_OFFSET     0x0038  /* Capture/compare register 5 */
#define N32_ATIM_CCDAT6_OFFSET     0x003c  /* Capture/compare register 6 */
#define N32_ATIM_PSC_OFFSET        0x0040  /* Prescaler */
#define N32_ATIM_AR_OFFSET         0x0044  /* Auto-reload register */
#define N32_ATIM_CNT_OFFSET        0x0048  /* Counter */
#define N32_ATIM_REPCNT_OFFSET     0x004c  /* Repetition counter register */
#define N32_ATIM_BKDT_OFFSET       0x0050  /* Break and dead-time register */
#define N32_ATIM_CCDAT7_OFFSET     0x0054  /* Capture/compare register 7 */
#define N32_ATIM_CCDAT8_OFFSET     0x0058  /* Capture/compare register 8 */
#define N32_ATIM_CCDAT9_OFFSET     0x005c  /* Capture/compare register 9 */
#define N32_ATIM_BKFR_OFFSET       0x0060  /* Break 1 filter register */
#define N32_ATIM_INSEL_OFFSET      0x0078  /* Input selection register */
#define N32_ATIM_AF1_OFFSET        0x007c  /* Alternate function register 1 */
#define N32_ATIM_AF2_OFFSET        0x0080  /* Alternate function register 2 */
#define N32_ATIM_BKFR2_OFFSET      0x0084  /* Break 2 filter register */
#define N32_ATIM_DCTRL_OFFSET      0x0094  /* DMA control register */
#define N32_ATIM_DADDR_OFFSET      0x0098  /* DMA address for burst mode */

/* Register Addresses - ATIM1 ***********************************************/

#define N32_ATIM1_CTRL1            (N32_ATIMER1_BASE + N32_ATIM_CTRL1_OFFSET)
#define N32_ATIM1_CTRL2            (N32_ATIMER1_BASE + N32_ATIM_CTRL2_OFFSET)
#define N32_ATIM1_STS              (N32_ATIMER1_BASE + N32_ATIM_STS_OFFSET)
#define N32_ATIM1_ETGEN            (N32_ATIMER1_BASE + N32_ATIM_ETGEN_OFFSET)
#define N32_ATIM1_SMCTRL           (N32_ATIMER1_BASE + N32_ATIM_SMCTRL_OFFSET)
#define N32_ATIM1_DINTEN           (N32_ATIMER1_BASE + N32_ATIM_DINTEN_OFFSET)
#define N32_ATIM1_CCMOD1           (N32_ATIMER1_BASE + N32_ATIM_CCMOD1_OFFSET)
#define N32_ATIM1_CCMOD2           (N32_ATIMER1_BASE + N32_ATIM_CCMOD2_OFFSET)
#define N32_ATIM1_CCMOD3           (N32_ATIMER1_BASE + N32_ATIM_CCMOD3_OFFSET)
#define N32_ATIM1_CCEN             (N32_ATIMER1_BASE + N32_ATIM_CCEN_OFFSET)
#define N32_ATIM1_CCDAT1           (N32_ATIMER1_BASE + N32_ATIM_CCDAT1_OFFSET)
#define N32_ATIM1_CCDAT2           (N32_ATIMER1_BASE + N32_ATIM_CCDAT2_OFFSET)
#define N32_ATIM1_CCDAT3           (N32_ATIMER1_BASE + N32_ATIM_CCDAT3_OFFSET)
#define N32_ATIM1_CCDAT4           (N32_ATIMER1_BASE + N32_ATIM_CCDAT4_OFFSET)
#define N32_ATIM1_CCDAT5           (N32_ATIMER1_BASE + N32_ATIM_CCDAT5_OFFSET)
#define N32_ATIM1_CCDAT6           (N32_ATIMER1_BASE + N32_ATIM_CCDAT6_OFFSET)
#define N32_ATIM1_PSC              (N32_ATIMER1_BASE + N32_ATIM_PSC_OFFSET)
#define N32_ATIM1_AR               (N32_ATIMER1_BASE + N32_ATIM_AR_OFFSET)
#define N32_ATIM1_CNT              (N32_ATIMER1_BASE + N32_ATIM_CNT_OFFSET)
#define N32_ATIM1_REPCNT           (N32_ATIMER1_BASE + N32_ATIM_REPCNT_OFFSET)
#define N32_ATIM1_BKDT             (N32_ATIMER1_BASE + N32_ATIM_BKDT_OFFSET)
#define N32_ATIM1_CCDAT7           (N32_ATIMER1_BASE + N32_ATIM_CCDAT7_OFFSET)
#define N32_ATIM1_CCDAT8           (N32_ATIMER1_BASE + N32_ATIM_CCDAT8_OFFSET)
#define N32_ATIM1_CCDAT9           (N32_ATIMER1_BASE + N32_ATIM_CCDAT9_OFFSET)
#define N32_ATIM1_BKFR             (N32_ATIMER1_BASE + N32_ATIM_BKFR_OFFSET)
#define N32_ATIM1_INSEL            (N32_ATIMER1_BASE + N32_ATIM_INSEL_OFFSET)
#define N32_ATIM1_AF1              (N32_ATIMER1_BASE + N32_ATIM_AF1_OFFSET)
#define N32_ATIM1_AF2              (N32_ATIMER1_BASE + N32_ATIM_AF2_OFFSET)
#define N32_ATIM1_BKFR2            (N32_ATIMER1_BASE + N32_ATIM_BKFR2_OFFSET)
#define N32_ATIM1_DCTRL            (N32_ATIMER1_BASE + N32_ATIM_DCTRL_OFFSET)
#define N32_ATIM1_DADDR            (N32_ATIMER1_BASE + N32_ATIM_DADDR_OFFSET)

/* Register Addresses - ATIM2 ***********************************************/

#define N32_ATIM2_CTRL1            (N32_ATIMER2_BASE + N32_ATIM_CTRL1_OFFSET)
#define N32_ATIM2_CTRL2            (N32_ATIMER2_BASE + N32_ATIM_CTRL2_OFFSET)
#define N32_ATIM2_STS              (N32_ATIMER2_BASE + N32_ATIM_STS_OFFSET)
#define N32_ATIM2_ETGEN            (N32_ATIMER2_BASE + N32_ATIM_ETGEN_OFFSET)
#define N32_ATIM2_SMCTRL           (N32_ATIMER2_BASE + N32_ATIM_SMCTRL_OFFSET)
#define N32_ATIM2_DINTEN           (N32_ATIMER2_BASE + N32_ATIM_DINTEN_OFFSET)
#define N32_ATIM2_CCMOD1           (N32_ATIMER2_BASE + N32_ATIM_CCMOD1_OFFSET)
#define N32_ATIM2_CCMOD2           (N32_ATIMER2_BASE + N32_ATIM_CCMOD2_OFFSET)
#define N32_ATIM2_CCMOD3           (N32_ATIMER2_BASE + N32_ATIM_CCMOD3_OFFSET)
#define N32_ATIM2_CCEN             (N32_ATIMER2_BASE + N32_ATIM_CCEN_OFFSET)
#define N32_ATIM2_CCDAT1           (N32_ATIMER2_BASE + N32_ATIM_CCDAT1_OFFSET)
#define N32_ATIM2_CCDAT2           (N32_ATIMER2_BASE + N32_ATIM_CCDAT2_OFFSET)
#define N32_ATIM2_CCDAT3           (N32_ATIMER2_BASE + N32_ATIM_CCDAT3_OFFSET)
#define N32_ATIM2_CCDAT4           (N32_ATIMER2_BASE + N32_ATIM_CCDAT4_OFFSET)
#define N32_ATIM2_CCDAT5           (N32_ATIMER2_BASE + N32_ATIM_CCDAT5_OFFSET)
#define N32_ATIM2_CCDAT6           (N32_ATIMER2_BASE + N32_ATIM_CCDAT6_OFFSET)
#define N32_ATIM2_PSC              (N32_ATIMER2_BASE + N32_ATIM_PSC_OFFSET)
#define N32_ATIM2_AR               (N32_ATIMER2_BASE + N32_ATIM_AR_OFFSET)
#define N32_ATIM2_CNT              (N32_ATIMER2_BASE + N32_ATIM_CNT_OFFSET)
#define N32_ATIM2_REPCNT           (N32_ATIMER2_BASE + N32_ATIM_REPCNT_OFFSET)
#define N32_ATIM2_BKDT             (N32_ATIMER2_BASE + N32_ATIM_BKDT_OFFSET)
#define N32_ATIM2_CCDAT7           (N32_ATIMER2_BASE + N32_ATIM_CCDAT7_OFFSET)
#define N32_ATIM2_CCDAT8           (N32_ATIMER2_BASE + N32_ATIM_CCDAT8_OFFSET)
#define N32_ATIM2_CCDAT9           (N32_ATIMER2_BASE + N32_ATIM_CCDAT9_OFFSET)
#define N32_ATIM2_BKFR             (N32_ATIMER2_BASE + N32_ATIM_BKFR_OFFSET)
#define N32_ATIM2_INSEL            (N32_ATIMER2_BASE + N32_ATIM_INSEL_OFFSET)
#define N32_ATIM2_AF1              (N32_ATIMER2_BASE + N32_ATIM_AF1_OFFSET)
#define N32_ATIM2_AF2              (N32_ATIMER2_BASE + N32_ATIM_AF2_OFFSET)
#define N32_ATIM2_BKFR2            (N32_ATIMER2_BASE + N32_ATIM_BKFR2_OFFSET)
#define N32_ATIM2_DCTRL            (N32_ATIMER2_BASE + N32_ATIM_DCTRL_OFFSET)
#define N32_ATIM2_DADDR            (N32_ATIMER2_BASE + N32_ATIM_DADDR_OFFSET)

/* Register Addresses - ATIM3 ***********************************************/

#define N32_ATIM3_CTRL1            (N32_ATIMER3_BASE + N32_ATIM_CTRL1_OFFSET)
#define N32_ATIM3_CTRL2            (N32_ATIMER3_BASE + N32_ATIM_CTRL2_OFFSET)
#define N32_ATIM3_STS              (N32_ATIMER3_BASE + N32_ATIM_STS_OFFSET)
#define N32_ATIM3_ETGEN            (N32_ATIMER3_BASE + N32_ATIM_ETGEN_OFFSET)
#define N32_ATIM3_SMCTRL           (N32_ATIMER3_BASE + N32_ATIM_SMCTRL_OFFSET)
#define N32_ATIM3_DINTEN           (N32_ATIMER3_BASE + N32_ATIM_DINTEN_OFFSET)
#define N32_ATIM3_CCMOD1           (N32_ATIMER3_BASE + N32_ATIM_CCMOD1_OFFSET)
#define N32_ATIM3_CCMOD2           (N32_ATIMER3_BASE + N32_ATIM_CCMOD2_OFFSET)
#define N32_ATIM3_CCMOD3           (N32_ATIMER3_BASE + N32_ATIM_CCMOD3_OFFSET)
#define N32_ATIM3_CCEN             (N32_ATIMER3_BASE + N32_ATIM_CCEN_OFFSET)
#define N32_ATIM3_CCDAT1           (N32_ATIMER3_BASE + N32_ATIM_CCDAT1_OFFSET)
#define N32_ATIM3_CCDAT2           (N32_ATIMER3_BASE + N32_ATIM_CCDAT2_OFFSET)
#define N32_ATIM3_CCDAT3           (N32_ATIMER3_BASE + N32_ATIM_CCDAT3_OFFSET)
#define N32_ATIM3_CCDAT4           (N32_ATIMER3_BASE + N32_ATIM_CCDAT4_OFFSET)
#define N32_ATIM3_CCDAT5           (N32_ATIMER3_BASE + N32_ATIM_CCDAT5_OFFSET)
#define N32_ATIM3_CCDAT6           (N32_ATIMER3_BASE + N32_ATIM_CCDAT6_OFFSET)
#define N32_ATIM3_PSC              (N32_ATIMER3_BASE + N32_ATIM_PSC_OFFSET)
#define N32_ATIM3_AR               (N32_ATIMER3_BASE + N32_ATIM_AR_OFFSET)
#define N32_ATIM3_CNT              (N32_ATIMER3_BASE + N32_ATIM_CNT_OFFSET)
#define N32_ATIM3_REPCNT           (N32_ATIMER3_BASE + N32_ATIM_REPCNT_OFFSET)
#define N32_ATIM3_BKDT             (N32_ATIMER3_BASE + N32_ATIM_BKDT_OFFSET)
#define N32_ATIM3_CCDAT7           (N32_ATIMER3_BASE + N32_ATIM_CCDAT7_OFFSET)
#define N32_ATIM3_CCDAT8           (N32_ATIMER3_BASE + N32_ATIM_CCDAT8_OFFSET)
#define N32_ATIM3_CCDAT9           (N32_ATIMER3_BASE + N32_ATIM_CCDAT9_OFFSET)
#define N32_ATIM3_BKFR             (N32_ATIMER3_BASE + N32_ATIM_BKFR_OFFSET)
#define N32_ATIM3_INSEL            (N32_ATIMER3_BASE + N32_ATIM_INSEL_OFFSET)
#define N32_ATIM3_AF1              (N32_ATIMER3_BASE + N32_ATIM_AF1_OFFSET)
#define N32_ATIM3_AF2              (N32_ATIMER3_BASE + N32_ATIM_AF2_OFFSET)
#define N32_ATIM3_BKFR2            (N32_ATIMER3_BASE + N32_ATIM_BKFR2_OFFSET)
#define N32_ATIM3_DCTRL            (N32_ATIMER3_BASE + N32_ATIM_DCTRL_OFFSET)
#define N32_ATIM3_DADDR            (N32_ATIMER3_BASE + N32_ATIM_DADDR_OFFSET)

/* Register Addresses - ATIM4 ***********************************************/

#define N32_ATIM4_CTRL1            (N32_ATIMER4_BASE + N32_ATIM_CTRL1_OFFSET)
#define N32_ATIM4_CTRL2            (N32_ATIMER4_BASE + N32_ATIM_CTRL2_OFFSET)
#define N32_ATIM4_STS              (N32_ATIMER4_BASE + N32_ATIM_STS_OFFSET)
#define N32_ATIM4_ETGEN            (N32_ATIMER4_BASE + N32_ATIM_ETGEN_OFFSET)
#define N32_ATIM4_SMCTRL           (N32_ATIMER4_BASE + N32_ATIM_SMCTRL_OFFSET)
#define N32_ATIM4_DINTEN           (N32_ATIMER4_BASE + N32_ATIM_DINTEN_OFFSET)
#define N32_ATIM4_CCMOD1           (N32_ATIMER4_BASE + N32_ATIM_CCMOD1_OFFSET)
#define N32_ATIM4_CCMOD2           (N32_ATIMER4_BASE + N32_ATIM_CCMOD2_OFFSET)
#define N32_ATIM4_CCMOD3           (N32_ATIMER4_BASE + N32_ATIM_CCMOD3_OFFSET)
#define N32_ATIM4_CCEN             (N32_ATIMER4_BASE + N32_ATIM_CCEN_OFFSET)
#define N32_ATIM4_CCDAT1           (N32_ATIMER4_BASE + N32_ATIM_CCDAT1_OFFSET)
#define N32_ATIM4_CCDAT2           (N32_ATIMER4_BASE + N32_ATIM_CCDAT2_OFFSET)
#define N32_ATIM4_CCDAT3           (N32_ATIMER4_BASE + N32_ATIM_CCDAT3_OFFSET)
#define N32_ATIM4_CCDAT4           (N32_ATIMER4_BASE + N32_ATIM_CCDAT4_OFFSET)
#define N32_ATIM4_CCDAT5           (N32_ATIMER4_BASE + N32_ATIM_CCDAT5_OFFSET)
#define N32_ATIM4_CCDAT6           (N32_ATIMER4_BASE + N32_ATIM_CCDAT6_OFFSET)
#define N32_ATIM4_PSC              (N32_ATIMER4_BASE + N32_ATIM_PSC_OFFSET)
#define N32_ATIM4_AR               (N32_ATIMER4_BASE + N32_ATIM_AR_OFFSET)
#define N32_ATIM4_CNT              (N32_ATIMER4_BASE + N32_ATIM_CNT_OFFSET)
#define N32_ATIM4_REPCNT           (N32_ATIMER4_BASE + N32_ATIM_REPCNT_OFFSET)
#define N32_ATIM4_BKDT             (N32_ATIMER4_BASE + N32_ATIM_BKDT_OFFSET)
#define N32_ATIM4_CCDAT7           (N32_ATIMER4_BASE + N32_ATIM_CCDAT7_OFFSET)
#define N32_ATIM4_CCDAT8           (N32_ATIMER4_BASE + N32_ATIM_CCDAT8_OFFSET)
#define N32_ATIM4_CCDAT9           (N32_ATIMER4_BASE + N32_ATIM_CCDAT9_OFFSET)
#define N32_ATIM4_BKFR             (N32_ATIMER4_BASE + N32_ATIM_BKFR_OFFSET)
#define N32_ATIM4_INSEL            (N32_ATIMER4_BASE + N32_ATIM_INSEL_OFFSET)
#define N32_ATIM4_AF1              (N32_ATIMER4_BASE + N32_ATIM_AF1_OFFSET)
#define N32_ATIM4_AF2              (N32_ATIMER4_BASE + N32_ATIM_AF2_OFFSET)
#define N32_ATIM4_BKFR2            (N32_ATIMER4_BASE + N32_ATIM_BKFR2_OFFSET)
#define N32_ATIM4_DCTRL            (N32_ATIMER4_BASE + N32_ATIM_DCTRL_OFFSET)
#define N32_ATIM4_DADDR            (N32_ATIMER4_BASE + N32_ATIM_DADDR_OFFSET)

/* Bitfield Definitions - ATIM **********************************************/

/* Control register 1 (N32_ATIM_CTRL1) */

#define N32_ATIM_CTRL1_CNTEN              (1 << 0)                           /* Counter enable */
#define N32_ATIM_CTRL1_DIR                (1 << 1)                           /* Direction */
#define N32_ATIM_CTRL1_CAMSEL_SHIFT       (2)                                /* Center-aligned mode selection */
#define N32_ATIM_CTRL1_CAMSEL_MASK        (3 << N32_ATIM_CTRL1_CAMSEL_SHIFT)
#  define N32_ATIM_CTRL1_EDGE             (0 << N32_ATIM_CTRL1_CAMSEL_SHIFT) /* Edge-aligned */
#  define N32_ATIM_CTRL1_CENTER1          (1 << N32_ATIM_CTRL1_CAMSEL_SHIFT) /* Center-aligned 1 */
#  define N32_ATIM_CTRL1_CENTER2          (2 << N32_ATIM_CTRL1_CAMSEL_SHIFT) /* Center-aligned 2 */
#  define N32_ATIM_CTRL1_CENTER3          (3 << N32_ATIM_CTRL1_CAMSEL_SHIFT) /* Center-aligned 3 */
#define N32_ATIM_CTRL1_UPRS               (1 << 4)                           /* Update request source */
#define N32_ATIM_CTRL1_UPDIS              (1 << 5)                           /* Update disable */
#define N32_ATIM_CTRL1_CLKD_SHIFT         (6)                                /* Clock division */
#define N32_ATIM_CTRL1_CLKD_MASK          (3 << N32_ATIM_CTRL1_CLKD_SHIFT)
#  define N32_ATIM_CTRL1_TCKINT           (0 << N32_ATIM_CTRL1_CLKD_SHIFT)   /* tDTS = tCK_INT */
#  define N32_ATIM_CTRL1_2TCKINT          (1 << N32_ATIM_CTRL1_CLKD_SHIFT)   /* tDTS = 2tCK_INT */
#  define N32_ATIM_CTRL1_4TCKINT          (2 << N32_ATIM_CTRL1_CLKD_SHIFT)   /* tDTS = 4tCK_INT */
#define N32_ATIM_CTRL1_ONEPM              (1 << 8)                           /* One pulse mode */
#define N32_ATIM_CTRL1_ARPEN              (1 << 9)                           /* Auto-reload preload enable */
#define N32_ATIM_CTRL1_LBKPEN             (1 << 10)                          /* LockUp as BRK enable */
#define N32_ATIM_CTRL1_PBKPEN             (1 << 11)                          /* PVD as BRK enable */
#define N32_ATIM_CTRL1_SMPARERREN         (1 << 12)                          /* SRAM parity error as BRK enable */
#define N32_ATIM_CTRL1_CLRSEL             (1 << 13)                          /* OcxRef clear selection */
#define N32_ATIM_CTRL1_SMECCERREN         (1 << 15)                          /* SRAM ECC error as BRK enable */
#define N32_ATIM_CTRL1_CMODE_SHIFT        (20)                               /* Center-aligned mode for TRGO */
#define N32_ATIM_CTRL1_CMODE_MASK         (3 << N32_ATIM_CTRL1_CMODE_SHIFT)
#  define N32_ATIM_CTRL1_CMODE_UP         (0 << N32_ATIM_CTRL1_CMODE_SHIFT)  /* Up-count trigger */
#  define N32_ATIM_CTRL1_CMODE_DOWN       (1 << N32_ATIM_CTRL1_CMODE_SHIFT)  /* Down-count trigger */
#  define N32_ATIM_CTRL1_CMODE_UPDOWN     (2 << N32_ATIM_CTRL1_CMODE_SHIFT)  /* Both edges trigger */
#define N32_ATIM_CTRL1_ASYMMETRIC         (1 << 23)                          /* Asymmetric mode enable */

/* Control register 2 (N32_ATIM_CTRL2) */

#define N32_ATIM_CTRL2_O11                (1 << 0)                           /* Output idle state 1 (OC1) */
#define N32_ATIM_CTRL2_O11N               (1 << 1)                           /* Output idle state 1N (OC1N) */
#define N32_ATIM_CTRL2_O12                (1 << 2)                           /* Output idle state 2 (OC2) */
#define N32_ATIM_CTRL2_O12N               (1 << 3)                           /* Output idle state 2N (OC2N) */
#define N32_ATIM_CTRL2_O13                (1 << 4)                           /* Output idle state 3 (OC3) */
#define N32_ATIM_CTRL2_O13N               (1 << 5)                           /* Output idle state 3N (OC3N) */
#define N32_ATIM_CTRL2_O14                (1 << 6)                           /* Output idle state 4 (OC4) */
#define N32_ATIM_CTRL2_O14N               (1 << 7)                           /* Output idle state 4N (OC4N) */
#define N32_ATIM_CTRL2_O15                (1 << 8)                           /* Output idle state 5 (OC5) */
#define N32_ATIM_CTRL2_O16                (1 << 10)                          /* Output idle state 6 (OC6) */
#define N32_ATIM_CTRL2_MMSEL_SHIFT        (12)                               /* Master mode selection (TRGO) */
#define N32_ATIM_CTRL2_MMSEL_MASK         (15 << N32_ATIM_CTRL2_MMSEL_SHIFT)
#  define N32_ATIM_CTRL2_MMSEL_RESET      (0 << N32_ATIM_CTRL2_MMSEL_SHIFT)  /* Reset */
#  define N32_ATIM_CTRL2_MMSEL_ENABLE     (1 << N32_ATIM_CTRL2_MMSEL_SHIFT)  /* Enable */
#  define N32_ATIM_CTRL2_MMSEL_UPDATE     (2 << N32_ATIM_CTRL2_MMSEL_SHIFT)  /* Update */
#  define N32_ATIM_CTRL2_MMSEL_COMPP      (3 << N32_ATIM_CTRL2_MMSEL_SHIFT)  /* Compare pulse */
#  define N32_ATIM_CTRL2_MMSEL_OC1REF     (4 << N32_ATIM_CTRL2_MMSEL_SHIFT)  /* OC1REF */
#  define N32_ATIM_CTRL2_MMSEL_OC2REF     (5 << N32_ATIM_CTRL2_MMSEL_SHIFT)  /* OC2REF */
#  define N32_ATIM_CTRL2_MMSEL_OC3REF     (6 << N32_ATIM_CTRL2_MMSEL_SHIFT)  /* OC3REF */
#  define N32_ATIM_CTRL2_MMSEL_OC4REF     (7 << N32_ATIM_CTRL2_MMSEL_SHIFT)  /* OC4REF */
#  define N32_ATIM_CTRL2_MMSEL_OC5REF     (8 << N32_ATIM_CTRL2_MMSEL_SHIFT)  /* OC5REF */
#  define N32_ATIM_CTRL2_MMSEL_OC6REF     (9 << N32_ATIM_CTRL2_MMSEL_SHIFT)  /* OC6REF */
#  define N32_ATIM_CTRL2_MMSEL_OC4REFEDGE (10 << N32_ATIM_CTRL2_MMSEL_SHIFT) /* OC4REF edge */
#  define N32_ATIM_CTRL2_MMSEL_OC6REFEDGE (11 << N32_ATIM_CTRL2_MMSEL_SHIFT) /* OC6REF edge */
#define N32_ATIM_CTRL2_CCUSEL             (1 << 16)                          /* Capture/compare control update sel */
#define N32_ATIM_CTRL2_CCDSEL             (1 << 17)                          /* Capture/compare DMA selection */
#define N32_ATIM_CTRL2_CCPCTL             (1 << 18)                          /* Capture/compare preloaded control */
#define N32_ATIM_CTRL2_TI1SEL             (1 << 19)                          /* TI1 selection */
#define N32_ATIM_CTRL2_TRIG4              (1 << 20)                          /* Trigger ADC on channel 4 compare */
#define N32_ATIM_CTRL2_TRIG7              (1 << 21)                          /* Trigger ADC on channel 7 compare */
#define N32_ATIM_CTRL2_TRIG8              (1 << 22)                          /* Trigger ADC on channel 8 compare */
#define N32_ATIM_CTRL2_TRIG9              (1 << 23)                          /* Trigger ADC on channel 9 compare */
#define N32_ATIM_CTRL2_MMSEL2_SHIFT       (24)                               /* Master mode selection 2 (TRGO2) */
#define N32_ATIM_CTRL2_MMSEL2_MASK        (15 << N32_ATIM_CTRL2_MMSEL2_SHIFT)
#  define N32_ATIM_CTRL2_MMSEL2_RESET     (0 << N32_ATIM_CTRL2_MMSEL2_SHIFT) /* Reset */
#  define N32_ATIM_CTRL2_MMSEL2_ENABLE    (1 << N32_ATIM_CTRL2_MMSEL2_SHIFT) /* Enable */
#  define N32_ATIM_CTRL2_MMSEL2_UPDATE    (2 << N32_ATIM_CTRL2_MMSEL2_SHIFT) /* Update */
#  define N32_ATIM_CTRL2_MMSEL2_COMPP     (3 << N32_ATIM_CTRL2_MMSEL2_SHIFT) /* Compare pulse */
#  define N32_ATIM_CTRL2_MMSEL2_OC1REF    (4 << N32_ATIM_CTRL2_MMSEL2_SHIFT) /* OC1REF */
#  define N32_ATIM_CTRL2_MMSEL2_OC2REF    (5 << N32_ATIM_CTRL2_MMSEL2_SHIFT) /* OC2REF */
#  define N32_ATIM_CTRL2_MMSEL2_OC3REF    (6 << N32_ATIM_CTRL2_MMSEL2_SHIFT) /* OC3REF */
#  define N32_ATIM_CTRL2_MMSEL2_OC4REF    (7 << N32_ATIM_CTRL2_MMSEL2_SHIFT) /* OC4REF */
#  define N32_ATIM_CTRL2_MMSEL2_OC5REF    (8 << N32_ATIM_CTRL2_MMSEL2_SHIFT) /* OC5REF */
#  define N32_ATIM_CTRL2_MMSEL2_OC6REF    (9 << N32_ATIM_CTRL2_MMSEL2_SHIFT) /* OC6REF */

/* Status register (N32_ATIM_STS) */

#define N32_ATIM_STS_CC1ITF              (1 << 0)  /* CC1 interrupt flag */
#define N32_ATIM_STS_CC2ITF              (1 << 1)  /* CC2 interrupt flag */
#define N32_ATIM_STS_CC3ITF              (1 << 2)  /* CC3 interrupt flag */
#define N32_ATIM_STS_CC4ITF              (1 << 3)  /* CC4 interrupt flag */
#define N32_ATIM_STS_CC5ITF              (1 << 4)  /* CC5 interrupt flag */
#define N32_ATIM_STS_CC6ITF              (1 << 5)  /* CC6 interrupt flag */
#define N32_ATIM_STS_CC1OCF              (1 << 8)  /* CC1 overcapture flag */
#define N32_ATIM_STS_CC2OCF              (1 << 9)  /* CC2 overcapture flag */
#define N32_ATIM_STS_CC3OCF              (1 << 10) /* CC3 overcapture flag */
#define N32_ATIM_STS_CC4OCF              (1 << 11) /* CC4 overcapture flag */
#define N32_ATIM_STS_UDITF               (1 << 16) /* Update interrupt flag */
#define N32_ATIM_STS_COMITF              (1 << 17) /* COM interrupt flag */
#define N32_ATIM_STS_TITF                (1 << 18) /* Trigger interrupt flag */
#define N32_ATIM_STS_BITF                (1 << 19) /* Break1 interrupt flag */
#define N32_ATIM_STS_BITF2               (1 << 20) /* Break2 interrupt flag */
#define N32_ATIM_STS_SBITF               (1 << 21) /* System break interrupt flag */
#define N32_ATIM_STS_CC7ITF              (1 << 24) /* CC7 interrupt flag */
#define N32_ATIM_STS_CC8ITF              (1 << 25) /* CC8 interrupt flag */
#define N32_ATIM_STS_CC9ITF              (1 << 26) /* CC9 interrupt flag */

/* Event generation register (N32_ATIM_ETGEN) */

#define N32_ATIM_ETGEN_CC1GN             (1 << 0)  /* CC1 generation */
#define N32_ATIM_ETGEN_CC2GN             (1 << 1)  /* CC2 generation */
#define N32_ATIM_ETGEN_CC3GN             (1 << 2)  /* CC3 generation */
#define N32_ATIM_ETGEN_CC4GN             (1 << 3)  /* CC4 generation */
#define N32_ATIM_ETGEN_UDGN              (1 << 8)  /* Update generation */
#define N32_ATIM_ETGEN_CCUDGN            (1 << 9)  /* CC control update generation */
#define N32_ATIM_ETGEN_TGN               (1 << 10) /* Trigger generation */
#define N32_ATIM_ETGEN_BGN               (1 << 11) /* Break1 generation */
#define N32_ATIM_ETGEN_BGN2              (1 << 12) /* Break2 generation */

/* Slave mode control register (N32_ATIM_SMCTRL) */

#define N32_ATIM_SMCTRL_TSEL_SHIFT       (0)                               /* Trigger selection */
#define N32_ATIM_SMCTRL_TSEL_MASK        (7 << N32_ATIM_SMCTRL_TSEL_SHIFT)
#  define N32_ATIM_SMCTRL_ITR            (0 << N32_ATIM_SMCTRL_TSEL_SHIFT) /* Internal trigger */
#  define N32_ATIM_SMCTRL_TI1FED         (4 << N32_ATIM_SMCTRL_TSEL_SHIFT) /* TI1 edge detector */
#  define N32_ATIM_SMCTRL_TI1FP1         (5 << N32_ATIM_SMCTRL_TSEL_SHIFT) /* Filtered TI1 */
#  define N32_ATIM_SMCTRL_TI2FP2         (6 << N32_ATIM_SMCTRL_TSEL_SHIFT) /* Filtered TI2 */
#  define N32_ATIM_SMCTRL_ETRF           (7 << N32_ATIM_SMCTRL_TSEL_SHIFT) /* ETRF */
#define N32_ATIM_SMCTRL_SMSEL_SHIFT      (4)                               /* Slave mode selection */
#define N32_ATIM_SMCTRL_SMSEL_MASK       (15 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#  define N32_ATIM_SMCTRL_DISAB          (0 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#  define N32_ATIM_SMCTRL_ENCMD1         (1 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#  define N32_ATIM_SMCTRL_ENCMD2         (2 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#  define N32_ATIM_SMCTRL_ENCMD3         (3 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#  define N32_ATIM_SMCTRL_RESET          (4 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#  define N32_ATIM_SMCTRL_GATED          (5 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#  define N32_ATIM_SMCTRL_TRIGGER        (6 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#  define N32_ATIM_SMCTRL_EXTCLK1        (7 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#  define N32_ATIM_SMCTRL_ENCMD4         (9 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#  define N32_ATIM_SMCTRL_ENCMD5         (10 << N32_ATIM_SMCTRL_SMSEL_SHIFT)
#define N32_ATIM_SMCTRL_EXTPS_SHIFT      (8)                               /* External trigger prescaler */
#define N32_ATIM_SMCTRL_EXTPS_MASK       (3 << N32_ATIM_SMCTRL_EXTPS_SHIFT)
#  define N32_ATIM_SMCTRL_PSCOFF         (0 << N32_ATIM_SMCTRL_EXTPS_SHIFT)
#  define N32_ATIM_SMCTRL_ETRPd2         (1 << N32_ATIM_SMCTRL_EXTPS_SHIFT)
#  define N32_ATIM_SMCTRL_ETRPd4         (2 << N32_ATIM_SMCTRL_EXTPS_SHIFT)
#  define N32_ATIM_SMCTRL_ETRPd8         (3 << N32_ATIM_SMCTRL_EXTPS_SHIFT)
#define N32_ATIM_SMCTRL_EXCEN            (1 << 10)                         /* External clock enable */
#define N32_ATIM_SMCTRL_EXTP             (1 << 11)                         /* External trigger polarity */
#define N32_ATIM_SMCTRL_EXTF_SHIFT       (12)                              /* External trigger filter */
#define N32_ATIM_SMCTRL_EXTF_MASK        (15 << N32_ATIM_SMCTRL_EXTF_SHIFT)
#define N32_ATIM_SMCTRL_MSMD             (1 << 16)                         /* Master/slave mode */
#define N32_ATIM_SMCTRL_OCRECFLRP        (1 << 19)                         /* tim_ocref_clr polarity */
#define N32_ATIM_SMCTRL_OCRECFLRF_SHIFT  (20)                              /* tim_ocref_clr filter */
#define N32_ATIM_SMCTRL_OCRECFLRF_MASK   (15 << N32_ATIM_SMCTRL_OCRECFLRF_SHIFT)

/* Common filter encoding (used by EXTF and OCRECFLRF) */

#define N32_ATIM_FILTER_NOFILT           (0)       /* No filter */
#define N32_ATIM_FILTER_FCKINT2          (1)       /* fCK_INT, N=2 */
#define N32_ATIM_FILTER_FCKINT4          (2)       /* fCK_INT, N=4 */
#define N32_ATIM_FILTER_FCKINT8          (3)       /* fCK_INT, N=8 */
#define N32_ATIM_FILTER_FDTSd2_6         (4)       /* fDTS/2, N=6 */
#define N32_ATIM_FILTER_FDTSd2_8         (5)       /* fDTS/2, N=8 */
#define N32_ATIM_FILTER_FDTSd4_6         (6)       /* fDTS/4, N=6 */
#define N32_ATIM_FILTER_FDTSd4_8         (7)       /* fDTS/4, N=8 */
#define N32_ATIM_FILTER_FDTSd8_6         (8)       /* fDTS/8, N=6 */
#define N32_ATIM_FILTER_FDTSd8_8         (9)       /* fDTS/8, N=8 */
#define N32_ATIM_FILTER_FDTSd16_5        (10)      /* fDTS/16, N=5 */
#define N32_ATIM_FILTER_FDTSd16_6        (11)      /* fDTS/16, N=6 */
#define N32_ATIM_FILTER_FDTSd16_8        (12)      /* fDTS/16, N=8 */
#define N32_ATIM_FILTER_FDTSd32_5        (13)      /* fDTS/32, N=5 */
#define N32_ATIM_FILTER_FDTSd32_F6       (14)      /* fDTS/32, N=6 */
#define N32_ATIM_FILTER_FDTSd32_8        (15)      /* fDTS/32, N=8 */

/* DMA/Interrupt enable register (N32_ATIM_DINTEN) */

#define N32_ATIM_DINTEN_CC1IEN           (1 << 0)  /* CC1 interrupt enable */
#define N32_ATIM_DINTEN_CC2IEN           (1 << 1)  /* CC2 interrupt enable */
#define N32_ATIM_DINTEN_CC3IEN           (1 << 2)  /* CC3 interrupt enable */
#define N32_ATIM_DINTEN_CC4IEN           (1 << 3)  /* CC4 interrupt enable */
#define N32_ATIM_DINTEN_CC5IEN           (1 << 4)  /* CC5 interrupt enable */
#define N32_ATIM_DINTEN_CC6IEN           (1 << 5)  /* CC6 interrupt enable */
#define N32_ATIM_DINTEN_CC7IEN           (1 << 6)  /* CC7 interrupt enable */
#define N32_ATIM_DINTEN_CC8IEN           (1 << 7)  /* CC8 interrupt enable */
#define N32_ATIM_DINTEN_CC1DEN           (1 << 8)  /* CC1 DMA request enable */
#define N32_ATIM_DINTEN_CC2DEN           (1 << 9)  /* CC2 DMA request enable */
#define N32_ATIM_DINTEN_CC3DEN           (1 << 10) /* CC3 DMA request enable */
#define N32_ATIM_DINTEN_CC4DEN           (1 << 11) /* CC4 DMA request enable */
#define N32_ATIM_DINTEN_UIEN             (1 << 16) /* Update interrupt enable */
#define N32_ATIM_DINTEN_TIEN             (1 << 17) /* Trigger interrupt enable */
#define N32_ATIM_DINTEN_BIEN             (1 << 18) /* Break interrupt enable */
#define N32_ATIM_DINTEN_UDEN             (1 << 19) /* Update DMA request enable */
#define N32_ATIM_DINTEN_COMDEN           (1 << 20) /* COM DMA request enable */
#define N32_ATIM_DINTEN_TDEN             (1 << 21) /* Trigger DMA request enable */
#define N32_ATIM_DINTEN_COMIEN           (1 << 22) /* COM interrupt enable */
#define N32_ATIM_DINTEN_CC9IEN           (1 << 23) /* CC9 interrupt enable */

/* Capture/compare mode register 1 - Output compare (N32_ATIM_CCMOD1) */

#define N32_ATIM_CCMOD1_CC1SEL_SHIFT     (0)                                 /* CC1 selection */
#define N32_ATIM_CCMOD1_CC1SEL_MASK      (3 << N32_ATIM_CCMOD1_CC1SEL_SHIFT)
#  define N32_ATIM_CCMOD1_CC1OUT         (0 << N32_ATIM_CCMOD1_CC1SEL_SHIFT) /* Output */
#  define N32_ATIM_CCMOD1_CC1IN_TI1      (1 << N32_ATIM_CCMOD1_CC1SEL_SHIFT) /* Input TI1 */
#  define N32_ATIM_CCMOD1_CC1IN_TI2      (2 << N32_ATIM_CCMOD1_CC1SEL_SHIFT) /* Input TI2 */
#  define N32_ATIM_CCMOD1_CC1IN_TRC      (3 << N32_ATIM_CCMOD1_CC1SEL_SHIFT) /* Input TRC */
#define N32_ATIM_CCMOD1_OC1PEN           (1 << 2)                            /* OC1 preload enable */
#define N32_ATIM_CCMOD1_OC1FEN           (1 << 3)                            /* OC1 fast enable */
#define N32_ATIM_CCMOD1_OC1CEN           (1 << 4)                            /* OC1 clear enable */
#define N32_ATIM_CCMOD1_OC1MD_SHIFT      (5)                                 /* OC1 mode */
#define N32_ATIM_CCMOD1_OC1MD_MASK       (7 << N32_ATIM_CCMOD1_OC1MD_SHIFT)
#  define N32_ATIM_CCMOD1_OCMODE_FRZN    (0 << N32_ATIM_CCMOD1_OC1MD_SHIFT)  /* Frozen */
#  define N32_ATIM_CCMOD1_OCMODE_ACTIVE  (1 << N32_ATIM_CCMOD1_OC1MD_SHIFT)  /* Active */
#  define N32_ATIM_CCMOD1_OCMODE_INACTIVE (2 << N32_ATIM_CCMOD1_OC1MD_SHIFT) /* Inactive */
#  define N32_ATIM_CCMOD1_OCMODE_TOGGLE  (3 << N32_ATIM_CCMOD1_OC1MD_SHIFT)  /* Toggle */
#  define N32_ATIM_CCMOD1_OCMODE_FORCELO (4 << N32_ATIM_CCMOD1_OC1MD_SHIFT)  /* Force low */
#  define N32_ATIM_CCMOD1_OCMODE_FORCEHI (5 << N32_ATIM_CCMOD1_OC1MD_SHIFT)  /* Force high */
#  define N32_ATIM_CCMOD1_OCMODE_PWM1    (6 << N32_ATIM_CCMOD1_OC1MD_SHIFT)  /* PWM mode 1 */
#  define N32_ATIM_CCMOD1_OCMODE_PWM2    (7 << N32_ATIM_CCMOD1_OC1MD_SHIFT)  /* PWM mode 2 */
#define N32_ATIM_CCMOD1_CC2SEL_SHIFT     (8)                                 /* CC2 selection */
#define N32_ATIM_CCMOD1_CC2SEL_MASK      (3 << N32_ATIM_CCMOD1_CC2SEL_SHIFT)
#  define N32_ATIM_CCMOD1_CC2OUT         (0 << N32_ATIM_CCMOD1_CC2SEL_SHIFT)
#  define N32_ATIM_CCMOD1_CC2IN_TI2      (1 << N32_ATIM_CCMOD1_CC2SEL_SHIFT)
#  define N32_ATIM_CCMOD1_CC2IN_TI1      (2 << N32_ATIM_CCMOD1_CC2SEL_SHIFT)
#  define N32_ATIM_CCMOD1_CC2IN_TRC      (3 << N32_ATIM_CCMOD1_CC2SEL_SHIFT)
#define N32_ATIM_CCMOD1_OC2PEN           (1 << 10)                            /* OC2 preload enable */
#define N32_ATIM_CCMOD1_OC2FEN           (1 << 11)                            /* OC2 fast enable */
#define N32_ATIM_CCMOD1_OC2CEN           (1 << 12)                            /* OC2 clear enable */
#define N32_ATIM_CCMOD1_OC2MD_SHIFT      (13)                                 /* OC2 mode */
#define N32_ATIM_CCMOD1_OC2MD_MASK       (7 << N32_ATIM_CCMOD1_OC2MD_SHIFT)

/* Capture/compare mode register 1 - Input capture (N32_ATIM_CCMOD1) */

#define N32_ATIM_CCMOD1_IC1PSC_SHIFT     (2)                                 /* IC1 prescaler */
#define N32_ATIM_CCMOD1_IC1PSC_MASK      (3 << N32_ATIM_CCMOD1_IC1PSC_SHIFT)
#  define N32_ATIM_CCMOD1_ICPSC_NONE     (0 << N32_ATIM_CCMOD1_IC1PSC_SHIFT) /* No prescaler */
#  define N32_ATIM_CCMOD1_ICPSC_2        (1 << N32_ATIM_CCMOD1_IC1PSC_SHIFT) /* Every 2 events */
#  define N32_ATIM_CCMOD1_ICPSC_4        (2 << N32_ATIM_CCMOD1_IC1PSC_SHIFT) /* Every 4 events */
#  define N32_ATIM_CCMOD1_ICPSC_8        (3 << N32_ATIM_CCMOD1_IC1PSC_SHIFT) /* Every 8 events */
#define N32_ATIM_CCMOD1_IC1F_SHIFT       (4)                                 /* IC1 filter */
#define N32_ATIM_CCMOD1_IC1F_MASK        (15 << N32_ATIM_CCMOD1_IC1F_SHIFT)
#  define N32_ATIM_CCMOD1_IC1F_DTS       (0 << N32_ATIM_CCMOD1_IC1F_SHIFT)   /* Direct time selection */
#  define N32_ATIM_CCMOD1_IC1F_INT_2     (1 << N32_ATIM_CCMOD1_IC1F_SHIFT)   /* Internal clock, Sample every 2 events */
#  define N32_ATIM_CCMOD1_IC1F_INT_4     (2 << N32_ATIM_CCMOD1_IC1F_SHIFT)   /* Internal clock, Sample every 4 events */
#  define N32_ATIM_CCMOD1_IC1F_INT_8     (3 << N32_ATIM_CCMOD1_IC1F_SHIFT)   /* Internal clock, Sample every 8 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_2_N6  (4 << N32_ATIM_CCMOD1_IC1F_SHIFT)   /* Direct time div2, Sample every 6 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_2_N8  (5 << N32_ATIM_CCMOD1_IC1F_SHIFT)   /* Direct time div2, Sample every 8 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_4_N6  (6 << N32_ATIM_CCMOD1_IC1F_SHIFT)   /* Direct time div4, Sample every 6 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_4_N8  (7 << N32_ATIM_CCMOD1_IC1F_SHIFT)   /* Direct time div4, Sample every 8 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_8_N6  (8 << N32_ATIM_CCMOD1_IC1F_SHIFT)   /* Direct time div8, Sample every 6 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_8_N8  (9 << N32_ATIM_CCMOD1_IC1F_SHIFT)   /* Direct time div8, Sample every 8 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_16_N5 (10 << N32_ATIM_CCMOD1_IC1F_SHIFT)  /* Direct time div16, Sample every 5 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_16_N6 (11 << N32_ATIM_CCMOD1_IC1F_SHIFT)  /* Direct time div16, Sample every 6 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_16_N8 (12 << N32_ATIM_CCMOD1_IC1F_SHIFT)  /* Direct time div16, Sample every 8 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_32_N5 (13 << N32_ATIM_CCMOD1_IC1F_SHIFT)  /* Direct time div32, Sample every 5 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_32_N6 (14 << N32_ATIM_CCMOD1_IC1F_SHIFT)  /* Direct time div32, Sample every 6 events */
#  define N32_ATIM_CCMOD1_IC1F_DTS_32_N8 (15 << N32_ATIM_CCMOD1_IC1F_SHIFT)  /* Direct time div32, Sample every 8 events */
#define N32_ATIM_CCMOD1_IC2PSC_SHIFT     (10)                                /* IC2 prescaler */
#define N32_ATIM_CCMOD1_IC2PSC_MASK      (3 << N32_ATIM_CCMOD1_IC2PSC_SHIFT)
#define N32_ATIM_CCMOD1_IC2F_SHIFT       (12)                                /* IC2 filter */
#define N32_ATIM_CCMOD1_IC2F_MASK        (15 << N32_ATIM_CCMOD1_IC2F_SHIFT)

/* Capture/compare mode register 2 - Output compare (N32_ATIM_CCMOD2) */

#define N32_ATIM_CCMOD2_CC3SEL_SHIFT     (0)                                 /* CC3 selection */
#define N32_ATIM_CCMOD2_CC3SEL_MASK      (3 << N32_ATIM_CCMOD2_CC3SEL_SHIFT)
#  define N32_ATIM_CCMOD2_CC3OUT         (0 << N32_ATIM_CCMOD2_CC3SEL_SHIFT) /* Output */
#  define N32_ATIM_CCMOD2_CC3IN_TI3      (1 << N32_ATIM_CCMOD2_CC3SEL_SHIFT) /* Input TI3 */
#  define N32_ATIM_CCMOD2_CC3IN_TI4      (2 << N32_ATIM_CCMOD2_CC3SEL_SHIFT) /* Input TI4 */
#  define N32_ATIM_CCMOD2_CC3IN_TRC      (3 << N32_ATIM_CCMOD2_CC3SEL_SHIFT) /* Input TRC */
#define N32_ATIM_CCMOD2_OC3PEN           (1 << 2)                            /* OC3 preload enable */
#define N32_ATIM_CCMOD2_OC3FEN           (1 << 3)                            /* OC3 fast enable */
#define N32_ATIM_CCMOD2_OC3CEN           (1 << 4)                            /* OC3 clear enable */
#define N32_ATIM_CCMOD2_OC3MD_SHIFT      (5)                                 /* OC3 mode */
#define N32_ATIM_CCMOD2_OC3MD_MASK       (7 << N32_ATIM_CCMOD2_OC3MD_SHIFT)
#define N32_ATIM_CCMOD2_CC4SEL_SHIFT     (8)                                 /* CC4 selection */
#define N32_ATIM_CCMOD2_CC4SEL_MASK      (3 << N32_ATIM_CCMOD2_CC4SEL_SHIFT)
#  define N32_ATIM_CCMOD2_CC4OUT         (0 << N32_ATIM_CCMOD2_CC4SEL_SHIFT) /* Output */
#  define N32_ATIM_CCMOD2_CC4IN_TI4      (1 << N32_ATIM_CCMOD2_CC4SEL_SHIFT) /* Input TI4 */
#  define N32_ATIM_CCMOD2_CC4IN_TI3      (2 << N32_ATIM_CCMOD2_CC4SEL_SHIFT) /* Input TI3 */
#  define N32_ATIM_CCMOD2_CC4IN_TRC      (3 << N32_ATIM_CCMOD2_CC4SEL_SHIFT) /* Input TRC */
#define N32_ATIM_CCMOD2_OC4PEN           (1 << 10)                           /* OC4 preload enable */
#define N32_ATIM_CCMOD2_OC4FEN           (1 << 11)                           /* OC4 fast enable */
#define N32_ATIM_CCMOD2_OC4CEN           (1 << 12)                           /* OC4 clear enable */
#define N32_ATIM_CCMOD2_OC4MD_SHIFT      (13)                                /* OC4 mode */
#define N32_ATIM_CCMOD2_OC4MD_MASK       (7 << N32_ATIM_CCMOD2_OC4MD_SHIFT)

/* Capture/compare mode register 2 - Input capture (N32_ATIM_CCMOD2) */

#define N32_ATIM_CCMOD2_IC3PSC_SHIFT     (2)       /* IC3 prescaler */
#define N32_ATIM_CCMOD2_IC3PSC_MASK      (3 << N32_ATIM_CCMOD2_IC3PSC_SHIFT)
#define N32_ATIM_CCMOD2_IC3F_SHIFT       (4)       /* IC3 filter */
#define N32_ATIM_CCMOD2_IC3F_MASK        (15 << N32_ATIM_CCMOD2_IC3F_SHIFT)
#define N32_ATIM_CCMOD2_IC4PSC_SHIFT     (10)      /* IC4 prescaler */
#define N32_ATIM_CCMOD2_IC4PSC_MASK      (3 << N32_ATIM_CCMOD2_IC4PSC_SHIFT)
#define N32_ATIM_CCMOD2_IC4F_SHIFT       (12)      /* IC4 filter */
#define N32_ATIM_CCMOD2_IC4F_MASK        (15 << N32_ATIM_CCMOD2_IC4F_SHIFT)

/* Capture/compare mode register 3 (N32_ATIM_CCMOD3) */

#define N32_ATIM_CCMOD3_OC5PEN           (1 << 2)  /* OC5 preload enable */
#define N32_ATIM_CCMOD3_OC5FEN           (1 << 3)  /* OC5 fast enable */
#define N32_ATIM_CCMOD3_OC5CEN           (1 << 4)  /* OC5 clear enable */
#define N32_ATIM_CCMOD3_OC5MD_SHIFT      (5)       /* OC5 mode */
#define N32_ATIM_CCMOD3_OC5MD_MASK       (7 << N32_ATIM_CCMOD3_OC5MD_SHIFT)
#define N32_ATIM_CCMOD3_OC6PEN           (1 << 10) /* OC6 preload enable */
#define N32_ATIM_CCMOD3_OC6FEN           (1 << 11) /* OC6 fast enable */
#define N32_ATIM_CCMOD3_OC6CEN           (1 << 12) /* OC6 clear enable */
#define N32_ATIM_CCMOD3_OC6MD_SHIFT      (13)      /* OC6 mode */
#define N32_ATIM_CCMOD3_OC6MD_MASK       (7 << N32_ATIM_CCMOD3_OC6MD_SHIFT)
#define N32_ATIM_CCMOD3_OC7PEN           (1 << 16) /* OC7 preload enable */
#define N32_ATIM_CCMOD3_OC8PEN           (1 << 20) /* OC8 preload enable */
#define N32_ATIM_CCMOD3_OC9PEN           (1 << 24) /* OC9 preload enable */

/* Capture/compare enable register (N32_ATIM_CCEN) */

#define N32_ATIM_CCEN_CC1NEN            (1 << 0)  /* CC1 complementary output enable */
#define N32_ATIM_CCEN_CC1NP             (1 << 1)  /* CC1 complementary output polarity */
#define N32_ATIM_CCEN_CC1EN             (1 << 2)  /* CC1 output enable */
#define N32_ATIM_CCEN_CC1P              (1 << 3)  /* CC1 output polarity */
#define N32_ATIM_CCEN_CC2NEN            (1 << 4)  /* CC2 complementary output enable */
#define N32_ATIM_CCEN_CC2NP             (1 << 5)  /* CC2 complementary output polarity */
#define N32_ATIM_CCEN_CC2EN             (1 << 6)  /* CC2 output enable */
#define N32_ATIM_CCEN_CC2P              (1 << 7)  /* CC2 output polarity */
#define N32_ATIM_CCEN_CC3NEN            (1 << 8)  /* CC3 complementary output enable */
#define N32_ATIM_CCEN_CC3NP             (1 << 9)  /* CC3 complementary output polarity */
#define N32_ATIM_CCEN_CC3EN             (1 << 10) /* CC3 output enable */
#define N32_ATIM_CCEN_CC3P              (1 << 11) /* CC3 output polarity */
#define N32_ATIM_CCEN_CC4NEN            (1 << 12) /* CC4 complementary output enable */
#define N32_ATIM_CCEN_CC4NP             (1 << 13) /* CC4 complementary output polarity */
#define N32_ATIM_CCEN_CC4EN             (1 << 14) /* CC4 output enable */
#define N32_ATIM_CCEN_CC4P              (1 << 15) /* CC4 output polarity */
#define N32_ATIM_CCEN_CC5EN             (1 << 18) /* CC5 output enable */
#define N32_ATIM_CCEN_CC5P              (1 << 19) /* CC5 output polarity */
#define N32_ATIM_CCEN_CC6EN             (1 << 22) /* CC6 output enable */
#define N32_ATIM_CCEN_CC6P              (1 << 23) /* CC6 output polarity */

/* Capture/compare registers (N32_ATIM_CCDAT1-9) */

#define N32_ATIM_CCDAT_MASK             (0xffff)  /* CCx value mask */
#define N32_ATIM_CCDAT_DOWN_SHIFT       (16)      /* Down-count value shift */

/* Prescaler, Auto-reload, Counter */

#define N32_ATIM_PSC_MASK               (0xffff)  /* Prescaler value mask */
#define N32_ATIM_AR_MASK                (0xffff)  /* Auto-reload value mask */
#define N32_ATIM_CNT_MASK               (0xffff)  /* Counter value mask */

/* Repetition counter */

#define N32_ATIM_REPCNT_MASK            (0xff)    /* Repetition counter value mask */

/* Break and dead-time register (N32_ATIM_BKDT) */

#define N32_ATIM_BKDT_DTGN_SHIFT        (0)                               /* Dead-time generator setup */
#define N32_ATIM_BKDT_DTGN_MASK         (0xff << N32_ATIM_BKDT_DTGN_SHIFT)
#define N32_ATIM_BKDT_MOEN              (1 << 8)                          /* Main output enable */
#define N32_ATIM_BKDT_AOEN              (1 << 9)                          /* Automatic output enable */
#define N32_ATIM_BKDT_BKP               (1 << 10)                         /* Break1 polarity */
#define N32_ATIM_BKDT_BKEN              (1 << 11)                         /* Break1 enable */
#define N32_ATIM_BKDT_OSSI              (1 << 12)                         /* Off-state selection for Idle mode */
#define N32_ATIM_BKDT_OSSR              (1 << 13)                         /* Off-state selection for Run mode */
#define N32_ATIM_BKDT_LCKCFG_SHIFT      (14)                              /* Lock configuration */
#define N32_ATIM_BKDT_LCKCFG_MASK       (3 << N32_ATIM_BKDT_LCKCFG_SHIFT)
#  define N32_ATIM_BKDT_LOCKOFF         (0 << N32_ATIM_BKDT_LCKCFG_SHIFT) /* No lock */
#  define N32_ATIM_BKDT_LOCK1           (1 << N32_ATIM_BKDT_LCKCFG_SHIFT) /* Lock level 1 */
#  define N32_ATIM_BKDT_LOCK2           (2 << N32_ATIM_BKDT_LCKCFG_SHIFT) /* Lock level 2 */
#  define N32_ATIM_BKDT_LOCK3           (3 << N32_ATIM_BKDT_LCKCFG_SHIFT) /* Lock level 3 */
#define N32_ATIM_BKDT_BK2P              (1 << 16)                         /* Break2 polarity */
#define N32_ATIM_BKDT_BK2EN             (1 << 17)                         /* Break2 enable */
#define N32_ATIM_BKDT_BRKDSRM           (1 << 18)                         /* Break1 disarm */
#define N32_ATIM_BKDT_BRK2DSRM          (1 << 19)                         /* Break2 disarm */
#define N32_ATIM_BKDT_BRKBID            (1 << 20)                         /* Break1 bidirectional enable */
#define N32_ATIM_BKDT_BRK2BID           (1 << 21)                         /* Break2 bidirectional enable */

/* Break filter register (N32_ATIM_BKFR) */

#define N32_ATIM_BKFR_SLIDFPSC_MASK     (0xffff)  /* Sampling clock prescaler */
#define N32_ATIM_BKFR_FILTEN            (1 << 16) /* Filter enable */
#define N32_ATIM_BKFR_WSIZE_SHIFT       (17)      /* Window size */
#define N32_ATIM_BKFR_WSIZE_MASK        (63 << N32_ATIM_BKFR_WSIZE_SHIFT)
#define N32_ATIM_BKFR_THRESH_SHIFT      (24)      /* Threshold */
#define N32_ATIM_BKFR_THRESH_MASK       (63 << N32_ATIM_BKFR_THRESH_SHIFT)

/* Input selection register (N32_ATIM_INSEL) */

#define N32_ATIM_INSEL_TI1S_SHIFT       (0)       /* TI1 input selection */
#define N32_ATIM_INSEL_TI1S_MASK        (15 << N32_ATIM_INSEL_TI1S_SHIFT)
#define N32_ATIM_INSEL_TI2S_SHIFT       (4)       /* TI2 input selection */
#define N32_ATIM_INSEL_TI2S_MASK        (15 << N32_ATIM_INSEL_TI2S_SHIFT)
#define N32_ATIM_INSEL_TI3S_SHIFT       (8)       /* TI3 input selection */
#define N32_ATIM_INSEL_TI3S_MASK        (15 << N32_ATIM_INSEL_TI3S_SHIFT)
#define N32_ATIM_INSEL_TI4S_SHIFT       (12)      /* TI4 input selection */
#define N32_ATIM_INSEL_TI4S_MASK        (15 << N32_ATIM_INSEL_TI4S_SHIFT)
#define N32_ATIM_INSEL_ETRS_SHIFT       (16)      /* ETR input selection */
#define N32_ATIM_INSEL_ETRS_MASK        (15 << N32_ATIM_INSEL_ETRS_SHIFT)
#define N32_ATIM_INSEL_ITRS_SHIFT       (20)      /* ITR input selection */
#define N32_ATIM_INSEL_ITRS_MASK        (15 << N32_ATIM_INSEL_ITRS_SHIFT)
#define N32_ATIM_INSEL_CLRS_SHIFT       (24)      /* OcxRef clear input selection */
#define N32_ATIM_INSEL_CLRS_MASK        (15 << N32_ATIM_INSEL_CLRS_SHIFT)

/* Alternate function registers (N32_ATIM_AF1, N32_ATIM_AF2) */

#define N32_ATIM_AF1_IOMBRKEN           (1 << 0)  /* TIMx_BKIN break input enable */
#define N32_ATIM_AF1_COMP1BRKEN         (1 << 1)  /* COMP1 break input enable */
#define N32_ATIM_AF1_COMP2BRKEN         (1 << 2)  /* COMP2 break input enable */
#define N32_ATIM_AF1_COMP3BRKEN         (1 << 3)  /* COMP3 break input enable */
#define N32_ATIM_AF1_COMP4BRKEN         (1 << 4)  /* COMP4 break input enable */
#define N32_ATIM_AF1_IOMBRKP            (1 << 9)  /* TIMx_BKIN polarity selection */
#define N32_ATIM_AF1_COMP1BRKP          (1 << 10) /* COMP1 break polarity */
#define N32_ATIM_AF1_COMP2BRKP          (1 << 11) /* COMP2 break polarity */
#define N32_ATIM_AF1_COMP3BRKP          (1 << 12) /* COMP3 break polarity */
#define N32_ATIM_AF1_COMP4BRKP          (1 << 13) /* COMP4 break polarity */

#define N32_ATIM_AF2_IOMBRK2EN          (1 << 0)  /* TIMx_BKIN2 break input enable */
#define N32_ATIM_AF2_COMP1BRK2EN        (1 << 1)  /* COMP1 break2 enable */
#define N32_ATIM_AF2_COMP2BRK2EN        (1 << 2)  /* COMP2 break2 enable */
#define N32_ATIM_AF2_COMP3BRK2EN        (1 << 3)  /* COMP3 break2 enable */
#define N32_ATIM_AF2_COMP4BRK2EN        (1 << 4)  /* COMP4 break2 enable */
#define N32_ATIM_AF2_IOMBRK2P           (1 << 9)  /* TIMx_BKIN2 polarity selection */
#define N32_ATIM_AF2_COMP1BRK2P         (1 << 10) /* COMP1 break2 polarity */
#define N32_ATIM_AF2_COMP2BRK2P         (1 << 11) /* COMP2 break2 polarity */
#define N32_ATIM_AF2_COMP3BRK2P         (1 << 12) /* COMP3 break2 polarity */
#define N32_ATIM_AF2_COMP4BRK2P         (1 << 13) /* COMP4 break2 polarity */
#define N32_ATIM_AF2_DSMU0BRK2EN        (1 << 24) /* DSMU0 break2 enable */
#define N32_ATIM_AF2_DSMU1BRK2EN        (1 << 25) /* DSMU1 break2 enable */
#define N32_ATIM_AF2_DSMU2BRK2EN        (1 << 26) /* DSMU2 break2 enable */
#define N32_ATIM_AF2_DSMU3BRK2EN        (1 << 27) /* DSMU3 break2 enable */

/* Break 2 filter (N32_ATIM_BKFR2) */

#define N32_ATIM_BKFR2_SLIDFPSC_MASK    (0xffff)  /* Sampling clock prescaler */
#define N32_ATIM_BKFR2_FILTEN           (1 << 16) /* Filter enable */
#define N32_ATIM_BKFR2_WSIZE_SHIFT      (17)      /* Window size */
#define N32_ATIM_BKFR2_WSIZE_MASK       (63 << N32_ATIM_BKFR2_WSIZE_SHIFT)
#define N32_ATIM_BKFR2_THRESH_SHIFT     (24)      /* Threshold */
#define N32_ATIM_BKFR2_THRESH_MASK      (63 << N32_ATIM_BKFR2_THRESH_SHIFT)

/* DMA control (N32_ATIM_DCTRL) */

#define N32_ATIM_DCTRL_DBLEN_SHIFT      (0)       /* DMA burst length */
#define N32_ATIM_DCTRL_DBLEN_MASK       (63 << N32_ATIM_DCTRL_DBLEN_SHIFT)
#  define N32_ATIM_DCTRL_DBLEN(n)       (((n)-1) << N32_ATIM_DCTRL_DBLEN_SHIFT)
#define N32_ATIM_DCTRL_DBADDR_SHIFT     (8)       /* DMA base address */
#define N32_ATIM_DCTRL_DBADDR_MASK      (63 << N32_ATIM_DCTRL_DBADDR_SHIFT)

#define N32_ATIM_DADDR_BURST_MASK       (0xffffffff) /* DMA burst data */

/* ==========================================================================
 *                     GENERAL-PURPOSE TIMERS A (GTIMA1-7)
 * ==========================================================================
 */

/* Register Offsets - GTIMA (common for all GTIMA1-7) ***********************/

#define N32_GTIMA_CTRL1_OFFSET      0x0000  /* Control register 1 */
#define N32_GTIMA_CTRL2_OFFSET      0x0004  /* Control register 2 */
#define N32_GTIMA_STS_OFFSET        0x0008  /* Status register */
#define N32_GTIMA_ETGEN_OFFSET      0x000c  /* Event generation register */
#define N32_GTIMA_SMCTRL_OFFSET     0x0010  /* Slave mode control register */
#define N32_GTIMA_DINTEN_OFFSET     0x0014  /* DMA/Interrupt enable register */
#define N32_GTIMA_CCMOD1_OFFSET     0x0018  /* Capture/compare mode register 1 */
#define N32_GTIMA_CCMOD2_OFFSET     0x001c  /* Capture/compare mode register 2 */
#define N32_GTIMA_CCEN_OFFSET       0x0024  /* Capture/compare enable register */
#define N32_GTIMA_CCDAT1_OFFSET     0x0028  /* Capture/compare register 1 */
#define N32_GTIMA_CCDAT2_OFFSET     0x002c  /* Capture/compare register 2 */
#define N32_GTIMA_CCDAT3_OFFSET     0x0030  /* Capture/compare register 3 */
#define N32_GTIMA_CCDAT4_OFFSET     0x0034  /* Capture/compare register 4 */
#define N32_GTIMA_PSC_OFFSET        0x0040  /* Prescaler */
#define N32_GTIMA_AR_OFFSET         0x0044  /* Auto-reload register */
#define N32_GTIMA_CNT_OFFSET        0x0048  /* Counter */
#define N32_GTIMA_C1FILT_OFFSET     0x0064  /* Channel 1 filter register */
#define N32_GTIMA_C2FILT_OFFSET     0x0068  /* Channel 2 filter register */
#define N32_GTIMA_C3FILT_OFFSET     0x006c  /* Channel 3 filter register */
#define N32_GTIMA_C4FILT_OFFSET     0x0070  /* Channel 4 filter register */
#define N32_GTIMA_FILTO_OFFSET      0x0074  /* Filter output register */
#define N32_GTIMA_INSEL_OFFSET      0x0078  /* Input selection register */
#define N32_GTIMA_DCTRL_OFFSET      0x0094  /* DMA control register */
#define N32_GTIMA_DADDR_OFFSET      0x0098  /* DMA address for burst mode */

/* Register Addresses - GTIMA1 **********************************************/

#define N32_GTIMA1_CTRL1            (N32_GTIMERA1_BASE + N32_GTIMA_CTRL1_OFFSET)
#define N32_GTIMA1_CTRL2            (N32_GTIMERA1_BASE + N32_GTIMA_CTRL2_OFFSET)
#define N32_GTIMA1_STS              (N32_GTIMERA1_BASE + N32_GTIMA_STS_OFFSET)
#define N32_GTIMA1_ETGEN            (N32_GTIMERA1_BASE + N32_GTIMA_ETGEN_OFFSET)
#define N32_GTIMA1_SMCTRL           (N32_GTIMERA1_BASE + N32_GTIMA_SMCTRL_OFFSET)
#define N32_GTIMA1_DINTEN           (N32_GTIMERA1_BASE + N32_GTIMA_DINTEN_OFFSET)
#define N32_GTIMA1_CCMOD1           (N32_GTIMERA1_BASE + N32_GTIMA_CCMOD1_OFFSET)
#define N32_GTIMA1_CCMOD2           (N32_GTIMERA1_BASE + N32_GTIMA_CCMOD2_OFFSET)
#define N32_GTIMA1_CCEN             (N32_GTIMERA1_BASE + N32_GTIMA_CCEN_OFFSET)
#define N32_GTIMA1_CCDAT1           (N32_GTIMERA1_BASE + N32_GTIMA_CCDAT1_OFFSET)
#define N32_GTIMA1_CCDAT2           (N32_GTIMERA1_BASE + N32_GTIMA_CCDAT2_OFFSET)
#define N32_GTIMA1_CCDAT3           (N32_GTIMERA1_BASE + N32_GTIMA_CCDAT3_OFFSET)
#define N32_GTIMA1_CCDAT4           (N32_GTIMERA1_BASE + N32_GTIMA_CCDAT4_OFFSET)
#define N32_GTIMA1_PSC              (N32_GTIMERA1_BASE + N32_GTIMA_PSC_OFFSET)
#define N32_GTIMA1_AR               (N32_GTIMERA1_BASE + N32_GTIMA_AR_OFFSET)
#define N32_GTIMA1_CNT              (N32_GTIMERA1_BASE + N32_GTIMA_CNT_OFFSET)
#define N32_GTIMA1_C1FILT           (N32_GTIMERA1_BASE + N32_GTIMA_C1FILT_OFFSET)
#define N32_GTIMA1_C2FILT           (N32_GTIMERA1_BASE + N32_GTIMA_C2FILT_OFFSET)
#define N32_GTIMA1_C3FILT           (N32_GTIMERA1_BASE + N32_GTIMA_C3FILT_OFFSET)
#define N32_GTIMA1_C4FILT           (N32_GTIMERA1_BASE + N32_GTIMA_C4FILT_OFFSET)
#define N32_GTIMA1_FILTO            (N32_GTIMERA1_BASE + N32_GTIMA_FILTO_OFFSET)
#define N32_GTIMA1_INSEL            (N32_GTIMERA1_BASE + N32_GTIMA_INSEL_OFFSET)
#define N32_GTIMA1_DCTRL            (N32_GTIMERA1_BASE + N32_GTIMA_DCTRL_OFFSET)
#define N32_GTIMA1_DADDR            (N32_GTIMERA1_BASE + N32_GTIMA_DADDR_OFFSET)

/* Register Addresses - GTIMA2 **********************************************/

#define N32_GTIMA2_CTRL1            (N32_GTIMERA2_BASE + N32_GTIMA_CTRL1_OFFSET)
#define N32_GTIMA2_CTRL2            (N32_GTIMERA2_BASE + N32_GTIMA_CTRL2_OFFSET)
#define N32_GTIMA2_STS              (N32_GTIMERA2_BASE + N32_GTIMA_STS_OFFSET)
#define N32_GTIMA2_ETGEN            (N32_GTIMERA2_BASE + N32_GTIMA_ETGEN_OFFSET)
#define N32_GTIMA2_SMCTRL           (N32_GTIMERA2_BASE + N32_GTIMA_SMCTRL_OFFSET)
#define N32_GTIMA2_DINTEN           (N32_GTIMERA2_BASE + N32_GTIMA_DINTEN_OFFSET)
#define N32_GTIMA2_CCMOD1           (N32_GTIMERA2_BASE + N32_GTIMA_CCMOD1_OFFSET)
#define N32_GTIMA2_CCMOD2           (N32_GTIMERA2_BASE + N32_GTIMA_CCMOD2_OFFSET)
#define N32_GTIMA2_CCEN             (N32_GTIMERA2_BASE + N32_GTIMA_CCEN_OFFSET)
#define N32_GTIMA2_CCDAT1           (N32_GTIMERA2_BASE + N32_GTIMA_CCDAT1_OFFSET)
#define N32_GTIMA2_CCDAT2           (N32_GTIMERA2_BASE + N32_GTIMA_CCDAT2_OFFSET)
#define N32_GTIMA2_CCDAT3           (N32_GTIMERA2_BASE + N32_GTIMA_CCDAT3_OFFSET)
#define N32_GTIMA2_CCDAT4           (N32_GTIMERA2_BASE + N32_GTIMA_CCDAT4_OFFSET)
#define N32_GTIMA2_PSC              (N32_GTIMERA2_BASE + N32_GTIMA_PSC_OFFSET)
#define N32_GTIMA2_AR               (N32_GTIMERA2_BASE + N32_GTIMA_AR_OFFSET)
#define N32_GTIMA2_CNT              (N32_GTIMERA2_BASE + N32_GTIMA_CNT_OFFSET)
#define N32_GTIMA2_C1FILT           (N32_GTIMERA2_BASE + N32_GTIMA_C1FILT_OFFSET)
#define N32_GTIMA2_C2FILT           (N32_GTIMERA2_BASE + N32_GTIMA_C2FILT_OFFSET)
#define N32_GTIMA2_C3FILT           (N32_GTIMERA2_BASE + N32_GTIMA_C3FILT_OFFSET)
#define N32_GTIMA2_C4FILT           (N32_GTIMERA2_BASE + N32_GTIMA_C4FILT_OFFSET)
#define N32_GTIMA2_FILTO            (N32_GTIMERA2_BASE + N32_GTIMA_FILTO_OFFSET)
#define N32_GTIMA2_INSEL            (N32_GTIMERA2_BASE + N32_GTIMA_INSEL_OFFSET)
#define N32_GTIMA2_DCTRL            (N32_GTIMERA2_BASE + N32_GTIMA_DCTRL_OFFSET)
#define N32_GTIMA2_DADDR            (N32_GTIMERA2_BASE + N32_GTIMA_DADDR_OFFSET)

/* Register Addresses - GTIMA3 **********************************************/

#define N32_GTIMA3_CTRL1            (N32_GTIMERA3_BASE + N32_GTIMA_CTRL1_OFFSET)
#define N32_GTIMA3_CTRL2            (N32_GTIMERA3_BASE + N32_GTIMA_CTRL2_OFFSET)
#define N32_GTIMA3_STS              (N32_GTIMERA3_BASE + N32_GTIMA_STS_OFFSET)
#define N32_GTIMA3_ETGEN            (N32_GTIMERA3_BASE + N32_GTIMA_ETGEN_OFFSET)
#define N32_GTIMA3_SMCTRL           (N32_GTIMERA3_BASE + N32_GTIMA_SMCTRL_OFFSET)
#define N32_GTIMA3_DINTEN           (N32_GTIMERA3_BASE + N32_GTIMA_DINTEN_OFFSET)
#define N32_GTIMA3_CCMOD1           (N32_GTIMERA3_BASE + N32_GTIMA_CCMOD1_OFFSET)
#define N32_GTIMA3_CCMOD2           (N32_GTIMERA3_BASE + N32_GTIMA_CCMOD2_OFFSET)
#define N32_GTIMA3_CCEN             (N32_GTIMERA3_BASE + N32_GTIMA_CCEN_OFFSET)
#define N32_GTIMA3_CCDAT1           (N32_GTIMERA3_BASE + N32_GTIMA_CCDAT1_OFFSET)
#define N32_GTIMA3_CCDAT2           (N32_GTIMERA3_BASE + N32_GTIMA_CCDAT2_OFFSET)
#define N32_GTIMA3_CCDAT3           (N32_GTIMERA3_BASE + N32_GTIMA_CCDAT3_OFFSET)
#define N32_GTIMA3_CCDAT4           (N32_GTIMERA3_BASE + N32_GTIMA_CCDAT4_OFFSET)
#define N32_GTIMA3_PSC              (N32_GTIMERA3_BASE + N32_GTIMA_PSC_OFFSET)
#define N32_GTIMA3_AR               (N32_GTIMERA3_BASE + N32_GTIMA_AR_OFFSET)
#define N32_GTIMA3_CNT              (N32_GTIMERA3_BASE + N32_GTIMA_CNT_OFFSET)
#define N32_GTIMA3_C1FILT           (N32_GTIMERA3_BASE + N32_GTIMA_C1FILT_OFFSET)
#define N32_GTIMA3_C2FILT           (N32_GTIMERA3_BASE + N32_GTIMA_C2FILT_OFFSET)
#define N32_GTIMA3_C3FILT           (N32_GTIMERA3_BASE + N32_GTIMA_C3FILT_OFFSET)
#define N32_GTIMA3_C4FILT           (N32_GTIMERA3_BASE + N32_GTIMA_C4FILT_OFFSET)
#define N32_GTIMA3_FILTO            (N32_GTIMERA3_BASE + N32_GTIMA_FILTO_OFFSET)
#define N32_GTIMA3_INSEL            (N32_GTIMERA3_BASE + N32_GTIMA_INSEL_OFFSET)
#define N32_GTIMA3_DCTRL            (N32_GTIMERA3_BASE + N32_GTIMA_DCTRL_OFFSET)
#define N32_GTIMA3_DADDR            (N32_GTIMERA3_BASE + N32_GTIMA_DADDR_OFFSET)

/* Register Addresses - GTIMA4 **********************************************/

#define N32_GTIMA4_CTRL1            (N32_GTIMERA4_BASE + N32_GTIMA_CTRL1_OFFSET)
#define N32_GTIMA4_CTRL2            (N32_GTIMERA4_BASE + N32_GTIMA_CTRL2_OFFSET)
#define N32_GTIMA4_STS              (N32_GTIMERA4_BASE + N32_GTIMA_STS_OFFSET)
#define N32_GTIMA4_ETGEN            (N32_GTIMERA4_BASE + N32_GTIMA_ETGEN_OFFSET)
#define N32_GTIMA4_SMCTRL           (N32_GTIMERA4_BASE + N32_GTIMA_SMCTRL_OFFSET)
#define N32_GTIMA4_DINTEN           (N32_GTIMERA4_BASE + N32_GTIMA_DINTEN_OFFSET)
#define N32_GTIMA4_CCMOD1           (N32_GTIMERA4_BASE + N32_GTIMA_CCMOD1_OFFSET)
#define N32_GTIMA4_CCMOD2           (N32_GTIMERA4_BASE + N32_GTIMA_CCMOD2_OFFSET)
#define N32_GTIMA4_CCEN             (N32_GTIMERA4_BASE + N32_GTIMA_CCEN_OFFSET)
#define N32_GTIMA4_CCDAT1           (N32_GTIMERA4_BASE + N32_GTIMA_CCDAT1_OFFSET)
#define N32_GTIMA4_CCDAT2           (N32_GTIMERA4_BASE + N32_GTIMA_CCDAT2_OFFSET)
#define N32_GTIMA4_CCDAT3           (N32_GTIMERA4_BASE + N32_GTIMA_CCDAT3_OFFSET)
#define N32_GTIMA4_CCDAT4           (N32_GTIMERA4_BASE + N32_GTIMA_CCDAT4_OFFSET)
#define N32_GTIMA4_PSC              (N32_GTIMERA4_BASE + N32_GTIMA_PSC_OFFSET)
#define N32_GTIMA4_AR               (N32_GTIMERA4_BASE + N32_GTIMA_AR_OFFSET)
#define N32_GTIMA4_CNT              (N32_GTIMERA4_BASE + N32_GTIMA_CNT_OFFSET)
#define N32_GTIMA4_C1FILT           (N32_GTIMERA4_BASE + N32_GTIMA_C1FILT_OFFSET)
#define N32_GTIMA4_C2FILT           (N32_GTIMERA4_BASE + N32_GTIMA_C2FILT_OFFSET)
#define N32_GTIMA4_C3FILT           (N32_GTIMERA4_BASE + N32_GTIMA_C3FILT_OFFSET)
#define N32_GTIMA4_C4FILT           (N32_GTIMERA4_BASE + N32_GTIMA_C4FILT_OFFSET)
#define N32_GTIMA4_FILTO            (N32_GTIMERA4_BASE + N32_GTIMA_FILTO_OFFSET)
#define N32_GTIMA4_INSEL            (N32_GTIMERA4_BASE + N32_GTIMA_INSEL_OFFSET)
#define N32_GTIMA4_DCTRL            (N32_GTIMERA4_BASE + N32_GTIMA_DCTRL_OFFSET)
#define N32_GTIMA4_DADDR            (N32_GTIMERA4_BASE + N32_GTIMA_DADDR_OFFSET)

/* Register Addresses - GTIMA5 **********************************************/

#define N32_GTIMA5_CTRL1            (N32_GTIMERA5_BASE + N32_GTIMA_CTRL1_OFFSET)
#define N32_GTIMA5_CTRL2            (N32_GTIMERA5_BASE + N32_GTIMA_CTRL2_OFFSET)
#define N32_GTIMA5_STS              (N32_GTIMERA5_BASE + N32_GTIMA_STS_OFFSET)
#define N32_GTIMA5_ETGEN            (N32_GTIMERA5_BASE + N32_GTIMA_ETGEN_OFFSET)
#define N32_GTIMA5_SMCTRL           (N32_GTIMERA5_BASE + N32_GTIMA_SMCTRL_OFFSET)
#define N32_GTIMA5_DINTEN           (N32_GTIMERA5_BASE + N32_GTIMA_DINTEN_OFFSET)
#define N32_GTIMA5_CCMOD1           (N32_GTIMERA5_BASE + N32_GTIMA_CCMOD1_OFFSET)
#define N32_GTIMA5_CCMOD2           (N32_GTIMERA5_BASE + N32_GTIMA_CCMOD2_OFFSET)
#define N32_GTIMA5_CCEN             (N32_GTIMERA5_BASE + N32_GTIMA_CCEN_OFFSET)
#define N32_GTIMA5_CCDAT1           (N32_GTIMERA5_BASE + N32_GTIMA_CCDAT1_OFFSET)
#define N32_GTIMA5_CCDAT2           (N32_GTIMERA5_BASE + N32_GTIMA_CCDAT2_OFFSET)
#define N32_GTIMA5_CCDAT3           (N32_GTIMERA5_BASE + N32_GTIMA_CCDAT3_OFFSET)
#define N32_GTIMA5_CCDAT4           (N32_GTIMERA5_BASE + N32_GTIMA_CCDAT4_OFFSET)
#define N32_GTIMA5_PSC              (N32_GTIMERA5_BASE + N32_GTIMA_PSC_OFFSET)
#define N32_GTIMA5_AR               (N32_GTIMERA5_BASE + N32_GTIMA_AR_OFFSET)
#define N32_GTIMA5_CNT              (N32_GTIMERA5_BASE + N32_GTIMA_CNT_OFFSET)
#define N32_GTIMA5_C1FILT           (N32_GTIMERA5_BASE + N32_GTIMA_C1FILT_OFFSET)
#define N32_GTIMA5_C2FILT           (N32_GTIMERA5_BASE + N32_GTIMA_C2FILT_OFFSET)
#define N32_GTIMA5_C3FILT           (N32_GTIMERA5_BASE + N32_GTIMA_C3FILT_OFFSET)
#define N32_GTIMA5_C4FILT           (N32_GTIMERA5_BASE + N32_GTIMA_C4FILT_OFFSET)
#define N32_GTIMA5_FILTO            (N32_GTIMERA5_BASE + N32_GTIMA_FILTO_OFFSET)
#define N32_GTIMA5_INSEL            (N32_GTIMERA5_BASE + N32_GTIMA_INSEL_OFFSET)
#define N32_GTIMA5_DCTRL            (N32_GTIMERA5_BASE + N32_GTIMA_DCTRL_OFFSET)
#define N32_GTIMA5_DADDR            (N32_GTIMERA5_BASE + N32_GTIMA_DADDR_OFFSET)

/* Register Addresses - GTIMA6 **********************************************/

#define N32_GTIMA6_CTRL1            (N32_GTIMERA6_BASE + N32_GTIMA_CTRL1_OFFSET)
#define N32_GTIMA6_CTRL2            (N32_GTIMERA6_BASE + N32_GTIMA_CTRL2_OFFSET)
#define N32_GTIMA6_STS              (N32_GTIMERA6_BASE + N32_GTIMA_STS_OFFSET)
#define N32_GTIMA6_ETGEN            (N32_GTIMERA6_BASE + N32_GTIMA_ETGEN_OFFSET)
#define N32_GTIMA6_SMCTRL           (N32_GTIMERA6_BASE + N32_GTIMA_SMCTRL_OFFSET)
#define N32_GTIMA6_DINTEN           (N32_GTIMERA6_BASE + N32_GTIMA_DINTEN_OFFSET)
#define N32_GTIMA6_CCMOD1           (N32_GTIMERA6_BASE + N32_GTIMA_CCMOD1_OFFSET)
#define N32_GTIMA6_CCMOD2           (N32_GTIMERA6_BASE + N32_GTIMA_CCMOD2_OFFSET)
#define N32_GTIMA6_CCEN             (N32_GTIMERA6_BASE + N32_GTIMA_CCEN_OFFSET)
#define N32_GTIMA6_CCDAT1           (N32_GTIMERA6_BASE + N32_GTIMA_CCDAT1_OFFSET)
#define N32_GTIMA6_CCDAT2           (N32_GTIMERA6_BASE + N32_GTIMA_CCDAT2_OFFSET)
#define N32_GTIMA6_CCDAT3           (N32_GTIMERA6_BASE + N32_GTIMA_CCDAT3_OFFSET)
#define N32_GTIMA6_CCDAT4           (N32_GTIMERA6_BASE + N32_GTIMA_CCDAT4_OFFSET)
#define N32_GTIMA6_PSC              (N32_GTIMERA6_BASE + N32_GTIMA_PSC_OFFSET)
#define N32_GTIMA6_AR               (N32_GTIMERA6_BASE + N32_GTIMA_AR_OFFSET)
#define N32_GTIMA6_CNT              (N32_GTIMERA6_BASE + N32_GTIMA_CNT_OFFSET)
#define N32_GTIMA6_C1FILT           (N32_GTIMERA6_BASE + N32_GTIMA_C1FILT_OFFSET)
#define N32_GTIMA6_C2FILT           (N32_GTIMERA6_BASE + N32_GTIMA_C2FILT_OFFSET)
#define N32_GTIMA6_C3FILT           (N32_GTIMERA6_BASE + N32_GTIMA_C3FILT_OFFSET)
#define N32_GTIMA6_C4FILT           (N32_GTIMERA6_BASE + N32_GTIMA_C4FILT_OFFSET)
#define N32_GTIMA6_FILTO            (N32_GTIMERA6_BASE + N32_GTIMA_FILTO_OFFSET)
#define N32_GTIMA6_INSEL            (N32_GTIMERA6_BASE + N32_GTIMA_INSEL_OFFSET)
#define N32_GTIMA6_DCTRL            (N32_GTIMERA6_BASE + N32_GTIMA_DCTRL_OFFSET)
#define N32_GTIMA6_DADDR            (N32_GTIMERA6_BASE + N32_GTIMA_DADDR_OFFSET)

/* Register Addresses - GTIMA7 **********************************************/

#define N32_GTIMA7_CTRL1            (N32_GTIMERA7_BASE + N32_GTIMA_CTRL1_OFFSET)
#define N32_GTIMA7_CTRL2            (N32_GTIMERA7_BASE + N32_GTIMA_CTRL2_OFFSET)
#define N32_GTIMA7_STS              (N32_GTIMERA7_BASE + N32_GTIMA_STS_OFFSET)
#define N32_GTIMA7_ETGEN            (N32_GTIMERA7_BASE + N32_GTIMA_ETGEN_OFFSET)
#define N32_GTIMA7_SMCTRL           (N32_GTIMERA7_BASE + N32_GTIMA_SMCTRL_OFFSET)
#define N32_GTIMA7_DINTEN           (N32_GTIMERA7_BASE + N32_GTIMA_DINTEN_OFFSET)
#define N32_GTIMA7_CCMOD1           (N32_GTIMERA7_BASE + N32_GTIMA_CCMOD1_OFFSET)
#define N32_GTIMA7_CCMOD2           (N32_GTIMERA7_BASE + N32_GTIMA_CCMOD2_OFFSET)
#define N32_GTIMA7_CCEN             (N32_GTIMERA7_BASE + N32_GTIMA_CCEN_OFFSET)
#define N32_GTIMA7_CCDAT1           (N32_GTIMERA7_BASE + N32_GTIMA_CCDAT1_OFFSET)
#define N32_GTIMA7_CCDAT2           (N32_GTIMERA7_BASE + N32_GTIMA_CCDAT2_OFFSET)
#define N32_GTIMA7_CCDAT3           (N32_GTIMERA7_BASE + N32_GTIMA_CCDAT3_OFFSET)
#define N32_GTIMA7_CCDAT4           (N32_GTIMERA7_BASE + N32_GTIMA_CCDAT4_OFFSET)
#define N32_GTIMA7_PSC              (N32_GTIMERA7_BASE + N32_GTIMA_PSC_OFFSET)
#define N32_GTIMA7_AR               (N32_GTIMERA7_BASE + N32_GTIMA_AR_OFFSET)
#define N32_GTIMA7_CNT              (N32_GTIMERA7_BASE + N32_GTIMA_CNT_OFFSET)
#define N32_GTIMA7_C1FILT           (N32_GTIMERA7_BASE + N32_GTIMA_C1FILT_OFFSET)
#define N32_GTIMA7_C2FILT           (N32_GTIMERA7_BASE + N32_GTIMA_C2FILT_OFFSET)
#define N32_GTIMA7_C3FILT           (N32_GTIMERA7_BASE + N32_GTIMA_C3FILT_OFFSET)
#define N32_GTIMA7_C4FILT           (N32_GTIMERA7_BASE + N32_GTIMA_C4FILT_OFFSET)
#define N32_GTIMA7_FILTO            (N32_GTIMERA7_BASE + N32_GTIMA_FILTO_OFFSET)
#define N32_GTIMA7_INSEL            (N32_GTIMERA7_BASE + N32_GTIMA_INSEL_OFFSET)
#define N32_GTIMA7_DCTRL            (N32_GTIMERA7_BASE + N32_GTIMA_DCTRL_OFFSET)
#define N32_GTIMA7_DADDR            (N32_GTIMERA7_BASE + N32_GTIMA_DADDR_OFFSET)

/* Bitfield Definitions - GTIMA *********************************************/

/* Control register 1 (N32_GTIMA_CTRL1) */

#define N32_GTIMA_CTRL1_CNTEN              (1 << 0)  /* Counter enable */
#define N32_GTIMA_CTRL1_DIR                (1 << 1)  /* Direction */
#define N32_GTIMA_CTRL1_CAMSEL_SHIFT       (2)       /* Center-aligned mode selection */
#define N32_GTIMA_CTRL1_CAMSEL_MASK        (3 << N32_GTIMA_CTRL1_CAMSEL_SHIFT)
#  define N32_GTIMA_CTRL1_EDGE             (0 << N32_GTIMA_CTRL1_CAMSEL_SHIFT)
#  define N32_GTIMA_CTRL1_CENTER1          (1 << N32_GTIMA_CTRL1_CAMSEL_SHIFT)
#  define N32_GTIMA_CTRL1_CENTER2          (2 << N32_GTIMA_CTRL1_CAMSEL_SHIFT)
#  define N32_GTIMA_CTRL1_CENTER3          (3 << N32_GTIMA_CTRL1_CAMSEL_SHIFT)
#define N32_GTIMA_CTRL1_UPRS               (1 << 4)  /* Update request source */
#define N32_GTIMA_CTRL1_UPDIS              (1 << 5)  /* Update disable */
#define N32_GTIMA_CTRL1_CLKD_SHIFT         (6)       /* Clock division */
#define N32_GTIMA_CTRL1_CLKD_MASK          (3 << N32_GTIMA_CTRL1_CLKD_SHIFT)
#  define N32_GTIMA_CTRL1_TCKINT           (0 << N32_GTIMA_CTRL1_CLKD_SHIFT)
#  define N32_GTIMA_CTRL1_2TCKINT          (1 << N32_GTIMA_CTRL1_CLKD_SHIFT)
#  define N32_GTIMA_CTRL1_4TCKINT          (2 << N32_GTIMA_CTRL1_CLKD_SHIFT)
#define N32_GTIMA_CTRL1_ONEPM              (1 << 8)  /* One pulse mode */
#define N32_GTIMA_CTRL1_ARPEN              (1 << 9)  /* Auto-reload preload enable */
#define N32_GTIMA_CTRL1_CLRSEL             (1 << 13) /* OcxRef clear selection */

/* GTIMA1-specific channel source selection */
#define N32_GTIMA_CTRL1_C1SEL              (1 << 16) /* GTIMA1: LSE for CH1 */
#define N32_GTIMA_CTRL1_C2SEL              (1 << 17) /* GTIMA1: LSE for CH2 */
#define N32_GTIMA_CTRL1_C3SEL              (1 << 18) /* GTIMA1: LSI for CH3 */
#define N32_GTIMA_CTRL1_C4SEL              (1 << 19) /* GTIMA1: HSE/128 for CH4 */

/* Control register 2 (N32_GTIMA_CTRL2) */

#define N32_GTIMA_CTRL2_MMSEL_SHIFT        (4)       /* Master mode selection (TRGO) */
#define N32_GTIMA_CTRL2_MMSEL_MASK         (7 << N32_GTIMA_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMA_CTRL2_MMSEL_RESET      (0 << N32_GTIMA_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMA_CTRL2_MMSEL_ENABLE     (1 << N32_GTIMA_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMA_CTRL2_MMSEL_UPDATE     (2 << N32_GTIMA_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMA_CTRL2_MMSEL_COMPP      (3 << N32_GTIMA_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMA_CTRL2_MMSEL_OC1REF     (4 << N32_GTIMA_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMA_CTRL2_MMSEL_OC2REF     (5 << N32_GTIMA_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMA_CTRL2_MMSEL_OC3REF     (6 << N32_GTIMA_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMA_CTRL2_MMSEL_OC4REF     (7 << N32_GTIMA_CTRL2_MMSEL_SHIFT)

/* Status register (N32_GTIMA_STS) */

#define N32_GTIMA_STS_CC1ITF              (1 << 0)  /* CC1 interrupt flag */
#define N32_GTIMA_STS_CC2ITF              (1 << 1)  /* CC2 interrupt flag */
#define N32_GTIMA_STS_CC3ITF              (1 << 2)  /* CC3 interrupt flag */
#define N32_GTIMA_STS_CC4ITF              (1 << 3)  /* CC4 interrupt flag */
#define N32_GTIMA_STS_CC1OCF              (1 << 8)  /* CC1 overcapture flag */
#define N32_GTIMA_STS_CC2OCF              (1 << 9)  /* CC2 overcapture flag */
#define N32_GTIMA_STS_CC3OCF              (1 << 10) /* CC3 overcapture flag */
#define N32_GTIMA_STS_CC4OCF              (1 << 11) /* CC4 overcapture flag */
#define N32_GTIMA_STS_UDITF               (1 << 16) /* Update interrupt flag */
#define N32_GTIMA_STS_TITF                (1 << 18) /* Trigger interrupt flag */

/* Event generation register (N32_GTIMA_ETGEN) */

#define N32_GTIMA_ETGEN_CC1GN             (1 << 0)  /* CC1 generation */
#define N32_GTIMA_ETGEN_CC2GN             (1 << 1)  /* CC2 generation */
#define N32_GTIMA_ETGEN_CC3GN             (1 << 2)  /* CC3 generation */
#define N32_GTIMA_ETGEN_CC4GN             (1 << 3)  /* CC4 generation */
#define N32_GTIMA_ETGEN_UDGN              (1 << 8)  /* Update generation */
#define N32_GTIMA_ETGEN_TGN               (1 << 10) /* Trigger generation */

/* Slave mode control (N32_GTIMA_SMCTRL) */

#define N32_GTIMA_SMCTRL_TSEL_SHIFT       (0)       /* Trigger selection */
#define N32_GTIMA_SMCTRL_TSEL_MASK        (7 << N32_GTIMA_SMCTRL_TSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_ITR            (0 << N32_GTIMA_SMCTRL_TSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_TI1FED         (4 << N32_GTIMA_SMCTRL_TSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_TI1FP1         (5 << N32_GTIMA_SMCTRL_TSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_TI2FP2         (6 << N32_GTIMA_SMCTRL_TSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_ETRF           (7 << N32_GTIMA_SMCTRL_TSEL_SHIFT)
#define N32_GTIMA_SMCTRL_SMSEL_SHIFT      (4)       /* Slave mode selection */
#define N32_GTIMA_SMCTRL_SMSEL_MASK       (15 << N32_GTIMA_SMCTRL_SMSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_DISAB          (0 << N32_GTIMA_SMCTRL_SMSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_ENCMD1         (1 << N32_GTIMA_SMCTRL_SMSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_ENCMD2         (2 << N32_GTIMA_SMCTRL_SMSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_ENCMD3         (3 << N32_GTIMA_SMCTRL_SMSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_RESET          (4 << N32_GTIMA_SMCTRL_SMSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_GATED          (5 << N32_GTIMA_SMCTRL_SMSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_TRIGGER        (6 << N32_GTIMA_SMCTRL_SMSEL_SHIFT)
#  define N32_GTIMA_SMCTRL_EXTCLK1        (7 << N32_GTIMA_SMCTRL_SMSEL_SHIFT)
#define N32_GTIMA_SMCTRL_EXTPS_SHIFT      (8)       /* External trigger prescaler */
#define N32_GTIMA_SMCTRL_EXTPS_MASK       (3 << N32_GTIMA_SMCTRL_EXTPS_SHIFT)
#  define N32_GTIMA_SMCTRL_PSCOFF         (0 << N32_GTIMA_SMCTRL_EXTPS_SHIFT)
#  define N32_GTIMA_SMCTRL_ETRPd2         (1 << N32_GTIMA_SMCTRL_EXTPS_SHIFT)
#  define N32_GTIMA_SMCTRL_ETRPd4         (2 << N32_GTIMA_SMCTRL_EXTPS_SHIFT)
#  define N32_GTIMA_SMCTRL_ETRPd8         (3 << N32_GTIMA_SMCTRL_EXTPS_SHIFT)
#define N32_GTIMA_SMCTRL_EXCEN            (1 << 10) /* External clock enable */
#define N32_GTIMA_SMCTRL_EXTP             (1 << 11) /* External trigger polarity */
#define N32_GTIMA_SMCTRL_EXTF_SHIFT       (12)      /* External trigger filter */
#define N32_GTIMA_SMCTRL_EXTF_MASK        (15 << N32_GTIMA_SMCTRL_EXTF_SHIFT)
#define N32_GTIMA_SMCTRL_MSMD             (1 << 16) /* Master/slave mode */
#define N32_GTIMA_SMCTRL_OCRECFLRP        (1 << 19) /* tim_ocref_clr polarity */
#define N32_GTIMA_SMCTRL_OCRECFLRF_SHIFT  (20)      /* tim_ocref_clr filter */
#define N32_GTIMA_SMCTRL_OCRECFLRF_MASK   (15 << N32_GTIMA_SMCTRL_OCRECFLRF_SHIFT)

/* DMA/Interrupt enable (N32_GTIMA_DINTEN) */

#define N32_GTIMA_DINTEN_CC1IEN           (1 << 0)  /* CC1 interrupt enable */
#define N32_GTIMA_DINTEN_CC2IEN           (1 << 1)  /* CC2 interrupt enable */
#define N32_GTIMA_DINTEN_CC3IEN           (1 << 2)  /* CC3 interrupt enable */
#define N32_GTIMA_DINTEN_CC4IEN           (1 << 3)  /* CC4 interrupt enable */
#define N32_GTIMA_DINTEN_CC1DEN           (1 << 8)  /* CC1 DMA request enable */
#define N32_GTIMA_DINTEN_CC2DEN           (1 << 9)  /* CC2 DMA request enable */
#define N32_GTIMA_DINTEN_CC3DEN           (1 << 10) /* CC3 DMA request enable */
#define N32_GTIMA_DINTEN_CC4DEN           (1 << 11) /* CC4 DMA request enable */
#define N32_GTIMA_DINTEN_UIEN             (1 << 16) /* Update interrupt enable */
#define N32_GTIMA_DINTEN_TIEN             (1 << 17) /* Trigger interrupt enable */
#define N32_GTIMA_DINTEN_UDEN             (1 << 19) /* Update DMA request enable */
#define N32_GTIMA_DINTEN_TDEN             (1 << 21) /* Trigger DMA request enable */

/* CCMOD1 - Output compare (N32_GTIMA_CCMOD1) */

#define N32_GTIMA_CCMOD1_CC1SEL_SHIFT     (0)       /* CC1 selection */
#define N32_GTIMA_CCMOD1_CC1SEL_MASK      (3 << N32_GTIMA_CCMOD1_CC1SEL_SHIFT)
#  define N32_GTIMA_CCMOD1_CC1OUT         (0 << N32_GTIMA_CCMOD1_CC1SEL_SHIFT)
#  define N32_GTIMA_CCMOD1_CC1IN_TI1      (1 << N32_GTIMA_CCMOD1_CC1SEL_SHIFT)
#  define N32_GTIMA_CCMOD1_CC1IN_TI2      (2 << N32_GTIMA_CCMOD1_CC1SEL_SHIFT)
#  define N32_GTIMA_CCMOD1_CC1IN_TRC      (3 << N32_GTIMA_CCMOD1_CC1SEL_SHIFT)
#define N32_GTIMA_CCMOD1_OC1PEN           (1 << 2)  /* OC1 preload enable */
#define N32_GTIMA_CCMOD1_OC1FEN           (1 << 3)  /* OC1 fast enable */
#define N32_GTIMA_CCMOD1_OC1CEN           (1 << 4)  /* OC1 clear enable */
#define N32_GTIMA_CCMOD1_OC1MD_SHIFT      (5)       /* OC1 mode */
#define N32_GTIMA_CCMOD1_OC1MD_MASK       (7 << N32_GTIMA_CCMOD1_OC1MD_SHIFT)
#define N32_GTIMA_CCMOD1_CC2SEL_SHIFT     (8)       /* CC2 selection */
#define N32_GTIMA_CCMOD1_CC2SEL_MASK      (3 << N32_GTIMA_CCMOD1_CC2SEL_SHIFT)
#  define N32_GTIMA_CCMOD1_CC2OUT         (0 << N32_GTIMA_CCMOD1_CC2SEL_SHIFT)
#  define N32_GTIMA_CCMOD1_CC2IN_TI2      (1 << N32_GTIMA_CCMOD1_CC2SEL_SHIFT)
#  define N32_GTIMA_CCMOD1_CC2IN_TI1      (2 << N32_GTIMA_CCMOD1_CC2SEL_SHIFT)
#  define N32_GTIMA_CCMOD1_CC2IN_TRC      (3 << N32_GTIMA_CCMOD1_CC2SEL_SHIFT)
#define N32_GTIMA_CCMOD1_OC2PEN           (1 << 10) /* OC2 preload enable */
#define N32_GTIMA_CCMOD1_OC2FEN           (1 << 11) /* OC2 fast enable */
#define N32_GTIMA_CCMOD1_OC2CEN           (1 << 12) /* OC2 clear enable */
#define N32_GTIMA_CCMOD1_OC2MD_SHIFT      (13)      /* OC2 mode */
#define N32_GTIMA_CCMOD1_OC2MD_MASK       (7 << N32_GTIMA_CCMOD1_OC2MD_SHIFT)

/* CCMOD1 - Input capture (N32_GTIMA_CCMOD1) */

#define N32_GTIMA_CCMOD1_IC1PSC_SHIFT     (2)       /* IC1 prescaler */
#define N32_GTIMA_CCMOD1_IC1PSC_MASK      (3 << N32_GTIMA_CCMOD1_IC1PSC_SHIFT)
#define N32_GTIMA_CCMOD1_IC1F_SHIFT       (4)       /* IC1 filter */
#define N32_GTIMA_CCMOD1_IC1F_MASK        (15 << N32_GTIMA_CCMOD1_IC1F_SHIFT)
#define N32_GTIMA_CCMOD1_IC2PSC_SHIFT     (10)      /* IC2 prescaler */
#define N32_GTIMA_CCMOD1_IC2PSC_MASK      (3 << N32_GTIMA_CCMOD1_IC2PSC_SHIFT)
#define N32_GTIMA_CCMOD1_IC2F_SHIFT       (12)      /* IC2 filter */
#define N32_GTIMA_CCMOD1_IC2F_MASK        (15 << N32_GTIMA_CCMOD1_IC2F_SHIFT)

/* CCMOD2 - Output compare (N32_GTIMA_CCMOD2) */

#define N32_GTIMA_CCMOD2_CC3SEL_SHIFT     (0)       /* CC3 selection */
#define N32_GTIMA_CCMOD2_CC3SEL_MASK      (3 << N32_GTIMA_CCMOD2_CC3SEL_SHIFT)
#  define N32_GTIMA_CCMOD2_CC3OUT         (0 << N32_GTIMA_CCMOD2_CC3SEL_SHIFT)
#  define N32_GTIMA_CCMOD2_CC3IN_TI3      (1 << N32_GTIMA_CCMOD2_CC3SEL_SHIFT)
#  define N32_GTIMA_CCMOD2_CC3IN_TI4      (2 << N32_GTIMA_CCMOD2_CC3SEL_SHIFT)
#  define N32_GTIMA_CCMOD2_CC3IN_TRC      (3 << N32_GTIMA_CCMOD2_CC3SEL_SHIFT)
#define N32_GTIMA_CCMOD2_OC3PEN           (1 << 2)  /* OC3 preload enable */
#define N32_GTIMA_CCMOD2_OC3FEN           (1 << 3)  /* OC3 fast enable */
#define N32_GTIMA_CCMOD2_OC3CEN           (1 << 4)  /* OC3 clear enable */
#define N32_GTIMA_CCMOD2_OC3MD_SHIFT      (5)       /* OC3 mode */
#define N32_GTIMA_CCMOD2_OC3MD_MASK       (7 << N32_GTIMA_CCMOD2_OC3MD_SHIFT)
#define N32_GTIMA_CCMOD2_CC4SEL_SHIFT     (8)       /* CC4 selection */
#define N32_GTIMA_CCMOD2_CC4SEL_MASK      (3 << N32_GTIMA_CCMOD2_CC4SEL_SHIFT)
#  define N32_GTIMA_CCMOD2_CC4OUT         (0 << N32_GTIMA_CCMOD2_CC4SEL_SHIFT)
#  define N32_GTIMA_CCMOD2_CC4IN_TI4      (1 << N32_GTIMA_CCMOD2_CC4SEL_SHIFT)
#  define N32_GTIMA_CCMOD2_CC4IN_TI3      (2 << N32_GTIMA_CCMOD2_CC4SEL_SHIFT)
#  define N32_GTIMA_CCMOD2_CC4IN_TRC      (3 << N32_GTIMA_CCMOD2_CC4SEL_SHIFT)
#define N32_GTIMA_CCMOD2_OC4PEN           (1 << 10) /* OC4 preload enable */
#define N32_GTIMA_CCMOD2_OC4FEN           (1 << 11) /* OC4 fast enable */
#define N32_GTIMA_CCMOD2_OC4CEN           (1 << 12) /* OC4 clear enable */
#define N32_GTIMA_CCMOD2_OC4MD_SHIFT      (13)      /* OC4 mode */
#define N32_GTIMA_CCMOD2_OC4MD_MASK       (7 << N32_GTIMA_CCMOD2_OC4MD_SHIFT)

/* CCMOD2 - Input capture (N32_GTIMA_CCMOD2) */

#define N32_GTIMA_CCMOD2_IC3PSC_SHIFT     (2)       /* IC3 prescaler */
#define N32_GTIMA_CCMOD2_IC3PSC_MASK      (3 << N32_GTIMA_CCMOD2_IC3PSC_SHIFT)
#define N32_GTIMA_CCMOD2_IC3F_SHIFT       (4)       /* IC3 filter */
#define N32_GTIMA_CCMOD2_IC3F_MASK        (15 << N32_GTIMA_CCMOD2_IC3F_SHIFT)
#define N32_GTIMA_CCMOD2_IC4PSC_SHIFT     (10)      /* IC4 prescaler */
#define N32_GTIMA_CCMOD2_IC4PSC_MASK      (3 << N32_GTIMA_CCMOD2_IC4PSC_SHIFT)
#define N32_GTIMA_CCMOD2_IC4F_SHIFT       (12)      /* IC4 filter */
#define N32_GTIMA_CCMOD2_IC4F_MASK        (15 << N32_GTIMA_CCMOD2_IC4F_SHIFT)

/* CCEN (N32_GTIMA_CCEN) - No complementary outputs */

#define N32_GTIMA_CCEN_CC1EN             (1 << 0)  /* CC1 output enable */
#define N32_GTIMA_CCEN_CC1P              (1 << 1)  /* CC1 output polarity */
#define N32_GTIMA_CCEN_CC2EN             (1 << 2)  /* CC2 output enable */
#define N32_GTIMA_CCEN_CC2P              (1 << 3)  /* CC2 output polarity */
#define N32_GTIMA_CCEN_CC3EN             (1 << 4)  /* CC3 output enable */
#define N32_GTIMA_CCEN_CC3P              (1 << 5)  /* CC3 output polarity */
#define N32_GTIMA_CCEN_CC4EN             (1 << 6)  /* CC4 output enable */
#define N32_GTIMA_CCEN_CC4P              (1 << 7)  /* CC4 output polarity */

/* CCDAT registers, PSC, AR, CNT */

#define N32_GTIMA_CCDAT_MASK             (0xffff)  /* CCx value mask */
#define N32_GTIMA_PSC_MASK               (0xffff)  /* Prescaler value mask */
#define N32_GTIMA_AR_MASK                (0xffff)  /* Auto-reload value mask */
#define N32_GTIMA_CNT_MASK               (0xffff)  /* Counter value mask */

/* Filter registers (C1FILT..C4FILT) */

#define N32_GTIMA_CXFILT_SLIDFPSC_MASK   (0xffff)  /* Sampling clock prescaler */
#define N32_GTIMA_CXFILT_FILTEN          (1 << 16) /* Filter enable */
#define N32_GTIMA_CXFILT_WSIZE_SHIFT     (17)      /* Window size */
#define N32_GTIMA_CXFILT_WSIZE_MASK      (63 << N32_GTIMA_CXFILT_WSIZE_SHIFT)
#define N32_GTIMA_CXFILT_THRESH_SHIFT    (24)      /* Threshold */
#define N32_GTIMA_CXFILT_THRESH_MASK     (63 << N32_GTIMA_CXFILT_THRESH_SHIFT)

/* FILTO register */

#define N32_GTIMA_FILTO_C1FILTO          (1 << 0)  /* Channel 1 filter output */
#define N32_GTIMA_FILTO_C2FILTO          (1 << 1)  /* Channel 2 filter output */
#define N32_GTIMA_FILTO_C3FILTO          (1 << 2)  /* Channel 3 filter output */
#define N32_GTIMA_FILTO_C4FILTO          (1 << 3)  /* Channel 4 filter output */

/* INSEL (same as ATIM) */

#define N32_GTIMA_INSEL_TI1S_SHIFT       (0)       /* TI1 input selection */
#define N32_GTIMA_INSEL_TI1S_MASK        (15 << N32_GTIMA_INSEL_TI1S_SHIFT)
#define N32_GTIMA_INSEL_TI2S_SHIFT       (4)       /* TI2 input selection */
#define N32_GTIMA_INSEL_TI2S_MASK        (15 << N32_GTIMA_INSEL_TI2S_SHIFT)
#define N32_GTIMA_INSEL_TI3S_SHIFT       (8)       /* TI3 input selection */
#define N32_GTIMA_INSEL_TI3S_MASK        (15 << N32_GTIMA_INSEL_TI3S_SHIFT)
#define N32_GTIMA_INSEL_TI4S_SHIFT       (12)      /* TI4 input selection */
#define N32_GTIMA_INSEL_TI4S_MASK        (15 << N32_GTIMA_INSEL_TI4S_SHIFT)
#define N32_GTIMA_INSEL_ETRS_SHIFT       (16)      /* ETR input selection */
#define N32_GTIMA_INSEL_ETRS_MASK        (15 << N32_GTIMA_INSEL_ETRS_SHIFT)
#define N32_GTIMA_INSEL_ITRS_SHIFT       (20)      /* ITR input selection */
#define N32_GTIMA_INSEL_ITRS_MASK        (15 << N32_GTIMA_INSEL_ITRS_SHIFT)
#define N32_GTIMA_INSEL_CLRS_SHIFT       (24)      /* OcxRef clear input selection */
#define N32_GTIMA_INSEL_CLRS_MASK        (15 << N32_GTIMA_INSEL_CLRS_SHIFT)

/* DCTRL/DADDR */

#define N32_GTIMA_DCTRL_DBLEN_SHIFT      (0)       /* DMA burst length */
#define N32_GTIMA_DCTRL_DBLEN_MASK       (63 << N32_GTIMA_DCTRL_DBLEN_SHIFT)
#  define N32_GTIMA_DCTRL_DBLEN(n)       (((n)-1) << N32_GTIMA_DCTRL_DBLEN_SHIFT)
#define N32_GTIMA_DCTRL_DBADDR_SHIFT     (8)       /* DMA base address */
#define N32_GTIMA_DCTRL_DBADDR_MASK      (63 << N32_GTIMA_DCTRL_DBADDR_SHIFT)
#define N32_GTIMA_DADDR_BURST_MASK       (0xffffffff)

/* ==========================================================================
 *                     GENERAL-PURPOSE TIMERS B (GTIMB1-3)
 * ==========================================================================
 */

/* Register Offsets - GTIMB (common for all GTIMB1-3) ***********************/

#define N32_GTIMB_CTRL1_OFFSET      0x0000  /* Control register 1 */
#define N32_GTIMB_CTRL2_OFFSET      0x0004  /* Control register 2 */
#define N32_GTIMB_STS_OFFSET        0x0008  /* Status register */
#define N32_GTIMB_ETGEN_OFFSET      0x000c  /* Event generation register */
#define N32_GTIMB_SMCTRL_OFFSET     0x0010  /* Slave mode control register */
#define N32_GTIMB_DINTEN_OFFSET     0x0014  /* DMA/Interrupt enable register */
#define N32_GTIMB_CCMOD1_OFFSET     0x0018  /* Capture/compare mode register 1 */
#define N32_GTIMB_CCMOD2_OFFSET     0x001c  /* Capture/compare mode register 2 */
#define N32_GTIMB_CCMOD3_OFFSET     0x0020  /* Capture/compare mode register 3 */
#define N32_GTIMB_CCEN_OFFSET       0x0024  /* Capture/compare enable register */
#define N32_GTIMB_CCDAT1_OFFSET     0x0028  /* Capture/compare register 1 */
#define N32_GTIMB_CCDAT2_OFFSET     0x002c  /* Capture/compare register 2 */
#define N32_GTIMB_CCDAT3_OFFSET     0x0030  /* Capture/compare register 3 */
#define N32_GTIMB_CCDAT4_OFFSET     0x0034  /* Capture/compare register 4 */
#define N32_GTIMB_CCDAT5_OFFSET     0x0038  /* Capture/compare register 5 */
#define N32_GTIMB_PSC_OFFSET        0x0040  /* Prescaler */
#define N32_GTIMB_AR_OFFSET         0x0044  /* Auto-reload register */
#define N32_GTIMB_CNT_OFFSET        0x0048  /* Counter */
#define N32_GTIMB_REPCNT_OFFSET     0x004c  /* Repetition counter register */
#define N32_GTIMB_BKDT_OFFSET       0x0050  /* Break and dead-time register */
#define N32_GTIMB_BKFR_OFFSET       0x0060  /* Break 1 filter register */
#define N32_GTIMB_C1FILT_OFFSET     0x0064  /* Channel 1 filter register */
#define N32_GTIMB_C2FILT_OFFSET     0x0068  /* Channel 2 filter register */
#define N32_GTIMB_C3FILT_OFFSET     0x006c  /* Channel 3 filter register */
#define N32_GTIMB_C4FILT_OFFSET     0x0070  /* Channel 4 filter register */
#define N32_GTIMB_FILTO_OFFSET      0x0074  /* Filter output register */
#define N32_GTIMB_INSEL_OFFSET      0x0078  /* Input selection register */
#define N32_GTIMB_AF1_OFFSET        0x007c  /* Alternate function register 1 */
#define N32_GTIMB_DCTRL_OFFSET      0x0094  /* DMA control register */
#define N32_GTIMB_DADDR_OFFSET      0x0098  /* DMA address for burst mode */

/* Register Addresses - GTIMB1 **********************************************/

#define N32_GTIMB1_CTRL1            (N32_GTIMERB1_BASE + N32_GTIMB_CTRL1_OFFSET)
#define N32_GTIMB1_CTRL2            (N32_GTIMERB1_BASE + N32_GTIMB_CTRL2_OFFSET)
#define N32_GTIMB1_STS              (N32_GTIMERB1_BASE + N32_GTIMB_STS_OFFSET)
#define N32_GTIMB1_ETGEN            (N32_GTIMERB1_BASE + N32_GTIMB_ETGEN_OFFSET)
#define N32_GTIMB1_SMCTRL           (N32_GTIMERB1_BASE + N32_GTIMB_SMCTRL_OFFSET)
#define N32_GTIMB1_DINTEN           (N32_GTIMERB1_BASE + N32_GTIMB_DINTEN_OFFSET)
#define N32_GTIMB1_CCMOD1           (N32_GTIMERB1_BASE + N32_GTIMB_CCMOD1_OFFSET)
#define N32_GTIMB1_CCMOD2           (N32_GTIMERB1_BASE + N32_GTIMB_CCMOD2_OFFSET)
#define N32_GTIMB1_CCMOD3           (N32_GTIMERB1_BASE + N32_GTIMB_CCMOD3_OFFSET)
#define N32_GTIMB1_CCEN             (N32_GTIMERB1_BASE + N32_GTIMB_CCEN_OFFSET)
#define N32_GTIMB1_CCDAT1           (N32_GTIMERB1_BASE + N32_GTIMB_CCDAT1_OFFSET)
#define N32_GTIMB1_CCDAT2           (N32_GTIMERB1_BASE + N32_GTIMB_CCDAT2_OFFSET)
#define N32_GTIMB1_CCDAT3           (N32_GTIMERB1_BASE + N32_GTIMB_CCDAT3_OFFSET)
#define N32_GTIMB1_CCDAT4           (N32_GTIMERB1_BASE + N32_GTIMB_CCDAT4_OFFSET)
#define N32_GTIMB1_CCDAT5           (N32_GTIMERB1_BASE + N32_GTIMB_CCDAT5_OFFSET)
#define N32_GTIMB1_PSC              (N32_GTIMERB1_BASE + N32_GTIMB_PSC_OFFSET)
#define N32_GTIMB1_AR               (N32_GTIMERB1_BASE + N32_GTIMB_AR_OFFSET)
#define N32_GTIMB1_CNT              (N32_GTIMERB1_BASE + N32_GTIMB_CNT_OFFSET)
#define N32_GTIMB1_REPCNT           (N32_GTIMERB1_BASE + N32_GTIMB_REPCNT_OFFSET)
#define N32_GTIMB1_BKDT             (N32_GTIMERB1_BASE + N32_GTIMB_BKDT_OFFSET)
#define N32_GTIMB1_BKFR             (N32_GTIMERB1_BASE + N32_GTIMB_BKFR_OFFSET)
#define N32_GTIMB1_C1FILT           (N32_GTIMERB1_BASE + N32_GTIMB_C1FILT_OFFSET)
#define N32_GTIMB1_C2FILT           (N32_GTIMERB1_BASE + N32_GTIMB_C2FILT_OFFSET)
#define N32_GTIMB1_C3FILT           (N32_GTIMERB1_BASE + N32_GTIMB_C3FILT_OFFSET)
#define N32_GTIMB1_C4FILT           (N32_GTIMERB1_BASE + N32_GTIMB_C4FILT_OFFSET)
#define N32_GTIMB1_FILTO            (N32_GTIMERB1_BASE + N32_GTIMB_FILTO_OFFSET)
#define N32_GTIMB1_INSEL            (N32_GTIMERB1_BASE + N32_GTIMB_INSEL_OFFSET)
#define N32_GTIMB1_AF1              (N32_GTIMERB1_BASE + N32_GTIMB_AF1_OFFSET)
#define N32_GTIMB1_DCTRL            (N32_GTIMERB1_BASE + N32_GTIMB_DCTRL_OFFSET)
#define N32_GTIMB1_DADDR            (N32_GTIMERB1_BASE + N32_GTIMB_DADDR_OFFSET)

/* Register Addresses - GTIMB2 **********************************************/

#define N32_GTIMB2_CTRL1            (N32_GTIMERB2_BASE + N32_GTIMB_CTRL1_OFFSET)
#define N32_GTIMB2_CTRL2            (N32_GTIMERB2_BASE + N32_GTIMB_CTRL2_OFFSET)
#define N32_GTIMB2_STS              (N32_GTIMERB2_BASE + N32_GTIMB_STS_OFFSET)
#define N32_GTIMB2_ETGEN            (N32_GTIMERB2_BASE + N32_GTIMB_ETGEN_OFFSET)
#define N32_GTIMB2_SMCTRL           (N32_GTIMERB2_BASE + N32_GTIMB_SMCTRL_OFFSET)
#define N32_GTIMB2_DINTEN           (N32_GTIMERB2_BASE + N32_GTIMB_DINTEN_OFFSET)
#define N32_GTIMB2_CCMOD1           (N32_GTIMERB2_BASE + N32_GTIMB_CCMOD1_OFFSET)
#define N32_GTIMB2_CCMOD2           (N32_GTIMERB2_BASE + N32_GTIMB_CCMOD2_OFFSET)
#define N32_GTIMB2_CCMOD3           (N32_GTIMERB2_BASE + N32_GTIMB_CCMOD3_OFFSET)
#define N32_GTIMB2_CCEN             (N32_GTIMERB2_BASE + N32_GTIMB_CCEN_OFFSET)
#define N32_GTIMB2_CCDAT1           (N32_GTIMERB2_BASE + N32_GTIMB_CCDAT1_OFFSET)
#define N32_GTIMB2_CCDAT2           (N32_GTIMERB2_BASE + N32_GTIMB_CCDAT2_OFFSET)
#define N32_GTIMB2_CCDAT3           (N32_GTIMERB2_BASE + N32_GTIMB_CCDAT3_OFFSET)
#define N32_GTIMB2_CCDAT4           (N32_GTIMERB2_BASE + N32_GTIMB_CCDAT4_OFFSET)
#define N32_GTIMB2_CCDAT5           (N32_GTIMERB2_BASE + N32_GTIMB_CCDAT5_OFFSET)
#define N32_GTIMB2_PSC              (N32_GTIMERB2_BASE + N32_GTIMB_PSC_OFFSET)
#define N32_GTIMB2_AR               (N32_GTIMERB2_BASE + N32_GTIMB_AR_OFFSET)
#define N32_GTIMB2_CNT              (N32_GTIMERB2_BASE + N32_GTIMB_CNT_OFFSET)
#define N32_GTIMB2_REPCNT           (N32_GTIMERB2_BASE + N32_GTIMB_REPCNT_OFFSET)
#define N32_GTIMB2_BKDT             (N32_GTIMERB2_BASE + N32_GTIMB_BKDT_OFFSET)
#define N32_GTIMB2_BKFR             (N32_GTIMERB2_BASE + N32_GTIMB_BKFR_OFFSET)
#define N32_GTIMB2_C1FILT           (N32_GTIMERB2_BASE + N32_GTIMB_C1FILT_OFFSET)
#define N32_GTIMB2_C2FILT           (N32_GTIMERB2_BASE + N32_GTIMB_C2FILT_OFFSET)
#define N32_GTIMB2_C3FILT           (N32_GTIMERB2_BASE + N32_GTIMB_C3FILT_OFFSET)
#define N32_GTIMB2_C4FILT           (N32_GTIMERB2_BASE + N32_GTIMB_C4FILT_OFFSET)
#define N32_GTIMB2_FILTO            (N32_GTIMERB2_BASE + N32_GTIMB_FILTO_OFFSET)
#define N32_GTIMB2_INSEL            (N32_GTIMERB2_BASE + N32_GTIMB_INSEL_OFFSET)
#define N32_GTIMB2_AF1              (N32_GTIMERB2_BASE + N32_GTIMB_AF1_OFFSET)
#define N32_GTIMB2_DCTRL            (N32_GTIMERB2_BASE + N32_GTIMB_DCTRL_OFFSET)
#define N32_GTIMB2_DADDR            (N32_GTIMERB2_BASE + N32_GTIMB_DADDR_OFFSET)

/* Register Addresses - GTIMB3 **********************************************/

#define N32_GTIMB3_CTRL1            (N32_GTIMERB3_BASE + N32_GTIMB_CTRL1_OFFSET)
#define N32_GTIMB3_CTRL2            (N32_GTIMERB3_BASE + N32_GTIMB_CTRL2_OFFSET)
#define N32_GTIMB3_STS              (N32_GTIMERB3_BASE + N32_GTIMB_STS_OFFSET)
#define N32_GTIMB3_ETGEN            (N32_GTIMERB3_BASE + N32_GTIMB_ETGEN_OFFSET)
#define N32_GTIMB3_SMCTRL           (N32_GTIMERB3_BASE + N32_GTIMB_SMCTRL_OFFSET)
#define N32_GTIMB3_DINTEN           (N32_GTIMERB3_BASE + N32_GTIMB_DINTEN_OFFSET)
#define N32_GTIMB3_CCMOD1           (N32_GTIMERB3_BASE + N32_GTIMB_CCMOD1_OFFSET)
#define N32_GTIMB3_CCMOD2           (N32_GTIMERB3_BASE + N32_GTIMB_CCMOD2_OFFSET)
#define N32_GTIMB3_CCMOD3           (N32_GTIMERB3_BASE + N32_GTIMB_CCMOD3_OFFSET)
#define N32_GTIMB3_CCEN             (N32_GTIMERB3_BASE + N32_GTIMB_CCEN_OFFSET)
#define N32_GTIMB3_CCDAT1           (N32_GTIMERB3_BASE + N32_GTIMB_CCDAT1_OFFSET)
#define N32_GTIMB3_CCDAT2           (N32_GTIMERB3_BASE + N32_GTIMB_CCDAT2_OFFSET)
#define N32_GTIMB3_CCDAT3           (N32_GTIMERB3_BASE + N32_GTIMB_CCDAT3_OFFSET)
#define N32_GTIMB3_CCDAT4           (N32_GTIMERB3_BASE + N32_GTIMB_CCDAT4_OFFSET)
#define N32_GTIMB3_CCDAT5           (N32_GTIMERB3_BASE + N32_GTIMB_CCDAT5_OFFSET)
#define N32_GTIMB3_PSC              (N32_GTIMERB3_BASE + N32_GTIMB_PSC_OFFSET)
#define N32_GTIMB3_AR               (N32_GTIMERB3_BASE + N32_GTIMB_AR_OFFSET)
#define N32_GTIMB3_CNT              (N32_GTIMERB3_BASE + N32_GTIMB_CNT_OFFSET)
#define N32_GTIMB3_REPCNT           (N32_GTIMERB3_BASE + N32_GTIMB_REPCNT_OFFSET)
#define N32_GTIMB3_BKDT             (N32_GTIMERB3_BASE + N32_GTIMB_BKDT_OFFSET)
#define N32_GTIMB3_BKFR             (N32_GTIMERB3_BASE + N32_GTIMB_BKFR_OFFSET)
#define N32_GTIMB3_C1FILT           (N32_GTIMERB3_BASE + N32_GTIMB_C1FILT_OFFSET)
#define N32_GTIMB3_C2FILT           (N32_GTIMERB3_BASE + N32_GTIMB_C2FILT_OFFSET)
#define N32_GTIMB3_C3FILT           (N32_GTIMERB3_BASE + N32_GTIMB_C3FILT_OFFSET)
#define N32_GTIMB3_C4FILT           (N32_GTIMERB3_BASE + N32_GTIMB_C4FILT_OFFSET)
#define N32_GTIMB3_FILTO            (N32_GTIMERB3_BASE + N32_GTIMB_FILTO_OFFSET)
#define N32_GTIMB3_INSEL            (N32_GTIMERB3_BASE + N32_GTIMB_INSEL_OFFSET)
#define N32_GTIMB3_AF1              (N32_GTIMERB3_BASE + N32_GTIMB_AF1_OFFSET)
#define N32_GTIMB3_DCTRL            (N32_GTIMERB3_BASE + N32_GTIMB_DCTRL_OFFSET)
#define N32_GTIMB3_DADDR            (N32_GTIMERB3_BASE + N32_GTIMB_DADDR_OFFSET)

/* Bitfield Definitions - GTIMB *********************************************/

/* Control register 1 (N32_GTIMB_CTRL1) */

#define N32_GTIMB_CTRL1_CNTEN              (1 << 0)  /* Counter enable */
#define N32_GTIMB_CTRL1_DIR                (1 << 1)  /* Direction */
#define N32_GTIMB_CTRL1_CAMSEL_SHIFT       (2)       /* Center-aligned mode selection */
#define N32_GTIMB_CTRL1_CAMSEL_MASK        (3 << N32_GTIMB_CTRL1_CAMSEL_SHIFT)
#  define N32_GTIMB_CTRL1_EDGE             (0 << N32_GTIMB_CTRL1_CAMSEL_SHIFT)
#  define N32_GTIMB_CTRL1_CENTER1          (1 << N32_GTIMB_CTRL1_CAMSEL_SHIFT)
#  define N32_GTIMB_CTRL1_CENTER2          (2 << N32_GTIMB_CTRL1_CAMSEL_SHIFT)
#  define N32_GTIMB_CTRL1_CENTER3          (3 << N32_GTIMB_CTRL1_CAMSEL_SHIFT)
#define N32_GTIMB_CTRL1_UPRS               (1 << 4)  /* Update request source */
#define N32_GTIMB_CTRL1_UPDIS              (1 << 5)  /* Update disable */
#define N32_GTIMB_CTRL1_CLKD_SHIFT         (6)       /* Clock division */
#define N32_GTIMB_CTRL1_CLKD_MASK          (3 << N32_GTIMB_CTRL1_CLKD_SHIFT)
#  define N32_GTIMB_CTRL1_TCKINT           (0 << N32_GTIMB_CTRL1_CLKD_SHIFT)
#  define N32_GTIMB_CTRL1_2TCKINT          (1 << N32_GTIMB_CTRL1_CLKD_SHIFT)
#  define N32_GTIMB_CTRL1_4TCKINT          (2 << N32_GTIMB_CTRL1_CLKD_SHIFT)
#define N32_GTIMB_CTRL1_ONEPM              (1 << 8)  /* One pulse mode */
#define N32_GTIMB_CTRL1_ARPEN              (1 << 9)  /* Auto-reload preload enable */
#define N32_GTIMB_CTRL1_LBKPEN             (1 << 10) /* LockUp as BRK enable */
#define N32_GTIMB_CTRL1_PBKPEN             (1 << 11) /* PVD as BRK enable */
#define N32_GTIMB_CTRL1_SRAMPARRERREN      (1 << 12) /* SRAM parity error as BRK enable */
#define N32_GTIMB_CTRL1_CLRSEL             (1 << 13) /* OcxRef clear selection */
#define N32_GTIMB_CTRL1_SRAMECERREN        (1 << 15) /* SRAM ECC error as BRK enable */
#define N32_GTIMB_CTRL1_ASYMMETRIC         (1 << 23) /* Asymmetric mode enable */
#define N32_GTIMB_CTRL1_UDITFREMAP         (1 << 24) /* UDITF status bit remapping */

/* Control register 2 (N32_GTIMB_CTRL2) */

#define N32_GTIMB_CTRL2_O11                (1 << 0)  /* Output idle state 1 (OC1) */
#define N32_GTIMB_CTRL2_O11N               (1 << 1)  /* Output idle state 1N (OC1N) */
#define N32_GTIMB_CTRL2_O12                (1 << 2)  /* Output idle state 2 (OC2) */
#define N32_GTIMB_CTRL2_O13                (1 << 4)  /* Output idle state 3 (OC3) */
#define N32_GTIMB_CTRL2_O14                (1 << 6)  /* Output idle state 4 (OC4) */
#define N32_GTIMB_CTRL2_O15                (1 << 8)  /* Output idle state 5 (OC5) */
#define N32_GTIMB_CTRL2_MMSEL_SHIFT        (12)      /* Master mode selection (TRGO) */
#define N32_GTIMB_CTRL2_MMSEL_MASK         (15 << N32_GTIMB_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMB_CTRL2_MMSEL_RESET      (0 << N32_GTIMB_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMB_CTRL2_MMSEL_ENABLE     (1 << N32_GTIMB_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMB_CTRL2_MMSEL_UPDATE     (2 << N32_GTIMB_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMB_CTRL2_MMSEL_COMPP      (3 << N32_GTIMB_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMB_CTRL2_MMSEL_OC1REF     (4 << N32_GTIMB_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMB_CTRL2_MMSEL_OC2REF     (5 << N32_GTIMB_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMB_CTRL2_MMSEL_OC3REF     (6 << N32_GTIMB_CTRL2_MMSEL_SHIFT)
#  define N32_GTIMB_CTRL2_MMSEL_OC4REF     (7 << N32_GTIMB_CTRL2_MMSEL_SHIFT)
#define N32_GTIMB_CTRL2_CCUSEL             (1 << 16) /* CC control update selection */
#define N32_GTIMB_CTRL2_CCDSEL             (1 << 17) /* CC DMA selection */
#define N32_GTIMB_CTRL2_CCPCTL             (1 << 18) /* CC preloaded control */
#define N32_GTIMB_CTRL2_TIISEL             (1 << 19) /* TI1 selection (XOR mode) */

/* Status register (N32_GTIMB_STS) */

#define N32_GTIMB_STS_CC1ITF              (1 << 0)  /* CC1 interrupt flag */
#define N32_GTIMB_STS_CC2ITF              (1 << 1)  /* CC2 interrupt flag */
#define N32_GTIMB_STS_CC3ITF              (1 << 2)  /* CC3 interrupt flag */
#define N32_GTIMB_STS_CC4ITF              (1 << 3)  /* CC4 interrupt flag */
#define N32_GTIMB_STS_CC5ITF              (1 << 4)  /* CC5 interrupt flag */
#define N32_GTIMB_STS_CC1OCF              (1 << 8)  /* CC1 overcapture flag */
#define N32_GTIMB_STS_CC2OCF              (1 << 9)  /* CC2 overcapture flag */
#define N32_GTIMB_STS_CC3OCF              (1 << 10) /* CC3 overcapture flag */
#define N32_GTIMB_STS_CC4OCF              (1 << 11) /* CC4 overcapture flag */
#define N32_GTIMB_STS_UDITF               (1 << 16) /* Update interrupt flag */
#define N32_GTIMB_STS_TITF                (1 << 18) /* Trigger interrupt flag */
#define N32_GTIMB_STS_BITF                (1 << 19) /* Break1 interrupt flag */
#define N32_GTIMB_STS_SBITF               (1 << 21) /* System break interrupt flag */

/* Event generation (N32_GTIMB_ETGEN) */

#define N32_GTIMB_ETGEN_CC1GN             (1 << 0)  /* CC1 generation */
#define N32_GTIMB_ETGEN_CC2GN             (1 << 1)  /* CC2 generation */
#define N32_GTIMB_ETGEN_CC3GN             (1 << 2)  /* CC3 generation */
#define N32_GTIMB_ETGEN_CC4GN             (1 << 3)  /* CC4 generation */
#define N32_GTIMB_ETGEN_UDGN              (1 << 8)  /* Update generation */
#define N32_GTIMB_ETGEN_CCUDGN            (1 << 9)  /* CC control update generation */
#define N32_GTIMB_ETGEN_TGN               (1 << 10) /* Trigger generation */
#define N32_GTIMB_ETGEN_BGN               (1 << 11) /* Break1 generation */

/* SMCTRL (same as ATIM/GTIMA) */

#define N32_GTIMB_SMCTRL_TSEL_SHIFT       (0)       /* Trigger selection */
#define N32_GTIMB_SMCTRL_TSEL_MASK        (7 << N32_GTIMB_SMCTRL_TSEL_SHIFT)
#define N32_GTIMB_SMCTRL_SMSEL_SHIFT      (4)       /* Slave mode selection */
#define N32_GTIMB_SMCTRL_SMSEL_MASK       (15 << N32_GTIMB_SMCTRL_SMSEL_SHIFT)
#define N32_GTIMB_SMCTRL_EXTPS_SHIFT      (8)       /* External trigger prescaler */
#define N32_GTIMB_SMCTRL_EXTPS_MASK       (3 << N32_GTIMB_SMCTRL_EXTPS_SHIFT)
#define N32_GTIMB_SMCTRL_EXCEN            (1 << 10) /* External clock enable */
#define N32_GTIMB_SMCTRL_EXTP             (1 << 11) /* External trigger polarity */
#define N32_GTIMB_SMCTRL_EXTF_SHIFT       (12)      /* External trigger filter */
#define N32_GTIMB_SMCTRL_EXTF_MASK        (15 << N32_GTIMB_SMCTRL_EXTF_SHIFT)
#define N32_GTIMB_SMCTRL_MSMD             (1 << 16) /* Master/slave mode */
#define N32_GTIMB_SMCTRL_OCRECFLRP        (1 << 19) /* tim_ocref_clr polarity */
#define N32_GTIMB_SMCTRL_OCRECFLRF_SHIFT  (20)      /* tim_ocref_clr filter */
#define N32_GTIMB_SMCTRL_OCRECFLRF_MASK   (15 << N32_GTIMB_SMCTRL_OCRECFLRF_SHIFT)

/* DINTEN (N32_GTIMB_DINTEN) */

#define N32_GTIMB_DINTEN_CC1IEN           (1 << 0)  /* CC1 interrupt enable */
#define N32_GTIMB_DINTEN_CC2IEN           (1 << 1)  /* CC2 interrupt enable */
#define N32_GTIMB_DINTEN_CC3IEN           (1 << 2)  /* CC3 interrupt enable */
#define N32_GTIMB_DINTEN_CC4IEN           (1 << 3)  /* CC4 interrupt enable */
#define N32_GTIMB_DINTEN_CC1DEN           (1 << 8)  /* CC1 DMA request enable */
#define N32_GTIMB_DINTEN_CC2DEN           (1 << 9)  /* CC2 DMA request enable */
#define N32_GTIMB_DINTEN_CC3DEN           (1 << 10) /* CC3 DMA request enable */
#define N32_GTIMB_DINTEN_CC4DEN           (1 << 11) /* CC4 DMA request enable */
#define N32_GTIMB_DINTEN_UIEN             (1 << 16) /* Update interrupt enable */
#define N32_GTIMB_DINTEN_TIEN             (1 << 17) /* Trigger interrupt enable */
#define N32_GTIMB_DINTEN_BIEN             (1 << 18) /* Break interrupt enable */
#define N32_GTIMB_DINTEN_UDEN             (1 << 19) /* Update DMA request enable */
#define N32_GTIMB_DINTEN_COMDEN           (1 << 20) /* COM DMA request enable */
#define N32_GTIMB_DINTEN_TDEN             (1 << 21) /* Trigger DMA request enable */
#define N32_GTIMB_DINTEN_COMIEN           (1 << 22) /* COM interrupt enable */

/* CCMOD1 - Output compare (N32_GTIMB_CCMOD1) - 4-bit OC mode */

#define N32_GTIMB_CCMOD1_CC1SEL_SHIFT     (0)       /* CC1 selection */
#define N32_GTIMB_CCMOD1_CC1SEL_MASK      (3 << N32_GTIMB_CCMOD1_CC1SEL_SHIFT)
#  define N32_GTIMB_CCMOD1_CC1OUT         (0 << N32_GTIMB_CCMOD1_CC1SEL_SHIFT)
#  define N32_GTIMB_CCMOD1_CC1IN_TI1      (1 << N32_GTIMB_CCMOD1_CC1SEL_SHIFT)
#  define N32_GTIMB_CCMOD1_CC1IN_TI2      (2 << N32_GTIMB_CCMOD1_CC1SEL_SHIFT)
#  define N32_GTIMB_CCMOD1_CC1IN_TRC      (3 << N32_GTIMB_CCMOD1_CC1SEL_SHIFT)
#define N32_GTIMB_CCMOD1_OC1PEN           (1 << 2)  /* OC1 preload enable */
#define N32_GTIMB_CCMOD1_OC1FEN           (1 << 3)  /* OC1 fast enable */
#define N32_GTIMB_CCMOD1_OC1CEN           (1 << 4)  /* OC1 clear enable */
#define N32_GTIMB_CCMOD1_OC1MD_SHIFT      (5)       /* OC1 mode (bits 0-2) */
#define N32_GTIMB_CCMOD1_OC1MD_MASK       (7 << N32_GTIMB_CCMOD1_OC1MD_SHIFT)
#define N32_GTIMB_CCMOD1_OC1MD3           (1 << 17) /* OC1 mode bit 3 */
#define N32_GTIMB_CCMOD1_CC2SEL_SHIFT     (8)       /* CC2 selection */
#define N32_GTIMB_CCMOD1_CC2SEL_MASK      (3 << N32_GTIMB_CCMOD1_CC2SEL_SHIFT)
#  define N32_GTIMB_CCMOD1_CC2OUT         (0 << N32_GTIMB_CCMOD1_CC2SEL_SHIFT)
#  define N32_GTIMB_CCMOD1_CC2IN_TI2      (1 << N32_GTIMB_CCMOD1_CC2SEL_SHIFT)
#  define N32_GTIMB_CCMOD1_CC2IN_TI1      (2 << N32_GTIMB_CCMOD1_CC2SEL_SHIFT)
#  define N32_GTIMB_CCMOD1_CC2IN_TRC      (3 << N32_GTIMB_CCMOD1_CC2SEL_SHIFT)
#define N32_GTIMB_CCMOD1_OC2PEN           (1 << 10) /* OC2 preload enable */
#define N32_GTIMB_CCMOD1_OC2FEN           (1 << 11) /* OC2 fast enable */
#define N32_GTIMB_CCMOD1_OC2CEN           (1 << 12) /* OC2 clear enable */
#define N32_GTIMB_CCMOD1_OC2MD_SHIFT      (13)      /* OC2 mode (bits 0-2) */
#define N32_GTIMB_CCMOD1_OC2MD_MASK       (7 << N32_GTIMB_CCMOD1_OC2MD_SHIFT)
#define N32_GTIMB_CCMOD1_OC2MD3           (1 << 18) /* OC2 mode bit 3 */

/* 4-bit OC mode values (OCxMD3 + OCxMD[2:0]) */
#define N32_GTIMB_OCMODE_FRZN             (0)       /* Frozen */
#define N32_GTIMB_OCMODE_ACTIVE           (1)       /* Active */
#define N32_GTIMB_OCMODE_INACTIVE         (2)       /* Inactive */
#define N32_GTIMB_OCMODE_TOGGLE           (3)       /* Toggle */
#define N32_GTIMB_OCMODE_FORCELO          (4)       /* Force low */
#define N32_GTIMB_OCMODE_FORCEHI          (5)       /* Force high */
#define N32_GTIMB_OCMODE_PWM1             (6)       /* PWM mode 1 */
#define N32_GTIMB_OCMODE_PWM2             (7)       /* PWM mode 2 */
#define N32_GTIMB_OCMODE_RETRIG1          (8)       /* Retriggerable OPM 1 */
#define N32_GTIMB_OCMODE_RETRIG2          (9)       /* Retriggerable OPM 2 */
#define N32_GTIMB_OCMODE_COMBINED1        (14)      /* Combined PWM 1 */
#define N32_GTIMB_OCMODE_COMBINED2        (15)      /* Combined PWM 2 */

/* CCMOD1 - Input capture (N32_GTIMB_CCMOD1) */

#define N32_GTIMB_CCMOD1_IC1PSC_SHIFT     (2)       /* IC1 prescaler */
#define N32_GTIMB_CCMOD1_IC1PSC_MASK      (3 << N32_GTIMB_CCMOD1_IC1PSC_SHIFT)
#define N32_GTIMB_CCMOD1_IC1F_SHIFT       (4)       /* IC1 filter */
#define N32_GTIMB_CCMOD1_IC1F_MASK        (15 << N32_GTIMB_CCMOD1_IC1F_SHIFT)
#define N32_GTIMB_CCMOD1_IC2PSC_SHIFT     (10)      /* IC2 prescaler */
#define N32_GTIMB_CCMOD1_IC2PSC_MASK      (3 << N32_GTIMB_CCMOD1_IC2PSC_SHIFT)
#define N32_GTIMB_CCMOD1_IC2F_SHIFT       (12)      /* IC2 filter */
#define N32_GTIMB_CCMOD1_IC2F_MASK        (15 << N32_GTIMB_CCMOD1_IC2F_SHIFT)

/* CCMOD2 - Output compare (N32_GTIMB_CCMOD2) */

#define N32_GTIMB_CCMOD2_CC3SEL_SHIFT     (0)       /* CC3 selection */
#define N32_GTIMB_CCMOD2_CC3SEL_MASK      (3 << N32_GTIMB_CCMOD2_CC3SEL_SHIFT)
#  define N32_GTIMB_CCMOD2_CC3OUT         (0 << N32_GTIMB_CCMOD2_CC3SEL_SHIFT)
#  define N32_GTIMB_CCMOD2_CC3IN_TI3      (1 << N32_GTIMB_CCMOD2_CC3SEL_SHIFT)
#  define N32_GTIMB_CCMOD2_CC3IN_TI4      (2 << N32_GTIMB_CCMOD2_CC3SEL_SHIFT)
#  define N32_GTIMB_CCMOD2_CC3IN_TRC      (3 << N32_GTIMB_CCMOD2_CC3SEL_SHIFT)
#define N32_GTIMB_CCMOD2_OC3PEN           (1 << 2)  /* OC3 preload enable */
#define N32_GTIMB_CCMOD2_OC3FEN           (1 << 3)  /* OC3 fast enable */
#define N32_GTIMB_CCMOD2_OC3CEN           (1 << 4)  /* OC3 clear enable */
#define N32_GTIMB_CCMOD2_OC3MD_SHIFT      (5)       /* OC3 mode (bits 0-2) */
#define N32_GTIMB_CCMOD2_OC3MD_MASK       (7 << N32_GTIMB_CCMOD2_OC3MD_SHIFT)
#define N32_GTIMB_CCMOD2_OC3MD3           (1 << 17) /* OC3 mode bit 3 */
#define N32_GTIMB_CCMOD2_CC4SEL_SHIFT     (8)       /* CC4 selection */
#define N32_GTIMB_CCMOD2_CC4SEL_MASK      (3 << N32_GTIMB_CCMOD2_CC4SEL_SHIFT)
#  define N32_GTIMB_CCMOD2_CC4OUT         (0 << N32_GTIMB_CCMOD2_CC4SEL_SHIFT)
#  define N32_GTIMB_CCMOD2_CC4IN_TI4      (1 << N32_GTIMB_CCMOD2_CC4SEL_SHIFT)
#  define N32_GTIMB_CCMOD2_CC4IN_TI3      (2 << N32_GTIMB_CCMOD2_CC4SEL_SHIFT)
#  define N32_GTIMB_CCMOD2_CC4IN_TRC      (3 << N32_GTIMB_CCMOD2_CC4SEL_SHIFT)
#define N32_GTIMB_CCMOD2_OC4PEN           (1 << 10) /* OC4 preload enable */
#define N32_GTIMB_CCMOD2_OC4FEN           (1 << 11) /* OC4 fast enable */
#define N32_GTIMB_CCMOD2_OC4CEN           (1 << 12) /* OC4 clear enable */
#define N32_GTIMB_CCMOD2_OC4MD_SHIFT      (13)      /* OC4 mode (bits 0-2) */
#define N32_GTIMB_CCMOD2_OC4MD_MASK       (7 << N32_GTIMB_CCMOD2_OC4MD_SHIFT)
#define N32_GTIMB_CCMOD2_OC4MD3           (1 << 18) /* OC4 mode bit 3 */

/* CCMOD2 - Input capture (N32_GTIMB_CCMOD2) */

#define N32_GTIMB_CCMOD2_IC3PSC_SHIFT     (2)       /* IC3 prescaler */
#define N32_GTIMB_CCMOD2_IC3PSC_MASK      (3 << N32_GTIMB_CCMOD2_IC3PSC_SHIFT)
#define N32_GTIMB_CCMOD2_IC3F_SHIFT       (4)       /* IC3 filter */
#define N32_GTIMB_CCMOD2_IC3F_MASK        (15 << N32_GTIMB_CCMOD2_IC3F_SHIFT)
#define N32_GTIMB_CCMOD2_IC4PSC_SHIFT     (10)      /* IC4 prescaler */
#define N32_GTIMB_CCMOD2_IC4PSC_MASK      (3 << N32_GTIMB_CCMOD2_IC4PSC_SHIFT)
#define N32_GTIMB_CCMOD2_IC4F_SHIFT       (12)      /* IC4 filter */
#define N32_GTIMB_CCMOD2_IC4F_MASK        (15 << N32_GTIMB_CCMOD2_IC4F_SHIFT)

/* CCMOD3 (N32_GTIMB_CCMOD3) - CC5 only */

#define N32_GTIMB_CCMOD3_OC5PEN           (1 << 2)  /* OC5 preload enable */
#define N32_GTIMB_CCMOD3_OC5FEN           (1 << 3)  /* OC5 fast enable */
#define N32_GTIMB_CCMOD3_OC5CEN           (1 << 4)  /* OC5 clear enable */
#define N32_GTIMB_CCMOD3_OC5MD_SHIFT      (5)       /* OC5 mode */
#define N32_GTIMB_CCMOD3_OC5MD_MASK       (7 << N32_GTIMB_CCMOD3_OC5MD_SHIFT)

/* CCEN (N32_GTIMB_CCEN) - With complementary outputs */

#define N32_GTIMB_CCEN_CC1NEN            (1 << 0)  /* CC1 complementary output enable */
#define N32_GTIMB_CCEN_CC1NP             (1 << 1)  /* CC1 complementary output polarity */
#define N32_GTIMB_CCEN_CC1EN             (1 << 2)  /* CC1 output enable */
#define N32_GTIMB_CCEN_CC1P              (1 << 3)  /* CC1 output polarity */
#define N32_GTIMB_CCEN_CC2EN             (1 << 6)  /* CC2 output enable */
#define N32_GTIMB_CCEN_CC2P              (1 << 7)  /* CC2 output polarity */
#define N32_GTIMB_CCEN_CC3EN             (1 << 10) /* CC3 output enable */
#define N32_GTIMB_CCEN_CC3P              (1 << 11) /* CC3 output polarity */
#define N32_GTIMB_CCEN_CC4EN             (1 << 14) /* CC4 output enable */
#define N32_GTIMB_CCEN_CC4P              (1 << 15) /* CC4 output polarity */
#define N32_GTIMB_CCEN_CC5EN             (1 << 18) /* CC5 output enable */
#define N32_GTIMB_CCEN_CC5P              (1 << 19) /* CC5 output polarity */

/* CCDAT (with down-count value in high 16 bits) */

#define N32_GTIMB_CCDAT_MASK             (0xffff)  /* CCx value mask */
#define N32_GTIMB_CCDAT_DOWN_SHIFT       (16)      /* Down-count value shift */

/* PSC, AR, CNT */

#define N32_GTIMB_PSC_MASK               (0xffff)  /* Prescaler value mask */
#define N32_GTIMB_AR_MASK                (0xffff)  /* Auto-reload value mask */
#define N32_GTIMB_CNT_MASK               (0xffff)  /* Counter value mask */
#define N32_GTIMB_CNT_UDITFCPY           (1 << 31) /* UDITF copy */

/* REPCNT */

#define N32_GTIMB_REPCNT_MASK            (0xff)    /* Repetition counter value mask */

/* BKDT (N32_GTIMB_BKDT) - Break1 only (no Break2) */

#define N32_GTIMB_BKDT_DTGN_SHIFT        (0)       /* Dead-time generator setup */
#define N32_GTIMB_BKDT_DTGN_MASK         (0xff << N32_GTIMB_BKDT_DTGN_SHIFT)
#define N32_GTIMB_BKDT_MOEN              (1 << 8)  /* Main output enable */
#define N32_GTIMB_BKDT_AOEN              (1 << 9)  /* Automatic output enable */
#define N32_GTIMB_BKDT_BKP               (1 << 10) /* Break1 polarity */
#define N32_GTIMB_BKDT_BKEN              (1 << 11) /* Break1 enable */
#define N32_GTIMB_BKDT_OSSI              (1 << 12) /* Off-state selection for Idle mode */
#define N32_GTIMB_BKDT_OSSR              (1 << 13) /* Off-state selection for Run mode */
#define N32_GTIMB_BKDT_LCKCFG_SHIFT      (14)      /* Lock configuration */
#define N32_GTIMB_BKDT_LCKCFG_MASK       (3 << N32_GTIMB_BKDT_LCKCFG_SHIFT)
#  define N32_GTIMB_BKDT_LOCKOFF         (0 << N32_GTIMB_BKDT_LCKCFG_SHIFT)
#  define N32_GTIMB_BKDT_LOCK1           (1 << N32_GTIMB_BKDT_LCKCFG_SHIFT)
#  define N32_GTIMB_BKDT_LOCK2           (2 << N32_GTIMB_BKDT_LCKCFG_SHIFT)
#  define N32_GTIMB_BKDT_LOCK3           (3 << N32_GTIMB_BKDT_LCKCFG_SHIFT)
#define N32_GTIMB_BKDT_BRKDSRM           (1 << 18) /* Break1 disarm */
#define N32_GTIMB_BKDT_BRKBID            (1 << 20) /* Break1 bidirectional enable */

/* BKFR (same as ATIM) */

#define N32_GTIMB_BKFR_SLIDFPSC_MASK     (0xffff)  /* Sampling clock prescaler */
#define N32_GTIMB_BKFR_FILTEN            (1 << 16) /* Filter enable */
#define N32_GTIMB_BKFR_WSIZE_SHIFT       (17)      /* Window size */
#define N32_GTIMB_BKFR_WSIZE_MASK        (63 << N32_GTIMB_BKFR_WSIZE_SHIFT)
#define N32_GTIMB_BKFR_THRESH_SHIFT      (24)      /* Threshold */
#define N32_GTIMB_BKFR_THRESH_MASK       (63 << N32_GTIMB_BKFR_THRESH_SHIFT)

/* Filter registers (C1FILT..C4FILT) */

#define N32_GTIMB_CXFILT_SLIDFPSC_MASK   (0xffff)  /* Sampling clock prescaler */
#define N32_GTIMB_CXFILT_FILTEN          (1 << 16) /* Filter enable */
#define N32_GTIMB_CXFILT_WSIZE_SHIFT     (17)      /* Window size */
#define N32_GTIMB_CXFILT_WSIZE_MASK      (63 << N32_GTIMB_CXFILT_WSIZE_SHIFT)
#define N32_GTIMB_CXFILT_THRESH_SHIFT    (24)      /* Threshold */
#define N32_GTIMB_CXFILT_THRESH_MASK     (63 << N32_GTIMB_CXFILT_THRESH_SHIFT)

/* FILTO */

#define N32_GTIMB_FILTO_C1FILTO          (1 << 0)  /* Channel 1 filter output */
#define N32_GTIMB_FILTO_C2FILTO          (1 << 1)  /* Channel 2 filter output */
#define N32_GTIMB_FILTO_C3FILTO          (1 << 2)  /* Channel 3 filter output */
#define N32_GTIMB_FILTO_C4FILTO          (1 << 3)  /* Channel 4 filter output */

/* INSEL (same as ATIM/GTIMA) */

#define N32_GTIMB_INSEL_TI1S_SHIFT       (0)       /* TI1 input selection */
#define N32_GTIMB_INSEL_TI1S_MASK        (15 << N32_GTIMB_INSEL_TI1S_SHIFT)
#define N32_GTIMB_INSEL_TI2S_SHIFT       (4)       /* TI2 input selection */
#define N32_GTIMB_INSEL_TI2S_MASK        (15 << N32_GTIMB_INSEL_TI2S_SHIFT)
#define N32_GTIMB_INSEL_TI3S_SHIFT       (8)       /* TI3 input selection */
#define N32_GTIMB_INSEL_TI3S_MASK        (15 << N32_GTIMB_INSEL_TI3S_SHIFT)
#define N32_GTIMB_INSEL_TI4S_SHIFT       (12)      /* TI4 input selection */
#define N32_GTIMB_INSEL_TI4S_MASK        (15 << N32_GTIMB_INSEL_TI4S_SHIFT)
#define N32_GTIMB_INSEL_ETRS_SHIFT       (16)      /* ETR input selection */
#define N32_GTIMB_INSEL_ETRS_MASK        (15 << N32_GTIMB_INSEL_ETRS_SHIFT)
#define N32_GTIMB_INSEL_ITRS_SHIFT       (20)      /* ITR input selection */
#define N32_GTIMB_INSEL_ITRS_MASK        (15 << N32_GTIMB_INSEL_ITRS_SHIFT)
#define N32_GTIMB_INSEL_CLRS_SHIFT       (24)      /* OcxRef clear input selection */
#define N32_GTIMB_INSEL_CLRS_MASK        (15 << N32_GTIMB_INSEL_CLRS_SHIFT)

/* AF1 (N32_GTIMB_AF1) - Break input sources */

#define N32_GTIMB_AF1_IOMBRKEN           (1 << 0)  /* TIMx_BKIN break input enable */
#define N32_GTIMB_AF1_COMP1BRKEN         (1 << 1)  /* COMP1 break input enable */
#define N32_GTIMB_AF1_COMP2BRKEN         (1 << 2)  /* COMP2 break input enable */
#define N32_GTIMB_AF1_COMP3BRKEN         (1 << 3)  /* COMP3 break input enable */
#define N32_GTIMB_AF1_COMP4BRKEN         (1 << 4)  /* COMP4 break input enable */
#define N32_GTIMB_AF1_DSMU0BRKEN         (1 << 24) /* DSMU0 break enable */
#define N32_GTIMB_AF1_DSMU1BRKEN         (1 << 25) /* DSMU1 break enable */
#define N32_GTIMB_AF1_DSMU2BRKEN         (1 << 26) /* DSMU2 break enable */
#define N32_GTIMB_AF1_DSMU3BRKEN         (1 << 27) /* DSMU3 break enable */
#define N32_GTIMB_AF1_IOMBRKP            (1 << 9)  /* TIMx_BKIN polarity selection */
#define N32_GTIMB_AF1_COMP1BRKP          (1 << 10) /* COMP1 break polarity */
#define N32_GTIMB_AF1_COMP2BRKP          (1 << 11) /* COMP2 break polarity */
#define N32_GTIMB_AF1_COMP3BRKP          (1 << 12) /* COMP3 break polarity */
#define N32_GTIMB_AF1_COMP4BRKP          (1 << 13) /* COMP4 break polarity */

/* DCTRL/DADDR */

#define N32_GTIMB_DCTRL_DBLEN_SHIFT      (0)       /* DMA burst length */
#define N32_GTIMB_DCTRL_DBLEN_MASK       (63 << N32_GTIMB_DCTRL_DBLEN_SHIFT)
#  define N32_GTIMB_DCTRL_DBLEN(n)       (((n)-1) << N32_GTIMB_DCTRL_DBLEN_SHIFT)
#define N32_GTIMB_DCTRL_DBADDR_SHIFT     (8)       /* DMA base address */
#define N32_GTIMB_DCTRL_DBADDR_MASK      (63 << N32_GTIMB_DCTRL_DBADDR_SHIFT)
#define N32_GTIMB_DADDR_BURST_MASK       (0xffffffff)

/* ==========================================================================
 *                          BASIC TIMERS (BTIM1-4)
 * ==========================================================================
 */

/* Register Offsets - BTIM (common for all BTIM1-4) *************************/

#define N32_BTIM_CTRL1_OFFSET      0x0000  /* Control register 1 */
#define N32_BTIM_CTRL2_OFFSET      0x0004  /* Control register 2 */
#define N32_BTIM_STS_OFFSET        0x0008  /* Status register */
#define N32_BTIM_ETGEN_OFFSET      0x000c  /* Event generation register */
#define N32_BTIM_DINTEN_OFFSET     0x0014  /* DMA/Interrupt enable register */
#define N32_BTIM_PSC_OFFSET        0x0040  /* Prescaler */
#define N32_BTIM_AR_OFFSET         0x0044  /* Auto-reload register (32-bit) */
#define N32_BTIM_CNT_OFFSET        0x0048  /* Counter (32-bit) */

/* Register Addresses - BTIM1 ***********************************************/

#define N32_BTIM1_CTRL1            (N32_BTIMER1_BASE + N32_BTIM_CTRL1_OFFSET)
#define N32_BTIM1_CTRL2            (N32_BTIMER1_BASE + N32_BTIM_CTRL2_OFFSET)
#define N32_BTIM1_STS              (N32_BTIMER1_BASE + N32_BTIM_STS_OFFSET)
#define N32_BTIM1_ETGEN            (N32_BTIMER1_BASE + N32_BTIM_ETGEN_OFFSET)
#define N32_BTIM1_DINTEN           (N32_BTIMER1_BASE + N32_BTIM_DINTEN_OFFSET)
#define N32_BTIM1_PSC              (N32_BTIMER1_BASE + N32_BTIM_PSC_OFFSET)
#define N32_BTIM1_AR               (N32_BTIMER1_BASE + N32_BTIM_AR_OFFSET)
#define N32_BTIM1_CNT              (N32_BTIMER1_BASE + N32_BTIM_CNT_OFFSET)

/* Register Addresses - BTIM2 ***********************************************/

#define N32_BTIM2_CTRL1            (N32_BTIMER2_BASE + N32_BTIM_CTRL1_OFFSET)
#define N32_BTIM2_CTRL2            (N32_BTIMER2_BASE + N32_BTIM_CTRL2_OFFSET)
#define N32_BTIM2_STS              (N32_BTIMER2_BASE + N32_BTIM_STS_OFFSET)
#define N32_BTIM2_ETGEN            (N32_BTIMER2_BASE + N32_BTIM_ETGEN_OFFSET)
#define N32_BTIM2_DINTEN           (N32_BTIMER2_BASE + N32_BTIM_DINTEN_OFFSET)
#define N32_BTIM2_PSC              (N32_BTIMER2_BASE + N32_BTIM_PSC_OFFSET)
#define N32_BTIM2_AR               (N32_BTIMER2_BASE + N32_BTIM_AR_OFFSET)
#define N32_BTIM2_CNT              (N32_BTIMER2_BASE + N32_BTIM_CNT_OFFSET)

/* Register Addresses - BTIM3 ***********************************************/

#define N32_BTIM3_CTRL1            (N32_BTIMER3_BASE + N32_BTIM_CTRL1_OFFSET)
#define N32_BTIM3_CTRL2            (N32_BTIMER3_BASE + N32_BTIM_CTRL2_OFFSET)
#define N32_BTIM3_STS              (N32_BTIMER3_BASE + N32_BTIM_STS_OFFSET)
#define N32_BTIM3_ETGEN            (N32_BTIMER3_BASE + N32_BTIM_ETGEN_OFFSET)
#define N32_BTIM3_DINTEN           (N32_BTIMER3_BASE + N32_BTIM_DINTEN_OFFSET)
#define N32_BTIM3_PSC              (N32_BTIMER3_BASE + N32_BTIM_PSC_OFFSET)
#define N32_BTIM3_AR               (N32_BTIMER3_BASE + N32_BTIM_AR_OFFSET)
#define N32_BTIM3_CNT              (N32_BTIMER3_BASE + N32_BTIM_CNT_OFFSET)

/* Register Addresses - BTIM4 ***********************************************/

#define N32_BTIM4_CTRL1            (N32_BTIMER4_BASE + N32_BTIM_CTRL1_OFFSET)
#define N32_BTIM4_CTRL2            (N32_BTIMER4_BASE + N32_BTIM_CTRL2_OFFSET)
#define N32_BTIM4_STS              (N32_BTIMER4_BASE + N32_BTIM_STS_OFFSET)
#define N32_BTIM4_ETGEN            (N32_BTIMER4_BASE + N32_BTIM_ETGEN_OFFSET)
#define N32_BTIM4_DINTEN           (N32_BTIMER4_BASE + N32_BTIM_DINTEN_OFFSET)
#define N32_BTIM4_PSC              (N32_BTIMER4_BASE + N32_BTIM_PSC_OFFSET)
#define N32_BTIM4_AR               (N32_BTIMER4_BASE + N32_BTIM_AR_OFFSET)
#define N32_BTIM4_CNT              (N32_BTIMER4_BASE + N32_BTIM_CNT_OFFSET)

/* Bitfield Definitions - BTIM **********************************************/

/* Control register 1 (N32_BTIM_CTRL1) */

#define N32_BTIM_CTRL1_CNTEN              (1 << 0)  /* Counter enable */
#define N32_BTIM_CTRL1_UPRS               (1 << 4)  /* Update request source */
#define N32_BTIM_CTRL1_UPDIS              (1 << 5)  /* Update disable */
#define N32_BTIM_CTRL1_ONEPM              (1 << 8)  /* One pulse mode */
#define N32_BTIM_CTRL1_ARPEN              (1 << 9)  /* Auto-reload preload enable */

/* Control register 2 (N32_BTIM_CTRL2) */

#define N32_BTIM_CTRL2_MMSEL_SHIFT        (12)      /* Master mode selection (TRGO) */
#define N32_BTIM_CTRL2_MMSEL_MASK         (15 << N32_BTIM_CTRL2_MMSEL_SHIFT)
#  define N32_BTIM_CTRL2_MMSEL_RESET      (0 << N32_BTIM_CTRL2_MMSEL_SHIFT) /* Reset */
#  define N32_BTIM_CTRL2_MMSEL_ENABLE     (1 << N32_BTIM_CTRL2_MMSEL_SHIFT) /* Enable */
#  define N32_BTIM_CTRL2_MMSEL_UPDATE     (2 << N32_BTIM_CTRL2_MMSEL_SHIFT) /* Update */

/* Status register (N32_BTIM_STS) */

#define N32_BTIM_STS_UDITF               (1 << 16) /* Update interrupt flag */

/* Event generation register (N32_BTIM_ETGEN) */

#define N32_BTIM_ETGEN_UDGN              (1 << 8)  /* Update generation */

/* DMA/Interrupt enable register (N32_BTIM_DINTEN) */

#define N32_BTIM_DINTEN_UIEN             (1 << 16) /* Update interrupt enable */
#define N32_BTIM_DINTEN_UDEN             (1 << 19) /* Update DMA request enable */

/* Prescaler (N32_BTIM_PSC) */

#define N32_BTIM_PSC_MASK                (0xffff)  /* Prescaler value mask */

/* Auto-reload register (N32_BTIM_AR) - 32-bit */

#define N32_BTIM_AR_MASK                 (0xffffffff) /* Auto-reload value mask */

/* Counter (N32_BTIM_CNT) - 32-bit */

#define N32_BTIM_CNT_MASK                (0xffffffff) /* Counter value mask */

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32_TIM_H */
