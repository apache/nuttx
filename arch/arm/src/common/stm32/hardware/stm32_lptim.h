/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_lptim.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_LPTIM_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_LPTIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if (defined(CONFIG_STM32_HAVE_IP_LPTIM_V1) + \
     defined(CONFIG_STM32_HAVE_IP_LPTIM_V2)) != 1
#  error Exactly one STM32 LPTIM IP version must be selected
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LPTIM IP versions (from the reference SVD files):
 *
 *   V1 - IER and CMP, optional OR, CFGR2 and RCR
 *        (F4, F7, G0, G4, H7, L0, L4, L5, WB, WL5)
 *   V2 - DIER, capture/compare channels, CFGR2 and RCR
 *        (C5, H5, H7RS, N6, U0, U3, U5)
 */

/* Register Offsets *********************************************************/

#define STM32_LPTIM_ISR_OFFSET      0x0000    /* Interrupt and status */
#define STM32_LPTIM_ICR_OFFSET      0x0004    /* Interrupt clear */
#define STM32_LPTIM_IER_OFFSET      0x0008    /* Interrupt enable */
#define STM32_LPTIM_CFGR_OFFSET     0x000c    /* Configuration */
#define STM32_LPTIM_CR_OFFSET       0x0010    /* Control */
#define STM32_LPTIM_CMP_OFFSET      0x0014    /* Compare */
#define STM32_LPTIM_ARR_OFFSET      0x0018    /* Autoreload */
#define STM32_LPTIM_CNT_OFFSET      0x001c    /* Counter */
#ifdef CONFIG_STM32_HAVE_IP_LPTIM_V1
#  define STM32_LPTIM_OR_OFFSET     0x0020    /* Option */
#endif
#define STM32_LPTIM_CFGR2_OFFSET    0x0024    /* Configuration 2 */
#define STM32_LPTIM_RCR_OFFSET      0x0028    /* Repetition */
#ifdef CONFIG_STM32_HAVE_IP_LPTIM_V2
#  define STM32_LPTIM_DIER_OFFSET   0x0008    /* Interrupt enable */
#  define STM32_LPTIM_CCR1_OFFSET   0x0014    /* Capture/compare 1 */
#  define STM32_LPTIM_CCMR1_OFFSET  0x002c    /* Capture/compare mode 1 */
#  define STM32_LPTIM_CCMR2_OFFSET  0x0030    /* Capture/compare mode 2 */
#  define STM32_LPTIM_CCR2_OFFSET   0x0034    /* Capture/compare 2 */
#  define STM32_LPTIM_CCR3_OFFSET   0x0038    /* Capture/compare 3 */
#  define STM32_LPTIM_CCR4_OFFSET   0x003c    /* Capture/compare 4 */
#endif

/* Register Bitfield Definitions ********************************************/

/* Interrupt flags for ISR, ICR and IER */

#define LPTIM_ISR_CMPM              (1 << 0)  /* Bit 0: Compare match */
#define LPTIM_ISR_ARRM              (1 << 1)  /* Bit 1: Autoreload match */
#define LPTIM_ISR_EXTTRIG           (1 << 2)  /* Bit 2: External trigger */
#define LPTIM_ISR_CMPOK             (1 << 3)  /* Bit 3: CMP update OK */
#define LPTIM_ISR_ARROK             (1 << 4)  /* Bit 4: ARR update OK */
#define LPTIM_ISR_UP                (1 << 5)  /* Bit 5: Down to up change */
#define LPTIM_ISR_DOWN              (1 << 6)  /* Bit 6: Up to down change */
#define LPTIM_ISR_UE                (1 << 7)  /* Bit 7: Update event */
#define LPTIM_ISR_REPOK             (1 << 8)  /* Bit 8: RCR update OK */
#ifdef CONFIG_STM32_HAVE_IP_LPTIM_V2
#  define LPTIM_ISR_CC2IF           (1 << 9)  /* Bit 9: Capture/compare 2 */
#  define LPTIM_ISR_CC3IF           (1 << 10) /* Bit 10: Capture/compare 3 */
#  define LPTIM_ISR_CC4IF           (1 << 11) /* Bit 11: Capture/compare 4 */
#  define LPTIM_ISR_CC1OF           (1 << 12) /* Bit 12: CC1 overcapture */
#  define LPTIM_ISR_CC2OF           (1 << 13) /* Bit 13: CC2 overcapture */
#  define LPTIM_ISR_CC3OF           (1 << 14) /* Bit 14: CC3 overcapture */
#  define LPTIM_ISR_CC4OF           (1 << 15) /* Bit 15: CC4 overcapture */
#  define LPTIM_ISR_CMP2OK          (1 << 19) /* Bit 19: CCR2 update OK */
#  define LPTIM_ISR_CMP3OK          (1 << 20) /* Bit 20: CCR3 update OK */
#  define LPTIM_ISR_CMP4OK          (1 << 21) /* Bit 21: CCR4 update OK */
#  define LPTIM_ISR_DIEROK          (1 << 24) /* Bit 24: DIER update OK */

#  define LPTIM_IER_CC1DE           (1 << 16) /* Bit 16: CC1 DMA request */
#  define LPTIM_IER_UEDE            (1 << 23) /* Bit 23: Update DMA request */
#  define LPTIM_IER_CC2DE           (1 << 25) /* Bit 25: CC2 DMA request */
#  define LPTIM_IER_CC3DE           (1 << 26) /* Bit 26: CC3 DMA request */
#  define LPTIM_IER_CC4DE           (1 << 27) /* Bit 27: CC4 DMA request */
#endif

/* Configuration register */

#define LPTIM_CFGR_CKSEL            (1 << 0)  /* Bit 0: Clock selector */
#define LPTIM_CFGR_CKPOL_SHIFT      (1)       /* Bits 1-2: Clock polarity */
#define LPTIM_CFGR_CKPOL_MASK       (3 << LPTIM_CFGR_CKPOL_SHIFT)
#  define LPTIM_CFGR_CKPOL_RISING   (0 << LPTIM_CFGR_CKPOL_SHIFT)
#  define LPTIM_CFGR_CKPOL_FALLING  (1 << LPTIM_CFGR_CKPOL_SHIFT)
#  define LPTIM_CFGR_CKPOL_BOTH     (2 << LPTIM_CFGR_CKPOL_SHIFT)
#define LPTIM_CFGR_CKFLT_SHIFT      (3)       /* Bits 3-4: Clock filter */
#define LPTIM_CFGR_CKFLT_MASK       (3 << LPTIM_CFGR_CKFLT_SHIFT)
#define LPTIM_CFGR_TRGFLT_SHIFT     (6)       /* Bits 6-7: Trigger filter */
#define LPTIM_CFGR_TRGFLT_MASK      (3 << LPTIM_CFGR_TRGFLT_SHIFT)
#define LPTIM_CFGR_PRESC_SHIFT      (9)       /* Bits 9-11: Clock prescaler */
#define LPTIM_CFGR_PRESC_MASK       (7 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_1        (0 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_2        (1 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_4        (2 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_8        (3 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_16       (4 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_32       (5 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_64       (6 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_128      (7 << LPTIM_CFGR_PRESC_SHIFT)
#define LPTIM_CFGR_TRIGSEL_SHIFT    (13)      /* Bits 13-15: Trigger select */
#define LPTIM_CFGR_TRIGSEL_MASK     (7 << LPTIM_CFGR_TRIGSEL_SHIFT)
#ifdef CONFIG_ARCH_CHIP_STM32G4
#  define LPTIM_CFGR_TRIGSEL_3      (1 << 29) /* Bit 29: TRIGSEL bit 3 */
#endif
#define LPTIM_CFGR_TRIGEN_SHIFT     (17)      /* Bits 17-18: Trigger enable */
#define LPTIM_CFGR_TRIGEN_MASK      (3 << LPTIM_CFGR_TRIGEN_SHIFT)
#  define LPTIM_CFGR_TRIGEN_SW      (0 << LPTIM_CFGR_TRIGEN_SHIFT)
#  define LPTIM_CFGR_TRIGEN_RISING  (1 << LPTIM_CFGR_TRIGEN_SHIFT)
#  define LPTIM_CFGR_TRIGEN_FALLING (2 << LPTIM_CFGR_TRIGEN_SHIFT)
#  define LPTIM_CFGR_TRIGEN_BOTH    (3 << LPTIM_CFGR_TRIGEN_SHIFT)
#define LPTIM_CFGR_TIMOUT           (1 << 19) /* Bit 19: Timeout enable */
#define LPTIM_CFGR_WAVE             (1 << 20) /* Bit 20: Waveform shape */
#define LPTIM_CFGR_WAVPOL           (1 << 21) /* Bit 21: Waveform polarity */
#define LPTIM_CFGR_PRELOAD          (1 << 22) /* Bit 22: Preload */
#define LPTIM_CFGR_COUNTMODE        (1 << 23) /* Bit 23: Count mode */
#define LPTIM_CFGR_ENC              (1 << 24) /* Bit 24: Encoder mode */

/* Control register */

#define LPTIM_CR_ENABLE             (1 << 0)  /* Bit 0: LPTIM enable */
#define LPTIM_CR_SNGSTRT            (1 << 1)  /* Bit 1: Single start */
#define LPTIM_CR_CNTSTRT            (1 << 2)  /* Bit 2: Continuous start */
#define LPTIM_CR_COUNTRST           (1 << 3)  /* Bit 3: Counter reset */
#define LPTIM_CR_RSTARE             (1 << 4)  /* Bit 4: Reset after read */

/* Configuration register 2 */

#define LPTIM_CFGR2_IN1SEL_SHIFT    (0)       /* Bits 0-1: Input 1 select */
#define LPTIM_CFGR2_IN1SEL_MASK     (3 << LPTIM_CFGR2_IN1SEL_SHIFT)
#define LPTIM_CFGR2_IN2SEL_SHIFT    (4)       /* Bits 4-5: Input 2 select */
#define LPTIM_CFGR2_IN2SEL_MASK     (3 << LPTIM_CFGR2_IN2SEL_SHIFT)
#ifdef CONFIG_STM32_HAVE_IP_LPTIM_V2
#  define LPTIM_CFGR2_IC1SEL_SHIFT  (16)      /* Bits 16-17: IC1 selection */
#  define LPTIM_CFGR2_IC1SEL_MASK   (3 << LPTIM_CFGR2_IC1SEL_SHIFT)
#  define LPTIM_CFGR2_IC2SEL_SHIFT  (20)      /* Bits 20-21: IC2 selection */
#  define LPTIM_CFGR2_IC2SEL_MASK   (3 << LPTIM_CFGR2_IC2SEL_SHIFT)
#endif

/* Repetition register */

#define LPTIM_RCR_REP_SHIFT         (0)       /* Bits 0-7: Repetition value */
#define LPTIM_RCR_REP_MASK          (0xff << LPTIM_RCR_REP_SHIFT)

#ifdef CONFIG_STM32_HAVE_IP_LPTIM_V2
/* Capture/compare mode registers 1 and 2 (channel n: 1/3 low, 2/4 high) */

#  define LPTIM_CCMR1_CC1SEL        (1 << 0)  /* Bit 0: CC1 selection */
#  define LPTIM_CCMR1_CC1E          (1 << 1)  /* Bit 1: CC1 output enable */
#  define LPTIM_CCMR1_CC1P_SHIFT    (2)       /* Bits 2-3: CC1 polarity */
#  define LPTIM_CCMR1_CC1P_MASK     (3 << LPTIM_CCMR1_CC1P_SHIFT)
#  define LPTIM_CCMR1_IC1PSC_SHIFT  (8)       /* Bits 8-9: IC1 prescaler */
#  define LPTIM_CCMR1_IC1PSC_MASK   (3 << LPTIM_CCMR1_IC1PSC_SHIFT)
#  define LPTIM_CCMR1_IC1F_SHIFT    (12)      /* Bits 12-13: IC1 filter */
#  define LPTIM_CCMR1_IC1F_MASK     (3 << LPTIM_CCMR1_IC1F_SHIFT)
#  define LPTIM_CCMR1_CC2SEL        (1 << 16) /* Bit 16: CC2 selection */
#  define LPTIM_CCMR1_CC2E          (1 << 17) /* Bit 17: CC2 output enable */
#  define LPTIM_CCMR1_CC2P_SHIFT    (18)      /* Bits 18-19: CC2 polarity */
#  define LPTIM_CCMR1_CC2P_MASK     (3 << LPTIM_CCMR1_CC2P_SHIFT)
#  define LPTIM_CCMR1_IC2PSC_SHIFT  (24)      /* Bits 24-25: IC2 prescaler */
#  define LPTIM_CCMR1_IC2PSC_MASK   (3 << LPTIM_CCMR1_IC2PSC_SHIFT)
#  define LPTIM_CCMR1_IC2F_SHIFT    (28)      /* Bits 28-29: IC2 filter */
#  define LPTIM_CCMR1_IC2F_MASK     (3 << LPTIM_CCMR1_IC2F_SHIFT)

#  define LPTIM_CCMR2_CC3SEL        (1 << 0)  /* Bit 0: CC3 selection */
#  define LPTIM_CCMR2_CC3E          (1 << 1)  /* Bit 1: CC3 output enable */
#  define LPTIM_CCMR2_CC3P_SHIFT    (2)       /* Bits 2-3: CC3 polarity */
#  define LPTIM_CCMR2_CC3P_MASK     (3 << LPTIM_CCMR2_CC3P_SHIFT)
#  define LPTIM_CCMR2_IC3PSC_SHIFT  (8)       /* Bits 8-9: IC3 prescaler */
#  define LPTIM_CCMR2_IC3PSC_MASK   (3 << LPTIM_CCMR2_IC3PSC_SHIFT)
#  define LPTIM_CCMR2_IC3F_SHIFT    (12)      /* Bits 12-13: IC3 filter */
#  define LPTIM_CCMR2_IC3F_MASK     (3 << LPTIM_CCMR2_IC3F_SHIFT)
#  define LPTIM_CCMR2_CC4SEL        (1 << 16) /* Bit 16: CC4 selection */
#  define LPTIM_CCMR2_CC4E          (1 << 17) /* Bit 17: CC4 output enable */
#  define LPTIM_CCMR2_CC4P_SHIFT    (18)      /* Bits 18-19: CC4 polarity */
#  define LPTIM_CCMR2_CC4P_MASK     (3 << LPTIM_CCMR2_CC4P_SHIFT)
#  define LPTIM_CCMR2_IC4PSC_SHIFT  (24)      /* Bits 24-25: IC4 prescaler */
#  define LPTIM_CCMR2_IC4PSC_MASK   (3 << LPTIM_CCMR2_IC4PSC_SHIFT)
#  define LPTIM_CCMR2_IC4F_SHIFT    (28)      /* Bits 28-29: IC4 filter */
#  define LPTIM_CCMR2_IC4F_MASK     (3 << LPTIM_CCMR2_IC4F_SHIFT)
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_LPTIM_H */
