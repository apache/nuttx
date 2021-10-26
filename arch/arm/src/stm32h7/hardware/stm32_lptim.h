/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_lptim.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_LPTIM_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_LPTIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_LPTIM_ISR_OFFSET     0x0000  /* Interrupt and status register */
#define STM32_LPTIM_ICR_OFFSET     0x0004  /* Interrupt clear register */
#define STM32_LPTIM_IER_OFFSET     0x0008  /* Interrupt enable register */
#define STM32_LPTIM_CFGR_OFFSET    0x000C  /* Configuration register */
#define STM32_LPTIM_CR_OFFSET      0x0010  /* Control register */
#define STM32_LPTIM_CMP_OFFSET     0x0014  /* Compare register */
#define STM32_LPTIM_ARR_OFFSET     0x0018  /* Autoreloar register */
#define STM32_LPTIM_CNT_OFFSET     0x001C  /* Counter register */
#define STM32_LPTIM_CFGR2_OFFSET   0x0024  /* Configuration register 2 */

/* Register Addresses *******************************************************/

#  define STM32_LPTIM1_ISR          (STM32_LPTIM1_BASE+STM32_LPTIM_ISR_OFFSET)
#  define STM32_LPTIM1_ICR          (STM32_LPTIM1_BASE+STM32_LPTIM_ICR_OFFSET)
#  define STM32_LPTIM1_IER          (STM32_LPTIM1_BASE+STM32_LPTIM_IER_OFFSET)
#  define STM32_LPTIM1_CFGR         (STM32_LPTIM1_BASE+STM32_LPTIM_CFGR_OFFSET)
#  define STM32_LPTIM1_CR           (STM32_LPTIM1_BASE+STM32_LPTIM_CR_OFFSET)
#  define STM32_LPTIM1_CMP          (STM32_LPTIM1_BASE+STM32_LPTIM_CMP_OFFSET)
#  define STM32_LPTIM1_ARR          (STM32_LPTIM1_BASE+STM32_LPTIM_ARR_OFFSET)
#  define STM32_LPTIM1_CNT          (STM32_LPTIM1_BASE+STM32_LPTIM_CNT_OFFSET)
#  define STM32_LPTIM1_CFGR2        (STM32_LPTIM1_BASE+STM32_LPTIM_CFGR2_OFFSET)

#  define STM32_LPTIM2_ISR          (STM32_LPTIM2_BASE+STM32_LPTIM_ISR_OFFSET)
#  define STM32_LPTIM2_ICR          (STM32_LPTIM2_BASE+STM32_LPTIM_ICR_OFFSET)
#  define STM32_LPTIM2_IER          (STM32_LPTIM2_BASE+STM32_LPTIM_IER_OFFSET)
#  define STM32_LPTIM2_CFGR         (STM32_LPTIM2_BASE+STM32_LPTIM_CFGR_OFFSET)
#  define STM32_LPTIM2_CR           (STM32_LPTIM2_BASE+STM32_LPTIM_CR_OFFSET)
#  define STM32_LPTIM2_CMP          (STM32_LPTIM2_BASE+STM32_LPTIM_CMP_OFFSET)
#  define STM32_LPTIM2_ARR          (STM32_LPTIM2_BASE+STM32_LPTIM_ARR_OFFSET)
#  define STM32_LPTIM2_CNT          (STM32_LPTIM2_BASE+STM32_LPTIM_CNT_OFFSET)
#  define STM32_LPTIM2_CFGR2        (STM32_LPTIM2_BASE+STM32_LPTIM_CFGR2_OFFSET)

#  define STM32_LPTIM3_ISR          (STM32_LPTIM3_BASE+STM32_LPTIM_ISR_OFFSET)
#  define STM32_LPTIM3_ICR          (STM32_LPTIM3_BASE+STM32_LPTIM_ICR_OFFSET)
#  define STM32_LPTIM3_IER          (STM32_LPTIM3_BASE+STM32_LPTIM_IER_OFFSET)
#  define STM32_LPTIM3_CFGR         (STM32_LPTIM3_BASE+STM32_LPTIM_CFGR_OFFSET)
#  define STM32_LPTIM3_CR           (STM32_LPTIM3_BASE+STM32_LPTIM_CR_OFFSET)
#  define STM32_LPTIM3_CMP          (STM32_LPTIM3_BASE+STM32_LPTIM_CMP_OFFSET)
#  define STM32_LPTIM3_ARR          (STM32_LPTIM3_BASE+STM32_LPTIM_ARR_OFFSET)
#  define STM32_LPTIM3_CNT          (STM32_LPTIM3_BASE+STM32_LPTIM_CNT_OFFSET)
#  define STM32_LPTIM3_CFGR2        (STM32_LPTIM3_BASE+STM32_LPTIM_CFGR2_OFFSET)

#  define STM32_LPTIM4_ISR          (STM32_LPTIM4_BASE+STM32_LPTIM_ISR_OFFSET)
#  define STM32_LPTIM4_ICR          (STM32_LPTIM4_BASE+STM32_LPTIM_ICR_OFFSET)
#  define STM32_LPTIM4_IER          (STM32_LPTIM4_BASE+STM32_LPTIM_IER_OFFSET)
#  define STM32_LPTIM4_CFGR         (STM32_LPTIM4_BASE+STM32_LPTIM_CFGR_OFFSET)
#  define STM32_LPTIM4_CR           (STM32_LPTIM4_BASE+STM32_LPTIM_CR_OFFSET)
#  define STM32_LPTIM4_CMP          (STM32_LPTIM4_BASE+STM32_LPTIM_CMP_OFFSET)
#  define STM32_LPTIM4_ARR          (STM32_LPTIM4_BASE+STM32_LPTIM_ARR_OFFSET)
#  define STM32_LPTIM4_CNT          (STM32_LPTIM4_BASE+STM32_LPTIM_CNT_OFFSET)
#  define STM32_LPTIM4_CFGR2        (STM32_LPTIM4_BASE+STM32_LPTIM_CFGR2_OFFSET)

#  define STM32_LPTIM5_ISR          (STM32_LPTIM5_BASE+STM32_LPTIM_ISR_OFFSET)
#  define STM32_LPTIM5_ICR          (STM32_LPTIM5_BASE+STM32_LPTIM_ICR_OFFSET)
#  define STM32_LPTIM5_IER          (STM32_LPTIM5_BASE+STM32_LPTIM_IER_OFFSET)
#  define STM32_LPTIM5_CFGR         (STM32_LPTIM5_BASE+STM32_LPTIM_CFGR_OFFSET)
#  define STM32_LPTIM5_CR           (STM32_LPTIM5_BASE+STM32_LPTIM_CR_OFFSET)
#  define STM32_LPTIM5_CMP          (STM32_LPTIM5_BASE+STM32_LPTIM_CMP_OFFSET)
#  define STM32_LPTIM5_ARR          (STM32_LPTIM5_BASE+STM32_LPTIM_ARR_OFFSET)
#  define STM32_LPTIM5_CNT          (STM32_LPTIM5_BASE+STM32_LPTIM_CNT_OFFSET)
#  define STM32_LPTIM5_CFGR2        (STM32_LPTIM5_BASE+STM32_LPTIM_CFGR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Interrupt flags for ISR, ICR and IER */

#define LPTIM_IxR_CMPM              (1 << 0)  /* Bit 0: Compare match */
#define LPTIM_IxR_ARRM              (1 << 1)  /* Bit 1: Autoreload match */
#define LPTIM_IxR_EXTTRIG           (1 << 2)  /* Bit 2: External trigger edge event */
#define LPTIM_IxR_CMPOK             (1 << 3)  /* Bit 3: Compare register update OK */
#define LPTIM_IxR_ARROK             (1 << 4)  /* Bit 4: Autoreload register update OK */
#define LPTIM_IxR_UP                (1 << 5)  /* Bit 5: Counter direction change down to up */
#define LPTIM_IxR_DOWN              (1 << 6)  /* Bit 6: Counter direction change up to down */

/* Configuration register */

#define LPTIM_CFGR_CKSEL            (1 << 0)  /* Bit 0: Clock selector */
#define LPTIM_CFGR_CKPOL_SHIFT      (1)       /* Bits 1-2: Clock polarity */
#define LPTIM_CFGR_CKPOL_MASK       (3 << LPTIM_CFGR_CKPOL_SHIFT)
#  define LPTIM_CFGR_CKPOL_RISE     (0 << LPTIM_CFGR_CKPOL_SHIFT) /* 00: Rising edge */
#  define LPTIM_CFGR_CKPOL_FALL     (1 << LPTIM_CFGR_CKPOL_SHIFT) /* 01: Falling edge */
#  define LPTIM_CFGR_CKPOL_BOTH     (2 << LPTIM_CFGR_CKPOL_SHIFT) /* 10: Both edges */

#define LPTIM_CFGR_CKFLT_SHIFT      (3)       /* Bits 3-4: Digital filter for clock*/
#define LPTIM_CFGR_CKFLT_MASK       (3 << LPTIM_CFGR_CKFLT_SHIFT)
#  define LPTIM_CFGR_CKFLT_ANY      (0 << LPTIM_CFGR_CKFLT_SHIFT) /* 00: Any */
#  define LPTIM_CFGR_CKFLT_2        (1 << LPTIM_CFGR_CKFLT_SHIFT) /* 01: 2 clocks */
#  define LPTIM_CFGR_CKFLT_4        (2 << LPTIM_CFGR_CKFLT_SHIFT) /* 10: 4 clocks */
#  define LPTIM_CFGR_CKFLT_8        (3 << LPTIM_CFGR_CKFLT_SHIFT) /* 11: 8 clocks */

#define LPTIM_CFGR_TRGFLT_SHIFT     (6)       /* Bits 6-7: digital filter for trigger */
#define LPTIM_CFGR_TRGFLT_MASK      (3 << LPTIM_CFGR_TRGFLT_SHIFT)
#  define LPTIM_CFGR_TRGFLT_ANY     (0 << LPTIM_CFGR_TRGFLT_SHIFT) /* 00: Any */
#  define LPTIM_CFGR_TRGFLT_2       (1 << LPTIM_CFGR_TRGFLT_SHIFT) /* 01: 2 clocks */
#  define LPTIM_CFGR_TRGFLT_4       (2 << LPTIM_CFGR_TRGFLT_SHIFT) /* 10: 4 clocks */
#  define LPTIM_CFGR_TRGFLT_8       (3 << LPTIM_CFGR_TRGFLT_SHIFT) /* 11: 8 clocks */

#define LPTIM_CFGR_PRESC_SHIFT      (9)       /* Bits 9-11: Clock prescaler */
#define LPTIM_CFGR_PRESC_MASK	    (7 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_1        (0 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_2        (1 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_4        (2 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_8        (3 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_16       (4 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_32       (5 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_64       (6 << LPTIM_CFGR_PRESC_SHIFT)
#  define LPTIM_CFGR_PRESC_128      (7 << LPTIM_CFGR_PRESC_SHIFT)

#define LPTIM_CFGR_TRIGSEL_SHIFT    (13)      /* Bits 13-15: Trigger selector */
#define LPTIM_CFGR_TRIGSEL_MASK     (7 << LPTIM_CFGR_TRIGSEL_SHIFT)
#define LPTIM_CFGR_TRIGEN_SHIFT     (17)      /* Bits 17-18: Trigger enable and polarity */
#define LPTIM_CFGR_TRIGEN_MASK      (3 << LPTIM_CFGR_TRIGEN_SHIFT)
#  define LPTIM_CFGR_TRIGEN_SW      (0 << LPTIM_CFGR_TRIGEN_SHIFT) /* 00: software */
#  define LPTIM_CFGR_TRIGEN_RISE    (1 << LPTIM_CFGR_TRIGEN_SHIFT) /* 01: rising edge */
#  define LPTIM_CFGR_TRIGEN_FALL    (2 << LPTIM_CFGR_TRIGEN_SHIFT) /* 10: falling edge */
#  define LPTIM_CFGR_TRIGEN_BOTH    (3 << LPTIM_CFGR_TRIGEN_SHIFT) /* 11: both edges */

#define LPTIM_CFGR_TIMEOUT          (1 << 19) /* Bit 19: Timeout enable */
#define LPTIM_CFGR_WAVE             (1 << 20) /* Bit 20: Waveform shape */
#define LPTIM_CFGR_WAVPOL           (1 << 21) /* Bit 21: Waveform shape polarity */
#define LPTIM_CFGR_PRELOAD          (1 << 22) /* Bit 22: Registers update mode */
#define LPTIM_CFGR_COUNTMODE        (1 << 23) /* Bit 23: Counter mode  enable */
#define LPTIM_CFGR_ENC              (1 << 24) /* Bit 24: Encoder mode enable */

/* Control register */

#define LPTIM_CR_ENABLE             (1 << 0)  /* Bit 0: Enable */
#define LPTIM_CR_SNGSTRT            (1 << 1)  /* Bit 1: Start in single mode */
#define LPTIM_CR_CNTSTRT            (1 << 2)  /* Bit 2: Start in continuous mode */
#define LPTIM_CR_COUNTRST           (1 << 3)  /* Bit 3: Counter reset */
#define LPTIM_CR_RSTARE             (1 << 4)  /* Reset after read enable */

/* Configuration register 2 */

#define LPTIM_CFGR2_IN1SEL_SHIFT    (0)       /* Input 1 selection */
#define LPTIM_CFGR2_IN1SEL_MASK     (3 << LPTIM_CFGR2_IN1SEL_SHIFT)
#define LPTIM_CFGR2_IN2SEL_SHIFT    (4)       /* Input 2 selection */
#define LPTIM_CFGR2_IN2SEL_MASK     (3 << LPTIM_CFGR2_IN2SEL_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_LPTIM_H */
