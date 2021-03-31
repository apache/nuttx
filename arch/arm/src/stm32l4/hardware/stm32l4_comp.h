/****************************************************************************
 * arch/arm/src/stm32l4/hardware/stm32l4_comp.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_COMP_H
#define __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_COMP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32L4_COMP_CSR_OFFSET(n) (((n)-1) << 2)
#define STM32L4_COMP1_CSR_OFFSET   0x0000  /* Comparator 1 control and status register */
#define STM32L4_COMP2_CSR_OFFSET   0x0004  /* Comparator 2 control and status register */

/* Register Addresses *******************************************************/

#define STM32L4_COMP_CSR(n)        (STM32L4_COMP_BASE+STM32L4_COMP_CSR_OFFSET(n))
#define STM32L4_COMP1_CSR          (STM32L4_COMP_BASE+STM32L4_COMP1_CSR_OFFSET)
#define STM32L4_COMP2_CSR          (STM32L4_COMP_BASE+STM32L4_COMP2_CSR_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define COMP_CSR_EN                (1 << 0)  /* Bit 0:  Comparator enable bit */
                                             /* Bit 1: Reserved */
#define COMP_CSR_PWRMODE_SHIFT     (2)       /* Bits 2-3: Power Mode */
#define COMP_CSR_PWRMODE_MASK      (3 << COMP_CSR_PWRMODE_SHIFT)
#  define COMP_CSR_PWRMODE_HIGH    (0 << COMP_CSR_PWRMODE_SHIFT) /* High speed */
#  define COMP_CSR_PWRMODE_MEDIUM  (1 << COMP_CSR_PWRMODE_SHIFT) /* Medium speed */
#  define COMP_CSR_PWRMODE_LOW     (3 << COMP_CSR_PWRMODE_SHIFT) /* Ultra low power */

#define COMP_CSR_INMSEL_SHIFT      (4)       /* Bits 4-6: Input minus selection bits */
#define COMP_CSR_INMSEL_MASK       (7 << COMP_CSR_INMSEL_SHIFT)
#  define COMP_CSR_INMSEL_25PCT    (0 << COMP_CSR_INMSEL_SHIFT) /* 1/4 VREFINT */
#  define COMP_CSR_INMSEL_50PCT    (1 << COMP_CSR_INMSEL_SHIFT) /* 1/2 VREFINT */
#  define COMP_CSR_INMSEL_75PCT    (2 << COMP_CSR_INMSEL_SHIFT) /* 3/4 VREFINT */
#  define COMP_CSR_INMSEL_VREF     (3 << COMP_CSR_INMSEL_SHIFT) /* VREFINT */
#  define COMP_CSR_INMSEL_DAC1     (4 << COMP_CSR_INMSEL_SHIFT) /* DAC Channel1 */
#  define COMP_CSR_INMSEL_DAC2     (5 << COMP_CSR_INMSEL_SHIFT) /* DAC Channel2 */
#  define COMP_CSR_INMSEL_PIN1     (6 << COMP_CSR_INMSEL_SHIFT) /* Input minus pin 1: COMP1=PB1; COMP2=PB3 */
#if defined(CONFIG_STM32L4_STM32L4X3)
#  define COMP_CSR_INMSEL_INMESEL  (7 << COMP_CSR_INMSEL_SHIFT) /* Input minus pin 2: Selected by INMESEL */
#else
#  define COMP_CSR_INMSEL_PIN2     (7 << COMP_CSR_INMSEL_SHIFT) /* Input minus pin 2: COMP1=PC4; COMP2=PB7 */
#endif

#define COMP_CSR_INPSEL_SHIFT      (7)       /* Bits 7-8: Input plus selection bits */
#define COMP_CSR_INPSEL_MASK       (3 << COMP_CSR_INPSEL_SHIFT)
#  define COMP_CSR_INPSEL_PIN1     (0 << COMP_CSR_INPSEL_SHIFT) /* Input plus pin 1: COMP1=PC5; COMP2=PB4 */
#  define COMP_CSR_INPSEL_PIN2     (1 << COMP_CSR_INPSEL_SHIFT) /* Input plus pin 2: COMP1=PB2; COMP2=PB6 */
#if defined(CONFIG_STM32L4_STM32L4X3)
#  define COMP_CSR_INPSEL_PIN3     (2 << COMP_CSR_INPSEL_SHIFT) /* Input plus pin 3: COMP1=PA1; COMP2=PA3 */
#endif

#define COMP2_CSR_WINMODE          (1 << 9)  /* Bit 9:  Windows mode selection bit (COMP2 only) */

#  define COMP2_CSR_WINMODE_NOCONN (0)                /* Comparator 2 input not connected to Comparator 1 */
#  define COMP2_CSR_WINMODE_CONN   COMP2_CSR_WINMODE  /* Comparator 2 input connected to Comparator 1 */

#define COMP_CSR_POLARITY_MASK     (1 << 15) /* Bit 15: Polarity selection bit */
#  define COMP_CSR_POLARITY_NORMAL (0)
#  define COMP_CSR_POLARITY_INVERT COMP_CSR_POLARITY_MASK
#define COMP_CSR_HYST_SHIFT        (16)      /* Bits 16-17: Hysteresis selection bits */
#define COMP_CSR_HYST_MASK         (3 << COMP_CSR_HYST_SHIFT)
#  define COMP_CSR_HYST_NONE       (0 << COMP_CSR_HYST_SHIFT) /* No hysteresis */
#  define COMP_CSR_HYST_LOW        (1 << COMP_CSR_HYST_SHIFT) /* Low hysteresis */
#  define COMP_CSR_HYST_MEDIUM     (2 << COMP_CSR_HYST_SHIFT) /* Medium hysteresis */
#  define COMP_CSR_HYST_HIGH       (3 << COMP_CSR_HYST_SHIFT) /* High hysteresis */

#define COMP_CSR_BLANK_SHIFT       (18)      /* Bits 18-20: Blanking source selection bits */
#define COMP_CSR_BLANK_MASK        (7 << COMP_CSR_BLANK_SHIFT)
#  define COMP_CSR_BLANK_NONE      (0 << COMP_CSR_BLANK_SHIFT) /* No blanking */
#  define COMP1_CSR_BLANK_TIM1OC5  (1 << COMP_CSR_BLANK_SHIFT) /* TIM1 OC5 is blanking source */
#  define COMP1_CSR_BLANK_TIM2OC3  (2 << COMP_CSR_BLANK_SHIFT) /* TIM2 OC3 is blanking source */
#  define COMP1_CSR_BLANK_TIM3OC3  (4 << COMP_CSR_BLANK_SHIFT) /* TIM3 OC3 is blanking source */
#  define COMP2_CSR_BLANK_TIM3OC4  (1 << COMP_CSR_BLANK_SHIFT) /* TIM3 OC4 is blanking source */
#  define COMP2_CSR_BLANK_TIM8OC5  (2 << COMP_CSR_BLANK_SHIFT) /* TIM8 OC5 is blanking source */
#  define COMP2_CSR_BLANK_TIM15OC1 (4 << COMP_CSR_BLANK_SHIFT) /* TIM15 OC1 is blanking source */

                                             /* Bit 21: Reserved */
#define COMP_CSR_BRGEN             (1 << 22) /* Bit 22: Scaler bridge enable */
#define COMP_CSR_SCALEN            (1 << 23) /* Bit 23: Voltage scaler enable bit */
                                             /* Bit 24: Reserved */
#if defined(CONFIG_STM32L4_STM32L4X3)
#  define COMP_CSR_INMESEL_SHIFT   (25)      /* Bits 25-26: Input minus extended selection bits */
#  define COMP_CSR_INMESEL_MASK    (3 << COMP_CSR_INMESEL_SHIFT)
#    define COMP_CSR_INMESEL_PIN2  (0 << COMP_CSR_INMESEL_SHIFT) /* Input minus pin 2: COMP1=PC4; COMP2=PB7 */
#    define COMP_CSR_INMESEL_PIN3  (1 << COMP_CSR_INMESEL_SHIFT) /* Input minus pin 3: COMP1=PA0; COMP2=PA2 */
#    define COMP_CSR_INMESEL_PIN4  (2 << COMP_CSR_INMESEL_SHIFT) /* Input minus pin 4: COMP1=PA4; COMP2=PA4 */
#    define COMP_CSR_INMESEL_PIN5  (3 << COMP_CSR_INMESEL_SHIFT) /* Input minus pin 5: COMP1=PA5; COMP2=PA5 */
#endif
                                             /* Bits 27-29: Reserved */
#define COMP_CSR_VALUE             (1 << 30) /* Bit 30: Comparator output status bit */
#define COMP_CSR_LOCK_MASK         (1 << 31) /* Bit 31: CSR register lock bit */
#  define COMP_CSR_LOCK_RW         (0)
#  define COMP_CSR_LOCK_RO         COMP_CSR_LOCK_MASK

#endif /* __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_COMP_H */
