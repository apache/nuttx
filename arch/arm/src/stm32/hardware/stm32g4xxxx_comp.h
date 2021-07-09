/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxxx_comp.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_COMP_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_COMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses *******************************************************/

#define STM32_COMP1_CSR             STM32_COMP1_BASE
#define STM32_COMP2_CSR             STM32_COMP2_BASE
#define STM32_COMP3_CSR             STM32_COMP3_BASE
#define STM32_COMP4_CSR             STM32_COMP4_BASE
#define STM32_COMP5_CSR             STM32_COMP5_BASE
#define STM32_COMP6_CSR             STM32_COMP6_BASE
#define STM32_COMP7_CSR             STM32_COMP7_BASE

/* Register Bitfield Definitions ********************************************/

/* COMP control and status register */

#define COMP_CSR_COMPEN             (1 << 0)                      /* Bit 0: Comparator enable */
                                                                  /* Bits 3-1: Reserved */
#define COMP_CSR_INMSEL_SHIFT       (4)                           /* Bits 7-4: Comparator inverting input selection */
#define COMP_CSR_INMSEL_MASK        (15 << COMP_CSR_INMSEL_SHIFT)
#  define COMP_CSR_INMSEL_1P4VREF   (0 << COMP_CSR_INMSEL_SHIFT)  /* 0000: 1/4 of Vrefint */
#  define COMP_CSR_INMSEL_1P2VREF   (1 << COMP_CSR_INMSEL_SHIFT)  /* 0001: 1/2 of Vrefint */
#  define COMP_CSR_INMSEL_3P4VREF   (2 << COMP_CSR_INMSEL_SHIFT)  /* 0010: 3/4 of Vrefint */
#  define COMP_CSR_INMSEL_VREF      (3 << COMP_CSR_INMSEL_SHIFT)  /* 0011: Vrefint */
#  define COMP_CSR_INMSEL_DAC3CH1   (4 << COMP_CSR_INMSEL_SHIFT)  /* 0100: DAC3_CH1 (COMP1 and COMP3 only) */
#  define COMP_CSR_INMSEL_DAC3CH2   (4 << COMP_CSR_INMSEL_SHIFT)  /* 0100: DAC3_CH2 (COMP2 and COMP4 only) */
#  define COMP_CSR_INMSEL_DAC4CH1   (4 << COMP_CSR_INMSEL_SHIFT)  /* 0100: DAC4_CH1 (COMP5 and COMP7 only) */
#  define COMP_CSR_INMSEL_DAC4CH2   (4 << COMP_CSR_INMSEL_SHIFT)  /* 0100: DAC4_CH2 (COMP6 only) */
#  define COMP_CSR_INMSEL_DAC1CH1   (5 << COMP_CSR_INMSEL_SHIFT)  /* 0101: DAC1_CH1 (COMP1, COMP3 and COMP4 only) */
#  define COMP_CSR_INMSEL_DAC1CH2   (5 << COMP_CSR_INMSEL_SHIFT)  /* 0101: DAC1_CH2 (COMP2 and COMP5 only) */
#  define COMP_CSR_INMSEL_DAC2CH1   (5 << COMP_CSR_INMSEL_SHIFT)  /* 0101: DAC2_CH1 (COMP6 and COMP7 only) */
#  define COMP_CSR_INMSEL_PA4       (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: PA4 (COMP1 only) */
#  define COMP_CSR_INMSEL_PA5       (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: PA5 (COMP2 only) */
#  define COMP_CSR_INMSEL_PF1       (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: PF1 (COMP3 only) */
#  define COMP_CSR_INMSEL_PE8       (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: PE8 (COMP4 only) */
#  define COMP_CSR_INMSEL_PB10      (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: PB10 (COMP5 only) */
#  define COMP_CSR_INMSEL_PD10      (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: PD10 (COMP6 only) */
#  define COMP_CSR_INMSEL_PD15      (6 << COMP_CSR_INMSEL_SHIFT)  /* 0110: PD15 (COMP7 only) */
#  define COMP_CSR_INMSEL_PA0       (7 << COMP_CSR_INMSEL_SHIFT)  /* 0111: PA0 (COMP1 only) */
#  define COMP_CSR_INMSEL_PA2       (7 << COMP_CSR_INMSEL_SHIFT)  /* 0111: PA2 (COMP2 only) */
#  define COMP_CSR_INMSEL_PC0       (7 << COMP_CSR_INMSEL_SHIFT)  /* 0111: PC0 (COMP3 only) */
#  define COMP_CSR_INMSEL_PB2       (7 << COMP_CSR_INMSEL_SHIFT)  /* 0111: PB2 (COMP4 only) */
#  define COMP_CSR_INMSEL_PD13      (7 << COMP_CSR_INMSEL_SHIFT)  /* 0111: PD13 (COMP5 only) */
#  define COMP_CSR_INMSEL_PB15      (7 << COMP_CSR_INMSEL_SHIFT)  /* 0111: PB15 (COMP6 only) */
#  define COMP_CSR_INMSEL_PB12      (7 << COMP_CSR_INMSEL_SHIFT)  /* 0111: PB12 (COMP7 only) */

#define COMP_CSR_INPSEL_SHIFT       (8)                           /* Bit 8: Comparator non-inverting input selection */
#  define COMP_CSR_INPSEL_PA1       (0 << COMP_CSR_INPSEL_SHIFT)  /* PA1 (COMP1 only) */
#  define COMP_CSR_INPSEL_PB1       (1 << COMP_CSR_INPSEL_SHIFT)  /* PB1 (COMP1 only) */
#  define COMP_CSR_INPSEL_PA7       (0 << COMP_CSR_INPSEL_SHIFT)  /* PA7 (COMP2 only) */
#  define COMP_CSR_INPSEL_PA3       (1 << COMP_CSR_INPSEL_SHIFT)  /* PA3 (COMP2 only) */
#  define COMP_CSR_INPSEL_PA0       (0 << COMP_CSR_INPSEL_SHIFT)  /* PA0 (COMP3 only) */
#  define COMP_CSR_INPSEL_PC1       (1 << COMP_CSR_INPSEL_SHIFT)  /* PC1 (COMP3 only) */
#  define COMP_CSR_INPSEL_PB0       (0 << COMP_CSR_INPSEL_SHIFT)  /* PB0 (COMP4 only) */
#  define COMP_CSR_INPSEL_PE7       (1 << COMP_CSR_INPSEL_SHIFT)  /* PE7 (COMP4 only) */
#  define COMP_CSR_INPSEL_PB13      (0 << COMP_CSR_INPSEL_SHIFT)  /* PB13 (COMP5 only) */
#  define COMP_CSR_INPSEL_PD12      (1 << COMP_CSR_INPSEL_SHIFT)  /* PD12 (COMP5 only) */
#  define COMP_CSR_INPSEL_PB11      (0 << COMP_CSR_INPSEL_SHIFT)  /* PB11 (COMP6 only) */
#  define COMP_CSR_INPSEL_PD11      (1 << COMP_CSR_INPSEL_SHIFT)  /* PD11 (COMP6 only) */
#  define COMP_CSR_INPSEL_PB14      (0 << COMP_CSR_INPSEL_SHIFT)  /* PB14 (COMP7 only) */
#  define COMP_CSR_INPSEL_PD14      (1 << COMP_CSR_INPSEL_SHIFT)  /* PD14 (COMP7 only) */
                                                                  /* Bits 14-9: Reserved */
#define COMP_CSR_POL                (1 << 15)                     /* Bit 15: comparator output polarity */

#define COMP_CSR_HYST_SHIFT         (16)                          /* Bits 18-16: Comparator hysteresis */
#define COMP_CSR_HYST_MASK          (7 << COMP_CSR_HYST_SHIFT)
#  define COMP_CSR_HYST_0MV         (0 << COMP_CSR_HYST_SHIFT)    /* No hysteresis */
#  define COMP_CSR_HYST_10MV        (1 << COMP_CSR_HYST_SHIFT)    /* 10mV hysteresis */
#  define COMP_CSR_HYST_20MV        (2 << COMP_CSR_HYST_SHIFT)    /* 20mV hysteresis */
#  define COMP_CSR_HYST_30MV        (3 << COMP_CSR_HYST_SHIFT)    /* 30mV hysteresis */
#  define COMP_CSR_HYST_40MV        (4 << COMP_CSR_HYST_SHIFT)    /* 40mV hysteresis */
#  define COMP_CSR_HYST_50MV        (5 << COMP_CSR_HYST_SHIFT)    /* 50mV hysteresis */
#  define COMP_CSR_HYST_60MV        (6 << COMP_CSR_HYST_SHIFT)    /* 60mV hysteresis */
#  define COMP_CSR_HYST_70MV        (7 << COMP_CSR_HYST_SHIFT)    /* 70mV hysteresis */

#define COMP_CSR_BLANKING_SHIFT     (19)                           /* Bit 21-19: Comparator blanking signal select */
#define COMP_CSR_BLANKING_MASK      (7 << COMP_CSR_BLANKING_SHIFT)
#  define COMP_CSR_BLANKING_DIS     (0 << COMP_CSR_BLANKING_SHIFT) /* 000: No blanking */
#  define COMP1_CSR_BLANKING_T1OC5  (1 << COMP_CSR_BLANKING_SHIFT) /* 001: TIM1_OC5 */
#  define COMP2_CSR_BLANKING_T1OC5  (1 << COMP_CSR_BLANKING_SHIFT) /* 001: TIM1_OC5 */
#  define COMP3_CSR_BLANKING_T1OC5  (1 << COMP_CSR_BLANKING_SHIFT) /* 001: TIM1_OC5 */
#  define COMP4_CSR_BLANKING_T3OC4  (1 << COMP_CSR_BLANKING_SHIFT) /* 001: TIM3_OC4 */
#  define COMP5_CSR_BLANKING_T2OC3  (1 << COMP_CSR_BLANKING_SHIFT) /* 001: TIM2_OC3 */
#  define COMP6_CSR_BLANKING_T8OC5  (1 << COMP_CSR_BLANKING_SHIFT) /* 001: TIM8_OC5 */
#  define COMP7_CSR_BLANKING_T1OC5  (1 << COMP_CSR_BLANKING_SHIFT) /* 001: TIM1_OC5 */
#  define COMP1_CSR_BLANKING_T2OC3  (2 << COMP_CSR_BLANKING_SHIFT) /* 010: TIM2_OC3 */
#  define COMP2_CSR_BLANKING_T2OC3  (2 << COMP_CSR_BLANKING_SHIFT) /* 010: TIM2_OC3 */
#  define COMP3_CSR_BLANKING_T3OC3  (2 << COMP_CSR_BLANKING_SHIFT) /* 010: TIM3_OC3 */
#  define COMP4_CSR_BLANKING_T8OC5  (2 << COMP_CSR_BLANKING_SHIFT) /* 010: TIM8_OC5 */
#  define COMP5_CSR_BLANKING_T8OC5  (2 << COMP_CSR_BLANKING_SHIFT) /* 010: TIM8_OC5 */
#  define COMP6_CSR_BLANKING_T2OC4  (2 << COMP_CSR_BLANKING_SHIFT) /* 010: TIM2_OC4 */
#  define COMP7_CSR_BLANKING_T8OC5  (2 << COMP_CSR_BLANKING_SHIFT) /* 010: TIM8_OC5 */
#  define COMP1_CSR_BLANKING_T3OC3  (3 << COMP_CSR_BLANKING_SHIFT) /* 011: TIM3_OC3 */
#  define COMP2_CSR_BLANKING_T3OC3  (3 << COMP_CSR_BLANKING_SHIFT) /* 011: TIM3_OC3 */
#  define COMP3_CSR_BLANKING_T2OC4  (3 << COMP_CSR_BLANKING_SHIFT) /* 011: TIM2_OC4 */
#  define COMP4_CSR_BLANKING_T15OC1 (3 << COMP_CSR_BLANKING_SHIFT) /* 011 or 110: TIM15_OC1 */
#  define COMP5_CSR_BLANKING_T3OC3  (3 << COMP_CSR_BLANKING_SHIFT) /* 011: TIM3_OC3 */
#  define COMP6_CSR_BLANKING_T15OC2 (3 << COMP_CSR_BLANKING_SHIFT) /* 011: TIM15_OC2 */
#  define COMP7_CSR_BLANKING_T3OC3  (3 << COMP_CSR_BLANKING_SHIFT) /* 011: TIM3_OC3 */
#  define COMP1_CSR_BLANKING_T8OC5  (4 << COMP_CSR_BLANKING_SHIFT) /* 100: TIM8_OC5 */
#  define COMP2_CSR_BLANKING_T8OC5  (4 << COMP_CSR_BLANKING_SHIFT) /* 100: TIM8_OC5 */
#  define COMP3_CSR_BLANKING_T8OC5  (4 << COMP_CSR_BLANKING_SHIFT) /* 100: TIM8_OC5 */
#  define COMP4_CSR_BLANKING_T1OC5  (4 << COMP_CSR_BLANKING_SHIFT) /* 100: TIM1_OC5 */
#  define COMP5_CSR_BLANKING_T1OC5  (4 << COMP_CSR_BLANKING_SHIFT) /* 100: TIM1_OC5 */
#  define COMP6_CSR_BLANKING_T1OC5  (4 << COMP_CSR_BLANKING_SHIFT) /* 100: TIM1_OC5 */
#  define COMP7_CSR_BLANKING_T15OC2 (4 << COMP_CSR_BLANKING_SHIFT) /* 100: TIM15_OC2 */
#  define COMP1_CSR_BLANKING_T20OC5 (5 << COMP_CSR_BLANKING_SHIFT) /* 101: TIM20_OC5 */
#  define COMP2_CSR_BLANKING_T20OC5 (5 << COMP_CSR_BLANKING_SHIFT) /* 101: TIM20_OC5 */
#  define COMP3_CSR_BLANKING_T20OC5 (5 << COMP_CSR_BLANKING_SHIFT) /* 101: TIM20_OC5 */
#  define COMP4_CSR_BLANKING_T20OC5 (5 << COMP_CSR_BLANKING_SHIFT) /* 101: TIM20_OC5 */
#  define COMP5_CSR_BLANKING_T20OC5 (5 << COMP_CSR_BLANKING_SHIFT) /* 101: TIM20_OC5 */
#  define COMP6_CSR_BLANKING_T20OC5 (5 << COMP_CSR_BLANKING_SHIFT) /* 101: TIM20_OC5 */
#  define COMP7_CSR_BLANKING_T20OC5 (5 << COMP_CSR_BLANKING_SHIFT) /* 101: TIM20_OC5 */
#  define COMP1_CSR_BLANKING_T15OC1 (6 << COMP_CSR_BLANKING_SHIFT) /* 110: TIM15_OC1 */
#  define COMP2_CSR_BLANKING_T15OC1 (6 << COMP_CSR_BLANKING_SHIFT) /* 110: TIM15_OC1 */
#  define COMP3_CSR_BLANKING_T15OC1 (6 << COMP_CSR_BLANKING_SHIFT) /* 110: TIM15_OC1 */
#  define COMP5_CSR_BLANKING_T15OC1 (6 << COMP_CSR_BLANKING_SHIFT) /* 110: TIM15_OC1 */
#  define COMP6_CSR_BLANKING_T15OC1 (6 << COMP_CSR_BLANKING_SHIFT) /* 110: TIM15_OC1 */
#  define COMP7_CSR_BLANKING_T15OC1 (6 << COMP_CSR_BLANKING_SHIFT) /* 110: TIM15_OC1 */
#  define COMP1_CSR_BLANKING_T4OC3  (6 << COMP_CSR_BLANKING_SHIFT) /* 111: TIM4_OC3 */
#  define COMP2_CSR_BLANKING_T4OC3  (6 << COMP_CSR_BLANKING_SHIFT) /* 111: TIM4_OC3 */
#  define COMP3_CSR_BLANKING_T4OC3  (6 << COMP_CSR_BLANKING_SHIFT) /* 111: TIM4_OC3 */
#  define COMP4_CSR_BLANKING_T4OC3  (6 << COMP_CSR_BLANKING_SHIFT) /* 111: TIM4_OC3 */
#  define COMP5_CSR_BLANKING_T4OC3  (6 << COMP_CSR_BLANKING_SHIFT) /* 111: TIM4_OC3 */
#  define COMP6_CSR_BLANKING_T4OC3  (6 << COMP_CSR_BLANKING_SHIFT) /* 111: TIM4_OC3 */
#  define COMP7_CSR_BLANKING_T4OC3  (6 << COMP_CSR_BLANKING_SHIFT) /* 111: TIM4_OC3 */

#define COMP_CSR_BRGEN              (1 << 22)                     /* Bit 22: Scaler resistor bridge enable */
#define COMP_CSR_SCALEN             (1 << 23)                     /* Bit 22: scaler enable */
                                                                  /* Bits 29-24: Reserved */
#define COMP_CSR_VALUE              (1 << 30)                     /* Bit 30: Comparator output status */
#define COMP_CSR_LOCK               (1 << 31)                     /* Bit 31: Register lock */

#endif                          /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_COMP_H */
