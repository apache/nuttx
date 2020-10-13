/****************************************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32l0_syscfg.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32L0_SYSCFG_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32L0_SYSCFG_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define STM32_SYSCFG_CFGR1_OFFSET      0x0000                    /* SYSCFG configuration register 1 */
#define STM32_SYSCFG_CFGR2_OFFSET      0x0004                    /* SYSCFG configuration register 2 */
#define STM32_SYSCFG_CFGR3_OFFSET      0x0020                    /* SYSCFG configuration register 3 */

#define STM32_SYSCFG_EXTICR_OFFSET(p)  (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */
#define STM32_SYSCFG_EXTICR1_OFFSET    0x0008                    /* SYSCFG external interrupt configuration register 1 */
#define STM32_SYSCFG_EXTICR2_OFFSET    0x000c                    /* SYSCFG external interrupt configuration register 2 */
#define STM32_SYSCFG_EXTICR3_OFFSET    0x0010                    /* SYSCFG external interrupt configuration register 3 */
#define STM32_SYSCFG_EXTICR4_OFFSET    0x0014                    /* SYSCFG external interrupt configuration register 4 */

#define STM32_SYSCFG_CFGR3_OFFSET      0x0020                    /* SYSCFG configuration register 3 */

/* Register Addresses *******************************************************************************/

#define STM32_SYSCFG_CFGR1             (STM32_SYSCFG_BASE+STM32_SYSCFG_CFGR1_OFFSET)
#define STM32_SYSCFG_CFGR2             (STM32_SYSCFG_BASE+STM32_SYSCFG_CFGR2_OFFSET)
#define STM32_SYSCFG_CFGR3             (STM32_SYSCFG_BASE+STM32_SYSCFG_CFGR3_OFFSET)

#define STM32_SYSCFG_EXTICR(p)         (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR_OFFSET(p))
#define STM32_SYSCFG_EXTICR1           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR1_OFFSET)
#define STM32_SYSCFG_EXTICR2           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR2_OFFSET)
#define STM32_SYSCFG_EXTICR3           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR3_OFFSET)
#define STM32_SYSCFG_EXTICR4           (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR4_OFFSET)

#define STM32_SYSCFG_CFGR3             (STM32_SYSCFG_BASE+STM32_SYSCFG_CFGR3_OFFSET)

/* Register Bitfield Definitions ********************************************************************/

/* TODO: SYSCFG configuration register 1 */

/* TODO: SYSCFG configuration register 2 */

/* TODO: SYSCFG configuration register 3 */

#define SYSCFG_CFGR3_ENVREFINT        (1 << 0)   /* Bit 0: VREFINT enable and scaler control for COMP2 enable */
                                                 /* Bits 1-3: Reserved */
#define SYSCFG_CFGR3_SELVREFOUT_SHIFT (4)        /* Bits 4-5: VREFINT_ADC connection */
#define SYSCFG_CFGR3_SELVREFOUT_MASK  (3 << SYSCFG_CFGR3_SELVREFOUT_SHIFT)
                                                 /* Bits 6-7: Reserved */
#define SYSCFG_CFGR3_ENBUFVREFADC     (1 << 8)   /* Bit 8: VREFINT reference for ADC enable */
#define SYSCFG_CFGR3_ENBUFSENSORADC   (1 << 9)   /* Bit 9: Temperature sensor reference for ADC enable */
                                                 /* Bits 10-11: Reserved */
#define SYSCFG_CFGR3_ENBUFVREFINTCOMP2 (1 << 12) /* Bit 12: VREFINT reference for COMP2 scaler enable */
#define SYSCFG_CFGR3_ENBUFVREFINTHSI48 (1 << 13) /* Bit 13: VREFINT reference for HSI48 scaler enable */
                                                 /* Bits 14-29: Reserved */
#define SYSCFG_CFGR3_VREFINTRDYF       (1 << 30) /* Bit 30: VREFINT ready flag */
#define SYSCFG_CFGR3_REFLOCK           (1 << 31) /* Bit 31: SYSCFG_CFG3 lock bit */

/* SYSCFG external interrupt configuration register 1-4 */

#define SYSCFG_EXTICR_PORTA           (0)       /* 0000: PA[x] pin */
#define SYSCFG_EXTICR_PORTB           (1)       /* 0001: PB[x] pin */
#define SYSCFG_EXTICR_PORTC           (2)       /* 0010: PC[x] pin */
#define SYSCFG_EXTICR_PORTD           (3)       /* 0011: PD[x] pin */
#define SYSCFG_EXTICR_PORTE           (4)       /* 0100: PE[x] pin */

#define SYSCFG_EXTICR_PORT_MASK       (15)
#define SYSCFG_EXTICR_EXTI_SHIFT(g)   (((g) & 3) << 2)
#define SYSCFG_EXTICR_EXTI_MASK(g)    (SYSCFG_EXTICR_PORT_MASK << (SYSCFG_EXTICR_EXTI_SHIFT(g)))

#define SYSCFG_EXTICR1_EXTI0_SHIFT    (0)       /* Bits 0-3: EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI0_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI0_SHIFT)
#define SYSCFG_EXTICR1_EXTI1_SHIFT    (4)       /* Bits 4-7: EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI1_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI1_SHIFT)
#define SYSCFG_EXTICR1_EXTI2_SHIFT    (8)       /* Bits 8-11: EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI2_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI2_SHIFT)
#define SYSCFG_EXTICR1_EXTI3_SHIFT    (12)      /* Bits 12-15: EXTI 3 configuration */
#define SYSCFG_EXTICR1_EXTI3_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI3_SHIFT)

#define SYSCFG_EXTICR2_EXTI4_SHIFT    (0)       /* Bits 0-3: EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI4_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI4_SHIFT)
#define SYSCFG_EXTICR2_EXTI5_SHIFT    (4)       /* Bits 4-7: EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI5_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI5_SHIFT)
#define SYSCFG_EXTICR2_EXTI6_SHIFT    (8)       /* Bits 8-11: EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI6_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI6_SHIFT)
#define SYSCFG_EXTICR2_EXTI7_SHIFT    (12)      /* Bits 12-15: EXTI 7 configuration */
#define SYSCFG_EXTICR2_EXTI7_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI7_SHIFT)

#define SYSCFG_EXTICR3_EXTI8_SHIFT    (0)       /* Bits 0-3: EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI8_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI8_SHIFT)
#define SYSCFG_EXTICR3_EXTI9_SHIFT    (4)       /* Bits 4-7: EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI9_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI9_SHIFT)
#define SYSCFG_EXTICR3_EXTI10_SHIFT   (8)       /* Bits 8-11: EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI10_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI10_SHIFT)
#define SYSCFG_EXTICR3_EXTI11_SHIFT   (12)      /* Bits 12-15: EXTI 11 configuration */
#define SYSCFG_EXTICR3_EXTI11_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI11_SHIFT)

#define SYSCFG_EXTICR4_EXTI12_SHIFT   (0)       /* Bits 0-3: EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI12_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI12_SHIFT)
#define SYSCFG_EXTICR4_EXTI13_SHIFT   (4)       /* Bits 4-7: EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI13_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI13_SHIFT)
#define SYSCFG_EXTICR4_EXTI14_SHIFT   (8)       /* Bits 8-11: EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI14_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI14_SHIFT)
#define SYSCFG_EXTICR4_EXTI15_SHIFT   (12)      /* Bits 12-15: EXTI 15 configuration */
#define SYSCFG_EXTICR4_EXTI15_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI15_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32L0_SYSCFG_H */
