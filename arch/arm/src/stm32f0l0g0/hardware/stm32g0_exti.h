/************************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32g0_exti.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_EXTI_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_EXTI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define STM32_NEXTI              18
#define STM32_EXTI_MASK          0xffffffff

#define STM32_EXTI_BIT(n)        (1 << (n))

/* Register Offsets *****************************************************************/

#define STM32_EXTI_RTSR1_OFFSET     0x0000                    /* Rising Trigger selection register 1 */
#define STM32_EXTI_FTSR1_OFFSET     0x0004                    /* Falling Trigger selection register 1 */
#define STM32_EXTI_SWIER1_OFFSET    0x0008                    /* Software interrupt event register 1 */
#define STM32_EXTI_RPR1_OFFSET      0x000c                    /* Rising edge pending register 1 */
#define STM32_EXTI_FPR1_OFFSET      0x0010                    /* Falling edge pending register 1 */

#define STM32_EXTI_EXTICR_OFFSET(p) (0x0060 + ((p) & 0x000c)) /* Registers are displaced by 4! */
#define STM32_EXTI_EXTICR1_OFFSET   0x0060                    /* External interrupt selection register 1 */
#define STM32_EXTI_EXTICR2_OFFSET   0x0064                    /* External interrupt selection register 2 */
#define STM32_EXTI_EXTICR3_OFFSET   0x0068                    /* External interrupt selection register 3 */
#define STM32_EXTI_EXTICR4_OFFSET   0x006c                    /* External interrupt selection register 4 */
#define STM32_EXTI_IMR1_OFFSET      0x0080                    /* CPU wakeup with interrupt mask register 1 */
#define STM32_EXTI_EMR1_OFFSET      0x0084                    /* CPU wakeup with event mask register 1 */
#define STM32_EXTI_IMR2_OFFSET      0x0090                    /* CPU wakeup with interrupt mask register 2 */
#define STM32_EXTI_EMR2_OFFSET      0x0094                    /* CPU wakeup with event mask register 2 */

/* Register Addresses ***************************************************************/

#define STM32_EXTI_RTSR1            (STM32_EXTI_BASE + STM32_EXTI_RTSR1_OFFSET)
#define STM32_EXTI_FTSR1            (STM32_EXTI_BASE + STM32_EXTI_FTSR1_OFFSET)
#define STM32_EXTI_SWIER1           (STM32_EXTI_BASE + STM32_EXTI_SWIER1_OFFSET)
#define STM32_EXTI_RPR1             (STM32_EXTI_BASE + STM32_EXTI_RPR1_OFFSET)
#define STM32_EXTI_FPR1             (STM32_EXTI_BASE + STM32_EXTI_FPR1_OFFSET)
#define STM32_EXTI_EXTICR(p)        (STM32_EXTI_BASE + STM32_EXTI_EXTICR_OFFSET(p))
#define STM32_EXTI_IMR1             (STM32_EXTI_BASE + STM32_EXTI_IMR1_OFFSET)
#define STM32_EXTI_EMR1             (STM32_EXTI_BASE + STM32_EXTI_EMR1_OFFSET)
#define STM32_EXTI_IMR2             (STM32_EXTI_BASE + STM32_EXTI_IMR2_OFFSET)
#define STM32_EXTI_EMR2             (STM32_EXTI_BASE + STM32_EXTI_EMR2_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* EXTI external interrupt configuration register 1-4 */

#define EXTI_EXTICR_PORTA           (0)       /* 0000: PA[x] pin */
#define EXTI_EXTICR_PORTB           (1)       /* 0001: PB[x] pin */
#define EXTI_EXTICR_PORTC           (2)       /* 0010: PC[x] pin */
#define EXTI_EXTICR_PORTD           (3)       /* 0011: PD[x] pin */
                                              /* 0100: Reserved */
#define EXTI_EXTICR_PORTF           (5)       /* 0100: PF[x] pin */

#define EXTI_EXTICR_PORT_MASK       (0xff)
#define EXTI_EXTICR_EXTI_SHIFT(g)   (((g) & 3) << 3)
#define EXTI_EXTICR_EXTI_MASK(g)    (EXTI_EXTICR_PORT_MASK << (EXTI_EXTICR_EXTI_SHIFT(g)))

/* TODO */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_EXTI_H */
