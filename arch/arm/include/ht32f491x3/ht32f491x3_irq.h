/****************************************************************************
 * arch/arm/include/ht32f491x3/ht32f491x3_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_HT32F491X3_HT32F491X3_IRQ_H
#define __ARCH_ARM_INCLUDE_HT32F491X3_HT32F491X3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* External interrupts.  These values follow the Holtek CMSIS device header
 * and startup vector table for the HT32F491x3 family.
 */

#define HT32_IRQ_WWDT          (HT32_IRQ_FIRST + 0)
#define HT32_IRQ_PVM           (HT32_IRQ_FIRST + 1)
#define HT32_IRQ_PVD           HT32_IRQ_PVM
#define HT32_IRQ_TAMP_STAMP    (HT32_IRQ_FIRST + 2)
#define HT32_IRQ_TAMPER        HT32_IRQ_TAMP_STAMP
#define HT32_IRQ_TIMESTAMP     HT32_IRQ_TAMP_STAMP
#define HT32_IRQ_ERTCWAKEUP    (HT32_IRQ_FIRST + 3)
#define HT32_IRQ_FLASH         (HT32_IRQ_FIRST + 4)
#define HT32_IRQ_CRM           (HT32_IRQ_FIRST + 5)
#define HT32_IRQ_EXINT0        (HT32_IRQ_FIRST + 6)
#define HT32_IRQ_EXINT1        (HT32_IRQ_FIRST + 7)
#define HT32_IRQ_EXINT2        (HT32_IRQ_FIRST + 8)
#define HT32_IRQ_EXINT3        (HT32_IRQ_FIRST + 9)
#define HT32_IRQ_EXINT4        (HT32_IRQ_FIRST + 10)
#define HT32_IRQ_DMA1CH1       (HT32_IRQ_FIRST + 11)
#define HT32_IRQ_DMA1CH2       (HT32_IRQ_FIRST + 12)
#define HT32_IRQ_DMA1CH3       (HT32_IRQ_FIRST + 13)
#define HT32_IRQ_DMA1CH4       (HT32_IRQ_FIRST + 14)
#define HT32_IRQ_DMA1CH5       (HT32_IRQ_FIRST + 15)
#define HT32_IRQ_DMA1CH6       (HT32_IRQ_FIRST + 16)
#define HT32_IRQ_DMA1CH7       (HT32_IRQ_FIRST + 17)
#define HT32_IRQ_ADC1          (HT32_IRQ_FIRST + 18)
#define HT32_IRQ_ADC           HT32_IRQ_ADC1
#define HT32_IRQ_CAN1TX        (HT32_IRQ_FIRST + 19)
#define HT32_IRQ_CAN1RX0       (HT32_IRQ_FIRST + 20)
#define HT32_IRQ_CAN1RX1       (HT32_IRQ_FIRST + 21)
#define HT32_IRQ_CAN1SE        (HT32_IRQ_FIRST + 22)
#define HT32_IRQ_CANSE         HT32_IRQ_CAN1SE
#define HT32_IRQ_EXINT95       (HT32_IRQ_FIRST + 23)
#define HT32_IRQ_TMR1BRK       (HT32_IRQ_FIRST + 24)
#define HT32_IRQ_TMR9          HT32_IRQ_TMR1BRK
#define HT32_IRQ_TMR1UP        (HT32_IRQ_FIRST + 25)
#define HT32_IRQ_TMR10         HT32_IRQ_TMR1UP
#define HT32_IRQ_TMR1TRGCOM    (HT32_IRQ_FIRST + 26)
#define HT32_IRQ_TMR11         HT32_IRQ_TMR1TRGCOM
#define HT32_IRQ_TMR1CC        (HT32_IRQ_FIRST + 27)
#define HT32_IRQ_TMR2          (HT32_IRQ_FIRST + 28)
#define HT32_IRQ_TMR3          (HT32_IRQ_FIRST + 29)
#define HT32_IRQ_TMR4          (HT32_IRQ_FIRST + 30)
#define HT32_IRQ_I2C1EV        (HT32_IRQ_FIRST + 31)
#define HT32_IRQ_I2C1ERR       (HT32_IRQ_FIRST + 32)
#define HT32_IRQ_I2C2EV        (HT32_IRQ_FIRST + 33)
#define HT32_IRQ_I2C2ERR       (HT32_IRQ_FIRST + 34)
#define HT32_IRQ_SPI1          (HT32_IRQ_FIRST + 35)
#define HT32_IRQ_SPI2          (HT32_IRQ_FIRST + 36)
#define HT32_IRQ_USART1        (HT32_IRQ_FIRST + 37)
#define HT32_IRQ_USART2        (HT32_IRQ_FIRST + 38)
#define HT32_IRQ_USART3        (HT32_IRQ_FIRST + 39)
#define HT32_IRQ_EXINT1510     (HT32_IRQ_FIRST + 40)
#define HT32_IRQ_ERTCALARM     (HT32_IRQ_FIRST + 41)
#define HT32_IRQ_OTGFSWKUP     (HT32_IRQ_FIRST + 42)
#define HT32_IRQ_TMR12         (HT32_IRQ_FIRST + 43)
#define HT32_IRQ_TMR13         (HT32_IRQ_FIRST + 44)
#define HT32_IRQ_TMR14         (HT32_IRQ_FIRST + 45)
#define HT32_IRQ_SPI3          (HT32_IRQ_FIRST + 51)
#define HT32_IRQ_USART4        (HT32_IRQ_FIRST + 52)
#define HT32_IRQ_USART5        (HT32_IRQ_FIRST + 53)
#define HT32_IRQ_TMR6          (HT32_IRQ_FIRST + 54)
#define HT32_IRQ_DAC           HT32_IRQ_TMR6
#define HT32_IRQ_TMR7          (HT32_IRQ_FIRST + 55)
#define HT32_IRQ_DMA2CH1       (HT32_IRQ_FIRST + 56)
#define HT32_IRQ_DMA2CH2       (HT32_IRQ_FIRST + 57)
#define HT32_IRQ_DMA2CH3       (HT32_IRQ_FIRST + 58)
#define HT32_IRQ_DMA2CH4       (HT32_IRQ_FIRST + 59)
#define HT32_IRQ_DMA2CH5       (HT32_IRQ_FIRST + 60)
#define HT32_IRQ_CAN2TX        (HT32_IRQ_FIRST + 63)
#define HT32_IRQ_CAN2RX0       (HT32_IRQ_FIRST + 64)
#define HT32_IRQ_CAN2RX1       (HT32_IRQ_FIRST + 65)
#define HT32_IRQ_CAN2SE        (HT32_IRQ_FIRST + 66)
#define HT32_IRQ_OTGFS1        (HT32_IRQ_FIRST + 67)
#define HT32_IRQ_OTGFS         HT32_IRQ_OTGFS1
#define HT32_IRQ_DMA2CH6       (HT32_IRQ_FIRST + 68)
#define HT32_IRQ_DMA2CH7       (HT32_IRQ_FIRST + 69)
#define HT32_IRQ_USART6        (HT32_IRQ_FIRST + 71)
#define HT32_IRQ_I2C3EV        (HT32_IRQ_FIRST + 72)
#define HT32_IRQ_I2C3ERR       (HT32_IRQ_FIRST + 73)
#define HT32_IRQ_FPU           (HT32_IRQ_FIRST + 81)
#define HT32_IRQ_USART7        (HT32_IRQ_FIRST + 82)
#define HT32_IRQ_USART8        (HT32_IRQ_FIRST + 83)
#define HT32_IRQ_DMAMUX        (HT32_IRQ_FIRST + 94)
#define HT32_IRQ_ACC           (HT32_IRQ_FIRST + 103)

#define HT32_IRQ_NEXTINT       (104)
#define NR_IRQS                (HT32_IRQ_FIRST + HT32_IRQ_NEXTINT)

#endif /* __ARCH_ARM_INCLUDE_HT32F491X3_HT32F491X3_IRQ_H */
