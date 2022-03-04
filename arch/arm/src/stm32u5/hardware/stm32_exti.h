/****************************************************************************
 * arch/arm/src/stm32u5/hardware/stm32_exti.h
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

#ifndef __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_EXTI_H
#define __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_EXTI_RTSR1_OFFSET      0x0000  /* Rising Trigger Selection 1       */
#define STM32_EXTI_FTSR1_OFFSET      0x0004  /* Falling Trigger Selection 1      */
#define STM32_EXTI_SWIER1_OFFSET     0x0008  /* Software Interrupt Event 1       */
#define STM32_EXTI_RPR1_OFFSET       0x000c  /* Rising Edge Pending 1            */
#define STM32_EXTI_FPR1_OFFSET       0x0010  /* Falling Edge Pending 1           */
#define STM32_EXTI_SECCFGR1_OFFSET   0x0014  /* Security Configuration 1         */
#define STM32_EXTI_PRIVCFGR1_OFFSET  0x0018  /* Privilege Configuration 1        */
#define STM32_EXTI_RTSR2_OFFSET      0x0020  /* Rising Trigger Selection 2       */
#define STM32_EXTI_FTSR2_OFFSET      0x0024  /* Falling Trigger Selection 2      */
#define STM32_EXTI_SWIER2_OFFSET     0x0028  /* Software Interrupt Event 2       */
#define STM32_EXTI_RPR2_OFFSET       0x002c  /* Rising Edge Pending 2            */
#define STM32_EXTI_FPR2_OFFSET       0x0030  /* Falling Edge Pending 2           */
#define STM32_EXTI_SECCFGR2_OFFSET   0x0034  /* Security Configuration 2         */
#define STM32_EXTI_PRIVCFGR2_OFFSET  0x0038  /* Privilege Configuration 2        */
#define STM32_EXTI_EXTICR1_OFFSET    0x0060  /* External Interrupt Selection 1   */
#define STM32_EXTI_EXTICR2_OFFSET    0x0060  /* External Interrupt Selection 2   */
#define STM32_EXTI_EXTICR3_OFFSET    0x0060  /* External Interrupt Selection 3   */
#define STM32_EXTI_EXTICR4_OFFSET    0x0060  /* External Interrupt Selection 4   */
#define STM32_EXTI_LOCKR_OFFSET      0x0070  /* Lock                             */
#define STM32_EXTI_IMR1_OFFSET       0x0080  /* CPU Wakeup with Interrupt Mask 1 */
#define STM32_EXTI_EMR1_OFFSET       0x0084  /* CPU Wakeup with Event Mask 1     */
#define STM32_EXTI_IMR2_OFFSET       0x0090  /* CPU Wakeup with Interrupt Mask 2 */
#define STM32_EXTI_EMR2_OFFSET       0x0094  /* CPU Wakeup with Event Mask 2     */

/* Register Addresses *******************************************************/

#define STM32_EXTI_RTSR1      (STM32_EXTI_BASE + STM32_EXTI_RTSR1_OFFSET)
#define STM32_EXTI_FTSR1      (STM32_EXTI_BASE + STM32_EXTI_FTSR1_OFFSET)
#define STM32_EXTI_SWIER1     (STM32_EXTI_BASE + STM32_EXTI_SWIER1_OFFSET)
#define STM32_EXTI_RPR1       (STM32_EXTI_BASE + STM32_EXTI_RPR1_OFFSET)
#define STM32_EXTI_FPR1       (STM32_EXTI_BASE + STM32_EXTI_FPR1_OFFSET)
#define STM32_EXTI_SECCFGR1   (STM32_EXTI_BASE + STM32_EXTI_SECCFGR1_OFFSET)
#define STM32_EXTI_PRIVCFGR1  (STM32_EXTI_BASE + STM32_EXTI_PRIVCFGR1_OFFSET)
#define STM32_EXTI_RTSR2      (STM32_EXTI_BASE + STM32_EXTI_RTSR2_OFFSET)
#define STM32_EXTI_FTSR2      (STM32_EXTI_BASE + STM32_EXTI_FTSR2_OFFSET)
#define STM32_EXTI_SWIER2     (STM32_EXTI_BASE + STM32_EXTI_SWIER2_OFFSET)
#define STM32_EXTI_RPR2       (STM32_EXTI_BASE + STM32_EXTI_RPR2_OFFSET)
#define STM32_EXTI_FPR2       (STM32_EXTI_BASE + STM32_EXTI_FPR2_OFFSET)
#define STM32_EXTI_SECCFGR2   (STM32_EXTI_BASE + STM32_EXTI_SECCFGR2_OFFSET)
#define STM32_EXTI_PRIVCFGR2  (STM32_EXTI_BASE + STM32_EXTI_PRIVCFGR2_OFFSET)
#define STM32_EXTI_EXTICR1    (STM32_EXTI_BASE + STM32_EXTI_EXTICR1_OFFSET)
#define STM32_EXTI_EXTICR2    (STM32_EXTI_BASE + STM32_EXTI_EXTICR2_OFFSET)
#define STM32_EXTI_EXTICR3    (STM32_EXTI_BASE + STM32_EXTI_EXTICR3_OFFSET)
#define STM32_EXTI_EXTICR4    (STM32_EXTI_BASE + STM32_EXTI_EXTICR4_OFFSET)
#define STM32_EXTI_LOCKR      (STM32_EXTI_BASE + STM32_EXTI_LOCKR_OFFSET)
#define STM32_EXTI_IMR1       (STM32_EXTI_BASE + STM32_EXTI_IMR1_OFFSET)
#define STM32_EXTI_EMR1       (STM32_EXTI_BASE + STM32_EXTI_EMR1_OFFSET)
#define STM32_EXTI_IMR2       (STM32_EXTI_BASE + STM32_EXTI_IMR2_OFFSET)
#define STM32_EXTI_EMR2       (STM32_EXTI_BASE + STM32_EXTI_EMR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* EXTI lines > 15 are associated with internal devices: */

#define EXTI1_PVD          (1 << 16)  /* EXTI line 16: PVD output     */
#define EXTI1_RTC          (1 << 17)  /* EXTI line 17: RTC            */
#define EXTI1_RTC_SECURE   (1 << 18)  /* EXTI line 18: RTC secure     */
#define EXTI1_TAMP         (1 << 19)  /* EXTI line 19: TAMP           */
#define EXTI1_TAMP_SECURE  (1 << 20)  /* EXTI line 20: TAMP secure    */
#define EXTI1_COMP1        (1 << 21)  /* EXTI line 21: COMP1 output   */
#define EXTI1_COMP2        (1 << 22)  /* EXTI line 22: COMP2 output   */
#define EXTI1_I2C1         (1 << 23)  /* EXTI line 23: I2C1 wakeup    */
#define EXTI1_I2C2         (1 << 24)  /* EXTI line 24: I2C2 wakeup    */
#define EXTI1_I2C3         (1 << 25)  /* EXTI line 25: I2C3 wakeup    */
#define EXTI1_USART1       (1 << 26)  /* EXTI line 26: USART1 wakeup  */
#define EXTI1_USART2       (1 << 27)  /* EXTI line 27: USART2 wakeup  */
#define EXTI1_USART3       (1 << 28)  /* EXTI line 28: USART3 wakeup  */
#define EXTI1_USART4       (1 << 29)  /* EXTI line 29: USART4 wakeup  */
#define EXTI1_USART5       (1 << 30)  /* EXTI line 30: USART5 wakeup  */
#define EXTI1_LPUART1      (1 << 31)  /* EXTI line 31: LPUART1 wakeup */
#define EXTI2_LPTIM1       (1 <<  0)  /* EXTI line 32: LPTIM1         */
#define EXTI2_LPTIM2       (1 <<  1)  /* EXTI line 33: LPTIM2         */
#define EXTI2_USBFS        (1 <<  2)  /* EXTI line 34: USB FS wakeup  */
#define EXTI2_PVM1         (1 <<  3)  /* EXTI line 35: PVM1 wakeup    */
#define EXTI2_PVM2         (1 <<  4)  /* EXTI line 36: PVM2 wakeup    */
#define EXTI2_PVM3         (1 <<  5)  /* EXTI line 37: PVM3 wakeup    */
#define EXTI2_PVM4         (1 <<  6)  /* EXTI line 38: PVM4 wakeup    */
#define EXTI2_RSVD         (1 <<  7)  /* EXTI line 39: reserved       */
#define EXTI2_I2C4         (1 <<  8)  /* EXTI line 40: I2C4 wakeup    */
#define EXTI2_UCPD1        (1 <<  9)  /* EXTI line 41: UCPD1 wakeup   */
#define EXTI2_LPTIM3       (1 << 10)  /* EXTI line 42: LPTIM3 wakeup  */

#endif /* __ARCH_ARM_SRC_STM32U5_HARDWARE_STM32_EXTI_H */
