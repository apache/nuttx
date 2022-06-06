/****************************************************************************
 * arch/arm/src/stm32wl5/hardware/stm32wl5_exti.h
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

#ifndef __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_EXTI_H
#define __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WL5_EXTI_RTSR1_OFFSET      0x0000  /* Rising Trigger Selection 1       */
#define STM32WL5_EXTI_FTSR1_OFFSET      0x0004  /* Falling Trigger Selection 1      */
#define STM32WL5_EXTI_SWIER1_OFFSET     0x0008  /* Software Interrupt Event 1       */
#define STM32WL5_EXTI_PR1_OFFSET        0x000c  /* Pending 1                        */
#define STM32WL5_EXTI_RTSR2_OFFSET      0x0020  /* Rising Trigger Selection 2       */
#define STM32WL5_EXTI_FTSR2_OFFSET      0x0024  /* Falling Trigger Selection 2      */
#define STM32WL5_EXTI_SWIER2_OFFSET     0x0028  /* Software Interrupt Event 2       */
#define STM32WL5_EXTI_PR2_OFFSET        0x002c  /* Pending 2                        */
#define STM32WL5_EXTI_C1IMR1_OFFSET     0x0080  /* CPU Wakeup with Interrupt Mask 1 for cpu1 */
#define STM32WL5_EXTI_C1EMR1_OFFSET     0x0084  /* CPU Wakeup with Event Mask 1 for cpu1 */
#define STM32WL5_EXTI_C1IMR2_OFFSET     0x0090  /* CPU Wakeup with Interrupt Mask 2 for cpu1 */
#define STM32WL5_EXTI_C1EMR2_OFFSET     0x0094  /* CPU Wakeup with Event Mask 2 for cpu1 */
#define STM32WL5_EXTI_C2IMR1_OFFSET     0x00c0  /* CPU Wakeup with Interrupt Mask 1 for cpu2 */
#define STM32WL5_EXTI_C2EMR1_OFFSET     0x00c4  /* CPU Wakeup with Event Mask 1 for cpu2 */
#define STM32WL5_EXTI_C2IMR2_OFFSET     0x00d0  /* CPU Wakeup with Interrupt Mask 2 for cpu2 */
#define STM32WL5_EXTI_C2EMR2_OFFSET     0x00d4  /* CPU Wakeup with Event Mask 2 for cpu2 */

/* Register Addresses *******************************************************/

#define STM32WL5_EXTI_RTSR1      (STM32WL5_EXTI_BASE+STM32WL5_EXTI_RTSR1_OFFSET)
#define STM32WL5_EXTI_FTSR1      (STM32WL5_EXTI_BASE+STM32WL5_EXTI_FTSR1_OFFSET)
#define STM32WL5_EXTI_SWIER1     (STM32WL5_EXTI_BASE+STM32WL5_EXTI_SWIER1_OFFSET)
#define STM32WL5_EXTI_PR1        (STM32WL5_EXTI_BASE+STM32WL5_EXTI_PR1_OFFSET)
#define STM32WL5_EXTI_RTSR2      (STM32WL5_EXTI_BASE+STM32WL5_EXTI_RTSR2_OFFSET)
#define STM32WL5_EXTI_FTSR2      (STM32WL5_EXTI_BASE+STM32WL5_EXTI_FTSR2_OFFSET)
#define STM32WL5_EXTI_SWIER2     (STM32WL5_EXTI_BASE+STM32WL5_EXTI_SWIER2_OFFSET)
#define STM32WL5_EXTI_PR2        (STM32WL5_EXTI_BASE+STM32WL5_EXTI_PR2_OFFSET)
#define STM32WL5_EXTI_C1IMR1     (STM32WL5_EXTI_BASE+STM32WL5_EXTI_C1IMR1_OFFSET)
#define STM32WL5_EXTI_C1EMR1     (STM32WL5_EXTI_BASE+STM32WL5_EXTI_C1EMR1_OFFSET)
#define STM32WL5_EXTI_C1IMR2     (STM32WL5_EXTI_BASE+STM32WL5_EXTI_C1IMR2_OFFSET)
#define STM32WL5_EXTI_C1EMR2     (STM32WL5_EXTI_BASE+STM32WL5_EXTI_C1EMR2_OFFSET)
#define STM32WL5_EXTI_C2IMR1     (STM32WL5_EXTI_BASE+STM32WL5_EXTI_C2IMR1_OFFSET)
#define STM32WL5_EXTI_C2EMR1     (STM32WL5_EXTI_BASE+STM32WL5_EXTI_C2EMR1_OFFSET)
#define STM32WL5_EXTI_C2IMR2     (STM32WL5_EXTI_BASE+STM32WL5_EXTI_C2IMR2_OFFSET)
#define STM32WL5_EXTI_C2EMR2     (STM32WL5_EXTI_BASE+STM32WL5_EXTI_C2EMR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* EXTI lines > 15 are associated with internal devices: */

#define EXTI1_PVD          (1 << 16)  /* EXTI line 16: PVD output     */
#define EXTI1_RTC          (1 << 17)  /* EXTI line 17: RTC            */
#define EXTI1_SSRU         (1 << 18)  /* EXTI line 18: RTC underflow  */
#define EXTI1_TAMP         (1 << 19)  /* EXTI line 19: TAMP           */
#define EXTI1_WKUP         (1 << 20)  /* EXTI line 20: RTC wakeup     */
#define EXTI1_COMP1        (1 << 21)  /* EXTI line 21: COMP1 output   */
#define EXTI1_COMP2        (1 << 22)  /* EXTI line 22: COMP2 output   */
#define EXTI1_I2C1         (1 << 23)  /* EXTI line 23: I2C1 wakeup    */
#define EXTI1_I2C2         (1 << 24)  /* EXTI line 24: I2C2 wakeup    */
#define EXTI1_I2C3         (1 << 25)  /* EXTI line 25: I2C3 wakeup    */
#define EXTI1_USART1       (1 << 26)  /* EXTI line 26: USART1 wakeup  */
#define EXTI1_USART2       (1 << 27)  /* EXTI line 27: USART2 wakeup  */
#define EXTI1_LPUART1      (1 << 28)  /* EXTI line 28: LPUART1 wakeup */
#define EXTI1_LPTIM1       (1 << 29)  /* EXTI line 29: LPTIM1         */
#define EXTI1_LPTIM2       (1 << 30)  /* EXTI line 30: LPTIM2         */
#define EXTI1_LPTIM3       (1 << 31)  /* EXTI line 31: LPTIM3         */
#define EXTI2_PVM3         (1 <<  3)  /* EXTI line 34: PVM3 wakeup    */
#define EXTI2_C1IPCC       (1 <<  5)  /* EXTI line 36: IPCC cpu1 RX occupied */
#define EXTI2_C2IPCC       (1 <<  6)  /* EXTI line 37: IPCC cpu1 RX occupied */
#define EXTI2_C1HSEM       (1 <<  7)  /* EXTI line 38: Semaphore irq 0 with cpu1 */
#define EXTI2_C2HSEM       (1 <<  8)  /* EXTI line 39: Semaphore irq 1 with cpu2 */
#define EXTI2_C2SEV        (1 <<  9)  /* EXTI line 40: CPU2 SEV line */
#define EXTI2_C1SEV        (1 << 10)  /* EXTI line 41: CPU1 SEV line */
#define EXTI2_FLASH        (1 << 11)  /* EXTI line 42: Flash ECC */
#define EXTI2_HSE32CSS     (1 << 12)  /* EXTI line 43: RCC HSE32 CSS interrupt */
#define EXTI2_RADIOIRQ     (1 << 13)  /* EXTI line 44: Radio interrupt */
#define EXTI2_RADIOBSY     (1 << 14)  /* EXTI line 45: Radio busy wakeup */
#define EXTI2_CDBGPWRUPREQ (1 << 15)  /* EXTI line 46: Debug power-up request wakup */

#endif /* __ARCH_ARM_SRC_STM32WL5_HARDWARE_STM32WL5_EXTI_H */
