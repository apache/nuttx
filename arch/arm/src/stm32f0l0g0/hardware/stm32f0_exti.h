/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32f0_exti.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F0_EXTI_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F0_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_NEXTI              31
#define STM32_EXTI_MASK          0xffffffff

#define STM32_EXTI_BIT(n)        (1 << (n))

/* Register Offsets *********************************************************/

#define STM32_EXTI_IMR_OFFSET    0x0000  /* Interrupt mask register */
#define STM32_EXTI_EMR_OFFSET    0x0004  /* Event mask register */
#define STM32_EXTI_RTSR_OFFSET   0x0008  /* Rising Trigger selection register */
#define STM32_EXTI_FTSR_OFFSET   0x000c  /* Falling Trigger selection register */
#define STM32_EXTI_SWIER_OFFSET  0x0010  /* Software interrupt event register */
#define STM32_EXTI_PR_OFFSET     0x0014  /* Pending register */

/* Register Addresses *******************************************************/

#define STM32_EXTI_IMR           (STM32_EXTI_BASE + STM32_EXTI_IMR_OFFSET)
#define STM32_EXTI_EMR           (STM32_EXTI_BASE + STM32_EXTI_EMR_OFFSET)
#define STM32_EXTI_RTSR          (STM32_EXTI_BASE + STM32_EXTI_RTSR_OFFSET)
#define STM32_EXTI_FTSR          (STM32_EXTI_BASE + STM32_EXTI_FTSR_OFFSET)
#define STM32_EXTI_SWIER         (STM32_EXTI_BASE + STM32_EXTI_SWIER_OFFSET)
#define STM32_EXTI_PR            (STM32_EXTI_BASE + STM32_EXTI_PR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* EXTI lines > 15 are associated with internal devices: */

#define EXTI_PVD_LINE            (1 << 16) /* EXTI line 16 is connected to the PVD output */
#define EXTI_RTC_ALARM           (1 << 17) /* EXTI line 17 is connected to the RTC Alarm event */
#define EXTI_USB_WAKEUP          (1 << 18) /* EXTI line 18 is connected to the USB wake up event */
#define EXTI_RTC_TAMPER          (1 << 19) /* EXTI line 19 is connected to the RTC Tamper and TimeStamp events */
#define EXTI_RTC_WAKEUP          (1 << 20) /* EXTI line 20 is connected to the RTC Wakeup event */
#define EXTI_COMP1               (1 << 21) /* EXTI line 21 is connected to the COMP1 (comparator) output */
#define EXTI_COMP2               (1 << 22) /* EXTI line 22 is connected to the COMP2 (comparator) output */
#define EXTI_I2C1                (1 << 23) /* EXTI line 23 is connected to the I2C1 wakeup */
                                           /* EXTI line 24 is reserved (internally held low) */
#define EXTI_USART1              (1 << 25) /* EXTI line 25 is connected to the USART1 wakeup */
#define EXTI_USART2              (1 << 26) /* EXTI line 26 is connected to the USART2 wakeup */
#define EXTI_CEC                 (1 << 27) /* EXTI line 27 is connected to the CEC wakeup */
#define EXTI_USART3              (1 << 28) /* EXTI line 28 is connected to the USART3 wakeup */
                                           /* EXTI line 29 is reserved (internally held low) */
                                           /* EXTI line 30 is reserved (internally held low) */
#define EXTI_VDDIO2              (1 << 31) /* EXTI line 31 is connected to the Vddio2 supply comparator */

/* Interrupt mask register */

#define EXTI_IMR_BIT(n)          STM32_EXTI_BIT(n) /* 1=Interrupt request from line x is not masked */
#define EXTI_IMR_SHIFT           (0)               /* Bits 0-X: Interrupt Mask for all lines */
#define EXTI_IMR_MASK            STM32_EXTI_MASK

/* Event mask register */

#define EXTI_EMR_BIT(n)          STM32_EXTI_BIT(n) /* 1=Event request from line x is not mask */
#define EXTI_EMR_SHIFT           (0)               /* Bits Bits 0-X:  Event Mask for all lines */
#define EXTI_EMR_MASK            STM32_EXTI_MASK

/* Rising Trigger selection register */

#define EXTI_RTSR_BIT(n)         STM32_EXTI_BIT(n) /* 1=Rising trigger enabled (for Event and Interrupt) for input line */
#define EXTI_RTSR_SHIFT          (0)               /* Bits 0-X: Rising trigger event configuration bit for all lines */
#define EXTI_RTSR_MASK           STM32_EXTI_MASK

/* Falling Trigger selection register */

#define EXTI_FTSR_BIT(n)         STM32_EXTI_BIT(n)  /* 1=Falling trigger enabled (for Event and Interrupt) for input line */
#define EXTI_FTSR_SHIFT          (0)                /* Bits 0-X: Falling trigger event configuration bitfor all lines */
#define EXTI_FTSR_MASK           STM32_EXTI_MASK

/* Software interrupt event register  */

#define EXTI_SWIER_BIT(n)        STM32_EXTI_BIT(n)  /* 1=Sets the corresponding pending bit in EXTI_PR */
#define EXTI_SWIER_SHIFT         (0)                /* Bits 0-X: Software Interrupt for all lines */
#define EXTI_SWIER_MASK          STM32_EXTI_MASK

/* Pending register */

#define EXTI_PR_BIT(n)           STM32_EXTI_BIT(n)  /* 1=Selected trigger request occurred */
#define EXTI_PR_SHIFT            (0)                /* Bits 0-X: Pending bit for all lines */
#define EXTI_PR_MASK             STM32_EXTI_MASK

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F0_EXTI_H */
