/****************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32_exti.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_EXTI_H
#define __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/* Content of this file requires verification before it is used with other
 * families
 */

#if defined(CONFIG_STM32F7_STM32F72XX) || defined(CONFIG_STM32F7_STM32F73XX) || \
    defined(CONFIG_STM32F7_STM32F74XX) || defined(CONFIG_STM32F7_STM32F75XX) || \
    defined(CONFIG_STM32F7_STM32F76XX) || defined(CONFIG_STM32F7_STM32F77XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_NEXTI              24
#define STM32_EXTI_MASK          0x00ffffff
#define STM32_EXTI_BIT(n)        (1 << (n))

/* Register Offsets *********************************************************/

#define STM32_EXTI_IMR_OFFSET    0x0000  /* Interrupt mask register */
#define STM32_EXTI_EMR_OFFSET    0x0004  /* Event mask register */
#define STM32_EXTI_RTSR_OFFSET   0x0008  /* Rising Trigger selection register */
#define STM32_EXTI_FTSR_OFFSET   0x000c  /* Falling Trigger selection register */
#define STM32_EXTI_SWIER_OFFSET  0x0010  /* Software interrupt event register */
#define STM32_EXTI_PR_OFFSET     0x0014  /* Pending register */

/* Register Addresses *******************************************************/

#define STM32_EXTI_IMR           (STM32_EXTI_BASE+STM32_EXTI_IMR_OFFSET)
#define STM32_EXTI_EMR           (STM32_EXTI_BASE+STM32_EXTI_EMR_OFFSET)
#define STM32_EXTI_RTSR          (STM32_EXTI_BASE+STM32_EXTI_RTSR_OFFSET)
#define STM32_EXTI_FTSR          (STM32_EXTI_BASE+STM32_EXTI_FTSR_OFFSET)
#define STM32_EXTI_SWIER         (STM32_EXTI_BASE+STM32_EXTI_SWIER_OFFSET)
#define STM32_EXTI_PR            (STM32_EXTI_BASE+STM32_EXTI_PR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* EXTI linex < 16 are associated with GPIO pins 0-15.
 * EXTI lines > 15 are associated with internal devices:
 */

#define EXTI_PVD_LINE            (1 << 16) /* EXTI line 16 = PVD output */
#define EXTI_RTC_ALARM           (1 << 17) /* EXTI line 17 = RTC Alarm event */
#define EXTI_OTGFS_WAKEUP        (1 << 18) /* EXTI line 18 = USB OTG FS Wakeup event */
#define EXTI_ETH_WAKEUP          (1 << 19) /* EXTI line 19 = Ethernet Wakeup event */
#define EXTI_OTGHS_WAKEUP        (1 << 20) /* EXTI line 20 = USB OTG HS (FS mode) Wakeup eventt */
#define EXTI_RTC_TAMPER          (1 << 21) /* EXTI line 21 = RTC Tamper and TimeStamp events */
#define EXTI_RTC_TIMESTAMP       (1 << 21) /* EXTI line 21 = RTC Tamper and TimeStamp events */
#define EXTI_RTC_WAKEUP          (1 << 22) /* EXTI line 22 = RTC Wakeup event */
#define EXTI_LPTIM1_WAKEUP       (1 << 23) /* EXTI line 23 = LPTIM1 asynchronous event */

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

#endif /* CONFIG_STM32F7_STM32F74XX || CONFIG_STM32F7_STM32F75XX || CONFIG_STM32F7_STM32F76XX || CONFIG_STM32F7_STM32F77XX */
#endif /* __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_EXTI_H */
