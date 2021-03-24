/****************************************************************************
 * arch/arm/src/stm32l4/hardware/stm32l4_exti.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_EXTI_H
#define __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32L4_NEXTI1         31
#define STM32L4_EXTI1_MASK     0xffffffff
#define STM32L4_NEXTI2         9
#define STM32L4_EXTI2_MASK     0x000001ff

#define STM32L4_EXTI1_BIT(n)        (1 << (n))
#define STM32L4_EXTI2_BIT(n)        (1 << (n))

/* Register Offsets *********************************************************/

#define STM32L4_EXTI1_OFFSET       0x0000  /* Offset to EXTI1 registers */
#define STM32L4_EXTI2_OFFSET       0x0020  /* Offset to EXTI2 registers */

#define STM32L4_EXTI_IMR_OFFSET    0x0000  /* Interrupt mask register */
#define STM32L4_EXTI_EMR_OFFSET    0x0004  /* Event mask register */
#define STM32L4_EXTI_RTSR_OFFSET   0x0008  /* Rising Trigger selection register */
#define STM32L4_EXTI_FTSR_OFFSET   0x000c  /* Falling Trigger selection register */
#define STM32L4_EXTI_SWIER_OFFSET  0x0010  /* Software interrupt event register */
#define STM32L4_EXTI_PR_OFFSET     0x0014  /* Pending register */

/* Register Addresses *******************************************************/

#define STM32L4_EXTI1_BASE       (STM32L4_EXTI_BASE+STM32L4_EXTI1_OFFSET)
#define STM32L4_EXTI2_BASE       (STM32L4_EXTI_BASE+STM32L4_EXTI2_OFFSET)

#define STM32L4_EXTI1_IMR        (STM32L4_EXTI1_BASE+STM32L4_EXTI_IMR_OFFSET)
#define STM32L4_EXTI1_EMR        (STM32L4_EXTI1_BASE+STM32L4_EXTI_EMR_OFFSET)
#define STM32L4_EXTI1_RTSR       (STM32L4_EXTI1_BASE+STM32L4_EXTI_RTSR_OFFSET)
#define STM32L4_EXTI1_FTSR       (STM32L4_EXTI1_BASE+STM32L4_EXTI_FTSR_OFFSET)
#define STM32L4_EXTI1_SWIER      (STM32L4_EXTI1_BASE+STM32L4_EXTI_SWIER_OFFSET)
#define STM32L4_EXTI1_PR         (STM32L4_EXTI1_BASE+STM32L4_EXTI_PR_OFFSET)

#define STM32L4_EXTI2_IMR        (STM32L4_EXTI2_BASE+STM32L4_EXTI_IMR_OFFSET)
#define STM32L4_EXTI2_EMR        (STM32L4_EXTI2_BASE+STM32L4_EXTI_EMR_OFFSET)
#define STM32L4_EXTI2_RTSR       (STM32L4_EXTI2_BASE+STM32L4_EXTI_RTSR_OFFSET)
#define STM32L4_EXTI2_FTSR       (STM32L4_EXTI2_BASE+STM32L4_EXTI_FTSR_OFFSET)
#define STM32L4_EXTI2_SWIER      (STM32L4_EXTI2_BASE+STM32L4_EXTI_SWIER_OFFSET)
#define STM32L4_EXTI2_PR         (STM32L4_EXTI2_BASE+STM32L4_EXTI_PR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* EXTI lines > 15 are associated with internal devices: */

#define EXTI1_PVD_LINE            (1 << 16) /* EXTI line 16 is connected to the PVD output */
#define EXTI1_OTGFS_WAKEUP        (1 << 17) /* EXTI line 17 is connected to the USB OTG FS Wakeup event */
#define EXTI1_RTC_ALARM           (1 << 18) /* EXTI line 18 is connected to the RTC Alarm event */
#define EXTI1_RTC_TAMPER          (1 << 19) /* EXTI line 19 is connected to the RTC Tamper and TimeStamp events */
#define EXTI1_RTC_WAKEUP          (1 << 20) /* EXTI line 20 is connected to the RTC Wakeup event */
#define EXTI1_COMP1               (1 << 21) /* EXTI line 21 is connected to the COMP1 (comparator) output */
#define EXTI1_COMP2               (1 << 22) /* EXTI line 22 is connected to the COMP2 (comparator) output */
#define EXTI1_I2C1                (1 << 23) /* EXTI line 23 is connected to the I2C1 wakeup */
#define EXTI1_I2C2                (1 << 24) /* EXTI line 24 is connected to the I2C2 wakeup */
#define EXTI1_I2C3                (1 << 25) /* EXTI line 25 is connected to the I2C3 wakeup */
#define EXTI1_USART1              (1 << 26) /* EXTI line 26 is connected to the USART1 wakeup */
#define EXTI1_USART2              (1 << 27) /* EXTI line 27 is connected to the USART2 wakeup */
#define EXTI1_USART3              (1 << 28) /* EXTI line 28 is connected to the USART3 wakeup */
#define EXTI1_UART4               (1 << 29) /* EXTI line 29 is connected to the UART4 wakeup */
#define EXTI1_UART5               (1 << 30) /* EXTI line 30 is connected to the UART5 wakeup */
#define EXTI1_LPUART1             (1 << 31) /* EXTI line 31 is connected to the LPUART1 wakeup */
#define EXTI2_LPTIM1              (1 <<  0) /* EXTI line 32 is connected to LPTIM1 */
#define EXTI2_LPTIM2              (1 <<  1) /* EXTI line 33 is connected to LPTIM2 */
#define EXTI2_SWPMI1              (1 <<  2) /* EXTI line 34 is connected to the SWPMI1 wakeup */
#define EXTI2_PVM1                (1 <<  3) /* EXTI line 35 is connected to the PVM1 wakeup */
#define EXTI2_PVM2                (1 <<  4) /* EXTI line 36 is connected to the PVM2 wakeup */
#define EXTI2_PVM3                (1 <<  5) /* EXTI line 37 is connected to the PVM3 wakeup */
#define EXTI2_PVM4                (1 <<  6) /* EXTI line 38 is connected to the PVM4 wakeup */
#define EXTI2_LCD                 (1 <<  7) /* EXTI line 39 is connected to the LCD wakeup */
#define EXTI2_I2C4                (1 <<  8) /* EXTI line 40 is connected to the I2C4 wakeup */

/* Interrupt mask register */

#define EXTI_IMR1_BIT(n)          STM32L4_EXTI1_BIT(n) /* 1=Interrupt request from line x is not masked */
#define EXTI_IMR1_SHIFT           (0)                  /* Bits 0-X: Interrupt Mask for all lines */
#define EXTI_IMR1_MASK            STM32L4_EXTI1_MASK

#define EXTI_IMR2_BIT(n)          STM32L4_EXTI2_BIT(n) /* 1=Interrupt request from line x is not masked */
#define EXTI_IMR2_SHIFT           (0)                  /* Bits 0-X: Interrupt Mask for all lines */
#define EXTI_IMR2_MASK            STM32L4_EXTI2_MASK

/* Event mask register */

#define EXTI_EMR1_BIT(n)          STM32L4_EXTI1_BIT(n) /* 1=Event request from line x is not mask */
#define EXTI_EMR1_SHIFT           (0)                  /* Bits Bits 0-X:  Event Mask for all lines */
#define EXTI_EMR1_MASK            STM32L4_EXTI1_MASK

#define EXTI_EMR2_BIT(n)          STM32L4_EXTI2_BIT(n) /* 1=Event request from line x is not mask */
#define EXTI_EMR2_SHIFT           (0)                  /* Bits Bits 0-X:  Event Mask for all lines */
#define EXTI_EMR2_MASK            STM32L4_EXTI2_MASK

/* Rising Trigger selection register */

#define EXTI_RTSR1_BIT(n)         STM32L4_EXTI1_BIT(n) /* 1=Rising trigger enabled (for Event and Interrupt) for input line */
#define EXTI_RTSR1_SHIFT          (0)                  /* Bits 0-X: Rising trigger event configuration bit for all lines */
#define EXTI_RTSR1_MASK           STM32L4_EXTI1_MASK

#define EXTI_RTSR2_BIT(n)         STM32L4_EXTI2_BIT(n) /* 1=Rising trigger enabled (for Event and Interrupt) for input line */
#define EXTI_RTSR2_SHIFT          (0)                  /* Bits 0-X: Rising trigger event configuration bit for all lines */
#define EXTI_RTSR2_MASK           STM32L4_EXTI2_MASK

/* Falling Trigger selection register */

#define EXTI_FTSR1_BIT(n)         STM32L4_EXTI1_BIT(n)  /* 1=Falling trigger enabled (for Event and Interrupt) for input line */
#define EXTI_FTSR1_SHIFT          (0)                   /* Bits 0-X: Falling trigger event configuration bitfor all lines */
#define EXTI_FTSR1_MASK           STM32L4_EXTI1_MASK

#define EXTI_FTSR2_BIT(n)         STM32L4_EXTI2_BIT(n)  /* 1=Falling trigger enabled (for Event and Interrupt) for input line */
#define EXTI_FTSR2_SHIFT          (0)                   /* Bits 0-X: Falling trigger event configuration bitfor all lines */
#define EXTI_FTSR2_MASK           STM32L4_EXTI2_MASK

/* Software interrupt event register  */

#define EXTI_SWIER1_BIT(n)        STM32L4_EXTI1_BIT(n)  /* 1=Sets the corresponding pending bit in EXTI_PR */
#define EXTI_SWIER1_SHIFT         (0)                   /* Bits 0-X: Software Interrupt for all lines */
#define EXTI_SWIER1_MASK          STM32L4_EXTI1_MASK

#define EXTI_SWIER2_BIT(n)        STM32L4_EXTI2_BIT(n)  /* 1=Sets the corresponding pending bit in EXTI_PR */
#define EXTI_SWIER2_SHIFT         (0)                   /* Bits 0-X: Software Interrupt for all lines */
#define EXTI_SWIER2_MASK          STM32L4_EXTI2_MASK

/* Pending register */

#define EXTI_PR1_BIT(n)           STM32L4_EXTI1_BIT(n)  /* 1=Selected trigger request occurred */
#define EXTI_PR1_SHIFT            (0)                   /* Bits 0-X: Pending bit for all lines */
#define EXTI_PR1_MASK             STM32L4_EXTI1_MASK

#define EXTI_PR2_BIT(n)           STM32L4_EXTI2_BIT(n)  /* 1=Selected trigger request occurred */
#define EXTI_PR2_SHIFT            (0)                   /* Bits 0-X: Pending bit for all lines */
#define EXTI_PR2_MASK             STM32L4_EXTI2_MASK

#endif /* __ARCH_ARM_SRC_STM32L4_HARDWARE_STM32L4_EXTI_H */
