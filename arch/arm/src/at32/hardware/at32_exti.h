/****************************************************************************
 * arch/arm/src/at32/hardware/at32_exti.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32_EXTI_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_AT32_AT32F43XX)
#  define AT32_NEXTI            23
#  define AT32_EXTI_MASK        0x007fffff
#endif

#define AT32_EXTI_BIT(n)        (1 << (n))

/* Register Offsets *********************************************************/

#define AT32_EXTI_IMR_OFFSET    0x0000  /* Interrupt mask register */
#define AT32_EXTI_EMR_OFFSET    0x0004  /* Event mask register */
#define AT32_EXTI_RTSR_OFFSET   0x0008  /* Rising Trigger selection register */
#define AT32_EXTI_FTSR_OFFSET   0x000c  /* Falling Trigger selection register */
#define AT32_EXTI_SWIER_OFFSET  0x0010  /* Software interrupt event register */
#define AT32_EXTI_PR_OFFSET     0x0014  /* Pending register */

/* Register Addresses *******************************************************/

#  define AT32_EXTI_IMR         (AT32_EXINT_BASE+AT32_EXTI_IMR_OFFSET)
#  define AT32_EXTI_EMR         (AT32_EXINT_BASE+AT32_EXTI_EMR_OFFSET)
#  define AT32_EXTI_RTSR        (AT32_EXINT_BASE+AT32_EXTI_RTSR_OFFSET)
#  define AT32_EXTI_FTSR        (AT32_EXINT_BASE+AT32_EXTI_FTSR_OFFSET)
#  define AT32_EXTI_SWIER       (AT32_EXINT_BASE+AT32_EXTI_SWIER_OFFSET)
#  define AT32_EXTI_PR          (AT32_EXINT_BASE+AT32_EXTI_PR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* EXTI lines > 15 are associated with internal devices: */

#if defined(CONFIG_AT32_AT32F43XX)
#  define EXTI_PVD_LINE          (1 << 16) /* EXTI line 16 is connected to the PVD output */
#  define EXTI_RTC_ALARM         (1 << 17) /* EXTI line 17 is connected to the RTC Alarm event */
#  define EXTI_OTGFS_WAKEUP      (1 << 18) /* EXTI line 18 is connected to the USB OTG FS Wakeup event */
#  define EXTI_ETH_WAKEUP        (1 << 19) /* EXTI line 19 is connected to the Ethernet Wakeup event */
#  define EXTI_OTGHS_WAKEUP      (1 << 20) /* EXTI line 20 is connected to the USB OTG HS Wakeup event */
#  define EXTI_RTC_TAMPER        (1 << 21) /* EXTI line 21 is connected to the RTC Tamper and TimeStamp events */
#  define EXTI_RTC_TIMESTAMP     (1 << 21) /* EXTI line 21 is connected to the RTC Tamper and TimeStamp events */
#  define EXTI_RTC_WAKEUP        (1 << 22) /* EXTI line 22 is connected to the RTC Wakeup event */
#endif

/* Interrupt mask register */

#define EXTI_IMR_BIT(n)          AT32_EXTI_BIT(n) /* 1=Interrupt request from line x is not masked */
#define EXTI_IMR_SHIFT           (0)              /* Bits 0-X: Interrupt Mask for all lines */
#define EXTI_IMR_MASK            AT32_EXTI_MASK

/* Event mask register */

#define EXTI_EMR_BIT(n)          AT32_EXTI_BIT(n) /* 1=Event request from line x is not mask */
#define EXTI_EMR_SHIFT           (0)              /* Bits Bits 0-X:  Event Mask for all lines */
#define EXTI_EMR_MASK            AT32_EXTI_MASK

/* Rising Trigger selection register */

#define EXTI_RTSR_BIT(n)         AT32_EXTI_BIT(n) /* 1=Rising trigger enabled (for Event and Interrupt) for input line */
#define EXTI_RTSR_SHIFT          (0)              /* Bits 0-X: Rising trigger event configuration bit for all lines */
#define EXTI_RTSR_MASK           AT32_EXTI_MASK

/* Falling Trigger selection register */

#define EXTI_FTSR_BIT(n)         AT32_EXTI_BIT(n)  /* 1=Falling trigger enabled (for Event and Interrupt) for input line */
#define EXTI_FTSR_SHIFT          (0)               /* Bits 0-X: Falling trigger event configuration bitfor all lines */
#define EXTI_FTSR_MASK           AT32_EXTI_MASK

/* Software interrupt event register  */

#define EXTI_SWIER_BIT(n)        AT32_EXTI_BIT(n)  /* 1=Sets the corresponding pending bit in EXTI_PR */
#define EXTI_SWIER_SHIFT         (0)               /* Bits 0-X: Software Interrupt for all lines */
#define EXTI_SWIER_MASK          AT32_EXTI_MASK

/* Pending register */

#define EXTI_PR_BIT(n)           AT32_EXTI_BIT(n)  /* 1=Selected trigger request occurred */
#define EXTI_PR_SHIFT            (0)               /* Bits 0-X: Pending bit for all lines */
#define EXTI_PR_MASK             AT32_EXTI_MASK

/* Compatibility Definitions ************************************************/

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32_EXTI_H */