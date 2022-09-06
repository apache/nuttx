/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_exti.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_EXTI_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GD32_NEXTI                        23
#define GD32_EXTI_MASK                    0x007fffff

#define GD32_EXTI_BIT(n)                  (1 << (n))

/* Register Offsets *********************************************************/

#define GD32_EXTI_INTEN_OFFSET           0x0000  /* interrupt enable register offset */
#define GD32_EXTI_EVEN_OFFSET            0x0004  /* event enable register offset */
#define GD32_EXTI_RTEN_OFFSET            0x0008  /* rising edge trigger enable register offset */
#define GD32_EXTI_FTEN_OFFSET            0x000c  /* falling trigger enable register offset */
#define GD32_EXTI_SWIEV_OFFSET           0x0010  /* software interrupt event register offset */
#define GD32_EXTI_PD_OFFSET              0x0014  /* pending register offset */

/* Register Addresses *******************************************************/

#define GD32_EXTI_INTEN                  (GD32_EXTI_BASE+GD32_EXTI_INTEN_OFFSET) /* interrupt enable register */
#define GD32_EXTI_EVEN                   (GD32_EXTI_BASE+GD32_EXTI_EVEN_OFFSET)  /* event enable register */
#define GD32_EXTI_RTEN                   (GD32_EXTI_BASE+GD32_EXTI_RTEN_OFFSET)  /* rising edge trigger enable register */
#define GD32_EXTI_FTEN                   (GD32_EXTI_BASE+GD32_EXTI_FTEN_OFFSET)  /* falling trigger enable register */
#define GD32_EXTI_SWIEV                  (GD32_EXTI_BASE+GD32_EXTI_SWIEV_OFFSET) /* software interrupt event register */
#define GD32_EXTI_PD                     (GD32_EXTI_BASE+GD32_EXTI_PD_OFFSET)    /* pending register */

/* Register Bitfield Definitions ********************************************/

/* EXTI lines > 15 are associated with internal devices: */

#  define EXTI_PVD_LINE                  (1 << 16)   /* EXTI line 16 is connected to the PVD output */
#  define EXTI_RTC_ALARM                 (1 << 17)   /* EXTI line 17 is connected to the RTC Alarm event */
#  define EXTI_OTGFS_WAKEUP              (1 << 18)   /* EXTI line 18 is connected to the USB OTG FS Wakeup event */
#  define EXTI_ETH_WAKEUP                (1 << 19)   /* EXTI line 19 is connected to the Ethernet Wakeup event */
#  define EXTI_OTGHS_WAKEUP              (1 << 20)   /* EXTI line 20 is connected to the USB OTG HS Wakeup event */
#  define EXTI_RTC_TAMPER                (1 << 21)   /* EXTI line 21 is connected to the RTC Tamper and TimeStamp events */
#  define EXTI_RTC_TIMESTAMP             (1 << 21)   /* EXTI line 21 is connected to the RTC Tamper and TimeStamp events */
#  define EXTI_RTC_WAKEUP                (1 << 22)   /* EXTI line 22 is connected to the RTC Wakeup event */

/* Interrupt mask register */

#define EXTI_INTEN_BIT(n)                GD32_EXTI_BIT(n)      /* 1=Interrupt request from line x is not masked */
#define EXTI_INTEN_SHIFT                 (0)                   /* Bits 0-X: Interrupt Mask for all lines */
#define EXTI_INTEN_MASK                  GD32_EXTI_MASK

/* Event mask register */

#define EXTI_EVEN_BIT(n)                 GD32_EXTI_BIT(n)      /* 1=Event request from line x is not mask */
#define EXTI_EVEN_SHIFT                  (0)                   /* Bits Bits 0-X:  Event Mask for all lines */
#define EXTI_EVEN_MASK                   GD32_EXTI_MASK

/* Rising Trigger selection register */

#define EXTI_RTEN_BIT(n)                 GD32_EXTI_BIT(n)      /* 1=Rising trigger enabled (for Event and Interrupt) for input line */
#define EXTI_RTEN_SHIFT                  (0)                   /* Bits 0-X: Rising trigger event configuration bit for all lines */
#define EXTI_RTEN_MASK                   GD32_EXTI_MASK

/* Falling Trigger selection register */

#define EXTI_FTEN_BIT(n)                 GD32_EXTI_BIT(n)      /* 1=Falling trigger enabled (for Event and Interrupt) for input line */
#define EXTI_FTEN_SHIFT                  (0)                   /* Bits 0-X: Falling trigger event configuration bitfor all lines */
#define EXTI_FTEN_MASK                   GD32_EXTI_MASK

/* Software interrupt event register  */

#define EXTI_SWIEV_BIT(n)                GD32_EXTI_BIT(n)      /* 1=Sets the corresponding pending bit in EXTI_PR */
#define EXTI_SWIEV_SHIFT                 (0)                   /* Bits 0-X: Software Interrupt for all lines */
#define EXTI_SWIEV_MASK                  GD32_EXTI_MASK

/* Pending register */

#define EXTI_PD_BIT(n)                   GD32_EXTI_BIT(n)      /* 1=Selected trigger request occurred */
#define EXTI_PD_SHIFT                    (0)                   /* Bits 0-X: Pending bit for all lines */
#define EXTI_PD_MASK                     GD32_EXTI_MASK

/* EXTI line number */
#define EXTI_0                           GD32_EXTI_BIT(0)      /* EXTI line 0 */
#define EXTI_1                           GD32_EXTI_BIT(1)      /* EXTI line 1 */
#define EXTI_2                           GD32_EXTI_BIT(2)      /* EXTI line 2 */
#define EXTI_3                           GD32_EXTI_BIT(3)      /* EXTI line 3 */
#define EXTI_4                           GD32_EXTI_BIT(4)      /* EXTI line 4 */
#define EXTI_5                           GD32_EXTI_BIT(5)      /* EXTI line 5 */
#define EXTI_6                           GD32_EXTI_BIT(6)      /* EXTI line 6 */
#define EXTI_7                           GD32_EXTI_BIT(7)      /* EXTI line 7 */
#define EXTI_8                           GD32_EXTI_BIT(8)      /* EXTI line 8 */
#define EXTI_9                           GD32_EXTI_BIT(9)      /* EXTI line 9 */
#define EXTI_10                          GD32_EXTI_BIT(10)     /* EXTI line 10 */
#define EXTI_11                          GD32_EXTI_BIT(11)     /* EXTI line 11 */
#define EXTI_12                          GD32_EXTI_BIT(12)     /* EXTI line 12 */
#define EXTI_13                          GD32_EXTI_BIT(13)     /* EXTI line 13 */
#define EXTI_14                          GD32_EXTI_BIT(14)     /* EXTI line 14 */
#define EXTI_15                          GD32_EXTI_BIT(15)     /* EXTI line 15 */
#define EXTI_16                          GD32_EXTI_BIT(16)     /* EXTI line 16 */
#define EXTI_17                          GD32_EXTI_BIT(17)     /* EXTI line 17 */
#define EXTI_18                          GD32_EXTI_BIT(18)     /* EXTI line 18 */
#define EXTI_19                          GD32_EXTI_BIT(19)     /* EXTI line 19 */
#define EXTI_20                          GD32_EXTI_BIT(20)     /* EXTI line 20 */    
#define EXTI_21                          GD32_EXTI_BIT(21)     /* EXTI line 21 */
#define EXTI_22                          GD32_EXTI_BIT(22)     /* EXTI line 22 */

/* External interrupt and event  */

#define EXTI_INTERRUPT                   0                     /* EXTI interrupt mode */
#define EXTI_EVENT                       1                     /* EXTI event mode */

/* Interrupt trigger mode */
#define EXTI_TRIG_RISING                 0                     /* EXTI rising edge trigger */
#define EXTI_TRIG_FALLING                1                     /* EXTI falling edge trigger */
#define EXTI_TRIG_BOTH                   2                     /* EXTI rising and falling edge trigger */
#define EXTI_TRIG_NONE                   3                     /* None EXTI edge trigger */

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_EXTI_H */
