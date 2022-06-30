/****************************************************************************
 * arch/arm/src/stm32wb/hardware/stm32wb_exti.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_EXTI_H
#define __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32WB_EXTI_RTSR1_OFFSET   0x0000  /* Rising trigger selection register 1 */
#define STM32WB_EXTI_FTSR1_OFFSET   0x0004  /* Falling trigger selection register 1 */
#define STM32WB_EXTI_SWIER1_OFFSET  0x0008  /* Software interrupt event register 1 */
#define STM32WB_EXTI_PR1_OFFSET     0x000c  /* Pending register 1 */
#define STM32WB_EXTI_RTSR2_OFFSET   0x0020  /* Rising trigger selection register 2 */
#define STM32WB_EXTI_FTSR2_OFFSET   0x0024  /* Falling trigger selection register 2 */
#define STM32WB_EXTI_SWIER2_OFFSET  0x0028  /* Software interrupt event register 2 */
#define STM32WB_EXTI_PR2_OFFSET     0x002c  /* Pending register 2 */
#define STM32WB_EXTI_C1IMR1_OFFSET  0x0080  /* CPU1 wakeup with interrupt mask register 1 */
#define STM32WB_EXTI_C1EMR1_OFFSET  0x0084  /* CPU1 wakeup with event mask register  1 */
#define STM32WB_EXTI_C1IMR2_OFFSET  0x0090  /* CPU1 wakeup with interrupt mask register 2 */
#define STM32WB_EXTI_C1EMR2_OFFSET  0x0094  /* CPU1 wakeup with event mask register 2 */
#define STM32WB_EXTI_C2IMR1_OFFSET  0x00c0  /* CPU2 wakeup with interrupt mask register 1 */
#define STM32WB_EXTI_C2EMR1_OFFSET  0x00c4  /* CPU2 wakeup with event mask register 1 */
#define STM32WB_EXTI_C2IMR2_OFFSET  0x00d0  /* CPU2 wakeup with interrupt mask register 2 */
#define STM32WB_EXTI_C2EMR2_OFFSET  0x00d4  /* CPU2 wakeup with event mask register 2 */

/* Register Addresses *******************************************************/

#define STM32WB_EXTI_RTSR1          (STM32WB_EXTI_BASE + STM32WB_EXTI_RTSR1_OFFSET)
#define STM32WB_EXTI_FTSR1          (STM32WB_EXTI_BASE + STM32WB_EXTI_FTSR1_OFFSET)
#define STM32WB_EXTI_SWIER1         (STM32WB_EXTI_BASE + STM32WB_EXTI_SWIER1_OFFSET)
#define STM32WB_EXTI_PR1            (STM32WB_EXTI_BASE + STM32WB_EXTI_PR1_OFFSET)
#define STM32WB_EXTI_RTSR2          (STM32WB_EXTI_BASE + STM32WB_EXTI_RTSR2_OFFSET)
#define STM32WB_EXTI_FTSR2          (STM32WB_EXTI_BASE + STM32WB_EXTI_FTSR2_OFFSET)
#define STM32WB_EXTI_SWIER2         (STM32WB_EXTI_BASE + STM32WB_EXTI_SWIER2_OFFSET)
#define STM32WB_EXTI_PR2            (STM32WB_EXTI_BASE + STM32WB_EXTI_PR2_OFFSET)
#define STM32WB_EXTI_C1IMR1         (STM32WB_EXTI_BASE + STM32WB_EXTI_C1IMR1_OFFSET)
#define STM32WB_EXTI_C1EMR1         (STM32WB_EXTI_BASE + STM32WB_EXTI_C1EMR1_OFFSET)
#define STM32WB_EXTI_C1IMR2         (STM32WB_EXTI_BASE + STM32WB_EXTI_C1IMR2_OFFSET)
#define STM32WB_EXTI_C1EMR2         (STM32WB_EXTI_BASE + STM32WB_EXTI_C1EMR2_OFFSET)
#define STM32WB_EXTI_C2IMR1         (STM32WB_EXTI_BASE + STM32WB_EXTI_C2IMR1_OFFSET)
#define STM32WB_EXTI_C2EMR1         (STM32WB_EXTI_BASE + STM32WB_EXTI_C2EMR1_OFFSET)
#define STM32WB_EXTI_C2IMR2         (STM32WB_EXTI_BASE + STM32WB_EXTI_C2IMR2_OFFSET)
#define STM32WB_EXTI_C2EMR2         (STM32WB_EXTI_BASE + STM32WB_EXTI_C2EMR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* EXTI lines > 15 peripheral event sources */

#define EXTI_EVT_PVD                (16) /* EXTI line 16: PVD output */
#define EXTI_EVT_RTCALARM           (17) /* EXTI line 17: RTC Alarm event */
#define EXTI_EVT_RTCTAMPER          (18) /* EXTI line 18: RTC Tamper, TimeStamp, LSE CSS interrupt */
#define EXTI_EVT_RTCWAKEUP          (19) /* EXTI line 19: RTC Wakeup event */
#define EXTI_EVT_COMP1              (20) /* EXTI line 20: COMP1 (comparator) output */
#define EXTI_EVT_COMP2              (21) /* EXTI line 21: COMP2 (comparator) output */
#define EXTI_EVT_I2C1               (22) /* EXTI line 22: I2C1 wakeup */
#define EXTI_EVT_I2C3               (23) /* EXTI line 23: I2C3 wakeup */
#define EXTI_EVT_USART1             (24) /* EXTI line 24: USART1 wakeup */
#define EXTI_EVT_LPUART1            (25) /* EXTI line 25: LPUART1 wakeup */
#define EXTI_EVT_LPTIM1             (29) /* EXTI line 29: LPTIM1 */
#define EXTI_EVT_LPTIM2             (30) /* EXTI line 30: LPTIM2 */
#define EXTI_EVT_PVM1               (31) /* EXTI line 31: PVM1 wakeup */
#define EXTI_EVT_PVM3               (33) /* EXTI line 33: PVM2 wakeup */
#define EXTI_EVT_C1IPCC             (36) /* EXTI line 36: CPU1 IPCC wakeup */
#define EXTI_EVT_C2IPCC             (37) /* EXTI line 37: CPU2 IPCC wakeup */
#define EXTI_EVT_C1HSEM             (38) /* EXTI line 38: CPU1 HSEM interrupt 0 */
#define EXTI_EVT_C2HSEM             (39) /* EXTI line 39: CPU2 HSEM interrupt 1 */
#define EXTI_EVT_C2SEV              (40) /* EXTI line 40: CPU2 SEV line */
#define EXTI_EVT_C1SEV              (41) /* EXTI line 41: CPU1 SEV line */
#define EXTI_EVT_FLASH              (42) /* EXTI line 42: Flash ECC and global interrupts */
#define EXTI_EVT_LCD                (43) /* EXTI line 43: LCD wakeup */
#define EXTI_EVT_HSECSS             (44) /* EXTI line 44: RCC HSE CSS interrupts */
#define EXTI_EVT_BLE                (45) /* EXTI line 45: BLE and RADIO interrupts */
#define EXTI_EVT_802                (46) /* EXTI line 46: 802.15.14 Int0 and Int1 interrupts */
#define EXTI_EVT_DBG                (48) /* EXTI line 48: Debug power up request wakeup */

/* Rising trigger selection registers */

#define EXTI_RTSR1_RT(n)            (1 << (n))        /* Rising trigger event enable for line n = 0..21, 31 */
#define EXTI_RTSR1_MASK             0x803fffff

#define EXTI_RTSR2_RT(n)            (1 << ((n) & 31)) /* Rising trigger event enable for line n = 33, 40, 41 */
#define EXTI_RTSR2_MASK             0x00000302

/* Falling trigger selection registers */

#define EXTI_FTSR1_FT(n)            (1 << (n))        /* Falling trigger event enable for line n = 0..21, 31 */
#define EXTI_FTSR1_MASK             0x803fffff

#define EXTI_FTSR2_FT(n)            (1 << ((n) & 31)) /* Falling trigger event enable for line n = 33, 40, 41 */
#define EXTI_FTSR2_MASK             0x00000302

/* Software interrupt event registers */

#define EXTI_SWIER1_SWI(n)          (1 << (n))        /* Software interrupt on line n = 0..21, 31 */
#define EXTI_SWIER1_MASK            0x803fffff

#define EXTI_SWIER2_SWI(n)          (1 << ((n) & 31)) /* Software interrupt on line n = 33, 40, 41 */
#define EXTI_SWIER2_MASK            0x00000302

/* Pending registers */

#define EXTI_PR1_PIF(n)             (1 << (n))        /* Pending interrupt flag on line n = 0..21, 31 */
#define EXTI_PR1_MASK               0x803fffff

#define EXTI_PR2_PIF(n)             (1 << ((n) & 31)) /* Pending interrupt flag on line n = 33, 40, 41 */
#define EXTI_PR2_MASK               0x00000302

/* Interrupt mask registers */

#define EXTI_C1IMR1_IM(n)           (1 << (n))        /* Interrupt mask on line n = 0..31 */
#define EXTI_C1IMR1_MASK            0xffffffff

#define EXTI_C1IMR2_IM(n)           (1 << ((n) & 31)) /* Interrupt mask on line n = 32..48 */
#define EXTI_C1IMR2_MASK            0x0001ffff

/* Event mask registers */

#define EXTI_C1EMR1_EM(n)           (1 << (n))        /* Event mask on line n = 0..15, 17..21 */
#define EXTI_C1EMR1_MASK            0x003effff

#define EXTI_C1EMR2_EM(n)           (1 << ((n) & 31)) /* Event mask on line n = 40, 41 */
#define EXTI_C1EMR2_MASK            0x00000300

/* CPU2 Interrupt mask registers */

#define EXTI_C2IMR1_IM(n)           (1 << (n))        /* CPU2 Interrupt mask on line n = 0..31 */
#define EXTI_C2IMR1_MASK            0xffffffff

#define EXTI_C2IMR2_IM(n)           (1 << ((n) & 31)) /* CPU2 Interrupt mask on line n = 32..48 */
#define EXTI_C2IMR2_MASK            0x0001ffff

/* CPU2 Event mask registers */

#define EXTI_C2EMR1_EM(n)           (1 << (n))        /* CPU2 Event mask on line n = 0..15, 17..21 */
#define EXTI_C2EMR1_MASK            0x003effff

#define EXTI_C2EMR2_EM(n)           (1 << ((n) & 31)) /* CPU2 Event mask on line n = 40, 41 */
#define EXTI_C2EMR2_MASK            0x00000300

#endif /* __ARCH_ARM_SRC_STM32WB_HARDWARE_STM32WB_EXTI_H */
