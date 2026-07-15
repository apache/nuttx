/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_exti.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_EXTI_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define GD32VW55X_EXTI_INTEN_OFFSET   0x0000  /* Interrupt enable */
#define GD32VW55X_EXTI_EVEN_OFFSET    0x0004  /* Event enable */
#define GD32VW55X_EXTI_RTEN_OFFSET    0x0008  /* Rising edge trigger enable */
#define GD32VW55X_EXTI_FTEN_OFFSET    0x000c  /* Falling edge trig enable */
#define GD32VW55X_EXTI_SWIEV_OFFSET   0x0010  /* Software interrupt event */
#define GD32VW55X_EXTI_PD_OFFSET      0x0014  /* Pending */

/* Register addresses *******************************************************/

#define GD32VW55X_EXTI_INTEN \
  (GD32VW55X_EXTI_BASE + GD32VW55X_EXTI_INTEN_OFFSET)
#define GD32VW55X_EXTI_EVEN \
  (GD32VW55X_EXTI_BASE + GD32VW55X_EXTI_EVEN_OFFSET)
#define GD32VW55X_EXTI_RTEN \
  (GD32VW55X_EXTI_BASE + GD32VW55X_EXTI_RTEN_OFFSET)
#define GD32VW55X_EXTI_FTEN \
  (GD32VW55X_EXTI_BASE + GD32VW55X_EXTI_FTEN_OFFSET)
#define GD32VW55X_EXTI_SWIEV \
  (GD32VW55X_EXTI_BASE + GD32VW55X_EXTI_SWIEV_OFFSET)
#define GD32VW55X_EXTI_PD \
  (GD32VW55X_EXTI_BASE + GD32VW55X_EXTI_PD_OFFSET)

/* Register bitfield definitions ********************************************/

/* The same bit position is used for a given line in the INTEN, EVEN, RTEN,
 * FTEN, SWIEV and PD registers.
 *
 * Lines 0-15 are the GPIO lines (each one is multiplexed over the three
 * GPIO ports by the SYSCFG EXTISSx registers).  Lines 16 and above are
 * connected to internal peripherals.  Line 18 does not exist.
 */

#define EXTI_LINE(n)                  (1 << (n))  /* Line n, n=0-25 */

#define EXTI_LINE_MASK                0x03ffffff  /* All implemented lines */
#define EXTI_GPIO_LINE_MASK           0x0000ffff  /* GPIO lines 0-15 */

/* GPIO lines */

#define EXTI_0                        EXTI_LINE(0)
#define EXTI_1                        EXTI_LINE(1)
#define EXTI_2                        EXTI_LINE(2)
#define EXTI_3                        EXTI_LINE(3)
#define EXTI_4                        EXTI_LINE(4)
#define EXTI_5                        EXTI_LINE(5)
#define EXTI_6                        EXTI_LINE(6)
#define EXTI_7                        EXTI_LINE(7)
#define EXTI_8                        EXTI_LINE(8)
#define EXTI_9                        EXTI_LINE(9)
#define EXTI_10                       EXTI_LINE(10)
#define EXTI_11                       EXTI_LINE(11)
#define EXTI_12                       EXTI_LINE(12)
#define EXTI_13                       EXTI_LINE(13)
#define EXTI_14                       EXTI_LINE(14)
#define EXTI_15                       EXTI_LINE(15)

/* Internal lines.  These are driven by on-chip peripherals (LVD, RTC and
 * the Wi-Fi/BLE wakeup logic).  Line 18 is reserved and is not implemented.
 */

#define EXTI_16                       EXTI_LINE(16)
#define EXTI_17                       EXTI_LINE(17)
#define EXTI_19                       EXTI_LINE(19)
#define EXTI_20                       EXTI_LINE(20)
#define EXTI_21                       EXTI_LINE(21)  /* RTC wakeup */
#define EXTI_22                       EXTI_LINE(22)
#define EXTI_23                       EXTI_LINE(23)
#define EXTI_24                       EXTI_LINE(24)  /* BLE wakeup */
#define EXTI_25                       EXTI_LINE(25)

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_EXTI_H */
