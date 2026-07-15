/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_gpio.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_GPIO_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32VW55x implements three GPIO ports.  Not every pin of every port
 * exists on the die.  The pins which do exist are (taken from the EXTI
 * source list of the vendor SYSCFG driver):
 *
 *   GPIOA: 0..15
 *   GPIOB: 0, 1, 2, 3, 4, 11, 12, 13, 15
 *   GPIOC: 8, 13, 14, 15
 *
 * That is 29 pins, which matches the GPIO count of the QFN40 package.  The
 * QFN32 package bonds out only 22 of them.  The masks below are provided
 * for reference; the GPIO driver does not reject the missing pins because
 * the exact set depends on the package.
 */

#define GD32VW55X_NGPIO_PORTS          3

#define GD32VW55X_GPIOA_PINMASK        0xffff
#define GD32VW55X_GPIOB_PINMASK        0xb81f
#define GD32VW55X_GPIOC_PINMASK        0xe100

/* Register offsets (relative to a GPIO port base) **************************/

#define GD32VW55X_GPIO_CTL_OFFSET      0x0000  /* Port control */
#define GD32VW55X_GPIO_OMODE_OFFSET    0x0004  /* Port output mode */
#define GD32VW55X_GPIO_OSPD_OFFSET     0x0008  /* Port output speed */
#define GD32VW55X_GPIO_PUD_OFFSET      0x000c  /* Port pull-up/pull-down */
#define GD32VW55X_GPIO_ISTAT_OFFSET    0x0010  /* Port input status */
#define GD32VW55X_GPIO_OCTL_OFFSET     0x0014  /* Port output control */
#define GD32VW55X_GPIO_BOP_OFFSET      0x0018  /* Port bit operate (set) */
#define GD32VW55X_GPIO_LOCK_OFFSET     0x001c  /* Port configuration lock */
#define GD32VW55X_GPIO_AFSEL0_OFFSET   0x0020  /* AF select, pins 0-7 */
#define GD32VW55X_GPIO_AFSEL1_OFFSET   0x0024  /* AF select, pins 8-15 */
#define GD32VW55X_GPIO_BC_OFFSET       0x0028  /* Port bit clear */
#define GD32VW55X_GPIO_TG_OFFSET       0x002c  /* Port bit toggle */

/* Register addresses *******************************************************/

#define GD32VW55X_GPIOA                GD32VW55X_GPIOA_BASE
#define GD32VW55X_GPIOB                GD32VW55X_GPIOB_BASE
#define GD32VW55X_GPIOC                GD32VW55X_GPIOC_BASE

#define GD32VW55X_GPIO_CTL(b)          ((b) + GD32VW55X_GPIO_CTL_OFFSET)
#define GD32VW55X_GPIO_OMODE(b)        ((b) + GD32VW55X_GPIO_OMODE_OFFSET)
#define GD32VW55X_GPIO_OSPD(b)         ((b) + GD32VW55X_GPIO_OSPD_OFFSET)
#define GD32VW55X_GPIO_PUD(b)          ((b) + GD32VW55X_GPIO_PUD_OFFSET)
#define GD32VW55X_GPIO_ISTAT(b)        ((b) + GD32VW55X_GPIO_ISTAT_OFFSET)
#define GD32VW55X_GPIO_OCTL(b)         ((b) + GD32VW55X_GPIO_OCTL_OFFSET)
#define GD32VW55X_GPIO_BOP(b)          ((b) + GD32VW55X_GPIO_BOP_OFFSET)
#define GD32VW55X_GPIO_LOCK(b)         ((b) + GD32VW55X_GPIO_LOCK_OFFSET)
#define GD32VW55X_GPIO_AFSEL0(b)       ((b) + GD32VW55X_GPIO_AFSEL0_OFFSET)
#define GD32VW55X_GPIO_AFSEL1(b)       ((b) + GD32VW55X_GPIO_AFSEL1_OFFSET)
#define GD32VW55X_GPIO_BC(b)           ((b) + GD32VW55X_GPIO_BC_OFFSET)
#define GD32VW55X_GPIO_TG(b)           ((b) + GD32VW55X_GPIO_TG_OFFSET)

/* Register bitfield definitions ********************************************/

/* GPIO port control register (CTL), 2 bits per pin */

#define GPIO_MODE_INPUT                0  /* Input mode */
#define GPIO_MODE_OUTPUT               1  /* Output mode */
#define GPIO_MODE_AF                   2  /* Alternate function mode */
#define GPIO_MODE_ANALOG               3  /* Analog mode */

#define GPIO_MODE_SHIFT(n)             ((n) << 1)
#define GPIO_MODE_MASK(n)              (3 << GPIO_MODE_SHIFT(n))

/* Aliases used by the low-level console code */

#define GPIO_CTL_INPUT                 GPIO_MODE_INPUT
#define GPIO_CTL_OUTPUT                GPIO_MODE_OUTPUT
#define GPIO_CTL_AF                    GPIO_MODE_AF
#define GPIO_CTL_ANALOG                GPIO_MODE_ANALOG

/* GPIO port output mode register (OMODE), 1 bit per pin */

#define GPIO_OTYPE_PP(n)               (0)         /* Push-pull output */
#define GPIO_OTYPE_OD(n)               (1 << (n))  /* Open-drain output */

/* GPIO port output speed register (OSPD), 2 bits per pin.
 *
 * NOTE: the GD32VW55x speed levels differ from the GD32F4xx ones.  Level 1
 * is 10 MHz (not 25 MHz), level 2 is 25 MHz (not 50 MHz), and there is no
 * 200 MHz level:  level 3 is simply the maximum the pad supports.
 */

#define GPIO_OSPEED_LEVEL0             0  /* Output max speed 2 MHz */
#define GPIO_OSPEED_LEVEL1             1  /* Output max speed 10 MHz */
#define GPIO_OSPEED_LEVEL2             2  /* Output max speed 25 MHz */
#define GPIO_OSPEED_LEVEL3             3  /* Output max speed, maximum */

#define GPIO_OSPEED_2MHZ               GPIO_OSPEED_LEVEL0
#define GPIO_OSPEED_10MHZ              GPIO_OSPEED_LEVEL1
#define GPIO_OSPEED_25MHZ              GPIO_OSPEED_LEVEL2
#define GPIO_OSPEED_MAX                GPIO_OSPEED_LEVEL3

#define GPIO_OSPEED_SHIFT(n)           ((n) << 1)
#define GPIO_OSPEED_MASK(n)            (3 << GPIO_OSPEED_SHIFT(n))

/* Aliases used by the low-level console code */

#define GPIO_OSPD_2MHZ                 GPIO_OSPEED_LEVEL0
#define GPIO_OSPD_10MHZ                GPIO_OSPEED_LEVEL1
#define GPIO_OSPD_25MHZ                GPIO_OSPEED_LEVEL2
#define GPIO_OSPD_MAX                  GPIO_OSPEED_LEVEL3

/* GPIO port pull-up/pull-down register (PUD), 2 bits per pin */

#define GPIO_PUPD_NONE                 0  /* Floating, no pull resistor */
#define GPIO_PUPD_PULLUP               1  /* With pull-up resistor */
#define GPIO_PUPD_PULLDOWN             2  /* With pull-down resistor */

#define GPIO_PUPD_SHIFT(n)             ((n) << 1)
#define GPIO_PUPD_MASK(n)              (3 << GPIO_PUPD_SHIFT(n))

/* Aliases used by the low-level console code */

#define GPIO_PUD_NONE                  GPIO_PUPD_NONE
#define GPIO_PUD_PULLUP                GPIO_PUPD_PULLUP
#define GPIO_PUD_PULLDOWN              GPIO_PUPD_PULLDOWN

/* GPIO port input status register (ISTAT) */

#define GPIO_ISTAT(n)                  (1 << (n))

/* GPIO port output control register (OCTL) */

#define GPIO_OCTL(n)                   (1 << (n))

/* GPIO port bit operate register (BOP) */

#define GPIO_BOP_SET(n)                (1 << (n))
#define GPIO_BOP_CLEAR(n)              (1 << ((n) + 16))

/* GPIO port configuration lock register (LOCK) */

#define GPIO_LOCK(n)                   (1 << (n))
#define GPIO_LOCK_LKK                  (1 << 16)  /* Lock key */

/* GPIO alternate function selected registers (AFSEL0/1), 4 bits per pin */

#define GPIO_AF_SHIFT(n)               ((n) << 2)
#define GPIO_AF_MASK(n)                (15 << GPIO_AF_SHIFT(n))

/* GPIO port bit clear register (BC) */

#define GPIO_BC_SET(n)                 (1 << (n))

/* GPIO port bit toggle register (TG) */

#define GPIO_TG_SET(n)                 (1 << (n))

/* GPIO pin definitions */

#define GPIO_PIN(n)                    (1 << (n))  /* Bit n: pin n, n=0-15 */

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_GPIO_H */
