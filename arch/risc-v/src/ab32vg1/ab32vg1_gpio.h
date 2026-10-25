/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_gpio.h
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

#ifndef __ARCH_RISCV_SRC_AB32VG1_AB32VG1_GPIO_H
#define __ARCH_RISCV_SRC_AB32VG1_AB32VG1_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin configuration encoding (16 bits):
 *
 *   1111 1100 0000 0000
 *   5432 1098 7654 3210
 *   ---- ---- ---- ----
 *   .... .... .... .PPP  Pin number 0..7
 *   .... .... ..PP P...  Port: A, B, E, F
 *   .... .... .D.. ....  Direction: 1 = output
 *   .... .... V... ....  Initial output value
 *   .... ..UU .... ....  Pull: none, up, down (10 Kohm)
 */

#define GPIO_PIN_SHIFT      0
#define GPIO_PIN_MASK       (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN(n)       ((n) << GPIO_PIN_SHIFT)

#define GPIO_PORT_SHIFT     3
#define GPIO_PORT_MASK      (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA        (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORTB        (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORTE        (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORTF        (3 << GPIO_PORT_SHIFT)

#define GPIO_OUTPUT         (1 << 6)
#define GPIO_INPUT          (0)
#define GPIO_VALUE_ONE      (1 << 7)
#define GPIO_VALUE_ZERO     (0)

#define GPIO_PULL_SHIFT     8
#define GPIO_PULL_MASK      (3 << GPIO_PULL_SHIFT)
#  define GPIO_FLOAT        (0 << GPIO_PULL_SHIFT)
#  define GPIO_PULLUP       (1 << GPIO_PULL_SHIFT)
#  define GPIO_PULLDOWN     (2 << GPIO_PULL_SHIFT)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint16_t gpio_pinset_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ab32vg1_configgpio
 *
 * Description:
 *   Configure a pin as a digital GPIO according to the encoded pinset.
 *
 ****************************************************************************/

int ab32vg1_configgpio(gpio_pinset_t cfgset);

/****************************************************************************
 * Name: ab32vg1_gpiowrite
 *
 * Description:
 *   Drive an output pin.
 *
 ****************************************************************************/

void ab32vg1_gpiowrite(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: ab32vg1_gpioread
 *
 * Description:
 *   Read the level of a pin.
 *
 ****************************************************************************/

bool ab32vg1_gpioread(gpio_pinset_t pinset);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_AB32VG1_AB32VG1_GPIO_H */
