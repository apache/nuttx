/****************************************************************************
 * arch/tricore/src/common/aurix/tricore_gpio.h
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

#ifndef __ARCH_TRICORE_SRC_COMMON_AURIX_TRICORE_GPIO_H
#define __ARCH_TRICORE_SRC_COMMON_AURIX_TRICORE_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A pin set describes one port pin and its configuration:
 *
 *   bits 3:0    pin of the port
 *   bits 9:4    port number
 *   bit  10     output, else input
 *   bits 14:11  output: alternate function, 0 is the GPIO (TC3x: 0 to 7)
 *               input: pull device
 *   bit  15     open drain output
 *   bit  16     initial level of a GPIO output
 *   bits 19:17  pad driver mode (TC3x: 0 to 3)
 *   bits 22:20  pad level selection (TC3x: 0 to 3)
 */

#define GPIO_PIN_SHIFT         0
#define GPIO_PIN_MASK          (0xf << GPIO_PIN_SHIFT)
#define GPIO_PIN(n)            ((n) << GPIO_PIN_SHIFT)

#define GPIO_PORT_SHIFT        4
#define GPIO_PORT_MASK         (0x3f << GPIO_PORT_SHIFT)
#define GPIO_PORT(n)           ((n) << GPIO_PORT_SHIFT)

#define GPIO_INPUT             (0 << 10)
#define GPIO_OUTPUT            (1 << 10)

#define GPIO_ALT_SHIFT         11
#define GPIO_ALT_MASK          (0xf << GPIO_ALT_SHIFT)
#define GPIO_ALT(n)            ((n) << GPIO_ALT_SHIFT)

#define GPIO_FLOAT             (0 << GPIO_ALT_SHIFT)
#define GPIO_PULLDOWN          (1 << GPIO_ALT_SHIFT)
#define GPIO_PULLUP            (2 << GPIO_ALT_SHIFT)

#define GPIO_OPENDRAIN         (1 << 15)
#define GPIO_OUTPUT_SET        (1 << 16)

#define GPIO_PADDRV_SHIFT      17
#define GPIO_PADDRV_MASK       (0x7 << GPIO_PADDRV_SHIFT)
#define GPIO_PADDRV(n)         ((n) << GPIO_PADDRV_SHIFT)

#define GPIO_PADLEVEL_SHIFT    20
#define GPIO_PADLEVEL_MASK     (0x7 << GPIO_PADLEVEL_SHIFT)
#define GPIO_PADLEVEL(n)       ((n) << GPIO_PADLEVEL_SHIFT)

/* Fields of a pin set */

#define GPIO_PIN_OF(p)         (((p) & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT)
#define GPIO_PORT_OF(p)        (((p) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)
#define GPIO_ALT_OF(p)         (((p) & GPIO_ALT_MASK) >> GPIO_ALT_SHIFT)
#define GPIO_PADDRV_OF(p)      (((p) & GPIO_PADDRV_MASK) >> \
                                GPIO_PADDRV_SHIFT)
#define GPIO_PADLEVEL_OF(p)    (((p) & GPIO_PADLEVEL_MASK) >> \
                                GPIO_PADLEVEL_SHIFT)
#define GPIO_IS_OUTPUT(p)      (((p) & GPIO_OUTPUT) != 0)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_gpio_config
 *
 * Description:
 *   Configure a port pin as described by a pin set.  A GPIO output first
 *   gets its initial level, then the pad, then the direction.
 *
 * Returned Value:
 *   OK, or -EINVAL if the pin set is not valid for the chip.
 *
 ****************************************************************************/

int tricore_gpio_config(uint32_t pinset);

/****************************************************************************
 * Name: tricore_gpio_write
 *
 * Description:
 *   Set the output level of the pin of a pin set.
 *
 ****************************************************************************/

void tricore_gpio_write(uint32_t pinset, bool value);

/****************************************************************************
 * Name: tricore_gpio_read
 *
 * Description:
 *   Read the input level of the pin of a pin set.
 *
 ****************************************************************************/

bool tricore_gpio_read(uint32_t pinset);

/****************************************************************************
 * Name: tricore_gpio_register
 *
 * Description:
 *   Register a pin set as the GPIO device /dev/gpio<minor>.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int tricore_gpio_register(int minor, uint32_t pinset);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_TRICORE_SRC_COMMON_AURIX_TRICORE_GPIO_H */
