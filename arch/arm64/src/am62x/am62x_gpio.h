/****************************************************************************
 * arch/arm64/src/am62x/am62x_gpio.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_AM62X_AM62X_GPIO_H
#define __ARCH_ARM64_SRC_AM62X_AM62X_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint32_t gpio_pinset_t;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM62X_GPIO_LINE_SHIFT       0
#define AM62X_GPIO_LINE_MASK        (0xffu << AM62X_GPIO_LINE_SHIFT)
#define AM62X_GPIO_BANK_SHIFT       8
#define AM62X_GPIO_BANK_MASK        (0x03u << AM62X_GPIO_BANK_SHIFT)
#define AM62X_GPIO_INPUT            (1u << 10)
#define AM62X_GPIO_OUTPUT_ONE       (1u << 11)
#define AM62X_GPIO_PULL_SHIFT       12
#define AM62X_GPIO_PULL_MASK        (0x03u << AM62X_GPIO_PULL_SHIFT)
#  define AM62X_GPIO_PULL_NONE      (0u << AM62X_GPIO_PULL_SHIFT)
#  define AM62X_GPIO_PULL_UP        (1u << AM62X_GPIO_PULL_SHIFT)
#  define AM62X_GPIO_PULL_DOWN      (2u << AM62X_GPIO_PULL_SHIFT)
#define AM62X_GPIO_IRQ_SHIFT        14
#define AM62X_GPIO_IRQ_MASK         (0x07u << AM62X_GPIO_IRQ_SHIFT)
#  define AM62X_GPIO_IRQ_NONE       (0u << AM62X_GPIO_IRQ_SHIFT)
#  define AM62X_GPIO_IRQ_RISING     (1u << AM62X_GPIO_IRQ_SHIFT)
#  define AM62X_GPIO_IRQ_FALLING    (2u << AM62X_GPIO_IRQ_SHIFT)
#  define AM62X_GPIO_IRQ_BOTH       (3u << AM62X_GPIO_IRQ_SHIFT)
#  define AM62X_GPIO_IRQ_HIGH       (4u << AM62X_GPIO_IRQ_SHIFT)
#  define AM62X_GPIO_IRQ_LOW        (5u << AM62X_GPIO_IRQ_SHIFT)

#define AM62X_GPIO_OUTPUT           0u

#define AM62X_GPIO_PIN(b,l)         ((((uint32_t)(b) << AM62X_GPIO_BANK_SHIFT) & \
                                      AM62X_GPIO_BANK_MASK) | \
                                     (((uint32_t)(l) << AM62X_GPIO_LINE_SHIFT) & \
                                      AM62X_GPIO_LINE_MASK))
#define AM62X_GPIO_BANK(p)          (((p) & AM62X_GPIO_BANK_MASK) >> \
                                     AM62X_GPIO_BANK_SHIFT)
#define AM62X_GPIO_LINE(p)          (((p) & AM62X_GPIO_LINE_MASK) >> \
                                     AM62X_GPIO_LINE_SHIFT)
#define AM62X_GPIO_IS_INPUT(p)      (((p) & AM62X_GPIO_INPUT) != 0)
#define AM62X_GPIO_INITVAL(p)       (((p) & AM62X_GPIO_OUTPUT_ONE) != 0)
#define AM62X_GPIO_IRQMODE(p)       ((p) & AM62X_GPIO_IRQ_MASK)

#define AM62X_PADCFG_RXACTIVE       (1 << 18)
#define AM62X_PADCFG_PULL_DISABLE   (1 << 16)
#define AM62X_PADCFG_PULL_UP        (1 << 17)
#define AM62X_PADCFG_MUXMODE(n)     ((n) & 0x0f)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int am62x_gpio_initialize(void);
int am62x_configgpio(gpio_pinset_t cfgset);
void am62x_gpiowrite(gpio_pinset_t pinset, bool value);
bool am62x_gpioread(gpio_pinset_t pinset);
bool am62x_gpiooutread(gpio_pinset_t pinset);
int am62x_gpio_irq_attach(gpio_pinset_t pinset, xcpt_t isr, void *arg);
void am62x_gpio_irq_detach(gpio_pinset_t pinset);
int am62x_pinmux_configure(uintptr_t offset, uint32_t value);

#endif /* __ARCH_ARM64_SRC_AM62X_AM62X_GPIO_H */
