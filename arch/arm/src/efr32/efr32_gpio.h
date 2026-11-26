/****************************************************************************
 * arch/arm/src/efr32/efr32_gpio.h
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

#ifndef __ARCH_ARM_SRC_EFR32_EFR32_GPIO_H
#define __ARCH_ARM_SRC_EFR32_EFR32_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include "hardware/efr32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to efr32_configgpio():
 *
 *   .... .... .... .... .IID MMMM ..PP NNNN
 *
 *   N - pin number (0-15)
 *   P - port (A-D)
 *   M - pin mode (GPIO_MODE_* in hardware/efr32_gpio.h)
 *   D - DOUT: the initial output level, or the pull-up (1) or pull-down
 *       (0) of the input modes with pull
 *   I - interrupt edges, used by efr32_gpioirq() only
 */

#define GPIO_PIN_SHIFT          (0)
#define GPIO_PIN_MASK           (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN(n)           ((n) << GPIO_PIN_SHIFT)

#define GPIO_PORT_SHIFT         (4)
#define GPIO_PORT_MASK          (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA            (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORTB            (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORTC            (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORTD            (3 << GPIO_PORT_SHIFT)

#define GPIO_CFG_SHIFT          (8)
#define GPIO_CFG_MASK           (15 << GPIO_CFG_SHIFT)
#  define GPIO_DISABLED         (GPIO_MODE_DISABLED << GPIO_CFG_SHIFT)
#  define GPIO_INPUT            (GPIO_MODE_INPUT << GPIO_CFG_SHIFT)
#  define GPIO_INPUT_PULL       (GPIO_MODE_INPUTPULL << GPIO_CFG_SHIFT)
#  define GPIO_INPUT_PULLFILTER (GPIO_MODE_INPUTPULLFILTER << GPIO_CFG_SHIFT)
#  define GPIO_PUSHPULL         (GPIO_MODE_PUSHPULL << GPIO_CFG_SHIFT)
#  define GPIO_WIREDAND         (GPIO_MODE_WIREDAND << GPIO_CFG_SHIFT)
#  define GPIO_WIREDAND_PULLUP  (GPIO_MODE_WIREDANDPULLUP << GPIO_CFG_SHIFT)

#define GPIO_DOUT               (1 << 12)
#  define GPIO_OUTPUT_SET       GPIO_DOUT
#  define GPIO_OUTPUT_CLEAR     (0)
#  define GPIO_PULLUP           GPIO_DOUT
#  define GPIO_PULLDOWN         (0)

#define GPIO_INT_SHIFT          (13)
#define GPIO_INT_MASK           (3 << GPIO_INT_SHIFT)
#  define GPIO_INT_NONE         (0)
#  define GPIO_INT_RISING       (1 << GPIO_INT_SHIFT)
#  define GPIO_INT_FALLING      (2 << GPIO_INT_SHIFT)
#  define GPIO_INT_BOTH         (3 << GPIO_INT_SHIFT)

/* Short forms */

#define GPIO_OUTPUT             GPIO_PUSHPULL
#define GPIO_INPUT_PULLUP       (GPIO_INPUT_PULL | GPIO_PULLUP)
#define GPIO_INPUT_PULLDOWN     (GPIO_INPUT_PULL | GPIO_PULLDOWN)

/* Port and pin of an encoded pin, for the peripheral route registers */

#define GPIO_PORTNO(pinset)     (((pinset) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)
#define GPIO_PINNO(pinset)      (((pinset) & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef uint32_t gpio_pinset_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: efr32_configgpio
 *
 * Description:
 *   Configure a pin according to its bit-encoded description.
 *
 * Returned Value:
 *   OK on success; a negated errno value if the pin is invalid.
 *
 ****************************************************************************/

int efr32_configgpio(gpio_pinset_t pinset);

/****************************************************************************
 * Name: efr32_unconfiggpio
 *
 * Description:
 *   Return a pin to its reset state (disabled, no pull).
 *
 ****************************************************************************/

int efr32_unconfiggpio(gpio_pinset_t pinset);

/****************************************************************************
 * Name: efr32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin.
 *
 ****************************************************************************/

void efr32_gpiowrite(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: efr32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin.
 *
 ****************************************************************************/

bool efr32_gpioread(gpio_pinset_t pinset);

#ifdef CONFIG_EFR32_GPIO_IRQ

/****************************************************************************
 * Name: efr32_gpioirqinitialize
 *
 * Description:
 *   Disable all pin interrupts and attach the even and odd GPIO interrupt
 *   handlers.  Called from up_irqinitialize().
 *
 ****************************************************************************/

void efr32_gpioirqinitialize(void);

/****************************************************************************
 * Name: efr32_gpioirq
 *
 * Description:
 *   Select the edges that set the interrupt flag of a pin.  Pin n uses
 *   EXTIn, so only pins 0-11 can interrupt, and only one port per pin
 *   number at a time.  The interrupt is left disabled; attach a handler
 *   to EFR32_IRQ_EXTI(pin) and enable it with up_enable_irq().
 *
 * Returned Value:
 *   OK on success; -EINVAL if the pin has no interrupt line.
 *
 ****************************************************************************/

int efr32_gpioirq(gpio_pinset_t pinset);

/****************************************************************************
 * Name: efr32_gpioirqenable and efr32_gpioirqdisable
 *
 * Description:
 *   Enable or disable one pin interrupt (EFR32_IRQ_EXTI0-11).  Called from
 *   up_enable_irq() and up_disable_irq().
 *
 ****************************************************************************/

void efr32_gpioirqenable(int irq);
void efr32_gpioirqdisable(int irq);

#endif /* CONFIG_EFR32_GPIO_IRQ */

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_EFR32_EFR32_GPIO_H */
