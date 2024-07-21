/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_gpio.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_BCM2711_BCM2711_GPIO_H
#define __ARCH_ARM64_SRC_BCM2711_BCM2711_GPIO_H

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "hardware/bcm2711_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Possible functions for the BCM2711 GPIO pins */

enum bcm2711_gpio_func_e
{
  BCM_GPIO_FUNC0 = 0,  /* Alternative function 0 */
  BCM_GPIO_FUNC1 = 1,  /* Alternative function 1 */
  BCM_GPIO_FUNC2 = 2,  /* Alternative function 2 */
  BCM_GPIO_FUNC3 = 3,  /* Alternative function 3 */
  BCM_GPIO_FUNC4 = 4,  /* Alternative function 4 */
  BCM_GPIO_FUNC5 = 5,  /* Alternative function 5 */
  BCM_GPIO_INPUT = 6,  /* Input */
  BCM_GPIO_OUTPUT = 7, /* Output */
};

/****************************************************************************
 * Name: bcm2711_gpio_set_pulls
 *
 * Description:
 *   Set the specified GPIO pin to have pull up, pull down or no resistor.
 *   With both `up` and `down` as false, the resistor will be set to none.
 *   It is not possible to set both pull-up and pull-down.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the resistors on.
 *   up - True to set pull-up resistor, false otherwise.
 *   down - True to set pull-down resistor, false otherwise.
 *
 ****************************************************************************/

void bcm2711_gpio_set_pulls(uint32_t gpio, bool up, bool down);

/****************************************************************************
 * Name: bcm2711_gpio_set_func
 *
 * Description:
 *   Set the specified GPIO pin to be input, output or use one of its
 *   alternative functions.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the function of.
 *   func - The function to set the GPIO pin to use.
 *
 ****************************************************************************/

void bcm2711_gpio_set_func(uint32_t gpio, enum bcm2711_gpio_func_e func);

/****************************************************************************
 * Name: bcm2711_gpio_pin_set
 *
 * Description:
 *   Set the output of a GPIO output pin to high or low.
 *   Calling this function on a GPIO pin set as an input does nothing.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set high or low.
 *   high  - True to set the pin high, false to set the pin low.
 *
 ****************************************************************************/

void bcm2711_gpio_pin_set(uint32_t gpio, bool set);

/****************************************************************************
 * Name: bcm2711_gpio_pin_get
 *
 * Description:
 *   Get the current value of the GPIO.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set high or low.
 *
 * Return:
 *    True for high, false for low.
 *
 ****************************************************************************/

bool bcm2711_gpio_pin_get(uint32_t gpio);

/****************************************************************************
 * Name: bcm2711_gpio_event_get
 *
 * Description:
 *   Check if an event was detected for the given GPIO pin.
 *   The event bit will be set if an event has happened that matches the
 *   event detection configuration for the given pin (rising edge,
 *   falling edge, level).
 *
 * Input parameters:
 *   gpio - The GPIO pin number to check for an event.
 *
 * Return:
 *    True if an event was detected, false otherwise.
 *
 ****************************************************************************/

bool bcm2711_gpio_event_get(uint32_t gpio);

/****************************************************************************
 * Name: bcm2711_gpio_event_clear
 *
 * Description:
 *   Clear the event detect status for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to clear the event status of.
 *
 ****************************************************************************/

void bcm2711_gpio_event_clear(uint32_t gpio);

/****************************************************************************
 * Name: bcm2711_gpio_rising_edge
 *
 * Description:
 *   Set/clear rising edge event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_rising_edge(uint32_t gpio, bool set);

/****************************************************************************
 * Name: bcm2711_gpio_falling_edge
 *
 * Description:
 *   Set/clear falling edge event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_falling_edge(uint32_t gpio, bool set);

/****************************************************************************
 * Name: bcm2711_gpio_high_level
 *
 * Description:
 *   Set/clear high level event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_high_level(uint32_t gpio, bool set);

/****************************************************************************
 * Name: bcm2711_gpio_low_level
 *
 * Description:
 *   Set/clear low level event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_low_level(uint32_t gpio, bool set);

/****************************************************************************
 * Name: bcm2711_gpio_rising_edge_async
 *
 * Description:
 *   Set/clear async rising edge event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_rising_edge_async(uint32_t gpio, bool set);

/****************************************************************************
 * Name: bcm2711_gpio_falling_edge_async
 *
 * Description:
 *   Set/clear async falling edge event detection for the given GPIO pin.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to set the event detection of.
 *   set - True to set, false to clear.
 *
 ****************************************************************************/

void bcm2711_gpio_falling_edge_async(uint32_t gpio, bool set);

/****************************************************************************
 * Name: bcm2711_gpio_irq_attach
 *
 * Description:
 *   Attach an interrupt handler for the specified GPIO pin.
 *   NOTE: Interrupt mode (rising edge, falling edge, etc.) is configured
 *   separately.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to attach the handler for.
 *   isr - The interrupt handler function.
 *   arg - The argument to be passed to the interrupt handler.
 *
 ****************************************************************************/

int bcm2711_gpio_irq_attach(uint32_t gpio, xcpt_t isr, void *arg);

/****************************************************************************
 * Name: bcm2711_gpio_irq_detach
 *
 * Description:
 *   Detach an interrupt handler for a GPIO pin. NOTE: this does not disable
 *   interrupts for that particular pin; this must be done by disabling event
 *   detection for that pin separately.
 *   This function just detaches the pin's ISR, ensuring it won't be called
 *   when an interrupt is triggered.
 *
 * Input parameters:
 *   gpio - The GPIO pin number to detach the handler of.
 *
 ****************************************************************************/

void bcm2711_gpio_irq_detach(uint32_t gpio);

#endif /* __ARCH_ARM64_SRC_BCM2711_BCM2711_GPIO_H */
