/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_gpioint.h
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

/* GPIO pin interrupts
 *
 * From all available GPIO pins, up to eight pins can be selected in the
 * system control block to serve as external interrupt pins. The external
 * interrupt pins are connected to eight individual interrupts in the NVIC
 * and are created based on rising or falling edges or on the input level
 * on the pin.
 *
 * GPIO group interrupt
 *
 * For each port/pin connected to one of the two the GPIO Grouped Interrupt
 * blocks (GROUP0 and GROUP1), the GPIO grouped interrupt registers determine
 * which pins are enabled to generate interrupts and what the active
 * polarities of each of those inputs are. The GPIO grouped interrupt
 * registers also select whether the interrupt output will be level or edge
 * triggered and whether it will be based on the OR or the AND of all of the
 * enabled inputs.
 */

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_GPIOINT_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_GPIOINT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/lpc43_gpio.h"

#ifdef CONFIG_LPC43_GPIO_IRQ

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_gpioint_grpinitialize
 *
 * Description:
 *   Initialize the properties of a GPIO group.  The properties of the group
 *   should be configured before any pins are added to the group by
 *   lpc32_gpioint_grpconfig().  As side effects, this call also removes
 *   all pins from the group and disables the group interrupt.  On return,
 *   this is a properly configured, empty GPIO interrupt group.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

int lpc43_gpioint_grpinitialize(int group, bool anded, bool level);

/****************************************************************************
 * Name: lpc43_gpioint_pinconfig
 *
 * Description:
 *   Configure a GPIO pin as an GPIO pin interrupt source (after it has been
 *   configured as an input).  This function should *not* be called directly
 *   from user application code; user code should call this function only
 *   indirectly through lpc32_gpio_config().
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

int lpc43_gpioint_pinconfig(uint16_t gpiocfg);

/****************************************************************************
 * Name: lpc43_gpioint_grpconfig
 *
 * Description:
 *   Configure a GPIO pin as an GPIO group interrupt member (after it has
 *   been configured as an input).  This function should *not* be called
 *   directly from user application code; user code should call this function
 *   only indirectly through lpc32_gpio_config().
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Interrupts are disabled so that read-modify-write operations are safe.
 *
 ****************************************************************************/

int lpc43_gpioint_grpconfig(uint16_t gpiocfg);

/****************************************************************************
 * Name: lpc43_gpioint_ack
 *
 * Description:
 *   Acknowledge the interrupt for a given pint interrupt number. Call this
 *   inside the interrupt handler. For edge sensitive interrupts, the
 *   interrupt status is cleared. For level sensitive interrupts, the
 *   active-high/-low sensitivity is inverted.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lpc43_gpioint_ack(uint8_t intnumber);

#endif /* CONFIG_LPC43_GPIO_IRQ */
#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_GPIOINT_H */
