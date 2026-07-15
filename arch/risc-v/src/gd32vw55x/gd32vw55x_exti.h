/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_exti.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_EXTI_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <nuttx/irq.h>

#include "chip.h"
#include "gd32vw55x_gpio.h"
#include "hardware/gd32vw55x_exti.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EXTI mode: the line may raise a CPU interrupt, a CPU event, or nothing */

#define EXTI_MODE_NONE            0
#define EXTI_MODE_INTERRUPT       1
#define EXTI_MODE_EVENT           2

/* EXTI trigger type */

#define EXTI_TRIG_NONE            0
#define EXTI_TRIG_RISING          1
#define EXTI_TRIG_FALLING         2
#define EXTI_TRIG_BOTH            3

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: gd32_gpio_setevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *   pinset      - GPIO pin configuration, as used by gd32_gpio_config().
 *                 Only the port and the pin number are used.
 *   risingedge  - Enables interrupt on rising edges
 *   fallingedge - Enables interrupt on falling edges
 *   event       - Generate an event instead of an interrupt.  In that case
 *                 no interrupt is delivered to the CPU and 'func' must be
 *                 NULL.
 *   func        - Interrupt handler.  A NULL value disables the interrupt
 *                 and detaches any previously installed handler.
 *   arg         - Argument passed to the interrupt handler
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port or pin.
 *
 ****************************************************************************/

int gd32_gpio_setevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg);

/****************************************************************************
 * Name: gd32_exti_init
 *
 * Description:
 *   Initialize the mode and the trigger type of the given EXTI lines.
 *
 * Input Parameters:
 *   linex     - Bit mask of the EXTI lines, EXTI_LINE(n)
 *   exti_mode - EXTI_MODE_NONE, EXTI_MODE_INTERRUPT or EXTI_MODE_EVENT
 *   trig_type - EXTI_TRIG_NONE, EXTI_TRIG_RISING, EXTI_TRIG_FALLING or
 *               EXTI_TRIG_BOTH
 *
 ****************************************************************************/

void gd32_exti_init(uint32_t linex, uint8_t exti_mode, uint8_t trig_type);

/****************************************************************************
 * Name: gd32_exti_interrupt_enable
 *
 * Description:
 *   Enable the interrupts from the given EXTI lines.
 *
 * Input Parameters:
 *   linex - Bit mask of the EXTI lines
 *
 ****************************************************************************/

void gd32_exti_interrupt_enable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_interrupt_disable
 *
 * Description:
 *   Disable the interrupts from the given EXTI lines.
 *
 * Input Parameters:
 *   linex - Bit mask of the EXTI lines
 *
 ****************************************************************************/

void gd32_exti_interrupt_disable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_event_enable
 *
 * Description:
 *   Enable the events from the given EXTI lines.
 *
 * Input Parameters:
 *   linex - Bit mask of the EXTI lines
 *
 ****************************************************************************/

void gd32_exti_event_enable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_event_disable
 *
 * Description:
 *   Disable the events from the given EXTI lines.
 *
 * Input Parameters:
 *   linex - Bit mask of the EXTI lines
 *
 ****************************************************************************/

void gd32_exti_event_disable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_software_interrupt_enable
 *
 * Description:
 *   Raise a software interrupt/event on the given EXTI lines.
 *
 * Input Parameters:
 *   linex - Bit mask of the EXTI lines
 *
 ****************************************************************************/

void gd32_exti_software_interrupt_enable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_software_interrupt_disable
 *
 * Description:
 *   Clear the software interrupt/event of the given EXTI lines.
 *
 * Input Parameters:
 *   linex - Bit mask of the EXTI lines
 *
 ****************************************************************************/

void gd32_exti_software_interrupt_disable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_interrupt_flag_get
 *
 * Description:
 *   Check whether any of the given EXTI lines has a pending interrupt.
 *
 * Input Parameters:
 *   linex - Bit mask of the EXTI lines
 *
 * Returned Value:
 *   true if at least one of the lines is enabled and pending.
 *
 ****************************************************************************/

bool gd32_exti_interrupt_flag_get(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_interrupt_flag_clear
 *
 * Description:
 *   Clear the pending flag of the given EXTI lines.
 *
 * Input Parameters:
 *   linex - Bit mask of the EXTI lines
 *
 ****************************************************************************/

void gd32_exti_interrupt_flag_clear(uint32_t linex);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_EXTI_H */
