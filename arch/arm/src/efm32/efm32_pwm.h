/****************************************************************************
 * arch/arm/src/efm32/efm32_pwm.h
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_PWM_H
#define __ARCH_ARM_SRC_EFM32_EFM32_PWM_H

/* The EFM32 does not have dedicated PWM hardware.  Rather, pulsed output
 * control is a capability of the EFM32 timers.  The logic in this file
 * implements the lower half of the standard, NuttX PWM interface using the
 * EFM32 timers. That interface is described in include/nuttx/timers/pwm.h.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.
 * If CONFIG_EFM32_TIMERn is defined then the CONFIG_EFM32_TIMERn_PWM must
 * also be defined to indicate that timer "n" is intended to be used for
 * pulsed output signal generation.
 */

#ifndef CONFIG_EFM32_TIMER0
#  undef CONFIG_EFM32_TIMER0_PWM
#endif
#ifndef CONFIG_EFM32_TIMER1
#  undef CONFIG_EFM32_TIMER1_PWM
#endif
#ifndef CONFIG_EFM32_TIMER2
#  undef CONFIG_EFM32_TIMER2_PWM
#endif
#ifndef CONFIG_EFM32_TIMER3
#  undef CONFIG_EFM32_TIMER3_PWM
#endif

/* Check if PWM support for any channel is enabled. */

#if defined(CONFIG_EFM32_TIMER0_PWM) || \
    defined(CONFIG_EFM32_TIMER1_PWM) || \
    defined(CONFIG_EFM32_TIMER2_PWM) || \
    defined(CONFIG_EFM32_TIMER3_PWM)

#include <arch/board/board.h>
#include "hardware/efm32_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For each timer that is enabled for PWM usage, we need the following
 * additional configuration settings:
 *
 * CONFIG_EFM32_TIMERx_CHANNEL - Specifies the timer output channel {0,1,3}
 * BOARD_PWM_TIMERx_PINCFG - Specifies the timer output pin configuration.
 *                 example : (GPIO_PORTC|GPIO_PIN0|GPIO_OUTPUT_PUSHPULL)
 *
 * BOARD_PWM_TIMERx_PINLOC - Specifies the timer output pin location.
 *                 example : _TIMER_ROUTE_LOCATION_LOC4
 *
 * BOARD_PWM_TIMERx_CLKIN  - Specifies the timer input clock frequency
 *                           (in Hz). example : 48e6 for 48MHz
 *
 * NOTE: The EFM32 timers are each capable of generating different signals on
 * each of the four channels with different duty cycles.  That capability is
 * not supported by this driver:  Only one output channel per timer.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the EFM32 MCU and MCU family but is somewhere in
 *     the range of {0,..,3}.
 *
 * Returned Value:
 *   On success, a pointer to the EFM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *efm32_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_EFM32_TIMERx_PWM */
#endif /* __ARCH_ARM_SRC_EFM32_EFM32_PWM_H */
