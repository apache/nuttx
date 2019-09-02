/****************************************************************************
 * arch/arm/src/efm32/efm32_pwm.h
 *
 *   Copyright (C) 2014 Pierre-Noel Bouteville. All rights reserved.
 *   Author: Pierre-Noel Bouteville <pnb990@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration **************************************************************/
/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.
 * If CONFIG_EFM32_TIMERn is defined then the CONFIG_EFM32_TIMERn_PWM must also
 * be defined to indicate that timer "n" is intended to be used for pulsed
 * output signal generation.
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

/* For each timer that is enabled for PWM usage, we need the following additional
 * configuration settings:
 *
 * CONFIG_EFM32_TIMERx_CHANNEL - Specifies the timer output channel {0,1,3}
 * BOARD_PWM_TIMERx_PINCFG - Specifies the timer output pin configuration.
 *                 example : (GPIO_PORTC|GPIO_PIN0|GPIO_OUTPUT_PUSHPULL)
 *
 * BOARD_PWM_TIMERx_PINLOC - Specifies the timer output pin location.
 *                 example : _TIMER_ROUTE_LOCATION_LOC4
 *
 * BOARD_PWM_TIMERx_CLKIN  - Specifies the timer input clock frequency (in Hz).
 *                 example : 48e6 for 48MHz
 *
 * NOTE: The EFM32 timers are each capable of generating different signals on
 * each of the four channels with different duty cycles.  That capability is
 * not supported by this driver:  Only one output channel per timer.
 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
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
 ************************************************************************************/

FAR struct pwm_lowerhalf_s *efm32_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_EFM32_TIMERx_PWM */
#endif /* __ARCH_ARM_SRC_EFM32_EFM32_PWM_H */
