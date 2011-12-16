/************************************************************************************
 * arch/arm/src/stm32/stm32_pwm.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_TIM_H
#define __ARCH_ARM_SRC_STM32_STM32_TIM_H

/* The STM32 does not have dedicated PWM hardware.  Rather, pulsed output control
 * is a capabilitiy of the STM32 timers.  The logic in this file implements the
 * lower half of the standard, NuttX PWM interface using the STM32 timers.  That
 * interface is described in include/nuttx/pwm.h.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/stm32_tim.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
 * is defined then the CONFIG_STM32_TIMn_PWM must also be defined to indicate that
 * timer "n" is intended to be used for pulsed output signal generation.
 */

#ifndef CONFIG_STM32_TIM1
#  undef CONFIG_STM32_TIM1_PWM
#endif
#ifndef CONFIG_STM32_TIM2
#  undef CONFIG_STM32_TIM2_PWM
#endif
#ifndef CONFIG_STM32_TIM3
#  undef CONFIG_STM32_TIM3_PWM
#endif
#ifndef CONFIG_STM32_TIM4
#  undef CONFIG_STM32_TIM4_PWM
#endif
#ifndef CONFIG_STM32_TIM5
#  undef CONFIG_STM32_TIM5_PWM
#endif
#ifndef CONFIG_STM32_TIM6
#  undef CONFIG_STM32_TIM6_PWM
#endif
#ifndef CONFIG_STM32_TIM7
#  undef CONFIG_STM32_TIM7_PWM
#endif
#ifndef CONFIG_STM32_TIM8
#  undef CONFIG_STM32_TIM8_PWM
#endif
#ifndef CONFIG_STM32_TIM9
#  undef CONFIG_STM32_TIM9_PWM
#endif
#ifndef CONFIG_STM32_TIM10
#  undef CONFIG_STM32_TIM10_PWM
#endif
#ifndef CONFIG_STM32_TIM11
#  undef CONFIG_STM32_TIM11_PWM
#endif
#ifndef CONFIG_STM32_TIM12
#  undef CONFIG_STM32_TIM12_PWM
#endif
#ifndef CONFIG_STM32_TIM13
#  undef CONFIG_STM32_TIM13_PWM
#endif
#ifndef CONFIG_STM32_TIM14
#  undef CONFIG_STM32_TIM14_PWM
#endif

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
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,14}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ************************************************************************************/

EXTERN FAR struct pwm_lowerhalf_s *stm32_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_TIM_H */
