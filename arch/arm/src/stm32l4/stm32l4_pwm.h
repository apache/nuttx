/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pwm.h
 *
 *   Copyright (C) 2011, 2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_PWM_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_PWM_H

/* The STM32L4 does not have dedicated PWM hardware.  Rather, pulsed output control
 * is a capability of the STM32L4 timers.  The logic in this file implements the
 * lower half of the standard, NuttX PWM interface using the STM32L4 timers.  That
 * interface is described in include/nuttx/drivers/pwm.h.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.  If CONFIG_STM32L4_TIMn
 * is defined then the CONFIG_STM32L4_TIMn_PWM must also be defined to indicate that
 * timer "n" is intended to be used for pulsed output signal generation.
 */

#ifndef CONFIG_STM32L4_TIM1
#  undef CONFIG_STM32L4_TIM1_PWM
#endif
#ifndef CONFIG_STM32L4_TIM2
#  undef CONFIG_STM32L4_TIM2_PWM
#endif
#ifndef CONFIG_STM32L4_TIM3
#  undef CONFIG_STM32L4_TIM3_PWM
#endif
#ifndef CONFIG_STM32L4_TIM4
#  undef CONFIG_STM32L4_TIM4_PWM
#endif
#ifndef CONFIG_STM32L4_TIM5
#  undef CONFIG_STM32L4_TIM5_PWM
#endif
#ifndef CONFIG_STM32L4_TIM8
#  undef CONFIG_STM32L4_TIM8_PWM
#endif
#ifndef CONFIG_STM32L4_TIM15
#  undef CONFIG_STM32L4_TIM15_PWM
#endif
#ifndef CONFIG_STM32L4_TIM16
#  undef CONFIG_STM32L4_TIM16_PWM
#endif
#ifndef CONFIG_STM32L4_TIM17
#  undef CONFIG_STM32L4_TIM17_PWM
#endif

/* The basic timers (timer 6 and 7) are not capable of generating output pulses */

#undef CONFIG_STM32L4_TIM6_PWM
#undef CONFIG_STM32L4_TIM7_PWM

/* Check if PWM support for any channel is enabled. */

#if defined(CONFIG_STM32L4_TIM1_PWM)  || defined(CONFIG_STM32L4_TIM2_PWM)  || \
    defined(CONFIG_STM32L4_TIM3_PWM)  || defined(CONFIG_STM32L4_TIM4_PWM)  || \
    defined(CONFIG_STM32L4_TIM5_PWM)  || defined(CONFIG_STM32L4_TIM8_PWM)  || \
    defined(CONFIG_STM32L4_TIM15_PWM) || defined(CONFIG_STM32L4_TIM16_PWM) || \
    defined(CONFIG_STM32L4_TIM17_PWM)

#include <arch/board/board.h>
#include "chip/stm32l4_tim.h"

#ifdef CONFIG_PWM_MULTICHAN

#ifdef CONFIG_STM32L4_TIM1_CHANNEL1
#  ifdef CONFIG_STM32L4_TIM1_CH1OUT
#    define PWM_TIM1_CH1CFG GPIO_TIM1_CH1OUT
#  else
#    define PWM_TIM1_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32L4_TIM1_CH1NOUT
#    define PWM_TIM1_CH1NCFG GPIO_TIM1_CH1NOUT
#  else
#    define PWM_TIM1_CH1NCFG 0
#  endif
#  define PWM_TIM1_CHANNEL1 1
#else
#  define PWM_TIM1_CHANNEL1 0
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL2
#  ifdef CONFIG_STM32L4_TIM1_CH2OUT
#    define PWM_TIM1_CH2CFG GPIO_TIM1_CH2OUT
#  else
#    define PWM_TIM1_CH2CFG 0
#  endif
#  ifdef CONFIG_STM32L4_TIM1_CH2NOUT
#    define PWM_TIM1_CH2NCFG GPIO_TIM1_CH2NOUT
#  else
#    define PWM_TIM1_CH2NCFG 0
#  endif
#  define PWM_TIM1_CHANNEL2 1
#else
#  define PWM_TIM1_CHANNEL2 0
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL3
#  ifdef CONFIG_STM32L4_TIM1_CH3OUT
#    define PWM_TIM1_CH3CFG GPIO_TIM1_CH3OUT
#  else
#    define PWM_TIM1_CH3CFG 0
#  endif
#  ifdef CONFIG_STM32L4_TIM1_CH3NOUT
#    define PWM_TIM1_CH3NCFG GPIO_TIM1_CH3NOUT
#  else
#    define PWM_TIM1_CH3NCFG 0
#  endif
#  define PWM_TIM1_CHANNEL3 1
#else
#  define PWM_TIM1_CHANNEL3 0
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL4
#  ifdef CONFIG_STM32L4_TIM1_CH4OUT
#    define PWM_TIM1_CH4CFG GPIO_TIM1_CH4OUT
#  else
#    define PWM_TIM1_CH4CFG 0
#  endif
#  define PWM_TIM1_CHANNEL4 1
#else
#  define PWM_TIM1_CHANNEL4 0
#endif
#define PWM_TIM1_NCHANNELS (PWM_TIM1_CHANNEL1 + PWM_TIM1_CHANNEL2 + \
                            PWM_TIM1_CHANNEL3 + PWM_TIM1_CHANNEL4)

#ifdef CONFIG_STM32L4_TIM2_CHANNEL1
#  ifdef CONFIG_STM32L4_TIM2_CH1OUT
#    define PWM_TIM2_CH1CFG GPIO_TIM2_CH1OUT
#  else
#    define PWM_TIM2_CH1CFG 0
#  endif
#  define PWM_TIM2_CHANNEL1 1
#else
#  define PWM_TIM2_CHANNEL1 0
#endif
#ifdef CONFIG_STM32L4_TIM2_CHANNEL2
#  ifdef CONFIG_STM32L4_TIM2_CH2OUT
#    define PWM_TIM2_CH2CFG GPIO_TIM2_CH2OUT
#  else
#    define PWM_TIM2_CH2CFG 0
#  endif
#  define PWM_TIM2_CHANNEL2 1
#else
#  define PWM_TIM2_CHANNEL2 0
#endif
#ifdef CONFIG_STM32L4_TIM2_CHANNEL3
#  ifdef CONFIG_STM32L4_TIM2_CH3OUT
#    define PWM_TIM2_CH3CFG GPIO_TIM2_CH3OUT
#  else
#    define PWM_TIM2_CH3CFG 0
#  endif
#  define PWM_TIM2_CHANNEL3 1
#else
#  define PWM_TIM2_CHANNEL3 0
#endif
#ifdef CONFIG_STM32L4_TIM2_CHANNEL4
#  ifdef CONFIG_STM32L4_TIM2_CH4OUT
#    define PWM_TIM2_CH4CFG GPIO_TIM2_CH4OUT
#  else
#    define PWM_TIM2_CH4CFG 0
#  endif
#  define PWM_TIM2_CHANNEL4 1
#else
#  define PWM_TIM2_CHANNEL4 0
#endif
#define PWM_TIM2_NCHANNELS (PWM_TIM2_CHANNEL1 + PWM_TIM2_CHANNEL2 + \
                            PWM_TIM2_CHANNEL3 + PWM_TIM2_CHANNEL4)

#ifdef CONFIG_STM32L4_TIM3_CHANNEL1
#  ifdef CONFIG_STM32L4_TIM3_CH1OUT
#    define PWM_TIM3_CH1CFG GPIO_TIM3_CH1OUT
#  else
#    define PWM_TIM3_CH1CFG 0
#  endif
#  define PWM_TIM3_CHANNEL1 1
#else
#  define PWM_TIM3_CHANNEL1 0
#endif
#ifdef CONFIG_STM32L4_TIM3_CHANNEL2
#  ifdef CONFIG_STM32L4_TIM3_CH2OUT
#    define PWM_TIM3_CH2CFG GPIO_TIM3_CH2OUT
#  else
#    define PWM_TIM3_CH2CFG 0
#  endif
#  define PWM_TIM3_CHANNEL2 1
#else
#  define PWM_TIM3_CHANNEL2 0
#endif
#ifdef CONFIG_STM32L4_TIM3_CHANNEL3
#  ifdef CONFIG_STM32L4_TIM3_CH3OUT
#    define PWM_TIM3_CH3CFG GPIO_TIM3_CH3OUT
#  else
#    define PWM_TIM3_CH3CFG 0
#  endif
#  define PWM_TIM3_CHANNEL3 1
#else
#  define PWM_TIM3_CHANNEL3 0
#endif
#ifdef CONFIG_STM32L4_TIM3_CHANNEL4
#  ifdef CONFIG_STM32L4_TIM3_CH4OUT
#    define PWM_TIM3_CH4CFG GPIO_TIM3_CH4OUT
#  else
#    define PWM_TIM3_CH4CFG 0
#  endif
#  define PWM_TIM3_CHANNEL4 1
#else
#  define PWM_TIM3_CHANNEL4 0
#endif
#define PWM_TIM3_NCHANNELS (PWM_TIM3_CHANNEL1 + PWM_TIM3_CHANNEL2 + \
                            PWM_TIM3_CHANNEL3 + PWM_TIM3_CHANNEL4)

#ifdef CONFIG_STM32L4_TIM4_CHANNEL1
#  ifdef CONFIG_STM32L4_TIM4_CH1OUT
#    define PWM_TIM4_CH1CFG GPIO_TIM4_CH1OUT
#  else
#    define PWM_TIM4_CH1CFG 0
#  endif
#  define PWM_TIM4_CHANNEL1 1
#else
#  define PWM_TIM4_CHANNEL1 0
#endif
#ifdef CONFIG_STM32L4_TIM4_CHANNEL2
#  ifdef CONFIG_STM32L4_TIM4_CH2OUT
#    define PWM_TIM4_CH2CFG GPIO_TIM4_CH2OUT
#  else
#    define PWM_TIM4_CH2CFG 0
#  endif
#  define PWM_TIM4_CHANNEL2 1
#else
#  define PWM_TIM4_CHANNEL2 0
#endif
#ifdef CONFIG_STM32L4_TIM4_CHANNEL3
#  ifdef CONFIG_STM32L4_TIM4_CH3OUT
#    define PWM_TIM4_CH3CFG GPIO_TIM4_CH3OUT
#  else
#    define PWM_TIM4_CH3CFG 0
#  endif
#  define PWM_TIM4_CHANNEL3 1
#else
#  define PWM_TIM4_CHANNEL3 0
#endif
#ifdef CONFIG_STM32L4_TIM4_CHANNEL4
#  ifdef CONFIG_STM32L4_TIM4_CH4OUT
#    define PWM_TIM4_CH4CFG GPIO_TIM4_CH4OUT
#  else
#    define PWM_TIM4_CH4CFG 0
#  endif
#  define PWM_TIM4_CHANNEL4 1
#else
#  define PWM_TIM4_CHANNEL4 0
#endif
#define PWM_TIM4_NCHANNELS (PWM_TIM4_CHANNEL1 + PWM_TIM4_CHANNEL2 + \
                            PWM_TIM4_CHANNEL3 + PWM_TIM4_CHANNEL4)

#ifdef CONFIG_STM32L4_TIM5_CHANNEL1
#  ifdef CONFIG_STM32L4_TIM5_CH1OUT
#    define PWM_TIM5_CH1CFG GPIO_TIM5_CH1OUT
#  else
#    define PWM_TIM5_CH1CFG 0
#  endif
#  define PWM_TIM5_CHANNEL1 1
#else
#  define PWM_TIM5_CHANNEL1 0
#endif
#ifdef CONFIG_STM32L4_TIM5_CHANNEL2
#  ifdef CONFIG_STM32L4_TIM5_CH2OUT
#    define PWM_TIM5_CH2CFG GPIO_TIM5_CH2OUT
#  else
#    define PWM_TIM5_CH2CFG 0
#  endif
#  define PWM_TIM5_CHANNEL2 1
#else
#  define PWM_TIM5_CHANNEL2 0
#endif
#ifdef CONFIG_STM32L4_TIM5_CHANNEL3
#  ifdef CONFIG_STM32L4_TIM5_CH3OUT
#    define PWM_TIM5_CH3CFG GPIO_TIM5_CH3OUT
#  else
#    define PWM_TIM5_CH3CFG 0
#  endif
#  define PWM_TIM5_CHANNEL3 1
#else
#  define PWM_TIM5_CHANNEL3 0
#endif
#ifdef CONFIG_STM32L4_TIM5_CHANNEL4
#  ifdef CONFIG_STM32L4_TIM5_CH4OUT
#    define PWM_TIM5_CH4CFG GPIO_TIM5_CH4OUT
#  else
#    define PWM_TIM5_CH4CFG 0
#  endif
#  define PWM_TIM5_CHANNEL4 1
#else
#  define PWM_TIM5_CHANNEL4 0
#endif
#define PWM_TIM5_NCHANNELS (PWM_TIM5_CHANNEL1 + PWM_TIM5_CHANNEL2 + \
                            PWM_TIM5_CHANNEL3 + PWM_TIM5_CHANNEL4)

#ifdef CONFIG_STM32L4_TIM8_CHANNEL1
#  ifdef CONFIG_STM32L4_TIM8_CH1OUT
#    define PWM_TIM8_CH1CFG GPIO_TIM8_CH1OUT
#  else
#    define PWM_TIM8_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32L4_TIM8_CH1OUT
#    define PWM_TIM8_CH1NCFG GPIO_TIM8_CH1NOUT
#  else
#    define PWM_TIM8_CH1NCFG 0
#  endif
#  define PWM_TIM8_CHANNEL1 1
#else
#  define PWM_TIM8_CHANNEL1 0
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL2
#  ifdef CONFIG_STM32L4_TIM8_CH2OUT
#    define PWM_TIM8_CH2CFG GPIO_TIM8_CH2OUT
#  else
#    define PWM_TIM8_CH2CFG 0
#  endif
#  ifdef CONFIG_STM32L4_TIM8_CH2NOUT
#    define PWM_TIM8_CH2NCFG GPIO_TIM8_CH2NOUT
#  else
#    define PWM_TIM8_CH2NCFG 0
#  endif
#  define PWM_TIM8_CHANNEL2 1
#else
#  define PWM_TIM8_CHANNEL2 0
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL3
#  ifdef CONFIG_STM32L4_TIM8_CH3OUT
#    define PWM_TIM8_CH3CFG GPIO_TIM8_CH3OUT
#  else
#    define PWM_TIM8_CH3CFG 0
#  endif
#  ifdef CONFIG_STM32L4_TIM8_CH3NOUT
#    define PWM_TIM8_CH3NCFG GPIO_TIM8_CH3NOUT
#  else
#    define PWM_TIM8_CH3NCFG 0
#  endif
#  define PWM_TIM8_CHANNEL3 1
#else
#  define PWM_TIM8_CHANNEL3 0
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL4
#  ifdef CONFIG_STM32L4_TIM8_CH4OUT
#    define PWM_TIM8_CH4CFG GPIO_TIM8_CH4OUT
#  else
#    define PWM_TIM8_CH4CFG 0
#  endif
#  define PWM_TIM8_CHANNEL4 1
#else
#  define PWM_TIM8_CHANNEL4 0
#endif
#define PWM_TIM8_NCHANNELS (PWM_TIM8_CHANNEL1 + PWM_TIM8_CHANNEL2 + \
                            PWM_TIM8_CHANNEL3 + PWM_TIM8_CHANNEL4)

#ifdef CONFIG_STM32L4_TIM15_CHANNEL1
#  ifdef CONFIG_STM32L4_TIM15_CH1OUT
#    define PWM_TIM15_CH1CFG GPIO_TIM15_CH1OUT
#  else
#    define PWM_TIM15_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32L4_TIM15_CH1NOUT
#    define PWM_TIM15_CH1NCFG GPIO_TIM15_CH1NOUT
#  else
#    define PWM_TIM15_CH1NCFG 0
#  endif
#  define PWM_TIM15_CHANNEL1 1
#else
#  define PWM_TIM15_CHANNEL1 0
#endif
#ifdef CONFIG_STM32L4_TIM15_CHANNEL2
#  ifdef CONFIG_STM32L4_TIM15_CH2OUT
#    define PWM_TIM15_CH2CFG GPIO_TIM15_CH2OUT
#  else
#    define PWM_TIM15_CH2CFG 0
#  endif
#  define PWM_TIM15_CHANNEL2 1
#else
#  define PWM_TIM15_CHANNEL2 0
#endif
#define PWM_TIM15_NCHANNELS (PWM_TIM15_CHANNEL1 + PWM_TIM15_CHANNEL2)

#ifdef CONFIG_STM32L4_TIM16_CHANNEL1
#  ifdef CONFIG_STM32L4_TIM16_CH1OUT
#    define PWM_TIM16_CH1CFG GPIO_TIM16_CH1OUT
#  else
#    define PWM_TIM16_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32L4_TIM16_CH1NOUT
#    define PWM_TIM16_CH1NCFG GPIO_TIM16_CH1NOUT
#  else
#    define PWM_TIM16_CH1NCFG 0
#  endif
#  define PWM_TIM16_CHANNEL1 1
#else
#  define PWM_TIM16_CHANNEL1 0
#endif
#define PWM_TIM16_NCHANNELS PWM_TIM16_CHANNEL1

#ifdef CONFIG_STM32L4_TIM17_CHANNEL1
#  ifdef CONFIG_STM32L4_TIM17_CH1OUT
#    define PWM_TIM17_CH1CFG GPIO_TIM17_CH1OUT
#  else
#    define PWM_TIM17_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32L4_TIM17_CH1NOUT
#    define PWM_TIM17_CH1NCFG GPIO_TIM17_CH1NOUT
#  else
#    define PWM_TIM17_CH1NCFG 0
#  endif
#  define PWM_TIM17_CHANNEL1 1
#else
#  define PWM_TIM17_CHANNEL1 0
#endif
#define PWM_TIM17_NCHANNELS PWM_TIM17_CHANNEL1

#define PWM_MAX(a, b) ((a) > (b) ? (a) : (b))

#define PWM_NCHANNELS PWM_MAX(PWM_TIM1_NCHANNELS, \
                      PWM_MAX(PWM_TIM2_NCHANNELS, \
                      PWM_MAX(PWM_TIM3_NCHANNELS, \
                      PWM_MAX(PWM_TIM4_NCHANNELS, \
                      PWM_MAX(PWM_TIM5_NCHANNELS, \
                      PWM_MAX(PWM_TIM8_NCHANNELS, \
                      PWM_MAX(PWM_TIM15_NCHANNELS, \
                      PWM_MAX(PWM_TIM16_NCHANNELS, \
                              PWM_TIM17_NCHANNELS))))))))

#else

/* For each timer that is enabled for PWM usage, we need the following additional
 * configuration settings:
 *
 * CONFIG_STM32L4_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}
 * PWM_TIMx_CHn - One of the values defined in chip/stm32*_pinmap.h.  In the case
 *   where there are multiple pin selections, the correct setting must be provided
 *   in the arch/board/board.h file.
 *
 * NOTE: The STM32L4 timers are each capable of generating different signals on
 * each of the four channels with different duty cycles.  That capability is
 * not supported by this driver:  Only one output channel per timer.
 */

#ifdef CONFIG_STM32L4_TIM1_PWM
#  if !defined(CONFIG_STM32L4_TIM1_CHANNEL)
#    error "CONFIG_STM32L4_TIM1_CHANNEL must be provided"
#  elif CONFIG_STM32L4_TIM1_CHANNEL == 1
#    define CONFIG_STM32L4_TIM1_CHANNEL1 1
#    define CONFIG_STM32L4_TIM1_CH1MODE  CONFIG_STM32L4_TIM1_CHMODE
#    define PWM_TIM1_CH1CFG            GPIO_TIM1_CH1OUT
#    define PWM_TIM1_CH1NCFG           0
#  elif CONFIG_STM32L4_TIM1_CHANNEL == 2
#    define CONFIG_STM32L4_TIM1_CHANNEL2 1
#    define CONFIG_STM32L4_TIM1_CH2MODE  CONFIG_STM32L4_TIM1_CHMODE
#    define PWM_TIM1_CH2CFG            GPIO_TIM1_CH2OUT
#    define PWM_TIM1_CH2NCFG           0
#  elif CONFIG_STM32L4_TIM1_CHANNEL == 3
#    define CONFIG_STM32L4_TIM1_CHANNEL3 1
#    define CONFIG_STM32L4_TIM1_CH3MODE  CONFIG_STM32L4_TIM1_CHMODE
#    define PWM_TIM1_CH3CFG            GPIO_TIM1_CH3OUT
#    define PWM_TIM1_CH3NCFG           0
#  elif CONFIG_STM32L4_TIM1_CHANNEL == 4
#    define CONFIG_STM32L4_TIM1_CHANNEL4 1
#    define CONFIG_STM32L4_TIM1_CH4MODE  CONFIG_STM32L4_TIM1_CHMODE
#    define PWM_TIM1_CH4CFG            GPIO_TIM1_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32L4_TIM1_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32L4_TIM2_PWM
#  if !defined(CONFIG_STM32L4_TIM2_CHANNEL)
#    error "CONFIG_STM32L4_TIM2_CHANNEL must be provided"
#  elif CONFIG_STM32L4_TIM2_CHANNEL == 1
#    define CONFIG_STM32L4_TIM2_CHANNEL1 1
#    define CONFIG_STM32L4_TIM2_CH1MODE  CONFIG_STM32L4_TIM2_CHMODE
#    define PWM_TIM2_CH1CFG            GPIO_TIM2_CH1OUT
#  elif CONFIG_STM32L4_TIM2_CHANNEL == 2
#    define CONFIG_STM32L4_TIM2_CHANNEL2 1
#    define CONFIG_STM32L4_TIM2_CH2MODE  CONFIG_STM32L4_TIM2_CHMODE
#    define PWM_TIM2_CH2CFG            GPIO_TIM2_CH2OUT
#  elif CONFIG_STM32L4_TIM2_CHANNEL == 3
#    define CONFIG_STM32L4_TIM2_CHANNEL3 1
#    define CONFIG_STM32L4_TIM2_CH3MODE  CONFIG_STM32L4_TIM2_CHMODE
#    define PWM_TIM2_CH3CFG            GPIO_TIM2_CH3OUT
#  elif CONFIG_STM32L4_TIM2_CHANNEL == 4
#    define CONFIG_STM32L4_TIM2_CHANNEL4 1
#    define CONFIG_STM32L4_TIM2_CH4MODE  CONFIG_STM32L4_TIM2_CHMODE
#    define PWM_TIM2_CH4CFG            GPIO_TIM2_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32L4_TIM2_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32L4_TIM3_PWM
#  if !defined(CONFIG_STM32L4_TIM3_CHANNEL)
#    error "CONFIG_STM32L4_TIM3_CHANNEL must be provided"
#  elif CONFIG_STM32L4_TIM3_CHANNEL == 1
#    define CONFIG_STM32L4_TIM3_CHANNEL1 1
#    define CONFIG_STM32L4_TIM3_CH1MODE  CONFIG_STM32L4_TIM3_CHMODE
#    define PWM_TIM3_CH1CFG            GPIO_TIM3_CH1OUT
#  elif CONFIG_STM32L4_TIM3_CHANNEL == 2
#    define CONFIG_STM32L4_TIM3_CHANNEL2 1
#    define CONFIG_STM32L4_TIM3_CH2MODE  CONFIG_STM32L4_TIM3_CHMODE
#    define PWM_TIM3_CH2CFG            GPIO_TIM3_CH2OUT
#  elif CONFIG_STM32L4_TIM3_CHANNEL == 3
#    define CONFIG_STM32L4_TIM3_CHANNEL3 1
#    define CONFIG_STM32L4_TIM3_CH3MODE  CONFIG_STM32L4_TIM3_CHMODE
#    define PWM_TIM3_CH3CFG            GPIO_TIM3_CH3OUT
#  elif CONFIG_STM32L4_TIM3_CHANNEL == 4
#    define CONFIG_STM32L4_TIM3_CHANNEL4 1
#    define CONFIG_STM32L4_TIM3_CH4MODE  CONFIG_STM32L4_TIM3_CHMODE
#    define PWM_TIM3_CH4CFG            GPIO_TIM3_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32L4_TIM3_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32L4_TIM4_PWM
#  if !defined(CONFIG_STM32L4_TIM4_CHANNEL)
#    error "CONFIG_STM32L4_TIM4_CHANNEL must be provided"
#  elif CONFIG_STM32L4_TIM4_CHANNEL == 1
#    define CONFIG_STM32L4_TIM4_CHANNEL1 1
#    define CONFIG_STM32L4_TIM4_CH1MODE  CONFIG_STM32L4_TIM4_CHMODE
#    define PWM_TIM4_CH1CFG            GPIO_TIM4_CH1OUT
#  elif CONFIG_STM32L4_TIM4_CHANNEL == 2
#    define CONFIG_STM32L4_TIM4_CHANNEL2 1
#    define CONFIG_STM32L4_TIM4_CH2MODE  CONFIG_STM32L4_TIM4_CHMODE
#    define PWM_TIM4_CH2CFG            GPIO_TIM4_CH2OUT
#  elif CONFIG_STM32L4_TIM4_CHANNEL == 3
#    define CONFIG_STM32L4_TIM4_CHANNEL3 1
#    define CONFIG_STM32L4_TIM4_CH3MODE  CONFIG_STM32L4_TIM4_CHMODE
#    define PWM_TIM4_CH3CFG            GPIO_TIM4_CH3OUT
#  elif CONFIG_STM32L4_TIM4_CHANNEL == 4
#    define CONFIG_STM32L4_TIM4_CHANNEL4 1
#    define CONFIG_STM32L4_TIM4_CH4MODE  CONFIG_STM32L4_TIM4_CHMODE
#    define PWM_TIM4_CH4CFG            GPIO_TIM4_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32L4_TIM4_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32L4_TIM5_PWM
#  if !defined(CONFIG_STM32L4_TIM5_CHANNEL)
#    error "CONFIG_STM32L4_TIM5_CHANNEL must be provided"
#  elif CONFIG_STM32L4_TIM5_CHANNEL == 1
#    define CONFIG_STM32L4_TIM5_CHANNEL1 1
#    define CONFIG_STM32L4_TIM5_CH1MODE  CONFIG_STM32L4_TIM5_CHMODE
#    define PWM_TIM5_CH1CFG            GPIO_TIM5_CH1OUT
#  elif CONFIG_STM32L4_TIM5_CHANNEL == 2
#    define CONFIG_STM32L4_TIM5_CHANNEL2 1
#    define CONFIG_STM32L4_TIM5_CH2MODE  CONFIG_STM32L4_TIM5_CHMODE
#    define PWM_TIM5_CH2CFG            GPIO_TIM5_CH2OUT
#  elif CONFIG_STM32L4_TIM5_CHANNEL == 3
#    define CONFIG_STM32L4_TIM5_CHANNEL3 1
#    define CONFIG_STM32L4_TIM5_CH3MODE  CONFIG_STM32L4_TIM5_CHMODE
#    define PWM_TIM5_CH3CFG            GPIO_TIM5_CH3OUT
#  elif CONFIG_STM32L4_TIM5_CHANNEL == 4
#    define CONFIG_STM32L4_TIM5_CHANNEL4 1
#    define CONFIG_STM32L4_TIM5_CH4MODE  CONFIG_STM32L4_TIM5_CHMODE
#    define PWM_TIM5_CH4CFG            GPIO_TIM5_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32L4_TIM5_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32L4_TIM8_PWM
#  if !defined(CONFIG_STM32L4_TIM8_CHANNEL)
#    error "CONFIG_STM32L4_TIM8_CHANNEL must be provided"
#  elif CONFIG_STM32L4_TIM8_CHANNEL == 1
#    define CONFIG_STM32L4_TIM8_CHANNEL1 1
#    define CONFIG_STM32L4_TIM8_CH1MODE  CONFIG_STM32L4_TIM8_CHMODE
#    define PWM_TIM8_CH1CFG            GPIO_TIM8_CH1OUT
#    define PWM_TIM8_CH1NCFG           0
#  elif CONFIG_STM32L4_TIM8_CHANNEL == 2
#    define CONFIG_STM32L4_TIM8_CHANNEL2 1
#    define CONFIG_STM32L4_TIM8_CH2MODE  CONFIG_STM32L4_TIM8_CHMODE
#    define PWM_TIM8_CH2CFG            GPIO_TIM8_CH2OUT
#    define PWM_TIM8_CH2NCFG           0
#  elif CONFIG_STM32L4_TIM8_CHANNEL == 3
#    define CONFIG_STM32L4_TIM8_CHANNEL3 1
#    define CONFIG_STM32L4_TIM8_CH3MODE  CONFIG_STM32L4_TIM8_CHMODE
#    define PWM_TIM8_CH3CFG            GPIO_TIM8_CH3OUT
#    define PWM_TIM8_CH3NCFG           0
#  elif CONFIG_STM32L4_TIM8_CHANNEL == 4
#    define CONFIG_STM32L4_TIM8_CHANNEL4 1
#    define CONFIG_STM32L4_TIM8_CH4MODE  CONFIG_STM32L4_TIM8_CHMODE
#    define PWM_TIM8_CH4CFG            GPIO_TIM8_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32L4_TIM8_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32L4_TIM15_PWM
#  if !defined(CONFIG_STM32L4_TIM15_CHANNEL)
#    error "CONFIG_STM32L4_TIM15_CHANNEL must be provided"
#  elif CONFIG_STM32L4_TIM15_CHANNEL == 1
#    define CONFIG_STM32L4_TIM15_CHANNEL1 1
#    define CONFIG_STM32L4_TIM15_CH1MODE  CONFIG_STM32L4_TIM15_CHMODE
#    define PWM_TIM15_CH1CFG            GPIO_TIM15_CH1OUT
#    define PWM_TIM15_CH1NCFG           0
#  elif CONFIG_STM32L4_TIM15_CHANNEL == 2
#    define CONFIG_STM32L4_TIM15_CHANNEL2 1
#    define CONFIG_STM32L4_TIM15_CH2MODE  CONFIG_STM32L4_TIM15_CHMODE
#    define PWM_TIM15_CH2CFG            GPIO_TIM15_CH2OUT
#  else
#    error "Unsupported value of CONFIG_STM32L4_TIM15_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32L4_TIM16_PWM
#  if !defined(CONFIG_STM32L4_TIM16_CHANNEL)
#    error "CONFIG_STM32L4_TIM16_CHANNEL must be provided"
#  elif CONFIG_STM32L4_TIM16_CHANNEL == 1
#    define CONFIG_STM32L4_TIM16_CHANNEL1 1
#    define CONFIG_STM32L4_TIM16_CH1MODE  CONFIG_STM32L4_TIM16_CHMODE
#    define PWM_TIM16_CH1CFG            GPIO_TIM16_CH1OUT
#    define PWM_TIM16_CH1NCFG           0
#  else
#    error "Unsupported value of CONFIG_STM32L4_TIM16_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32L4_TIM17_PWM
#  if !defined(CONFIG_STM32L4_TIM17_CHANNEL)
#    error "CONFIG_STM32L4_TIM17_CHANNEL must be provided"
#  elif CONFIG_STM32L4_TIM17_CHANNEL == 1
#    define CONFIG_STM32L4_TIM17_CHANNEL1 1
#    define CONFIG_STM32L4_TIM17_CH1MODE  CONFIG_STM32L4_TIM17_CHMODE
#    define PWM_TIM17_CH1CFG            GPIO_TIM17_CH1OUT
#    define PWM_TIM17_CH1NCFG           0
#  else
#    error "Unsupported value of CONFIG_STM32L4_TIM17_CHANNEL"
#  endif
#endif

#define PWM_NCHANNELS 1

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
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32l4_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,17}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ************************************************************************************/

FAR struct pwm_lowerhalf_s *stm32l4_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32L4_TIMx_PWM */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_PWM_H */
