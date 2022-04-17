/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_pwm.h
 *
 *   Copyright (C) 2019 Fundação CERTI. All rights reserved.
 *   Author: Daniel Pereira Volpato <dpo@certi.org.br>
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_STM32_PWM_H
#define __ARCH_ARM_SRC_STM32F0L0G0_STM32_PWM_H

/* The STM32F0L0G0 does not have dedicated PWM hardware.  Rather, pulsed
 * output control is a capability of the STM32F0L0G0 timers.  The logic in
 * this file implements the lower half of the standard, NuttX PWM interface
 * using the STM32F0L0G0 timers.  That interface is described in
 * include/nuttx/timers/pwm.h.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"
#include "hardware/stm32_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.  If
 * CONFIG_STM32F0L0G0_TIMn is defined then the CONFIG_STM32F0L0G0_TIMn_PWM
 * must also be defined to indicate that timer "n" is intended to be used for
 * pulsed output signal generation.
 */

#ifndef CONFIG_STM32F0L0G0_TIM1
#  undef CONFIG_STM32F0L0G0_TIM1_PWM
#endif
#ifndef CONFIG_STM32F0L0G0_TIM2
#  undef CONFIG_STM32F0L0G0_TIM2_PWM
#endif
#ifndef CONFIG_STM32F0L0G0_TIM3
#  undef CONFIG_STM32F0L0G0_TIM3_PWM
#endif
#ifndef CONFIG_STM32F0L0G0_TIM14
#  undef CONFIG_STM32F0L0G0_TIM14_PWM
#endif
#ifndef CONFIG_STM32F0L0G0_TIM15
#  undef CONFIG_STM32F0L0G0_TIM15_PWM
#endif
#ifndef CONFIG_STM32F0L0G0_TIM16
#  undef CONFIG_STM32F0L0G0_TIM16_PWM
#endif
#ifndef CONFIG_STM32F0L0G0_TIM17
#  undef CONFIG_STM32F0L0G0_TIM17_PWM
#endif

/* The basic timers (timer 6 and 7)
 * are not capable of generating output pulses
 */

#undef CONFIG_STM32F0L0G0_TIM6_PWM
#undef CONFIG_STM32F0L0G0_TIM7_PWM

/* Check if PWM support for any channel is enabled. */

#if defined(CONFIG_STM32F0L0G0_TIM1_PWM)  || defined(CONFIG_STM32F0L0G0_TIM2_PWM)   || \
    defined(CONFIG_STM32F0L0G0_TIM3_PWM)  || defined(CONFIG_STM32F0L0G0_TIM14_PWM)  || \
    defined(CONFIG_STM32F0L0G0_TIM15_PWM) || defined(CONFIG_STM32F0L0G0_TIM16_PWM)  || \
    defined(CONFIG_STM32F0L0G0_TIM17_PWM)

#ifdef CONFIG_PWM_MULTICHAN

#ifdef CONFIG_STM32F0L0G0_TIM1_CHANNEL1
#  ifdef CONFIG_STM32F0L0G0_TIM1_CH1OUT
#    define PWM_TIM1_CH1CFG GPIO_TIM1_CH1OUT
#  else
#    define PWM_TIM1_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32F0L0G0_TIM1_CH1NOUT
#    define PWM_TIM1_CH1NCFG GPIO_TIM1_CH1NOUT
#  else
#    define PWM_TIM1_CH1NCFG 0
#  endif
#  define PWM_TIM1_CHANNEL1 1
#else
#  define PWM_TIM1_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F0L0G0_TIM1_CHANNEL2
#  ifdef CONFIG_STM32F0L0G0_TIM1_CH2OUT
#    define PWM_TIM1_CH2CFG GPIO_TIM1_CH2OUT
#  else
#    define PWM_TIM1_CH2CFG 0
#  endif
#  ifdef CONFIG_STM32F0L0G0_TIM1_CH2NOUT
#    define PWM_TIM1_CH2NCFG GPIO_TIM1_CH2NOUT
#  else
#    define PWM_TIM1_CH2NCFG 0
#  endif
#  define PWM_TIM1_CHANNEL2 1
#else
#  define PWM_TIM1_CHANNEL2 0
#endif
#ifdef CONFIG_STM32F0L0G0_TIM1_CHANNEL3
#  ifdef CONFIG_STM32F0L0G0_TIM1_CH3OUT
#    define PWM_TIM1_CH3CFG GPIO_TIM1_CH3OUT
#  else
#    define PWM_TIM1_CH3CFG 0
#  endif
#  ifdef CONFIG_STM32F0L0G0_TIM1_CH3NOUT
#    define PWM_TIM1_CH3NCFG GPIO_TIM1_CH3NOUT
#  else
#    define PWM_TIM1_CH3NCFG 0
#  endif
#  define PWM_TIM1_CHANNEL3 1
#else
#  define PWM_TIM1_CHANNEL3 0
#endif
#ifdef CONFIG_STM32F0L0G0_TIM1_CHANNEL4
#  ifdef CONFIG_STM32F0L0G0_TIM1_CH4OUT
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

#ifdef CONFIG_STM32F0L0G0_TIM2_CHANNEL1
#  ifdef CONFIG_STM32F0L0G0_TIM2_CH1OUT
#    define PWM_TIM2_CH1CFG GPIO_TIM2_CH1OUT
#  else
#    define PWM_TIM2_CH1CFG 0
#  endif
#  define PWM_TIM2_CHANNEL1 1
#else
#  define PWM_TIM2_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2_CHANNEL2
#  ifdef CONFIG_STM32F0L0G0_TIM2_CH2OUT
#    define PWM_TIM2_CH2CFG GPIO_TIM2_CH2OUT
#  else
#    define PWM_TIM2_CH2CFG 0
#  endif
#  define PWM_TIM2_CHANNEL2 1
#else
#  define PWM_TIM2_CHANNEL2 0
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2_CHANNEL3
#  ifdef CONFIG_STM32F0L0G0_TIM2_CH3OUT
#    define PWM_TIM2_CH3CFG GPIO_TIM2_CH3OUT
#  else
#    define PWM_TIM2_CH3CFG 0
#  endif
#  define PWM_TIM2_CHANNEL3 1
#else
#  define PWM_TIM2_CHANNEL3 0
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2_CHANNEL4
#  ifdef CONFIG_STM32F0L0G0_TIM2_CH4OUT
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

#ifdef CONFIG_STM32F0L0G0_TIM3_CHANNEL1
#  ifdef CONFIG_STM32F0L0G0_TIM3_CH1OUT
#    define PWM_TIM3_CH1CFG GPIO_TIM3_CH1OUT
#  else
#    define PWM_TIM3_CH1CFG 0
#  endif
#  define PWM_TIM3_CHANNEL1 1
#else
#  define PWM_TIM3_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3_CHANNEL2
#  ifdef CONFIG_STM32F0L0G0_TIM3_CH2OUT
#    define PWM_TIM3_CH2CFG GPIO_TIM3_CH2OUT
#  else
#    define PWM_TIM3_CH2CFG 0
#  endif
#  define PWM_TIM3_CHANNEL2 1
#else
#  define PWM_TIM3_CHANNEL2 0
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3_CHANNEL3
#  ifdef CONFIG_STM32F0L0G0_TIM3_CH3OUT
#    define PWM_TIM3_CH3CFG GPIO_TIM3_CH3OUT
#  else
#    define PWM_TIM3_CH3CFG 0
#  endif
#  define PWM_TIM3_CHANNEL3 1
#else
#  define PWM_TIM3_CHANNEL3 0
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3_CHANNEL4
#  ifdef CONFIG_STM32F0L0G0_TIM3_CH4OUT
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

#ifdef CONFIG_STM32F0L0G0_TIM14_CHANNEL1
#  ifdef CONFIG_STM32F0L0G0_TIM14_CH1OUT
#    define PWM_TIM14_CH1CFG GPIO_TIM14_CH1OUT
#  else
#    define PWM_TIM14_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32F0L0G0_TIM14_CH1NOUT
#    define PWM_TIM14_CH1NCFG GPIO_TIM14_CH1NOUT
#  else
#    define PWM_TIM14_CH1NCFG 0
#  endif
#  define PWM_TIM14_CHANNEL1 1
#else
#  define PWM_TIM14_CHANNEL1 0
#endif
#define PWM_TIM14_NCHANNELS PWM_TIM14_CHANNEL1

#ifdef CONFIG_STM32F0L0G0_TIM15_CHANNEL1
#  ifdef CONFIG_STM32F0L0G0_TIM15_CH1OUT
#    define PWM_TIM15_CH1CFG GPIO_TIM15_CH1OUT
#  else
#    define PWM_TIM15_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32F0L0G0_TIM15_CH1NOUT
#    define PWM_TIM15_CH1NCFG GPIO_TIM15_CH1NOUT
#  else
#    define PWM_TIM15_CH1NCFG 0
#  endif
#  define PWM_TIM15_CHANNEL1 1
#else
#  define PWM_TIM15_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F0L0G0_TIM15_CHANNEL2
#  ifdef CONFIG_STM32F0L0G0_TIM15_CH2OUT
#    define PWM_TIM15_CH2CFG GPIO_TIM15_CH2OUT
#  else
#    define PWM_TIM15_CH2CFG 0
#  endif
#  define PWM_TIM15_CHANNEL2 1
#else
#  define PWM_TIM15_CHANNEL2 0
#endif
#define PWM_TIM15_NCHANNELS (PWM_TIM15_CHANNEL1 + PWM_TIM15_CHANNEL2)

#ifdef CONFIG_STM32F0L0G0_TIM16_CHANNEL1
#  ifdef CONFIG_STM32F0L0G0_TIM16_CH1OUT
#    define PWM_TIM16_CH1CFG GPIO_TIM16_CH1OUT
#  else
#    define PWM_TIM16_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32F0L0G0_TIM16_CH1NOUT
#    define PWM_TIM16_CH1NCFG GPIO_TIM16_CH1NOUT
#  else
#    define PWM_TIM16_CH1NCFG 0
#  endif
#  define PWM_TIM16_CHANNEL1 1
#else
#  define PWM_TIM16_CHANNEL1 0
#endif
#define PWM_TIM16_NCHANNELS PWM_TIM16_CHANNEL1

#ifdef CONFIG_STM32F0L0G0_TIM17_CHANNEL1
#  ifdef CONFIG_STM32F0L0G0_TIM17_CH1OUT
#    define PWM_TIM17_CH1CFG GPIO_TIM17_CH1OUT
#  else
#    define PWM_TIM17_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32F0L0G0_TIM17_CH1NOUT
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
                      PWM_MAX(PWM_TIM14_NCHANNELS, \
                      PWM_MAX(PWM_TIM15_NCHANNELS, \
                      PWM_MAX(PWM_TIM16_NCHANNELS, \
                              PWM_TIM17_NCHANNELS))))))

#else  /* !CONFIG_PWM_MULTICHAN */

/* For each timer that is enabled for PWM usage, we need the following
 * additional configuration settings:
 *
 * CONFIG_STM32F0L0G0_TIMx_CHANNEL - Specifies the timer output channel
 * {1,..,4} PWM_TIMx_CHn - One of the values defined in
 * chip/stm32*_pinmap.h.  In the case where there are multiple pin
 * selections, the correct setting must be provided in the arch/board/board.h
 * file.
 *
 * NOTE: The STM32 timers are each capable of generating different signals on
 * each of the four channels with different duty cycles.  That capability is
 * not supported by this driver:  Only one output channel per timer.
 */

#ifdef CONFIG_STM32F0L0G0_TIM1_PWM
#  if !defined(CONFIG_STM32F0L0G0_TIM1_CHANNEL)
#    error "CONFIG_STM32F0L0G0_TIM1_CHANNEL must be provided"
#  elif CONFIG_STM32F0L0G0_TIM1_CHANNEL == 1
#    define CONFIG_STM32F0L0G0_TIM1_CHANNEL1  1
#    define CONFIG_STM32F0L0G0_TIM1_CH1MODE   CONFIG_STM32F0L0G0_TIM1_CHMODE
#    define PWM_TIM1_CH1CFG                   GPIO_TIM1_CH1OUT
#    define PWM_TIM1_CH1NCFG                  0
#  elif CONFIG_STM32F0L0G0_TIM1_CHANNEL == 2
#    define CONFIG_STM32F0L0G0_TIM1_CHANNEL2  1
#    define CONFIG_STM32F0L0G0_TIM1_CH2MODE   CONFIG_STM32F0L0G0_TIM1_CHMODE
#    define PWM_TIM1_CH2CFG                   GPIO_TIM1_CH2OUT
#    define PWM_TIM1_CH2NCFG                  0
#  elif CONFIG_STM32F0L0G0_TIM1_CHANNEL == 3
#    define CONFIG_STM32F0L0G0_TIM1_CHANNEL3  1
#    define CONFIG_STM32F0L0G0_TIM1_CH3MODE   CONFIG_STM32F0L0G0_TIM1_CHMODE
#    define PWM_TIM1_CH3CFG                   GPIO_TIM1_CH3OUT
#    define PWM_TIM1_CH3NCFG                  0
#  elif CONFIG_STM32F0L0G0_TIM1_CHANNEL == 4
#    define CONFIG_STM32F0L0G0_TIM1_CHANNEL4  1
#    define CONFIG_STM32F0L0G0_TIM1_CH4MODE   CONFIG_STM32F0L0G0_TIM1_CHMODE
#    define PWM_TIM1_CH4CFG                   GPIO_TIM1_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32F0L0G0_TIM1_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM2_PWM
#  if !defined(CONFIG_STM32F0L0G0_TIM2_CHANNEL)
#    error "CONFIG_STM32F0L0G0_TIM2_CHANNEL must be provided"
#  elif CONFIG_STM32F0L0G0_TIM2_CHANNEL == 1
#    define CONFIG_STM32F0L0G0_TIM2_CHANNEL1 1
#    define CONFIG_STM32F0L0G0_TIM2_CH1MODE  CONFIG_STM32F0L0G0_TIM2_CHMODE
#    define PWM_TIM2_CH1CFG            GPIO_TIM2_CH1OUT
#  elif CONFIG_STM32F0L0G0_TIM2_CHANNEL == 2
#    define CONFIG_STM32F0L0G0_TIM2_CHANNEL2 1
#    define CONFIG_STM32F0L0G0_TIM2_CH2MODE  CONFIG_STM32F0L0G0_TIM2_CHMODE
#    define PWM_TIM2_CH2CFG            GPIO_TIM2_CH2OUT
#  elif CONFIG_STM32F0L0G0_TIM2_CHANNEL == 3
#    define CONFIG_STM32F0L0G0_TIM2_CHANNEL3 1
#    define CONFIG_STM32F0L0G0_TIM2_CH3MODE  CONFIG_STM32F0L0G0_TIM2_CHMODE
#    define PWM_TIM2_CH3CFG            GPIO_TIM2_CH3OUT
#  elif CONFIG_STM32F0L0G0_TIM2_CHANNEL == 4
#    define CONFIG_STM32F0L0G0_TIM2_CHANNEL4 1
#    define CONFIG_STM32F0L0G0_TIM2_CH4MODE  CONFIG_STM32F0L0G0_TIM2_CHMODE
#    define PWM_TIM2_CH4CFG            GPIO_TIM2_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32F0L0G0_TIM2_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM3_PWM
#  if !defined(CONFIG_STM32F0L0G0_TIM3_CHANNEL)
#    error "CONFIG_STM32F0L0G0_TIM3_CHANNEL must be provided"
#  elif CONFIG_STM32F0L0G0_TIM3_CHANNEL == 1
#    define CONFIG_STM32F0L0G0_TIM3_CHANNEL1 1
#    define CONFIG_STM32F0L0G0_TIM3_CH1MODE  CONFIG_STM32F0L0G0_TIM3_CHMODE
#    define PWM_TIM3_CH1CFG            GPIO_TIM3_CH1OUT
#  elif CONFIG_STM32F0L0G0_TIM3_CHANNEL == 2
#    define CONFIG_STM32F0L0G0_TIM3_CHANNEL2 1
#    define CONFIG_STM32F0L0G0_TIM3_CH2MODE  CONFIG_STM32F0L0G0_TIM3_CHMODE
#    define PWM_TIM3_CH2CFG            GPIO_TIM3_CH2OUT
#  elif CONFIG_STM32F0L0G0_TIM3_CHANNEL == 3
#    define CONFIG_STM32F0L0G0_TIM3_CHANNEL3 1
#    define CONFIG_STM32F0L0G0_TIM3_CH3MODE  CONFIG_STM32F0L0G0_TIM3_CHMODE
#    define PWM_TIM3_CH3CFG            GPIO_TIM3_CH3OUT
#  elif CONFIG_STM32F0L0G0_TIM3_CHANNEL == 4
#    define CONFIG_STM32F0L0G0_TIM3_CHANNEL4 1
#    define CONFIG_STM32F0L0G0_TIM3_CH4MODE  CONFIG_STM32F0L0G0_TIM3_CHMODE
#    define PWM_TIM3_CH4CFG            GPIO_TIM3_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32F0L0G0_TIM3_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM14_PWM
#  if !defined(CONFIG_STM32F0L0G0_TIM14_CHANNEL)
#    error "CONFIG_STM32F0L0G0_TIM14_CHANNEL must be provided"
#  elif CONFIG_STM32F0L0G0_TIM14_CHANNEL == 1
#    define CONFIG_STM32F0L0G0_TIM14_CHANNEL1 1
#    define CONFIG_STM32F0L0G0_TIM14_CH1MODE  CONFIG_STM32F0L0G0_TIM14_CHMODE
#    define PWM_TIM14_CH1CFG            GPIO_TIM14_CH1OUT
#    define PWM_TIM14_CH1NCFG           0
#  else
#    error "Unsupported value of CONFIG_STM32F0L0G0_TIM14_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15_PWM
#  if !defined(CONFIG_STM32F0L0G0_TIM15_CHANNEL)
#    error "CONFIG_STM32F0L0G0_TIM15_CHANNEL must be provided"
#  elif CONFIG_STM32F0L0G0_TIM15_CHANNEL == 1
#    define CONFIG_STM32F0L0G0_TIM15_CHANNEL1 1
#    define CONFIG_STM32F0L0G0_TIM15_CH1MODE  CONFIG_STM32F0L0G0_TIM15_CHMODE
#    define PWM_TIM15_CH1CFG            GPIO_TIM15_CH1OUT
#    define PWM_TIM15_CH1NCFG           0
#  elif CONFIG_STM32F0L0G0_TIM15_CHANNEL == 2
#    define CONFIG_STM32F0L0G0_TIM15_CHANNEL2 1
#    define CONFIG_STM32F0L0G0_TIM15_CH2MODE  CONFIG_STM32F0L0G0_TIM15_CHMODE
#    define PWM_TIM15_CH2CFG            GPIO_TIM15_CH2OUT
#  else
#    error "Unsupported value of CONFIG_STM32F0L0G0_TIM15_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16_PWM
#  if !defined(CONFIG_STM32F0L0G0_TIM16_CHANNEL)
#    error "CONFIG_STM32F0L0G0_TIM16_CHANNEL must be provided"
#  elif CONFIG_STM32F0L0G0_TIM16_CHANNEL == 1
#    define CONFIG_STM32F0L0G0_TIM16_CHANNEL1 1
#    define CONFIG_STM32F0L0G0_TIM16_CH1MODE  CONFIG_STM32F0L0G0_TIM16_CHMODE
#    define PWM_TIM16_CH1CFG            GPIO_TIM16_CH1OUT
#    define PWM_TIM16_CH1NCFG           0
#  else
#    error "Unsupported value of CONFIG_STM32F0L0G0_TIM16_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17_PWM
#  if !defined(CONFIG_STM32F0L0G0_TIM17_CHANNEL)
#    error "CONFIG_STM32F0L0G0_TIM17_CHANNEL must be provided"
#  elif CONFIG_STM32F0L0G0_TIM17_CHANNEL == 1
#    define CONFIG_STM32F0L0G0_TIM17_CHANNEL1 1
#    define CONFIG_STM32F0L0G0_TIM17_CH1MODE  CONFIG_STM32F0L0G0_TIM17_CHMODE
#    define PWM_TIM17_CH1CFG            GPIO_TIM17_CH1OUT
#    define PWM_TIM17_CH1NCFG           0
#  else
#    error "Unsupported value of CONFIG_STM32F0L0G0_TIM17_CHANNEL"
#  endif
#endif

#define PWM_NCHANNELS 1

#endif

/* Complementary outputs support */

#if defined(CONFIG_STM32F0L0G0_TIM1_CH1NOUT) || defined(CONFIG_STM32F0L0G0_TIM1_CH2NOUT) || \
    defined(CONFIG_STM32F0L0G0_TIM1_CH3NOUT)
#  define HAVE_TIM1_COMPLEMENTARY
#endif
#if defined(CONFIG_STM32F0L0G0_TIM15_CH1NOUT)
#  define HAVE_TIM15_COMPLEMENTARY
#endif
#if defined(CONFIG_STM32F0L0G0_TIM16_CH1NOUT)
#  define HAVE_TIM16_COMPLEMENTARY
#endif
#if defined(CONFIG_STM32F0L0G0_TIM17_CH1NOUT)
#  define HAVE_TIM17_COMPLEMENTARY
#endif
#if defined(HAVE_TIM1_COMPLEMENTARY)  || defined(HAVE_TIM8_COMPLEMENTARY)  || \
    defined(HAVE_TIM15_COMPLEMENTARY) || defined(HAVE_TIM16_COMPLEMENTARY) || \
    defined(HAVE_TIM17_COMPLEMENTARY)
#  define HAVE_PWM_COMPLEMENTARY
#endif

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
 * Name: stm32_pwminitialize
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
 ****************************************************************************/

struct pwm_lowerhalf_s *stm32_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32F0L0G0_TIMx_PWM */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_STM32_PWM_H */
