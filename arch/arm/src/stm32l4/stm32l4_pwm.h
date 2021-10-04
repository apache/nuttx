/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pwm.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_PWM_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_PWM_H

/* The STM32L4 does not have dedicated PWM hardware.  Rather, pulsed output
 * control is a capability of the STM32L4 timers.  The logic in this file
 * implements the lower half of the standard, NuttX PWM interface using the
 * STM32L4 timers. That interface is described in include/nuttx/timers/pwm.h.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/timers/pwm.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.  If
 * CONFIG_STM32L4_TIMn is defined then the CONFIG_STM32L4_TIMn_PWM must also
 * be defined to indicate that timer "n" is intended to be used for pulsed
 * output signal generation.
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
#ifndef CONFIG_STM32L4_LPTIM1
#  undef CONFIG_STM32L4_LPTIM1_PWM
#endif
#ifndef CONFIG_STM32L4_LPTIM2
#  undef CONFIG_STM32L4_LPTIM2_PWM
#endif

/* The basic timers (timer 6 and 7) are not capable of generating output
 *  pulses.
 */

#undef CONFIG_STM32L4_TIM6_PWM
#undef CONFIG_STM32L4_TIM7_PWM

/* Check if PWM support for any channel is enabled. */

#if defined(CONFIG_STM32L4_TIM1_PWM)  || defined(CONFIG_STM32L4_TIM2_PWM)   || \
    defined(CONFIG_STM32L4_TIM3_PWM)  || defined(CONFIG_STM32L4_TIM4_PWM)   || \
    defined(CONFIG_STM32L4_TIM5_PWM)  || defined(CONFIG_STM32L4_TIM8_PWM)   || \
    defined(CONFIG_STM32L4_TIM15_PWM) || defined(CONFIG_STM32L4_TIM16_PWM)  || \
    defined(CONFIG_STM32L4_TIM17_PWM) || defined(CONFIG_STM32L4_LPTIM1_PWM) || \
    defined(CONFIG_STM32L4_LPTIM2_PWM)

#include <arch/board/board.h>
#include "hardware/stm32l4_tim.h"
#include "hardware/stm32l4_lptim.h"

/* PWM driver channels configuration */

#ifdef CONFIG_STM32L4_PWM_MULTICHAN

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

#ifdef CONFIG_STM32L4_LPTIM1_CHANNEL1
#  ifdef CONFIG_STM32L4_LPTIM1_CH1OUT
#    define PWM_LPTIM1_CH1CFG GPIO_LPTIM1_CH1OUT
#  else
#    define PWM_LPTIM1_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32L4_LPTIM1_CH1NOUT
#    define PWM_LPTIM1_CH1NCFG GPIO_LPTIM1_CH1NOUT
#  else
#    define PWM_LPTIM1_CH1NCFG 0
#  endif
#  define PWM_LPTIM1_CHANNEL1 1
#else
#  define PWM_LPTIM1_CHANNEL1 0
#endif
#define PWM_LPTIM1_NCHANNELS PWM_LPTIM1_CHANNEL1

#ifdef CONFIG_STM32L4_LPTIM2_CHANNEL1
#  ifdef CONFIG_STM32L4_LPTIM2_CH1OUT
#    define PWM_LPTIM2_CH1CFG GPIO_LPTIM2_CH1OUT
#  else
#    define PWM_LPTIM2_CH1CFG 0
#  endif
#  ifdef CONFIG_STM32L4_LPTIM2_CH1NOUT
#    define PWM_LPTIM2_CH1NCFG GPIO_LPTIM2_CH1NOUT
#  else
#    define PWM_LPTIM2_CH1NCFG 0
#  endif
#  define PWM_LPTIM2_CHANNEL1 1
#else
#  define PWM_LPTIM2_CHANNEL1 0
#endif
#define PWM_LPTIM2_NCHANNELS PWM_LPTIM2_CHANNEL1

#else /* !CONFIG_STM32L4_PWM_MULTICHAN */

/* For each timer that is enabled for PWM usage, we need the following
 * additional configuration settings:
 *
 * CONFIG_STM32L4_TIMx_CHANNEL - Specifies the timer output channel
 * {1,..,4}
 * PWM_TIMx_CHn - One of the values defined in chip/stm32*_pinmap.h.
 *   In the case where there are multiple pin selections, the correct
 *   setting must be provided in the arch/board/board.h file.
 *
 * NOTE:
 * The STM32L4 timers are each capable of generating different signals
 * on each of the four channels with different duty cycles.  That
 * capability is not supported by this driver:
 *  Only one output channel per timer.
 */

#ifdef CONFIG_STM32L4_TIM1_PWM
#  if !defined(CONFIG_STM32L4_TIM1_CHANNEL)
#    error "CONFIG_STM32L4_TIM1_CHANNEL must be provided"
#  elif CONFIG_STM32L4_TIM1_CHANNEL == 1
#    define CONFIG_STM32L4_TIM1_CHANNEL1 1
#    define CONFIG_STM32L4_TIM1_CH1MODE  CONFIG_STM32L4_TIM1_CHMODE
#    ifdef CONFIG_STM32L4_TIM1_CH1OUT
#      define PWM_TIM1_CH1CFG            GPIO_TIM1_CH1OUT
#    endif
#    ifdef CONFIG_STM32L4_TIM1_CH1NOUT
#      define PWM_TIM1_CH1NCFG           GPIO_TIM1_CH1NOUT
#    else
#      define PWM_TIM1_CH1NCFG           0
#    endif
#  elif CONFIG_STM32L4_TIM1_CHANNEL == 2
#    define CONFIG_STM32L4_TIM1_CHANNEL2 1
#    define CONFIG_STM32L4_TIM1_CH2MODE  CONFIG_STM32L4_TIM1_CHMODE
#    ifdef CONFIG_STM32L4_TIM1_CH2OUT
#      define PWM_TIM1_CH2CFG            GPIO_TIM1_CH2OUT
#    endif
#    ifdef CONFIG_STM32L4_TIM1_CH2NOUT
#      define PWM_TIM1_CH2NCFG           GPIO_TIM1_CH2NOUT
#    else
#      define PWM_TIM1_CH2NCFG           0
#    endif
#  elif CONFIG_STM32L4_TIM1_CHANNEL == 3
#    define CONFIG_STM32L4_TIM1_CHANNEL3 1
#    define CONFIG_STM32L4_TIM1_CH3MODE  CONFIG_STM32L4_TIM1_CHMODE
#    ifdef CONFIG_STM32L4_TIM1_CH3OUT
#      define PWM_TIM1_CH3CFG            GPIO_TIM1_CH3OUT
#    endif
#    ifdef CONFIG_STM32L4_TIM1_CH3NOUT
#      define PWM_TIM1_CH3NCFG           GPIO_TIM1_CH3NOUT
#    else
#      define PWM_TIM1_CH3NCFG           0
#    endif
#  elif CONFIG_STM32L4_TIM1_CHANNEL == 4
#    define CONFIG_STM32L4_TIM1_CHANNEL4 1
#    define CONFIG_STM32L4_TIM1_CH4MODE  CONFIG_STM32L4_TIM1_CHMODE
#    ifdef CONFIG_STM32L4_TIM1_CH4OUT
#      define PWM_TIM1_CH4CFG            GPIO_TIM1_CH4OUT
#    endif
#  else
#    error "Unsupported value of CONFIG_STM32L4_TIM1_CHANNEL"
#  endif
#  define PWM_TIM1_NCHANNELS 1
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
#  define PWM_TIM2_NCHANNELS 1
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
#  define PWM_TIM3_NCHANNELS 1
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
#  define PWM_TIM4_NCHANNELS 1
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
#  define PWM_TIM5_NCHANNELS 1
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
#  define PWM_TIM8_NCHANNELS 1
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
#  define PWM_TIM15_NCHANNELS 1
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
#  define PWM_TIM16_NCHANNELS 1
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
#  define PWM_TIM17_NCHANNELS 1
#endif

#ifdef CONFIG_STM32L4_LPTIM1_PWM
#  if !defined(CONFIG_STM32L4_LPTIM1_CHANNEL)
#    error "CONFIG_STM32L4_LPTIM1_CHANNEL must be provided"
#  elif CONFIG_STM32L4_LPTIM1_CHANNEL == 1
#    define CONFIG_STM32L4_LPTIM1_CHANNEL1  1
#    define PWM_LPTIM1_CH1CFG               GPIO_LPTIM1_CH1OUT
#    define PWM_LPTIM1_CH1NCFG              0
#  else
#    error "Unsupported value of CONFIG_STM32L4_LPTIM1_CHANNEL"
#  endif
#  define PWM_LPTIM1_NCHANNELS 1
#endif

#ifdef CONFIG_STM32L4_LPTIM2_PWM
#  if !defined(CONFIG_STM32L4_LPTIM2_CHANNEL)
#    error "CONFIG_STM32L4_LPTIM2_CHANNEL must be provided"
#  elif CONFIG_STM32L4_LPTIM2_CHANNEL == 1
#    define CONFIG_STM32L4_LPTIM2_CHANNEL1  1
#    define PWM_LPTIM2_CH1CFG               GPIO_LPTIM2_CH1OUT
#    define PWM_LPTIM2_CH1NCFG              0
#  else
#    error "Unsupported value of CONFIG_STM32L4_LPTIM2_CHANNEL"
#  endif
#  define PWM_LPTIM2_NCHANNELS 1
#endif

#endif

/* Complementary outputs support */

#if defined(CONFIG_STM32L4_TIM1_CH1NOUT) || defined(CONFIG_STM32L4_TIM1_CH2NOUT) || \
    defined(CONFIG_STM32L4_TIM1_CH3NOUT)
#  define HAVE_TIM1_COMPLEMENTARY
#endif
#if defined(CONFIG_STM32L4_TIM8_CH1NOUT) || defined(CONFIG_STM32L4_TIM8_CH2NOUT) || \
    defined(CONFIG_STM32L4_TIM8_CH3NOUT)
#  define HAVE_TIM8_COMPLEMENTARY
#endif
#if defined(CONFIG_STM32L4_TIM15_CH1NOUT)
#  define HAVE_TIM15_COMPLEMENTARY
#endif
#if defined(CONFIG_STM32L4_TIM16_CH1NOUT)
#  define HAVE_TIM16_COMPLEMENTARY
#endif
#if defined(CONFIG_STM32L4_TIM17_CH1NOUT)
#  define HAVE_TIM17_COMPLEMENTARY
#endif
#if defined(CONFIG_STM32L4_LPTIM1_CH1NOUT)
#  define HAVE_LPTIM1_COMPLEMENTARY
#endif
#if defined(CONFIG_STM32L4_LPTIM2_CH1NOUT)
#  define HAVE_LPTIM2_COMPLEMENTARY
#endif
#if defined(HAVE_TIM1_COMPLEMENTARY) || defined(HAVE_TIM8_COMPLEMENTARY) ||   \
    defined(HAVE_TIM15_COMPLEMENTARY) || defined(HAVE_TIM16_COMPLEMENTARY) || \
    defined(HAVE_TIM17_COMPLEMENTARY) || defined(HAVE_LPTIM1_COMPLEMENTARY) || \
    defined(HAVE_LPTIM2_COMPLEMENTARY)
#  define HAVE_PWM_COMPLEMENTARY
#endif

/* Low-level ops helpers ****************************************************/

#ifdef CONFIG_STM32L4_PWM_LL_OPS

/* NOTE: low-level ops accept pwm_lowerhalf_s as first argument, but llops
 *       access can be found in stm32l4_pwm_dev_s
 */

#define PWM_SETUP(dev)                                                             \
        (dev)->ops->setup((FAR struct pwm_lowerhalf_s *)dev)
#define PWM_SHUTDOWN(dev)                                                          \
        (dev)->ops->shutdown((FAR struct pwm_lowerhalf_s *)dev)
#define PWM_CCR_UPDATE(dev, index, ccr)                                            \
        (dev)->llops->ccr_update((FAR struct pwm_lowerhalf_s *)dev, index, ccr)
#define PWM_MODE_UPDATE(dev, index, mode)                                          \
        (dev)->llops->mode_update((FAR struct pwm_lowerhalf_s *)dev, index, mode)
#define PWM_CCR_GET(dev, index)                                                    \
        (dev)->llops->ccr_get((FAR struct pwm_lowerhalf_s *)dev, index)
#define PWM_ARR_UPDATE(dev, arr)                                                   \
        (dev)->llops->arr_update((FAR struct pwm_lowerhalf_s *)dev, arr)
#define PWM_ARR_GET(dev)                                                           \
        (dev)->llops->arr_get((FAR struct pwm_lowerhalf_s *)dev)
#define PWM_OUTPUTS_ENABLE(dev, out, state)                                        \
        (dev)->llops->outputs_enable((FAR struct pwm_lowerhalf_s *)dev, out, state)
#define PWM_SOFT_UPDATE(dev)                                                       \
        (dev)->llops->soft_update((FAR struct pwm_lowerhalf_s *)dev)
#define PWM_CONFIGURE(dev)                                                         \
        (dev)->llops->configure((FAR struct pwm_lowerhalf_s *)dev)
#define PWM_SOFT_BREAK(dev, state)                                                 \
        (dev)->llops->soft_break((FAR struct pwm_lowerhalf_s *)dev, state)
#define PWM_FREQ_UPDATE(dev, freq)                                                 \
        (dev)->llops->freq_update((FAR struct pwm_lowerhalf_s *)dev, freq)
#define PWM_TIM_ENABLE(dev, state)                                                 \
        (dev)->llops->tim_enable((FAR struct pwm_lowerhalf_s *)dev, state)
#ifdef CONFIG_DEBUG_STM32L4_PWM_INFO
#  define PWM_DUMP_REGS(dev, msg)                                                  \
        (dev)->llops->dump_regs((FAR struct pwm_lowerhalf_s *)dev, msg)
#else
#  define PWM_DUMP_REGS(dev, msg)
#endif
#define PWM_DT_UPDATE(dev, dt)                                                     \
        (dev)->llops->dt_update((FAR struct pwm_lowerhalf_s *)dev, dt)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Timer mode */

enum stm32l4_timmode_e
{
  STM32L4_TIMMODE_COUNTUP   = 0,
  STM32L4_TIMMODE_COUNTDOWN = 1,
  STM32L4_TIMMODE_CENTER1   = 2,
  STM32L4_TIMMODE_CENTER2   = 3,
  STM32L4_TIMMODE_CENTER3   = 4,
};

/* Timer output polarity */

enum stm32l4_pwm_pol_e
{
  STM32L4_POL_POS  = 0,
  STM32L4_POL_NEG  = 1,
};

/* Timer output IDLE state */

enum stm32l4_pwm_idle_e
{
  STM32L4_IDLE_INACTIVE = 0,
  STM32L4_IDLE_ACTIVE   = 1
};

/* PWM channel mode */

enum stm32l4_chanmode_e
{
  STM32L4_CHANMODE_FRZN        = 0,  /* CCRx matches has no effects on outputs */
  STM32L4_CHANMODE_CHACT       = 1,  /* OCxREF active on match */
  STM32L4_CHANMODE_CHINACT     = 2,  /* OCxREF inactive on match */
  STM32L4_CHANMODE_OCREFTOG    = 3,  /* OCxREF toggles when TIMy_CNT=TIMyCCRx */
  STM32L4_CHANMODE_OCREFLO     = 4,  /* OCxREF is forced low */
  STM32L4_CHANMODE_OCREFHI     = 5,  /* OCxREF is forced high */
  STM32L4_CHANMODE_PWM1        = 6,  /* PWM mode 1 */
  STM32L4_CHANMODE_PWM2        = 7,  /* PWM mode 2 */
  STM32L4_CHANMODE_COMBINED1   = 8,  /* Combined PWM mode 1 */
  STM32L4_CHANMODE_COMBINED2   = 9,  /* Combined PWM mode 2 */
  STM32L4_CHANMODE_ASYMMETRIC1 = 10, /* Asymmetric PWM mode 1 */
  STM32L4_CHANMODE_ASYMMETRIC2 = 11, /* Asymmetric PWM mode 2 */
};

/* PWM timer channel */

enum stm32l4_pwm_chan_e
{
  STM32L4_PWM_CHAN1  = 1,
  STM32L4_PWM_CHAN2  = 2,
  STM32L4_PWM_CHAN3  = 3,
  STM32L4_PWM_CHAN4  = 4,
  STM32L4_PWM_CHAN5  = 5,
  STM32L4_PWM_CHAN6  = 6,
};

/* PWM timer channel output */

enum stm32l4_pwm_output_e
{
  STM32L4_PWM_OUT1  = (1 << 0),
  STM32L4_PWM_OUT1N = (1 << 1),
  STM32L4_PWM_OUT2  = (1 << 2),
  STM32L4_PWM_OUT2N = (1 << 3),
  STM32L4_PWM_OUT3  = (1 << 4),
  STM32L4_PWM_OUT3N = (1 << 5),
  STM32L4_PWM_OUT4  = (1 << 6),

  /* 1 << 7 reserved - no complementary output for CH4 */

  /* Only available inside micro */

  STM32L4_PWM_OUT5  = (1 << 8),

  /* 1 << 9 reserved - no complementary output for CH5 */

  STM32L4_PWM_OUT6  = (1 << 10),

  /* 1 << 11 reserved - no complementary output for CH6 */
};

#ifdef CONFIG_STM32L4_PWM_LL_OPS

/* This structure provides the publicly visible representation of the
 * "lower-half" PWM driver structure.
 */

struct stm32l4_pwm_dev_s
{
  /* The first field of this state structure must be a pointer to the PWM
   * callback structure to be consistent with upper-half PWM driver.
   */

  FAR const struct pwm_ops_s *ops;

  /* Publicly visible portion of the "lower-half" PWM driver structure */

  FAR const struct stm32l4_pwm_ops_s *llops;

  /* Require cast-compatibility with private "lower-half" PWM structure */
};

/* Low-level operations for PWM */

struct pwm_lowerhalf_s;
struct stm32l4_pwm_ops_s
{
  /* Update CCR register */

  int (*ccr_update)(FAR struct pwm_lowerhalf_s *dev,
                    uint8_t index, uint32_t ccr);

  /* Update PWM mode */

  int (*mode_update)(FAR struct pwm_lowerhalf_s *dev,
                     uint8_t index, uint32_t mode);

  /* Get CCR register */

  uint32_t (*ccr_get)(FAR struct pwm_lowerhalf_s *dev, uint8_t index);

  /* Update ARR register */

  int (*arr_update)(FAR struct pwm_lowerhalf_s *dev, uint32_t arr);

  /* Get ARR register */

  uint32_t (*arr_get)(FAR struct pwm_lowerhalf_s *dev);

  /* Enable outputs */

  int (*outputs_enable)(FAR struct pwm_lowerhalf_s *dev,
                        uint16_t outputs, bool state);

  /* Software update */

  int (*soft_update)(FAR struct pwm_lowerhalf_s *dev);

  /* PWM configure */

  int (*configure)(FAR struct pwm_lowerhalf_s *dev);

  /* Software break */

  int (*soft_break)(FAR struct pwm_lowerhalf_s *dev, bool state);

  /* Update frequency */

  int (*freq_update)(FAR struct pwm_lowerhalf_s *dev, uint32_t frequency);

  /* Enable timer counter */

  int (*tim_enable)(FAR struct pwm_lowerhalf_s *dev, bool state);

#ifdef CONFIG_DEBUG_PWM_INFO
  /* Dump timer registers */

  void (*dump_regs)(FAR struct pwm_lowerhalf_s *dev, FAR const char *msg);
#endif

#ifdef HAVE_PWM_COMPLEMENTARY
  /* Deadtime update */

  int (*dt_update)(FAR struct pwm_lowerhalf_s *dev, uint8_t dt);
#endif
};

#endif /* CONFIG_STM32L4_PWM_LL_OPS */

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
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *stm32l4_pwminitialize(int timer);

/****************************************************************************
 * Name: stm32l4_lp_pwminitialize
 *
 * Description:
 *   Initialize one low-power timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,2}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *stm32l4_lp_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32L4_TIMx_PWM */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_PWM_H */
