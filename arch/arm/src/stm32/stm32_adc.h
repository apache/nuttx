/************************************************************************************
 * arch/arm/src/stm32/stm32_adc.h
 *
 *   Copyright (C) 2009, 2011, 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_ADC_H
#define __ARCH_ARM_SRC_STM32_STM32_ADC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_STM32_STM32F30XX)
#  include "chip/stm32f30xxx_adc.h"
#else
#  include "chip/stm32_adc.h"
#endif

#include <nuttx/analog/adc.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Timer devices may be used for different purposes.  One special purpose is to
 * control periodic ADC sampling.  If CONFIG_STM32_TIMn is defined then
 * CONFIG_STM32_TIMn_ADC must also be defined to indicate that timer "n" is intended
 * to be used for that purpose.
 */

/* For the STM32 F1 line, timers 1-4 may be used.  For STM32 F4 line, timers 1-5 and
 * 8 may be used. For the STM32L15XX line, timers 2-4, 6, 7, 9, 10 may be used
 */

#ifndef CONFIG_STM32_TIM1
#  undef CONFIG_STM32_TIM1_ADC
#  undef CONFIG_STM32_TIM1_ADC1
#  undef CONFIG_STM32_TIM1_ADC2
#  undef CONFIG_STM32_TIM1_ADC3
#endif
#ifndef CONFIG_STM32_TIM2
#  undef CONFIG_STM32_TIM2_ADC
#  undef CONFIG_STM32_TIM2_ADC1
#  undef CONFIG_STM32_TIM2_ADC2
#  undef CONFIG_STM32_TIM2_ADC3
#endif
#ifndef CONFIG_STM32_TIM3
#  undef CONFIG_STM32_TIM3_ADC
#  undef CONFIG_STM32_TIM3_ADC1
#  undef CONFIG_STM32_TIM3_ADC2
#  undef CONFIG_STM32_TIM3_ADC3
#endif
#ifndef CONFIG_STM32_TIM4
#  undef CONFIG_STM32_TIM4_ADC
#  undef CONFIG_STM32_TIM4_ADC1
#  undef CONFIG_STM32_TIM4_ADC2
#  undef CONFIG_STM32_TIM4_ADC3
#endif

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  ifndef CONFIG_STM32_TIM5
#    undef CONFIG_STM32_TIM5_ADC
#    undef CONFIG_STM32_TIM5_ADC1
#    undef CONFIG_STM32_TIM5_ADC2
#    undef CONFIG_STM32_TIM5_ADC3
#  endif
#  ifndef CONFIG_STM32_TIM8
#    undef CONFIG_STM32_TIM8_ADC
#    undef CONFIG_STM32_TIM8_ADC1
#    undef CONFIG_STM32_TIM8_ADC2
#    undef CONFIG_STM32_TIM8_ADC3
#  endif
#
#else
#  undef CONFIG_STM32_TIM5_ADC
#  undef CONFIG_STM32_TIM5_ADC1
#  undef CONFIG_STM32_TIM5_ADC2
#  undef CONFIG_STM32_TIM5_ADC3
#  undef CONFIG_STM32_TIM8_ADC
#  undef CONFIG_STM32_TIM8_ADC1
#  undef CONFIG_STM32_TIM8_ADC2
#  undef CONFIG_STM32_TIM8_ADC3
#endif

/* Timers 6, 7, 9, 10 used by STM32L15XX family devices. Though there is only ADC
 * presented in specification and in device as well, the ADC1 is used here in code.
 * See definition of the STM32_NADC
 */

#if defined(CONFIG_STM32_STM32L15XX)
#  ifndef CONFIG_STM32_TIM6
#    undef CONFIG_STM32_TIM6_ADC
#    undef CONFIG_STM32_TIM6_ADC1
#    undef CONFIG_STM32_TIM6_ADC2
#    undef CONFIG_STM32_TIM6_ADC3
#  endif
#  ifndef CONFIG_STM32_TIM7
#    undef CONFIG_STM32_TIM7_ADC
#    undef CONFIG_STM32_TIM7_ADC1
#    undef CONFIG_STM32_TIM7_ADC2
#    undef CONFIG_STM32_TIM7_ADC3
#  endif
#  ifndef CONFIG_STM32_TIM9
#    undef CONFIG_STM32_TIM9_ADC
#    undef CONFIG_STM32_TIM9_ADC1
#    undef CONFIG_STM32_TIM9_ADC2
#    undef CONFIG_STM32_TIM9_ADC3
#  endif
#  ifndef CONFIG_STM32_TIM10
#    undef CONFIG_STM32_TIM10_ADC
#    undef CONFIG_STM32_TIM10_ADC1
#    undef CONFIG_STM32_TIM10_ADC2
#    undef CONFIG_STM32_TIM10_ADC3
#  endif
#
#else
#  undef CONFIG_STM32_TIM6_ADC
#  undef CONFIG_STM32_TIM6_ADC1
#  undef CONFIG_STM32_TIM6_ADC2
#  undef CONFIG_STM32_TIM6_ADC3
#  undef CONFIG_STM32_TIM7_ADC
#  undef CONFIG_STM32_TIM7_ADC1
#  undef CONFIG_STM32_TIM7_ADC2
#  undef CONFIG_STM32_TIM7_ADC3
#  undef CONFIG_STM32_TIM9_ADC
#  undef CONFIG_STM32_TIM9_ADC1
#  undef CONFIG_STM32_TIM9_ADC2
#  undef CONFIG_STM32_TIM9_ADC3
#  undef CONFIG_STM32_TIM10_ADC
#  undef CONFIG_STM32_TIM10_ADC1
#  undef CONFIG_STM32_TIM10_ADC2
#  undef CONFIG_STM32_TIM10_ADC3
#
#endif

/* Timers 6, 7, and 10-14 are not used with the ADC by any supported family
 */

#undef CONFIG_STM32_TIM11_ADC
#undef CONFIG_STM32_TIM11_ADC1
#undef CONFIG_STM32_TIM11_ADC2
#undef CONFIG_STM32_TIM11_ADC3
#undef CONFIG_STM32_TIM12_ADC
#undef CONFIG_STM32_TIM12_ADC1
#undef CONFIG_STM32_TIM12_ADC2
#undef CONFIG_STM32_TIM12_ADC3
#undef CONFIG_STM32_TIM13_ADC
#undef CONFIG_STM32_TIM13_ADC1
#undef CONFIG_STM32_TIM13_ADC2
#undef CONFIG_STM32_TIM13_ADC3
#undef CONFIG_STM32_TIM14_ADC
#undef CONFIG_STM32_TIM14_ADC1
#undef CONFIG_STM32_TIM14_ADC2
#undef CONFIG_STM32_TIM14_ADC3

/* Up to 3 ADC interfaces are supported */

#if STM32_NADC < 3
#  undef CONFIG_STM32_ADC3
#endif

#if STM32_NADC < 2
#  undef CONFIG_STM32_ADC2
#endif

#if STM32_NADC < 1
#  undef CONFIG_STM32_ADC1
#endif

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2) || \
    defined(CONFIG_STM32_ADC3)

/* DMA support */

#undef ADC_HAVE_DMA
#if defined(CONFIG_STM32_ADC1_DMA) || defined(CONFIG_STM32_ADC2_DMA) || \
    defined(CONFIG_STM32_ADC3_DMA) || defined(CONFIG_STM32_ADC4_DMA)
# if defined(CONFIG_STM32_STM32F40XX)
#   define ADC_HAVE_DMA        1
#else
#    warning DMA is only supported for the stm32f40xx family
# endif
#endif

#ifdef CONFIG_STM32_ADC1_DMA
#  define ADC1_HAVE_DMA 1
#else
#  undef  ADC1_HAVE_DMA
#endif

#ifdef CONFIG_STM32_ADC2_DMA
#  define ADC2_HAVE_DMA 1
#else
#  undef  ADC2_HAVE_DMA
#endif

#ifdef CONFIG_STM32_ADC3_DMA
#  define ADC3_HAVE_DMA 1
#else
#  undef  ADC3_HAVE_DMA
#endif

#ifdef CONFIG_STM32_ADC4_DMA
#  define ADC4_HAVE_DMA 1
#else
#  undef  ADC4_HAVE_DMA
#endif

/* Timer configuration:  If a timer trigger is specified, then get
 * information about the timer.
 *
 * STM32L15XX-family has only one ADC onboard, thus there is no definition
 * for other 3 ADC's
 */

#if defined(CONFIG_STM32_TIM1_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM1_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM1_CLKIN
#elif defined(CONFIG_STM32_TIM2_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM2_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_STM32_TIM3_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM3_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_STM32_TIM4_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM4_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_STM32_TIM5_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM5_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM5_CLKIN
#elif defined(CONFIG_STM32_TIM6_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM6_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM6_CLKIN
#elif defined(CONFIG_STM32_TIM7_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM7_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM7_CLKIN
#elif defined(CONFIG_STM32_TIM8_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM8_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM8_CLKIN
#elif defined(CONFIG_STM32_TIM9_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM9_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM9_CLKIN
#elif defined(CONFIG_STM32_TIM10_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM10_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM10_CLKIN
#else
#    undef  ADC1_HAVE_TIMER
#endif

#ifdef ADC1_HAVE_TIMER
#  ifndef CONFIG_STM32_ADC1_SAMPLE_FREQUENCY
#    error "CONFIG_STM32_ADC1_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32_ADC1_TIMTRIG
#    error "CONFIG_STM32_ADC1_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(CONFIG_STM32_TIM1_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM1_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB2_TIM1_CLKIN
#elif defined(CONFIG_STM32_TIM2_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM2_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_STM32_TIM3_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM3_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_STM32_TIM4_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM4_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_STM32_TIM5_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM5_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM5_CLKIN
#elif defined(CONFIG_STM32_TIM8_ADC2)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM8_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM8_CLKIN
#else
#    undef  ADC2_HAVE_TIMER
#endif

#ifdef ADC2_HAVE_TIMER
#  ifndef CONFIG_STM32_ADC2_SAMPLE_FREQUENCY
#    error "CONFIG_STM32_ADC2_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32_ADC2_TIMTRIG
#    error "CONFIG_STM32_ADC2_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(CONFIG_STM32_TIM1_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32_TIM1_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32_APB2_TIM1_CLKIN
#elif defined(CONFIG_STM32_TIM2_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32_TIM2_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_STM32_TIM3_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32_TIM3_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_STM32_TIM4_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32_TIM4_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_STM32_TIM5_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32_TIM5_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM5_CLKIN
#elif defined(CONFIG_STM32_TIM8_ADC3)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM8_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM8_CLKIN
#else
#    undef  ADC3_HAVE_TIMER
#endif

#ifdef ADC3_HAVE_TIMER
#  ifndef CONFIG_STM32_ADC3_SAMPLE_FREQUENCY
#    error "CONFIG_STM32_ADC3_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32_ADC3_TIMTRIG
#    error "CONFIG_STM32_ADC3_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(ADC1_HAVE_TIMER) || defined(ADC2_HAVE_TIMER) || defined(ADC3_HAVE_TIMER)
#  define ADC_HAVE_TIMER 1
#  if defined(CONFIG_STM32_STM32F10XX) && !defined(CONFIG_STM32_FORCEPOWER)
#    warning "CONFIG_STM32_FORCEPOWER must be defined to enable the timer(s)"
#  endif
#else
#  undef ADC_HAVE_TIMER
#endif

/* NOTE:  The following assumes that all possible combinations of timers and
 * values are support EXTSEL.  That is not so and it varies from one STM32
 * to another.  But this (wrong) assumptions keeps the logic as simple as
 * possible.  If unsupported combination is used, an error will show up
 * later during compilation although it may be difficult to track it back
 * to this simplification.
 *
 * STM32L15XX-family has only one ADC onboard, thus there is no definition
 * for other 3 ADC's
 */

#if defined(CONFIG_STM32_TIM1_ADC1)
#  if CONFIG_STM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC1
#  elif CONFIG_STM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC2
#  elif CONFIG_STM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC3
#  elif CONFIG_STM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC4
#  elif CONFIG_STM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T1TRGO
#  else
#    error "CONFIG_STM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM2_ADC1)
#  if CONFIG_STM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC1
#  elif CONFIG_STM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC2
#  elif CONFIG_STM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC3
#  elif CONFIG_STM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC4
#  elif CONFIG_STM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T2TRGO
#  else
#    error "CONFIG_STM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM3_ADC1)
#  if CONFIG_STM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC1
#  elif CONFIG_STM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC2
#  elif CONFIG_STM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC3
#  elif CONFIG_STM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC4
#  elif CONFIG_STM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T3TRGO
#  else
#    error "CONFIG_STM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM4_ADC1)
#  if CONFIG_STM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC1
#  elif CONFIG_STM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC2
#  elif CONFIG_STM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC3
#  elif CONFIG_STM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC4
#  elif CONFIG_STM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T4TRGO
#  else
#    error "CONFIG_STM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM5_ADC1)
#  if CONFIG_STM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC1
#  elif CONFIG_STM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC2
#  elif CONFIG_STM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC3
#  elif CONFIG_STM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC4
#  elif CONFIG_STM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5TRGO
#  else
#    error "CONFIG_STM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM6_ADC1)
#  if CONFIG_STM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC1
#  elif CONFIG_STM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC2
#  elif CONFIG_STM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC3
#  elif CONFIG_STM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC4
#  elif CONFIG_STM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5TRGO
#  else
#    error "CONFIG_STM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM7_ADC1)
#  if CONFIG_STM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC1
#  elif CONFIG_STM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC2
#  elif CONFIG_STM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC3
#  elif CONFIG_STM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC4
#  elif CONFIG_STM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5TRGO
#  else
#    error "CONFIG_STM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM8_ADC1)
#  if CONFIG_STM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC1
#  elif CONFIG_STM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC2
#  elif CONFIG_STM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC3
#  elif CONFIG_STM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC4
#  elif CONFIG_STM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8TRGO
#  else
#    error "CONFIG_STM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM9_ADC1)
#  if CONFIG_STM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC1
#  elif CONFIG_STM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC2
#  elif CONFIG_STM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC3
#  elif CONFIG_STM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC4
#  elif CONFIG_STM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8TRGO
#  else
#    error "CONFIG_STM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM10_ADC1)
#  if CONFIG_STM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC1
#  elif CONFIG_STM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC2
#  elif CONFIG_STM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC3
#  elif CONFIG_STM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC4
#  elif CONFIG_STM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8TRGO
#  else
#    error "CONFIG_STM32_ADC1_TIMTRIG is out of range"
#  endif
#endif

#if defined(CONFIG_STM32_TIM1_ADC2)
#  if CONFIG_STM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC1
#  elif CONFIG_STM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC2
#  elif CONFIG_STM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC3
#  elif CONFIG_STM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC4
#  elif CONFIG_STM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T1TRGO
#  else
#    error "CONFIG_STM32_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM2_ADC2)
#  if CONFIG_STM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC1
#  elif CONFIG_STM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC2
#  elif CONFIG_STM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC3
#  elif CONFIG_STM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC4
#  elif CONFIG_STM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T2TRGO
#  else
#    error "CONFIG_STM32_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM3_ADC2)
#  if CONFIG_STM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC1
#  elif CONFIG_STM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC2
#  elif CONFIG_STM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC3
#  elif CONFIG_STM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC4
#  elif CONFIG_STM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T3TRGO
#  else
#    error "CONFIG_STM32_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM4_ADC2)
#  if CONFIG_STM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC1
#  elif CONFIG_STM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC2
#  elif CONFIG_STM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC3
#  elif CONFIG_STM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC4
#  elif CONFIG_STM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T4TRGO
#  else
#    error "CONFIG_STM32_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM5_ADC2)
#  if CONFIG_STM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC1
#  elif CONFIG_STM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC2
#  elif CONFIG_STM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC3
#  elif CONFIG_STM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC4
#  elif CONFIG_STM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T5TRGO
#  else
#    error "CONFIG_STM32_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM8_ADC2)
#  if CONFIG_STM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC1
#  elif CONFIG_STM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC2
#  elif CONFIG_STM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC3
#  elif CONFIG_STM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC4
#  elif CONFIG_STM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T8TRGO
#  else
#    error "CONFIG_STM32_ADC2_TIMTRIG is out of range"
#  endif
#endif

#if defined(CONFIG_STM32_TIM1_ADC3)
#  if CONFIG_STM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC1
#  elif CONFIG_STM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC2
#  elif CONFIG_STM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC3
#  elif CONFIG_STM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC4
#  elif CONFIG_STM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T1TRGO
#  else
#    error "CONFIG_STM32_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM2_ADC3)
#  if CONFIG_STM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC1
#  elif CONFIG_STM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC2
#  elif CONFIG_STM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC3
#  elif CONFIG_STM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC4
#  elif CONFIG_STM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T2TRGO
#  else
#    error "CONFIG_STM32_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM3_ADC3)
#  if CONFIG_STM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC1
#  elif CONFIG_STM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC2
#  elif CONFIG_STM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC3
#  elif CONFIG_STM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC4
#  elif CONFIG_STM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T3TRGO
#  else
#    error "CONFIG_STM32_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM4_ADC3)
#  if CONFIG_STM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC1
#  elif CONFIG_STM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC2
#  elif CONFIG_STM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC3
#  elif CONFIG_STM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC4
#  elif CONFIG_STM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T4TRGO
#  else
#    error "CONFIG_STM32_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM5_ADC3)
#  if CONFIG_STM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC1
#  elif CONFIG_STM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC2
#  elif CONFIG_STM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC3
#  elif CONFIG_STM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC4
#  elif CONFIG_STM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T5TRGO
#  else
#    error "CONFIG_STM32_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32_TIM8_ADC3)
#  if CONFIG_STM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC1
#  elif CONFIG_STM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC2
#  elif CONFIG_STM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC3
#  elif CONFIG_STM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC4
#  elif CONFIG_STM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T8TRGO
#  else
#    error "CONFIG_STM32_ADC3_TIMTRIG is out of range"
#  endif
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifdef CONFIG_STM32_STM32L15XX
typedef enum ADC_IO_CMDS
{
  IO_ENABLE_TEMPER_VOLT_CH = 0,
  IO_ENABLE_DISABLE_PDI,
  IO_ENABLE_DISABLE_PDD,
  IO_ENABLE_DISABLE_PDD_PDI,
  IO_ENABLE_DISABLE_AWDIE,
  IO_ENABLE_DISABLE_EOCIE,
  IO_ENABLE_DISABLE_JEOCIE,
  IO_ENABLE_DISABLE_OVRIE = 7,
  IO_ENABLE_DISABLE_ALL_INTS,
  IO_START_CONV,
  IO_STOP_ADC,
  IO_START_ADC,
} ADC_IO_CMDS;

/* Channel and sample time pair */

typedef struct adc_channel_s
{
  uint8_t channel : 5;

  /* Sampling time individually for each channel
   * 000: 4 cycles
   * 001: 9 cycles
   * 010: 16 cycles
   * 011: 24 cycles
   * 100: 48 cycles
   * 101: 96 cycles
   * 110: 192 cycles
   * 111: 384 cycles    - selected for all channels
   */

  uint8_t sample_time : 3;
} adc_channel_t;

/* This structure will be used while setting channels to specified by the
 * "channel-sample time" pairs' values
 */

struct adc_sample_time_s {
    adc_channel_t *channel;                /* array of channels */
    uint8_t channels_nbr:5;                /* number of channels in array */
    bool all_same:1;                       /* All 32 channels will get the
                                            * same value of the sample time */
    uint8_t all_ch_sample_time:3;          /* Sample time for all 32 channels */
};

#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_adcinitialize
 *
 * Description:
 *   Initialize the ADC.
 *
 * Input Parameters:
 *   intf      - Could be {1,2,3} for ADC1, ADC2, or ADC3
 *   chanlist  - The list of channels
 *   nchannels - Number of channels
 *
 * Returned Value:
 *   Valid can device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s;
struct adc_dev_s *stm32_adcinitialize(int intf, FAR const uint8_t *chanlist,
                                      int nchannels);

#ifdef CONFIG_STM32_STM32L15XX
void stm32_adcchange_sample_time(FAR struct adc_dev_s *dev,
                                 FAR struct adc_sample_time_s *time_samples);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32_ADC || CONFIG_STM32_ADC2 || CONFIG_STM32_ADC3 */
#endif /* __ARCH_ARM_SRC_STM32_STM32_ADC_H */
