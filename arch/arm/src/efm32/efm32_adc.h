/****************************************************************************
 * arch/arm/src/efm32/efm32_adc.h
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_ADC_H
#define __ARCH_ARM_SRC_EFM32_EFM32_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#include "hardware/efm32_adc.h"

#include <nuttx/analog/adc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to control periodic ADC sampling.  If CONFIG_EFM32_TIMn is defined then
 * CONFIG_EFM32_TIMn_ADC must also be defined to indicate that timer "n" is
 * intended to be used for that purpose.
 */

/* For the EFM32 F1 line, timers 1-4 may be used.
 *  For EFM32 F4 line, timers 1-5 and 8 may be used.
 */

#ifndef CONFIG_EFM32_TIM1
#  undef CONFIG_EFM32_TIM1_ADC
#  undef CONFIG_EFM32_TIM1_ADC1
#  undef CONFIG_EFM32_TIM1_ADC2
#  undef CONFIG_EFM32_TIM1_ADC3
#endif
#ifndef CONFIG_EFM32_TIM2
#  undef CONFIG_EFM32_TIM2_ADC
#  undef CONFIG_EFM32_TIM2_ADC1
#  undef CONFIG_EFM32_TIM2_ADC2
#  undef CONFIG_EFM32_TIM2_ADC3
#endif
#ifndef CONFIG_EFM32_TIM3
#  undef CONFIG_EFM32_TIM3_ADC
#  undef CONFIG_EFM32_TIM3_ADC1
#  undef CONFIG_EFM32_TIM3_ADC2
#  undef CONFIG_EFM32_TIM3_ADC3
#endif
#ifndef CONFIG_EFM32_TIM4
#  undef CONFIG_EFM32_TIM4_ADC
#  undef CONFIG_EFM32_TIM4_ADC1
#  undef CONFIG_EFM32_TIM4_ADC2
#  undef CONFIG_EFM32_TIM4_ADC3
#endif
#ifndef CONFIG_EFM32_TIM5
#  undef CONFIG_EFM32_TIM5_ADC
#  undef CONFIG_EFM32_TIM5_ADC1
#  undef CONFIG_EFM32_TIM5_ADC2
#  undef CONFIG_EFM32_TIM5_ADC3
#endif
#ifndef CONFIG_EFM32_TIM8
#  undef CONFIG_EFM32_TIM8_ADC
#  undef CONFIG_EFM32_TIM8_ADC1
#  undef CONFIG_EFM32_TIM8_ADC2
#  undef CONFIG_EFM32_TIM8_ADC3
#endif

/* Timers 6, 7,
 * and 10-14 are not used with the ADC by any supported family
 */

#undef CONFIG_EFM32_TIM6_ADC
#undef CONFIG_EFM32_TIM6_ADC1
#undef CONFIG_EFM32_TIM6_ADC2
#undef CONFIG_EFM32_TIM6_ADC3
#undef CONFIG_EFM32_TIM7_ADC
#undef CONFIG_EFM32_TIM7_ADC1
#undef CONFIG_EFM32_TIM7_ADC2
#undef CONFIG_EFM32_TIM7_ADC3
#undef CONFIG_EFM32_TIM9_ADC
#undef CONFIG_EFM32_TIM9_ADC1
#undef CONFIG_EFM32_TIM9_ADC2
#undef CONFIG_EFM32_TIM9_ADC3
#undef CONFIG_EFM32_TIM10_ADC
#undef CONFIG_EFM32_TIM10_ADC1
#undef CONFIG_EFM32_TIM10_ADC2
#undef CONFIG_EFM32_TIM10_ADC3
#undef CONFIG_EFM32_TIM11_ADC
#undef CONFIG_EFM32_TIM11_ADC1
#undef CONFIG_EFM32_TIM11_ADC2
#undef CONFIG_EFM32_TIM11_ADC3
#undef CONFIG_EFM32_TIM12_ADC
#undef CONFIG_EFM32_TIM12_ADC1
#undef CONFIG_EFM32_TIM12_ADC2
#undef CONFIG_EFM32_TIM12_ADC3
#undef CONFIG_EFM32_TIM13_ADC
#undef CONFIG_EFM32_TIM13_ADC1
#undef CONFIG_EFM32_TIM13_ADC2
#undef CONFIG_EFM32_TIM13_ADC3
#undef CONFIG_EFM32_TIM14_ADC
#undef CONFIG_EFM32_TIM14_ADC1
#undef CONFIG_EFM32_TIM14_ADC2
#undef CONFIG_EFM32_TIM14_ADC3

/* Up to 3 ADC interfaces are supported */

#if EFM32_NADC < 3
#  undef CONFIG_EFM32_ADC3
#endif

#if EFM32_NADC < 2
#  undef CONFIG_EFM32_ADC2
#endif

#if EFM32_NADC < 1
#  undef CONFIG_EFM32_ADC1
#endif

#if defined(CONFIG_EFM32_ADC1) || defined(CONFIG_EFM32_ADC2) || defined(CONFIG_EFM32_ADC3)

/* DMA support is not yet implemented for this driver */

#ifdef CONFIG_ADC_DMA
#  warning "DMA is not supported by the current driver"
#endif

/* Timer configuration:
 * If a timer trigger is specified, then get information about the timer.
 */

#if defined(CONFIG_EFM32_TIM1_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           EFM32_TIM1_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY EFM32_APB2_TIM1_CLKIN
#elif defined(CONFIG_EFM32_TIM2_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           EFM32_TIM2_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_EFM32_TIM3_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           EFM32_TIM3_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_EFM32_TIM4_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           EFM32_TIM4_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_EFM32_TIM5_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           EFM32_TIM5_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM5_CLKIN
#elif defined(CONFIG_EFM32_TIM8_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           EFM32_TIM8_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY EFM32_APB2_TIM8_CLKIN
#else
#    undef  ADC1_HAVE_TIMER
#endif

#ifdef ADC1_HAVE_TIMER
#  ifndef CONFIG_EFM32_ADC1_SAMPLE_FREQUENCY
#    error "CONFIG_EFM32_ADC1_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_EFM32_ADC1_TIMTRIG
#    error "CONFIG_EFM32_ADC1_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(CONFIG_EFM32_TIM1_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           EFM32_TIM1_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY EFM32_APB2_TIM1_CLKIN
#elif defined(CONFIG_EFM32_TIM2_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           EFM32_TIM2_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_EFM32_TIM3_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           EFM32_TIM3_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_EFM32_TIM4_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           EFM32_TIM4_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_EFM32_TIM5_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           EFM32_TIM5_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM5_CLKIN
#elif defined(CONFIG_EFM32_TIM8_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           EFM32_TIM8_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY EFM32_APB2_TIM8_CLKIN
#else
#    undef  ADC2_HAVE_TIMER
#endif

#ifdef ADC2_HAVE_TIMER
#  ifndef CONFIG_EFM32_ADC2_SAMPLE_FREQUENCY
#    error "CONFIG_EFM32_ADC2_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_EFM32_ADC2_TIMTRIG
#    error "CONFIG_EFM32_ADC2_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(CONFIG_EFM32_TIM1_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           EFM32_TIM1_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY EFM32_APB2_TIM1_CLKIN
#elif defined(CONFIG_EFM32_TIM2_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           EFM32_TIM2_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_EFM32_TIM3_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           EFM32_TIM3_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_EFM32_TIM4_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           EFM32_TIM4_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_EFM32_TIM5_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           EFM32_TIM5_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY EFM32_APB1_TIM5_CLKIN
#elif defined(CONFIG_EFM32_TIM8_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           EFM32_TIM8_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY EFM32_APB2_TIM8_CLKIN
#else
#    undef  ADC3_HAVE_TIMER
#endif

#ifdef ADC3_HAVE_TIMER
#  ifndef CONFIG_EFM32_ADC3_SAMPLE_FREQUENCY
#    error "CONFIG_EFM32_ADC3_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_EFM32_ADC3_TIMTRIG
#    error "CONFIG_EFM32_ADC3_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(ADC1_HAVE_TIMER) || defined(ADC2_HAVE_TIMER) || defined(ADC3_HAVE_TIMER)
#  define ADC_HAVE_TIMER 1
#  if defined(CONFIG_EFM32_EFM32F10XX) && !defined(CONFIG_EFM32_FORCEPOWER)
#    warning "CONFIG_EFM32_FORCEPOWER must be defined to enable the timer(s)"
#  endif
#else
#  undef ADC_HAVE_TIMER
#endif

/* NOTE:
 * The following assumes that all possible combinations of timers and values
 * are support EXTSEL.  That is not so and it varies from one EFM32 to
 * another. But this (wrong) assumptions keeps the logic as simple as
 * possible.  If un unsupported combination is used, an error will show
 * up later during compilation although it may be difficult to track it back
 * to this simplification.
 */

#if defined(CONFIG_EFM32_TIM1_ADC1)
#  if CONFIG_EFM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC1
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC2
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC3
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC4
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T1TRGO
#  else
#    error "CONFIG_EFM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM2_ADC1)
#  if CONFIG_EFM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC1
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC2
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC3
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC4
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T2TRGO
#  else
#    error "CONFIG_EFM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM3_ADC1)
#  if CONFIG_EFM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC1
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC2
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC3
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC4
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T3TRGO
#  else
#    error "CONFIG_EFM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM4_ADC1)
#  if CONFIG_EFM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC1
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC2
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC3
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC4
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T4TRGO
#  else
#    error "CONFIG_EFM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM5_ADC1)
#  if CONFIG_EFM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC1
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC2
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC3
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC4
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T5TRGO
#  else
#    error "CONFIG_EFM32_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM8_ADC1)
#  if CONFIG_EFM32_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC1
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC2
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC3
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC4
#  elif CONFIG_EFM32_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CR2_EXTSEL_T8TRGO
#  else
#    error "CONFIG_EFM32_ADC1_TIMTRIG is out of range"
#  endif
#endif

#if defined(CONFIG_EFM32_TIM1_ADC2)
#  if CONFIG_EFM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC1
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC2
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC3
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC4
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T1TRGO
#  else
#    error "CONFIG_EFM32_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM2_ADC2)
#  if CONFIG_EFM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC1
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC2
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC3
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC4
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T2TRGO
#  else
#    error "CONFIG_EFM32_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM3_ADC2)
#  if CONFIG_EFM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC1
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC2
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC3
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC4
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T3TRGO
#  else
#    error "CONFIG_EFM32_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM4_ADC2)
#  if CONFIG_EFM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC1
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC2
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC3
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC4
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T4TRGO
#  else
#    error "CONFIG_EFM32_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM5_ADC2)
#  if CONFIG_EFM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC1
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC2
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC3
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC4
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T5TRGO
#  else
#    error "CONFIG_EFM32_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM8_ADC2)
#  if CONFIG_EFM32_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC1
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC2
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC3
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC4
#  elif CONFIG_EFM32_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CR2_EXTSEL_T8TRGO
#  else
#    error "CONFIG_EFM32_ADC2_TIMTRIG is out of range"
#  endif
#endif

#if defined(CONFIG_EFM32_TIM1_ADC3)
#  if CONFIG_EFM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC1
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC2
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC3
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T1CC4
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T1TRGO
#  else
#    error "CONFIG_EFM32_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM2_ADC3)
#  if CONFIG_EFM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC1
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC2
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC3
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T2CC4
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T2TRGO
#  else
#    error "CONFIG_EFM32_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM3_ADC3)
#  if CONFIG_EFM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC1
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC2
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC3
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T3CC4
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T3TRGO
#  else
#    error "CONFIG_EFM32_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM4_ADC3)
#  if CONFIG_EFM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC1
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC2
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC3
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T4CC4
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T4TRGO
#  else
#    error "CONFIG_EFM32_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM5_ADC3)
#  if CONFIG_EFM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC1
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC2
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC3
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T5CC4
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T5TRGO
#  else
#    error "CONFIG_EFM32_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_EFM32_TIM8_ADC3)
#  if CONFIG_EFM32_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC1
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC2
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC3
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T8CC4
#  elif CONFIG_EFM32_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC_CR2_EXTSEL_T8TRGO
#  else
#    error "CONFIG_EFM32_ADC3_TIMTRIG is out of range"
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: efm32_adcinitialize
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
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s;
struct adc_dev_s *efm32_adcinitialize(int intf, const uint8_t *chanlist,
                                     int nchannels);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_EFM32_ADC || CONFIG_EFM32_ADC2 || CONFIG_EFM32_ADC3 */
#endif /* __ARCH_ARM_SRC_EFM32_EFM32_ADC_H */
