/****************************************************************************
 * arch/arm/src/stm32h5/stm32_adc.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_ADC_H
#define __ARCH_ARM_SRC_STM32H5_STM32_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/adc.h>
#include "chip.h"
#include "hardware/stm32_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_STM32H5_ADC1) || defined(CONFIG_STM32H5_ADC2)

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to control periodic ADC sampling.  If CONFIG_STM32H5_TIMn is defined then
 * CONFIG_STM32H5_TIMn_ADC must also be defined to indicate that timer "n"
 * is intended to be used for that purpose.
 */

#ifndef CONFIG_STM32H5_TIM1
#  undef CONFIG_STM32H5_TIM1_ADC
#  undef CONFIG_STM32H5_TIM1_ADC1
#  undef CONFIG_STM32H5_TIM1_ADC2
#endif
#ifndef CONFIG_STM32H5_TIM2
#  undef CONFIG_STM32H5_TIM2_ADC
#  undef CONFIG_STM32H5_TIM2_ADC1
#  undef CONFIG_STM32H5_TIM2_ADC2
#endif
#ifndef CONFIG_STM32H5_TIM3
#  undef CONFIG_STM32H5_TIM3_ADC
#  undef CONFIG_STM32H5_TIM3_ADC1
#  undef CONFIG_STM32H5_TIM3_ADC2
#endif
#ifndef CONFIG_STM32H5_TIM4
#  undef CONFIG_STM32H5_TIM4_ADC
#  undef CONFIG_STM32H5_TIM4_ADC1
#  undef CONFIG_STM32H5_TIM4_ADC2
#endif
#ifndef CONFIG_STM32H5_TIM6
#  undef CONFIG_STM32H5_TIM6_ADC
#  undef CONFIG_STM32H5_TIM6_ADC1
#  undef CONFIG_STM32H5_TIM6_ADC2
#endif
#ifndef CONFIG_STM32H5_TIM8
#  undef CONFIG_STM32H5_TIM8_ADC
#  undef CONFIG_STM32H5_TIM8_ADC1
#  undef CONFIG_STM32H5_TIM8_ADC2
#endif
#ifndef CONFIG_STM32H5_TIM15
#  undef CONFIG_STM32H5_TIM15_ADC
#  undef CONFIG_STM32H5_TIM15_ADC1
#  undef CONFIG_STM32H5_TIM15_ADC2
#endif

/* DMA support */

#undef ADC_HAVE_DMA
#if defined(CONFIG_STM32H5_ADC1_DMA) || defined(CONFIG_STM32H5_ADC2_DMA)
#  define ADC_HAVE_DMA 1
#endif

#if defined(CONFIG_STM32H5_ADC1_DMA)
#  define ADC1_HAVE_DMA 1
#endif

#if defined(CONFIG_STM32H5_ADC2_DMA)
#  define ADC2_HAVE_DMA 1
#endif

/* Oversampling support */

#undef ADC_HAVE_OVERSAMPLE
#if defined(CONFIG_STM32H5_ADC1_OVERSAMPLE) || \
    defined(CONFIG_STM32H5_ADC2_OVERSAMPLE)
#  define ADC_HAVE_OVERSAMPLE 1
#endif

#if defined(CONFIG_STM32H5_ADC1_OVERSAMPLE)
#  define ADC1_HAVE_OVERSAMPLE 1
#endif

#if defined(CONFIG_STM32H5_ADC2_OVERSAMPLE)
#  define ADC2_HAVE_OVERSAMPLE 1
#endif

/* Timer configuration:  If a timer trigger is specified, then get
 * information about the timer.
 */

#if defined(CONFIG_STM32H5_TIM1_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM1_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM1_CLKIN
#    define ADC1_TIMER_RCC_ENR        STM32_RCC_APB2ENR
#    define ADC1_TIMER_RCC_EN         RCC_APB2ENR_TIM1EN
#elif defined(CONFIG_STM32H5_TIM2_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM2_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#    define ADC1_TIMER_RCC_ENR        STM32_RCC_APB1LENR
#    define ADC1_TIMER_RCC_EN         RCC_APB1LENR_TIM2EN
#elif defined(CONFIG_STM32H5_TIM3_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM3_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#    define ADC1_TIMER_RCC_ENR        STM32_RCC_APB1LENR
#    define ADC1_TIMER_RCC_EN         RCC_APB1LENR_TIM3EN
#elif defined(CONFIG_STM32H5_TIM4_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM4_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#    define ADC1_TIMER_RCC_ENR        STM32_RCC_APB1LENR
#    define ADC1_TIMER_RCC_EN         RCC_APB1LENR_TIM4EN
#elif defined(CONFIG_STM32H5_TIM6_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM6_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM6_CLKIN
#    define ADC1_TIMER_RCC_ENR        STM32_RCC_APB1LENR
#    define ADC1_TIMER_RCC_EN         RCC_APB1LENR_TIM6EN
#elif defined(CONFIG_STM32H5_TIM8_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM8_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM8_CLKIN
#    define ADC1_TIMER_RCC_ENR        STM32_RCC_APB2ENR
#    define ADC1_TIMER_RCC_EN         RCC_APB2ENR_TIM8EN
#elif defined(CONFIG_STM32H5_TIM15_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM15_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM15_CLKIN
#    define ADC1_TIMER_RCC_ENR        STM32_RCC_APB2ENR
#    define ADC1_TIMER_RCC_EN         RCC_APB2ENR_TIM15EN
#else
#    undef  ADC1_HAVE_TIMER
#endif

#ifdef ADC1_HAVE_TIMER
#  ifndef CONFIG_STM32H5_ADC1_SAMPLE_FREQUENCY
#    error "CONFIG_STM32H5_ADC1_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32H5_ADC1_TIMTRIG
#    error "CONFIG_STM32H5_ADC1_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(CONFIG_STM32H5_TIM1_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM1_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB2_TIM1_CLKIN
#    define ADC2_TIMER_RCC_ENR        STM32_RCC_APB2ENR
#    define ADC2_TIMER_RCC_EN         RCC_APB2ENR_TIM1EN
#elif defined(CONFIG_STM32H5_TIM2_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM2_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#    define ADC2_TIMER_RCC_ENR        STM32_RCC_APB1LENR
#    define ADC2_TIMER_RCC_EN         RCC_APB1LENR_TIM2EN
#elif defined(CONFIG_STM32H5_TIM3_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM3_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#    define ADC2_TIMER_RCC_ENR        STM32_RCC_APB1LENR
#    define ADC2_TIMER_RCC_EN         RCC_APB1LENR_TIM3EN
#elif defined(CONFIG_STM32H5_TIM4_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM4_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#    define ADC2_TIMER_RCC_ENR        STM32_RCC_APB1LENR
#    define ADC2_TIMER_RCC_EN         RCC_APB1LENR_TIM4EN
#elif defined(CONFIG_STM32H5_TIM6_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM6_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM6_CLKIN
#    define ADC2_TIMER_RCC_ENR        STM32_RCC_APB1LENR
#    define ADC2_TIMER_RCC_EN         RCC_APB1LENR_TIM6EN
#elif defined(CONFIG_STM32H5_TIM8_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM8_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB2_TIM8_CLKIN
#    define ADC2_TIMER_RCC_ENR        STM32_RCC_APB2ENR
#    define ADC2_TIMER_RCC_EN         RCC_APB2ENR_TIM8EN
#elif defined(CONFIG_STM32H5_TIM15_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32_TIM15_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32_APB2_TIM15_CLKIN
#    define ADC2_TIMER_RCC_ENR        STM32_RCC_APB2ENR
#    define ADC2_TIMER_RCC_EN         RCC_APB2ENR_TIM15EN
#else
#    undef  ADC2_HAVE_TIMER
#endif

#ifdef ADC2_HAVE_TIMER
#  ifndef CONFIG_STM32H5_ADC2_SAMPLE_FREQUENCY
#    error "CONFIG_STM32H5_ADC2_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32H5_ADC2_TIMTRIG
#    error "CONFIG_STM32H5_ADC2_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(ADC1_HAVE_TIMER) || defined(ADC2_HAVE_TIMER)
#  define ADC_HAVE_TIMER 1
#else
#  undef ADC_HAVE_TIMER
#endif

#if defined(CONFIG_STM32H5_TIM1_ADC1)
#  if CONFIG_STM32H5_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1CC1
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1CC2
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC_CFGR__EXTSEL_T1CC3
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1CC4
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1TRGO
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 5
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1TRGO2
#  else
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is out of range (TIM1)"
#  endif
#elif defined(CONFIG_STM32H5_TIM2_ADC1)
#  if CONFIG_STM32H5_ADC1_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM2)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T2CC2
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM2)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 3
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM2)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T2TRGO
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 5
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM2)"
#  else
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is out of range (TIM2)"
#  endif
#elif defined(CONFIG_STM32H5_TIM3_ADC1)
#  if CONFIG_STM32H5_ADC1_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM3)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 1
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM3)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM3)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T3CC4
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T3TRGO
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 5
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM3)"
#  else
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is out of range (TIM3)"
#  endif
#elif defined(CONFIG_STM32H5_TIM4_ADC1)
#  if CONFIG_STM32H5_ADC1_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM4)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 1
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM4)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM4)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T4CC4
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CRFT_EXTSEL_T4TRGO
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 5
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM4)"
#  else
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is out of range (TIM4)"
#  endif
#elif defined(CONFIG_STM32H5_TIM6_ADC1)
#  if CONFIG_STM32H5_ADC1_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM6)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 1
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM6)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM6)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 3
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM6)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T6TRGO
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 5
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM6)"
#  else
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is out of range (TIM6)"
#  endif
#elif defined(CONFIG_STM32H5_TIM8_ADC1)
#  if CONFIG_STM32H5_ADC1_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM8)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 1
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM8)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM8)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 3
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM8)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T8TRGO
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 5
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T8TRGO2
#  else
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is out of range (TIM8)"
#  endif
#elif defined(CONFIG_STM32H5_TIM15_ADC1)
#  if CONFIG_STM32H5_ADC1_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM15)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 1
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM15)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM15)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 3
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM15)"
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC_CFGR_EXTSEL_T15TRGO
#  elif CONFIG_STM32H5_ADC1_TIMTRIG == 5
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is invalid (TIM15)"
#  else
#    error "CONFIG_STM32H5_ADC1_TIMTRIG is out of range (TIM15)"
#  endif
#endif

#if defined(CONFIG_STM32H5_TIM1_ADC2)
#  if CONFIG_STM32H5_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1CC1
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1CC2
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1CC3
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1CC4
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1TRGO
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 5
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T1TRGO2
#  else
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is out of range (TIM1)"
#  endif
#elif defined(CONFIG_STM32H5_TIM2_ADC2)
#  if CONFIG_STM32H5_ADC2_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM2)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T2CC2
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM2)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 3
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM2)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T2TRGO
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 5
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM2)"
#  else
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is out of range (TIM2)"
#  endif
#elif defined(CONFIG_STM32H5_TIM3_ADC2)
#  if CONFIG_STM32H5_ADC2_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM3)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 1
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM3)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM3)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T3CC4
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T3TRGO
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 5
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM3)"
#  else
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is out of range (TIM3)"
#  endif
#elif defined(CONFIG_STM32H5_TIM4_ADC2)
#  if CONFIG_STM32H5_ADC2_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM4)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 1
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM4)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM4)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T4CC4
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T4TRGO
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 5
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM4)"
#  else
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is out of range (TIM4)"
#  endif
#elif defined(CONFIG_STM32H5_TIM6_ADC2)
#  if CONFIG_STM32H5_ADC2_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM6)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 1
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM6)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM6)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 3
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM6)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T6TRGO
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 5
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM6)"
#  else
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is out of range (TIM6)"
#  endif
#elif defined(CONFIG_STM32H5_TIM8_ADC2)
#  if CONFIG_STM32H5_ADC2_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM8)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 1
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM8)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM8)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 3
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM8)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T8TRGO
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 5
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T8TRGO2
#  else
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is out of range (TIM8)"
#  endif
#elif defined(CONFIG_STM32H5_TIM15_ADC2)
#  if CONFIG_STM32H5_ADC2_TIMTRIG == 0
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM15)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 1
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM15)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 2
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM15)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 3
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM15)"
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC_CFGR_EXTSEL_T15TRGO
#  elif CONFIG_STM32H5_ADC2_TIMTRIG == 5
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is invalid (TIM15)"
#  else
#    error "CONFIG_STM32H5_ADC2_TIMTRIG is out of range (TIM15)"
#  endif
#endif

/* IOCTL Support */

#define ANIOC_STM32H5_WDOG_CFG       _ANIOC(AN_STM32H5_FIRST + 0)
#define ANIOC_STM32H5_WDOG2_CFG      _ANIOC(AN_STM32H5_FIRST + 1)
#define ANIOC_STM32H5_WDOG3_CFG      _ANIOC(AN_STM32H5_FIRST + 2)
#define ANIOC_STM32H5_WDOG_GET_EVENT _ANIOC(AN_STM32H5_FIRST + 3) /* OUT: struct stm32_adc_wdg_event_s* */
#define ANIOC_STM32H5_WDOG_SIGCFG    _ANIOC(AN_STM32H5_FIRST + 4) /*  IN: struct stm32_adc_sigcfg_s*  */

/* Watchdog Event Queue Support */

#define WQ_LEN    128
#define WQ_MASK   (WQ_LEN - 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* STM32H5 ADC channel configuration */

struct stm32_adc_channel_s
{
  uint8_t chan;                 /* Channel Number */
  uint32_t p_gpio;              /* P GPIO */
  uint32_t n_gpio;              /* N GPIO */
  uint8_t tsamp:3;              /* Sampling time */
  uint8_t mode:1;               /* Single-ended 0 or differential mode 1 */
  uint8_t injected:1;           /* Regular 0 or Injected 1 */
  uint8_t _res:3;               /* Reserved */
};

struct stm32_adc_watchdog1_cfg_s
{
  uint8_t filter;
  uint8_t channel;
  uint8_t all_channels;
  uint16_t high_thresh;
  uint16_t low_thresh;
};

struct stm32_adc_watchdog23_cfg_s
{
  uint32_t channels;
  uint8_t high_thresh;
  uint8_t low_thresh;
};

struct stm32_adc_wdg_event_s
{
  uint8_t  wdg_num;  /* 1..3 => AWD1/2/3 */
  uint32_t isr;      /* ISR snapshot */
};

struct stm32_adc_wdg_queue_s
{
  struct stm32_adc_wdg_event_s wq[WQ_LEN];
  volatile uint8_t wq_head;        /* next write idx [0..WQ_LEN-1] */
  volatile uint8_t wq_tail;        /* next read  idx [0..WQ_LEN-1] */
  volatile uint16_t wq_dropped;    /* stats: dropped when full */
};

struct stm32_adc_sigcfg_s
{
  pid_t    pid;     /* 0 disables signaling */
  uint8_t  signo;   /* e.g., SIGUSR1; 0 disables */
  uint8_t  mask;    /* bit0:AWD1, bit1:AWD2, bit2:AWD3 */
};

struct stm32_adc_sig_s
{
  /* Who to notify and which AWDs */

  struct stm32_adc_sigcfg_s sigcfg;

  /* Work item used to deliver the signal outside ISR context */

  struct work_s work;

  /* Last event cached for the signal payload (coalesced) */

  uint8_t last_wdg_num;         /* 1..3 */
};

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
 * Name: stm32h5_adc_initialize
 *
 * Description:
 *    Initialize the ADC.
 *
 * Input Parameters:
 *    intf      - Could be {1,2} for ADC1, ADC2
 *    chanlist  - The list of channels
 *    nchannels - Number of channels
 *
 * Returned Value:
 *    Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s;
struct adc_dev_s *stm32h5_adc_initialize(int intf,
                                         struct stm32_adc_channel_s *chanlist
                                         , int nchannels);
#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32H5_ADC1 || CONFIG_STM32H5_ADC2*/
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_ADC_H */
