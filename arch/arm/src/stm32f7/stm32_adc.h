/****************************************************************************
 * arch/arm/src/stm32f7/stm32_adc.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_ADC_H
#define __ARCH_ARM_SRC_STM32F7_STM32_ADC_H

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

/* Generalized definitions for ADC  *****************************************/

#define STM32_ADC_DMAREG_OFFSET    STM32_ADC_CR2_OFFSET
#define ADC_DMAREG_DMA             ADC_CR2_DMA
#define STM32_ADC_EXTREG_OFFSET    STM32_ADC_CR2_OFFSET
#define ADC_EXTREG_EXTSEL_MASK     ADC_CR2_EXTSEL_MASK
#define ADC_EXTREG_EXTSEL_SHIFT    ADC_CR2_EXTSEL_SHIFT
#define STM32_ADC_JEXTREG_OFFSET   STM32_ADC_CR2_OFFSET
#define ADC_JEXTREG_JEXTSEL_MASK   ADC_CR2_JEXTSEL_MASK
#define ADC_EXTREG_JEXTSEL_SHIFT   ADC_CR2_JEXTSEL_SHIFT
#define STM32_ADC_ISR_OFFSET       STM32_ADC_SR_OFFSET
#define STM32_ADC_IER_OFFSET       STM32_ADC_CR1_OFFSET
#define ADC_EXTREG_EXTEN_MASK      ADC_CR2_EXTEN_MASK
#define ADC_EXTREG_EXTEN_NONE      ADC_CR2_EXTEN_NONE
#define ADC_EXTREG_EXTEN_DEFAULT   ADC_CR2_EXTEN_RISING
#define ADC_JEXTREG_JEXTEN_MASK    ADC_CR2_JEXTEN_MASK
#define ADC_JEXTREG_JEXTEN_NONE    ADC_CR2_JEXTEN_NONE
#define ADC_JEXTREG_JEXTEN_DEFAULT ADC_CR2_JEXTEN_RISING

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to control periodic ADC sampling.  If CONFIG_STM32F7_TIMn is defined then
 * CONFIG_STM32F7_TIMn_ADC must also be defined to indicate that timer "n"
 * is intended to be used for that purpose. Timers 1-6 and 8 may be used.
 */

#ifndef CONFIG_STM32F7_TIM1
#  undef CONFIG_STM32F7_TIM1_ADC
#  undef CONFIG_STM32F7_TIM1_ADC1
#  undef CONFIG_STM32F7_TIM1_ADC2
#  undef CONFIG_STM32F7_TIM1_ADC3
#endif
#ifndef CONFIG_STM32F7_TIM2
#  undef CONFIG_STM32F7_TIM2_ADC
#  undef CONFIG_STM32F7_TIM2_ADC1
#  undef CONFIG_STM32F7_TIM2_ADC2
#  undef CONFIG_STM32F7_TIM2_ADC3
#endif
#ifndef CONFIG_STM32F7_TIM3
#  undef CONFIG_STM32F7_TIM3_ADC
#  undef CONFIG_STM32F7_TIM3_ADC1
#  undef CONFIG_STM32F7_TIM3_ADC2
#  undef CONFIG_STM32F7_TIM3_ADC3
#endif
#ifndef CONFIG_STM32F7_TIM4
#  undef CONFIG_STM32F7_TIM4_ADC
#  undef CONFIG_STM32F7_TIM4_ADC1
#  undef CONFIG_STM32F7_TIM4_ADC2
#  undef CONFIG_STM32F7_TIM4_ADC3
#endif
#ifndef CONFIG_STM32F7_TIM5
#  undef CONFIG_STM32F7_TIM5_ADC
#  undef CONFIG_STM32F7_TIM5_ADC1
#  undef CONFIG_STM32F7_TIM5_ADC2
#  undef CONFIG_STM32F7_TIM5_ADC3
#endif
#ifndef CONFIG_STM32F7_TIM6
#  undef CONFIG_STM32F7_TIM6_ADC
#  undef CONFIG_STM32F7_TIM6_ADC1
#  undef CONFIG_STM32F7_TIM6_ADC2
#  undef CONFIG_STM32F7_TIM6_ADC3
#endif
#ifndef CONFIG_STM32F7_TIM8
#  undef CONFIG_STM32F7_TIM8_ADC
#  undef CONFIG_STM32F7_TIM8_ADC1
#  undef CONFIG_STM32F7_TIM8_ADC2
#  undef CONFIG_STM32F7_TIM8_ADC3
#endif

/* Up to 3 ADC interfaces are supported */

#if defined(CONFIG_STM32F7_ADC1) || defined(CONFIG_STM32F7_ADC2) || \
    defined(CONFIG_STM32F7_ADC3)

/* DMA support */

#undef ADC_HAVE_DMA
#if defined(CONFIG_STM32F7_ADC1_DMA) || defined(CONFIG_STM32F7_ADC2_DMA) || \
    defined(CONFIG_STM32F7_ADC3_DMA)
#  define ADC_HAVE_DMA  1
#endif

#ifdef CONFIG_STM32F7_ADC1_DMA
#  define ADC1_HAVE_DMA 1
#else
#  undef  ADC1_HAVE_DMA
#endif

#ifdef CONFIG_STM32F7_ADC2_DMA
#  define ADC2_HAVE_DMA 1
#else
#  undef  ADC2_HAVE_DMA
#endif

#ifdef CONFIG_STM32F7_ADC3_DMA
#  define ADC3_HAVE_DMA 1
#else
#  undef  ADC3_HAVE_DMA
#endif

/* Injected channels support */

#if (defined(CONFIG_STM32F7_ADC1) && (CONFIG_STM32F7_ADC1_INJECTED_CHAN > 0)) || \
    (defined(CONFIG_STM32F7_ADC2) && (CONFIG_STM32F7_ADC2_INJECTED_CHAN > 0)) || \
    (defined(CONFIG_STM32F7_ADC3) && (CONFIG_STM32F7_ADC3_INJECTED_CHAN > 0))
#  define ADC_HAVE_INJECTED
#endif

/* Timer configuration:  If a timer trigger is specified, then get
 * information about the timer.
 */

#if defined(CONFIG_STM32F7_TIM1_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM1_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM1_CLKIN
#elif defined(CONFIG_STM32F7_TIM2_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM2_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_STM32F7_TIM3_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM3_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_STM32F7_TIM4_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM4_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_STM32F7_TIM5_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM5_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM5_CLKIN
#elif defined(CONFIG_STM32F7_TIM6_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM6_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM6_CLKIN
#elif defined(CONFIG_STM32F7_TIM8_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM8_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM8_CLKIN
#else
#    undef  ADC1_HAVE_TIMER
#endif

#ifdef ADC1_HAVE_TIMER
#  ifndef CONFIG_STM32F7_ADC1_SAMPLE_FREQUENCY
#    error "CONFIG_STM32F7_ADC1_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32F7_ADC1_TIMTRIG
#    error "CONFIG_STM32F7_ADC1_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(CONFIG_STM32F7_TIM1_ADC2)
#  define ADC2_HAVE_TIMER           1
#  define ADC2_TIMER_BASE           STM32_TIM1_BASE
#  define ADC2_TIMER_PCLK_FREQUENCY STM32_APB2_TIM1_CLKIN
#elif defined(CONFIG_STM32F7_TIM2_ADC2)
#  define ADC2_HAVE_TIMER           1
#  define ADC2_TIMER_BASE           STM32_TIM2_BASE
#  define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_STM32F7_TIM3_ADC2)
#  define ADC2_HAVE_TIMER           1
#  define ADC2_TIMER_BASE           STM32_TIM3_BASE
#  define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_STM32F7_TIM4_ADC2)
#  define ADC2_HAVE_TIMER           1
#  define ADC2_TIMER_BASE           STM32_TIM4_BASE
#  define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_STM32F7_TIM5_ADC2)
#  define ADC2_HAVE_TIMER           1
#  define ADC2_TIMER_BASE           STM32_TIM5_BASE
#  define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM5_CLKIN
#elif defined(CONFIG_STM32F7_TIM6_ADC2)
#  define ADC2_HAVE_TIMER           1
#  define ADC2_TIMER_BASE           STM32_TIM6_BASE
#  define ADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM6_CLKIN
#elif defined(CONFIG_STM32F7_TIM8_ADC2)
#  define ADC2_HAVE_TIMER           1
#  define ADC2_TIMER_BASE           STM32_TIM8_BASE
#  define ADC2_TIMER_PCLK_FREQUENCY STM32_APB2_TIM8_CLKIN
#else
#  undef  ADC2_HAVE_TIMER
#endif

#ifdef ADC2_HAVE_TIMER
#  ifndef CONFIG_STM32F7_ADC2_SAMPLE_FREQUENCY
#    error "CONFIG_STM32F7_ADC2_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32F7_ADC2_TIMTRIG
#    error "CONFIG_STM32F7_ADC2_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(CONFIG_STM32F7_TIM1_ADC3)
#  define ADC3_HAVE_TIMER           1
#  define ADC3_TIMER_BASE           STM32_TIM1_BASE
#  define ADC3_TIMER_PCLK_FREQUENCY STM32_APB2_TIM1_CLKIN
#elif defined(CONFIG_STM32F7_TIM2_ADC3)
#  define ADC3_HAVE_TIMER           1
#  define ADC3_TIMER_BASE           STM32_TIM2_BASE
#  define ADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_STM32F7_TIM3_ADC3)
#  define ADC3_HAVE_TIMER           1
#  define ADC3_TIMER_BASE           STM32_TIM3_BASE
#  define ADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_STM32F7_TIM4_ADC3)
#  define ADC3_HAVE_TIMER           1
#  define ADC3_TIMER_BASE           STM32_TIM4_BASE
#  define ADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_STM32F7_TIM5_ADC3)
#  define ADC3_HAVE_TIMER           1
#  define ADC3_TIMER_BASE           STM32_TIM5_BASE
#  define ADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM5_CLKIN
#elif defined(CONFIG_STM32F7_TIM8_ADC3)
#  define ADC3_HAVE_TIMER           1
#  define ADC3_TIMER_BASE           STM32_TIM8_BASE
#  define ADC3_TIMER_PCLK_FREQUENCY STM32_APB2_TIM8_CLKIN
#else
#  undef  ADC3_HAVE_TIMER
#endif

#ifdef ADC3_HAVE_TIMER
#  ifndef CONFIG_STM32F7_ADC3_SAMPLE_FREQUENCY
#    error "CONFIG_STM32F7_ADC3_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32F7_ADC3_TIMTRIG
#    error "CONFIG_STM32F7_ADC3_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO"
#  endif
#endif

#if defined(ADC1_HAVE_TIMER) || defined(ADC2_HAVE_TIMER) || \
    defined(ADC3_HAVE_TIMER)
#  define ADC_HAVE_TIMER 1
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
 */

#define ADC1_EXTSEL_T1CC1      ADC_CR2_EXTSEL_T1CC1
#define ADC1_EXTSEL_T1CC2      ADC_CR2_EXTSEL_T1CC2
#define ADC1_EXTSEL_T1CC3      ADC_CR2_EXTSEL_T1CC3
#define ADC1_EXTSEL_T1CC4      ADC_CR2_EXTSEL_T1CC4
#define ADC1_EXTSEL_T1TRGO     ADC_CR2_EXTSEL_T1TRGO
#define ADC1_EXTSEL_T1TRGO2    ADC_CR2_EXTSEL_T1TRGO2
#define ADC2_EXTSEL_T1CC1      ADC_CR2_EXTSEL_T1CC1
#define ADC2_EXTSEL_T1CC2      ADC_CR2_EXTSEL_T1CC2
#define ADC2_EXTSEL_T1CC3      ADC_CR2_EXTSEL_T1CC3
#define ADC2_EXTSEL_T1CC4      ADC_CR2_EXTSEL_T1CC4
#define ADC2_EXTSEL_T1TRGO     ADC_CR2_EXTSEL_T1TRGO
#define ADC2_EXTSEL_T1TRGO2    ADC_CR2_EXTSEL_T1TRGO2
#define ADC3_EXTSEL_T1CC1      ADC_CR2_EXTSEL_T1CC1
#define ADC3_EXTSEL_T1CC2      ADC_CR2_EXTSEL_T1CC2
#define ADC3_EXTSEL_T1CC3      ADC_CR2_EXTSEL_T1CC3
#define ADC3_EXTSEL_T1CC4      ADC_CR2_EXTSEL_T1CC4
#define ADC3_EXTSEL_T1TRGO     ADC_CR2_EXTSEL_T1TRGO
#define ADC3_EXTSEL_T1TRGO2    ADC_CR2_EXTSEL_T1TRGO2

#define ADC1_EXTSEL_T2CC1      ADC_CR2_EXTSEL_T2CC1
#define ADC1_EXTSEL_T2CC2      ADC_CR2_EXTSEL_T2CC2
#define ADC1_EXTSEL_T2CC3      ADC_CR2_EXTSEL_T2CC3
#define ADC1_EXTSEL_T2CC4      ADC_CR2_EXTSEL_T2CC4
#define ADC1_EXTSEL_T2TRGO     ADC_CR2_EXTSEL_T2TRGO
#define ADC2_EXTSEL_T2CC1      ADC_CR2_EXTSEL_T2CC1
#define ADC2_EXTSEL_T2CC2      ADC_CR2_EXTSEL_T2CC2
#define ADC2_EXTSEL_T2CC3      ADC_CR2_EXTSEL_T2CC3
#define ADC2_EXTSEL_T2CC4      ADC_CR2_EXTSEL_T2CC4
#define ADC2_EXTSEL_T2TRGO     ADC_CR2_EXTSEL_T2TRGO
#define ADC3_EXTSEL_T2CC1      ADC_CR2_EXTSEL_T2CC1
#define ADC3_EXTSEL_T2CC2      ADC_CR2_EXTSEL_T2CC2
#define ADC3_EXTSEL_T2CC3      ADC_CR2_EXTSEL_T2CC3
#define ADC3_EXTSEL_T2CC4      ADC_CR2_EXTSEL_T2CC4
#define ADC3_EXTSEL_T2TRGO     ADC_CR2_EXTSEL_T2TRGO

#define ADC1_EXTSEL_T3CC1      ADC_CR2_EXTSEL_T3CC1
#define ADC1_EXTSEL_T3CC2      ADC_CR2_EXTSEL_T3CC2
#define ADC1_EXTSEL_T3CC3      ADC_CR2_EXTSEL_T3CC3
#define ADC1_EXTSEL_T3CC4      ADC_CR2_EXTSEL_T3CC4
#define ADC1_EXTSEL_T3TRGO     ADC_CR2_EXTSEL_T3TRGO
#define ADC2_EXTSEL_T3CC1      ADC_CR2_EXTSEL_T3CC1
#define ADC2_EXTSEL_T3CC2      ADC_CR2_EXTSEL_T3CC2
#define ADC2_EXTSEL_T3CC3      ADC_CR2_EXTSEL_T3CC3
#define ADC2_EXTSEL_T3CC4      ADC_CR2_EXTSEL_T3CC4
#define ADC2_EXTSEL_T3TRGO     ADC_CR2_EXTSEL_T3TRGO
#define ADC3_EXTSEL_T3CC1      ADC_CR2_EXTSEL_T3CC1
#define ADC3_EXTSEL_T3CC2      ADC_CR2_EXTSEL_T3CC2
#define ADC3_EXTSEL_T3CC3      ADC_CR2_EXTSEL_T3CC3
#define ADC3_EXTSEL_T3CC4      ADC_CR2_EXTSEL_T3CC4
#define ADC3_EXTSEL_T3TRGO     ADC_CR2_EXTSEL_T3TRGO

#define ADC1_EXTSEL_T4CC1      ADC_CR2_EXTSEL_T4CC1
#define ADC1_EXTSEL_T4CC2      ADC_CR2_EXTSEL_T4CC2
#define ADC1_EXTSEL_T4CC3      ADC_CR2_EXTSEL_T4CC3
#define ADC1_EXTSEL_T4CC4      ADC_CR2_EXTSEL_T4CC4
#define ADC1_EXTSEL_T4TRGO     ADC_CR2_EXTSEL_T4TRGO
#define ADC2_EXTSEL_T4CC1      ADC_CR2_EXTSEL_T4CC1
#define ADC2_EXTSEL_T4CC2      ADC_CR2_EXTSEL_T4CC2
#define ADC2_EXTSEL_T4CC3      ADC_CR2_EXTSEL_T4CC3
#define ADC2_EXTSEL_T4CC4      ADC_CR2_EXTSEL_T4CC4
#define ADC2_EXTSEL_T4TRGO     ADC_CR2_EXTSEL_T4TRGO
#define ADC3_EXTSEL_T4CC1      ADC_CR2_EXTSEL_T4CC1
#define ADC3_EXTSEL_T4CC2      ADC_CR2_EXTSEL_T4CC2
#define ADC3_EXTSEL_T4CC3      ADC_CR2_EXTSEL_T4CC3
#define ADC3_EXTSEL_T4CC4      ADC_CR2_EXTSEL_T4CC4
#define ADC3_EXTSEL_T4TRGO     ADC_CR2_EXTSEL_T4TRGO

#define ADC1_EXTSEL_T5CC1      ADC_CR2_EXTSEL_T5CC1
#define ADC1_EXTSEL_T5CC2      ADC_CR2_EXTSEL_T5CC2
#define ADC1_EXTSEL_T5CC3      ADC_CR2_EXTSEL_T5CC3
#define ADC1_EXTSEL_T5CC4      ADC_CR2_EXTSEL_T5CC4
#define ADC1_EXTSEL_T5TRGO     ADC_CR2_EXTSEL_T5TRGO
#define ADC2_EXTSEL_T5CC1      ADC_CR2_EXTSEL_T5CC1
#define ADC2_EXTSEL_T5CC2      ADC_CR2_EXTSEL_T5CC2
#define ADC2_EXTSEL_T5CC3      ADC_CR2_EXTSEL_T5CC3
#define ADC2_EXTSEL_T5CC4      ADC_CR2_EXTSEL_T5CC4
#define ADC2_EXTSEL_T5TRGO     ADC_CR2_EXTSEL_T5TRGO
#define ADC3_EXTSEL_T5CC1      ADC_CR2_EXTSEL_T5CC1
#define ADC3_EXTSEL_T5CC2      ADC_CR2_EXTSEL_T5CC2
#define ADC3_EXTSEL_T5CC3      ADC_CR2_EXTSEL_T5CC3
#define ADC3_EXTSEL_T5CC4      ADC_CR2_EXTSEL_T5CC4
#define ADC3_EXTSEL_T5TRGO     ADC_CR2_EXTSEL_T5TRGO

#define ADC1_EXTSEL_T6CC1      ADC_CR2_EXTSEL_T6CC1
#define ADC1_EXTSEL_T6CC2      ADC_CR2_EXTSEL_T6CC2
#define ADC1_EXTSEL_T6CC3      ADC_CR2_EXTSEL_T6CC3
#define ADC1_EXTSEL_T6CC4      ADC_CR2_EXTSEL_T6CC4
#define ADC1_EXTSEL_T6TRGO     ADC_CR2_EXTSEL_T6TRGO
#define ADC2_EXTSEL_T6CC1      ADC_CR2_EXTSEL_T6CC1
#define ADC2_EXTSEL_T6CC2      ADC_CR2_EXTSEL_T6CC2
#define ADC2_EXTSEL_T6CC3      ADC_CR2_EXTSEL_T6CC3
#define ADC2_EXTSEL_T6CC4      ADC_CR2_EXTSEL_T6CC4
#define ADC2_EXTSEL_T6TRGO     ADC_CR2_EXTSEL_T6TRGO
#define ADC3_EXTSEL_T6CC1      ADC_CR2_EXTSEL_T6CC1
#define ADC3_EXTSEL_T6CC2      ADC_CR2_EXTSEL_T6CC2
#define ADC3_EXTSEL_T6CC3      ADC_CR2_EXTSEL_T6CC3
#define ADC3_EXTSEL_T6CC4      ADC_CR2_EXTSEL_T6CC4
#define ADC3_EXTSEL_T6TRGO     ADC_CR2_EXTSEL_T6TRGO

#define ADC1_EXTSEL_T8CC1      ADC_CR2_EXTSEL_T8CC1
#define ADC1_EXTSEL_T8CC2      ADC_CR2_EXTSEL_T8CC2
#define ADC1_EXTSEL_T8CC3      ADC_CR2_EXTSEL_T8CC3
#define ADC1_EXTSEL_T8CC4      ADC_CR2_EXTSEL_T8CC4
#define ADC1_EXTSEL_T8TRGO     ADC_CR2_EXTSEL_T8TRGO
#define ADC1_EXTSEL_T8TRGO2    ADC_CR2_EXTSEL_T8TRGO2
#define ADC2_EXTSEL_T8CC1      ADC_CR2_EXTSEL_T8CC1
#define ADC2_EXTSEL_T8CC2      ADC_CR2_EXTSEL_T8CC2
#define ADC2_EXTSEL_T8CC3      ADC_CR2_EXTSEL_T8CC3
#define ADC2_EXTSEL_T8CC4      ADC_CR2_EXTSEL_T8CC4
#define ADC2_EXTSEL_T8TRGO     ADC_CR2_EXTSEL_T8TRGO
#define ADC2_EXTSEL_T8TRGO2    ADC_CR2_EXTSEL_T8TRGO2
#define ADC3_EXTSEL_T8CC1      ADC_CR2_EXTSEL_T8CC1
#define ADC3_EXTSEL_T8CC2      ADC_CR2_EXTSEL_T8CC2
#define ADC3_EXTSEL_T8CC3      ADC_CR2_EXTSEL_T8CC3
#define ADC3_EXTSEL_T8CC4      ADC_CR2_EXTSEL_T8CC4
#define ADC3_EXTSEL_T8TRGO     ADC_CR2_EXTSEL_T8TRGO
#define ADC3_EXTSEL_T8TRGO2    ADC_CR2_EXTSEL_T8TRGO2

/* EXTSEL configuration *****************************************************/

/* NOTE:
 * this configuration if used only if CONFIG_STM32F7_TIMx_ADCy is selected.
 * You can still connect the ADC with a timer trigger using the
 * CONFIG_STM32F7_ADCx_EXTSEL option.
 */

#if defined(CONFIG_STM32F7_TIM1_ADC1)
#  if CONFIG_STM32F7_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T1CC1
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T1CC2
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T1CC3
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T1CC4
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T1TRGO
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 5
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T1TRGO2
#  else
#    error "CONFIG_STM32F7_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM2_ADC1)
#  if CONFIG_STM32F7_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T2CC1
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T2CC2
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T2CC3
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T2CC4
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T2TRGO
#  else
#    error "CONFIG_STM32F7_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM3_ADC1)
#  if CONFIG_STM32F7_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T3CC1
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T3CC2
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T3CC3
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T3CC4
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T3TRGO
#  else
#    error "CONFIG_STM32F7_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM4_ADC1)
#  if CONFIG_STM32F7_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T4CC1
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T4CC2
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T4CC3
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T4CC4
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T4TRGO
#  else
#    error "CONFIG_STM32F7_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM5_ADC1)
#  if CONFIG_STM32F7_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T5CC1
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T5CC2
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T5CC3
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T5CC4
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T5TRGO
#  else
#    error "CONFIG_STM32F7_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM6_ADC1)
#  if CONFIG_STM32F7_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T6CC1
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T6CC2
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T6CC3
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T6CC4
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T6TRGO
#  else
#    error "CONFIG_STM32F7_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM8_ADC1)
#  if CONFIG_STM32F7_ADC1_TIMTRIG == 0
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T8CC1
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T8CC2
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 2
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T8CC3
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T8CC4
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T8TRGO
#  elif CONFIG_STM32F7_ADC1_TIMTRIG == 5
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T8TRGO2
#  else
#    error "CONFIG_STM32F7_ADC1_TIMTRIG is out of range"
#  endif
#endif

#if defined(CONFIG_STM32F7_TIM1_ADC2)
#  if CONFIG_STM32F7_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T1CC1
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T1CC2
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T1CC3
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T1CC4
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T1TRGO
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 5
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T1TRGO2
#  else
#    error "CONFIG_STM32F7_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM2_ADC2)
#  if CONFIG_STM32F7_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T2CC1
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T2CC2
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T2CC3
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T2CC4
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T2TRGO
#  else
#    error "CONFIG_STM32F7_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM3_ADC2)
#  if CONFIG_STM32F7_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T3CC1
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T3CC2
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T3CC3
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T3CC4
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T3TRGO
#  else
#    error "CONFIG_STM32F7_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM4_ADC2)
#  if CONFIG_STM32F7_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T4CC1
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T4CC2
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T4CC3
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T4CC4
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T4TRGO
#  else
#    error "CONFIG_STM32F7_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM5_ADC2)
#  if CONFIG_STM32F7_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T5CC1
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T5CC2
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T5CC3
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T5CC4
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T5TRGO
#  else
#    error "CONFIG_STM32F7_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM6_ADC2)
#  if CONFIG_STM32F7_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T6CC1
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T6CC2
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T6CC3
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T6CC4
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T6TRGO
#  else
#    error "CONFIG_STM32F7_ADC2_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM8_ADC2)
#  if CONFIG_STM32F7_ADC2_TIMTRIG == 0
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T8CC1
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 1
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T8CC2
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 2
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T8CC3
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 3
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T8CC4
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 4
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T8TRGO
#  elif CONFIG_STM32F7_ADC2_TIMTRIG == 5
#    define ADC2_EXTSEL_VALUE ADC2_EXTSEL_T8TRGO2
#  else
#    error "CONFIG_STM32F7_ADC2_TIMTRIG is out of range"
#  endif
#endif

#if defined(CONFIG_STM32F7_TIM1_ADC3)
#  if CONFIG_STM32F7_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T1CC1
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T1CC2
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T1CC3
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T1CC4
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T1TRGO
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 5
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T1TRGO2
#  else
#    error "CONFIG_STM32F7_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM2_ADC3)
#  if CONFIG_STM32F7_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T2CC1
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T2CC2
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T2CC3
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T2CC4
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T2TRGO
#  else
#    error "CONFIG_STM32F7_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM3_ADC3)
#  if CONFIG_STM32F7_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T3CC1
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T3CC2
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T3CC3
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T3CC4
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T3TRGO
#  else
#    error "CONFIG_STM32F7_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM4_ADC3)
#  if CONFIG_STM32F7_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T4CC1
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T4CC2
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T4CC3
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T4CC4
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T4TRGO
#  else
#    error "CONFIG_STM32F7_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM5_ADC3)
#  if CONFIG_STM32F7_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T5CC1
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T5CC2
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T5CC3
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T5CC4
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T5TRGO
#  else
#    error "CONFIG_STM32F7_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM6_ADC3)
#  if CONFIG_STM32F7_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T6CC1
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T6CC2
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T6CC3
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T6CC4
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T6TRGO
#  else
#    error "CONFIG_STM32F7_ADC3_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F7_TIM8_ADC3)
#  if CONFIG_STM32F7_ADC3_TIMTRIG == 0
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T8CC1
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 1
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T8CC2
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 2
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T8CC3
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 3
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T8CC4
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 4
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T8TRGO
#  elif CONFIG_STM32F7_ADC3_TIMTRIG == 5
#    define ADC3_EXTSEL_VALUE ADC3_EXTSEL_T8TRGO2
#  else
#    error "CONFIG_STM32F7_ADC3_TIMTRIG is out of range"
#  endif
#endif

/* Regular channels external trigger support */

#ifdef ADC1_EXTSEL_VALUE
#  define ADC1_HAVE_EXTCFG  1
#  define ADC1_EXTCFG_VALUE (ADC1_EXTSEL_VALUE | ADC_EXTREG_EXTEN_DEFAULT)
#elif defined(CONFIG_STM32F7_ADC1_EXTSEL)
#  define ADC1_HAVE_EXTCFG  1
#  define ADC1_EXTCFG_VALUE 0
#else
#  undef ADC1_HAVE_EXTCFG
#endif
#ifdef ADC2_EXTSEL_VALUE
#  define ADC2_HAVE_EXTCFG  1
#  define ADC2_EXTCFG_VALUE (ADC2_EXTSEL_VALUE | ADC_EXTREG_EXTEN_DEFAULT)
#elif defined(CONFIG_STM32F7_ADC2_EXTSEL)
#  define ADC2_HAVE_EXTCFG  1
#  define ADC2_EXTCFG_VALUE 0
#else
#  undef ADC2_HAVE_EXTCFG
#endif
#ifdef ADC3_EXTSEL_VALUE
#  define ADC3_HAVE_EXTCFG  1
#  define ADC3_EXTCFG_VALUE (ADC3_EXTSEL_VALUE | ADC_EXTREG_EXTEN_DEFAULT)
#elif defined(CONFIG_STM32F7_ADC3_EXTSEL)
#  define ADC3_HAVE_EXTCFG  1
#  define ADC3_EXTCFG_VALUE 0
#else
#  undef ADC3_HAVE_EXTCFG
#endif

#if defined(ADC1_HAVE_EXTCFG) || defined(ADC2_HAVE_EXTCFG) || \
    defined(ADC3_HAVE_EXTCFG) || defined(ADC3_HAVE_EXTCFG)
#  define ADC_HAVE_EXTCFG
#endif

/* JEXTSEL configuration ****************************************************/

/* There is no automatic timer tirgger configuration from Kconfig for
 * injected channels conversion.
 */

/* Injected channels external trigger support */

#ifdef ADC1_JEXTSEL_VALUE
#  define ADC1_HAVE_JEXTCFG  1
#  define ADC1_JEXTCFG_VALUE (ADC1_JEXTSEL_VALUE | ADC_JEXTREG_JEXTEN_DEFAULT)
#elif defined(CONFIG_STM32F7_ADC1_JEXTSEL)
#  define ADC1_HAVE_JEXTCFG  1
#  define ADC1_JEXTCFG_VALUE 0
#else
#  undef ADC1_HAVE_JEXTCFG
#endif
#ifdef ADC2_JEXTSEL_VALUE
#  define ADC2_HAVE_JEXTCFG  1
#  define ADC2_JEXTCFG_VALUE (ADC2_JEXTSEL_VALUE | ADC_JEXTREG_JEXTEN_DEFAULT)
#elif defined(CONFIG_STM32F7_ADC2_JEXTSEL)
#  define ADC2_HAVE_JEXTCFG  1
#  define ADC2_JEXTCFG_VALUE 0
#else
#  undef ADC2_HAVE_JEXTCFG
#endif
#ifdef ADC3_JEXTSEL_VALUE
#  define ADC3_HAVE_JEXTCFG  1
#  define ADC3_JEXTCFG_VALUE (ADC3_JEXTSEL_VALUE | ADC_JEXTREG_JEXTEN_DEFAULT)
#elif defined(CONFIG_STM32F7_ADC3_JEXTSEL)
#  define ADC3_HAVE_JEXTCFG  1
#  define ADC3_JEXTCFG_VALUE 0
#else
#  undef ADC3_HAVE_JEXTCFG
#endif

#if defined(ADC1_HAVE_JEXTCFG) || defined(ADC2_HAVE_JEXTCFG) || \
    defined(ADC3_HAVE_JEXTCFG)
#  define ADC_HAVE_JEXTCFG
#endif

/* ADC interrupts ***********************************************************/

#define ADC_ISR_EOC                ADC_SR_EOC
#define ADC_IER_EOC                ADC_CR1_EOCIE
#define ADC_ISR_AWD                ADC_SR_AWD
#define ADC_IER_AWD                ADC_CR1_AWDIE
#define ADC_ISR_JEOC               ADC_SR_JEOC
#define ADC_IER_JEOC               ADC_CR1_JEOCIE
#define ADC_ISR_JEOS               0 /* No JEOS */
#define ADC_IER_JEOS               0 /* No JEOS */
#define ADC_ISR_OVR                ADC_SR_OVR
#define ADC_IER_OVR                ADC_CR1_OVRIE

#define ADC_ISR_ALLINTS (ADC_ISR_EOC | ADC_ISR_AWD | ADC_ISR_JEOC | \
                         ADC_ISR_JEOS | ADC_ISR_OVR)
#define ADC_IER_ALLINTS (ADC_IER_EOC | ADC_IER_AWD | ADC_IER_JEOC | \
                         ADC_IER_JEOS | ADC_IER_OVR)

/* Low-level ops helpers ****************************************************/

#define STM32_ADC_INT_ACK(adc, source)              \
        (adc)->llops->int_ack(adc, source)
#define STM32_ADC_INT_GET(adc)                      \
        (adc)->llops->int_get(adc)
#define STM32_ADC_INT_ENABLE(adc, source)           \
        (adc)->llops->int_en(adc, source)
#define STM32_ADC_INT_DISABLE(adc, source)          \
        (adc)->llops->int_dis(adc, source)
#define STM32_ADC_REGDATA_GET(adc)                  \
        (adc)->llops->val_get(adc)
#define STM32_ADC_REGBUF_REGISTER(adc, buffer, len) \
        (adc)->llops->regbuf_reg(adc, buffer, len)
#define STM32_ADC_REG_STARTCONV(adc, state)         \
        (adc)->llops->reg_startconv(adc, state)
#define STM32_ADC_OFFSET_SET(adc, ch, i, o)         \
        (adc)->llops->offset_set(adc, ch, i, o)
#define STM32_ADC_EXTCFG_SET(adc, c)                \
        (adc)->llops->extcfg_set(adc, c)
#define STM32_ADC_INJ_STARTCONV(adc, state)         \
        (adc)->llops->inj_startconv(adc, state)
#define STM32_ADC_INJDATA_GET(adc, chan)            \
        (adc)->llops->inj_get(adc, chan)
#define STM32_ADC_JEXTCFG_SET(adc, c)               \
        (adc)->llops->jextcfg_set(adc, c)
#define STM32_ADC_SAMPLETIME_SET(adc, time_samples) \
        (adc)->llops->stime_set(adc, time_samples)
#define STM32_ADC_SAMPLETIME_WRITE(adc)             \
        (adc)->llops->stime_write(adc)
#define STM32_ADC_DUMP_REGS(adc)                    \
        (adc)->llops->dump_regs(adc)
#define STM32_ADC_SETUP(adc)                        \
        (adc)->llops->setup(adc)
#define STM32_ADC_SHUTDOWN(adc)                     \
        (adc)->llops->shutdown(adc)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum adc_io_cmds_e
{
  IO_STOP_ADC,
  IO_START_ADC,
  IO_START_CONV,
  IO_TRIGGER_REG,
#ifdef ADC_HAVE_INJECTED
  IO_TRIGGER_INJ,
#endif
};

/* ADC resolution can be reduced in order to perform faster conversion */

enum stm32_adc_resoluton_e
{
  ADC_RESOLUTION_12BIT = 0,     /* 12 bit */
  ADC_RESOLUTION_10BIT = 1,     /* 10 bit */
  ADC_RESOLUTION_8BIT  = 2,     /* 8 bit */
  ADC_RESOLUTION_6BIT  = 3      /* 6 bit */
};

#ifdef CONFIG_STM32F7_ADC_LL_OPS

#ifdef CONFIG_STM32F7_ADC_CHANGE_SAMPLETIME

/* Channel and sample time pair */

typedef struct adc_channel_s
{
  uint8_t channel:5;

  /* Sampling time individually for each channel */

  uint8_t sample_time:3;
} adc_channel_t;

/* This structure will be used while setting channels to specified by the
 * "channel-sample time" pairs' values
 */

struct adc_sample_time_s
{
  adc_channel_t *channel;                /* Array of channels */
  uint8_t        channels_nbr:5;         /* Number of channels in array */
  bool           all_same:1;             /* All channels will get the
                                          * same value of the sample time */
  uint8_t        all_ch_sample_time:3;   /* Sample time for all channels */
};
#endif /* CONFIG_STM32F7_ADC_CHANGE_SAMPLETIME */

/* This structure provides the publicly visible representation of the
 * "lower-half" ADC driver structure.
 */

struct stm32_adc_dev_s
{
  /* Publicly visible portion of the "lower-half" ADC driver structure */

  const struct stm32_adc_ops_s *llops;

  /* Require cast-compatibility with private "lower-half" ADC structure */
};

/* Low-level operations for ADC */

struct stm32_adc_ops_s
{
  /* Low-level ADC setup */

  int (*setup)(struct stm32_adc_dev_s *dev);

  /* Low-level ADC shutdown */

  void (*shutdown)(struct stm32_adc_dev_s *dev);

  /* Acknowledge interrupts */

  void (*int_ack)(struct stm32_adc_dev_s *dev, uint32_t source);

  /* Get pending interrupts */

  uint32_t (*int_get)(struct stm32_adc_dev_s *dev);

  /* Enable interrupts */

  void (*int_en)(struct stm32_adc_dev_s *dev, uint32_t source);

  /* Disable interrupts */

  void (*int_dis)(struct stm32_adc_dev_s *dev, uint32_t source);

  /* Get current ADC data register */

  uint32_t (*val_get)(struct stm32_adc_dev_s *dev);

  /* Register buffer for ADC DMA transfer */

  int (*regbuf_reg)(struct stm32_adc_dev_s *dev,
                    uint16_t *buffer, uint8_t len);

  /* Start/stop regular conversion */

  void (*reg_startconv)(struct stm32_adc_dev_s *dev, bool state);

  /* Set offset for channel */

  int (*offset_set)(struct stm32_adc_dev_s *dev, uint8_t ch, uint8_t i,
                    uint16_t offset);

#ifdef ADC_HAVE_EXTCFG
  /* Configure the ADC external trigger for regular conversion */

  void (*extcfg_set)(struct stm32_adc_dev_s *dev, uint32_t extcfg);
#endif

#ifdef ADC_HAVE_JEXTCFG
  /* Configure the ADC external trigger for injected conversion */

  void (*jextcfg_set)(struct stm32_adc_dev_s *dev, uint32_t jextcfg);
#endif

#ifdef ADC_HAVE_INJECTED
  /* Get current ADC injected data register */

  uint32_t (*inj_get)(struct stm32_adc_dev_s *dev, uint8_t chan);

  /* Start/stop injected conversion */

  void (*inj_startconv)(struct stm32_adc_dev_s *dev, bool state);
#endif

#ifdef CONFIG_STM32F7_ADC_CHANGE_SAMPLETIME
  /* Set ADC sample time */

  void (*stime_set)(struct stm32_adc_dev_s *dev,
                    struct adc_sample_time_s *time_samples);

  /* Write ADC sample time */

  void (*stime_write)(struct stm32_adc_dev_s *dev);
#endif

  void (*dump_regs)(struct stm32_adc_dev_s *dev);
};

#endif /* CONFIG_STM32F7_ADC_LL_OPS */

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
 * Name: stm32_adc_initialize
 *
 * Description:
 *   Initialize the ADC. See stm32_adc.c for more details.
 *
 * Input Parameters:
 *   intf      - Could be {1,2,3} for ADC1, ADC2, ADC3
 *   chanlist  - The list of channels (regular + injected)
 *   nchannels - Number of channels (regular + injected)
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s;
struct adc_dev_s *stm32_adc_initialize(int intf,
                                       const uint8_t *chanlist,
                                       int nchannels);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32F7_ADC1 || CONFIG_STM32F7_ADC2 ||
        * CONFIG_STM32F7_ADC3
        */
#endif /* __ARCH_ARM_SRC_STM32F7_STM32_ADC_H */
