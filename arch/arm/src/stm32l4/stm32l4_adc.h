/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_adc.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_ADC_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/adc.h>
#include "chip.h"
#include "hardware/stm32l4_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to control periodic ADC sampling.  If CONFIG_STM32L4_TIMn is defined then
 * CONFIG_STM32L4_TIMn_ADC must also be defined to indicate that timer "n" is
 * intended to be used for that purpose. Timers 1,2,3,6 and 15 may be used on
 * STM32L4X3, while STM32L4X6 adds support for timers 4 and 8 as well.
 */

#ifndef CONFIG_STM32L4_TIM1
#  undef CONFIG_STM32L4_TIM1_ADC
#  undef CONFIG_STM32L4_TIM1_ADC1
#  undef CONFIG_STM32L4_TIM1_ADC2
#  undef CONFIG_STM32L4_TIM1_ADC3
#endif
#ifndef CONFIG_STM32L4_TIM2
#  undef CONFIG_STM32L4_TIM2_ADC
#  undef CONFIG_STM32L4_TIM2_ADC1
#  undef CONFIG_STM32L4_TIM2_ADC2
#  undef CONFIG_STM32L4_TIM2_ADC3
#endif
#ifndef CONFIG_STM32L4_TIM3
#  undef CONFIG_STM32L4_TIM3_ADC
#  undef CONFIG_STM32L4_TIM3_ADC1
#  undef CONFIG_STM32L4_TIM3_ADC2
#  undef CONFIG_STM32L4_TIM3_ADC3
#endif
#ifndef CONFIG_STM32L4_TIM4
#  undef CONFIG_STM32L4_TIM4_ADC
#  undef CONFIG_STM32L4_TIM4_ADC1
#  undef CONFIG_STM32L4_TIM4_ADC2
#  undef CONFIG_STM32L4_TIM4_ADC3
#endif
#ifndef CONFIG_STM32L4_TIM6
#  undef CONFIG_STM32L4_TIM6_ADC
#  undef CONFIG_STM32L4_TIM6_ADC1
#  undef CONFIG_STM32L4_TIM6_ADC2
#  undef CONFIG_STM32L4_TIM6_ADC3
#endif
#ifndef CONFIG_STM32L4_TIM8
#  undef CONFIG_STM32L4_TIM8_ADC
#  undef CONFIG_STM32L4_TIM8_ADC1
#  undef CONFIG_STM32L4_TIM8_ADC2
#  undef CONFIG_STM32L4_TIM8_ADC3
#endif
#ifndef CONFIG_STM32L4_TIM15
#  undef CONFIG_STM32L4_TIM15_ADC
#  undef CONFIG_STM32L4_TIM15_ADC1
#  undef CONFIG_STM32L4_TIM15_ADC2
#  undef CONFIG_STM32L4_TIM15_ADC3
#endif

/* Up to 3 ADC interfaces are supported */

#if STM32L4_NADC < 3
#  undef CONFIG_STM32L4_ADC3
#endif

#if STM32L4_NADC < 2
#  undef CONFIG_STM32L4_ADC2
#endif

#if STM32L4_NADC < 1
#  undef CONFIG_STM32L4_ADC1
#endif

#if defined(CONFIG_STM32L4_ADC1) || defined(CONFIG_STM32L4_ADC2) || \
    defined(CONFIG_STM32L4_ADC3)

/* ADC output to DFSDM support. Note that DFSDM and DMA are
 * mutually exclusive.
 */

#undef ADC_HAVE_DFSDM
#if defined(CONFIG_STM32L4_ADC1_OUTPUT_DFSDM) || \
    defined(CONFIG_STM32L4_ADC2_OUTPUT_DFSDM) || \
    defined(CONFIG_STM32L4_ADC3_OUTPUT_DFSDM)
#  define ADC_HAVE_DFSDM
#endif

#if defined(CONFIG_STM32L4_ADC1_OUTPUT_DFSDM)
#  define ADC1_HAVE_DFSDM 1
#  undef  CONFIG_STM32L4_ADC1_DMA
#else
#  undef  ADC1_HAVE_DFSDM
#endif

#if defined(CONFIG_STM32L4_ADC2_OUTPUT_DFSDM)
#  define ADC2_HAVE_DFSDM 1
#  undef  CONFIG_STM32L4_ADC2_DMA
#else
#  undef  ADC2_HAVE_DFSDM
#endif

#if defined(CONFIG_STM32L4_ADC3_OUTPUT_DFSDM)
#  define ADC3_HAVE_DFSDM 1
#  undef  CONFIG_STM32L4_ADC3_DMA
#else
#  undef  ADC3_HAVE_DFSDM
#endif

/* DMA support */

#undef ADC_HAVE_DMA
#if defined(CONFIG_STM32L4_ADC1_DMA) || defined(CONFIG_STM32L4_ADC2_DMA) || \
    defined(CONFIG_STM32L4_ADC3_DMA)
#  define ADC_HAVE_DMA  1
#endif

#ifdef CONFIG_STM32L4_ADC1_DMA
#  define ADC1_HAVE_DMA 1
#else
#  undef  ADC1_HAVE_DMA
#endif

#ifdef CONFIG_STM32L4_ADC2_DMA
#  define ADC2_HAVE_DMA 1
#else
#  undef  ADC2_HAVE_DMA
#endif

#ifdef CONFIG_STM32L4_ADC3_DMA
#  define ADC3_HAVE_DMA 1
#else
#  undef  ADC3_HAVE_DMA
#endif

/* Injected channels support */

#if (defined(CONFIG_STM32L4_ADC1) && (CONFIG_STM32L4_ADC1_INJ_CHAN > 0)) || \
    (defined(CONFIG_STM32L4_ADC2) && (CONFIG_STM32L4_ADC2_INJ_CHAN > 0)) || \
    (defined(CONFIG_STM32L4_ADC3) && (CONFIG_STM32L4_ADC3_INJ_CHAN > 0))
#  define ADC_HAVE_INJECTED
#endif

/* Timer configuration:  If a timer trigger is specified, then get
 * information about the timer.
 */

#if defined(CONFIG_STM32L4_TIM1_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32L4_TIM1_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32L4_APB2_TIM1_CLKIN
#    define ADC1_TIMER_CHANNEL        CONFIG_STM32L4_TIM1_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM2_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32L4_TIM2_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM2_CLKIN
#    define ADC1_TIMER_CHANNEL        CONFIG_STM32L4_TIM2_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM3_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32L4_TIM3_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM3_CLKIN
#    define ADC1_TIMER_CHANNEL        CONFIG_STM32L4_TIM3_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM4_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32L4_TIM4_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM4_CLKIN
#    define ADC1_TIMER_CHANNEL        CONFIG_STM32L4_TIM4_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM6_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32L4_TIM6_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM6_CLKIN
#    define ADC1_TIMER_CHANNEL        CONFIG_STM32L4_TIM6_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM8_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32L4_TIM8_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32L4_APB2_TIM8_CLKIN
#    define ADC1_TIMER_CHANNEL        CONFIG_STM32L4_TIM8_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM15_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32L4_TIM15_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32L4_APB2_TIM15_CLKIN
#    define ADC1_TIMER_CHANNEL        CONFIG_STM32L4_TIM15_ADC_CHAN
#else
#    undef  ADC1_HAVE_TIMER
#endif

#ifdef ADC1_HAVE_TIMER
#  ifndef CONFIG_STM32L4_ADC1_SAMPLE_FREQUENCY
#    error "CONFIG_STM32L4_ADC1_SAMPLE_FREQUENCY not defined"
#  endif
#  if ((CONFIG_STM32L4_ADC1_EXTTRIG == 0) && \
       (CONFIG_STM32L4_ADC1_JEXTTRIG == 0))
#    error "ADC1 External trigger must be enabled"
#  endif
#endif

#if defined(CONFIG_STM32L4_TIM1_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32L4_TIM1_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32L4_APB2_TIM1_CLKIN
#    define ADC2_TIMER_CHANNEL        CONFIG_STM32L4_TIM1_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM2_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32L4_TIM2_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM2_CLKIN
#    define ADC2_TIMER_CHANNEL        CONFIG_STM32L4_TIM2_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM3_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32L4_TIM3_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM3_CLKIN
#    define ADC2_TIMER_CHANNEL        CONFIG_STM32L4_TIM3_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM4_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32L4_TIM4_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM4_CLKIN
#    define ADC2_TIMER_CHANNEL        CONFIG_STM32L4_TIM4_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM6_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32L4_TIM6_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM6_CLKIN
#    define ADC2_TIMER_CHANNEL        CONFIG_STM32L4_TIM6_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM8_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32L4_TIM8_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32L4_APB2_TIM8_CLKIN
#    define ADC2_TIMER_CHANNEL        CONFIG_STM32L4_TIM8_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM15_ADC2)
#    define ADC2_HAVE_TIMER           1
#    define ADC2_TIMER_BASE           STM32L4_TIM15_BASE
#    define ADC2_TIMER_PCLK_FREQUENCY STM32L4_APB2_TIM15_CLKIN
#    define ADC2_TIMER_CHANNEL        CONFIG_STM32L4_TIM15_ADC_CHAN
#else
#    undef  ADC2_HAVE_TIMER
#endif

#ifdef ADC2_HAVE_TIMER
#  ifndef CONFIG_STM32L4_ADC2_SAMPLE_FREQUENCY
#    error "CONFIG_STM32L4_ADC2_SAMPLE_FREQUENCY not defined"
#  endif
#  if ((CONFIG_STM32L4_ADC2_EXTTRIG == 0) && \
       (CONFIG_STM32L4_ADC2_JEXTTRIG == 0))
#    error "ADC2 External trigger must be enabled"
#  endif
#endif

#if defined(CONFIG_STM32L4_TIM1_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32L4_TIM1_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32L4_APB2_TIM1_CLKIN
#    define ADC3_TIMER_CHANNEL        CONFIG_STM32L4_TIM1_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM2_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32L4_TIM2_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM2_CLKIN
#    define ADC3_TIMER_CHANNEL        CONFIG_STM32L4_TIM1_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM3_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32L4_TIM3_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM3_CLKIN
#    define ADC3_TIMER_CHANNEL        CONFIG_STM32L4_TIM3_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM4_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32L4_TIM4_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM4_CLKIN
#    define ADC3_TIMER_CHANNEL        CONFIG_STM32L4_TIM4_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM6_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32L4_TIM6_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32L4_APB1_TIM6_CLKIN
#    define ADC3_TIMER_CHANNEL        CONFIG_STM32L4_TIM6_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM8_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32L4_TIM8_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32L4_APB2_TIM8_CLKIN
#    define ADC3_TIMER_CHANNEL        CONFIG_STM32L4_TIM8_ADC_CHAN
#elif defined(CONFIG_STM32L4_TIM15_ADC3)
#    define ADC3_HAVE_TIMER           1
#    define ADC3_TIMER_BASE           STM32L4_TIM15_BASE
#    define ADC3_TIMER_PCLK_FREQUENCY STM32L4_APB2_TIM15_CLKIN
#    define ADC3_TIMER_CHANNEL        CONFIG_STM32L4_TIM15_ADC_CHAN
#else
#    undef  ADC3_HAVE_TIMER
#endif

#ifdef ADC3_HAVE_TIMER
#  ifndef CONFIG_STM32L4_ADC3_SAMPLE_FREQUENCY
#    error "CONFIG_STM32L4_ADC3_SAMPLE_FREQUENCY not defined"
#  endif
#  if ((CONFIG_STM32L4_ADC3_EXTTRIG == 0) && \
       (CONFIG_STM32L4_ADC3_JEXTTRIG == 0))
#    error "ADC3 External trigger must be enabled"
#  endif
#endif

#if defined(ADC1_HAVE_TIMER) || defined(ADC2_HAVE_TIMER) || \
    defined(ADC3_HAVE_TIMER)
#  define ADC_HAVE_TIMER 1
#else
#  undef ADC_HAVE_TIMER
#endif

/* EXTSEL configuration *****************************************************/

/* If external trigger is enabled, (CONFIG_STM32L4_ADC1_EXTTRIG > 0),
 * ADCx_EXTSEL_VALUE is set based on trigger polarity and event number. No
 * effort is made to check if the configuration is valid.
 */

#ifdef CONFIG_STM32L4_ADC1_EXTTRIG
#  if CONFIG_STM32L4_ADC1_EXTTRIG > 0
#    define ADC1_EXTCFG_VALUE \
            ADC_CFGR_EXTEN(CONFIG_STM32L4_ADC1_EXTTRIG)  | \
            ADC_CFGR_EXTSEL(CONFIG_STM32L4_ADC1_EXTSEL)
#  endif
#endif /* CONFIG_STM32L4_ADC1_EXTTRIG */

#ifdef ADC1_EXTCFG_VALUE
#  define ADC1_HAVE_EXTCFG  1
#else
#  undef ADC1_HAVE_EXTCFG
#endif

#ifdef CONFIG_STM32L4_ADC2_EXTTRIG
#  if CONFIG_STM32L4_ADC2_EXTTRIG > 0
#    define ADC2_EXTCFG_VALUE \
            ADC_CFGR_EXTEN(CONFIG_STM32L4_ADC2_EXTTRIG)  | \
            ADC_CFGR_EXTSEL(CONFIG_STM32L4_ADC2_EXTSEL)
#  endif
#endif /* CONFIG_STM32L4_ADC2_EXTTRIG */

#ifdef ADC2_EXTCFG_VALUE
#  define ADC2_HAVE_EXTCFG  1
#else
#  undef ADC2_HAVE_EXTCFG
#endif

#ifdef CONFIG_STM32L4_ADC3_EXTTRIG
#  if CONFIG_STM32L4_ADC3_EXTTRIG > 0
#    define ADC3_EXTCFG_VALUE \
            ADC_CFGR_EXTEN(CONFIG_STM32L4_ADC3_EXTTRIG)  | \
            ADC_CFGR_EXTSEL(CONFIG_STM32L4_ADC3_EXTSEL)
#  endif
#endif /* CONFIG_STM32L4_ADC3_EXTTRIG */

#ifdef ADC3_EXTCFG_VALUE
#  define ADC3_HAVE_EXTCFG  1
#else
#  undef ADC3_HAVE_EXTCFG
#endif

#if defined(ADC1_HAVE_EXTCFG) || defined(ADC2_HAVE_EXTCFG) || \
    defined(ADC3_HAVE_EXTCFG)
#  define ADC_HAVE_EXTCFG
#endif

/* JEXTSEL configuration ****************************************************/

#ifdef CONFIG_STM32L4_ADC1_JEXTTRIG
#  if CONFIG_STM32L4_ADC1_JEXTTRIG > 0
#    define ADC1_JEXTCFG_VALUE \
            ADC_JSQR_JEXTEN(CONFIG_STM32L4_ADC1_JEXTTRIG) | \
            ADC_JSQR_JEXTSEL(CONFIG_STM32L4_ADC1_JEXTSEL)
#  endif
#endif /* CONFIG_STM32L4_ADC1_JEXTTRIG */

#ifdef ADC1_JEXTCFG_VALUE
#  define ADC1_HAVE_JEXTCFG  1
#endif

#ifdef CONFIG_STM32L4_ADC2_JEXTTRIG
#  if CONFIG_STM32L4_ADC2_JEXTTRIG > 0
#    define ADC2_JEXTCFG_VALUE \
            ADC_JSQR_JEXTEN(CONFIG_STM32L4_ADC2_JEXTTRIG) | \
            ADC_JSQR_JEXTSEL(CONFIG_STM32L4_ADC2_JEXTSEL)
#  endif
#endif /* CONFIG_STM32L4_ADC2_JEXTTRIG */

#ifdef ADC2_JEXTCFG_VALUE
#  define ADC2_HAVE_JEXTCFG  1
#endif

#ifdef CONFIG_STM32L4_ADC3_JEXTTRIG
#  if CONFIG_STM32L4_ADC3_JEXTTRIG > 0
#    define ADC3_JEXTCFG_VALUE \
            ADC_JSQR_JEXTEN(CONFIG_STM32L4_ADC3_JEXTTRIG) | \
            ADC_JSQR_JEXTSEL(CONFIG_STM32L4_ADC3_JEXTSEL)
#  endif
#endif /* CONFIG_STM32L4_ADC3_JEXTTRIG */

#ifdef ADC3_JEXTCFG_VALUE
#  define ADC3_HAVE_JEXTCFG  1
#endif

#if defined(ADC1_HAVE_JEXTCFG) || defined(ADC2_HAVE_JEXTCFG) || \
    defined(ADC3_HAVE_JEXTCFG)
#  define ADC_HAVE_JEXTCFG
#endif

/* ADC interrupts ***********************************************************/

#define ADC_ISR_EOC                  ADC_INT_EOC
#define ADC_IER_EOC                  ADC_INT_EOC
#define ADC_ISR_EOS                  ADC_INT_EOS
#define ADC_IER_EOS                  ADC_INT_EOS
#define ADC_ISR_AWD                  ADC_INT_AWD1
#define ADC_IER_AWD                  ADC_INT_AWD1
#define ADC_ISR_JEOC                 ADC_INT_JEOC
#define ADC_IER_JEOC                 ADC_INT_JEOC
#define ADC_ISR_OVR                  ADC_INT_OVR
#define ADC_IER_OVR                  ADC_INT_OVR
#define ADC_ISR_JEOS                 ADC_INT_JEOS
#define ADC_IER_JEOS                 ADC_INT_JEOS

#define ADC_ISR_ALLINTS (ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_AWD | \
                         ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_OVR)
#define ADC_IER_ALLINTS (ADC_IER_EOC | ADC_IER_EOS | ADC_IER_AWD | \
                         ADC_IER_JEOC | ADC_IER_JEOS | ADC_IER_OVR)

/* Low-level ops helpers ****************************************************/

#define ADC_INT_ACK(adc, source)                     \
        (adc)->llops->int_ack(adc, source)
#define ADC_INT_GET(adc)                             \
        (adc)->llops->int_get(adc)
#define ADC_INT_ENABLE(adc, source)                  \
        (adc)->llops->int_en(adc, source)
#define ADC_INT_DISABLE(adc, source)                 \
        (adc)->llops->int_dis(adc, source)
#define ADC_REGDATA_GET(adc)                         \
        (adc)->llops->val_get(adc)
#define ADC_INJDATA_GET(adc, chan)                   \
        (adc)->llops->inj_get(adc, chan)
#define ADC_REGBUF_REGISTER(adc, buffer, len)        \
        (adc)->llops->regbuf_reg(adc, buffer, len)
#define ADC_REG_STARTCONV(adc, state)                \
        (adc)->llops->reg_startconv(adc, state)
#define ADC_INJ_STARTCONV(adc, state)                \
        (adc)->llops->inj_startconv(adc, state)
#define ADC_OFFSET_SET(adc, ch, i, o)                \
        (adc)->llops->offset_set(adc, ch, i, o)
#define ADC_EXTSEL_SET(adc, extcfg)                  \
        (adc)->llops->extsel_set(adc, extcfg)
#define ADC_DUMP_REGS(adc)                           \
        (adc)->llops->dump_regs(adc)

/* IOCTL Commands ***********************************************************
 *
 * Cmd: ANIOC_STM32L4_TRIGGER_REG           Arg:
 * Cmd: ANIOC_STM32L4_TRIGGER_INJ           Arg:
 *
 */

#define ANIOC_STM32L4_TRIGGER_REG           _ANIOC(AN_STM32L4_FIRST + 0)
#define ANIOC_STM32L4_TRIGGER_INJ           _ANIOC(AN_STM32L4_FIRST + 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_STM32L4_ADC_LL_OPS

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

  int (*regbuf_reg)(struct stm32_adc_dev_s *dev, uint16_t *buffer,
                    uint8_t len);

  /* Start/stop regular conversion */

  void (*reg_startconv)(struct stm32_adc_dev_s *dev, bool state);

  /* Set offset for channel */

  int (*offset_set)(struct stm32_adc_dev_s *dev, uint8_t ch, uint8_t i,
                    uint16_t offset);

  /* Configure external event for regular group */

  int (*extsel_set)(struct stm32_adc_dev_s *dev, uint32_t extcfg);

#ifdef ADC_HAVE_JEXTCFG
  /* Configure the ADC external trigger for injected conversion */

  void (*jextsel_set)(struct stm32_adc_dev_s *dev, uint32_t jextcfg);
#endif

#ifdef ADC_HAVE_INJECTED
  /* Get current ADC injected data register */

  uint32_t (*inj_get)(struct stm32_adc_dev_s *dev, uint8_t chan);

  /* Start/stop injected conversion */

  void (*inj_startconv)(struct stm32_adc_dev_s *dev, bool state);
#endif

  /* Dump ADC regs */

  void (*dump_regs)(struct stm32_adc_dev_s *dev);
};

#endif  /* CONFIG_STM32L4_ADC_LL_OPS */

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
 * Name: stm32l4_adc_initialize
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
struct adc_dev_s *stm32l4_adc_initialize(int intf,
                                         const uint8_t *chanlist,
                                         int nchannels);
#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32L4_ADC1 || CONFIG_STM32L4_ADC2 || CONFIG_STM32L4_ADC3 */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_ADC_H */