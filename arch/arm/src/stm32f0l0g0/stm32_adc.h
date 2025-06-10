/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_adc.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_STM32_ADC_H
#define __ARCH_ARM_SRC_STM32F0L0G0_STM32_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#include <nuttx/analog/adc.h>
#include <arch/chip/chip.h>

#include "hardware/stm32_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to control periodic ADC sampling.  If CONFIG_STM32F0L0G0_TIMn is defined
 * then CONFIG_STM32F0L0G0_TIMn_ADC must also be defined to indicate that
 * timer "n" is intended to be used for that purpose. Timers 1-6 and 8 may
 * be used.
 */

#ifndef CONFIG_STM32F0L0G0_TIM1
#  undef CONFIG_STM32F0L0G0_TIM1_ADC
#  undef CONFIG_STM32F0L0G0_TIM1_ADC1
#endif
#ifndef CONFIG_STM32F0L0G0_TIM2
#  undef CONFIG_STM32F0L0G0_TIM2_ADC
#  undef CONFIG_STM32F0L0G0_TIM2_ADC1
#endif
#ifndef CONFIG_STM32F0L0G0_TIM3
#  undef CONFIG_STM32F0L0G0_TIM3_ADC
#  undef CONFIG_STM32F0L0G0_TIM3_ADC1
#endif
#ifndef CONFIG_STM32F0L0G0_TIM15
#  undef CONFIG_STM32F0L0G0_TIM15_ADC
#  undef CONFIG_STM32F0L0G0_TIM15_ADC1
#endif

/* Up to 1 ADC interfaces are supported */

#if STM32_NADC < 1
#  undef CONFIG_STM32F0L0G0_ADC1
#endif

#if defined(CONFIG_STM32F0L0G0_ADC1)

/* DMA support */

#undef ADC_HAVE_DMA
#if defined(CONFIG_STM32F0L0G0_ADC1_DMA)
#  define ADC_HAVE_DMA  1
#endif

#ifdef CONFIG_STM32F0L0G0_ADC1_DMA
#  define ADC1_HAVE_DMA 1
#else
#  undef  ADC1_HAVE_DMA
#endif

/* Timer configuration:  If a timer trigger is specified, then get
 * information about the timer.
 */

#if defined(CONFIG_STM32F0L0G0_TIM1_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM1_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM1_CLKIN
#elif defined(CONFIG_STM32F0L0G0_TIM2_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM2_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_STM32F0L0G0_TIM3_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM3_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_STM32F0L0G0_TIM15_ADC1)
#    define ADC1_HAVE_TIMER           1
#    define ADC1_TIMER_BASE           STM32_TIM15_BASE
#    define ADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM15_CLKIN
#else
#    undef  ADC1_HAVE_TIMER
#endif

#ifdef ADC1_HAVE_TIMER
#  ifndef CONFIG_STM32F0L0G0_ADC1_SAMPLE_FREQUENCY
#    error "CONFIG_STM32F0L0G0_ADC1_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32F0L0G0_ADC1_TIMTRIG
#    error "CONFIG_STM32F0L0G0_ADC1_TIMTRIG not defined"
#    warning "Values 0:CC1 1:CC2 2:CC3 3:CC4 4:TRGO 5:TRGO2"
#  endif
#endif

#if defined(ADC1_HAVE_TIMER)
#  define ADC_HAVE_TIMER 1
#else
#  undef ADC_HAVE_TIMER
#endif

/* EXTSEL */

#if defined(CONFIG_STM32F0L0G0_STM32F0)
#  define ADC1_EXTSEL_T1TRGO  ADC12_CFGR1_EXTSEL_TRG0
#  define ADC1_EXTSEL_T1CC4   ADC12_CFGR1_EXTSEL_TRG1
#  define ADC1_EXTSEL_T2TRGO  ADC12_CFGR1_EXTSEL_TRG2
#  define ADC1_EXTSEL_T3TRGO  ADC12_CFGR1_EXTSEL_TRG3
#  define ADC1_EXTSEL_T15TRGO ADC12_CFGR1_EXTSEL_TRG4
                              /* TRG5 reserved
                               * TRG6 reserved
                               * TRG7 reserved
                               */
#elif defined(CONFIG_STM32F0L0G0_STM32L0)
                              /* TRG0 reserved */
#  define ADC1_EXTSEL_T21CC2  ADC12_CFGR1_EXTSEL_TRG1
#  define ADC1_EXTSEL_T2TRGO  ADC12_CFGR1_EXTSEL_TRG2
#  define ADC1_EXTSEL_T2CC4   ADC12_CFGR1_EXTSEL_TRG3
#  define ADC1_EXTSEL_T21TRGO ADC12_CFGR1_EXTSEL_TRG4
#  define ADC1_EXTSEL_T2CC3   ADC12_CFGR1_EXTSEL_TRG5
                              /* TRG6 reserved */
#  define ADC1_EXTSEL_EXTI11  ADC12_CFGR1_EXTSEL_TRG7
#elif defined(CONFIG_STM32F0L0G0_STM32G0)
#  define ADC1_EXTSEL_T1TRGO2 ADC12_CFGR1_EXTSEL_TRG0
#  define ADC1_EXTSEL_T1CC4   ADC12_CFGR1_EXTSEL_TRG1
#  define ADC1_EXTSEL_T2TRGO  ADC12_CFGR1_EXTSEL_TRG2
#  define ADC1_EXTSEL_T3TRGO  ADC12_CFGR1_EXTSEL_TRG3
#  define ADC1_EXTSEL_T15TRGO ADC12_CFGR1_EXTSEL_TRG4
                              /* TRG5 and TRG6 reserved */
#  define ADC1_EXTSEL_EXTI11  ADC12_CFGR1_EXTSEL_TRG7
#elif defined(CONFIG_STM32F0L0G0_STM32C0)
#  define ADC1_EXTSEL_T1TRGO2 ADC12_CFGR1_EXTSEL_TRG0
#  define ADC1_EXTSEL_T1CC4   ADC12_CFGR1_EXTSEL_TRG1
#  define ADC1_EXTSEL_T2TRGO  ADC12_CFGR1_EXTSEL_TRG2
#  define ADC1_EXTSEL_T3TRGO  ADC12_CFGR1_EXTSEL_TRG3
#  define ADC1_EXTSEL_T15TRGO ADC12_CFGR1_EXTSEL_TRG4
                              /* TRG5 and TRG6 reserved */
#  define ADC1_EXTSEL_EXTI11  ADC12_CFGR1_EXTSEL_TRG7
#else
#  error
#endif

/* EXTSEL configuration *****************************************************/

/* NOTE:
 * this configuration if used only if CONFIG_STM32F0L0G0_TIMx_ADCy is
 * selected.
 * You can still connect the ADC with a timer trigger using the
 * CONFIG_STM32F0L0G0_ADCx_EXTSEL option.
 */

#if defined(CONFIG_STM32F0L0G0_TIM1_ADC1)
#  if CONFIG_STM32F0L0G0_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T1CC4
#  elif CONFIG_STM32F0L0G0_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T1TRGO
#  elif CONFIG_STM32F0L0G0_ADC1_TIMTRIG == 5
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T1TRGO2
#  else
#    error "CONFIG_STM32F0L0G0_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F0L0G0_TIM2_ADC1)
#  if CONFIG_STM32F0L0G0_ADC1_TIMTRIG == 3
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T2CC4
#  elif CONFIG_STM32F0L0G0_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T2TRGO
#  else
#    error "CONFIG_STM32F0L0G0_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F0L0G0_TIM3_ADC1)
#  if CONFIG_STM32F0L0G0_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T3TRGO
#  else
#    error "CONFIG_STM32F0L0G0_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F0L0G0_TIM15_ADC1)
#  if CONFIG_STM32F0L0G0_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T15TRGO
#  else
#    error "CONFIG_STM32F0L0G0_ADC1_TIMTRIG is out of range"
#  endif
#elif defined(CONFIG_STM32F0L0G0_TIM21_ADC1)
#  if CONFIG_STM32F0L0G0_ADC1_TIMTRIG == 1
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T21CC2
#  elif CONFIG_STM32F0L0G0_ADC1_TIMTRIG == 4
#    define ADC1_EXTSEL_VALUE ADC1_EXTSEL_T21TRGO
#  else
#    error "CONFIG_STM32F0L0G0_ADC1_TIMTRIG is out of range"
#  endif
#endif

/* Regular channels external trigger support */

#ifdef ADC1_EXTSEL_VALUE
#  define ADC1_HAVE_EXTCFG  1
#  define ADC1_EXTCFG_VALUE (ADC1_EXTSEL_VALUE | ADC_EXTREG_EXTEN_DEFAULT)
#elif defined(CONFIG_STM32F0L0G0_ADC1_EXTSEL)
#  define ADC1_HAVE_EXTCFG  1
#  define ADC1_EXTCFG_VALUE 0
#else
#  undef ADC1_HAVE_EXTCFG
#endif

#if defined(ADC1_HAVE_EXTCFG)
#  define ADC_HAVE_EXTCFG
#endif

/* ADC interrupts ***********************************************************/

#define ADC_ISR_EOC                  ADC_INT_EOC
#define ADC_IER_EOC                  ADC_INT_EOC
#define ADC_ISR_AWD                  ADC_INT_AWD
#define ADC_IER_AWD                  ADC_INT_AWD
#define ADC_ISR_OVR                  ADC_INT_OVR
#define ADC_IER_OVR                  ADC_INT_OVR

#define ADC_ISR_ALLINTS (ADC_ISR_EOC | ADC_ISR_AWD | ADC_ISR_OVR)
#define ADC_IER_ALLINTS (ADC_IER_EOC | ADC_IER_AWD | ADC_IER_OVR)

/* ADC registers ************************************************************/

#define STM32_ADC_DMAREG_OFFSET      STM32_ADC_CFGR1_OFFSET
#define ADC_DMAREG_DMA               ADC_CFGR1_DMAEN
#define STM32_ADC_EXTREG_OFFSET      STM32_ADC_CFGR1_OFFSET
#define ADC_EXTREG_EXTSEL_MASK       ADC_CFGR1_EXTSEL_MASK
#define ADC_EXTREG_EXTEN_MASK        ADC_CFGR1_EXTEN_MASK
#define ADC_EXTREG_EXTEN_DEFAULT     ADC_CFGR1_EXTEN_RISING

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
#define ADC_REGBUF_REGISTER(adc, buffer, len)        \
        (adc)->llops->regbuf_reg(adc, buffer, len)
#define ADC_REG_STARTCONV(adc, state)                \
        (adc)->llops->reg_startconv(adc, state)
#define ADC_SAMPLETIME_SET(adc, time_samples)        \
        (adc)->llops->stime_set(adc, time_samples)
#define ADC_SAMPLETIME_WRITE(adc)                    \
        (adc)->llops->stime_write(adc)
#define ADC_DUMP_REGS(adc)                           \
        (adc)->llops->dump_regs(adc)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* On STM32F42xx and STM32F43xx devices,VBAT and temperature sensor are
 * connected to the same ADC internal channel (ADC1_IN18). Only one
 * conversion, either temperature sensor or VBAT, must be selected at a time.
 * When both conversion are enabled simultaneously, only the VBAT conversion
 * is performed.
 */

enum adc_io_cmds_e
{
#ifdef HAVE_ADC_VBAT
  IO_ENABLE_DISABLE_VBAT_CH,
#endif
  IO_ENABLE_DISABLE_AWDIE,
  IO_ENABLE_DISABLE_EOCIE,
  IO_ENABLE_DISABLE_JEOCIE,
  IO_ENABLE_DISABLE_OVRIE,
  IO_ENABLE_DISABLE_ALL_INTS,
  IO_STOP_ADC,
  IO_START_ADC,
  IO_START_CONV,
  IO_TRIGGER_REG,
#ifdef ADC_HAVE_INJECTED
  IO_TRIGGER_INJ,
#endif
#ifdef HAVE_ADC_POWERDOWN
  IO_ENABLE_DISABLE_PDI,
  IO_ENABLE_DISABLE_PDD,
  IO_ENABLE_DISABLE_PDD_PDI
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

#ifdef CONFIG_STM32F0L0G0_ADC_LL_OPS

#ifdef CONFIG_STM32F0L0G0_ADC_CHANGE_SAMPLETIME

struct adc_sample_time_s
{
  uint8_t smp1;     /* Sample time for channels with SMPSEL bit = 0 */
  uint8_t smp2;     /* Sample time for channels with SMPSEL bit = 1 */
  uint32_t smpsel;  /* Bitmask for selecting which channels use SMP2 */
};
#endif /* CONFIG_STM32F0L0G0_ADC_CHANGE_SAMPLETIME */

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

#ifdef CONFIG_STM32F0L0G0_ADC_CHANGE_SAMPLETIME
  /* Set ADC sample time */

  void (*stime_set)(struct stm32_adc_dev_s *dev,
                    struct adc_sample_time_s *time_samples);

  /* Write ADC sample time */

  void (*stime_write)(struct stm32_adc_dev_s *dev);
#endif

  void (*dump_regs)(struct stm32_adc_dev_s *dev);
};

#endif /* CONFIG_STM32F0L0G0_ADC_LL_OPS */

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
 * Name: stm32_adcinitialize
 *
 * Description:
 *   Initialize the ADC. See stm32_adc.c for more details.
 *
 * Input Parameters:
 *   intf      - Could be {1,2,3,4} for ADC1, ADC2, ADC3 or ADC4
 *   chanlist  - The list of channels (regular + injected)
 *   nchannels - Number of channels (regular + injected)
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s;
struct adc_dev_s *stm32_adcinitialize(int intf, const uint8_t *chanlist,
                                      int channels);

/****************************************************************************
 * Name: stm32_adc_llops_get
 ****************************************************************************/

#ifdef CONFIG_STM32F0L0G0_ADC_LL_OPS
const struct stm32_adc_ops_s
*stm32_adc_llops_get(struct adc_dev_s *dev);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32F0L0G0_ADC1 */
#endif /* __ARCH_ARM_SRC_STM32F0L0G0_STM32_ADC_H */
