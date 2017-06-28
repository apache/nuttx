/************************************************************************************
 * arch/arm/src/stm32/stm32_sdadc.h
 *
 *   Copyright (C) 2009, 2011, 2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Studelec. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Marc Rechté <mrechte@studelec-sa.com>
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_SDADC_H
#define __ARCH_ARM_SRC_STM32_STM32_SDADC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_STM32_STM32F37XX)
#  include "chip/stm32f37xxx_sdadc.h"
#else
/* No generic chip/stm32_sdadc.h yet */

#  error "This chip is not yet supported"
#endif

#include <nuttx/analog/adc.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Timer devices may be used for different purposes.  One special purpose is to
 * control periodic SDADC sampling.  If CONFIG_STM32_TIMn is defined then
 * CONFIG_STM32_TIMn_SDADC must also be defined to indicate that timer "n" is intended
 * to be used for that purpose.
 */

/* For the STM32 F37XX line, timers 2-4, 12-17 an 19 may be used. */

/* TODO cf. stm32_adc.h */

/* Up to 3 SDADC interfaces are supported */

#if STM32_NSDADC < 3
#  undef CONFIG_STM32_SDADC3
#endif

#if STM32_NSDADC < 2
#  undef CONFIG_STM32_SDADC2
#endif

#if STM32_NSDADC < 1
#  undef CONFIG_STM32_SDADC1
#endif

#if defined(CONFIG_STM32_SDADC1) || defined(CONFIG_STM32_SDADC2) || \
    defined(CONFIG_STM32_SDADC3)

/* DMA support */

#if defined(CONFIG_STM32_SDADC1_DMA) || defined(CONFIG_STM32_SDADC2_DMA) || \
    defined(CONFIG_STM32_SDADC3_DMA)
#  define SDADC_HAVE_DMA  1
#endif

#ifdef CONFIG_STM32_SDADC1_DMA
#  define SDADC1_HAVE_DMA 1
#else
#  undef  SDADC1_HAVE_DMA
#endif

#ifdef CONFIG_STM32_SDADC2_DMA
#  define SDADC2_HAVE_DMA 1
#else
#  undef  SDADC2_HAVE_DMA
#endif

#ifdef CONFIG_STM32_SDADC3_DMA
#  define SDADC3_HAVE_DMA 1
#else
#  undef  SDADC3_HAVE_DMA
#endif

/* SDADC Channels/DMA ******************************************************
 * The maximum number of channels that can be sampled at each scan.
 * If DMA support is not enabled, then only a single channel
 * ought to be sampled.
 * Otherwise, unless sampling frequency is reduced,
 * data overruns would occur.
 */

#define SDADC_MAX_CHANNELS_DMA   9
#define SDADC_MAX_CHANNELS_NODMA 1

#ifndef SDADC_MAX_SAMPLES
#ifdef SDADC_HAVE_DMA
#  define SDADC_MAX_SAMPLES SDADC_MAX_CHANNELS_DMA
#else
#  define SDADC_MAX_SAMPLES SDADC_MAX_CHANNELS_NODMA
#endif
#endif

/* Timer configuration:  If a timer trigger is specified, then get
 * information about the timer.
 */

#if defined(CONFIG_STM32_TIM3_SDADC1)
#  define SDADC1_HAVE_TIMER           1
#  define SDADC1_TIMER_BASE           STM32_TIM3_BASE
#  define SDADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_STM32_TIM4_SDADC1)
#  define SDADC1_HAVE_TIMER           1
#  define SDADC1_TIMER_BASE           STM32_TIM4_BASE
#  define SDADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_STM32_TIM13_SDADC1)
#  define SDADC1_HAVE_TIMER           1
#  define SDADC1_TIMER_BASE           STM32_TIM13_BASE
#  define SDADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM13_CLKIN
#elif defined(CONFIG_STM32_TIM14_SDADC1)
#  define SDADC1_HAVE_TIMER           1
#  define SDADC1_TIMER_BASE           STM32_TIM14_BASE
#  define SDADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM14_CLKIN
#elif defined(CONFIG_STM32_TIM15_SDADC1)
#  define SDADC1_HAVE_TIMER           1
#  define SDADC1_TIMER_BASE           STM32_TIM15_BASE
#  define SDADC1_TIMER_PCLK_FREQUENCY STM32_APB2_TIM15_CLKIN
#elif defined(CONFIG_STM32_TIM19_SDADC1)
#  define SDADC1_HAVE_TIMER           1
#  define SDADC1_TIMER_BASE           STM32_TIM19_BASE
#  define SDADC1_TIMER_PCLK_FREQUENCY STM32_APB1_TIM19_CLKIN
#else
#  undef  SDADC1_HAVE_TIMER
#endif

#ifdef SDADC1_HAVE_TIMER
#  ifndef CONFIG_STM32_SDADC1_SAMPLE_FREQUENCY
#    error "CONFIG_STM32_SDADC1_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32_SDADC1_TIMTRIG
#    error "CONFIG_STM32_SDADC1_TIMTRIG not defined"
#    warning "Values 0:TIM13_CH1 1:TIM14_CH1 2:TIM15_CH2 3:TIM3_CH1 4:TIM4_CH1 5:TIM19_CH2"
#  endif
#endif

#if defined(CONFIG_STM32_TIM2_SDADC2)
#  define SDADC2_HAVE_TIMER           1
#  define SDADC2_TIMER_BASE           STM32_TIM2_BASE
#  define SDADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_STM32_TIM3_SDADC2)
#  define SDADC2_HAVE_TIMER           1
#  define SDADC2_TIMER_BASE           STM32_TIM3_BASE
#  define SDADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_STM32_TIM4_SDADC2)
#  define SDADC2_HAVE_TIMER           1
#  define SDADC2_TIMER_BASE           STM32_TIM4_BASE
#  define SDADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_STM32_TIM12_SDADC2)
#  define SDADC2_HAVE_TIMER           1
#  define SDADC2_TIMER_BASE           STM32_TIM12_BASE
#  define SDADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM12_CLKIN
#elif defined(CONFIG_STM32_TIM17_SDADC2)
#  define SDADC2_HAVE_TIMER           1
#  define SDADC2_TIMER_BASE           STM32_TIM17_BASE
#  define SDADC2_TIMER_PCLK_FREQUENCY STM32_APB2_TIM17_CLKIN
#elif defined(CONFIG_STM32_TIM19_SDADC2)
#  define SDADC2_HAVE_TIMER           1
#  define SDADC2_TIMER_BASE           STM32_TIM19_BASE
#  define SDADC2_TIMER_PCLK_FREQUENCY STM32_APB1_TIM19_CLKIN
#else
#  undef  SDADC2_HAVE_TIMER
#endif

#ifdef SDADC2_HAVE_TIMER
#  ifndef CONFIG_STM32_SDADC2_SAMPLE_FREQUENCY
#    error "CONFIG_STM32_SDADC2_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32_SDADC2_TIMTRIG
#    error "CONFIG_STM32_SDADC2_TIMTRIG not defined"
#    warning "Values 0:TIM17_CH1 1:TIM12_CH1 2:TIM2_CH3 3:TIM3_CH2 4:TIM4_CH2 5:TIM19_CH3"
#  endif
#endif

#if defined(CONFIG_STM32_TIM2_SDADC3)
#  define SDADC3_HAVE_TIMER           1
#  define SDADC3_TIMER_BASE           STM32_TIM2_BASE
#  define SDADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM2_CLKIN
#elif defined(CONFIG_STM32_TIM3_SDADC3)
#  define SDADC3_HAVE_TIMER           1
#  define SDADC3_TIMER_BASE           STM32_TIM3_BASE
#  define SDADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM3_CLKIN
#elif defined(CONFIG_STM32_TIM4_SDADC3)
#  define SDADC3_HAVE_TIMER           1
#  define SDADC3_TIMER_BASE           STM32_TIM4_BASE
#  define SDADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM4_CLKIN
#elif defined(CONFIG_STM32_TIM12_SDADC3)
#  define SDADC3_HAVE_TIMER           1
#  define SDADC3_TIMER_BASE           STM32_TIM12_BASE
#  define SDADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM12_CLKIN
#elif defined(CONFIG_STM32_TIM16_SDADC3)
#  define SDADC3_HAVE_TIMER           1
#  define SDADC3_TIMER_BASE           STM32_TIM16_BASE
#  define SDADC3_TIMER_PCLK_FREQUENCY STM32_APB2_TIM16_CLKIN
#elif defined(CONFIG_STM32_TIM19_SDADC3)
#  define SDADC3_HAVE_TIMER           1
#  define SDADC3_TIMER_BASE           STM32_TIM19_BASE
#  define SDADC3_TIMER_PCLK_FREQUENCY STM32_APB1_TIM19_CLKIN
#else
#  undef  SDADC3_HAVE_TIMER
#endif

#ifdef SDADC3_HAVE_TIMER
#  ifndef CONFIG_STM32_SDADC3_SAMPLE_FREQUENCY
#    error "CONFIG_STM32_SDADC3_SAMPLE_FREQUENCY not defined"
#  endif
#  ifndef CONFIG_STM32_SDADC3_TIMTRIG
#    error "CONFIG_STM32_SDADC3_TIMTRIG not defined"
#    warning "Values 0:TIM16_CH1 1:TIM12_CH2 2:TIM2_CH4 3:TIM3_CH3 4:TIM4_CH3 5:TIM19_CH4"
#  endif
#endif

#if defined(SDADC1_HAVE_TIMER) || defined(SDADC2_HAVE_TIMER) || \
    defined(SDADC3_HAVE_TIMER)
#  define SDADC_HAVE_TIMER 1
#  if defined(CONFIG_STM32_STM32F37XX) && !defined(CONFIG_STM32_FORCEPOWER)
#    warning "CONFIG_STM32_FORCEPOWER must be defined to enable the timer(s)"
#  endif
#else
#  undef SDADC_HAVE_TIMER
#endif

/* NOTE:  The following assumes that all possible combinations of timers and
 * values are support JEXTSEL.  That is not so and it varies from one STM32
 * to another.  But this (wrong) assumptions keeps the logic as simple as
 * possible.  If unsupported combination is used, an error will show up
 * later during compilation although it may be difficult to track it back
 * to this simplification.
 *
 * STM32L37XX-family has 3 SDADC onboard
 */

#ifdef CONFIG_STM32_STM32F37XX
#  define SDADC1_JEXTSEL_TIM13_CH1  SDADC1_CR2_JEXTSEL_TIM13_CH1
#  define SDADC1_JEXTSEL_TIM14_CH1  SDADC1_CR2_JEXTSEL_TIM14_CH1
#  define SDADC1_JEXTSEL_TIM15_CH2  SDADC1_CR2_JEXTSEL_TIM15_CH2
#  define SDADC1_JEXTSEL_TIM3_CH1   SDADC1_CR2_JEXTSEL_TIM3_CH1
#  define SDADC1_JEXTSEL_TIM4_CH1   SDADC1_CR2_JEXTSEL_TIM4_CH1
#  define SDADC1_JEXTSEL_TIM19_CH2  SDADC1_CR2_JEXTSEL_TIM19_CH2
#  define SDADC1_JEXTSEL_EXTI15     SDADC1_CR2_JEXTSEL_EXTI15
#  define SDADC1_JEXTSEL_EXTI11     SDADC1_CR2_JEXTSEL_EXTI11
#  define SDADC2_JEXTSEL_TIM17_CH1  SDADC2_CR2_JEXTSEL_TIM17_CH1
#  define SDADC2_JEXTSEL_TIM12_CH1  SDADC2_CR2_JEXTSEL_TIM12_CH1
#  define SDADC2_JEXTSEL_TIM2_CH3   SDADC2_CR2_JEXTSEL_TIM2_CH3
#  define SDADC2_JEXTSEL_TIM3_CH2   SDADC2_CR2_JEXTSEL_TIM3_CH2
#  define SDADC2_JEXTSEL_TIM4_CH2   SDADC2_CR2_JEXTSEL_TIM4_CH2
#  define SDADC2_JEXTSEL_TIM19_CH3  SDADC2_CR2_JEXTSEL_TIM19_CH3
#  define SDADC2_JEXTSEL_EXTI15     SDADC2_CR2_JEXTSEL_EXTI15
#  define SDADC2_JEXTSEL_EXTI11     SDADC2_CR2_JEXTSEL_EXTI11
#  define SDADC3_JEXTSEL_TIM16_CH1  SDADC3_CR2_JEXTSEL_TIM16_CH1
#  define SDADC3_JEXTSEL_TIM12_CH1  SDADC3_CR2_JEXTSEL_TIM12_CH1
#  define SDADC3_JEXTSEL_TIM2_CH4   SDADC3_CR2_JEXTSEL_TIM2_CH4
#  define SDADC3_JEXTSEL_TIM3_CH3   SDADC3_CR2_JEXTSEL_TIM3_CH3
#  define SDADC3_JEXTSEL_TIM4_CH3   SDADC3_CR2_JEXTSEL_TIM4_CH3
#  define SDADC3_JEXTSEL_TIM19_CH4  SDADC3_CR2_JEXTSEL_TIM19_CH4
#  define SDADC3_JEXTSEL_EXTI15     SDADC3_CR2_JEXTSEL_EXTI15
#  define SDADC3_JEXTSEL_EXTI11     SDADC3_CR2_JEXTSEL_EXTI11
#endif

#if defined(CONFIG_STM32_TIM3_SDADC1)
#  define SDADC1_JEXTSEL_VALUE 3
#elif defined(CONFIG_STM32_TIM4_SDADC1)
#  define SDADC1_JEXTSEL_VALUE 4
#elif defined(CONFIG_STM32_TIM13_SDADC1)
#  define SDADC1_JEXTSEL_VALUE 0
#elif defined(CONFIG_STM32_TIM14_SDADC1)
#  define SDADC1_JEXTSEL_VALUE 1
#elif defined(CONFIG_STM32_TIM15_SDADC1)
#  define SDADC1_JEXTSEL_VALUE 2
#elif defined(CONFIG_STM32_TIM19_SDADC1)
#  define SDADC1_JEXTSEL_VALUE 5
#else
#  undef SDADC1_JEXTSEL_VALUE
#endif

#if defined(CONFIG_STM32_TIM2_SDADC2)
#  define SDADC2_JEXTSEL_VALUE 2
#elif defined(CONFIG_STM32_TIM3_SDADC2)
#  define SDADC2_JEXTSEL_VALUE 3
#elif defined(CONFIG_STM32_TIM4_SDADC2)
#  define SDADC2_JEXTSEL_VALUE 4
#elif defined(CONFIG_STM32_TIM12_SDADC2)
#  define SDADC2_JEXTSEL_VALUE 1
#elif defined(CONFIG_STM32_TIM17_SDADC2)
#  define SDADC2_JEXTSEL_VALUE 0
#elif defined(CONFIG_STM32_TIM19_SDADC2)
#  define SDADC2_JEXTSEL_VALUE 5
#else
#  undef SDADC2_JEXTSEL_VALUE
#endif

#if defined(CONFIG_STM32_TIM2_SDADC3)
#  define SDADC3_JEXTSEL_VALUE 2
#elif defined(CONFIG_STM32_TIM3_SDADC3)
#  define SDADC3_JEXTSEL_VALUE 3
#elif defined(CONFIG_STM32_TIM4_SDADC3)
#  define SDADC3_JEXTSEL_VALUE 4
#elif defined(CONFIG_STM32_TIM12_SDADC3)
#  define SDADC3_JEXTSEL_VALUE 1
#elif defined(CONFIG_STM32_TIM16_SDADC3)
#  define SDADC3_JEXTSEL_VALUE 0
#elif defined(CONFIG_STM32_TIM19_SDADC3)
#  define SDADC3_JEXTSEL_VALUE 5
#else
#  undef SDADC3_JEXTSEL_VALUE
#endif

/* SDADC Configurations ********************************************************
 * Up to 3 configuration profiles may be defined in order to define:
 * - calibration method
 * - SE/differential mode
 * - input gain
 * Each of the 9 SDADC channels is assigned to a configuration profile
 */
#ifndef SDADC_CONF0R_DEFAULT
#  define SDADC_CONF0R_DEFAULT (SDADC_CONFR_GAIN_1X | SDADC_CONFR_SE_SE_OFFSET | SDADC_CONFR_COMMON_GND)
#endif
#ifndef SDADC_CONF1R_DEFAULT
#  define SDADC_CONF1R_DEFAULT (SDADC_CONFR_GAIN_2X | SDADC_CONFR_SE_SE_OFFSET | SDADC_CONFR_COMMON_GND)
#endif
#ifndef SDADC_CONF2R_DEFAULT
#  define SDADC_CONF2R_DEFAULT (SDADC_CONFR_GAIN_4X | SDADC_CONFR_SE_SE_OFFSET | SDADC_CONFR_COMMON_GND)
#endif
#ifndef SDADC_CONFCHR1_DEFAULT
#  define SDADC_CONFCHR1_DEFAULT ((SDADC_CONF0R << SDADC_CONFCHR1_CH_SHIFT(0)) | \
                                  (SDADC_CONF0R << SDADC_CONFCHR1_CH_SHIFT(1)) | \
                                  (SDADC_CONF0R << SDADC_CONFCHR1_CH_SHIFT(2)) | \
                                  (SDADC_CONF0R << SDADC_CONFCHR1_CH_SHIFT(3)) | \
                                  (SDADC_CONF0R << SDADC_CONFCHR1_CH_SHIFT(4)) | \
                                  (SDADC_CONF0R << SDADC_CONFCHR1_CH_SHIFT(5)) | \
                                  (SDADC_CONF0R << SDADC_CONFCHR1_CH_SHIFT(6)) | \
                                  (SDADC_CONF0R << SDADC_CONFCHR1_CH_SHIFT(7)))
#endif
#ifndef SDADC_CONFCHR2_DEFAULT
#  define SDADC_CONFCHR2_DEFAULT (SDADC_CONF0R << SDADC_CONFCHR2_CH8_SHIFT)
#endif

/* SDADC Reference voltage selection ************************************************/

#ifndef SDADC_REFV_DEFAULT
#  define SDADC_REFV_DEFAULT  SDADC_CR1_REFV_EXT
#endif
#ifndef SDADC1_REFV
#  define SDADC1_REFV  SDADC_REFV_DEFAULT
#endif
#ifndef SDADC2_REFV
#  define SDADC2_REFV  SDADC_REFV_DEFAULT
#endif
#ifndef SDADC3_REFV
#  define SDADC3_REFV  SDADC_REFV_DEFAULT
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

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
 * Name: stm32_sdadcinitialize
 *
 ****************************************************************************/

struct adc_dev_s *stm32_sdadcinitialize(int intf, FAR const uint8_t *chanlist,
                                        int nchannels);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32_SDADC1 || CONFIG_STM32_SDADC2 ||
        * CONFIG_STM32_SDADC3
        */
#endif /* __ARCH_ARM_SRC_STM32_STM32_SDADC_H */
