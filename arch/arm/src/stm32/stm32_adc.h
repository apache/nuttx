/************************************************************************************
 * arch/arm/src/stm32/stm32_adc.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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
#include "chip/stm32_adc.h"

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
 * 8 may be used.
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

#if defined(CONFIG_STM32_STM32F40XX)
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

/* Timers 6, 7, and 10-14 are not used with the ADC by any supported family */

#undef CONFIG_STM32_TIM6_ADC
#undef CONFIG_STM32_TIM6_ADC1
#undef CONFIG_STM32_TIM6_ADC2
#undef CONFIG_STM32_TIM6_ADC3
#undef CONFIG_STM32_TIM7_ADC
#undef CONFIG_STM32_TIM7_ADC1
#undef CONFIG_STM32_TIM7_ADC2
#undef CONFIG_STM32_TIM7_ADC3
#undef CONFIG_STM32_TIM9_ADC
#undef CONFIG_STM32_TIM9_ADC1
#undef CONFIG_STM32_TIM9_ADC2
#undef CONFIG_STM32_TIM9_ADC3
#undef CONFIG_STM32_TIM10_ADC
#undef CONFIG_STM32_TIM10_ADC1
#undef CONFIG_STM32_TIM10_ADC2
#undef CONFIG_STM32_TIM10_ADC3
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

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
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
EXTERN struct adc_dev_s *stm32_adcinitialize(int intf, const uint8_t *chanlist,
                                             int nchannels);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32_STM32_ADC_H */

