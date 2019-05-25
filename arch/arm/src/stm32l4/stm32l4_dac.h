/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_dac.h
 *
 *   Copyright (C) 2011, 2015, 2017 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_DAC_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_DAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32l4_dac.h"

#include <nuttx/analog/dac.h>

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/

/* Configuration ********************************************************************/
/* Timer devices may be used for different purposes.  One special purpose is to
 * control periodic DAC outputs.  If CONFIG_STM32L4_TIMn is defined then
 * CONFIG_STM32L4_TIMn_DAC must also be defined to indicate that timer "n" is
 * intended to be used for that purpose.
 */

#ifndef CONFIG_STM32L4_TIM1
#  undef CONFIG_STM32L4_TIM1_DAC
#endif
#ifndef CONFIG_STM32L4_TIM2
#  undef CONFIG_STM32L4_TIM2_DAC
#endif
#ifndef CONFIG_STM32L4_TIM3
#  undef CONFIG_STM32L4_TIM3_DAC
#endif
#ifndef CONFIG_STM32L4_TIM4
#  undef CONFIG_STM32L4_TIM4_DAC
#endif
#ifndef CONFIG_STM32L4_TIM5
#  undef CONFIG_STM32L4_TIM5_DAC
#endif
#ifndef CONFIG_STM32L4_TIM6
#  undef CONFIG_STM32L4_TIM6_DAC
#endif
#ifndef CONFIG_STM32L4_TIM7
#  undef CONFIG_STM32L4_TIM7_DAC
#endif
#ifndef CONFIG_STM32L4_TIM8
#  undef CONFIG_STM32L4_TIM8_DAC
#endif
#ifndef CONFIG_STM32L4_TIM15
#  undef CONFIG_STM32L4_TIM15_DAC
#endif
#ifndef CONFIG_STM32L4_TIM16
#  undef CONFIG_STM32L4_TIM16_DAC
#endif
#ifndef CONFIG_STM32L4_TIM17
#  undef CONFIG_STM32L4_TIM17_DAC
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifdef CONFIG_STM32L4_DAC1_DMA
extern uint16_t stm32l4_dac1_dmabuffer[];
#endif
#ifdef CONFIG_STM32L4_DAC2_DMA
extern uint16_t stm32l4_dac2_dmabuffer[];
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
 * Name: stm32l4_dacinitialize
 *
 * Description:
 *   Initialize the DAC
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid dac device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct dac_dev_s;
FAR struct dac_dev_s *stm32l4_dacinitialize(int intf);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_DAC_H */
