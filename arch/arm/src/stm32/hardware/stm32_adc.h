/****************************************************************************************************
 * arch/arm/src/stm32/hardware/stm32_adc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* There are 2 main types of ADC IP cores among STM32 chips:
 *   1. STM32 ADC IPv1:
 *     a) basic version for F1 and F37x
 *     b) extended version for F2, F4, F7, L1:
 *   2. STM32 ADC IPv2:
 *     a) basic version for F0 and L0
 *     b) extended version for F3 (without F37x), G4, H7, L4, L4+
 *
 *   We also distinguish the modified STM32 ADC IPv1 core for the L1 family,
 *   which differs too much to keep it in the same file as ADC IPv1.
 */

#if defined(CONFIG_STM32_HAVE_IP_ADC_V1) && defined(CONFIG_STM32_HAVE_IP_ADC_V2)
#  error Only one STM32 ADC IP version must be selected
#endif

#if defined(CONFIG_STM32_HAVE_IP_ADC_V1)
#  if defined(CONFIG_STM32_STM32L15XX)
#    include "stm32_adc_v1l1.h"   /* Special case for L1 */
#  else
#    include "stm32_adc_v1.h"
#  endif
#elif defined(CONFIG_STM32_HAVE_IP_ADC_V2)
#  include "stm32_adc_v2.h"
#else
#  error "STM32 ADC IP version not specified"
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_ADC_H */
