/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_adc.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32l4_rcc.h"
#include "stm32l4_tim.h"
#include "stm32l4_dma.h"
#include "stm32l4_adc.h"

/* ADC "upper half" support must be enabled */

#ifdef CONFIG_ADC

/* Some ADC peripheral must be enabled */

#if defined(CONFIG_STM32L4_ADC1) || defined(CONFIG_STM32L4_ADC2) || \
    defined(CONFIG_STM32L4_ADC3)

/* This implementation is for the STM32 L4X3 and L4X6 only */

#if defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L4X3)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_initialize
 *
 * Description:
 *   Initialize the ADC.
 *
 *   The logic is, save nchannels : # of channels (conversions) in ADC_SQR1_L
 *   Then, take the chanlist array and store it in the SQR Regs,
 *     chanlist[0] -> ADC_SQR3_SQ1
 *     chanlist[1] -> ADC_SQR3_SQ2
 *     ...
 *     chanlist[15]-> ADC_SQR1_SQ16
 *
 *   up to
 *     chanlist[nchannels]
 *
 * Input Parameters:
 *   intf      - Could be {1,2,3} for ADC1, ADC2, or ADC3
 *   chanlist  - The list of channels
 *   cchannels - Number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *stm32l4_adc_initialize(int intf, FAR const uint8_t *chanlist,
                                         int cchannels)
{
  FAR struct adc_dev_s *dev = NULL;

  ainfo("intf: %d cchannels: %d\n", intf, cchannels);

  switch (intf)
    {
#ifdef CONFIG_STM32L4_ADC1
      case 1:
        ainfo("ADC1 selected\n");
        //dev = &g_adcdev1;
        break;
#endif
#ifdef CONFIG_STM32L4_ADC2
      case 2:
        ainfo("ADC2 selected\n");
        //dev = &g_adcdev2;
        break;
#endif
#ifdef CONFIG_STM32L4_ADC3
      case 3:
        ainfo("ADC3 selected\n");
        //dev = &g_adcdev3;
        break;
#endif
      default:
        aerr("ERROR: No ADC interface defined\n");
        return NULL;
    }

#warning "Missing ADC implementation!"

  return dev;
}

#endif /* CONFIG_STM32L4_STM32L4X3 || CONFIG_STM32L4_STM32L4X6 */
#endif /* CONFIG_STM32L4_ADC1 || CONFIG_STM32L4_ADC2 || CONFIG_STM32L4_ADC3
        */
#endif /* CONFIG_ADC */
