/*****************************************************************************
 * configs/nucleo-l496zg/src/stm32_dfsdm.c
 *
 *   Copyright (C) 2017 Haltian Ltd. All rights reserved.
 *   Authors: Juha Niskanen <juha.niskanen@haltian.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>
#include <arch/board/board.h>
#include "stm32l4_gpio.h"
#include "stm32l4_dfsdm.h"
#include "nucleo-144.h"

#if defined(CONFIG_ADC) && defined(CONFIG_STM32L4_DFSDM)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_dfsdm_setup
 ************************************************************************************/

int stm32_dfsdm_setup(void)
{
  static bool initialized = false;

  if (!initialized)
    {
      int ret;
      struct adc_dev_s *adc;

      ainfo("Initializing DFSDM\n");

      /* TODO: just some arbitrary channels selected, missing input pin
       * configuration and DFSDM mode selection: SPI/Manchester or internal
       * parallel inputs (CPU/DMA/ADC).
       */

#ifdef CONFIG_STM32L4_DFSDM1_FLT0
      adc = stm32l4_dfsdm_initialize(0, (const uint8_t[1]){0}, 1);
      if (adc == NULL)
        {
          aerr("Failed to get DFSDM FLT0 interface\n");
          return -ENODEV;
        }

      ret = adc_register("/dev/adc_flt0", adc);
      if (ret < 0)
        {
          aerr("adc_register failed: %d\n", ret);
          return ret;
        }
#endif
#ifdef CONFIG_STM32L4_DFSDM1_FLT1
      adc = stm32l4_dfsdm_initialize(1, (const uint8_t[2]){0,1}, 2);
      if (adc == NULL)
        {
          aerr("Failed to get DFSDM FLT1 interface\n");
          return -ENODEV;
        }

      ret = adc_register("/dev/adc_flt1", adc);
      if (ret < 0)
        {
          aerr("adc_register failed: %d\n", ret);
          return ret;
        }
#endif
#ifdef CONFIG_STM32L4_DFSDM1_FLT2
      adc = stm32l4_dfsdm_initialize(2, (const uint8_t[8]){0,1,2,3,4,5,6,7}, 8);
      if (adc == NULL)
        {
          aerr("Failed to get DFSDM FLT2 interface\n");
          return -ENODEV;
        }

      ret = adc_register("/dev/adc_flt2", adc);
      if (ret < 0)
        {
          aerr("adc_register failed: %d\n", ret);
          return ret;
        }
#endif
#ifdef CONFIG_STM32L4_DFSDM1_FLT3
      adc = stm32l4_dfsdm_initialize(3, (const uint8_t[4]){6,5,4,3}, 4);
      if (adc == NULL)
        {
          aerr("Failed to get DFSDM FLT3 interface\n");
          return -ENODEV;
        }

      ret = adc_register("/dev/adc_flt3", adc);
      if (ret < 0)
        {
          aerr("adc_register failed: %d\n", ret);
          return ret;
        }
#endif

      initialized = true;
    }

  return OK;
}
#endif /* CONFIG_ADC && CONFIG_STM32L4_DFSDM */
