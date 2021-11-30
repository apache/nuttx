/****************************************************************************
 * boards/arm/stm32l4/nucleo-l496zg/src/stm32_dfsdm.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dfsdm_setup
 ****************************************************************************/

int stm32_dfsdm_setup(void)
{
  static bool initialized = false;

  if (!initialized)
    {
      int ret;
      struct adc_dev_s *adc;
#ifdef CONFIG_STM32L4_DFSDM1_FLT0
      const uint8_t chanlist0[1] =
      {
        0
      };
#endif

#ifdef CONFIG_STM32L4_DFSDM1_FLT1
      const uint8_t chanlist1[2] =
      {
        0, 1
      };
#endif

#ifdef CONFIG_STM32L4_DFSDM1_FLT2
      const uint8_t chanlist2[8] =
      {
        0, 1, 2, 3, 4, 5, 6, 7
      };
#endif

#ifdef CONFIG_STM32L4_DFSDM1_FLT3
      const uint8_t chanlist3[4] =
      {
        6, 5, 4, 3
      };
#endif

      ainfo("Initializing DFSDM\n");

      /* TODO: just some arbitrary channels selected, missing input pin
       * configuration and DFSDM mode selection: SPI/Manchester or internal
       * parallel inputs (CPU/DMA/ADC).
       */

#ifdef CONFIG_STM32L4_DFSDM1_FLT0
      adc = stm32l4_dfsdm_initialize(0, chanlist0, 1);
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
      adc = stm32l4_dfsdm_initialize(1, chanlist1, 2);
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
      adc = stm32l4_dfsdm_initialize(2, chanlist2, 8);
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
      adc = stm32l4_dfsdm_initialize(3, chanlist3, 4);
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
