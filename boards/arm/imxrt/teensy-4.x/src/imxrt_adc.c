/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/imxrt_adc.c
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

#include "chip.h"
#include "arm_internal.h"
#include "imxrt_adc.h"
#include "teensy-4.h"

#ifdef CONFIG_IMXRT_ADC

/* channels 1 and 2 have the same number of pins on Teensy */

#define ADC_NCHANNELS 16

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_chanlist[ADC_NCHANNELS] =
  {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15
  };

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_adc_setup
 *
 * Description:
 *   Initialize and register the ADC driver.
 *
 ****************************************************************************/

int imxrt_adc_initialize(void)
{
  struct adc_dev_s *adc;
  int ret;

  /* Call imxrt_adcinitialize() to get an instance of the ADC interface */

  #ifdef CONFIG_IMXRT_ADC1
      adc = imxrt_adcinitialize(1, g_chanlist, ADC_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC1 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc1" */

      ret = adc_register("/dev/adc1", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register adc1 failed: %d\n", ret);
          return ret;
        }

  #endif
  #ifdef CONFIG_IMXRT_ADC2
      adc = imxrt_adcinitialize(2, g_chanlist, ADC_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC2 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc2" */

      ret = adc_register("/dev/adc2", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register adc2 failed: %d\n", ret);
          return ret;
        }
  #endif

  return OK;
}

#endif /* CONFIG_IMXRT_ADC */
