/****************************************************************************
 * boards/arm/imxrt/imxrt1060-evk/src/imxrt_adc.c
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

#ifdef CONFIG_IMXRT_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define ADC1_NCHANNELS 3
#define ADC2_NCHANNELS 3

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_chanlist1[ADC1_NCHANNELS] =
  {
    /* Arduino Interface pins A0 to A2 */

    15,
    0,
    9,
  };

static const uint8_t g_chanlist2[ADC2_NCHANNELS] =
  {
    /* Arduino Interface pins A3 to A5 */

    10,
    6,
    5
  };

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int imxrt_adc_initialize(void)
{
  static bool initialized = false;
  struct adc_dev_s *adc;
  int ret;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Call imxrt_adcinitialize() to get an instance of the ADC interface */

#ifdef CONFIG_IMXRT_ADC1
      adc = imxrt_adcinitialize(1, g_chanlist1, ADC1_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface for ADC1\n");
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
      adc = imxrt_adcinitialize(2, g_chanlist2, ADC2_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface for ADC2\n");
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

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_ADC */
