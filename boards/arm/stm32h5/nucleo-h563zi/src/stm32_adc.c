/****************************************************************************
 * boards/arm/stm32h5/nucleo-h563zi/src/stm32_adc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/analog/adc.h>

#include "stm32.h"

#if defined(CONFIG_ADC) && defined(CONFIG_STM32H5_ADC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The number of ADC channels in the conversion list */

#define ADC1_NCHANNELS 2

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Identifying number of each ADC channel (even if NCHANNELS is less ) */

static const uint8_t g_chanlist1[2] =
{
  3,
  10
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist1[2]  =
{
  GPIO_ADC1_IN3,
  GPIO_ADC1_IN10,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
  static bool initialized = false;
  struct adc_dev_s *adc;
  int ret;
  int i;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC1_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist1[i]);
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc = stm32h5_adc_initialize(1, g_chanlist1, ADC1_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface 1\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc0", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register /dev/adc0 failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return OK;
}

#endif