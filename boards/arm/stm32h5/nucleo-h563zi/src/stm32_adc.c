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

#if defined(CONFIG_ADC)
#if defined(CONFIG_STM32H5_ADC1) || defined(CONFIG_STM32H5_ADC2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The number of ADC channels in the conversion list */

#define ADC1_NCHANNELS 2
#define ADC2_NCHANNELS 0

/* Dependent on NCHANNELS and Differential/Single-Ended Mode
 * ADCx_NCHANNELS <= ADCx_NPINS <= 2 * ADCx_NCHANNELS
 */

#define ADC1_NPINS     2
#define ADC2_NPINS     0

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Identifying number of each ADC channel (even if NCHANNELS is less ) */

#ifdef CONFIG_STM32H5_ADC1

static const uint8_t g_chanlist1[ADC1_NCHANNELS] =
{
  3,
  10
};

static uint32_t g_pinlist1[ADC1_NPINS] =
{
  GPIO_ADC1_INP3, /* Channel  3 - Positive */
  GPIO_ADC1_INP10 /* Channel 10 - Positive */
};
#endif

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
#ifdef CONFIG_STM32H5_ADC1
  struct adc_dev_s *adc1;
#endif
#ifdef CONFIG_STM32H5_ADC2
  struct adc_dev_s *adc2;
#endif
  int ret;
  int i;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Configure the pins as analog inputs for the selected channels */

#ifdef CONFIG_STM32H5_ADC1
      for (i = 0; i < ADC1_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist1[i]);
        }

      adc1 = stm32h5_adc_initialize(1, g_chanlist1, ADC1_NCHANNELS);
      if (adc1 == NULL)
        {
          aerr("ERROR: Failed to get ADC interface 1\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc0", adc1);
      if (ret < 0)
        {
          aerr("ERROR: adc_register /dev/adc0 failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_STM32H5_ADC2
      for (i = 0; i < ADC2_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist2[i]);
        }

      adc2 = stm32h5_adc_initialize(2, g_chanlist2, ADC2_NCHANNELS);
      if (adc2 == NULL)
        {
          aerr("ERROR: Failed to get ADC interface 1\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc1", adc2);
      if (ret < 0)
        {
          aerr("ERROR: adc_register /dev/adc1 failed: %d\n", ret);
          return ret;
        }
#endif

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_STM32H5_ADC1 || CONFIG_STM32H5_ADC2 */
#endif /* CONFIG_ADC */
