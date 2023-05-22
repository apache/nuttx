/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/src/stm32_adc.c
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

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>

#include "stm32_gpio.h"
#include "stm32_adc.h"

#ifndef CONFIG_STM32F7_ADC3
#  error "Only ADC3 channels are available on the arduino header of the board"
#endif

#if defined(CONFIG_ADC) && defined(CONFIG_STM32F7_ADC3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If DMA support is not enabled, then only a single channel
 * can be sampled.  Otherwise, data overruns would occur.
 */

#ifdef ADC_HAVE_DMA
#  define ADC3_NCHANNELS 6
#else
#  define ADC3_NCHANNELS 1
#endif

/* The number of ADC channels in the conversion list */

/* TODO DMA */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Identifying number of each ADC channel (even if NCHANNELS is less ) */

static const uint8_t g_chanlist[6] =
{
  0,
  4,
  5,
  6,
  7,
  8
};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist[6]  =
{
  GPIO_ADC3_IN0,                /* PA0/A0 */
  GPIO_ADC3_IN4,                /* PF/A1 */
  GPIO_ADC3_IN5,                /* PF7/A3 */
  GPIO_ADC3_IN6,                /* PF8/A0 */
  GPIO_ADC3_IN7,                /* PF9/A1 */
  GPIO_ADC3_IN8                 /* PF10/A3 */
};

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
#ifdef CONFIG_STM32F7_ADC3
  static bool initialized = false;
  struct adc_dev_s *adc;
  int ret;
  int i;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC3_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist[i]);
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc = stm32_adc_initialize(3, g_chanlist, ADC3_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc3", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
#else
  return -ENOSYS;
#endif
}

#endif /* (CONFIG_ADC) && (CONFIG_STM32F7_ADC3) */
