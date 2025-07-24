/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/src/stm32_adc.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_adc.h"
#include "nucleo-h743zi.h"

#ifdef CONFIG_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Up to 3 ADC interfaces are supported */

#if defined(CONFIG_STM32H7_ADC1) || defined(CONFIG_STM32H7_ADC2) || \
    defined(CONFIG_STM32H7_ADC3)

/* The number of ADC channels in the conversion list */

#define ADC1_NCHANNELS 7
#define ADC2_NCHANNELS 5
#define ADC3_NCHANNELS 1

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32H7_ADC1
/* Identifying number of each ADC channel: Variable Resistor.
 *
 * ADC1: {5, 10, 15, 18, 19, 7, 12};
 */

static const uint8_t g_adc1_chanlist[ADC1_NCHANNELS] =
{
  5, 10, 15, 18, 19, 7, 12
};

static const uint32_t g_adc1_pinlist[ADC1_NCHANNELS] =
  {
    GPIO_ADC12_INP5,
    GPIO_ADC123_INP10,
    GPIO_ADC12_INP15,
    GPIO_ADC12_INP18,
    GPIO_ADC12_INP19,
    GPIO_ADC123_INP7,
    GPIO_ADC123_INP12
  };

#endif /* CONFIG_STM32H7_ADC1 */

/****************************************************************************
 * ADC2
 ****************************************************************************/
#ifdef CONFIG_STM32H7_ADC2

static const uint8_t g_adc2_chanlist[ADC2_NCHANNELS] =
{
  2, 3, 14, 4, 8
};

static const uint32_t g_adc2_pinlist[ADC2_NCHANNELS] =
{
  GPIO_ADC2_INP2,
  GPIO_ADC12_INP3,
  GPIO_ADC12_INP14,
  GPIO_ADC12_INP4,
  GPIO_ADC12_INP8
};
#endif /* CONFIG_STM32H7_ADC2 */

#ifdef CONFIG_STM32H7_ADC3
/* Identifying number of each ADC channel: Variable Resistor.
 *
 * ADC3: {6,};
 */

static const uint8_t  g_adc3_chanlist[ADC1_NCHANNELS] =
{
  11
};

/* Configurations of pins used by each ADC channels
 *
 *
 * ADC3: {GPIO_ADC3_INP6}
 */

static const uint32_t g_adc3_pinlist[ADC3_NCHANNELS] =
{
  GPIO_ADC123_INP11,
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
#if defined(CONFIG_STM32H7_ADC1) || defined(CONFIG_STM32H7_ADC3)
  static bool initialized = false;
  struct adc_dev_s *adc;
  int ret;
  int i;
  char devname[] = "/dev/adc0";

  /* Check if we have already initialized */

  if (!initialized)
    {
#endif
#if defined(CONFIG_STM32H7_ADC1)
      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC1_NCHANNELS; i++)
        {
          if (g_adc1_pinlist[i] != 0)
            {
              stm32_configgpio(g_adc1_pinlist[i]);
            }
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc = stm32h7_adc_initialize(1, g_adc1_chanlist, ADC1_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC1 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register(devname, adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register(%s) failed: %d\n", devname, ret);
          return ret;
        }

      devname[8]++;
#endif

#ifdef CONFIG_STM32H7_ADC2
      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC2_NCHANNELS; i++)
        {
          if (g_adc2_pinlist[i] != 0)
            {
              stm32_configgpio(g_adc2_pinlist[i]);
            }
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc = stm32h7_adc_initialize(2, g_adc2_chanlist, ADC2_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC2 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc[0-1]" */

      ret = adc_register(devname, adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register(%s) failed: %d\n", devname, ret);
          return ret;
        }

      devname[8]++;
#endif

#if defined(CONFIG_STM32H7_ADC3)
      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC3_NCHANNELS; i++)
        {
          if (g_adc3_pinlist[i] != 0)
            {
              stm32_configgpio(g_adc3_pinlist[i]);
            }
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc = stm32h7_adc_initialize(3, g_adc3_chanlist, ADC3_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC3 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc[0-2]" */

      ret = adc_register(devname, adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register(%s) failed: %d\n", devname, ret);
          return ret;
        }
#endif

#if defined(CONFIG_STM32H7_ADC1) || defined(CONFIG_STM32H7_ADC2) || \
    defined(CONFIG_STM32H7_ADC3)
      /* Now we are initialized */

      initialized = true;
    }

  return OK;
#else
  return -ENOSYS;
#endif
}

#endif /* CONFIG_STM32H7_ADC1 || CONFIG_STM32H7_ADC2 || CONFIG_STM32H7_ADC3 */
#endif /* CONFIG_ADC */
