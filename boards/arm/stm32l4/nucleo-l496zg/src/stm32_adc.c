/****************************************************************************
 * boards/arm/stm32l4/nucleo-l496zg/src/stm32_adc.c
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
#include "stm32l4_gpio.h"
#include "stm32l4_adc.h"
#include "nucleo-144.h"

#ifdef CONFIG_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Up to 3 ADC interfaces are supported */

#if STM32L4_NADC < 3
#  undef CONFIG_STM32L4_ADC3
#endif

#if STM32L4_NADC < 2
#  undef CONFIG_STM32L4_ADC2
#endif

#if STM32L4_NADC < 1
#  undef CONFIG_STM32L4_ADC1
#endif

#if defined(CONFIG_STM32L4_ADC1) || defined(CONFIG_STM32L4_ADC2) || defined(CONFIG_STM32L4_ADC3)

/* The number of ADC channels in the conversion list */

#define ADC1_NCHANNELS 1
#define ADC2_NCHANNELS 3
#define ADC3_NCHANNELS 2

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Identifying number of each ADC channel: Variable Resistor.
 *
 * {1,  2,  3, 4,  5,  6, 7,  8,  9, 10, 11, 12, 13, 15};
 */

#ifdef CONFIG_STM32L4_ADC1
static const uint8_t  g_chanlist_adc1[ADC1_NCHANNELS] =
{
  3
};

/* Configurations of pins used by each ADC channels
 *
 * {GPIO_ADC1_IN1,  GPIO_ADC1_IN2,  GPIO_ADC1_IN3,
 *  GPIO_ADC1_IN4,  GPIO_ADC1_IN5,  GPIO_ADC1_IN6,
 *  GPIO_ADC1_IN7,  GPIO_ADC1_IN8,  GPIO_ADC1_IN9,
 *  GPIO_ADC1_IN10, GPIO_ADC1_IN11, GPIO_ADC1_IN12,
 *  GPIO_ADC1_IN13, GPIO_ADC1_IN15};
 */

static const uint32_t g_pinlist_adc1[ADC1_NCHANNELS] =
{
  GPIO_ADC1_IN3
};
#endif

#ifdef CONFIG_STM32L4_ADC2
static const uint8_t  g_chanlist_adc2[ADC2_NCHANNELS] =
{
  4,
  17,
  18
}; /* IN4, DAC1 and DAC2 */
static const uint32_t g_pinlist_adc2[ADC2_NCHANNELS] =
{
  GPIO_ADC1_IN4, 0, 0
};
#endif

#ifdef CONFIG_STM32L4_ADC3
static const uint8_t  g_chanlist_adc3[ADC3_NCHANNELS] =
{
  17,
  18
}; /* TS and VBAT/3 */
static const uint32_t g_pinlist_adc3[ADC3_NCHANNELS] =
{
  0,
  0
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
  struct adc_dev_s *adc;
  int ret;
  int i;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Configure the pins as analog inputs for the selected channels */

#ifdef CONFIG_STM32L4_ADC1
      for (i = 0; i < ADC1_NCHANNELS; i++)
        {
          if (g_pinlist_adc1[i] != 0)
            {
              stm32l4_configgpio(g_pinlist_adc1[i]);
            }
        }
#endif

#ifdef CONFIG_STM32L4_ADC2
      for (i = 0; i < ADC2_NCHANNELS; i++)
        {
          if (g_pinlist_adc2[i] != 0)
            {
              stm32l4_configgpio(g_pinlist_adc2[i]);
            }
        }
#endif

#ifdef CONFIG_STM32L4_ADC3
      for (i = 0; i < ADC3_NCHANNELS; i++)
        {
          if (g_pinlist_adc3[i] != 0)
            {
              stm32l4_configgpio(g_pinlist_adc3[i]);
            }
        }
#endif

      /* Call stm32l4_adc_initialize() to get an instance of the ADC
       * interface
       */

#ifdef CONFIG_STM32L4_ADC1
      adc = stm32l4_adc_initialize(1, g_chanlist_adc1, ADC1_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC1 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc0", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_STM32L4_ADC2
      adc = stm32l4_adc_initialize(2, g_chanlist_adc2, ADC2_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC2 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc1" */

      ret = adc_register("/dev/adc1", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_STM32L4_ADC3
      adc = stm32l4_adc_initialize(3, g_chanlist_adc3, ADC3_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC3 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc2" */

      ret = adc_register("/dev/adc2", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register failed: %d\n", ret);
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_STM32L4_ADC1 || CONFIG_STM32L4_ADC2 || CONFIG_STM32L4_ADC3 */
#endif /* CONFIG_ADC */
