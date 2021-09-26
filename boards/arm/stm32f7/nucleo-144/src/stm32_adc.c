/****************************************************************************
 * boards/arm/stm32f7/nucleo-144/src/stm32_adc.c
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
#include "nucleo-144.h"

#ifdef CONFIG_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Up to 3 ADC interfaces are supported */

#if STM32F7_NADC < 3
#  undef CONFIG_STM32F7_ADC3
#endif

#if STM32F7_NADC < 2
#  undef CONFIG_STM32F7_ADC2
#endif

#if STM32F7_NADC < 1
#  undef CONFIG_STM32F7_ADC1
#endif

#if defined(CONFIG_STM32F7_ADC1) || defined(CONFIG_STM32F7_ADC2) || defined(CONFIG_STM32F7_ADC3)
#ifndef CONFIG_STM32F7_ADC1
#  warning "Channel information only available for ADC1"
#endif

/* The number of ADC channels in the conversion list */

#define ADC1_NCHANNELS 3

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Identifying number of each ADC channel: Variable Resistor.
 *
 * {1,  2,  3, 4,  5,  6, 7,  8,  9, 10, 11, 12, 13, 15};
 */

#ifdef CONFIG_STM32F7_ADC1
static const uint8_t  g_chanlist[ADC1_NCHANNELS] =
{
  3, 10, 13
};

/* Configurations of pins used byte each ADC channels
 *
 * {GPIO_ADC1_IN1,  GPIO_ADC1_IN2,  GPIO_ADC1_IN3,
 *  GPIO_ADC1_IN4,  GPIO_ADC1_IN5,  GPIO_ADC1_IN6,
 *  GPIO_ADC1_IN7,  GPIO_ADC1_IN8,  GPIO_ADC1_IN9,
 *  GPIO_ADC1_IN10, GPIO_ADC1_IN11, GPIO_ADC1_IN12,
 *  GPIO_ADC1_IN13, GPIO_ADC1_IN15};
 */

static const uint32_t g_pinlist[ADC1_NCHANNELS] =
{
  GPIO_ADC1_IN3,
  GPIO_ADC1_IN10,
  GPIO_ADC1_IN13
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
#ifdef CONFIG_STM32F7_ADC1
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
          if (g_pinlist[i] != 0)
            {
              stm32_configgpio(g_pinlist[i]);
            }
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc = stm32_adc_initialize(1, g_chanlist, ADC1_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc0", adc);
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

#endif /* CONFIG_STM32F7_ADC1 || CONFIG_STM32F7_ADC2 || CONFIG_STM32F7_ADC3 */
#endif /* CONFIG_ADC */
