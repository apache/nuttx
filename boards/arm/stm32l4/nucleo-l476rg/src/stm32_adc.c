/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_adc.c
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
#include "stm32l4_pwm.h"
#include "nucleo-l476rg.h"

#ifdef CONFIG_STM32L4_ADC1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The number of ADC channels in the conversion list */

#ifdef CONFIG_ADC_DMA
#  define ADC1_NCHANNELS 2
#else
#  define ADC1_NCHANNELS 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Identifying number of each ADC channel. */

#ifdef CONFIG_INPUT_AJOYSTICK
#ifdef CONFIG_ADC_DMA
/* The Itead analog joystick gets inputs on ADC_IN1 and ADC_IN2 */

static const uint8_t  g_adc1_chanlist[ADC1_NCHANNELS] =
{
  1,
  2
};

/* Configurations of pins used byte each ADC channels */

static const uint32_t g_adc1_pinlist[ADC1_NCHANNELS]  =
{
  GPIO_ADC1_IN1,
  GPIO_ADC1_IN2
};

#else
/* Without DMA, only a single channel can be supported */

/* The Itead analog joystick gets input on ADC_IN1 */

static const uint8_t  g_adc1_chanlist[ADC1_NCHANNELS] =
{
  1
};

/* Configurations of pins used byte each ADC channels */

static const uint32_t g_adc1_pinlist[ADC1_NCHANNELS]  =
{
  GPIO_ADC1_IN1
};

#endif /* CONFIG_ADC_DMA */
#endif /* CONFIG_INPUT_AJOYSTICK */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32l4_adc_setup(void)
{
  struct adc_dev_s *adc;
  int ret;
  int i;

  /* Configure the pins as analog inputs for the selected channels */

  for (i = 0; i < ADC1_NCHANNELS; i++)
    {
      stm32l4_configgpio(g_adc1_pinlist[i]);
    }

  /* Call stm32l4_adc_initialize() to get an instance of the ADC interface */

  adc = stm32l4_adc_initialize(1, g_adc1_chanlist, ADC1_NCHANNELS);
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

  return OK;
}

#endif /* CONFIG_STM32L4_ADC1 */
