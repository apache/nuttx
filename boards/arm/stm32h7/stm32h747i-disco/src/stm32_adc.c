/****************************************************************************
 * boards/arm/stm32h7/stm32h747i-disco/src/stm32_adc.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david.sidrane@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "stm32h747i-disco.h"

#ifdef CONFIG_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Up to 3 ADC interfaces are supported */

#if defined(CONFIG_STM32H7_ADC1) || defined(CONFIG_STM32H7_ADC2) || \
    defined(CONFIG_STM32H7_ADC3)
#ifndef CONFIG_STM32H7_ADC1
#  warning "Channel information only available for ADC1"
#endif

/* The number of ADC channels in the conversion list */

#define ADC1_NCHANNELS 5
#define ADC3_NCHANNELS 1

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32H7_ADC1
/* Identifying number of each ADC channel: Variable Resistor.
 *
 * ADC1: {5, 10, 12, 13, 15};
 */

static const uint8_t  g_adc1_chanlist[ADC1_NCHANNELS] =
{
  5, 10, 12, 13, 15
};

/* Configurations of pins used by each ADC channels
 *
 * ADC1: {GPIO_ADC12_INP5, GPIO_ADC123_INP10, GPIO_ADC123_INP12,
 *        GPIO_ADC12_INP13, GPIO_ADC12_INP15};
 */

static const uint32_t g_adc1_pinlist[ADC1_NCHANNELS] =
{
  GPIO_ADC12_INP5, GPIO_ADC123_INP10, GPIO_ADC123_INP12, GPIO_ADC12_INP13,
  GPIO_ADC12_INP15
};
#endif

#ifdef CONFIG_STM32H7_ADC3
/* Identifying number of each ADC channel: Variable Resistor.
 *
 * ADC3: {6,};
 */

static const uint8_t  g_adc3_chanlist[ADC1_NCHANNELS] =
{
  6
};

/* Configurations of pins used by each ADC channels
 *
 *
 * ADC3: {GPIO_ADC3_INP6}
 */

static const uint32_t g_adc3_pinlist[ADC3_NCHANNELS] =
{
  GPIO_ADC3_INP6
};
#endif

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

      /* Register the ADC driver at "/dev/adc0 or 1" */

      ret = adc_register(devname, adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register(%s) failed: %d\n", devname, ret);
          return ret;
        }
#endif

#if defined(CONFIG_STM32H7_ADC1) || defined(CONFIG_STM32H7_ADC3)
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
