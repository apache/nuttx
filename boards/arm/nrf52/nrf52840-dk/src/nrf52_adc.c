/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_adc.c
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

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include "nrf52_gpio.h"
#include "nrf52_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only one channel supported if TIMER triger enabled */

#ifdef CONFIG_NRF52_SAADC_TIMER
#  define ADC_NCHANNELS (1)
#else
#  define ADC_NCHANNELS (4)
#endif

/****************************************************************************
 * Private data
 ****************************************************************************/

/* ADC pins configuration */

static uint32_t g_adc_pins[ADC_NCHANNELS] =
{
  NRF52_ADC_CH0_PIN,            /* AIN1 */
#if (ADC_NCHANNELS == 4)
  NRF52_ADC_CH1_PIN,            /* AIN2 */
  NRF52_ADC_CH2_PIN,            /* AIN4 */
  NRF52_ADC_CH3_PIN,            /* AIN5 */
#endif
};

/* ADC channels configuration */

static struct nrf52_adc_channel_s g_adc_chanlist[ADC_NCHANNELS] =
{
  /* Channel 0 */

  {
    .p_psel  = NRF52_ADC_IN_IN1,
    .n_psel  = 0,
#ifdef CONFIG_NRF52_SAADC_LIMITS
    .limith = 0,
    .limitl = 0,
#endif
    .resp   = NRF52_ADC_RES_BYPASS,
    .resn   = NRF52_ADC_RES_BYPASS,
    .gain   = NRF52_ADC_GAIN_1,
    .refsel = NRF52_ADC_REFSEL_INTERNAL,
    .tacq   = NRF52_ADC_TACQ_40US,
    .mode   = NRF52_ADC_MODE_SE,
    .burst  = NRF52_ADC_BURST_DISABLE,
  },

#if (ADC_NCHANNELS == 4)
  /* Channel 1 */

  {
    .p_psel  = NRF52_ADC_IN_IN2,
    .n_psel  = 0,
#ifdef CONFIG_NRF52_SAADC_LIMITS
    .limith = 0,
    .limitl = 0,
#endif
    .resp   = NRF52_ADC_RES_BYPASS,
    .resn   = NRF52_ADC_RES_BYPASS,
    .gain   = NRF52_ADC_GAIN_1,
    .refsel = NRF52_ADC_REFSEL_INTERNAL,
    .tacq   = NRF52_ADC_TACQ_40US,
    .mode   = NRF52_ADC_MODE_SE,
    .burst  = NRF52_ADC_BURST_DISABLE,
  },

  /* Channel 2 */

  {
    .p_psel  = NRF52_ADC_IN_IN4,
    .n_psel  = 0,
#ifdef CONFIG_NRF52_SAADC_LIMITS
    .limith = 0,
    .limitl = 0,
#endif
    .resp   = NRF52_ADC_RES_BYPASS,
    .resn   = NRF52_ADC_RES_BYPASS,
    .gain   = NRF52_ADC_GAIN_1,
    .refsel = NRF52_ADC_REFSEL_INTERNAL,
    .tacq   = NRF52_ADC_TACQ_40US,
    .mode   = NRF52_ADC_MODE_SE,
    .burst  = NRF52_ADC_BURST_DISABLE,
  },

  /* Channel 3 */

  {
    .p_psel  = NRF52_ADC_IN_IN5,
    .n_psel  = 0,
#ifdef CONFIG_NRF52_SAADC_LIMITS
    .limith = 0,
    .limitl = 0,
#endif
    .resp   = NRF52_ADC_RES_BYPASS,
    .resn   = NRF52_ADC_RES_BYPASS,
    .gain   = NRF52_ADC_GAIN_1,
    .refsel = NRF52_ADC_REFSEL_INTERNAL,
    .tacq   = NRF52_ADC_TACQ_40US,
    .mode   = NRF52_ADC_MODE_SE,
    .burst  = NRF52_ADC_BURST_DISABLE,
  }
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC device.
 *
 ****************************************************************************/

int nrf52_adc_setup(void)
{
  static bool       initialized = false;
  struct adc_dev_s *adc         = NULL;
  int               ret         = OK;
  int               i           = 0;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Configure ADC pins */

      for (i = 0; i < ADC_NCHANNELS; i += 1)
        {
          nrf52_gpio_config(g_adc_pins[i]);
        }

      /* Call nrf52_adcinitialize() to get an instance of the ADC interface */

      adc = nrf52_adcinitialize(g_adc_chanlist, ADC_NCHANNELS);
      if (!adc)
        {
          aerr("ERROR: Failed to get the NRF52 ADC lower half\n");
          ret = -ENODEV;
          goto errout;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc0", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register failed: %d\n", ret);
          goto errout;
        }

      /* Now we are initialized */

      initialized = true;
    }

errout:
  return ret;
}
