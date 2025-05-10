/****************************************************************************
 * boards/arm/nrf53/thingy53/src/nrf53_adc.c
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

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include "nrf53_gpio.h"
#include "nrf53_adc.h"

#include "thingy53.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only one channel supported if TIMER trigger enabled */

#define ADC_NCHANNELS (1)

/****************************************************************************
 * Private data
 ****************************************************************************/

/* ADC pins configuration */

static uint32_t g_adc_pins[ADC_NCHANNELS] =
{
  NRF53_ADC_CH0_PIN,            /* AIN2 */
};

/* ADC channels configuration */

static struct nrf53_adc_channel_s g_adc_chanlist[ADC_NCHANNELS] =
{
  /* Channel 0 */

  {
    .p_psel  = NRF53_ADC_IN_IN2,         /* AIN2 / P0.06 - BAT_MEAS */
    .n_psel  = 0,
#ifdef CONFIG_NRF53_SAADC_LIMITS
    .limith = 0,
    .limitl = 0,
#endif
    .resp   = NRF53_ADC_RES_BYPASS,
    .resn   = NRF53_ADC_RES_BYPASS,
    .gain   = NRF53_ADC_GAIN_1,
    .refsel = NRF53_ADC_REFSEL_INTERNAL,
    .tacq   = NRF53_ADC_TACQ_40US,
    .mode   = NRF53_ADC_MODE_SE,
    .burst  = NRF53_ADC_BURST_DISABLE,
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC device.
 *
 ****************************************************************************/

int nrf53_adc_setup(void)
{
  static bool       initialized = false;
  struct adc_dev_s *adc         = NULL;
  int               ret         = OK;
  int               i           = 0;

  /* Enable battery measurement.
   * TODO: this should be controlled by power management logic
   */

  nrf53_gpio_config(GPIO_BATT_EN);
  nrf53_gpio_write(GPIO_BATT_EN, true);

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Configure ADC pins */

      for (i = 0; i < ADC_NCHANNELS; i += 1)
        {
          nrf53_gpio_config(g_adc_pins[i]);
        }

      /* Call nrf53_adcinitialize() to get an instance of the ADC interface */

      adc = nrf53_adcinitialize(g_adc_chanlist, ADC_NCHANNELS);
      if (!adc)
        {
          aerr("ERROR: Failed to get the NRF53 ADC lower half\n");
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
