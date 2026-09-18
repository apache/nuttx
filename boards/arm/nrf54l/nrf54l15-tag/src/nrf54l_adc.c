/****************************************************************************
 * boards/arm/nrf54l/nrf54l15-tag/src/nrf54l_adc.c
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
#include <nuttx/debug.h>
#include <nuttx/analog/adc.h>

#include "nrf54l_adc.h"
#include "nrf54l15-tag.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Internal VDD input, 0.9 V reference and gain 1/4: 3.6 V full scale. */

static const struct nrf54l_adc_channel_s g_adc_chanlist[] =
{
  {
    .p_psel = NRF54L_ADC_IN_VDD,
    .n_psel = 0,
    .gain   = NRF54L_ADC_GAIN_1_4,
    .refsel = NRF54L_ADC_REFSEL_INTERNAL,
    .tacq   = NRF54L_ADC_TACQ_40US,
    .mode   = NRF54L_ADC_MODE_SE,
    .burst  = NRF54L_ADC_BURST_DISABLE,
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_adc_setup
 *
 * Description:
 *   Register the supply voltage ADC channel.
 *
 ****************************************************************************/

int nrf54l_adc_setup(void)
{
  static bool initialized;
  struct adc_dev_s *adc;
  int ret;

  if (initialized)
    {
      return OK;
    }

  adc = nrf54l_adcinitialize(g_adc_chanlist, 1);
  if (adc == NULL)
    {
      return -ENODEV;
    }

  ret = adc_register("/dev/adc0", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register failed: %d\n", ret);
      return ret;
    }

  initialized = true;
  return OK;
}
