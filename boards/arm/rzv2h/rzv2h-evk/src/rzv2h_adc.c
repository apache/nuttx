/****************************************************************************
 * boards/arm/rzv2h/rzv2h-evk/src/rzv2h_adc.c
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
#include <stdint.h>
#include <errno.h>

#include <nuttx/analog/adc.h>
#include <nuttx/debug.h>

#include <arch/board/board.h>

#include "rzv2h_adc.h"
#include "rzv2h_gpio.h"
#include "rzv2h-evk.h"

#ifdef CONFIG_RZV2H_ADC

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_adc0_registered;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_adc_channel_mask
 *
 * Description:
 *   Build the ADC0 group-A scan mask from the channels selected in Kconfig.
 *
 * Return Value:
 *   ADC0 channel mask. Bit n selects ANInnn.
 *
 ****************************************************************************/

static uint32_t rzv2h_adc_channel_mask(void)
{
  uint32_t mask = 0;

#ifdef CONFIG_RZV2H_ADC0_ANI000
  mask |= RZV2H_ADC_CHANNEL_BIT(0);
#endif
#ifdef CONFIG_RZV2H_ADC0_ANI001
  mask |= RZV2H_ADC_CHANNEL_BIT(1);
#endif
#ifdef CONFIG_RZV2H_ADC0_ANI002
  mask |= RZV2H_ADC_CHANNEL_BIT(2);
#endif
#ifdef CONFIG_RZV2H_ADC0_ANI003
  mask |= RZV2H_ADC_CHANNEL_BIT(3);
#endif
#ifdef CONFIG_RZV2H_ADC0_ANI004
  mask |= RZV2H_ADC_CHANNEL_BIT(4);
#endif
#ifdef CONFIG_RZV2H_ADC0_ANI005
  mask |= RZV2H_ADC_CHANNEL_BIT(5);
#endif
#ifdef CONFIG_RZV2H_ADC0_ANI006
  mask |= RZV2H_ADC_CHANNEL_BIT(6);
#endif
#ifdef CONFIG_RZV2H_ADC0_ANI007
  mask |= RZV2H_ADC_CHANNEL_BIT(7);
#endif

  return mask;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_adc_setup
 *
 * Description:
 *   Create the ADC0 lower-half instance for the board-selected dedicated
 *   analog inputs and register it as /dev/adc0.
 *
 * Return Value:
 *   OK on success, including repeated calls; -EINVAL if no ADC channel is
 *   selected; or another negated errno on initialization or registration
 *   failure.
 *
 ****************************************************************************/

int rzv2h_adc_setup(void)
{
  struct adc_dev_s *adc;
  uint32_t channel_mask;
  int ret;

  channel_mask = rzv2h_adc_channel_mask();
  if (channel_mask == 0)
    {
      return -EINVAL;
    }

  if (g_adc0_registered)
    {
      return OK;
    }

#ifdef CONFIG_RZV2H_ADC_TRIGGER_EXTERNAL
  ret = rzv2h_configgpio(BOARD_ADC0_ADTRG_GPIO);
  if (ret < 0)
    {
      aerr("ERROR: Failed to configure ADC ADTRG pin: %d\n", ret);
      return ret;
    }
#endif

  adc = rzv2h_adc_initialize(channel_mask);
  if (adc == NULL)
    {
      aerr("ERROR: Failed to initialize RZ/V2H ADC0\n");
      return -ENODEV;
    }

  ret = adc_register("/dev/adc0", adc);
  if (ret < 0)
    {
      aerr("ERROR: Failed to register /dev/adc0: %d\n", ret);
      return ret;
    }

  g_adc0_registered = true;
  return OK;
}

#endif /* CONFIG_RZV2H_ADC */
