/****************************************************************************
 * boards/risc-v/esp32c3/common/src/esp_board_adc.c
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
#include <stdio.h>
#include <syslog.h>

#include <nuttx/analog/adc.h>

#include "espressif/esp_adc.h"

#include "esp_board_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The number of channels for each ADC */

#define ADC_MAX_CHANNELS 5

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Select channels to be used for each ADC.
 *
 * GPIOs are fixed for each channel and configured in the lower-half driver.
 *
 *               ADC 1
 * Channel: 0 1 2 3 4 5
 * GPIO:    0 1 2 3 4 5
 *
 * On the chanlist arrays below, channels are added +1. Do not change.
 */

#ifdef CONFIG_ESPRESSIF_ADC_1
static const uint8_t g_chanlist_adc1[ADC_MAX_CHANNELS] =
{
#ifdef CONFIG_ESPRESSIF_ADC_1_CH0
  1,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH1
  2,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH2
  3,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH3
  4,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH4
  5,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH5
  6,
#endif
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_adc_init
 *
 * Description:
 *   This function configures and initializes the ADC driver for the board.
 *   It allocates memory for the ADC device structure, sets up the ADC
 *   hardware, and registers the ADC device with the system.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Returns zero (OK) on successful initialization and registration of the
 *   ADC device; a negated errno value is returned to indicate the nature
 *   of any failure.
 *
 ****************************************************************************/

int board_adc_init(void)
{
  int ret;
  struct adc_dev_s *adcdev;

  adcdev = kmm_malloc(sizeof(struct adc_dev_s));
  if (adcdev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to allocate adc_dev_s instance\n");
      return -ENOMEM;
    }

  memset(adcdev, 0, sizeof(struct adc_dev_s));

  adcdev = esp_adc_initialize(1, g_chanlist_adc1);

  if (adcdev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize ADC1\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adcx" */

  ret = adc_register("/dev/adc0", adcdev);
  if (ret < 0)
    {
      kmm_free(adcdev);
      syslog(LOG_ERR, "ERROR: adc_register /dev/adc0 failed: %d\n", ret);
    }

  return ret;
}
