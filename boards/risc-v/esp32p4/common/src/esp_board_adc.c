/****************************************************************************
 * boards/risc-v/esp32p4/common/src/esp_board_adc.c
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

#define ADC0_MAX_CHANNELS 8
#define ADC1_MAX_CHANNELS 6

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Select channels to be used for each ADC.
 *
 * GPIOs are fixed for each channel and configured in the lower-half driver.
 *
 *               ADC 1
 * Channel: 0  1  2  3  4  5  6  7
 * GPIO:    16 17 18 19 20 21 22 23
 *
 *               ADC 2
 * Channel: 0  1  2  3  4  5
 * GPIO:    49 50 51 52 53 54
 * On the chanlist arrays below, channels are added +1. Do not change.
 */

#ifdef CONFIG_ESPRESSIF_ADC_1
static const uint8_t g_chanlist_adc1[ADC0_MAX_CHANNELS] =
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
#ifdef CONFIG_ESPRESSIF_ADC_1_CH6
  7,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH7
  8,
#endif
};
#endif

#ifdef CONFIG_ESPRESSIF_ADC_2
static const uint8_t g_chanlist_adc2[ADC1_MAX_CHANNELS] =
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_adc_register
 *
 * Description:
 *   This function registers the ADC driver for the specified ADC channel.
 *   It initializes the ADC hardware, creates the device name, and registers
 *   the ADC device with the system.
 *
 * Input Parameters:
 *   adc_num - The ADC channel number to register.
 *
 * Returned Value:
 *   Returns zero (OK) on successful registration; a negated errno value is
 *   returned to indicate the nature of any failure.
 *
 ****************************************************************************/

static int board_adc_register(int adc_num)
{
  int ret;
  char devname[12];
  struct adc_dev_s *adcdev;

  adcdev = kmm_malloc(sizeof(struct adc_dev_s));
  if (adcdev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to allocate adc_dev_s instance\n");
      return -ENOMEM;
    }

  memset(adcdev, 0, sizeof(struct adc_dev_s));

  switch (adc_num)
    {
      case 1:
#ifdef CONFIG_ESPRESSIF_ADC_1
        adcdev = esp_adc_initialize(adc_num, g_chanlist_adc1);
        break;
#endif
      case 2:
#ifdef CONFIG_ESPRESSIF_ADC_2
        adcdev = esp_adc_initialize(adc_num, g_chanlist_adc2);
        break;
#endif
      default:
        syslog(LOG_ERR, "ERROR: Unsupported ADC number: %d\n", adc_num);
        return ERROR;
    }

  if (adcdev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize ADC %d\n", adc_num);
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adcx" */

  snprintf(devname, sizeof(devname), "/dev/adc%d", adc_num - 1);
  ret = adc_register(devname, adcdev);
  if (ret < 0)
    {
      kmm_free(adcdev);
      syslog(LOG_ERR, "ERROR: adc_register %s failed: %d\n", devname, ret);
      return ret;
    }

  return ret;
}

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

#ifdef CONFIG_ESPRESSIF_ADC_1
  ret = board_adc_register(1);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESPRESSIF_ADC_2
  ret = board_adc_register(2);
  if (ret != OK)
    {
      return ret;
    }
#endif

  return ret;
}
