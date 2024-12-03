/****************************************************************************
 * boards/xtensa/esp32s3/common/src/esp32s3_board_adc.c
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
#include <debug.h>
#include <sys/types.h>

#include "esp32s3_adc.h"

#include "esp32s3_board_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_adc_register
 *
 * Description:
 *   Register the ADC driver.
 *
 * Input Parameters:
 *   channel - ADC channel number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

static int board_adc_register(int channel)
{
  int ret;
  char devname[12];
  struct adc_dev_s *adcdev;

  adcdev = kmm_malloc(sizeof(struct adc_dev_s));
  if (adcdev == NULL)
    {
      aerr("ERROR: Failed to allocate adc_dev_s instance\n");
      return -ENOMEM;
    }

  memset(adcdev, 0, sizeof(struct adc_dev_s));
  esp32s3_adc_init(channel, adcdev);
  snprintf(devname, sizeof(devname), "/dev/adc%d", channel);

  /* Register the ADC driver at "/dev/adcx_x" */

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
 *   Configure the ADC driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_adc_init(void)
{
  int ret;

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL0
  ret = board_adc_register(ESP32S3_ADC1_CHANNEL0);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL1
  ret = board_adc_register(ESP32S3_ADC1_CHANNEL1);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL2
  ret = board_adc_register(ESP32S3_ADC1_CHANNEL2);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL3
  ret = board_adc_register(ESP32S3_ADC1_CHANNEL3);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL4
  ret = board_adc_register(ESP32S3_ADC1_CHANNEL4);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL5
  ret = board_adc_register(ESP32S3_ADC1_CHANNEL5);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL6
  ret = board_adc_register(ESP32S3_ADC1_CHANNEL6);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL7
  ret = board_adc_register(ESP32S3_ADC1_CHANNEL7);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL8
  ret = board_adc_register(ESP32S3_ADC1_CHANNEL8);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_ESP32S3_ADC1_CHANNEL9
  ret = board_adc_register(ESP32S3_ADC1_CHANNEL9);
  if (ret != OK)
    {
      return ret;
    }
#endif

  return ret;
}
