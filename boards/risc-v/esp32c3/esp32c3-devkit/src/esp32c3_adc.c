/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit/src/esp32c3_adc.c
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

#include <debug.h>
#include <sys/types.h>

#include "esp32c3_adc.h"

#include "esp32c3-devkit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_adc_init
 *
 * Description:
 *   Configure the ADC driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_adc_init(void)
{
  int ret;
  struct adc_dev_s *adc;

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL0
  adc = esp32c3_adc_init(0);
  if (!adc)
    {
      syslog(LOG_ERR, "ERROR: Failed to get ADC channel 0 dev\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc0" */

  ret = adc_register("/dev/adc0", adc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL1
  adc = esp32c3_adc_init(1);
  if (!adc)
    {
      syslog(LOG_ERR, "ERROR: Failed to get ADC channel 1 dev\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc1" */

  ret = adc_register("/dev/adc1", adc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL2
  adc = esp32c3_adc_init(2);
  if (!adc)
    {
      syslog(LOG_ERR, "ERROR: Failed to get ADC channel 2 dev\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc2" */

  ret = adc_register("/dev/adc2", adc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL3
  adc = esp32c3_adc_init(3);
  if (!adc)
    {
      syslog(LOG_ERR, "ERROR: Failed to get ADC channel 3 dev\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc3" */

  ret = adc_register("/dev/adc3", adc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32C3_ADC1_CHANNEL4
  adc = esp32c3_adc_init(4);
  if (!adc)
    {
      syslog(LOG_ERR, "ERROR: Failed to get ADC channel 4 dev\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc4" */

  ret = adc_register("/dev/adc4", adc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
#endif

  return ret;
}
