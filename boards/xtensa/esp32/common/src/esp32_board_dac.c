/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_board_dac.c
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
#include <nuttx/analog/dac.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <esp32_dac.h>

#ifdef CONFIG_ESP_SDM
#  include "espressif/esp_sdm.h"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_dac_initialize
 *
 * Description:
 *   Initialize and register the Digital to Analog Converter (DAC) driver.
 *
 * Input Parameters:
 *   path - The device number, used to build the device path as
 *           /dev/dacN
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_dac_initialize(const char *path)
{
  int ret;

  /* Initialize DAC */

#ifdef CONFIG_ESP_SDM
  struct esp_sdm_chan_config_s config =
  {
    .gpio_num = 5,
    .sample_rate_hz = 1000 * 1000,
    .flags = 0,
  };

  struct dac_dev_s *dev = esp_sdminitialize(config);
#else
  struct dac_dev_s *dev = esp32_dac_initialize();
#endif
  if (dev != NULL)
    {
      /* Try to register the DAC */

      ret = dac_register(path, dev);
      if (ret < 0)
        {
          snerr("ERROR: Error registering DAC\n");
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
