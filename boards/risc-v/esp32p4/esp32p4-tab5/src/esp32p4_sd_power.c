/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/src/esp32p4_sd_power.c
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
#include <syslog.h>

#include <errno.h>
#include "espressif/esp_ldo.h"

#include "esp32p4-tab5.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MIPI PHY LDO channel and voltage */

#define ESP_LDO_SD_CARD_CHAN       4
#define ESP_LDO_SD_CARD_VOLTAGE_MV 3300

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_ldo_config_t g_sd_card_ldo_config =
  {
    .chan_id = ESP_LDO_SD_CARD_CHAN,
    .voltage_mv = ESP_LDO_SD_CARD_VOLTAGE_MV,
    .handler = NULL,
  };

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tab5_sd_card_power
 *
 * Description:
 *   Enable or disable SD card power.
 *
 * Input Parameters:
 *   on - True to enable the SD card power, false to disable it.
 *
 * Returned Value:
 *   Zero on success, -1 on failure.
 *
 ****************************************************************************/

int tab5_sd_card_power(bool on)
{
  int ret = OK;
  if (on)
    {
      ret = esp_ldo_channel_acquire(&g_sd_card_ldo_config);
    }
  else
    {
      ret = esp_ldo_channel_release(&g_sd_card_ldo_config);
    }

  return ret;
}
