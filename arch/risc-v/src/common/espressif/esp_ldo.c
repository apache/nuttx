/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_ldo.c
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

#include <string.h>
#include <nuttx/debug.h>

#include "soc/soc_caps.h"
#include "esp_ldo.h"
#include "hal/ldo_ll.h"
#include "esp_ldo_regulator.h"

#ifndef CONFIG_ARCH_CHIP_ESP32P4
# error "ESP LDO Control usable for ESP32-P4 only"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_ldo_channel_acquire
 *
 * Description:
 *   Acquire an on-chip LDO regulator channel and enable its output at the
 *   requested voltage.
 *
 * Input Parameters:
 *   config - Channel description. chan_id and voltage_mv are inputs;
 *            handler receives the channel handle on success.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; -EIO is returned if the channel
 *   could not be acquired.
 *
 ****************************************************************************/

int esp_ldo_channel_acquire(struct esp_ldo_config_t *config)
{
  esp_ldo_channel_handle_t ldo_chan = NULL;
  esp_ldo_channel_config_t chan_cfg;
  esp_err_t err;

  DEBUGASSERT(config != NULL);

  memset(&chan_cfg, 0, sizeof(chan_cfg));
  chan_cfg.chan_id          = config->chan_id;
  chan_cfg.voltage_mv       = config->voltage_mv;
  chan_cfg.flags.adjustable = false;

  err = esp_ldo_acquire_channel(&chan_cfg, &ldo_chan);
  if (err != ESP_OK)
    {
      _err("acquire channel failed: %d\n", (int)err);
      return -EIO;
    }

  config->handler = (void *)ldo_chan;

  return OK;
}

/****************************************************************************
 * Name: esp_ldo_channel_release
 *
 * Description:
 *   Release a channel previously acquired with
 *   esp_ldo_channel_acquire().
 *
 * Input Parameters:
 *   config - The same structure passed to esp_ldo_channel_acquire(). Its
 *            handler field is cleared on success.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; -EINVAL if the channel was not
 *   acquired, or -EIO if the release failed.
 *
 ****************************************************************************/

int esp_ldo_channel_release(struct esp_ldo_config_t *config)
{
  esp_err_t err;

  DEBUGASSERT(config != NULL);

  if (config->handler == NULL)
    {
      return -EINVAL;
    }

  err = esp_ldo_release_channel((esp_ldo_channel_handle_t)config->handler);
  if (err != ESP_OK)
    {
      _err("release channel failed: %d\n", (int)err);
      return -EIO;
    }

  config->handler = NULL;

  return OK;
}
