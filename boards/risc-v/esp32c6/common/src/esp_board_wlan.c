/****************************************************************************
 * boards/risc-v/esp32c6/common/src/esp_board_wlan.c
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
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <syslog.h>
#include <debug.h>

#include <nuttx/wireless/wireless.h>

#include "espressif/esp_wlan.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_wlan_init
 *
 * Description:
 *   Configure the wireless subsystem.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_wlan_init(void)
{
  int ret = OK;

#ifdef ESP_WLAN_HAS_STA
  ret = esp_wlan_sta_initialize();
  if (ret)
    {
      wlerr("ERROR: Failed to initialize Wi-Fi station\n");
      return ret;
    }
#endif /* ESP_WLAN_HAS_STA */

#ifdef ESP_WLAN_HAS_SOFTAP
  ret = esp_wlan_softap_initialize();
  if (ret)
    {
      wlerr("ERROR: Failed to initialize Wi-Fi softAP\n");
      return ret;
    }
#endif /* ESP_WLAN_HAS_SOFTAP */

  return ret;
}

