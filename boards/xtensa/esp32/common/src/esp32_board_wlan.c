/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_board_wlan.c
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

#include "esp32_spiflash.h"
#include "esp32_wlan.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
static int esp32_init_wifi_storage(void)
{
  int ret;
  const char *path = "/dev/mtdblock1";
  FAR struct mtd_dev_s *mtd_part;

  mtd_part = esp32_spiflash_alloc_mtdpart();
  if (!mtd_part)
    {
      syslog(LOG_ERR, "ERROR: Failed to alloc MTD partition of SPI Flash\n");
      return -1;
    }

  ret = register_mtddriver(path, mtd_part, 0777, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to regitser MTD: %d\n", ret);
      return -1;
    }

  ret = nx_mount(path, CONFIG_ESP32_WIFI_FS_MOUNTPT, "spiffs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the FS volume: %d\n", ret);
      return ret;
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_wlan_init
 *
 * Description:
 *   Configure the wireless subsystem.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_wlan_init(void)
{
  int ret = OK;

#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  ret = esp32_init_wifi_storage();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize WiFi storage\n");
      return ret;
    }
#endif

#ifdef ESP32_WLAN_HAS_STA
  ret = esp32_wlan_sta_initialize();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize WiFi station\n");
      return ret;
    }
#endif

#ifdef ESP32_WLAN_HAS_SOFTAP
  ret = esp32_wlan_softap_initialize();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize WiFi softAP\n");
      return ret;
    }
#endif

  return ret;
}

