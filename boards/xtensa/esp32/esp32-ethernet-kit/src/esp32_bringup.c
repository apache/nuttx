/****************************************************************************
 * boards/xtensa/esp32/esp32-ethernet-kit/src/esp32_bringup.c
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
#include <syslog.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mount.h>
#include <syslog.h>
#include <debug.h>
#include <stdio.h>

#include <syslog.h>
#include <sys/errno.h>
#include <nuttx/himem/himem.h>

#include "esp32_procfs_imm.h"

#include "esp32_wlan.h"
#include "esp32_spiflash.h"
#include "esp32_partition.h"

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_TIMER
#  include "esp32_board_tim.h"
#endif

#ifdef CONFIG_WATCHDOG
#  include "esp32_board_wdt.h"
#endif

#include "esp32-ethernet-kit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
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

  ret = mount(path, CONFIG_ESP32_WIFI_FS_MOUNTPT, "spiffs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the FS volume: %d\n", errno);
      return ret;
    }

  return 0;
}
#endif

int esp32_bringup(void)
{
  int ret;

#ifdef CONFIG_XTENSA_IMEM_PROCFS
  /* Register the internal memory procfs entry.
   * This must be done before the procfs is mounted.
   */

  ret = imm_procfs_register();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register internal memory to PROCFS: %d\n",
             ret);
    }

#endif

#if defined(CONFIG_ESP32_SPIRAM) && \
    defined(CONFIG_ESP32_SPIRAM_BANKSWITCH_ENABLE)
  ret = esp_himem_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init HIMEM: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_MMCSD
  ret = esp32_mmcsd_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32_SPIFLASH

#ifdef CONFIG_ESP32_SPIFLASH_ENCRYPTION_TEST
  esp32_spiflash_encrypt_test();
#endif

  ret = esp32_spiflash_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
      return ret;
    }
#endif

#ifdef CONFIG_ESP32_PARTITION
  ret = esp32_partition_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize partition error=%d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32_WIRELESS

#ifdef CONFIG_ESP32_WIFI_SAVE_PARAM
  ret = esp32_init_wifi_storage();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize WiFi storage\n");
      return ret;
    }
#endif

#ifdef CONFIG_NET
  ret = esp32_wlan_sta_initialize();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize WiFi\n");
      return ret;
    }
#endif

#endif

#ifdef CONFIG_TIMER
  /* Configure timer driver */

  ret = board_timer_init();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer drivers: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_WATCHDOG
  /* Configure watchdog timer */

  ret = board_wdt_init();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize watchdog drivers: %d\n",
             ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
