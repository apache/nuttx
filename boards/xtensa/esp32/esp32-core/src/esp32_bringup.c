/****************************************************************************
 * boards/xtensa/esp32/esp32-core/src/esp32_bringup.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#ifdef CONFIG_TIMER
#  include <nuttx/timers/timer.h>
#endif
#include <syslog.h>
#include <sys/errno.h>

#include "esp32_procfs_imm.h"
#include "esp32-core.h"

#include "esp32_wlan.h"
#include "esp32_spiflash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32_TIMER0
#define ESP32_TIMER0 (0)
#endif

#ifdef CONFIG_ESP32_TIMER1
#define ESP32_TIMER1 (1)
#endif

#ifdef CONFIG_ESP32_TIMER2
#define ESP32_TIMER2 (2)
#endif

#ifdef CONFIG_ESP32_TIMER3
#define ESP32_TIMER3 (3)
#endif

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
      syslog(LOG_ERR, "Failed to initialize SD slot %d: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP32_SPIFLASH
  ret = esp32_spiflash_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
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
  ret = esp32_wlan_initialize();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize WiFi\n");
      return ret;
    }
#endif

#endif

#ifdef CONFIG_TIMER
  /* Configure TIMER driver */
#ifdef CONFIG_ESP32_TIMER0
  ret = esp32_timer_driver_setup("/dev/timer0", ESP32_TIMER0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32_TIMER1
  ret = esp32_timer_driver_setup("/dev/timer1", ESP32_TIMER1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32_TIMER2
  ret = esp32_timer_driver_setup("/dev/timer2", ESP32_TIMER2);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32_TIMER3
  ret = esp32_timer_driver_setup("/dev/timer3", ESP32_TIMER3);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
