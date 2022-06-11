/****************************************************************************
 * boards/arm/kinetis/freedom-k66f/src/k66_bringup.c
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

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include <nuttx/spi/spi.h>
#include <nuttx/input/buttons.h>

#include "kinetis_spi.h"
#include "freedom-k66f.h"

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k66_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int k66_bringup(void)
{
#ifdef HAVE_SPI
  struct spi_dev_s *spi1;
#endif
  int ret;

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, PROCFS_MOUNTPOUNT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_MMCSD
  /* Initialize the SDHC driver */

  ret = k66_sdhc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: k66_sdhc_initialize() failed: %d\n", ret);
    }

#  ifdef CONFIG_FRDMK66F_SDHC_MOUNT
  else
    {
      /* Mount the volume on HSMCI0 */

      ret = nx_mount(CONFIG_FRDMK66F_SDHC_MOUNT_BLKDEV,
                     CONFIG_FRDMK66F_SDHC_MOUNT_MOUNTPOINT,
                     CONFIG_FRDMK66F_SDHC_MOUNT_FSTYPE,
                     0, NULL);

      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount %s: %d\n",
                 CONFIG_FRDMK66F_SDHC_MOUNT_MOUNTPOINT, ret);
        }
    }

#  endif /* CONFIG_FRDMK66F_SDHC_MOUNT */
#endif /* HAVE_MMCSD */

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = k66_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: k66_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  k66_automount_initialize();
#endif

#ifdef HAVE_RTC_DRIVER
  /* Initialize the RTC */

  ret = k66_rtc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize the RTC driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_SPI

  /* Verify we can initialize SPI bus 1 */

  spi1 = kinetis_spibus_initialize(1);

  if (!spi1)
    {
      syslog(LOG_ERR, "ERROR:FAILED to initialize SPI port 1\n");
      return -ENODEV;
    }
#endif

  UNUSED(ret);
  return OK;
}
#endif /* CONFIG_BOARDCTL CONFIG_BOARD_LATE_INITIALIZE */
