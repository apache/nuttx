/****************************************************************************
 * boards/arm/kinetis/freedom-k64f/src/k64_bringup.c
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

#include <nuttx/fs/fs.h>

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "kinetis_alarm.h"
#endif

#include "freedom-k64f.h"

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k64_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int k64_bringup(void)
{
  int ret;
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *lower;
#endif

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, PROCFS_MOUNTPOUNT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
      return ret;
    }
#endif

#if defined(CONFIG_KINETIS_I2C0)
  /* Initialize I2C buses */

  k64_i2cdev_initialize();
#endif

#ifdef HAVE_MMCSD
  /* Initialize the SDHC driver */

  ret = k64_sdhc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: k64_sdhc_initialize() failed: %d\n", ret);
    }

#ifdef CONFIG_FRDMK64F_SDHC_MOUNT
  else
    {
      /* Mount the volume on HSMCI0 */

      ret = nx_mount(CONFIG_FRDMK64F_SDHC_MOUNT_BLKDEV,
                     CONFIG_FRDMK64F_SDHC_MOUNT_MOUNTPOINT,
                     CONFIG_FRDMK64F_SDHC_MOUNT_FSTYPE,
                     0, NULL);

      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount %s: %d\n",
                 CONFIG_FRDMK64F_SDHC_MOUNT_MOUNTPOINT, ret);
        }
    }

#endif /* CONFIG_FRDMK64F_SDHC_MOUNT */
#endif /* HAVE_MMCSD */

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = k64_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: k64_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  k64_automount_initialize();
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the KINETIS lower-half RTC driver */

  lower = kinetis_rtc_lowerhalf();
  if (!lower)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to instantiate the RTC lower-half driver\n");
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind/register the RTC driver: %d\n",
                 ret);
        }
    }
#endif

  UNUSED(ret);
  return OK;
}

#endif /* CONFIG_BOARDCTL CONFIG_BOARD_LATE_INITIALIZE */
