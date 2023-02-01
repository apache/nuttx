/****************************************************************************
 * boards/risc-v/litex/arty_a7/src/litex_bringup.c
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
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include "litex.h"
#include "arty_a7.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: litex_bringup
 ****************************************************************************/

int litex_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, LITEX_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "failed to mount procfs at %s %d\n",
              LITEX_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_FS_AUTOMOUNTER
  /* Configure uSD automounter */

  ret = litex_automount_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: litex_automount_initialize() failed: %d\n",
              ret);
    }
#endif

#ifdef CONFIG_LITEX_SDIO
  /* Initialize the SDIO block driver */

  ret = litex_sdio_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "litex_sdio_initialize() failed %d\n", ret);
      return ret;
    }

  /* If automount not configured, force a mount point.
   * Assumes the card is always present.
   */

#ifndef CONFIG_FS_AUTOMOUNTER
#ifdef CONFIG_LITEX_SDIO_MOUNT
  /* Mount the volume on SDMMC0 */

  ret = nx_mount(CONFIG_LITEX_SDIO_MOUNT_BLKDEV,
                 CONFIG_LITEX_SDIO_MOUNT_MOUNTPOINT,
                 CONFIG_LITEX_SDIO_MOUNT_FSTYPE,
                 0, NULL);

  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount %s: %d\n",
              CONFIG_LITEX_SDIO_MOUNT_MOUNTPOINT, ret);
    }
#endif /* CONFIG_LITEX_SDIO_MOUNT */
#endif

#endif /* CONFIG_LITEX_SDIO */

#ifdef CONFIG_LITEX_PWM
  ret = litex_pwm_setup();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup PWM driver \n");
    }

#endif /* CONFIG_LITEX_PWM */

  return ret;
}
