/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/src/lc823450_bringup.c
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

#include <stdbool.h>
#include <syslog.h>

#ifdef CONFIG_SMP
#  include <sched.h>
#endif

#include <nuttx/fs/fs.h>
#include <nuttx/sched.h>

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#ifdef CONFIG_WATCHDOG
#  include "lc823450_wdt.h"
#endif

#ifdef CONFIG_DVFS
#  include "lc823450_dvfs2.h"
#endif

#include "lc823450-xgevk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int lc823450_bringup(void)
{
  int ret;

#ifdef CONFIG_WATCHDOG
  lc823450_wdt_initialize();
#endif

#ifdef CONFIG_FS_PROCFS

#ifdef CONFIG_DVFS
  dvfs_procfs_register();
#endif

  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_FAT
  ret = nx_mount("/dev/mtdblock0p10", "/mnt/sd0", "vfat", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount vfat at /mnt/sd0: %d\n", ret);
    }
#endif

#ifdef CONFIG_BMA250
  lc823450_bma250initialize("/dev/accel");
#endif

#ifdef CONFIG_AUDIO_WM8776
  lc823450_wm8776initialize(0);
#endif

#if defined(CONFIG_RNDIS)
  uint8_t mac[6];
  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
