/****************************************************************************
 * boards/arm/kinetis/freedom-k28f/src/k28_bringup.c
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

#include <sys/mount.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include "freedom-k28f.h"

#ifdef CONFIG_PL2303
#  include <nuttx/usb/pl2303.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#  include <net/if.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k28_bringup
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

int k28_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = k28_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: k28_pwm_setup() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_KINETIS_I2C0) || defined(CONFIG_KINETIS_I2C1)
  /* Initialize I2C buses */

  k28_i2cdev_initialize();
#endif

#ifdef HAVE_MMCSD
  /* Initialize the SDHC driver */

  ret = k28_sdhc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: k28_sdhc_initialize() failed: %d\n", ret);
    }

#ifdef CONFIG_FRDMK28F_SDHC_MOUNT
  else
    {
      /* Mount the volume on HSMCI0 */

      ret = mount(CONFIG_FRDMK28F_SDHC_MOUNT_BLKDEV,
                  CONFIG_FRDMK28F_SDHC_MOUNT_MOUNTPOINT,
                  CONFIG_FRDMK28F_SDHC_MOUNT_FSTYPE,
                  0, NULL);

      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount %s: %d\n",
                 CONFIG_FRDMK28F_SDHC_MOUNT_MOUNTPOINT, errno);
        }
    }

#endif /* CONFIG_FRDMK28F_SDHC_MOUNT */
#endif /* HAVE_MMCSD */

#if defined(CONFIG_USBDEV) && defined(CONFIG_KINETIS_USBOTG)
  if (k28_usbdev_initialize)
    {
      k28_usbdev_initialize();
    }
#endif

#ifdef CONFIG_PL2303
  usbdev_serialinitialize(0);
#endif

#ifdef CONFIG_RNDIS
  /* Register USB RNDIS Driver */

  uint8_t mac[6];
  mac[0] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 1)) & 0xff;
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;

  ret = usbdev_rndis_initialize(mac);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: usbdev_rndis_initialize() failed %d\n", ret);
    }
#endif

#if defined(CONFIG_USBHOST) && defined(CONFIG_KINETIS_USBHS)
  /* Initialize the USB highspeed host */

  k28_usbhost_initialize();
#endif

#ifdef HAVE_SDHC_AUTOMOUNTER
  /* Initialize the auto-mounter */

  k28_automount_initialize();
#endif

  UNUSED(ret);
  return OK;
}
