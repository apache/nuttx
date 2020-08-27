/****************************************************************************
 * boards/sim/sim/sim/src/sam_bringup.c
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
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/video/fb.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/wireless/pktradio.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_null.h>
#include <nuttx/wireless/ieee802154/ieee802154_loopback.h>

#include "up_internal.h"
#include "sim.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up simulated board features
 *
 ****************************************************************************/

int sim_bringup(void)
{
#ifdef CONFIG_ONESHOT
  FAR struct oneshot_lowerhalf_s *oneshot;
#endif
#ifdef CONFIG_RAMMTD
  FAR uint8_t *ramstart;
#endif
  int ret = OK;

#ifdef CONFIG_FS_BINFS
  /* Mount the binfs file system */

  ret = mount(NULL, "/bin", "binfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount binfs at /bin: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, SIM_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             SIM_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#ifdef CONFIG_LIB_ZONEINFO_ROMFS
  /* Mount the TZ database */

  sim_zoneinfo(3);
#endif

#ifdef CONFIG_EXAMPLES_GPIO
  /* Initialize simulated GPIO drivers */

  sim_gpio_initialize();
#endif

#ifdef CONFIG_RAMMTD
  /* Create a RAM MTD device if configured */

  ramstart = (FAR uint8_t *)kmm_malloc(128 * 1024);
  if (ramstart == NULL)
    {
      syslog(LOG_ERR, "ERROR: Allocation for RAM MTD failed\n");
    }
  else
    {
      /* Initialized the RAM MTD */

      FAR struct mtd_dev_s *mtd = rammtd_initialize(ramstart, 128 * 1024);
      if (mtd == NULL)
        {
          syslog(LOG_ERR, "ERROR: rammtd_initialize failed\n");
          kmm_free(ramstart);
        }
      else
        {
          /* Erase the RAM MTD */

          ret = mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: IOCTL MTDIOC_BULKERASE failed\n");
            }

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
          /* Initialize a SMART Flash block device and bind it to the MTD
           * device.
           */

          smart_initialize(0, mtd, NULL);

#elif defined(CONFIG_FS_SPIFFS)
          /* Register the MTD driver so that it can be accessed from the
           * VFS.
           */

          ret = register_mtddriver("/dev/rammtd", mtd, 0755, NULL);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n",
                     ret);
            }

          /* Mount the SPIFFS file system */

          ret = mount("/dev/rammtd", "/mnt/spiffs", "spiffs", 0, NULL);
          if (ret < 0)
            {
              syslog(LOG_ERR,
                     "ERROR: Failed to mount SPIFFS at /mnt/spiffs: %d\n",
                     ret);
            }

#elif defined(CONFIG_FS_LITTLEFS)
          /* Register the MTD driver so that it can be accessed from the
           * VFS.
           */

          ret = register_mtddriver("/dev/rammtd", mtd, 0755, NULL);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n",
                     ret);
            }

          /* Mount the LittleFS file system */

          ret = mount("/dev/rammtd", "/mnt/lfs", "littlefs", 0,
                      "forceformat");
          if (ret < 0)
            {
              syslog(LOG_ERR,
                     "ERROR: Failed to mount LittleFS at /mnt/lfs: %d\n",
                     ret);
            }

#elif defined(CONFIG_FS_NXFFS)
          /* Initialize to provide NXFFS on the MTD interface */

          ret = nxffs_initialize(mtd);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: NXFFS initialization failed: %d\n",
                     ret);
            }
#endif
        }
    }
#endif

#ifdef CONFIG_ONESHOT
  /* Get an instance of the simulated oneshot timer */

  oneshot = oneshot_initialize(0, 0);
  if (oneshot == NULL)
    {
      syslog(LOG_ERR, "ERROR: oneshot_initialize failed\n");
    }
  else
    {
#ifdef CONFIG_CPULOAD_ONESHOT
      /* Configure the oneshot timer to support CPU load measurement */

      nxsched_oneshot_extclk(oneshot);

#else
      /* Initialize the simulated oneshot driver */

      ret = oneshot_register("/dev/oneshot", oneshot);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to register oneshot at /dev/oneshot: %d\n",
                 ret);
        }
#endif
    }
#endif

#ifdef CONFIG_AJOYSTICK
  /* Initialize the simulated analog joystick input device */

  sim_ajoy_initialize();
#endif

#if defined(CONFIG_NX) && defined(CONFIG_SIM_TOUCHSCREEN)
  /* Initialize the touchscreen */

  ret = sim_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sim_tsc_setup failed: %d\n", ret);
    }
#else

#  ifdef CONFIG_VIDEO_FB
  /* Initialize and register the simulated framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#  endif

#  ifdef CONFIG_SIM_TOUCHSCREEN
  /* Initialize the touchscreen */

  ret = sim_tsc_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sim_tsc_initialize failed: %d\n", ret);
    }
#  endif

#endif

#ifdef CONFIG_IEEE802154_LOOPBACK
  /* Initialize and register the IEEE802.15.4 MAC network loop device */

  ret = ieee8021514_loopback();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ieee8021514_loopback() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_PKTRADIO_LOOPBACK
  /* Initialize and register the IEEE802.15.4 MAC network loop device */

  ret = pktradio_loopback();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pktradio_loopback() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_BLUETOOTH_NULL
  /* Register the NULL Bluetooth network device */

  ret = btnull_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btnull_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SIM_HCISOCKET
  /* Register the Host Bluetooth network device via HCI socket */

  ret = bthcisock_register(0);  /* Use HCI0 */
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bthcisock_register() failed: %d\n", ret);
    }
#endif

  return ret;
}
