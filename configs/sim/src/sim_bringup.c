/****************************************************************************
 * configs/sim/src/sam_bringup.c
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_GRAPHICS_TRAVELER_ROMFSDEMO
int trv_mount_world(int minor, FAR const char *mountpoint);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#define NEED_FRAMEBUFFER 1

/* If we are using the X11 touchscreen simulation, then the frame buffer
 * initialization will need to be done here.
 */

#if defined(CONFIG_SIM_X11FB) && defined(CONFIG_SIM_TOUCHSCREEN)
#  undef NEED_FRAMEBUFFER
#endif

/* Currently the only case we need to initialize the framebuffer here is
 * when we are testing the framebuffer character driver.
 */

#ifndef CONFIG_VIDEO_FB
#  undef NEED_FRAMEBUFFER
#endif

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
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, SIM_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             SIM_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_LIB_ZONEINFO_ROMFS
  /* Mount the TZ database */

  (void)sim_zoneinfo(3);
#endif

#ifdef CONFIG_GRAPHICS_TRAVELER_ROMFSDEMO
  /* Special initialization for the Traveler game simulation */

  (void)trv_mount_world(0, CONFIG_GRAPHICS_TRAVELER_DEFPATH);
#endif

#ifdef CONFIG_EXAMPLES_GPIO
  /* Initialize simulated GPIO drivers */

  (void)sim_gpio_initialize();
#endif

#ifdef CONFIG_RAMMTD
  /* Create a RAM MTD device if configured */

  ramstart = (FAR uint8_t *)kmm_malloc(32 * 1024);
  if (ramstart == NULL)
    {
      syslog(LOG_ERR, "ERROR: Allocation for RAM MTD failed\n");
    }
  else
    {
      /* Initialized the RAM MTD */

      FAR struct mtd_dev_s *mtd = rammtd_initialize(ramstart, 32 * 1024);
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

      sched_oneshot_extclk(oneshot);

#else
      /* Initialize the simulated oneshot driver */

      ret = oneshot_register("/dev/oneshot", oneshot);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register oneshot at /dev/oneshot: %d\n",
                 ret);
        }
#endif
    }
#endif

#ifdef CONFIG_AJOYSTICK
  /* Initialize the simulated analog joystick input device */

  sim_ajoy_initialize();
#endif

#if defined(CONFIG_SIM_X11FB) && defined(CONFIG_SIM_TOUCHSCREEN)
  /* Initialize the touchscreen */

  ret = sim_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sim_tsc_setup failed: %d\n", ret);
    }
#endif

#ifdef NEED_FRAMEBUFFER
  /* Initialize and register the simulated framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
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

#ifdef CONFIG_WIRELESS_BLUETOOTH
#ifdef CONFIG_BLUETOOTH_NULL
  /* Register the NULL Bluetooth network device */

  ret = btnull_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btnull_register() failed: %d\n", ret);
    }
#endif

  /* Initialize the Bluetooth stack (This will fail if no device has been
   * registered).
   */

  ret = bt_netdev_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bt_netdev_register() failed: %d\n", ret);
    }

#endif

  UNUSED(ret);
  return OK;
}
