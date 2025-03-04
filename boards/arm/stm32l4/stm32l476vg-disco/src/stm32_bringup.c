/****************************************************************************
 * boards/arm/stm32l4/stm32l476vg-disco/src/stm32_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdio.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include <stm32l4.h>
#include <stm32l4_uart.h>
#include <stm32l4_uid.h>

#include <arch/board/board.h>
#include <arch/board/boardctl.h>

#include <nuttx/drivers/drivers.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/i2c/i2c_master.h>

#include "stm32l476vg-disco.h"

/* Conditional logic in stm32l476vg-disco.h will determine if certain
 * features are supported.  Tests for these features need to be made after
 * including stm32l476vg-disco.h.
 */

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32l4_rtc.h"
#endif

#if defined(HAVE_N25QXXX)
#  include <nuttx/mtd/mtd.h>
#endif

#ifdef HAVE_N25QXXX
#  include <nuttx/spi/qspi.h>
#  include "stm32l4_qspi.h"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_N25QXXX
struct qspi_dev_s *g_qspi;
struct mtd_dev_s *g_mtd_fs;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *rtclower;
#endif
#if defined(HAVE_N25QXXX)
  struct mtd_dev_s *mtd_temp;
#endif
#if defined(HAVE_N25QXXX_CHARDEV)
#if defined(CONFIG_BCH)
  char blockdev[18];
  char chardev[12];
#endif /* defined(CONFIG_BCH) */
#endif
  int ret = OK;

#ifdef HAVE_PROC
  /* mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32 lower-half RTC driver */

  rtclower = stm32l4_rtc_lowerhalf();
  if (!rtclower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, rtclower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#ifdef HAVE_N25QXXX
  /* Create an instance of the STM32L4 QSPI device driver */

  g_qspi = stm32l4_qspi_initialize(0);
  if (!g_qspi)
    {
      _err("ERROR: stm32l4_qspi_initialize failed\n");
      return ret;
    }
  else
    {
      /* Use the QSPI device instance to initialize the
       * N25QXXX device.
       */

      mtd_temp = n25qxxx_initialize(g_qspi, true);
      if (!mtd_temp)
        {
          _err("ERROR: n25qxxx_initialize failed\n");
          return ret;
        }

      g_mtd_fs = mtd_temp;

#ifdef CONFIG_MTD_PARTITION
        {
          struct mtd_geometry_s geo;
          off_t nblocks;

          /* Setup a partition of 256KiB for our file system. */

          ret = MTD_IOCTL(g_mtd_fs, MTDIOC_GEOMETRY,
                          (unsigned long)(uintptr_t)&geo);
          if (ret < 0)
            {
              _err("ERROR: MTDIOC_GEOMETRY failed\n");
              return ret;
            }

          nblocks = (256 * 1024) / geo.blocksize;

          mtd_temp = mtd_partition(g_mtd_fs, 0, nblocks);
          if (!mtd_temp)
            {
              _err("ERROR: mtd_partition failed\n");
              return ret;
            }

          g_mtd_fs = mtd_temp;
        }
#endif

#ifdef HAVE_N25QXXX_SMARTFS
      /* Configure the device with no partition support */

      ret = smart_initialize(N25QXXX_SMART_MINOR, g_mtd_fs, NULL);
      if (ret != OK)
        {
          _err("ERROR: Failed to initialize SmartFS: %d\n", ret);
        }

#elif defined(HAVE_N25QXXX_NXFFS)
      /* Initialize to provide NXFFS on the N25QXXX MTD interface */

      ret = nxffs_initialize(g_mtd_fs);
      if (ret < 0)
        {
         _err("ERROR: NXFFS initialization failed: %d\n", ret);
        }

      /* Mount the file system at /mnt/nxffs */

      ret = nx_mount(NULL, "/mnt/nxffs", "nxffs", 0, NULL);
      if (ret < 0)
        {
          _err("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
          return ret;
        }

#else /* if  defined(HAVE_N25QXXX_CHARDEV) */
      /* Use the FTL layer to wrap the MTD driver as a block driver */

      ret = ftl_initialize(N25QXXX_MTD_MINOR, g_mtd_fs);
      if (ret < 0)
        {
          _err("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#if defined(CONFIG_BCH)
      /* Use the minor number to create device paths */

      snprintf(blockdev, sizeof(blockdev), "/dev/mtdblock%d",
               N25QXXX_MTD_MINOR);
      snprintf(chardev, sizeof(chardev), "/dev/mtd%d", N25QXXX_MTD_MINOR);

      /* Now create a character device on the block device */

      ret = bchdev_register(blockdev, chardev, false);
      if (ret < 0)
        {
          _err("ERROR: bchdev_register %s failed: %d\n", chardev, ret);
          return ret;
        }
#endif /* defined(CONFIG_BCH) */
#endif
    }
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32l4_usbhost_initialize() starts a
   * thread that will monitor for USB connection and disconnection events.
   */

  ret = stm32l4_usbhost_initialize();
  if (ret != OK)
    {
      udbg("ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start(0, NULL);
  if (ret != OK)
    {
      udbg("ERROR: Failed to start USB monitor: %d\n", ret);
      return ret;
    }
#endif

  return ret;
}

