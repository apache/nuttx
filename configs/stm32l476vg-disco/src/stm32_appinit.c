/****************************************************************************
 * configs/stm32l476vg-disco/src/stm32_appinit.c
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

#include <sys/types.h>
#include <sys/mount.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <stm32l4.h>
#include <stm32l4_uart.h>

#include <arch/board/board.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ramdisk.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/mkfatfs.h>
#include <nuttx/binfmt/elf.h>
#include <nuttx/i2c/i2c_master.h>

#include "stm32l476vg-disco.h"

/* Conditional logic in stm32l476vg-disco.h will determine if certain features
 * are supported.  Tests for these features need to be made after including
 * stm32l476vg-disco.h.
 */

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32l4_rtc.h"
#endif

#if defined(HAVE_N25QXXX) || defined(HAVE_PROGMEM_CHARDEV)
#  include <nuttx/mtd/mtd.h>
#endif

#ifdef HAVE_N25QXXX
#  include <nuttx/spi/qspi.h>
#  include "stm32l4_qspi.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
#  define SYSLOG lldbg
#else
#  define SYSLOG dbg
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Application initialization stub for boardctl()
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_BOARDCTL
int board_app_initialize(void)
{
#ifdef HAVE_RTC_DRIVER
  FAR struct rtc_lowerhalf_s *lower;
#endif
#ifdef HAVE_N25QXXX
  FAR struct qspi_dev_s *qspi;
#endif
#if defined(HAVE_N25QXXX) || defined(HAVE_PROGMEM_CHARDEV)
  FAR struct mtd_dev_s *mtd;
#endif
#if defined(HAVE_N25QXXX_CHARDEV) || defined(HAVE_PROGMEM_CHARDEV)
  char blockdev[18];
  char chardev[12];
#endif
  int ret;

  (void)ret;
  
#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Configure CPU load estimation */

  cpuload_initialize_once();
#endif

#ifdef HAVE_PROC
  /* mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d (%d)\n",
             ret, errno);
      return ret;
    }
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32 lower-half RTC driver */

  lower = stm32l4_rtc_lowerhalf();
  if (!lower)
    {
      sdbg("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          sdbg("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#ifdef HAVE_N25QXXX
  /* Create an instance of the STM32L4 QSPI device driver */

  qspi = stm32l4_qspi_initialize(0);
  if (!qspi)
    {
      SYSLOG("ERROR: sam_qspi_initialize failed\n");
    }
  else
    {
      /* Use the QSPI device instance to initialize the
       * N25QXXX device.
       */

      mtd = n25qxxx_initialize(qspi, true);
      if (!mtd)
        {
          SYSLOG("ERROR: n25qxxx_initialize failed\n");
        }

#ifdef HAVE_N25QXXX_SMARTFS
      /* Configure the device with no partition support */

      SYSLOG("doing smart_initialize()\n");
      ret = smart_initialize(N25QXXX_SMART_MINOR, mtd, NULL);
      if (ret != OK)
        {
          SYSLOG("ERROR: Failed to initialize SmartFS: %d\n", ret);
        }

#elif defined(HAVE_N25QXXX_NXFFS)
      /* Initialize to provide NXFFS on the N25QXXX MTD interface */

      SYSLOG("doing nxffs_initialize()\n");
      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
         SYSLOG("ERROR: NXFFS initialization failed: %d\n", ret);
        }

      /* Mount the file system at /mnt/n25qxxx */

      ret = mount(NULL, "/mnt/n25qxxx", "nxffs", 0, NULL);
      if (ret < 0)
        {
          SYSLOG("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
          return ret;
        }

#else /* if  defined(HAVE_N25QXXX_CHARDEV) */
      /* Use the FTL layer to wrap the MTD driver as a block driver */

      ret = ftl_initialize(N25QXXX_MTD_MINOR, mtd);
      if (ret < 0)
        {
          SYSLOG("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

      /* Use the minor number to create device paths */

      snprintf(blockdev, 18, "/dev/mtdblock%d", N25QXXX_MTD_MINOR);
      snprintf(chardev, 12, "/dev/mtd%d", N25QXXX_MTD_MINOR);

      /* Now create a character device on the block device */

      /* NOTE:  for this to work, you will need to make sure that
       * CONFIG_FS_WRITABLE is set in the config.  It's not a user-
       * visible setting, but you can make it set by selecting an
       * arbitrary writable file system (you don't have to actually
       * use it, just select it so that the block device created via
       * ftl_initialize() will be writable).  Personally, I chose FAT,
       * because SMARTFS and NXFFS will cause the other code branches
       * above to become active.
       */

      ret = bchdev_register(blockdev, chardev, false);
      if (ret < 0)
        {
          SYSLOG("ERROR: bchdev_register %s failed: %d\n", chardev, ret);
          return ret;
        }
#endif
    }
#endif

  return OK;
}
#endif /* CONFIG_LIB_BOARDCTL */



#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
    return OK;
}
#endif
