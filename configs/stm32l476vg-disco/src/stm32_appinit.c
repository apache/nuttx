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
#include <stm32l4_uid.h>

#include <arch/board/board.h>
#include <arch/board/boardctl.h>

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

#if defined(HAVE_N25QXXX)
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
 * Private Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

 /****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_N25QXXX
FAR struct qspi_dev_s *g_qspi;
FAR struct mtd_dev_s *g_mtd_fs;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_BOARDCTL
int board_app_initialize(uintptr_t arg)
{
#ifdef HAVE_RTC_DRIVER
  FAR struct rtc_lowerhalf_s *rtclower;
#endif
#if defined(HAVE_N25QXXX)
FAR struct mtd_dev_s *mtd_temp;
#endif
#if defined(HAVE_N25QXXX_CHARDEV)
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

  rtclower = stm32l4_rtc_lowerhalf();
  if (!rtclower)
    {
      sdbg("ERROR: Failed to instantiate the RTC lower-half driver\n");
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
          sdbg("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#ifdef HAVE_N25QXXX
  /* Create an instance of the STM32L4 QSPI device driver */

  g_qspi = stm32l4_qspi_initialize(0);
  if (!g_qspi)
    {
      SYSLOG("ERROR: stm32l4_qspi_initialize failed\n");
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
          SYSLOG("ERROR: n25qxxx_initialize failed\n");
          return ret;
        }
      g_mtd_fs = mtd_temp;
        
#ifdef CONFIG_MTD_PARTITION
      /* Setup a partition of 256KiB for our file system. */

#if defined(CONFIG_N25QXXX_SECTOR512)
      mtd_temp = mtd_partition(g_mtd_fs, 0, 512);
#else
      mtd_temp = mtd_partition(g_mtd_fs, 0, 64);
#endif
      if (!g_mtd_fs)
        {
          SYSLOG("ERROR: mtd_partition failed\n");
          return ret;
        }

      g_mtd_fs = mtd_temp;
#endif

      /* Use the FTL layer to wrap the MTD driver as a block driver */

      ret = ftl_initialize(N25QXXX_MTD_MINOR, g_mtd_fs);
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
       * ftl_initialize() will be writable).
       */

      ret = bchdev_register(blockdev, chardev, false);
      if (ret < 0)
        {
          SYSLOG("ERROR: bchdev_register %s failed: %d\n", chardev, ret);
          return ret;
        }
    }
#endif

  return OK;
}
#endif /* CONFIG_LIB_BOARDCTL */

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  switch(cmd)
    {
#ifdef HAVE_N25QXXX
      case BIOC_ENTER_MEMMAP:
        {
          struct qspi_meminfo_s meminfo;

          /* Set up the meminfo like a regular memory transaction, many of the fields
           * are not used, the others are to set up for the 'read' command that will
           * automatically be issued by the controller as needed.
           * 6 = CONFIG_N25QXXX_DUMMIES;
           * 0xeb = N25QXXX_FAST_READ_QUADIO;
           */
          
          meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
          meminfo.addrlen = 3;
          meminfo.dummies = 6;    //CONFIG_N25QXXX_DUMMIES;
          meminfo.cmd     = 0xeb;//N25QXXX_FAST_READ_QUADIO;
          meminfo.addr    = 0;
          meminfo.buflen  = 0;
          meminfo.buffer  = NULL;
          
          stm32l4_qspi_enter_memorymapped(g_qspi, &meminfo, 80000000);
        }
        break;
      
      case BIOC_EXIT_MEMMAP:
        stm32l4_qspi_exit_memorymapped(g_qspi);
        break;
      
#endif
      
      default:
          return -EINVAL;
        break;
    }

    return OK;
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == 0)
    {
      return -EINVAL;
    }

  stm32l4_get_uniqueid(uniqueid);
  return OK;
}
#endif
