/****************************************************************************
 * arch/sim/src/up_initialize.c
 *
 *   Copyright (C) 2007-2009, 2011-2014 Gregory Nutt. All rights reserved.
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

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/syslog/ramlog.h>

#include "sim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_init_smartfs
 *
 * Description:
 *   Initialize a simulated SPI FLASH block device m25p MTD driver and bind
 *   it to a SMART Flash block device.
 *
 ****************************************************************************/

#if defined(CONFIG_FS_SMARTFS) && defined(CONFIG_SIM_SPIFLASH)
static void up_init_smartfs(void)
{
  FAR struct mtd_dev_s *mtd;
  FAR struct spi_dev_s *spi;

  /* Initialize a simulated SPI FLASH block device m25p MTD driver */

  spi = up_spiflashinitialize();
  mtd = m25p_initialize(spi);

  /* Now initialize a SMART Flash block device and bind it to the MTD device */

  smart_initialize(0, mtd, NULL);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS
 *   initialization after the basic OS services have been
 *   initialized.  The architecture specific details of
 *   initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the
 *   clock, and registering device drivers are some of the
 *   things that are different for each processor and hardware
 *   platform.
 *
 *   up_initialize is called after the OS initialized but
 *   before the init process has been started and before the
 *   libraries have been initialized.  OS services and driver
 *   services are available.
 *
 ****************************************************************************/

void up_initialize(void)
{
  /* The real purpose of the following is to make sure that syslog
   * is drawn into the link.  It is needed by up_tapdev which is linked
   * separately.
   */

#ifdef CONFIG_NET
  syslog("SIM: Initializing");
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0
  /* Register devices */

#if defined(CONFIG_DEV_NULL)
  devnull_register();   /* Standard /dev/null */
#endif

#if defined(CONFIG_DEV_ZERO)
  devzero_register();   /* Standard /dev/zero */
#endif

#endif /* CONFIG_NFILE_DESCRIPTORS */

#if defined(USE_DEVCONSOLE)
  /* Start the sumulated UART device */

  simuart_start();

  /* Register a console (or not) */

  up_devconsole();          /* Our private /dev/console */
#elif defined(CONFIG_RAMLOG_CONSOLE)
  ramlog_consoleinit();
#endif

#ifdef CONFIG_SYSLOG_CHAR
  syslog_initialize();
#endif
#ifdef CONFIG_RAMLOG_SYSLOG
  ramlog_sysloginit();      /* System logging device */
#endif

#if defined(CONFIG_FS_FAT) && !defined(CONFIG_DISABLE_MOUNTPOINT)
  up_registerblockdevice(); /* Our FAT ramdisk at /dev/ram0 */
#endif

#ifdef CONFIG_NET
  netdriver_init();         /* Our "real" network driver */
#endif

#if defined(CONFIG_FS_SMARTFS) && defined(CONFIG_SIM_SPIFLASH)
  up_init_smartfs();
#endif
}
