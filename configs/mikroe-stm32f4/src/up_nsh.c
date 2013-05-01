/****************************************************************************
 * config/mikroe_stm32f4/src/up_nsh.c
 * arch/arm/src/board/up_nsh.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#ifdef CONFIG_STM32_SPI3
#  include <nuttx/mmcsd.h>
#endif

#ifdef CONFIG_MTD_MP25P
#  include <nuttx/mtd.h>
#endif

#ifdef CONFIG_SYSTEM_USBMONITOR
#  include <apps/usbmonitor.h>
#endif

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#include "stm32.h"
#include "mikroe-stm32f4-internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define NSH_HAVEMMCSD   1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device is USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_SYSTEM_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO support
 * is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SPI3)
#  undef NSH_HAVEMMCSD
#endif

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

#  ifndef CONFIG_RAMMTD_BLOCKSIZE
#    define CONFIG_RAMMTD_BLOCKSIZE 512
#  endif

#  ifndef CONFIG_RAMMTD_ERASESIZE
#    define CONFIG_RAMMTD_ERASESIZE 4096
#  endif

#  ifndef CONFIG_EXAMPLES_SMART_NEBLOCKS
#    define CONFIG_EXAMPLES_SMART_NEBLOCKS (22)
#  endif

#  undef CONFIG_EXAMPLES_SMART_BUFSIZE
#  define CONFIG_EXAMPLES_SMART_BUFSIZE \
  (CONFIG_RAMMTD_ERASESIZE * CONFIG_EXAMPLES_SMART_NEBLOCKS)

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Pre-allocated simulated flash */

#ifdef CONFIG_MTD_RAM
//static uint8_t g_simflash[CONFIG_EXAMPLES_SMART_BUFSIZE];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int nsh_archinitialize(void)
{
#if defined(HAVE_USBHOST) || defined(HAVE_USBMONITOR)
  int ret;
#endif
#ifdef CONFIG_STM32_SPI3
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
#endif
  int ret;

  /* Configure SPI-based devices */

#ifdef CONFIG_STM32_SPI3
  /* Get the SPI port */

  message("nsh_archinitialize: Initializing SPI port 3\n");
  spi = up_spiinitialize(3);
  if (!spi)
    {
      message("nsh_archinitialize: Failed to initialize SPI port 3\n");
      return -ENODEV;
    }
  message("nsh_archinitialize: Successfully initialized SPI port 3\n");

  /* Now bind the SPI interface to the M25P8 SPI FLASH driver */

#ifdef CONFIG_MTD
  message("nsh_archinitialize: Bind SPI to the SPI flash driver\n");
  mtd = m25p_initialize(spi);
  if (!mtd)
    {
      message("nsh_archinitialize: Failed to bind SPI port 3 to the SPI FLASH driver\n");
    }
  else
    {
      message("nsh_archinitialize: Successfully bound SPI port 3 to the SPI FLASH driver\n");

      /* Now initialize a SMART Flash block device and bind it to the MTD device */
#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
      smart_initialize(0, mtd);
#endif
    }

  /* Create a RAM MTD device if configured */

#ifdef CONFIG_MTD_RAM

  {
      uint8_t *start = (uint8_t *) kmalloc(CONFIG_EXAMPLES_SMART_BUFSIZE);
      mtd = rammtd_initialize(start, CONFIG_EXAMPLES_SMART_BUFSIZE);
      mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);

      /* Now initialize a SMART Flash block device and bind it to the MTD device */
#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
      smart_initialize(1, mtd);
#endif
  }

#endif /* CONFIG_MTD_RAM */

#endif /* CONFIG_MTD */

//#warning "Now what are we going to do with this SPI FLASH driver?"
#endif

  /* Create the SPI FLASH MTD instance */
  /* The M25Pxx is not a good media to implement a file system..
   * its block sizes are too large
   */

  /* Mount the SDIO-based MMC/SD block driver */

#ifdef NSH_HAVEMMCSD
  /* Bind the spi interface to the MMC/SD driver */

  message("nsh_archinitialize: Bind SDIO to the MMC/SD driver, minor=%d\n",
          CONFIG_NSH_MMCSDMINOR);
  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi);
  if (ret != OK)
    {
      message("nsh_archinitialize: Failed to bind SPI to the MMC/SD driver: %d\n", ret);
    }
  else
    {
      message("nsh_archinitialize: Successfully bound SPI to the MMC/SD driver\n");
  
    }
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize() starts a thread
   * will monitor for USB connection and disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      message("nsh_archinitialize: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start(0, NULL);
  if (ret != OK)
    {
      message("nsh_archinitialize: Start USB monitor: %d\n", ret);
    }
#endif

  return OK;
}
