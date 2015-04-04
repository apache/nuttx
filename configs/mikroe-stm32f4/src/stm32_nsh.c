/****************************************************************************
 * config/mikroe_stm32f4/src/stm32_nsh.c
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
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
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#ifdef CONFIG_STM32_SPI3
#  include <nuttx/mmcsd.h>
#endif

#ifdef CONFIG_MTD_M25P
#  include <nuttx/mtd/mtd.h>
#endif

#ifdef CONFIG_SYSTEM_USBMONITOR
#  include <apps/usbmonitor.h>
#endif

#ifdef CONFIG_MIKROE_FLASH_CONFIG_PART
#ifdef CONFIG_PLATFORM_CONFIGDATA
#  include <nuttx/configdata.h>
#endif
#endif

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#ifdef CONFIG_AUDIO
#  include "nuttx/audio/audio.h"
#endif

#include "stm32.h"
#include "mikroe-stm32f4-internal.h"

/****************************************************************************
 * Pre-processor Definitions
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

#ifdef CONFIG_MIKROE_RAMMTD
#  ifndef CONFIG_MIKROE_RAMMTD_MINOR
#    define CONFIG_MIKROE_RAMMTD_MINOR    1
#  endif
#  ifndef CONFIG_MIKROE_RAMMTD_SIZE
#    define CONFIG_MIKROE_RAMMTD_SIZE     32
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int board_app_initialize(void)
{
#ifdef CONFIG_STM32_SPI3
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
#endif
#if defined(NSH_HAVEMMCSD) || defined(HAVE_USBHOST) || \
    defined(HAVE_USBMONITOR) || defined(CONFIG_LCD_MIO283QT2) || \
    defined(CONFIG_LCD_MIO283QT9A)
  int ret;
#endif

  /* Configure SPI-based devices */

#ifdef CONFIG_STM32_SPI3
  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port 3\n");
  spi = up_spiinitialize(3);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port 3\n");
      return -ENODEV;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port 3\n");

  /* Now bind the SPI interface to the M25P8 SPI FLASH driver */

#if defined(CONFIG_MTD) && defined(CONFIG_MIKROE_FLASH)
  syslog(LOG_INFO, "Bind SPI to the SPI flash driver\n");

  mtd = m25p_initialize(spi);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port 3 to the SPI FLASH driver\n");
    }
  else
    {
      syslog(LOG_INFO, "Successfully bound SPI port 3 to the SPI FLASH driver\n");

#ifdef CONFIG_MIKROE_FLASH_PART
      {
        int partno;
        int partsize;
        int partoffset;
        const char *partstring = CONFIG_MIKROE_FLASH_PART_LIST;
        const char *ptr;
        FAR struct mtd_dev_s *mtd_part;
        char  partname[4];

        /* Now create a partition on the FLASH device */

        partno = 0;
        ptr = partstring;
        partoffset = 0;

        while (*ptr != '\0')
          {
            /* Get the partition size */

            partsize = atoi(ptr);
            mtd_part = mtd_partition(mtd, partoffset, (partsize>>2)*16);
            partoffset += (partsize >> 2) * 16;

#ifdef CONFIG_MIKROE_FLASH_CONFIG_PART
            /* Test if this is the config partition */

            if (CONFIG_MIKROE_FLASH_CONFIG_PART_NUMBER == partno)
              {
                /* Register the partition as the config device */

                mtdconfig_register(mtd_part);
              }
            else
#endif
              {
                /* Now initialize a SMART Flash block device and bind it
                 * to the MTD device.
                 */

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
                sprintf(partname, "p%d", partno);
                smart_initialize(CONFIG_MIKROE_FLASH_MINOR, mtd_part, partname);
#endif
              }

            /* Update the pointer to point to the next size in the list */

            while ((*ptr >= '0') && (*ptr <= '9'))
              {
                ptr++;
              }

            if (*ptr == ',')
              {
                ptr++;
              }

            /* Increment the part number */

            partno++;
          }
      }
#else /* CONFIG_MIKROE_FLASH_PART */

      /* Configure the device with no partition support */

      smart_initialize(CONFIG_MIKROE_FLASH_MINOR, mtd, NULL);

#endif /* CONFIG_MIKROE_FLASH_PART */
    }

  /* Create a RAM MTD device if configured */

#if defined(CONFIG_RAMMTD) && defined(CONFIG_MIKROE_RAMMTD)
  {
    uint8_t *start = (uint8_t *) kmm_malloc(CONFIG_MIKROE_RAMMTD_SIZE * 1024);
    mtd = rammtd_initialize(start, CONFIG_MIKROE_RAMMTD_SIZE * 1024);
    mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);

    /* Now initialize a SMART Flash block device and bind it to the MTD device */

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
    smart_initialize(CONFIG_MIKROE_RAMMTD_MINOR, mtd, NULL);
#endif
  }

#endif /* CONFIG_RAMMTD && CONFIG_MIKROE_RAMMTD */

#endif /* CONFIG_MTD */
#endif /* CONFIG_STM32_SPI3 */

  /* Create the SPI FLASH MTD instance */
  /* The M25Pxx is not a good media to implement a file system..
   * its block sizes are too large
   */

  /* Mount the SDIO-based MMC/SD block driver */

#ifdef NSH_HAVEMMCSD
  /* Bind the spi interface to the MMC/SD driver */

  syslog(LOG_INFO, "Bind SDIO to the MMC/SD driver, minor=%d\n",
         CONFIG_NSH_MMCSDMINOR);

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI to the MMC/SD driver: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "Successfully bound SPI to the MMC/SD driver\n");
    }
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize() starts a thread
   * will monitor for USB connection and disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start(0, NULL);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

#if defined(CONFIG_LCD_MIO283QT2) || defined(CONFIG_LCD_MIO283QT9A)
  /* Configure the TFT LCD module */

  syslog(LOG_INFO, "Initializing TFT LCD module\n");

  ret = board_lcd_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize TFT LCD module\n");
    }

#endif

  /* Configure the Audio sub-system if enabled and bind it to SPI 3 */

#ifdef CONFIG_AUDIO

  up_vs1053initialize(spi);

#endif

  return OK;
}
