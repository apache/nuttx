/****************************************************************************
 * boards/arm/stm32/mikroe-stm32f4/src/stm32_appinit.c
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

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_MIKROE_FLASH_CONFIG_PART
#ifdef CONFIG_PLATFORM_CONFIGDATA
#  include <nuttx/mtd/configdata.h>
#endif
#endif

#ifdef CONFIG_AUDIO
#  include <nuttx/audio/audio.h>
#endif

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#include "stm32.h"
#include "mikroe-stm32f4.h"

#ifdef CONFIG_SENSORS_QENCODER
#include "board_qencoder.h"
#endif

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
#endif

/* Can't support USB device is USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
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

#  ifndef CONFIG_TESTING_SMART_NEBLOCKS
#    define CONFIG_TESTING_SMART_NEBLOCKS (22)
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
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

int board_app_initialize(uintptr_t arg)
{
#ifdef CONFIG_STM32_SPI3
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
#endif
  int ret = OK;

  /* Configure SPI-based devices */

#ifdef CONFIG_STM32_SPI3
  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port 3\n");
  spi = stm32_spibus_initialize(3);
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
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port 3 to the SPI"
                      " FLASH driver\n");
    }
  else
    {
      syslog(LOG_INFO, "Successfully bound SPI port 3 to the SPI"
                       " FLASH driver\n");

#ifdef CONFIG_MIKROE_FLASH_PART
        {
          int partno;
          int partsize;
          int partoffset;
          const char *partstring = CONFIG_MIKROE_FLASH_PART_LIST;
          const char *ptr;
          struct mtd_dev_s *mtd_part;
          char  partname[16];

          /* Now create a partition on the FLASH device */

          partno = 0;
          ptr = partstring;
          partoffset = 0;

          while (*ptr != '\0')
            {
              /* Get the partition size */

              partsize = atoi(ptr);
              mtd_part = mtd_partition(mtd, partoffset,
                                       (partsize >> 2) * 16);
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
                  snprintf(partname, sizeof(partname), "p%d", partno);
                  smart_initialize(CONFIG_MIKROE_FLASH_MINOR, mtd_part,
                                   partname);
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
      uint8_t *start =
          (uint8_t *) kmm_malloc(CONFIG_MIKROE_RAMMTD_SIZE * 1024);
      mtd = rammtd_initialize(start, CONFIG_MIKROE_RAMMTD_SIZE * 1024);
      mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);

      /* Now initialize a SMART Flash block device and bind it to the
       * MTD device
       */

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

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR,
                                CONFIG_NSH_MMCSDSLOTNO, spi);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI to the MMC/SD driver:"
                      " %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "Successfully bound SPI to the MMC/SD driver\n");
    }
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation. stm32_usbhost_initialize() starts a
   * thread will monitor for USB connection and disconnection events.
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

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT
  /* Initialize the touchscreen */

  ret = stm32_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_tsc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_LCD_MIO283QT2) || defined(CONFIG_LCD_MIO283QT9A)
  /* Configure the TFT LCD module */

  syslog(LOG_INFO, "Initializing TFT LCD module\n");

  ret = board_lcd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize TFT LCD module\n");
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER
  /* Initialize and register the qencoder driver */

  ret = board_qencoder_initialize(0, CONFIG_MIKROE_QETIMER);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_AUDIO
  /* Configure the Audio sub-system if enabled and bind it to SPI 3 */

  up_vs1053initialize(spi);
#endif

  return ret;
}
