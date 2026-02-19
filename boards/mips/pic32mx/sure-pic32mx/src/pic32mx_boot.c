/****************************************************************************
 * boards/mips/pic32mx/sure-pic32mx/src/pic32mx_boot.c
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

#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>
#include <assert.h>

#include <arch/board/board.h>
#include <nuttx/kthread.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>
#include <nuttx/lcd/hd4478ou.h>
#include <nuttx/usb/usbhost.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include "mips_internal.h"
#include "pic32mx.h"
#include "sure-pic32mx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#define NSH_HAVE_MMCSD      1
#define NSH_HAVE_USBHOST    1
#define NSH_HAVE_USBMONITOR 1

/* Can't support MMC/SD if SPI2 is not enabled */

#ifndef CONFIG_PIC32MX_SPI2
#  undef NSH_HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef NSH_HAVE_MMCSD
#endif

/* MMC/SD configuration */

#ifdef NSH_HAVE_MMCSD
#    undef  CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO 2
#    undef  CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#    undef  CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#endif

/* USB Host */

#ifdef CONFIG_USBHOST
#  ifndef CONFIG_PIC32MX_USBHOST
#    warning "CONFIG_PIC32MX_USBHOST is not selected"
#    undef CONFIG_USBHOST
#  endif
#endif

#ifdef CONFIG_PIC32MX_USBHOST
#  ifndef CONFIG_USBHOST
#    warning "CONFIG_USBHOST is not selected"
#    undef CONFIG_PIC32MX_USBHOST
#  endif
#endif

#if !defined(CONFIG_USBHOST) || !defined(CONFIG_PIC32MX_USBHOST)
#  undef NSH_HAVE_USBHOST
#endif

#ifdef NSH_HAVE_USBHOST
#  ifndef CONFIG_USBHOST_DEFPRIO
#    define CONFIG_USBHOST_DEFPRIO 50
#  endif
#  ifndef CONFIG_USBHOST_STACKSIZE
#    ifdef CONFIG_USBHOST_HUB
#      define CONFIG_USBHOST_STACKSIZE 1536
#    else
#      define CONFIG_USBHOST_STACKSIZE 1024
#    endif
#  endif
#endif

/* USB Monitor */

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef NSH_HAVE_USBMONITOR
#endif

#ifndef NSH_HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef NSH_HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef NSH_HAVE_USBMONITOR
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef NSH_HAVE_USBHOST
static struct usbhost_connection_s *g_usbconn;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

#ifdef NSH_HAVE_USBHOST
static int nsh_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;

  syslog(LOG_INFO, "nsh_waiter: Running\n");
  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_usbconn, &hport));
      syslog(LOG_INFO, "nsh_waiter: %s\n",
             hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          CONN_ENUMERATE(g_usbconn, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}
#endif

/****************************************************************************
 * Name: nsh_sdinitialize
 *
 * Description:
 *   Initialize SPI-based microSD.
 *
 ****************************************************************************/

#ifdef NSH_HAVE_MMCSD
static int nsh_sdinitialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  spi = pic32mx_spibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO);
      ret = -ENODEV;
      goto errout;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO);

  /* The SPI should be in 8-bit (default) and mode2: CKP=1, CKE=0.
   * The MMC/SD driver will control the SPI frequency.  WARNING:
   * this is not the right way to do this... this should be done
   * the MMC/SD driver:  Other device on SPI1 may need other mode
   * settings.
   */

  SPI_SETMODE(spi, SPIDEV_MODE2);

  /* Bind the SPI port to the slot */

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR,
                               CONFIG_NSH_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO,
             CONFIG_NSH_MMCSDSLOTNO, ret);
      goto errout;
    }

  syslog(LOG_INFO, "Successfully bound SPI port %d to MMC/SD slot %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO,
         CONFIG_NSH_MMCSDSLOTNO);
  return OK;

errout:
  return ret;
}
#else
#  define nsh_sdinitialize() (OK)
#endif

/****************************************************************************
 * Name: nsh_usbhostinitialize
 *
 * Description:
 *   Initialize USB Host
 *
 ****************************************************************************/

#ifdef NSH_HAVE_USBHOST
static int nsh_usbhostinitialize(void)
{
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  syslog(LOG_INFO, "Register class drivers\n");

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB mass storage class class */

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDCACM
  /* Register the CDC/ACM serial class */

  ret = usbhost_cdcacm_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the CDC/ACM serial class: %d\n",
             ret);
    }
#endif

  /* Then get an instance of the USB host interface */

  syslog(LOG_INFO, "Initialize USB host\n");

  g_usbconn = pic32_usbhost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      syslog(LOG_INFO, "Start nsh_waiter\n");

      ret = kthread_create("usbhost", CONFIG_USBHOST_DEFPRIO,
                           CONFIG_USBHOST_STACKSIZE,
                           nsh_waiter, NULL);
      return ret < 0 ? -ENOEXEC : OK;
    }

  return -ENODEV;
}
#else
#  define nsh_usbhostinitialize() (OK)
#endif

/****************************************************************************
 * Name: nsh_usbdevinitialize
 *
 * Description:
 *   Initialize SPI-based microSD.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV
static int nsh_usbdevinitialize(void)
{
  /* The Sure board has no way to know when the USB is connected.  So we
   * will fake it and tell the USB driver that the USB is connected now.
   */

  pic32mx_usbattach();
  return OK;
}
#else
#  define nsh_usbdevinitialize() (OK)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_boardinitialize
 *
 * Description:
 *   All PIC32MX architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void pic32mx_boardinitialize(void)
{
  /* Configure SPI chip selects if 1) at least one SPI is enabled, and 2)
   * the weak function pic32mx_spidev_initialize() has been brought into
   * the link.
   */

#if defined(CONFIG_PIC32MX_SPI2)
  if (pic32mx_spidev_initialize)
    {
      pic32mx_spidev_initialize();
    }
#endif

  /* Configure the mini-B PHY on the Sure PIC32MX board for the USB device */

#if defined(CONFIG_PIC32MX_USBDEV)
  if (pic32mx_usbdevinitialize)
    {
      pic32mx_usbdevinitialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  pic32mx_led_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

  /* Initialize the LCD1602 and register the device as /dev/lcd1602 */

#ifdef CONFIG_LCD_LCD1602
  ret = up_lcd1602_initialize();
  if (ret == OK)
#endif
    {
      /* Initialize SPI-based microSD */

      ret = nsh_sdinitialize();
    }

  if (ret == OK)
    {
      /* Initialize USB host */

      ret = nsh_usbhostinitialize();
    }

  if (ret == OK)
    {
      /* Initialize USB device */

      ret = nsh_usbdevinitialize();
    }

#ifdef NSH_HAVE_USBMONITOR
  if (ret == OK)
    {
      /* Start the USB Monitor */

      ret = usbmonitor_start();
    }
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
