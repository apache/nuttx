/****************************************************************************
 * boards/mips/pic32mx/pic32mx7mmb/src/pic32_bringup.c
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

#include <nuttx/kthread.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>
#include <nuttx/usb/usbhost.h>

#include "pic32mx.h"
#include "pic32mx7mmb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Assume that we have MMC/SD, USB host (and USB device) */

#define NSH_HAVEMMCSD   1
#define NSH_HAVEUSBHOST 1

/* The Mikroelektronika PIC32MX7 MMB has one SD slot on board,
 * connected to SPI 1.
 */

#ifndef CONFIG_PIC32MX_MMCSDSPIPORTNO
#  define CONFIG_PIC32MX_MMCSDSPIPORTNO 1
#endif

/* Make sure that the configuration will support the SD card */

#ifdef NSH_HAVEMMCSD

/* Make sure that the NSH configuration uses the correct SPI */

#  if !defined(CONFIG_NSH_MMCSDSPIPORTNO)
#    define CONFIG_NSH_MMCSDSPIPORTNO CONFIG_PIC32MX_MMCSDSPIPORTNO
#  elif CONFIG_NSH_MMCSDSPIPORTNO != CONFIG_PIC32MX_MMCSDSPIPORTNO
#    warning "CONFIG_PIC32MX_MMCSDSPIPORTNO does not match CONFIG_NSH_MMCSDSPIPORTNO"
#    undef CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO CONFIG_PIC32MX_MMCSDSPIPORTNO
#  endif

/* Make sure that the NSH configuration uses slot 0 (there is only one
 * SD slot on the Mikroelektronica PIC32MX7 MMB).
 */

#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  elif CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "The Mikroelektronika PIC32MX7 MMB has only one slot (0)"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif

/* Make sure that the correct SPI is enabled in the configuration */

#  if CONFIG_PIC32MX_MMCSDSPIPORTNO == 1 && !defined(CONFIG_PIC32MX_SPI1)
#    warning "CONFIG_PIC32MX_SPI1 is not enabled"
#    undef NSH_HAVEMMCSD
#  elif CONFIG_PIC32MX_MMCSDSPIPORTNO == 2 && !defined(CONFIG_PIC32MX_SPI2)
#    warning "CONFIG_PIC32MX_SPI2 is not enabled"
#    undef NSH_HAVEMMCSD
#  elif CONFIG_PIC32MX_MMCSDSPIPORTNO == 3 && !defined(CONFIG_PIC32MX_SPI3)
#    warning "CONFIG_PIC32MX_SPI3 is not enabled"
#    undef NSH_HAVEMMCSD
#  elif CONFIG_PIC32MX_MMCSDSPIPORTNO == 4 && !defined(CONFIG_PIC32MX_SPI4)
#    warning "CONFIG_PIC32MX_SPI4 is not enabled"
#    undef NSH_HAVEMMCSD
#  endif
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef NSH_HAVEMMCSD
#endif

/* Select /dev/mmcsd0 if no other minor number is provided */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

/* USB Host */

#ifdef CONFIG_USBHOST
#  ifndef CONFIG_PIC32MX_USBHOST
#    error "CONFIG_PIC32MX_USBHOST is not selected"
#    undef NSH_HAVEUSBHOST
#  endif
#endif

#ifdef CONFIG_PIC32MX_USBHOST
#  ifndef CONFIG_USBHOST
#    warning "CONFIG_USBHOST is not selected"
#    undef NSH_HAVEUSBHOST
#  endif
#endif

#if !defined(CONFIG_USBHOST) || !defined(CONFIG_PIC32MX_USBHOST)
#  undef NSH_HAVEUSBHOST
#endif

#ifdef NSH_HAVEUSBHOST
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

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef NSH_HAVEUSBHOST
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

#ifdef NSH_HAVEUSBHOST
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

#ifdef NSH_HAVEMMCSD
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

  syslog(LOG_INFO,
         "Successfully bound SPI port %d to MMC/SD slot %d\n",
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
 *   Initialize SPI-based microSD.
 *
 ****************************************************************************/

#ifdef NSH_HAVEUSBHOST
static int nsh_usbhostinitialize(void)
{
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  syslog(LOG_INFO, "Register class drivers\n");

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB host Mass Storage Class */

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
  /* The Mikroelektronika PIC32MX7 MMB has no way to know when the USB is
   * connected.  So we will fake it and tell the USB driver that the USB is
   * connected now.
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
 * Name: pic32mx_bringup
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

int pic32mx_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }

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

#ifdef CONFIG_INPUT
  /* Initialize the touchscreen */

  ret = pic32mx_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pic32mx_tsc_setup failed: %d\n", ret);
    }
#endif

  return ret;
}
