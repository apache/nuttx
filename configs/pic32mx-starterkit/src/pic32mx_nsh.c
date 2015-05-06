/****************************************************************************
 * config/pic32mx-starterkit/src/pic32mx_nsh.c
 *
 *   Copyright (C) 2011, 2015 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>
#include <nuttx/usb/usbhost.h>

#include "pic32mx-internal.h"
#include "pic32mx-starterkit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Assume that we have MMC/SD, USB host (and USB device) */

#define NSH_HAVEMMCSD   1
#define NSH_HAVEUSBHOST 1

/* The PIC32 Ethernet Starter Kit does not have an SD slot on board.  If one
 * is added, then it must be specified by defining which SPI bus that it
 * is connected on.
 */

#ifndef CONFIG_PIC32MX_MMCSDSPIPORTNO
#  undef NSH_HAVEMMCSD
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

   /* Make sure that the NSH configuration uses the slot */

#  if !defined(CONFIG_NSH_MMCSDSLOTNO) || CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "The PIC32 Starter Kit has only one slot (0)"
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
  for (;;)
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_usbconn, &hport));
      syslog(LOG_INFO, "nsh_waiter: %s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          (void)CONN_ENUMERATE(g_usbconn, hport);
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
  FAR struct spi_dev_s *ssp;
  int ret;

  /* Get the SPI port */

  ssp = up_spiinitialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!ssp)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO);
      ret = -ENODEV;
      goto errout;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR,
                                CONFIG_NSH_MMCSDSLOTNO, ssp);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO,
             CONFIG_NSH_MMCSDSLOTNO, ret);
      goto errout;
    }

  syslog(LOG_INFO, "Successfuly bound SPI port %d to MMC/SD slot %d\n",
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
  int pid;
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
      syslog(LOG_ERR, "ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDCACM
  /* Register the CDC/ACM serial class */

  ret = usbhost_cdcacm_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the CDC/ACM serial class: %d\n", ret);
    }
#endif

  /* Then get an instance of the USB host interface */

  syslog(LOG_INFO, "Initialize USB host\n");
  g_usbconn = pic32_usbhost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      syslog(LOG_INFO, "Start nsh_waiter\n");

      pid = task_create("usbhost", CONFIG_USBHOST_DEFPRIO,
                        CONFIG_USBHOST_STACKSIZE,
                        (main_t)nsh_waiter, (FAR char * const *)NULL);
      return pid < 0 ? -ENOEXEC : OK;
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
  /* The PIC32 Starter Kit has no way to know when the USB is connected.  So
   * we will fake it and tell the USB driver that the USB is connected now.
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
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int board_app_initialize(void)
{
  int ret;

  /* Initialize SPI-based microSD */

  ret = nsh_sdinitialize();
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

  return ret;
}
