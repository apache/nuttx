/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpc4088-devkit/src/lpc17_40_bringup.c
 *
 *   Copyright (C) 2013, 2016-2017 Gregory Nutt. All rights reserved.
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

#include <sys/mount.h>
#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/kthread.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/video/fb.h>
#include <nuttx/usb/usbhost.h>

#include "lpc17_40_gpio.h"
#include "lpc17_40_sdcard.h"
#include "lpc17_40_usbhost.h"
#include "lpc4088-devkit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define NSH_HAVE_MMCSD      1
#define NSH_HAVE_USBHOST    1
#define NSH_HAVE_USBHDEV    1

#undef NSH_HAVE_MMCSD_CD
#undef NSH_HAVE_MMCSD_CDINT

/* MMC/SD support */

#if !defined(CONFIG_LPC17_40_SDCARD) || !defined(CONFIG_MMCSD) && !defined(CONFIG_MMCD_SDIO)
#  undef NSH_HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef NSH_HAVE_MMCSD
#endif

/* MMC/SD support requires that an SPI support is enabled
 * and an SPI port is selected
 */

#ifdef NSH_HAVE_MMCSD
#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#     warning "Assuming slot MMC/SD slot 0"
#     define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
#endif

#ifdef NSH_HAVE_MMCSD
#  if !defined(CONFIG_NSH_MMCSDMINOR)
#     warning "Assuming /dev/mmcsd0"
#     define CONFIG_NSH_MMCSDMINOR 0
#  endif
#endif

/* The SD card detect (CD) signal is on the port expander U8, bit 4.
 * The port expander is not currently supported.  If support is added,
 * this section can be uncommented.
 */
#if 0
#  ifdef NSH_HAVE_MMCSD
#    ifdef CONFIG_MMCSD_HAVE_CARDDETECT
#      define NSH_HAVE_MMCSD_CD 1
#      ifdef CONFIG_LPC17_40_GPIOIRQ
#        define NSH_HAVE_MMCSD_CDINT 1
#      endif
#    endif
#  endif
#endif

/* USB Host */

#ifndef CONFIG_USBHOST
#  undef NSH_HAVE_USBHOST
#endif

#ifndef CONFIG_LPC17_40_USBHOST
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

/* USB Device */

#ifndef CONFIG_USBDEV
#  undef NSH_HAVE_USBDEV
#endif

#ifndef CONFIG_LPC17_40_USBDEV
#  undef NSH_HAVE_USBDEV
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef NSH_HAVE_USBHOST
static struct usbhost_connection_s *g_usbconn;
#endif
#ifdef NSH_HAVE_MMCSD
static FAR struct sdio_dev_s *g_sdiodev;
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
  int ret;

  /* Enable power to the SD card */

  lpc17_40_configgpio(GPIO_SD_PWR);

#ifdef NSH_HAVE_MMCSD_CD
  /* Configure the SD card detect GPIO */

  lpc17_40_configgpio(GPIO_SD_CD);

#endif

  /* First, get an instance of the SDIO interface */

  g_sdiodev = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
  if (!g_sdiodev)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDIO slot %d\n",
             CONFIG_NSH_MMCSDSLOTNO);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_sdiodev);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
             ret);
      return ret;
    }

  /* Check if there is a card in the slot and inform the SDCARD driver.  If
   * we do not support the card detect, then let's assume that there is
   * one.
   */

#ifdef NSH_HAVE_MMCSD_CD
  sdio_mediachange(g_sdiodev, !lpc17_40_gpioread(GPIO_SD_CD));
#else
  sdio_mediachange(g_sdiodev, true);
#endif
  return OK;
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

#ifdef NSH_HAVE_USBHOST
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
  g_usbconn = lpc17_40_usbhost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      syslog(LOG_INFO, "Start nsh_waiter\n");

      pid = kthread_create("usbhost", CONFIG_USBHOST_DEFPRIO,
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc4088_devkit_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library via boardctl()
 *
 ****************************************************************************/

int lpc4088_devkit_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system at the default location, /proc */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs: %d\n", ret);
    }
#endif

  /* Initialize SD Card */

  ret = nsh_sdinitialize();
  if (ret == OK)
    {
      /* Initialize USB host */

      ret = nsh_usbhostinitialize();
    }

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_INPUT_ADS7843E
  /* Initialize the touchscreen */

  ret = lpc4088_devkit_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lpc4088_devkit_tsc_setup failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_LPC4088_DEVKIT_DJOYSTICK
  /* Initialize and register the joystick driver */

  ret = lpc17_40_djoy_initialization();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the joystick driver: %d\n",
             ret);
      return ret;
    }
#endif

  return ret;
}
