/****************************************************************************
 * boards/arm/lpc17xx_40xx/lx_cpu/src/lpc17_40_bringup.c
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

#include <nuttx/arch.h>
#include <nuttx/kthread.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/video/fb.h>
#include <nuttx/usb/usbhost.h>

#include "lpc17_40_gpio.h"
#include "lpc17_40_sdcard.h"
#include "lpc17_40_usbhost.h"
#include "lx_cpu.h"

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

#if !defined(CONFIG_LPC17_40_SDCARD) || !defined(CONFIG_MMCSD) && \
    !defined(CONFIG_MMCD_SDIO)
#  undef NSH_HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef NSH_HAVE_MMCSD
#endif

/* MMC/SD support requires that an SPI support is enabled and an SPI port
 * is selected.
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

/* The SD card detect (CD) signal is on P0[13].  This signal is shared.  It
 * is also used for MOSI1 and USB_UP_LED.  The CD pin may be disconnected.
 * There is a jumper on board that enables the CD pin.
 */

#ifdef NSH_HAVE_MMCSD
#  ifdef CONFIG_MMCSD_HAVE_CARDDETECT
#    define NSH_HAVE_MMCSD_CD 1
#    ifdef CONFIG_LPC17_40_GPIOIRQ
#      define NSH_HAVE_MMCSD_CDINT 1
#    endif
#  endif
#endif

#if defined(NSH_HAVE_MMCSD_CD) && \
    (defined(CONFIG_LPC17_40_SSP1) || defined(CONFIG_LPC17_40_USBDEV))
#  warning "Use of SD Card Detect pin conflicts with SSP1 and/or USB device"
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
static struct sdio_dev_s *g_sdiodev;
#endif
#ifdef NSH_HAVE_MMCSD_CD
static bool g_sd_inserted;
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
 * Name: nsh_cdinterrupt
 *
 * Description:
 *   Card detect interrupt handler.
 *
 ****************************************************************************/

#ifdef NSH_HAVE_MMCSD_CDINT
static int nsh_cdinterrupt(int irq, void *context, void *arg)
{
  bool present;

  present = !lpc17_40_gpioread(GPIO_SD_CD);
  if (present != g_sd_inserted)
    {
      sdio_mediachange(g_sdiodev, present);
      g_sd_inserted = present;
    }

  return OK;
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

#ifdef NSH_HAVE_MMCSD_CD
  /* Configure the SD card detect GPIO */

  lpc17_40_configgpio(GPIO_SD_CD);

#ifdef NSH_HAVE_MMCSD_CDINT
  /* Attach an interrupt handler to get notifications when a card is
   * inserted or deleted.
   */

  irq_attach(LPC17_40_IRQ_P0p13, nsh_cdinterrupt, NULL);
  up_enable_irq(LPC17_40_IRQ_P0p13);
#endif
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
   * we do not support the card  detect, then let's assume that there is
   * one.
   */

#ifdef NSH_HAVE_MMCSD_CD
  g_sd_inserted = !lpc17_40_gpioread(GPIO_SD_CD);
  sdio_mediachange(g_sdiodev, g_sd_inserted);
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
             "ERROR: Failed to register the mass storage class: %d\n",
             ret);
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

      ret = kthread_create("usbhost", CONFIG_USBHOST_DEFPRIO,
                           CONFIG_USBHOST_STACKSIZE,
                           nsh_waiter, NULL);
      return ret < 0 ? -ENOEXEC : OK;
    }

  UNUSED(ret);
  return -ENODEV;
}
#else
#  define nsh_usbhostinitialize() (OK)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lx_cpu_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library via boardctl()
 *
 ****************************************************************************/

int lx_cpu_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system at the default location, /proc */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs: %d\n", ret);
    }
#endif

  /* Initialize SPI-based microSD */

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
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAN
  /* Initialize CAN and register the CAN driver. */

  ret = lx_cpu_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lx_xpu_can_setup failed: %d\n", ret);
    }
#endif

  return ret;
}
