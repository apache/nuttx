/****************************************************************************
 * config/olimex-lpc1766stk/src/lpc17_appinit.c
 *
 *   Copyright (C) 2010, 2013-2016, 2018 Gregory Nutt. All rights reserved.
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

#include <nuttx/kthread.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>
#include <nuttx/usb/usbhost.h>

#include "lpc17_ssp.h"
#include "lpc17_gpio.h"
#include "lpc17_usbhost.h"

#include "lpc1766stk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_MMCSD  1
#define HAVE_USBHOST  1

/* MMC/SD is on SSP port 1.  There is only a single slot, slot 0 */

#ifdef CONFIG_NSH_ARCHINIT
#  if !defined(CONFIG_NSH_MMCSDSPIPORTNO) || CONFIG_NSH_MMCSDSPIPORTNO != 1
#    undef CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO 1
#  endif

#  if !defined(CONFIG_NSH_MMCSDSLOTNO) || CONFIG_NSH_MMCSDSLOTNO != 0
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif

#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif

#else
#  undef  CONFIG_NSH_MMCSDSPIPORTNO
#  define CONFIG_NSH_MMCSDSPIPORTNO 1
#  undef  CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#  undef  CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

/* Can't support MMC/SD is SSP1 is not enabled */

#ifndef CONFIG_LPC17_SSP1
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_MMCSD
#endif

/* USB Host */

#ifdef CONFIG_USBHOST
#  ifndef CONFIG_LPC17_USBHOST
#    error "CONFIG_LPC17_USBHOST is not selected"
#  endif
#endif

#ifdef CONFIG_LPC17_USBHOST
#  ifndef CONFIG_USBHOST
#    warning "CONFIG_USBHOST is not selected"
#  endif
#endif

#if !defined(CONFIG_USBHOST) || !defined(CONFIG_LPC17_USBHOST)
#  undef HAVE_USBHOST
#endif

#ifdef HAVE_USBHOST
#  ifndef CONFIG_LPC1766STK_USBHOST_PRIO
#    define CONFIG_LPC1766STK_USBHOST_PRIO 50
#  endif
#  ifndef CONFIG_LPC1766STK_USBHOST_STACKSIZE
#    define CONFIG_LPC1766STK_USBHOST_STACKSIZE 1024
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_USBHOST
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

#ifdef HAVE_USBHOST
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

#ifdef HAVE_MMCSD
static int nsh_sdinitialize(void)
{
  FAR struct spi_dev_s *ssp;
  int ret;

  /* Enable power to the SD/MMC via a GPIO. LOW enables SD/MMC. */

  lpc17_gpiowrite(LPC1766STK_MMC_PWR, false);

  /* Get the SSP port.   MMC/SD is on SSP port 1.  */

  ssp = lpc17_sspbus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!ssp)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SSP port %d\n",
              CONFIG_NSH_MMCSDSPIPORTNO);
      ret = -ENODEV;
      goto errout;
    }

  syslog(LOG_INFO, "Successfully initialized SSP port %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO);

  /* Bind the SSP port to the slot */

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR,
                               CONFIG_NSH_MMCSDSLOTNO, ssp);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SSP port %d to MMC/SD slot %d: %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO,
             CONFIG_NSH_MMCSDSLOTNO, ret);
      goto errout;
    }

  syslog(LOG_INFO, "Successfully bound SSP port %d to MMC/SD slot %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO,
         CONFIG_NSH_MMCSDSLOTNO);
  return OK;

  /* Disable power to the SD/MMC via a GPIO. HIGH disables SD/MMC. */

errout:
  lpc17_gpiowrite(LPC1766STK_MMC_PWR, true);
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

#ifdef HAVE_USBHOST
static int nsh_usbhostinitialize(void)
{
  int pid;
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  syslog(LOG_INFO, "Register class drivers\n");

#ifdef CONFIG_USBHOST_HUB
  /* Initialize USB hub support */

  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: usbhost_hub_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_MSC
  /* Initialize mass storage support */

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

  UNUSED(ret);

  /* Then get an instance of the USB host interface */

  syslog(LOG_INFO, "Initialize USB host\n");
  g_usbconn = lpc17_usbhost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      syslog(LOG_ERR, "ERROR: Start nsh_waiter\n");

      pid = kthread_create("usbhost", CONFIG_LPC1766STK_USBHOST_PRIO,
                           CONFIG_LPC1766STK_USBHOST_STACKSIZE,
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
 *         between the board-specific initialization logic and the
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

int board_app_initialize(uintptr_t arg)
{
  int ret;

  /* Initialize SPI-based microSD */

  ret = nsh_sdinitialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize SPI-based SD card: %d\n",
             ret);
    }

  /* Initialize USB host */

  ret = nsh_usbhostinitialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
    }

#ifdef CONFIG_USBHOST_HIDMOUSE
  /* Initialize the HID Mouse class */

  ret = lpc1766stk_hidmouse_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lpc1766stk_hidmouse_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAN
  /* Initialize CAN and register the CAN driver. */

  ret = lpc1766stk_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lpc1766stk_can_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

  return ret;
}
