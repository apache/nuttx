/****************************************************************************
 * boards/renesas/rx65n/rx65n-rsk2mb/src/rx65n_bringup.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/usb/usbhost.h>

#include "rx65n_usbhost.h"
#include "rx65n_rsk2mb.h"
#include <rx65n_definitions.h>
#ifdef CONFIG_BOARDCTL

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "rx65n_rtc.h"
#endif

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef HAVE_DTC_DRIVER
#  include "rx65n_dtc.h"
#endif

#ifdef HAVE_RSPI_DRIVER
#  include <nuttx/spi/spi_transfer.h>
#  include "rx65n_rspi.h"
#endif

#ifdef HAVE_RIIC_DRIVER
#  include <nuttx/i2c/i2c_master.h>
#  include "rx65n_riic.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define NSH_HAVE_USBHOST    1

/* USB Host */

#ifndef CONFIG_USBHOST
#  undef NSH_HAVE_USBHOST
#endif

#ifdef NSH_HAVE_USBHOST
#  ifndef CONFIG_USBHOST_DEFPRIO
#    define CONFIG_USBHOST_DEFPRIO 100
#  endif
#  ifndef CONFIG_USBHOST_STACKSIZE
#    ifdef CONFIG_USBHOST_HUB
#      define CONFIG_USBHOST_STACKSIZE 1536
#    else
#      define CONFIG_USBHOST_STACKSIZE 1024
#    endif
#  endif
#endif

#ifdef NSH_HAVE_USBHOST
static struct usbhost_connection_s *g_usbconn;
#endif

/****************************************************************************
 * Public Functions
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

      /* Did we just become connected? */

      if (hport->connected && hport->port == 0)
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

        printf ("USB Host MSC\n");
  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDCACM
  /* Register the CDC/ACM serial class */

  printf ("USB Host CDCACM\n");
  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the KBD class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  /* Register the HID KBD class */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      syslog(LOG_ERR,
        "ERROR: Failed to register the CDC/ACM serial class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HUB
  /* Initialize USB hub class support */

  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      uerr("ERROR: usbhost_hub_initialize failed: %d\n", ret);
    }
#endif

  /* Then get an instance of the USB host interface */

  g_usbconn = rx65n_usbhost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      syslog(LOG_INFO, "Start nsh_waiter\n");

      ret = kthread_create("usbhost", CONFIG_USBHOST_DEFPRIO,
                           CONFIG_USBHOST_STACKSIZE,
                           (main_t)nsh_waiter, (FAR char * const *)NULL);
      syslog(LOG_INFO, "USBHost: Created pid = %d\n", ret);
      return ret < 0 ? -ENOEXEC : OK;
    }

  return -ENODEV;
}
#else
#  define nsh_usbhostinitialize() (OK)
#endif

/****************************************************************************
 * Name: rx65n_rspi_initialize
 *
 * Description:
 *   Initialize and register the RSPI driver.
 *
 ****************************************************************************/
#ifdef CONFIG_RX65N_RSPI
static void rx65n_rspi_initialize(void)
{
  int ret;
#ifdef CONFIG_RX65N_RSPI0
  struct spi_dev_s *rspi0;
#endif
#ifdef CONFIG_RX65N_RSPI1
  struct spi_dev_s *rspi1;
#endif
#ifdef CONFIG_RX65N_RSPI2
  struct spi_dev_s *rspi2;
#endif

#ifdef CONFIG_RX65N_RSPI0
  rspi0 = rx65n_rspibus_initialize(0);
  if (!rspi0)
    {
      spierr("ERROR: [boot] FAILED to initialize SPI port 0\n");
    }

#ifdef CONFIG_SPI_DRIVER
  ret = spi_register(rspi0, 0);
  if (ret < 0)
    {
      spierr("ERROR: [boot] FAILED to register driver for channel 0\n");
    }
#endif

#endif

#ifdef CONFIG_RX65N_RSPI1
  rspi1 = rx65n_rspibus_initialize(1);
  if (!rspi1)
    {
      spierr("ERROR: [boot] FAILED to initialize SPI port 1\n");
    }

#ifdef CONFIG_SPI_DRIVER
  ret = spi_register(rspi1, 1);
  if (ret < 0)
    {
      spierr("ERROR: [boot] FAILED to register driver for channel 1\n");
    }
#endif

#endif

#ifdef CONFIG_RX65N_RSPI2
  rspi2 = rx65n_rspibus_initialize(2);
  if (!rspi2)
    {
      spierr("ERROR: [boot] FAILED to initialize SPI port 2\n");
    }

#ifdef CONFIG_SPI_DRIVER
  ret = spi_register(rspi2, 2);
  if (ret < 0)
    {
      spierr("ERROR: [boot] FAILED to register driver for channel 2\n");
    }
#endif

#endif
}
#endif

/****************************************************************************
 * Name: rtc_driver_initialize
 *
 * Description:
 *   Initialize and register the RTC driver.
 *
 ****************************************************************************/

#ifdef HAVE_RTC_DRIVER
static int rtc_driver_initialize(void)
{
  FAR struct rtc_lowerhalf_s *lower;
  int ret;

  /* Instantiate the rx65n lower-half RTC driver */

  lower = rx65n_rtc_lowerhalf();
  if (lower == NULL)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      ret = -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
        }
    }

  return ret;
}

#endif
/****************************************************************************
 * Name: rx65n_bringup
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

int rx65n_bringup(void)
{
#if defined (HAVE_RTC_DRIVER) || defined (CONFIG_FS_PROCFS)
  int ret;
#ifdef HAVE_RTC_DRIVER
  ret = rtc_driver_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rtc_driver_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS

  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n", ret);
    }

#endif
#endif

#ifdef CONFIG_RX65N_SBRAM
  /* Initialize standby RAM */

  rx65n_sbram_int();
#endif

#ifdef HAVE_DTC_DRIVER
  /* Initialize DTC */

  rx65n_dtc_initialize();
#endif

#ifdef CONFIG_RX65N_RSPI
  rx65n_rspi_initialize();
#endif

#if defined(CONFIG_USBHOST)
  ret = nsh_usbhostinitialize();
  printf ("USB Initialization done!!! with return value = %d\n", ret);
#endif

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE)
  /* Initialize CDCACM */

  syslog(LOG_INFO, "Initialize CDCACM device\n");

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize failed: %d\n", ret);
    }
#endif /* CONFIG_CDCACM & !CONFIG_CDCACM_CONSOLE */

#ifdef HAVE_RIIC_DRIVER
  FAR struct i2c_master_s *i2c;

  /* Get the I2C lower half instance */

#ifdef CONFIG_RX65N_RIIC0
  i2c = rx65n_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      i2cerr("ERROR: Initialization of RIIC Channel 0 failed: %d\n", ret);
    }
  else
    {
      /* Register the I2C character driver */

      ret = i2c_register(i2c, 0);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register RIIC device: %d\n", ret);
        }
    }

#endif
#ifdef CONFIG_RX65N_RIIC1
  i2c = rx65n_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      i2cerr("ERROR: Initialization of RIIC Channel 1 failed: %d\n", ret);
    }
  else
    {
      /* Register the I2C character driver */

      ret = i2c_register(i2c, 1);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register RIIC device: %d\n", ret);
        }
    }

#endif
#ifdef CONFIG_RX65N_RIIC2
  i2cerr("RIIC2 setting is not supported on RSK-2MB Board\n");
  i2cerr("It is used for USB port and cannot be configured\n");
#endif
#endif
  return OK;
}

#if defined (CONFIG_ARCH_HAVE_LEDS) && (CONFIG_ARCH_LEDS)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled1_on
 *
 * Description:
 *   Turns on LED 0
 *
 ****************************************************************************/

void board_autoled1_on(int led)
{
  LED0 = LED_ON;
}

/****************************************************************************
 * Name: board_autoled2_on
 *
 * Description:
 *   Turns on LED 1
 *
 ****************************************************************************/

void board_autoled2_on(int led)
{
  LED1 = LED_ON;
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Turns on LED 0 & LED 1
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  LED0 = LED_ON;
  LED1 = LED_ON;
}

/****************************************************************************
 * Name: board_autoled1_off
 *
 * Description:
 *   Turns off LED 0
 *
 ****************************************************************************/

void board_autoled1_off(int led)
{
  LED0 = LED_OFF;
}

/****************************************************************************
 * Name: board_autoled2_off
 *
 * Description:
 *   Turns off LED 1
 *
 ****************************************************************************/

void board_autoled2_off(int led)
{
  LED1 = LED_OFF;
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Turns off LED 0 & LED 1
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  LED0 = LED_OFF;
  LED1 = LED_OFF;
}

#endif
#endif
