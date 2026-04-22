/****************************************************************************
 * boards/arm64/imx9/imx95-a55-evk/src/imx9_usbhost.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/ehci.h>

#include "imx9_ehci.h"
#include "hardware/imx9_memorymap.h"
#include "hardware/imx9_usbotg.h"
#include "imx95-evk.h"

#include <nuttx/usb/ehci.h>
#include <arch/board/board.h>

#if defined(CONFIG_IMX9_USBHOST) && defined(CONFIG_USBHOST)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_USBHOST_DEFPRIO
#  define CONFIG_USBHOST_DEFPRIO 50
#endif

#ifndef CONFIG_USBHOST_STACKSIZE
#  ifdef CONFIG_USBHOST_HUB
#    define CONFIG_USBHOST_STACKSIZE 1536
#  else
#    define CONFIG_USBHOST_STACKSIZE 1024
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Retained device driver handle */

static struct usbhost_connection_s *g_ehciconn;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ehci_waiter
 *
 * Description:
 *   Wait for USB devices to be connected to the EHCI root hub.
 *
 ****************************************************************************/

static int ehci_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;

  uinfo("ehci_waiter:  Running\n");
  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_ehciconn, &hport));
      syslog(LOG_INFO, "ehci_waiter: %s\n",
             hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          CONN_ENUMERATE(g_ehciconn, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.  This function will start a thread that will monitor
 *   for device connection/disconnection events.
 *
 ****************************************************************************/

int imx9_usbhost_initialize(void)
{
  int ret;
  uint32_t regval;

  /* Configure USBNC: disable overcurrent, set VBUS power polarity */

  regval = *(volatile uint32_t *)IMX9_USBNC_OTG2_BASE;
  regval |= USBNC_CTRL1_OVER_CUR_DIS;
  regval |= USBNC_CTRL1_PWR_POL;
  *(volatile uint32_t *)IMX9_USBNC_OTG2_BASE = regval;

  /* Register all of the class drivers needed to support the drivers
   * that we care about
   */

#ifdef CONFIG_USBHOST_HUB
  /* Initialize USB hub support */

  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: usbhost_hub_initialize failed: %d\n", ret);
    }
#endif

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

#ifdef CONFIG_USBHOST_HIDMOUSE
  /* Register the USB host HID mouse class driver */

  ret = usbhost_mouse_init();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the mouse class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  /* Register the USB host HID keyboard class driver */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the KBD class: %d\n", ret);
    }
#endif

  /* Then get an instance of the USB EHCI interface. */

  g_ehciconn = imx9_ehci_initialize(0);
  if (!g_ehciconn)
    {
      syslog(LOG_ERR, "ERROR: imx9_ehci_initialize failed\n");
      return -ENODEV;
    }

  /* Start a thread to handle device connection. */

  ret = kthread_create("EHCI Monitor", CONFIG_USBHOST_DEFPRIO,
                       CONFIG_USBHOST_STACKSIZE,
                       ehci_waiter, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: imx9_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.
 *
 ****************************************************************************/

#define HCOR_BASE (IMX9_USB_OTG2_BASE + 0x140)

void imx9_usbhost_vbusdrive(int rhport, bool enable)
{
  volatile uint32_t *portsc;
  uint32_t regval;

  uinfo("RHPort%d: enable=%d\n", rhport + 1, enable);

  /* The IMX9 has only a single root hub port */

  if (rhport == 0)
    {
      portsc = (volatile uint32_t *)(HCOR_BASE + 0x44 + rhport * 4);

      /* Then enable or disable VBUS power */

      regval = *portsc;
      regval &= ~EHCI_PORTSC_PP;
      if (enable)
        {
          regval |= EHCI_PORTSC_PP;
        }

      *portsc = regval;
    }
}

/****************************************************************************
 * Name: imx9_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an overcurrent
 *   condition is detected.
 *
 ****************************************************************************/

int imx9_setup_overcurrent(xcpt_t handler, void *arg)
{
  /* Not implemented yet for imx95 */

  return OK;
}

#endif /* CONFIG_IMX9_USBHOST && CONFIG_USBHOST */
