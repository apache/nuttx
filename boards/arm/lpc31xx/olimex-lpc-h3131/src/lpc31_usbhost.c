/****************************************************************************
 * boards/arm/lpc31xx/olimex-lpc-h3131/src/lpc31_usbhost.c
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
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "lpc31.h"
#include "lpc_h3131.h"

#ifdef HAVE_USBHOST

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

/* Overcurrent interrupt handler */

#if 0 /* Not yet implemented */
static xcpt_t g_ochandler;
#endif

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
 * Name: lpc31_usbhost_bootinitialize
 *
 * Description:
 *   Called from lpc31_boardinitialize very early in inialization to setup
 *   USB host-related GPIO pins for the LPC-H3131 board.
 *
 *   SIGNAL      GPIO
 *   ----------- -------
 *   #OTG_PWR_E  GPIO19
 *   #OTG_OVRCR  GPIO20
 *
 ****************************************************************************/

void weak_function lpc31_usbhost_bootinitialize(void)
{
  /* Configure output pin to drive VBUS power (initial state: power off) */

  gpio_outputhigh(LPC31_IOCONFIG_GPIO, GPIO_NOTG_PWR_E);

  /* Configure input pin to detect overrcurrent errors */

  gpio_configinput(LPC31_IOCONFIG_GPIO, GPIO_NOTG_OVRCR);

  /* Configure to receive interrupts on the overrcurrent input pin */

#warning Missing logic
}

/****************************************************************************
 * Name: lpc31_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

int lpc31_usbhost_initialize(void)
{
  int ret;

  /* First, register all of the class drivers needed to support the drivers
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
  /* Register theUSB host Mass Storage Class */

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDCACM
  /* Register the CDC/ACM serial class */

  ret = usbhost_cdcacm_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the CDC/ACM serial class\n");
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  /* Register the USB host HID keyboard class driver */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the KBD class\n");
    }
#endif

  /* Then get an instance of the USB EHCI interface. */

  g_ehciconn = lpc31_ehci_initialize(0);
  if (!g_ehciconn)
    {
      uerr("ERROR: lpc31_ehci_initialize failed\n");
      return -ENODEV;
    }

  /* Start a thread to handle device connection. */

  ret = kthread_create("EHCI Monitor", CONFIG_USBHOST_DEFPRIO, i
                       CONFIG_USBHOST_STACKSIZE,
                       (main_t)ehci_waiter, (char * const *)NULL);
  if (ret < 0)
    {
      uerr("ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lpc31_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.
 *   This function must be provided by each platform that implements the
 *   OHCI or EHCI host interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface.
 *     Since the LPC31 has only a downstream port, zero is the only
 *     possible value for this parameter.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc31_usbhost_vbusdrive(int rhport, bool enable)
{
  uinfo("RHPort%d: enable=%d\n", rhport + 1, enable);

  /* The LPC3131 has only a single root hub port */

  if (rhport == 0)
    {
      /* Then enable or disable VBUS power */

      if (enable)
        {
          /* Enable the Power Switch by driving the enable pin low */

          gpio_outputlow(LPC31_IOCONFIG_GPIO, GPIO_NOTG_PWR_E);
        }
      else
        {
          /* Disable the Power Switch by driving the enable pin high */

          gpio_outputhigh(LPC31_IOCONFIG_GPIO, GPIO_NOTG_PWR_E);
        }
    }
}

/****************************************************************************
 * Name: lpc31_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an overcurrent
 *   condition is detected.
 *
 * Input Parameters:
 *   handler - New overcurrent interrupt handler
 *   arg     - The argument that will accompany the interrupt
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#if 0 /* Not ready yet */
int lpc31_setup_overcurrent(xcpt_t handler, void *arg)
{
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Configure the interrupt */

#warning Missing logic

  leave_critical_section(flags);
  return oldhandler;
}
#endif /* 0 */
#endif /* HAVE_USBHOST */
