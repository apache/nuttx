/****************************************************************************
 * boards/arm/sama5/jti-toucan2/src/sam_usb.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
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

#include "arm_arch.h"
#include "sam_pio.h"
#include "sam_usbhost.h"
#include "hardware/sam_ohci.h"
#include "jti-toucan2.h"

#if defined(CONFIG_SAMA5_UHPHS) || defined(CONFIG_SAMA5_UDPHS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SAMA5D27_JTI_TOUCAN2_USBHOST_PRIO
#  define CONFIG_SAMA5D27_JTI_TOUCAN2_USBHOST_PRIO 50
#endif

#ifndef CONFIG_SAMA5D27_JTI_TOUCAN2_USBHOST_STACKSIZE
#  define CONFIG_SAMA5D27_JTI_TOUCAN2_USBHOST_STACKSIZE 1024
#endif

#ifdef HAVE_USBDEV
#  undef CONFIG_SAMA5_UHPHS_RHPORT1
#  undef CONFIG_SAMA5_UHPHS_RHPORT1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Retained device driver handles */

#ifdef CONFIG_SAMA5_OHCI
static struct usbhost_connection_s *g_ohciconn;
#endif
#ifdef CONFIG_SAMA5_EHCI
static struct usbhost_connection_s *g_ehciconn;
#endif

/* Overcurrent interrupt handler */

#if defined(HAVE_USBHOST) && defined(CONFIG_SAMA5_PIOD_IRQ)
static xcpt_t g_ochandler;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected to either the OHCI or EHCI hub.
 *
 ****************************************************************************/
// need to wait in FUSB302 driver to do an auto-switch of types, I think
#ifdef HAVE_USBHOST
#ifdef CONFIG_DEBUG_USB
static int usbhost_waiter(struct usbhost_connection_s *dev,
                          const char *hcistr)
#else
static int usbhost_waiter(struct usbhost_connection_s *dev)
#endif
{
  struct usbhost_hubport_s *hport;

  uinfo("Running\n");
  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(dev, &hport));
      uinfo("%s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          CONN_ENUMERATE(dev, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}
#endif

/****************************************************************************
 * Name: ohci_waiter
 *
 * Description:
 *   Wait for USB devices to be connected to the OHCI hub.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_OHCI
static int ohci_waiter(int argc, char *argv[])
{
#ifdef CONFIG_DEBUG_USB
  return usbhost_waiter(g_ohciconn, "OHCI");
#else
  return usbhost_waiter(g_ohciconn);
#endif
}
#endif

/****************************************************************************
 * Name: ehci_waiter
 *
 * Description:
 *   Wait for USB devices to be connected to the EHCI hub.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_EHCI
static int ehci_waiter(int argc, char *argv[])
{
#ifdef CONFIG_DEBUG_USB
  return usbhost_waiter(g_ehciconn, "EHCI");
#else
  return usbhost_waiter(g_ehciconn);
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called from sam_usbinitialize very early in inialization to setup
 *   USB-related GPIO pins for the JTI Toucan2 board.
 *
 * USB Ports
 *   The SAMA5D27 features:
 *   
 *   - One USB high-speed device port (UDPHS) and one USB high-speed host port or two USB high-speed host ports (UHPHS)
 *   - One One USB high-speed host port with a High-Speed Inter-Chip (HSIC) interface 
 *
 *   The JTI Toucan2 board has a single USB C connector on Port A, which can be used as:
 *	 - High Speed host or
 *   - High Speed device
 *
 *	There is an FUSB302 USB Type-C controller 
 *  A high side switch allows 5V/500mA power to be switched to the USB connector as required. 
 
 ****************************************************************************/

void weak_function sam_usbinitialize(void)
{
#ifdef HAVE_USBDEV
  /* Configure Port A to support the USB device function */

#endif

#ifdef HAVE_USBHOST
#  ifdef CONFIG_SAMA5_UHPHS_RHPORT1
#  endif

#  ifdef CONFIG_SAMA5_UHPHS_RHPORT2
#  endif
#endif /* HAVE_USBHOST */
}

/****************************************************************************
 * Name: sam_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST
int sam_usbhost_initialize(void)
{
  pid_t pid;
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about
   */

#ifdef CONFIG_USBHOST_HUB
  /* Initialize USB hub class support */

  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      uerr("ERROR: usbhost_hub_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB host Mass Storage Class */

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
      uerr("ERROR: Failed to register the CDC/ACM serial class: %d\n", ret);
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

  /* Then get an instance of the USB host interface. */

#ifdef CONFIG_SAMA5_OHCI
  /* Get an instance of the USB OHCI interface */

  g_ohciconn = sam_ohci_initialize(0);
  if (!g_ohciconn)
    {
      uerr("ERROR: sam_ohci_initialize failed\n");
      return -ENODEV;
    }

  /* Start a thread to handle device connection. */

  pid = kthread_create("OHCI Monitor",
                       CONFIG_SAMA5D27_JTI_TOUCAN2_USBHOST_PRIO,
                       CONFIG_SAMA5D27_JTI_TOUCAN2_USBHOST_STACKSIZE,
                       (main_t)ohci_waiter, (FAR char * const *)NULL);
  if (pid < 0)
    {
      uerr("ERROR: Failed to create ohci_waiter task: %d\n", ret);
      return -ENODEV;
    }
#endif

#ifdef CONFIG_SAMA5_EHCI
  /* Get an instance of the USB EHCI interface */

  g_ehciconn = sam_ehci_initialize(0);
  if (!g_ehciconn)
    {
      uerr("ERROR: sam_ehci_initialize failed\n");
      return -ENODEV;
    }

  /* Start a thread to handle device connection. */

  pid = kthread_create("EHCI Monitor",
                       CONFIG_SAMA5D27_JTI_TOUCAN2_USBHOST_PRIO,
                       CONFIG_SAMA5D27_JTI_TOUCAN2_USBHOST_STACKSIZE,
                       (main_t)ehci_waiter, (FAR char * const *)NULL);
  if (pid < 0)
    {
      uerr("ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: sam_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.
 *   This function must be provided by each platform that implements the
 *   OHCI or EHCI host interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface.
 *            See SAM_RHPORT_* definitions above.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST
void sam_usbhost_vbusdrive(int rhport, bool enable)
{
  pio_pinset_t pinset = 0;

  uinfo("RHPort%d: enable=%d\n", rhport + 1, enable);

  /* Pick the PIO configuration associated with the selected root hub port */

  switch (rhport)
    {
    case SAM_RHPORT1:
#if !defined(CONFIG_SAMA5_UHPHS_RHPORT1)
      uerr("ERROR: RHPort1 is not available in this configuration\n");
      return;

#elif !defined(PIO_USBA_VBUS_ENABLE)
      /* SAMA5D2-XULT has no port A VBUS enable */

      uerr("ERROR: RHPort1 has no VBUS enable\n");
      return;
#else
      break;
#endif

    case SAM_RHPORT2:
#ifndef CONFIG_SAMA5_UHPHS_RHPORT2
      uerr("ERROR: RHPort2 is not available in this configuration\n");
      return;
#else
      break;
#endif

    default:
      uerr("ERROR: RHPort%d is not supported\n", rhport + 1);
      return;
    }

  /* Then enable or disable VBUS power (active high) */

  if (enable)
    {
      /* Enable the Power Switch by driving the enable pin high */

      sam_piowrite(pinset, true);
    }
  else
    {
      /* Disable the Power Switch by driving the enable pin low */

      sam_piowrite(pinset, false);
    }
}
#endif

/****************************************************************************
 * Name: sam_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an overcurrent condition
 *   is detected on port B or C.
 *
 *   REVISIT: Since this is a common signal, we will need to come up with
 *   some way to inform both EHCI and OHCI drivers when this error occurs.
 *
 * Input Parameters:
 *   handler - New overcurrent interrupt handler
 *
 * Returned Value:
 *   Old overcurrent interrupt handler
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST
xcpt_t sam_setup_overcurrent(xcpt_t handler)
{
#if defined(CONFIG_SAMA5_PIOD_IRQ) && (defined(CONFIG_SAMA5_UHPHS_RHPORT2) || \
    defined(CONFIG_SAMA5_UHPHS_RHPORT3))

  xcpt_t oldhandler;
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Get the old interrupt handler and save the new one */

  oldhandler  = g_ochandler;
  g_ochandler = handler;

  /* Configure the interrupt */

  sam_pioirq(PIO_USBBC_VBUS_OVERCURRENT);
  irq_attach(IRQ_USBBC_VBUS_OVERCURRENT, handler, NULL);
  sam_pioirqenable(IRQ_USBBC_VBUS_OVERCURRENT);

  /* Return the old handler (so that it can be restored) */

  leave_critical_section(flags);
  return oldhandler;

#else
  return NULL;

#endif
}
#endif /* CONFIG_SAMA5_PIOD_IRQ ... */

/****************************************************************************
 * Name:  sam_usbsuspend
 *
 * Description:
 *   Board logic must provide the sam_usbsuspend logic if the USBDEV driver
 *   is used.
 *   This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power,
 *   etc. while the USB is suspended.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV
void sam_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}
#endif
#endif /* CONFIG_SAMA5_UHPHS || CONFIG_SAMA5_UDPHS */
