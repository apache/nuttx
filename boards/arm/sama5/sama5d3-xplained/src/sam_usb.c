/****************************************************************************
 * boards/arm/sama5/sama5d3-xplained/src/sam_usb.c
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
#include "sam_pio.h"
#include "sam_usbhost.h"
#include "hardware/sam_ohci.h"
#include "sama5d3-xplained.h"

#if defined(CONFIG_SAMA5_UHPHS) || defined(CONFIG_SAMA5_UDPHS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SAMA5D3XPLAINED_USBHOST_PRIO
#  define CONFIG_SAMA5D3XPLAINED_USBHOST_PRIO 50
#endif

#ifndef CONFIG_SAMA5D3XPLAINED_USBHOST_STACKSIZE
#  define CONFIG_SAMA5D3XPLAINED_USBHOST_STACKSIZE 1024
#endif

#ifdef HAVE_USBDEV
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
 *   USB-related GPIO pins for the SAMA5D3-Xplained board.
 *
 * USB Ports
 *   The SAMA5D3 series-MB features three USB communication ports:
 *
 *     1. Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed
 *        with USB Device High Speed Micro AB connector, J20
 *
 *     2. Port B Host High Speed (EHCI) and Full Speed (OHCI) standard type A
 *        connector, J19 upper port
 *
 *     3. Port C Host Full Speed (OHCI) only standard type A connector, J19
 *        lower port
 *
 *   The two USB host ports (only) are equipped with 500-mA high-side power
 *   switch for self-powered and bus-powered applications.
 *
 *   The USB device port A (J6) features a VBUS insert detection function.
 *
 *   Port A
 *
 *     PIO  Signal Name Function
 *     ---- ----------- -----------------------------------------------------
 *     PE9  VBUS_SENSE VBus detection
 *
 *     Note: No VBus power switch enable on port A.  I think that this limits
 *     this port to a device port or as a host port for self-powered devices
 *     only.
 *
 *   Port B
 *
 *     PIO  Signal Name Function
 *     ---- ----------- -----------------------------------------------------
 *     PE4  EN5V_USBB   VBus power enable (via MN3 power switch).  To the A1
 *                      pin of J19 Dual USB A connector
 *
 *   Port C
 *
 *     PIO  Signal Name Function
 *     ---- ----------- -----------------------------------------------------
 *     PE3  EN5V_USBC   VBus power enable (via MN3 power switch).  To the B1
 *                      pin of J19 Dual USB A connector
 *
 *    Both Ports B and C
 *
 *     PIO  Signal Name Function
 *     ---- ----------- -----------------------------------------------------
 *     PE5  OVCUR_USB   Combined over-current indication from port A and B
 *
 * That offers a lot of flexibility.  However, here we enable the ports only
 * as follows:
 *
 *   Port A -- USB device
 *   Port B -- EHCI host
 *   Port C -- OHCI host
 *
 ****************************************************************************/

void weak_function sam_usbinitialize(void)
{
#ifdef HAVE_USBDEV
  /* Configure Port A to support the USB device function */

  sam_configpio(PIO_USBA_VBUS_SENSE); /* VBUS sense */

  /* TODO:  Configure an interrupt on VBUS sense */

#endif

#ifdef HAVE_USBHOST
#ifdef CONFIG_SAMA5_UHPHS_RHPORT1
  /* Configure Port A to support the USB OHCI/EHCI function */

#ifdef PIO_USBA_VBUS_ENABLE /* SAMA5D3-Xplained has no port A VBUS enable */
  sam_configpio(PIO_USBA_VBUS_ENABLE); /* VBUS enable, initially OFF */
#endif
#endif

#ifdef CONFIG_SAMA5_UHPHS_RHPORT2
  /* Configure Port B to support the USB OHCI/EHCI function */

  sam_configpio(PIO_USBB_VBUS_ENABLE); /* VBUS enable, initially OFF */
#endif

#ifdef CONFIG_SAMA5_UHPHS_RHPORT3
  /* Configure Port C to support the USB OHCI/EHCI function */

  sam_configpio(PIO_USBC_VBUS_ENABLE); /* VBUS enable, initially OFF */
#endif

#if defined(CONFIG_SAMA5_UHPHS_RHPORT2) || defined(CONFIG_SAMA5_UHPHS_RHPORT3)
  /* Configure Port B/C VBUS overrcurrent detection */

  sam_configpio(PIO_USBBC_VBUS_OVERCURRENT); /* VBUS overcurrent */
#endif
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

  ret = kthread_create("OHCI Monitor", CONFIG_SAMA5D3XPLAINED_USBHOST_PRIO,
                       CONFIG_SAMA5D3XPLAINED_USBHOST_STACKSIZE,
                       (main_t)ohci_waiter, (char * const *)NULL);
  if (ret < 0)
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

  ret = kthread_create("EHCI Monitor", CONFIG_SAMA5D3XPLAINED_USBHOST_PRIO,
                       CONFIG_SAMA5D3XPLAINED_USBHOST_STACKSIZE,
                       (main_t)ehci_waiter, (char * const *)NULL);
  if (ret < 0)
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
      /* SAMA5D3-Xplained has no port A VBUS enable */

      uerr("ERROR: RHPort1 has no VBUS enable\n");
      return;
#else
      pinset = PIO_USBA_VBUS_ENABLE;
      break;
#endif

    case SAM_RHPORT2:
#ifndef CONFIG_SAMA5_UHPHS_RHPORT2
      uerr("ERROR: RHPort2 is not available in this configuration\n");
      return;
#else
      pinset = PIO_USBB_VBUS_ENABLE;
      break;
#endif

    case SAM_RHPORT3:
#ifndef CONFIG_SAMA5_UHPHS_RHPORT3
      uerr("ERROR: RHPort3 is not available in this configuration\n");
      return;
#else
      pinset = PIO_USBC_VBUS_ENABLE;
      break;
#endif

    default:
      uerr("ERROR: RHPort%d is not supported\n", rhport + 1);
      return;
    }

  /* Then enable or disable VBUS power (active low for SP2526A-2) */

  if (enable)
    {
      /* Enable the Power Switch by driving the enable pin low */

      sam_piowrite(pinset, false);
    }
  else
    {
      /* Disable the Power Switch by driving the enable pin high */

      sam_piowrite(pinset, true);
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
void sam_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}
#endif
#endif /* CONFIG_SAMA5_UHPHS || CONFIG_SAMA5_UDPHS */
