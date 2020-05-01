/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/sam_usb.c
 *
 *   Copyright (C) 2013, 2015-2016 Gregory Nutt. All rights reserved.
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
#include "sama5d3x-ek.h"

#if defined(CONFIG_SAMA5_UHPHS) || defined(CONFIG_SAMA5_UDPHS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SAMA5D3xEK_USBHOST_PRIO
#  define CONFIG_SAMA5D3xEK_USBHOST_PRIO 50
#endif

#ifndef CONFIG_SAMA5D3xEK_USBHOST_STACKSIZE
#  define CONFIG_SAMA5D3xEK_USBHOST_STACKSIZE 1024
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
 *   USB-related GPIO pins for the SAMA5D3x-EK board.
 *
 * USB Ports
 *   The SAMA5D3 series-MB features three USB communication ports:
 *
 *     1. Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed
 *        with USB Device High Speed Micro AB connector, J20
 *
 *     2. Port B Host High Speed (EHCI) and Full Speed (OHCI) standard
 *        type A connector, J19 upper port
 *
 *     3. Port C Host Full Speed (OHCI) only standard type A connector,
 *        J19 lower port
 *
 *   All three USB host ports are equipped with 500 mA high-side power switch
 *   for self-powered and buspowered applications.
 *   The USB device port feature VBUS inserts detection function.
 *
 *   Port A
 *
 *     PIO  Signal Name Function
 *     ---- ----------- -----------------------------------------------------
 *     PD29  VBUS_SENSE VBus detection
 *     PD25  EN5V_USBA  VBus power enable (via MN15 AIC1526 Dual USB
 *                      High-Side Power Switch.
 *                       The other channel of the switch is for the LCD)
 *
 *   Port B
 *
 *     PIO  Signal Name Function
 *     ---- ----------- -----------------------------------------------------
 *     PD26 EN5V_USBB   VBus power enable (via MN14 AIC1526 Dual USB
 *                      High-Side Power Switch).
 *                      To the A1 pin of J19 Dual USB A connector
 *
 *   Port C
 *
 *     PIO  Signal Name Function
 *     ---- ----------- -----------------------------------------------------
 *     PD27 EN5V_USBC   VBus power enable (via MN14 AIC1526 Dual USB
 *                      High-Side Power Switch).
 *                      To the B1 pin of J19 Dual USB A connector
 *
 *    Both Ports B and C
 *
 *     PIO  Signal Name Function
 *     ---- ----------- -----------------------------------------------------
 *     PD28 OVCUR_USB   Combined overrcurrent indication from port A and B
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

  sam_configpio(PIO_USBA_VBUS_ENABLE); /* VBUS enable, initially OFF */
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
  pid_t pid;
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about/
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
                      CONFIG_SAMA5D3xEK_USBHOST_PRIO,
                      CONFIG_SAMA5D3xEK_USBHOST_STACKSIZE,
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

  pid = kthread_create("EHCI Monitor", CONFIG_SAMA5D3xEK_USBHOST_PRIO,
                       CONFIG_SAMA5D3xEK_USBHOST_STACKSIZE,
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
 *   This function must be provided by each platform that implements the OHCI
 *   or EHCI host interface
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
#ifndef CONFIG_SAMA5_UHPHS_RHPORT1
      uerr("ERROR: RHPort1 is not available in this configuration\n");
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

  /* Then enable or disable VBUS power */

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
 *   Board logic must provide the sam_usbsuspend logic if the USBDEV driver is
 *   used.
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
