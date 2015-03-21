/************************************************************************************
 * configs/ea3131/src/lpc31_usbhost.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "up_arch.h"

#include "lpc31_internal.h"
#include "ea3131_internal.h"

#if defined(CONFIG_LPC31_USBOTG) || defined(CONFIG_USBHOST)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_USBHOST_DEFPRIO
#  define CONFIG_USBHOST_DEFPRIO 50
#endif

#ifndef CONFIG_USBHOST_STACKSIZE
#  define CONFIG_USBHOST_STACKSIZE 1024
#endif

#ifdef HAVE_USBDEV
#  undef CONFIG_LPC31_USBOTG_RHPORT1
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/
/* Retained device driver handle */

static struct usbhost_connection_s *g_ehciconn;

/* Overcurrent interrupt handler */

#if 0 /* Not yet implemented */
static xcpt_t g_ochandler;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: ehci_waiter
 *
 * Description:
 *   Wait for USB devices to be connected to the EHCI root hub.
 *
 ************************************************************************************/

static int ehci_waiter(int argc, char *argv[])
{
  bool connected = false;
  int rhpndx;
  int ret;

  uvdbg("Waiter Running\n");
  for (;;)
    {
      /* Wait for the device to change state */

      rhpndx = CONN_WAIT(g_ehciconn, &connected);
      DEBUGASSERT(rhpndx >= 0 && rhpndx < 1);

      connected = !connected;

      uvdbg("RHport1 %s\n",
            connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (connected)
        {
          /* Yes.. enumerate the newly connected device */

          ret = CONN_ENUMERATE(g_ehciconn, rhpndx);
          if (ret < 0)
            {
              uvdbg("RHport1 CONN_ENUMERATE failed: %d\n", ret);
              connected = false;
            }
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc31_usbhost_bootinitialize
 *
 * Description:
 *   Called from lpc31_boardinitialize very early in inialization to setup USB
 *   host-related GPIO pins for the EA3131 board.
 *
 *   USB host VBUS power is controlled by a Micrel USB power switch.  That switch is
 *   driver by a discrete that comes from the I2C-contrrol PCA9532 GPIO expander.
 *
 ************************************************************************************/

void weak_function lpc31_usbhost_bootinitialize(void)
{
  /* Get an instance of the I2C interface.  This will be needed to control the
   * PCA9532 GPIO expander.
   */
#warning Missing logic

  /* Use the I2C interface to initialize the PCA9532 GPIO expander driver */
#warning Missing logic

  /* Configure pin to drive VBUS power using the PCA8532 GPIO expander */
#warning Missing logic

  /* Configure pin to detect overrcurrent errors */
#warning Missing logic
}

/***********************************************************************************
 * Name: lpc31_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ***********************************************************************************/

int lpc31_usbhost_initialize(void)
{
  pid_t pid;
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about
   *
   * Register theUSB host Mass Storage Class:
   */

#ifdef CONFIG_USBHOST_MSC
  ret = usbhost_storageinit();
  if (ret != OK)
    {
      udbg("ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

  /* Register the USB host HID keyboard class driver */

#ifdef CONFIG_USBHOST_HIDKBD
  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      udbg("ERROR: Failed to register the KBD class\n");
    }
#endif

  /* Then get an instance of the USB EHCI interface. */

  g_ehciconn = lpc31_ehci_initialize(0);
  if (!g_ehciconn)
    {
      udbg("ERROR: lpc31_ehci_initialize failed\n");
      return -ENODEV;
    }

  /* Start a thread to handle device connection. */

  pid = task_create("EHCI Monitor", CONFIG_USBHOST_DEFPRIO,  CONFIG_USBHOST_STACKSIZE,
                    (main_t)ehci_waiter, (FAR char * const *)NULL);
  if (pid < 0)
    {
      udbg("ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }

  return OK;
}

/***********************************************************************************
 * Name: lpc31_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be provided by
 *   each platform that implements the OHCI or EHCI host interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface.  See SAM_RHPORT_*
 *            definitions above.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ***********************************************************************************/

void lpc31_usbhost_vbusdrive(int rhport, bool enable)
{
  uvdbg("RHPort%d: enable=%d\n", rhport+1, enable);

  /* The LPC3131 has only a single root hub port */

  if (rhport == 0)
    {
      /* Then enable or disable VBUS power */

      if (enable)
        {
          /* Enable the Power Switch by driving the enable pin low */
#warning Missing logic
        }
      else
        {
          /* Disable the Power Switch by driving the enable pin high */
#warning Missing logic
        }
    }
}

/************************************************************************************
 * Name: lpc31_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an overcurrent condition is
 *   detected.
 *
 * Input parameter:
 *   handler - New overcurrent interrupt handler
 *
 * Returned value:
 *   Old overcurrent interrupt handler
 *
 ************************************************************************************/

#if 0 /* Not ready yet */
xcpt_t lpc31_setup_overcurrent(xcpt_t handler)
{
  xcpt_t oldhandler;
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = irqsave();

  /* Get the old button interrupt handler and save the new one */

  oldhandler  = g_ochandler;
  g_ochandler = handler;

  /* Configure the interrupt */
#warning Missing logic

  /* Return the old button handler (so that it can be restored) */

  irqrestore(flags);
  return oldhandler;
}
#endif /* 0 */

#endif /* CONFIG_LPC31_USBOTG || CONFIG_USBHOST */
