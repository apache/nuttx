/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_usbhost.c
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

#include <nuttx/kthread.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "chip.h"
#include "arm_internal.h"
#include "metro-m4.h"
#include "sam_port.h"
#include "sam_usbhost.h"
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
#define HAVE_USB 1
#endif

#ifndef CONFIG_METRO_M4_USBHOST_PRIO
#define CONFIG_METRO_M4_USBHOST_PRIO 100
#endif

#ifndef CONFIG_METRO_M4_USBHOST_STACKSIZE
#define CONFIG_METRO_M4_USBHOST_STACKSIZE 1024
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBHOST
static struct usbhost_connection_s *g_usbconn;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
static int usbhost_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;
  uinfo("Running\n");

  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_usbconn, &hport));
      uinfo("%s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

         (void)CONN_ENUMERATE(g_usbconn, hport);
        }

#ifdef CONFIG_METRO_M4_USB_AUTOMOUNT
      /* Let the automounter know about the insertion event */

      sam_automount_event(hport->connected);
#endif
    }

  /* Keep the compiler from complaining */

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *
 ****************************************************************************/

void sam_usbinitialize(void)
{
}

/****************************************************************************
 * Name: sam_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output. This function must be provided
 *   be each platform that implements the HS host interface
 *
 * Input Parameters:
 *   iface  - For future growth to handle multiple USB host interface. Should
 *            be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
void sam_usbhost_vbusdrive(int iface, bool enable)
{
  DEBUGASSERT(iface == 0);
  if (enable)
    {
      /* Set your function here! */
    }
  else
    {
      /* Set your function here! */
    }
}
#endif

/****************************************************************************
 * Name: samd_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
int samd_usbhost_initialize(void)
{
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  uinfo("Register class drivers\n");

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB mass storage class class */

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  /* Initialize the HID keyboard class */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID keyboard class\n");
    }
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
  /* Initialize the HID mouse class */

  ret = usbhost_mouse_init();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID mouse class\n");
    }
#endif

  /* Then get an instance of the USB host interface */

  uinfo("Initialize USB host\n");
  g_usbconn = sam_usbhost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      uinfo("Start usbhost_waiter\n");
      ret = kthread_create("usbhost", CONFIG_METRO_M4_USBHOST_PRIO,
                           CONFIG_METRO_M4_USBHOST_STACKSIZE,
                           usbhost_waiter, NULL);
      return ret < 0 ? -ENOEXEC : OK;
    }

  return -ENODEV;
}
#endif
