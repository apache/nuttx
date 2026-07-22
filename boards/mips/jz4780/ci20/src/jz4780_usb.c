/****************************************************************************
 * boards/mips/jz4780/ci20/src/jz4780_usb.c
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

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "mips_internal.h"
#include "chip.h"
#include "jz_usbhost.h"

#include <arch/board/board.h>

#if defined(CONFIG_USBHOST)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jz_usbhost_bootinitialize
 *
 * Description:
 *   Called from jz_boardinitialize very early in initialization to setup
 *   USB host-related hardware for the board.
 *
 *
 ****************************************************************************/

void weak_function jz_usbhost_bootinitialize(void)
{
}

/****************************************************************************
 * Name: jz_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

int jz_usbhost_initialize(void)
{
  int ret;

#ifdef CONFIG_JZ4780_OHCI
  struct usbhost_connection_s *ohciconn;
#endif
#ifdef CONFIG_JZ4780_EHCI
  struct usbhost_connection_s *ehciconn;
#endif

  /* First, register all of the class drivers needed to support the drivers
   * that we care about
   */

#ifdef CONFIG_USBHOST_HUB
  /* Initialize USB hub support */

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

#ifdef CONFIG_USBHOST_HIDMOUSE
  /* Initialize the HID mouse class */

  ret = usbhost_mouse_init();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID mouse class\n");
    }
#endif

#ifdef CONFIG_JZ4780_OHCI
  /* Get an instance of the USB OHCI interface. */

  ohciconn = jz_ohci_initialize(0);
  if (!ohciconn)
    {
      uerr("ERROR: jz_ohci_initialize failed\n");
      return ENODEV;
    }

  /* Initialize waiter */

  ret = usbhost_waiter_initialize(ohciconn);

  if (ret < 0)
    {
      uerr("ERROR: Failed to create ohci_waiter task: %d\n", ret);
      return -ENODEV;
    }
#endif

#ifdef CONFIG_JZ4780_EHCI
  /* Then get an instance of the USB EHCI interface. */

  ehciconn = jz_ehci_initialize(0);
  if (!ehciconn)
    {
      uerr("ERROR: jz_ehci_initialize failed\n");
      return ENODEV;
    }

  /* Initialize waiter */

  ret = usbhost_waiter_initialize(ehciconn);

  if (ret < 0)
    {
      uerr("ERROR: Failed to create ehci_waiter task: %d\n", ret);
      return -ENODEV;
    }
#endif

  return OK;
}

#endif /* CONFIG_USBHOST */
