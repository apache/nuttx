/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_usbhost.c
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

#include <debug.h>
#include <errno.h>
#include <nuttx/config.h>
#include <nuttx/usb/usbhost.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include "stm32.h"
#include "stm32_butterfly2.h"
#include "stm32_otgfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_OTGFS
#  error "CONFIG_USBHOST requires CONFIG_STM32_OTGFS to be enabled"
#endif

#ifdef CONFIG_USBDEV
#  error "CONFIG_USBHOST cannot be set alongside CONFIG_USBDEV"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usbhost_connection_s *g_usbconn;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_detect
 *
 * Description:
 *   Wait for USB devices to be connected.
 ****************************************************************************/

static void *usbhost_detect(void *arg)
{
  struct usbhost_hubport_s *hport;

  uinfo("INFO: Starting usb detect thread\n");

  for (; ; )
    {
      CONN_WAIT(g_usbconn, &hport);

      if (hport->connected)
        {
          CONN_ENUMERATE(g_usbconn, hport);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Initializes USB host functionality.
 ****************************************************************************/

int stm32_usbhost_initialize(void)
{
  int rv;

#ifdef CONFIG_USBHOST_MSC
  uinfo("INFO: Initializing USB MSC class\n");

  if ((rv = usbhost_msc_initialize()) < 0)
    {
      uerr("ERROR: Failed to register mass storage class: %d\n", rv);
    }
#endif

#ifdef CONFIG_USBHOST_CDACM
  uinfo("INFO: Initializing CDCACM usb class\n");

  if ((rv = usbhost_cdacm_initialize()) < 0)
    {
      uerr("ERROR: Failed to register CDC/ACM serial class: %d\n", rv);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  uinfo("INFO: Initializing HID Keyboard usb class\n");

  if ((rv = usbhost_kbdinit()) < 0)
    {
      uerr("ERROR: Failed to register the KBD class: %d\n", rv);
    }
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
  uinfo("INFO: Initializing HID Mouse usb class\n");

  if ((rv = usbhost_mouse_init()) < 0)
    {
      uerr("ERROR: Failed to register the mouse class: %d\n", rv);
    }
#endif

#ifdef CONFIG_USBHOST_HUB
  uinfo("INFO: Initializing USB HUB class\n");

  if ((rv = usbhost_hub_initialize()) < 0)
    {
      uerr("ERROR: Failed to register hub class: %d\n", rv);
    }
#endif

  if ((g_usbconn = stm32_otgfshost_initialize(0)))
    {
      pthread_attr_t pattr;
      struct sched_param schparam;

      pthread_attr_init(&pattr);
      pthread_attr_setstacksize(&pattr, 2048);

      schparam.sched_priority = 50;
      pthread_attr_setschedparam(&pattr, &schparam);

      return pthread_create(NULL, &pattr, usbhost_detect, NULL);
    }

  return -ENODEV;
}

/****************************************************************************
 * Name: stm32_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.
 *
 *   The application uses this field to control power to this port, and the
 *   core clears this bit on an overcurrent condition.
 *
 * Input Parameters:
 *   iface - For future growth to handle multiple USB host interface.
 *     Should be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void stm32_usbhost_vbusdrive(int iface, bool enable)
{
  stm32_gpiowrite(GPIO_OTGFS_PWRON, enable);
}
