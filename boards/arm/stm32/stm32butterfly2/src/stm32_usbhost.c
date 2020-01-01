/*****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_usbhost.c
 *
 *   Copyright (C) 2016 Michał Łyszczek. All rights reserved.
 *   Author: Michał Łyszczek <michal.lyszczek@gmail.com>
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
 ****************************************************************************/

/*****************************************************************************
 * Include Files
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

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_OTGFS
#  error "CONFIG_USBHOST requires CONFIG_STM32_OTGFS to be enabled"
#endif

#ifdef CONFIG_USBDEV
#  error "CONFIG_USBHOST cannot be set alongside CONFIG_USBDEV"
#endif

/*****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usbhost_connection_s *g_usbconn;

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: usbhost_detect
 *
 * Description:
 *   Wait for USB devices to be connected.
 ****************************************************************************/

static void* usbhost_detect(void *arg)
{
  struct usbhost_hubport_s *hport;

  uinfo("INFO: Starting usb detect thread\n");

  for (;;)
    {
      CONN_WAIT(g_usbconn, &hport);

      if (hport->connected)
        {
          CONN_ENUMERATE(g_usbconn, hport);
        }
    }

  return 0;
}

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
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

/*****************************************************************************
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
