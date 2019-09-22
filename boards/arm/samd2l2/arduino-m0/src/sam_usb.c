/****************************************************************************
 * boards/arm/samd2l2/arduino-m0/src/sam_usb.c
 *
 *   Copyright (C) 2019 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 *   Parts of code from here and there
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "debug.h"
#include "chip.h"
#include "sam_port.h"
#include "saml_periphclks.h"
#include "arduino_m0.h"

#if defined(CONFIG_SAMD2L2_USB)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called from sam_boot very early in initialization to setup
 *   USB-related GPIO pins for the SAML21-Xplained board.
 *
 * USB Ports
 *   The SAML21 features USB device and host:
 *
 *
 ****************************************************************************/

void weak_function sam_usbinitialize(void)
{
#ifdef HAVE_USBDEV

  /* Detect when a target USB cable is connected in self-powered mode */

  /* The Engie CE0035 doesn't have VBUS SENSE support */

  /* sam_configport(PORT_USB_VBUS_SENSE); */

  /* TODO:  Configure an interrupt on VBUS sense */

#endif

#ifdef HAVE_USBHOST
  /* In USB host mode VBUS voltage is provided by the kit and can thus not
   * identify a connected device, so another GPIO is used to detect
   * the USB ID of the device.
   */

  /* The Engie CE0035 doesn't have the USB ID pin connected */

  /* sam_configport(PORT_USB_ID_DETECT); */  /* ID detect */

#endif
}

/****************************************************************************
 * Name:  sam_usbsuspend
 *
 * Description:
 *   Board logic must provide the sam_usbsuspend logic if the USBDEV driver is
 *   used.
 *   This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV
void sam_usb_suspend(FAR struct usbdev_s *dev, bool resume)
{
  uinfo("board: resume: %d\n", resume);
}
#endif

#endif /* CONFIG_SAMDL_USB */
