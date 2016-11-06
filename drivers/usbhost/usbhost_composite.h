/****************************************************************************
 * drivers/usbhost/usbdev_composite.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __DRIVERS_USBHOST_USBHOST_COMPOSITE_H
#define __DRIVERS_USBHOST_USBHOST_COMPOSITE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

#ifdef CONFIG_USBHOST_COMPOSITE

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: usbhost_composite
 *
 * Description:
 *   As the final steps in the device enumeration sequence this function
 *   will be called in order to determine (1) determine if the device is
 *   a composite device, and if so, (2) create the composite class which
 *   contains all of the individual class instances making up the composite.
 *
 * Input Parameters:
 *   hport      - The downstream port to which the (potential) composite
 *                device has been connected.
 *   configdesc - The full configuration descriptor
 *   desclen    - The length of the configuration descriptor
 *   usbclass   - If the class driver for the device is successful located
 *                and bound to the hub port, the allocated class instance
 *                is returned into this caller-provided memory location.
 *
 * Returned Value:
 *   Zero (OK) is returned if (1) the device was determined to be a
 *   composite device and (2) the composite class wrapper was sucessfully
 *   created and bound to the HCD.  A negated errno value is returned on
 *   any failure.  The value -ENOENT, in particular means that the attached
 *   device is not a composite device.  Other values would indicate other
 *   various, unexpected failures.
 *
 ****************************************************************************/

int usbhost_composite(FAR struct usbhost_hubport_s *hport,
                      FAR const uint8_t *configdesc, int desclen,
                      FAR struct usbhost_id_s *id,
                      FAR struct usbhost_class_s **usbclass);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_USBHOST_COMPOSITE */
#endif /* #define __DRIVERS_USBHOST_USBHOST_COMPOSITE_H */
