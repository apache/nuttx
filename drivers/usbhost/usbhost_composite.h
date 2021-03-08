/****************************************************************************
 * drivers/usbhost/usbhost_composite.h
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
 *   composite device and (2) the composite class wrapper was successfully
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
