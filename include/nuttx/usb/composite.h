/****************************************************************************
 * include/nuttx/usb/composite.h
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

#ifndef __INCLUDE_NUTTX_USB_COMPOSITE_H
#define __INCLUDE_NUTTX_USB_COMPOSITE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbdev.h>

#ifdef CONFIG_USBDEV_COMPOSITE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_USBDEV_COMPOSITE
 *   Enables USB composite device support
 */

#define COMPOSITE_NSTRIDS             (5)  /* The number of String-IDs to
                                            * reserve for the composite device */
#define COMPOSITE_NCONFIGS            (1)  /* The number of configurations
                                            * supported */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Forward declarations
 ****************************************************************************/

struct composite_devdesc_s;

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: composite_initialize
 *
 * Description:
 *   Register USB composite device as configured.  This function will call
 *   board-specific implementations in order to obtain the class objects for
 *   each of the members of the composite.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A non-NULL "handle" is returned on success.  This handle may be used
 *   later with composite_uninitialize() in order to removed the composite
 *   device.  This handle is the (untyped) internal representation of the
 *   the class driver instance.
 *
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR void *composite_initialize(uint8_t ndevices,
                               FAR struct composite_devdesc_s *pdevices);

/****************************************************************************
 * Name: composite_uninitialize
 *
 * Description:
 *   Un-initialize the USB composite driver.  The handle is the USB composite
 *   class' device object as was returned by composite_initialize().  This
 *   function will call  board-specific implementations in order to free the
 *   class objects for each of the members of the composite.
 *
 * Input Parameters:
 *   handle - The handle returned by a previous call to
 *            composite_initialize().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void composite_uninitialize(FAR void *handle);

/****************************************************************************
 * Name: composite_ep0submit
 *
 * Description:
 *   Members of the composite cannot send on EP0 directly because EP0 is
 *   is "owned" by the composite device.  Instead, when configured as members
 *   of a composite device, those classes should call this method so that
 *   the composite device can send on EP0 on behalf of the class.
 *
 ****************************************************************************/

struct usbdevclass_driver_s;
struct usbdev_s;
struct usbdev_req_s;

int composite_ep0submit(FAR struct usbdevclass_driver_s *driver,
                        FAR struct usbdev_s *dev,
                        FAR struct usbdev_req_s *ctrlreq);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_USBDEV_COMPOSITE */
#endif /* __INCLUDE_NUTTX_USB_COMPOSITE_H */
