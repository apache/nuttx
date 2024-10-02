/****************************************************************************
 * include/nuttx/usb/mtp.h
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

#ifndef __INCLUDE_NUTTX_USB_MTP_H
#define __INCLUDE_NUTTX_USB_MTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* Indexes for devinfo.epno[] array.
 * Used for composite device configuration.
 */

#define USBMTP_NUM_EPS             (3)

#define USBMTP_EP_BULKIN_IDX       (0)
#define USBMTP_EP_BULKOUT_IDX      (1)
#define USBMTP_EP_INTIN_IDX        (2)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Name: usbdev_mtp_initialize
 *
 * Description:
 *   Initialize the Media Transfer Protocol USB device driver.
 *
 * Returned Value:
 *   A non-NULL "handle" is returned on success.
 *
 ****************************************************************************/

FAR void *usbdev_mtp_initialize(void);

/****************************************************************************
 * Name: usbdev_mtp_uninitialize
 *
 * Description:
 *   Uninitialize the Media Transfer Protocol USB device driver.
 *
 ****************************************************************************/

void usbdev_mtp_uninitialize(FAR void *handle);

/****************************************************************************
 * Name: usbdev_mtp_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void usbdev_mtp_get_composite_devdesc(FAR struct composite_devdesc_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_ADB_H */
