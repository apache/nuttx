/****************************************************************************
 * drivers/usbdev/usbdev_fs.h
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

#ifndef __DRIVERS_USBDEV_FS_H
#define __DRIVERS_USBDEV_FS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_fs_initialize
 *
 * Description:
 *   USBDEV fs initialize
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

FAR void *usbdev_fs_initialize(FAR const struct usbdev_devdescs_s *devdescs,
                               FAR struct composite_devdesc_s *pdevice);

/****************************************************************************
 * Name: usbdev_fs_uninitialize
 *
 * Description:
 *   USBDEV fs uninitialize
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

void usbdev_fs_uninitialize(FAR void *handle);

/****************************************************************************
 * Name: usbclass_classobject
 *
 * Description:
 *   Register USB driver and return the class object.
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

int usbdev_fs_classobject(int minor,
                          FAR struct usbdev_devinfo_s *devinfo,
                          FAR struct usbdevclass_driver_s **classdev);

/****************************************************************************
 * Name: usbdev_fs_classuninitialize
 *
 * Description:
 *   Free allocated class memory
 *
 ****************************************************************************/

void usbdev_fs_classuninitialize(FAR struct usbdevclass_driver_s *classdev);

#endif /* __DRIVERS_USBDEV_FS_H */
