/****************************************************************************
 * drivers/usbdev/composite.h
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

#ifndef __DRIVERS_USBDEV_COMPOSITE_H
#define __DRIVERS_USBDEV_COMPOSITE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/types.h>
#include <stdint.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_USBDEV_COMPOSITE
#  define NUM_DEVICES_TO_HANDLE       (CONFIG_COMPOSITE_DEVICES)
#else
#  define NUM_DEVICES_TO_HANDLE       (1)
#endif

/* These settings are not modifiable via the NuttX configuration */

#define COMPOSITE_CONFIGIDNONE        (0)  /* Config ID = 0 means to return to address mode */
#define COMPOSITE_CONFIGID            (1)  /* The only supported configuration ID */

/* Descriptor strings */

#define COMPOSITE_MANUFACTURERSTRID   (1)
#define COMPOSITE_PRODUCTSTRID        (2)
#define COMPOSITE_SERIALSTRID         (3)
#define COMPOSITE_CONFIGSTRID         (4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct composite_devobj_s
{
  /* Device description given by the user code in the dynamic
   * configuration.
   */

  struct composite_devdesc_s compdesc;

  /* Pointer to device class */

  FAR struct usbdevclass_driver_s *dev;
};

/* This structure describes the internal state of the driver */

struct composite_dev_s
{
  FAR struct usbdev_s                *usbdev;                        /* usbdev driver pointer */
  FAR struct usbdev_req_s            *ctrlreq;                       /* Allocated control request */
  int                                 cfgdescsize;                   /* Total size of the configuration descriptor */
  int                                 ninterfaces;                   /* The total number of interfaces in this composite device */
  uint8_t                             config;                        /* Configuration number */
  uint8_t                             ndevices;                      /* Num devices in this composite device */
  struct composite_devobj_s           device[NUM_DEVICES_TO_HANDLE]; /* Device class object */
  FAR const struct usbdev_devdescs_s *descs;                         /* Device descriptors */
};

#endif /* __DRIVERS_USBDEV_COMPOSITE_H */
