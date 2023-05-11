/****************************************************************************
 * include/nuttx/usb/usbhost_devaddr.h
 * Manage USB device addresses
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

#ifndef __INCLUDE_NUTTX_USB_USBHOST_DEVADDR_H
#define __INCLUDE_NUTTX_USB_USBHOST_DEVADDR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define USBHOST_DEVADDR_HASHSIZE 8
#define USBHOST_DEVADDR_HASHMASK (USBHOST_DEVADDR_HASHSIZE - 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct usbhost_devaddr_s
{
  uint8_t   next;           /* Next device address */
  mutex_t   lock;           /* Enforces mutually exclusive access */
  uint32_t  alloctab[4];    /* Bit allocation table */
};

/****************************************************************************
 * Public Data
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
 * Public Functions Definitions
 ****************************************************************************/

struct usbhost_hubport_s;     /* Forward reference */
struct usbhost_roothubport_s; /* Forward reference */

/****************************************************************************
 * Name: usbhost_devaddr_initialize
 *
 * Description:
 *   Initialize the caller provided struct usbhost_devaddr_s instance in
 *   preparation for the management of device addresses on behalf of an root
 *   hub port.
 *
 * Input Parameters:
 *   devgen - A reference to a usbhost_devaddr_s structure.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

int usbhost_devaddr_initialize(FAR struct usbhost_devaddr_s *devgen);

/****************************************************************************
 * Name: usbhost_devaddr_create
 *
 * Description:
 *   Create a new unique device address for this hub port.
 *
 * Input Parameters:
 *   hport - A reference to a hub port structure to which a device has been
 *     newly connected and so is in need of a function address.
 *
 * Returned Value:
 *   On success, a new device function address in the range 0x01 to 0x7f
 *   is returned.  On failure, a negated errno value is returned.
 *
 ****************************************************************************/

int usbhost_devaddr_create(FAR struct usbhost_hubport_s *hport);

/****************************************************************************
 * Name: usbhost_devaddr_destroy
 *
 * Description:
 *  Release a device address previously assigned by usbhost_devaddr_create().
 *
 * Input Parameters:
 *  hport - A reference to a hub port structure from which a device has been
 *     disconnected and so no longer needs the function address.
 *  devaddr - The address to be released.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void usbhost_devaddr_destroy(FAR struct usbhost_hubport_s *hport,
                             uint8_t devaddr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_USBHOST_DEVADDR_H */
