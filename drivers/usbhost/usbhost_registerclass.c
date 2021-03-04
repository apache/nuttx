/****************************************************************************
 * drivers/usbhost/usbhost_registerclass.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/usb/usbhost.h>

#include "usbhost_registry.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_registerclass
 *
 * Description:
 *   Register a USB host class implementation.  The caller provides an
 *   instance of struct usbhost_registry_s that contains all of the
 *   information that will be needed later to (1) associate the USB host
 *   class implementation with a connected USB device, and (2) to obtain and
 *   bind a struct usbhost_class_s instance for the device.
 *
 * Input Parameters:
 *   usbclass - An write-able instance of struct usbhost_registry_s that
 *     will be maintained in a registry.
 *
 * Returned Value:
 *   On success, this function will return zero (OK).  Otherwise, a negated
 *   errno value is returned.
 *
 ****************************************************************************/

int usbhost_registerclass(struct usbhost_registry_s *usbclass)
{
  irqstate_t flags;

  uinfo("Registering class:%p nids:%d\n", usbclass, usbclass->nids);

  /* g_classregistry is a singly-linkedlist of class ID information added by
   * calls to usbhost_registerclass().  Since this list is accessed from USB
   * host controller interrupt handling logic, accesses to this list must be
   * protected by disabling interrupts.
   */

  flags = enter_critical_section();

  /* Add the new class ID info to the head of the list */

  usbclass->flink = g_classregistry;
  g_classregistry = usbclass;

  leave_critical_section(flags);
  return OK;
}
