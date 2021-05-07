/****************************************************************************
 * drivers/usbhost/usbhost_findclass.c
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
#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

#include "usbhost_registry.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_idmatch
 *
 * Description:
 *   Check if the class ID matches what the host controller found.
 *
 * Input Parameters:
 *   classid - ID info for the class under consideration.
 *   devid - ID info reported by the device.
 *
 * Returned Value:
 *   TRUE - the class will support this device.
 *
 ****************************************************************************/

static bool usbhost_idmatch(const struct usbhost_id_s *classid,
                            const struct usbhost_id_s *devid)
{
  uinfo("Compare to class:%d subclass:%d protocol:%d vid:%04x pid:%04x\n",
         classid->base, classid->subclass, classid->proto,
         classid->vid, classid->pid);

  /* The base class ID, subclass and protocol have to match up in any event */

  if (devid->base     == classid->base &&
      devid->subclass == classid->subclass &&
      devid->proto    == classid->proto)
    {
      /* If this is a vendor-specific class ID, then the VID and PID have to
       * match as well.
       */

      if (devid->base == USB_CLASS_VENDOR_SPEC)
        {
          /* Vendor specific... do the VID and PID also match? */

          if (devid->vid == classid->vid && devid->pid == classid->pid)
            {
              /* Yes.. then we have a match */

              return true;
            }
        }
      else
        {
          /* Not vendor specific?  Then we have a match */

          return true;
        }
    }

  /* No match.. not supported */

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_findclass
 *
 * Description:
 *   Find a USB host class implementation previously registered by
 *   usbhost_registerclass().  On success, an instance of struct
 *   usbhost_registry_s will be returned.  That instance will contain all of
 *   the information that will be needed to obtain and bind a struct
 *   usbhost_class_s instance for the device.
 *
 * Input Parameters:
 *   id - Identifies the USB device class that has connect to the USB host.
 *
 * Returned Value:
 *   On success this function will return a non-NULL instance of struct
 *   usbhost_registry_s.  NULL will be returned on failure.  This function
 *   can only fail if (1) id is NULL, or (2) no USB host class is registered
 *   that matches the device class ID.
 *
 ****************************************************************************/

const struct usbhost_registry_s *usbhost_findclass(
                                             const struct usbhost_id_s *id)
{
  struct usbhost_registry_s *usbclass;
  irqstate_t flags;
  int ndx;

  DEBUGASSERT(id);
  uinfo("Looking for class:%d subclass:%d protocol:%d vid:%04x pid:%04x\n",
        id->base, id->subclass, id->proto, id->vid, id->pid);

  /* g_classregistry is a singly-linked list of class ID information added by
   * calls to usbhost_registerclass().  Since this list is accessed from USB
   * host controller interrupt handling logic, accesses to this list must be
   * protected by disabling interrupts.
   */

  flags = enter_critical_section();

  /* Examine each register class in the linked list */

  for (usbclass = g_classregistry; usbclass; usbclass = usbclass->flink)
    {
      /* If the registered class supports more than one ID, subclass, or
       * protocol, then try each.
       */

      uinfo("Checking class:%p nids:%d\n", usbclass, usbclass->nids);
      for (ndx = 0; ndx < usbclass->nids; ndx++)
        {
          /* Did we find a matching ID? */

          if (usbhost_idmatch(&usbclass->id[ndx], id))
            {
              /* Yes.. restore interrupts and return the class info */

              leave_critical_section(flags);
              return usbclass;
            }
        }
    }

  /* Not found... restore interrupts and return NULL */

  leave_critical_section(flags);
  return NULL;
}
