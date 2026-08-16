/****************************************************************************
 * include/nuttx/usb/xhci.h
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

#ifndef __INCLUDE_NUTTX_USB_XHCI_H
#define __INCLUDE_NUTTX_USB_XHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/usb/usbhost.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* How the controller was found *********************************************/

/* What a controller cannot work out about itself.
 *
 * xHCI is the same silicon on PCI or wired into an SoC, and everything the
 * specification describes is reached through the register block.  What
 * differs is how the interrupt arrives: PCI needs a message the device is
 * configured to send, a memory mapped bus has a wire the interrupt
 * controller already knows.  The bus answers that, and nothing else.
 */

struct xhci_bus_ops_s
{
  /* Attach the handler and make interrupts start arriving.  Enabling
   * belongs here rather than in the controller driver because on some
   * buses attaching and enabling are one operation.
   */

  CODE int (*irq_attach)(FAR void *arg, xcpt_t handler, FAR void *priv);

  /* Undo it, and release anything the bus allocated to make it work */

  CODE void (*irq_detach)(FAR void *arg);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xhci_initialize
 *
 * Description:
 *   Bring up an xHCI controller and start watching its root hub ports.
 *
 * Input Parameters:
 *   name - What to call this controller when reporting what is attached to
 *          it, since a system may have more than one and "port 1" alone
 *          does not say which
 *   base - Where the controller's register block starts
 *   ops  - How to reach the interrupt this controller raises
 *   arg  - Opaque value passed back to ops
 *
 * Returned Value:
 *   A connection to hand to usbhost_waiter_initialize(); NULL on failure.
 *
 ****************************************************************************/

FAR struct usbhost_connection_s *
xhci_initialize(FAR const char *name, uintptr_t base,
                FAR const struct xhci_bus_ops_s *ops, FAR void *arg);

/****************************************************************************
 * Name: xhci_uninitialize
 *
 * Description:
 *   Stop watching a controller's ports and give back everything
 *   xhci_initialize() took.
 *
 ****************************************************************************/

void xhci_uninitialize(FAR struct usbhost_connection_s *conn);

#endif /* __INCLUDE_NUTTX_USB_XHCI_H */
