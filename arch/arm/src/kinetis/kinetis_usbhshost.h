/****************************************************************************
 * arch/arm/src/kinetis/kinetis_usbhshost.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_USBHSHOST_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_USBHSHOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided by each platform that implements the OHCI or EHCI host
 *   interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface. Since the
 *   KINETIS has only a downstream port, zero is the only possible value for
 *           this parameter.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

extern void kinetis_usbhost_vbusdrive(int rhport, bool enable);

/****************************************************************************
 * Name: kinetis_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an over-current
 *   condition is detected.
 *
 * Input Parameters:
 *   handler - New over-current interrupt handler
 *   arg     - The argument that will accompany the interrupt
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

extern int kinetis_setup_overcurrent(xcpt_t handler, void *arg);

/****************************************************************************
 * Name: kinetis_ehci_initialize
 *
 * Description:
 *   Initialize USB EHCI host controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than one EHCI interface, then
 *     this identifies which controller is being initialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 ****************************************************************************/

#if defined(CONFIG_KINETIS_USBHS) && defined(CONFIG_USBHOST)
struct usbhost_connection_s;
struct usbhost_connection_s *kinetis_ehci_initialize(int controller);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_USBHSHOST_H */
