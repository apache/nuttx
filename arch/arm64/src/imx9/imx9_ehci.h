/****************************************************************************
 * arch/arm64/src/imx9/imx9_ehci.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_EHCI_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_EHCI_H

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

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided by each platform that implements the EHCI host interface.
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx9_usbhost_vbusdrive(int rhport, bool enable);

/****************************************************************************
 * Name: imx9_setup_overcurrent
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

int imx9_setup_overcurrent(xcpt_t handler, void *arg);

/****************************************************************************
 * Name: imx9_ehci_initialize
 *
 * Description:
 *   Initialize USB EHCI host controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than one EHCI interface,
 *     then this identifies which controller is being initialized.
 *     Normally, this is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the
 *   device to a class driver.
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_USBHOST
struct usbhost_connection_s;
struct usbhost_connection_s *imx9_ehci_initialize(int controller);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_EHCI_H */
