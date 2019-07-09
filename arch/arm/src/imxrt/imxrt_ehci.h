/*****************************************************************************
 * arch/arm/src/imxrt/imxrt_ehci.h
 *
 *   Copyright (C) 2012, 2015, 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Dave Marples <dave@marples.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_EHCI_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_EHCI_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/*****************************************************************************
 * Public Types
 *****************************************************************************/

#ifndef __ASSEMBLY__

/*****************************************************************************
 * Public Data
 *****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/*****************************************************************************
 * Name: imxrt_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be provided
 *   by each platform that implements the OHCI or EHCI host interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface. Since the
 *   IMXRT has only a downstream port, zero is the only possible value for
 *           this parameter.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

extern void imxrt_usbhost_vbusdrive(int rhport, bool enable);

/*****************************************************************************
 * Name: imxrt_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an over-current condition
 *   is detected.
 *
 * Input Parameters:
 *   handler - New over-current interrupt handler
 *   arg     - The argument that will accompany the interrupt
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 *****************************************************************************/

extern int imxrt_setup_overcurrent(xcpt_t handler, void *arg);

/*****************************************************************************
 * Name: imxrt_ehci_initialize
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

#if defined(CONFIG_IMXRT_USBOTG) && defined(CONFIG_USBHOST)
struct usbhost_connection_s;
FAR struct usbhost_connection_s *imxrt_ehci_initialize(int controller);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_EHCI_H */
