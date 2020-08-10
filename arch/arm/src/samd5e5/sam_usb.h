/****************************************************************************
 * arch/arm/src/samd5e5/sam_usb.h
 *
 *   Copyright (C) 2015 Filament - www.filament.com
 *   Copyright (C) 2015 Offcode Ltd. All rights reserved.
 *   Author: Janne Rosberg <janne@offcode.fi>
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org> (port to SAMD5E5)
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRCSAMD5E5_SAM_USB_H
#define __ARCH_ARM_SRCSAMD5E5_SAM_USB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbdev.h>
#include <stdint.h>

#include "chip.h"
#include "hardware/sam_usb.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name:  sam_usb_suspend
 *
 * Description:
 *   Board logic must provide the sam_usb_suspend logic if the USB driver is
 *   used. This function is called whenever the USB enters or leaves
 *   suspend mode.
 *
 *   When 'resume' is false, this function call provides an opportunity to
 *   perform board-specific power-saving actions so that less power is
 *   consumed while the USB is suspended.
 *
 * NOTE:
 *   Certain power-saving operations are performed by the UDP driver when it
 *   enters suspend mode:  The USB device peripheral clocks are be switched
 *   off. MCK and UDPCK are switched off and the USB transceiver is disabled.
 *
 *   When 'resume' is true, normal clocking and operations must all be
 *   restored.
 *
 ****************************************************************************/

void sam_usb_suspend(FAR struct usbdev_s *dev, bool resume);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRCSAMD5E5_SAM_USB_H */
