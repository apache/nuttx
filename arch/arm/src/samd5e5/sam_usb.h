/****************************************************************************
 * arch/arm/src/samd5e5/sam_usb.h
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

void sam_usb_suspend(struct usbdev_s *dev, bool resume);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRCSAMD5E5_SAM_USB_H */
