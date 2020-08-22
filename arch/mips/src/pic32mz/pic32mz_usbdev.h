/************************************************************************************
 * arch/mips/src/pic32mz/pic32mz_usbdev.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_USBDEV_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_USBDEV_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: pic32mz_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB software
 *   connect and disconnect), then the board software must provide pic32mz_usbpullup.
 *   See include/nuttx/usb/usbdev.h for additional description of this method.
 *   Alternatively, if no pull-up GPIO the following can be redefined to be
 *   NULL.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MZ_USBDEV
struct usbdev_s;
int pic32mz_usbpullup(FAR struct usbdev_s *dev,  bool enable);
#endif

/************************************************************************************
 * Name: pic32mz_usbsuspend
 *
 * Description:
 *   Board logic must provide the pic32mz_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc. while
 *   the USB is suspended.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MZ_USBDEV
void pic32mz_usbsuspend(FAR struct usbdev_s *dev, bool resume);
#endif

/************************************************************************************

 * Name: pic32mz_usbattach and pic32mz_usbdetach
 *
 * Description:
 *   The USB stack must be notified when the device is attached or detached by
 *   calling one of these functions.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MZ_USBDEV
void pic32mz_usbattach(void);
void pic32mz_usbdetach(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_USBDEV_H */
