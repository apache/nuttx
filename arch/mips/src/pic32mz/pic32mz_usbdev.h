/************************************************************************************
 * arch/mips/src/pic32mz/pic32mz_usbdev.h
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
int pic32mz_usbpullup(struct usbdev_s *dev,  bool enable);
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
void pic32mz_usbsuspend(struct usbdev_s *dev, bool resume);
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
