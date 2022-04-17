/****************************************************************************
 * arch/arm/src/lpc54xx/hardware/lpc54_rom.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_ROM_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_ROM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Table offsets ************************************************************/

/* First level table offsets */

#define LPC54_USB_API_OFFSET                      0x0000
#define LPC54_OTP_API_OFFSET                      0x0038

/* USB API table offsets (to be provided) */

/* OTP API driver table offsets */

#define LPC54_OTP_API_INIT_OFFSET                 0x0000
#define LPC54_OTP_API_ENABLEBANKWRITEMASK_OFFSET  0x0004
#define LPC54_OTP_API_DISABLEBANKWRITEMASK_OFFSET 0x0008
#define LPC54_OTP_API_ENABLEBANKWRITELOCK_OFFSET  0x000c
#define LPC54_OTP_API_ENABLEBANKREADLOCK_OFFSET   0x0010
#define LPC54_OTP_API_PROGRAMREG_OFFSET           0x0014
#define LPC54_OTP_API_RNGREAD_OFFSET              0x002c
#define LPC54_OTP_API_GETDRIVERVERSION_OFFSET     0x0030

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Dereference the LPC54_ROM_DRIVERTAB address to get the address of the ROM
 * driver table. Not often that I get to use a
 * pointer-to-a-pointer-to-a-pointer.  The result of de referencing the
 * LPC54_ROM_DRIVERTAB is a pointer to an array of type uinptr_t *.
 */

#define lpc54_driver_vtable (*(uintptr_t ***)LPC54_ROM_DRIVERTAB)

/* Index the ROM driver table to get the specific driver table.  Perhaps in
 * the future these uintptr_t * arrays would be replaced with proper vtable
 * structures.
 */

#define lpc54_usb_vtable    lpc54_driver_vtable[LPC54_USB_API_OFFSET >> 2]
#define lpc54_otg_vtable    lpc54_driver_vtable[LPC54_OTP_API_OFFSET >> 2]

/* Then, finally, index the specific driver table to get the API entry
 * point
 */

/****************************************************************************
 * Public Types/Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  lpc54_rng_read
 *
 * Description:
 *   Returns a 32 bit random number from hardware.  The Random Number
 *   Generator is accessed through an API call located in the ROM.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Unsigned random number
 *
 ****************************************************************************/

typedef unsigned int (*rng_read_t)(void);

#define LPC54_RNG_READ ((rng_read_t)(lpc54_otg_vtable[LPC54_OTP_API_RNGREAD_OFFSET >> 2]))

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_ROM_H */
