/********************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_rom.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_ROM_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_ROM_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Table offsets ****************************************************************************/

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

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/* Dereference the LPC54_ROM_DRIVERTAB address to get the address of the ROM driver table.
 * Not often that I get to use a pointer-to-a-pointer-to-a-pointer.  The result of de-
 * referencing the LPC54_ROM_DRIVERTAB is a pointer to an array of type uinptr_t *.
 */

#define lpc54_driver_vtable (*(uintptr_t ***)LPC54_ROM_DRIVERTAB)

/* Index the ROM driver table to get the specific driver table.  Perhaps in the future these
 * uintptr_t * arrays would be replaced with proper vtable structures.
 */

#define lpc54_usb_vtable    lpc54_driver_vtable[LPC54_USB_API_OFFSET >> 2]
#define lpc54_otg_vtable    lpc54_driver_vtable[LPC54_OTP_API_OFFSET >> 2]

/* Then, finally, index the specific driver table to get the API entry point */

/********************************************************************************************
 * Public Types/Functions
 ********************************************************************************************/

/********************************************************************************************
 * Name:  lpc54_rng_read
 *
 * Description:
 *   Returns a 32 bit random number from hardware.  The Random Number Generator is accessed
 *   through an API call located in the ROM.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Unsigned random number
 *
 ********************************************************************************************/

typedef CODE unsigned int (*rng_read_t)(void);

#define LPC54_RNG_READ ((rng_read_t)(lpc54_otg_vtable[LPC54_OTP_API_RNGREAD_OFFSET >> 2]))

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_ROM_H */
