/************************************************************************************
 * arch/avr/src/at90usb/chip.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_AVR_SRC_ATMEGA_CHIP_H
#define __ARCH_AVR_SRC_ATMEGA_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Define features for supported chip in the ATMEGA family */

#if defined(CONFIG_ARCH_CHIP_AT90USB646)
#  define AVR_FLASH_SIZE  (64*1024)
#  define AVR_SRAM_SIZE   (4*1024)
#  define AVR_EEPROM_SIZE (2*1024)
#  define HAVE_USBDEV     1
#  undef  HAVE_USBHOST
#  undef  HAVE_RAMPZ
#elif defined(CONFIG_ARCH_CHIP_AT90USB647)
#  define AVR_FLASH_SIZE  (64*1024)
#  define AVR_SRAM_SIZE   (4*1024)
#  define AVR_EEPROM_SIZE (2*1024)
#  define HAVE_USBDEV     1
#  define HAVE_USBHOST    1
#  undef HAVE_RAMPZ
#elif defined(CONFIG_ARCH_CHIP_AT90USB1286)
#  define AVR_FLASH_SIZE  (128*1024)
#  define AVR_SRAM_SIZE   (8*1024)
#  define AVR_EEPROM_SIZE (4*1024)
#  define HAVE_USBDEV     1
#  undef  HAVE_USBHOST
#  define HAVE_RAMPZ      1
#elif defined(CONFIG_ARCH_CHIP_AT90USB1287)
#  define AVR_FLASH_SIZE  (128*1024)
#  define AVR_SRAM_SIZE   (8*1024)
#  define AVR_EEPROM_SIZE (4*1024)
#  define HAVE_USBDEV     1
#  define HAVE_USBHOST    1
#  define HAVE_RAMPZ      1
#else
#  error "Unsupported AVR chip"
#endif

/* Include only the memory map.  Other chip hardware files should then include this
 * file for the proper setup
 */

#include "at90usb_memorymap.h"

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_ATMEGA_CHIP_H */
