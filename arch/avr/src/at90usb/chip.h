/************************************************************************************
 * arch/avr/src/at90usb/chip.h
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

#ifndef __ARCH_AVR_SRC_AT90USB_CHIP_H
#define __ARCH_AVR_SRC_AT90USB_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/* Include only the memory map.  Other chip hardware files should then include this
 * file for the proper setup
 */

#include "at90usb_memorymap.h"

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

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions Prototypes
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT90USB_CHIP_H */
