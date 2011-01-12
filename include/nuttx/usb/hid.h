/****************************************************************************
 * include/nuttx/usb/hid.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References:
 *   Universal Serial Bus (USB), Device Class Definition for Human Interface
 *   Devices (HID), Firmware Specification—6/27/01, Version 1.11.
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

#ifndef __INCLUDE_NUTTX_USB_HID_H
#define __INCLUDE_NUTTX_USB_HID_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* Subclass and Protocol ****************************************************/
/* Subclass codes (4.2) */

#define USBHID_SUBCLASS_NONE       0 /* No subclass */
#define USBHID_SUBCLASS_BOOTIF     1 /* Boot Interface Subclass */

/* A variety of protocols are supported HID devices. The protocol member of
 * an Interface descriptor only has meaning if the subclass member declares
 * that the device supports a boot interface, otherwise it is 0. (4.3)
 */

#define USBHID_PROTOCOL_NONE       0
#define USBHID_PROTOCOL_KEYBOARD   1
#define USBHID_PROTOCOL_MOUSE      2

/* HID Descriptor ***********************************************************/

#define USBHID_COUNTRY_NONE        0x00 /* Not Supported */
#define USBHID_COUNTRY_ARABIC      0x01 /* Arabic */
#define USBHID_COUNTRY_BELGIAN     0x02 /* Belgian */
#define USBHID_COUNTRY_CANADA      0x03 /* Canadian-Bilingual */
#define USBHID_COUNTRY_CANADRFR    0x04 /* Canadian-French */
#define USBHID_COUNTRY_CZECH       0x05 /* Czech Republic */
#define USBHID_COUNTRY_DANISH      0x06 /* Danish */
#define USBHID_COUNTRY_FINNISH     0x07 /* Finnish */
#define USBHID_COUNTRY_FRENCH      0x08 /* French */
#define USBHID_COUNTRY_GERMAN      0x09 /* German */
#define USBHID_COUNTRY_GREEK       0x10 /* Greek */
#define USBHID_COUNTRY_HEBREW      0x11 /* Hebrew */
#define USBHID_COUNTRY_HUNGARY     0x12 /* Hungary */
#define USBHID_COUNTRY_ISO         0x13 /* International (ISO) */
#define USBHID_COUNTRY_ITALIAN     0x14 /* Italian */
#define USBHID_COUNTRY_JAPAN       0x15 /* Japan (Katakana) */
#define USBHID_COUNTRY_KOREAN      0x16 /* Korean  */
#define USBHID_COUNTRY_LATINAM     0x17 /* Latin American */
#define USBHID_COUNTRY_DUTCH       0x18 /* Netherlands/Dutch */
#define USBHID_COUNTRY_NORWEGIAN   0x19 /* Norwegian */
#define USBHID_COUNTRY_PERSIAN     0x20 /* Persian (Farsi) */
#define USBHID_COUNTRY_POLAND      0x21 /* Poland */
#define USBHID_COUNTRY_PORTUGUESE  0x22 /* Portuguese */
#define USBHID_COUNTRY_RUSSIA      0x23 /* Russia */
#define USBHID_COUNTRY_SLOVAKIA    0x24 /* Slovakia */
#define USBHID_COUNTRY_SPANISH     0x25 /* Spanish */
#define USBHID_COUNTRY_SWEDISH     0x26 /* Swedish */
#define USBHID_COUNTRY_SWISSFR     0x27 /* Swiss/French */
#define USBHID_COUNTRY_SWISSGR     0x28 /* Swiss/German */
#define USBHID_COUNTRY_SWITZERLAND 0x29 /* Switzerland */
#define USBHID_COUNTRY_TAIWAN      0x30 /* Taiwan */
#define USBHID_COUNTRY_TURKISHQ    0x31 /* Turkish-Q */
#define USBHID_COUNTRY_UK          0x32 /* UK */
#define USBHID_COUNTRY_US          0x33 /* US */
#define USBHID_COUNTRY_YUGOSLAVIA  0x34 /* Yugoslavia */
#define USBHID_COUNTRY_TURKISHF    0x35 /* Turkish-F */

/* Main Items (6.2.2.4) */

#define USBHID_MAIN_SIZE(pfx)             ((pfx) & 3)
#define USBHID_MAIN_INPUT_PREFIX          0x80
#define USBHID_MAIN_INPUT_CONSTANT        (1 << 0) /* Constant(1) vs Data(0) */
#define USBHID_MAIN_INPUT_VARIABLE        (1 << 1) /* Variable(1) vs Array(0) */
#define USBHID_MAIN_INPUT_RELATIVE        (1 << 2) /* Relative(1) vs Absolute(0) */
#define USBHID_MAIN_INPUT_WRAP            (1 << 3) /* Wrap(1) vs No Wrap(0) */
#define USBHID_MAIN_INPUT_NONLINEAR       (1 << 4) /* Non Linear(1) vs Linear(0) */
#define USBHID_MAIN_INPUT_NOPREFERRED     (1 << 5) /* No Preferred (1) vs Preferred State(0) */
#define USBHID_MAIN_INPUT_NULLSTATE       (1 << 6) /* Null state(1) vs No Null position(0) */
#define USBHID_MAIN_INPUT_BUFFEREDBYTES   (1 << 8) /* Buffered Bytes(1) vs Bit Field(0) */

#define USBHID_MAIN_OUTPUT_PREFIX         0x90
#define USBHID_MAIN_OUTPUT_CONSTANT       (1 << 0) /* Constant(1) vs Data(0) */
#define USBHID_MAIN_OUTPUT_VARIABLE       (1 << 1) /* Variable(1) vs Array(0) */
#define USBHID_MAIN_OUTPUT_RELATIVE       (1 << 2) /* Relative(1) vs Absolute(0) */
#define USBHID_MAIN_OUTPUT_WRAP           (1 << 3) /* Wrap(1) vs No Wrap(0) */
#define USBHID_MAIN_OUTPUT_NONLINEAR      (1 << 4) /* Non Linear(1) vs Linear(0) */
#define USBHID_MAIN_OUTPUT_NOPREFERRED    (1 << 5) /* No Preferred (1) vs Preferred State(0) */
#define USBHID_MAIN_OUTPUT_NULLSTATE      (1 << 6) /* Null state(1) vs No Null position(0) */
#define USBHID_MAIN_OUTPUT_VOLATILE       (1 << 7) /* Volatile(1) vs Non volatile(0) */
#define USBHID_MAIN_OUTPUT_BUFFEREDBYTES  (1 << 8) /* Buffered Bytes(1) vs Bit Field(0) */

#define USBHID_MAIN_FEATURE_PREFIX        0xb0
#define USBHID_MAIN_FEATURE_CONSTANT      (1 << 0) /* Constant(1) vs Data(0) */
#define USBHID_MAIN_FEATURE_VARIABLE      (1 << 1) /* Variable(1) vs Array(0) */
#define USBHID_MAIN_FEATURE_RELATIVE      (1 << 2) /* Relative(1) vs Absolute(0) */
#define USBHID_MAIN_FEATURE_WRAP          (1 << 3) /* Wrap(1) vs No Wrap(0) */
#define USBHID_MAIN_FEATURE_NONLINEAR     (1 << 4) /* Non Linear(1) vs Linear(0) */
#define USBHID_MAIN_FEATURE_NOPREFERRED   (1 << 5) /* No Preferred (1) vs Preferred State(0) */
#define USBHID_MAIN_FEATURE_NULLSTATE     (1 << 6) /* Null state(1) vs No Null position(0) */
#define USBHID_MAIN_FEATURE_VOLATILE      (1 << 7) /* Volatile(1) vs Non volatile(0) */
#define USBHID_MAIN_FEATURE_BUFFEREDBYTES (1 << 8) /* Buffered Bytes(1) vs Bit Field(0) */

#define USBHID_MAIN_COLLECTION_PREFIX     0xa0
#define USBHID_MAIN_COLLECTION_PHYSICAL   0x00 /* Physical (group of axes) */
#define USBHID_MAIN_COLLECTION_APPL       0x01 /* Application (mouse, keyboard) */
#define USBHID_MAIN_COLLECTION_LOGICAL    0x02 /* Logical (interrelated data) */
#define USBHID_MAIN_COLLECTION_REPORT     0x03 /* Report */
#define USBHID_MAIN_COLLECTION_ARRAY      0x04 /* Named Array */
#define USBHID_MAIN_COLLECTION_SWITCH     0x05 /* Usage Switch */
#define USBHID_MAIN_COLLECTION_MODIFIER   0x06 /* Usage Modifier */

#define USBHID_MAIN_ENDCOLLECTION_PREFIX  0xc0

/* Global Items (6.2.2.7) */

#define USBHID_GLOBAL_SIZE(pfx)           ((pfx) & 3)
#define USBHID_GLOBAL_USAGEPAGE_PREFIX    0x04 /* Usage Page */
#define USBHID_GLOBAL_LOGMINIMUM_PREFIX   0x14 /* Logical Minimum */
#define USBHID_GLOBAL_LOGMAXIMUM_PREFIX   0x24 /* Logical Maximum */
#define USBHID_GLOBAL_PHYSMINIMUM_PREFIX  0x34 /* Physical Minimum */
#define USBHID_GLOBAL_PHYSMAXIMUM_PREFIX  0x44 /* Physical Maximum */
#define USBHID_GLOBAL_UNITEXP_PREFIX      0x54 /* Unit Exponent */
#define USBHID_GLOBAL_UNIT_PREFIX         0x64 /* Unit */
#define USBHID_GLOBAL_REPORTSIZE_PREFIX   0x74 /* Report Size */
#define USBHID_GLOBAL_REPORTID_PREFIX     0x84 /*Report ID */
#define USBHID_GLOBAL_REPORTCOUNT_PREFIX  0x94 /* Report Count */
#define USBHID_GLOBAL_PUSH_PREFIX         0xa4 /* Push */
#define USBHID_GLOBAL_Pop_PREFIX          0xb4 /* Pop */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_HID_H */
