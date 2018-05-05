/***************************************************************************************************
 * arch/arm/src/nrf52/chip/nrf52_ficr.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Alan Carvalho de Assis <acassis@gmail.com>
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
 ***************************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_CHIP_NRF52_FICR_H
#define __ARCH_ARM_SRC_NRF52_CHIP_NRF52_FICR_H

/***************************************************************************************************
 * Included Files
 ***************************************************************************************************/

#include <nuttx/config.h>
#include "chip/nrf52_memorymap.h"

/***************************************************************************************************
 * Pre-processor Definitions
 ***************************************************************************************************/

/* FICR Register Offsets ****************************************************************************/

/* Registers for the FICR */

#define NRF52_FICR_CODEPAGESIZE_OFFSET   0x010  /* Code memory page size */
#define NRF52_FICR_CODESIZE_OFFSET       0x014  /* Code memory size */

#define NRF52_FICR_DEVICEID0_OFFSET      0x060  /* Device identifier */
#define NRF52_FICR_DEVICEID1_OFFSET      0x064  /* Device identifier */

#define NRF52_FICR_ER0_OFFSET            0x080  /* Encryption Root, word 0 */
#define NRF52_FICR_ER1_OFFSET            0x084  /* Encryption Root, word 1 */
#define NRF52_FICR_ER2_OFFSET            0x088  /* Encryption Root, word 2 */
#define NRF52_FICR_ER3_OFFSET            0x08c  /* Encryption Root, word 3 */
#define NRF52_FICR_IR0_OFFSET            0x090  /* Identity Root, word 0 */
#define NRF52_FICR_IR1_OFFSET            0x094  /* Identity Root, word 1 */
#define NRF52_FICR_IR2_OFFSET            0x098  /* Identity Root, word 2 */
#define NRF52_FICR_IR3_OFFSET            0x09c  /* Identity Root, word 3 */
#define NRF52_FICR_DEVICEADDRTYPE_OFFSET 0x0a0  /* Device address type */
#define NRF52_FICR_DEVICEADDR0_OFFSET    0x0a4  /* Device address 0 */
#define NRF52_FICR_DEVICEADDR1_OFFSET    0x0a8  /* Device address 1 */

#define NRF52_FICR_INFO_PART_OFFSET      0x100  /* Part code */
#define NRF52_FICR_INFO_VARIANT_OFFSET   0x104  /* Part Variant, Hardware version and Production configuration */
#define NRF52_FICR_INFO_PACKAGE_OFFSET   0x108  /* Package option */
#define NRF52_FICR_INFO_RAM_OFFSET       0x10c  /* RAM variant */
#define NRF52_FICR_INFO_FLASH_OFFSET     0x110  /* Flash variant */

#define NRF52_FICR_TEMP_A0_OFFSET        0x404  /* Slope definition A0 */
#define NRF52_FICR_TEMP_A1_OFFSET        0x408  /* Slope definition A1 */
#define NRF52_FICR_TEMP_A2_OFFSET        0x40c  /* Slope definition A2 */
#define NRF52_FICR_TEMP_A3_OFFSET        0x410  /* Slope definition A3 */
#define NRF52_FICR_TEMP_A4_OFFSET        0x414  /* Slope definition A4 */
#define NRF52_FICR_TEMP_A5_OFFSET        0x418  /* Slope definition A5 */
#define NRF52_FICR_TEMP_B0_OFFSET        0x41c  /* y-intercept B0 */
#define NRF52_FICR_TEMP_B1_OFFSET        0x420  /* y-intercept B1 */
#define NRF52_FICR_TEMP_B2_OFFSET        0x424  /* y-intercept B2 */
#define NRF52_FICR_TEMP_B3_OFFSET        0x428  /* y-intercept B3 */
#define NRF52_FICR_TEMP_B4_OFFSET        0x42c  /* y-intercept B4 */
#define NRF52_FICR_TEMP_B5_OFFSET        0x430  /* y-intercept B5 */
#define NRF52_FICR_TEMP_T0_OFFSET        0x434  /* Segment end T0 */
#define NRF52_FICR_TEMP_T1_OFFSET        0x438  /* Segment end T1 */
#define NRF52_FICR_TEMP_T2_OFFSET        0x43c  /* Segment end T2 */
#define NRF52_FICR_TEMP_T3_OFFSET        0x440  /* Segment end T3 */
#define NRF52_FICR_TEMP_T4_OFFSET        0x444  /* Segment end T4 */

#define NRF52_FICR_NFC_TAGHEADER0_OFFSET 0x450  /* Default header for NFC Tag */
#define NRF52_FICR_NFC_TAGHEADER1_OFFSET 0x454  /* Default header for NFC Tag */
#define NRF52_FICR_NFC_TAGHEADER2_OFFSET 0x458  /* Default header for NFC Tag */
#define NRF52_FICR_NFC_TAGHEADER3_OFFSET 0x45c  /* Default header for NFC Tag */

/* FICR Register Addresses **************************************************************************/

#define NRF52_FICR_CODEPAGESIZE          (NRF52_FICR_BASE + NRF52_FICR_CODEPAGESIZE_OFFSET)
#define NRF52_FICR_CODESIZE              (NRF52_FICR_BASE + NRF52_FICR_CODESIZE_OFFSET)

#define NRF52_FICR_DEVICEID0             (NRF52_FICR_BASE + NRF52_FICR_DEVICEID0_OFFSET)
#define NRF52_FICR_DEVICEID1             (NRF52_FICR_BASE + NRF52_FICR_DEVICEID1_OFFSET)

#define NRF52_FICR_ER0                   (NRF52_FICR_BASE + NRF52_FICR_ER0_OFFSET)
#define NRF52_FICR_ER1                   (NRF52_FICR_BASE + NRF52_FICR_ER1_OFFSET)
#define NRF52_FICR_ER2                   (NRF52_FICR_BASE + NRF52_FICR_ER2_OFFSET)
#define NRF52_FICR_ER3                   (NRF52_FICR_BASE + NRF52_FICR_ER3_OFFSET)
#define NRF52_FICR_IR0                   (NRF52_FICR_BASE + NRF52_FICR_IR0_OFFSET)
#define NRF52_FICR_IR1                   (NRF52_FICR_BASE + NRF52_FICR_IR1_OFFSET)
#define NRF52_FICR_IR2                   (NRF52_FICR_BASE + NRF52_FICR_IR2_OFFSET)
#define NRF52_FICR_IR3                   (NRF52_FICR_BASE + NRF52_FICR_IR3_OFFSET)
#define NRF52_FICR_DEVICEADDRTYPE        (NRF52_FICR_BASE + NRF52_FICR_DEVICEADDRTYPE_OFFSET)
#define NRF52_FICR_DEVICEADDR0           (NRF52_FICR_BASE + NRF52_FICR_DEVICEADDR0_OFFSET)
#define NRF52_FICR_DEVICEADDR1           (NRF52_FICR_BASE + NRF52_FICR_DEVICEADDR1_OFFSET)

#define NRF52_FICR_INFO_PART             (NRF52_FICR_BASE + NRF52_FICR_INFO_PART_OFFSET)
#define NRF52_FICR_INFO_VARIANT          (NRF52_FICR_BASE + NRF52_FICR_INFO_VARIANT_OFFSET)
#define NRF52_FICR_INFO_PACKAGE          (NRF52_FICR_BASE + NRF52_FICR_INFO_PACKAGE_OFFSET)
#define NRF52_FICR_INFO_RAM              (NRF52_FICR_BASE + NRF52_FICR_INFO_RAM_OFFSET)
#define NRF52_FICR_INFO_FLASH            (NRF52_FICR_BASE + NRF52_FICR_INFO_FLASH_OFFSET)

#define NRF52_FICR_TEMP_A0               (NRF52_FICR_BASE + NRF52_FICR_TEMP_A0_OFFSET)
#define NRF52_FICR_TEMP_A1               (NRF52_FICR_BASE + NRF52_FICR_TEMP_A1_OFFSET)
#define NRF52_FICR_TEMP_A2               (NRF52_FICR_BASE + NRF52_FICR_TEMP_A2_OFFSET)
#define NRF52_FICR_TEMP_A3               (NRF52_FICR_BASE + NRF52_FICR_TEMP_A3_OFFSET)
#define NRF52_FICR_TEMP_A4               (NRF52_FICR_BASE + NRF52_FICR_TEMP_A4_OFFSET)
#define NRF52_FICR_TEMP_A5               (NRF52_FICR_BASE + NRF52_FICR_TEMP_A5_OFFSET)
#define NRF52_FICR_TEMP_B0               (NRF52_FICR_BASE + NRF52_FICR_TEMP_B0_OFFSET)
#define NRF52_FICR_TEMP_B1               (NRF52_FICR_BASE + NRF52_FICR_TEMP_B1_OFFSET)
#define NRF52_FICR_TEMP_B2               (NRF52_FICR_BASE + NRF52_FICR_TEMP_B2_OFFSET)
#define NRF52_FICR_TEMP_B3               (NRF52_FICR_BASE + NRF52_FICR_TEMP_B3_OFFSET)
#define NRF52_FICR_TEMP_B4               (NRF52_FICR_BASE + NRF52_FICR_TEMP_B4_OFFSET)
#define NRF52_FICR_TEMP_B5               (NRF52_FICR_BASE + NRF52_FICR_TEMP_B5_OFFSET)
#define NRF52_FICR_TEMP_T0               (NRF52_FICR_BASE + NRF52_FICR_TEMP_T0_OFFSET)
#define NRF52_FICR_TEMP_T1               (NRF52_FICR_BASE + NRF52_FICR_TEMP_T1_OFFSET)
#define NRF52_FICR_TEMP_T2               (NRF52_FICR_BASE + NRF52_FICR_TEMP_T2_OFFSET)
#define NRF52_FICR_TEMP_T3               (NRF52_FICR_BASE + NRF52_FICR_TEMP_T3_OFFSET)
#define NRF52_FICR_TEMP_T4               (NRF52_FICR_BASE + NRF52_FICR_TEMP_T4_OFFSET)

#define NRF52_FICR_NFC_TAGHEADER0        (NRF52_FICR_BASE + NRF52_FICR_NFC_TAGHEADER0_OFFSET)
#define NRF52_FICR_NFC_TAGHEADER1        (NRF52_FICR_BASE + NRF52_FICR_NFC_TAGHEADER1_OFFSET)
#define NRF52_FICR_NFC_TAGHEADER2        (NRF52_FICR_BASE + NRF52_FICR_NFC_TAGHEADER2_OFFSET)
#define NRF52_FICR_NFC_TAGHEADER3        (NRF52_FICR_BASE + NRF52_FICR_NFC_TAGHEADER3_OFFSET)

/* FICR Register Bitfield Definitions **************************************************************/

#define NRF52_FICR_READY_READY           (1 << 0) /* FICR is ready */

#define NRF52_FICR_CONFIG_WEN            (1 << 0) /* Enable write program memory */

#define NRF52_FICR_ICACHECNF_CACHEEN     (1 << 0) /* Cache enable */
#define NRF52_FICR_ICACHECNF_CACHEPROFEN (1 << 8) /* Cache profiling enable */

/* ENABLE Register */

/* INTENSET Register */

#endif /* __ARCH_ARM_SRC_NRF52_CHIP_NRF52_FICR_H */
