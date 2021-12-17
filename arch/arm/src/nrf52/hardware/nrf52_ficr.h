/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_ficr.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_FICR_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_FICR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FICR Register Offsets ****************************************************/

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

/* FICR Register Addresses **************************************************/

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

/* FICR Register Bitfield Definitions ***************************************/

/* These values are ASCII encoded definitions of each four letter
 * combination. The datasheet does not list all values, so we added a few
 * more.
 */

#define NRF52_FICR_INFO_VARIANT_AAAA     (0x41414141)
#define NRF52_FICR_INFO_VARIANT_AAAB     (0x41414142)
#define NRF52_FICR_INFO_VARIANT_AABA     (0x41414241)
#define NRF52_FICR_INFO_VARIANT_AABB     (0x41414242)
#define NRF52_FICR_INFO_VARIANT_AAB0     (0x41414230)
#define NRF52_FICR_INFO_VARIANT_ABB0     (0x41424230)
#define NRF52_FICR_INFO_VARIANT_AAE0     (0x41414530)

/* Device address type */

#define FICR_DEVICEADDRTYPE_PUBLIC (0)
#define FICR_DEVICEADDRTYPE_RANDOM (1)

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_FICR_H */
