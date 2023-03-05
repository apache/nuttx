/***************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_ficr_cpunet.h
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
 ***************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_FICR_NET_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_FICR_NET_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* FICR Register Offsets ***************************************************/

#define NRF53_FICR_INFO_CONFIGID_OFFSET     0x200  /* Configuration identifier */
#define NRF53_FICR_INFO_DEVICEID0_OFFSET    0x204  /* Device identifier */
#define NRF53_FICR_INFO_DEVICEID1_OFFSET    0x208  /* Device identifier  */
#define NRF53_FICR_INFO_PART_OFFSET         0x20c  /* Part code */
#define NRF53_FICR_INFO_VARIANT_OFFSET      0x210  /* Part Variant, Hardware version and Production configuration */
#define NRF53_FICR_INFO_PACKAGE_OFFSET      0x214  /* Package option */
#define NRF53_FICR_INFO_RAM_OFFSET          0x218  /* RAM variant */
#define NRF53_FICR_INFO_FLASH_OFFSET        0x21c  /* Flash variant */
#define NRF53_FICR_INFO_CODEPAGESIZE_OFFSET 0x220  /* Code memory page size in bytes */
#define NRF53_FICR_INFO_CODESIZE_OFFSET     0x224  /* Code memory size  */
#define NRF53_FICR_INFO_DEVICETYPE_OFFSET   0x228  /* Device type */
#define NRF52_FICR_ER0_OFFSET               0x280  /* Encryption Root, word 0 */
#define NRF52_FICR_ER1_OFFSET               0x284  /* Encryption Root, word 1 */
#define NRF52_FICR_ER2_OFFSET               0x288  /* Encryption Root, word 2 */
#define NRF52_FICR_ER3_OFFSET               0x28c  /* Encryption Root, word 3 */
#define NRF52_FICR_IR0_OFFSET               0x290  /* Identity Root, word 0 */
#define NRF52_FICR_IR1_OFFSET               0x294  /* Identity Root, word 1 */
#define NRF52_FICR_IR2_OFFSET               0x298  /* Identity Root, word 2 */
#define NRF52_FICR_IR3_OFFSET               0x29c  /* Identity Root, word 3 */
#define NRF52_FICR_DEVICEADDRTYPE_OFFSET    0x2a0  /* Device address type */
#define NRF52_FICR_DEVICEADDR0_OFFSET       0x2a4  /* Device address 0 */
#define NRF52_FICR_DEVICEADDR1_OFFSET       0x2a8  /* Device address 1 */

/* FICR Register Addresses *************************************************/

/* TODO */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_FICR_NET_H */
