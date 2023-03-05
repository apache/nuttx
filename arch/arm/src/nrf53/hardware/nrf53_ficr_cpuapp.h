/***************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_ficr_cpuapp.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_FICR_CPUAPP_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_FICR_CPUAPP_H

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
                                                   /* TODO */

/* FICR Register Addresses *************************************************/

/* TODO */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_FICR_CPUAPP_H */
