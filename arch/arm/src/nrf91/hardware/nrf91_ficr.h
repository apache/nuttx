/***************************************************************************
 * arch/arm/src/nrf91/hardware/nrf91_ficr.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_FICR_H
#define __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_FICR_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf91_memorymap.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* FICR Register Offsets ***************************************************/

                                                   /* TODO: SIPINFO */
#define NRF91_FICR_INFO_DEVICEID0_OFFSET    0x204  /* Device identifier */
#define NRF91_FICR_INFO_DEVICEID1_OFFSET    0x208  /* Device identifier  */
#define NRF91_FICR_INFO_RAM_OFFSET          0x218  /* RAM variant */
#define NRF91_FICR_INFO_FLASH_OFFSET        0x21c  /* Flash variant */
#define NRF91_FICR_INFO_CODEPAGESIZE_OFFSET 0x220  /* Code memory page size in bytes */
#define NRF91_FICR_INFO_CODESIZE_OFFSET     0x224  /* Code memory size  */
#define NRF91_FICR_INFO_DEVICETYPE_OFFSET   0x228  /* Device type */
                                                   /* TODO */

#define NRF91_FICR_LAST_OFFSET              0xc1c

/* FICR Register Addresses *************************************************/

#define NRF91_FICR_INFO_DEVICEID0           (NRF91_FICR_BASE + NRF91_FICR_INFO_DEVICEID0_OFFSET)
#define NRF91_FICR_INFO_DEVICEID1           (NRF91_FICR_BASE + NRF91_FICR_INFO_DEVICEID1_OFFSET)
#define NRF91_FICR_INFO_RAM                 (NRF91_FICR_BASE + NRF91_FICR_INFO_RAM_OFFSET)
#define NRF91_FICR_INFO_FLASH               (NRF91_FICR_BASE + NRF91_FICR_INFO_FLASH_OFFSET)
#define NRF91_FICR_INFO_CODEPAGESIZE        (NRF91_FICR_BASE + NRF91_FICR_INFO_CODEPAGESIZE_OFFSET)
#define NRF91_FICR_INFO_CODESIZE            (NRF91_FICR_BASE + NRF91_FICR_INFO_CODESIZE_OFFSET)
#define NRF91_FICR_INFO_DEVICETYPE          (NRF91_FICR_BASE + NRF91_FICR_INFO_DEVICETYPE_OFFSET)

/* TODO */

#endif /* __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_FICR_H */
