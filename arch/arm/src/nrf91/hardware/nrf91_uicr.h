/***************************************************************************
 * arch/arm/src/nrf91/hardware/nrf91_uicr.h
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

#ifndef __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_UICR_H
#define __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_UICR_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf91_memorymap.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* UICR Register Offsets ***************************************************/

#define NRF91_UICR_APPROTECT_OFFSET         0x000  /* Access port protection */
#define NRF91_UICR_XOSC32M_OFFSET           0x014  /* Oscillator control */
#define NRF91_UICR_HFXOSRC_OFFSET           0x01C  /* HFXO clock source selection */
#define NRF91_UICR_HFXOCNT_OFFSET           0x020  /* HFXO startup counter */
#define NRF91_UICR_APPNVMCPOFGUARD_OFFSET   0x024  /* Enable blocking NVM WRITE and aborting NVM ERASE for Application NVM in POFWARN condition */
#define NRF91_UICR_SECUREAPPROTECT_OFFSET   0x02c  /* Secure access port protection */
#define NRF91_UICR_ERASEPROTECT_OFFSET      0x030  /* Erase protection */
#define NRF91_UICR_OTP_OFFSET               0x100  /* One time programmable memory */
#define NRF91_UICR_KEYSLOTCONFIGDEST_OFFSET 0x400  /* Key slot destination address */
#define NRF91_UICR_KEYSLOTCONFIGPERM_OFFSET 0x404  /* Permissions for the key slot */
#define NRF91_UICR_KEYSLOTKEYVALUE_OFFSET   0x800  /* Define bits [31+o*32:0+o*32] of value assigned to KMU key slot. */

/* UICR Register Addresses *************************************************/

#define NRF91_UICR_APPROTECT         (NRF91_UICR_BASE + NRF91_UICR_APPROTECT_OFFSET)
#define NRF91_UICR_XOSC32M           (NRF91_UICR_BASE + NRF91_UICR_XOSC32M_OFFSET)
#define NRF91_UICR_HFXOSRC           (NRF91_UICR_BASE + NRF91_UICR_HFXOSRC_OFFSET)
#define NRF91_UICR_HFXOCNT           (NRF91_UICR_BASE + NRF91_UICR_HFXOCNT_OFFSET)
#define NRF91_UICR_APPNVMCPOFGUARD   (NRF91_UICR_BASE + NRF91_UICR_APPNVMCPOFGUARD_OFFSET)
#define NRF91_UICR_SECUREAPPROTECT   (NRF91_UICR_BASE + NRF91_UICR_SECUREAPPROTECT_OFFSET)
#define NRF91_UICR_ERASEPROTECT      (NRF91_UICR_BASE + NRF91_UICR_ERASEPROTECT_OFFSET)
#define NRF91_UICR_OTP               (NRF91_UICR_BASE + NRF91_UICR_OTP_OFFSET)
#define NRF91_UICR_KEYSLOTCONFIGDEST (NRF91_UICR_BASE + NRF91_UICR_KEYSLOTCONFIGDEST_OFFSET)
#define NRF91_UICR_KEYSLOTCONFIGPERM (NRF91_UICR_BASE + NRF91_UICR_KEYSLOTCONFIGPERM_OFFSET)
#define NRF91_UICR_KEYSLOTKEYVALUE   (NRF91_UICR_BASE + NRF91_UICR_KEYSLOTKEYVALUE_OFFSET)

#endif /* __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_UICR_H */
