/***************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_uicr.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_UICR_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_UICR_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* UICR Register Offsets ***************************************************/

#define NRF53_UICR_APPROTECT_OFFSET         0x000  /* Access port protection */
#define NRF53_UICR_EXTSUPPLY_OFFSET         0x00C  /* Enable external circuitry to be supplied from VDD pin. */
#define NRF53_UICR_VREGHVOUT_OFFSET         0x010  /* GPIO reference voltage / external output supply voltage in High voltage mode */
#define NRF53_UICR_HFXOCNT_OFFSET           0x014  /* HFXO startup counter */
#define NRF53_UICR_SECUREAPPROTECT_OFFSET   0x01C  /* Secure access port protection */
#define NRF53_UICR_ERASEPROTECT_OFFSET      0x020  /* Erase protection */
#define NRF53_UICR_TINSTANCE_OFFSET         0x024  /* SW-DP Target instance */
#define NRF53_UICR_NFCPINS_OFFSET           0x028  /* Setting of pins dedicated to NFC functionality: NFC antenna or GPIO */
#define NRF53_UICR_OTP_OFFSET               0x100  /* One time programmable memory */
#define NRF53_UICR_KEYSLOTCONFIGDEST_OFFSET 0x400  /* Key slot destination address */
#define NRF53_UICR_KEYSLOTCONFIGPERM_OFFSET 0x404  /* Permissions for the key slot */
#define NRF53_UICR_KEYSLOTKEYVALUE_OFFSET   0x800  /* Define bits [31+o*32:0+o*32] of value assigned to KMU key slot. */

/* UICR Register Addresses *************************************************/

#define NRF53_UICR_APPROTECT         (NRF53_UICR_BASE + NRF53_UICR_APPROTECT_OFFSET)
#define NRF53_UICR_EXTSUPPLY         (NRF53_UICR_BASE + NRF53_UICR_EXTSUPPLY_OFFSET)
#define NRF53_UICR_VREGHVOUT         (NRF53_UICR_BASE + NRF53_UICR_VREGHVOUT_OFFSET)
#define NRF53_UICR_HFXOCNT           (NRF53_UICR_BASE + NRF53_UICR_HFXOCNT_OFFSET)
#define NRF53_UICR_SECUREAPPROTECT   (NRF53_UICR_BASE + NRF53_UICR_SECUREAPPROTECT_OFFSET)
#define NRF53_UICR_ERASEPROTECT      (NRF53_UICR_BASE + NRF53_UICR_ERASEPROTECT_OFFSET)
#define NRF53_UICR_TINSTANCE         (NRF53_UICR_BASE + NRF53_UICR_TINSTANCE_OFFSET)
#define NRF53_UICR_NFCPINS           (NRF53_UICR_BASE + NRF53_UICR_NFCPINS_OFFSET)
#define NRF53_UICR_OTP               (NRF53_UICR_BASE + NRF53_UICR_OTP_OFFSET)
#define NRF53_UICR_KEYSLOTCONFIGDEST (NRF53_UICR_BASE + NRF53_UICR_KEYSLOTCONFIGDEST_OFFSET)
#define NRF53_UICR_KEYSLOTCONFIGPERM (NRF53_UICR_BASE + NRF53_UICR_KEYSLOTCONFIGPERM_OFFSET)
#define NRF53_UICR_KEYSLOTKEYVALUE   (NRF53_UICR_BASE + NRF53_UICR_KEYSLOTKEYVALUE_OFFSET)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_UICR_H */
