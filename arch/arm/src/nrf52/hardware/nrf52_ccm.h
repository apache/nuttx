/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_ccm.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_CCM_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets for CCM *************************************************/

#define NRF52_CCM_TASKS_KSGEN_OFFSET        0x0000 /* Start generation of key-stream */
#define NRF52_CCM_TASKS_CRYPT_OFFSET        0x0004 /* Start encryption/decryption */
#define NRF52_CCM_TASKS_STOP_OFFSET         0x0008 /* Stop encryption/decryption */
#define NRF52_CCM_TASKS_RATEOVERRIDE_OFFSET 0x000c /* Override DATARATE setting */
#define NRF52_CCM_EVENTS_ENDKSGEN_OFFSET    0x0100 /* Key-stream generation complete */
#define NRF52_CCM_EVENTS_ENDCRYPT_OFFSET    0x0104 /* Encrypt/decrypt complete */
#define NRF52_CCM_EVENTS_ERROR_OFFSET       0x0108 /* Deprecated */
#define NRF52_CCM_SHORTS_OFFSET             0x0200 /* Shortcuts between local events and tasks */
#define NRF52_CCM_INTENSET_OFFSET           0x0304 /* Enable interrupt */
#define NRF52_CCM_INTENCLR_OFFSET           0x0308 /* Disable interrupt */
#define NRF52_CCM_MICSTATUS_OFFSET          0x0400 /* MIC check result */
#define NRF52_CCM_ENABLE_OFFSET             0x0500 /* Enable */
#define NRF52_CCM_MODE_OFFSET               0x0504 /* Operation mode */
#define NRF52_CCM_CNFPTR_OFFSET             0x0508 /* Pointer to data structure */
#define NRF52_CCM_INPTR_OFFSET              0x050c /* Input pointer */
#define NRF52_CCM_OUTPTR_OFFSET             0x0510 /* Output pointer */
#define NRF52_CCM_SCRATCHPTR_OFFSET         0x0514 /* Pointer to scratch area */
#define NRF52_CCM_MAXPACKETSIZE_OFFSET      0x0518 /* Length of key-stream for packet length 0 */
#define NRF52_CCM_RATEOVERRIDE_OFFSET       0x051c /* Data rate override setting */

/* Register Addresses for CCM ***********************************************/

#define NRF52_CCM_TASKS_KSGEN               (NRF52_CCM_BASE + NRF52_CCM_TASKS_KSGEN_OFFSET)
#define NRF52_CCM_TASKS_CRYPT               (NRF52_CCM_BASE + NRF52_CCM_TASKS_CRYPT_OFFSET)
#define NRF52_CCM_TASKS_STOP                (NRF52_CCM_BASE + NRF52_CCM_TASKS_STOP_OFFSET)
#define NRF52_CCM_TASKS_RATEOVERRIDE        (NRF52_CCM_BASE + NRF52_CCM_TASKS_RATEOVERRIDE_OFFSET)
#define NRF52_CCM_EVENTS_ENDKSGEN           (NRF52_CCM_BASE + NRF52_CCM_EVENTS_ENDKSGEN_OFFSET)
#define NRF52_CCM_EVENTS_ENDCRYPT           (NRF52_CCM_BASE + NRF52_CCM_EVENTS_ENDCRYPT_OFFSET)
#define NRF52_CCM_EVENTS_ERROR              (NRF52_CCM_BASE + NRF52_CCM_EVENTS_ERROR_OFFSET)
#define NRF52_CCM_SHORTS                    (NRF52_CCM_BASE + NRF52_CCM_SHORTS_OFFSET)
#define NRF52_CCM_INTENSET                  (NRF52_CCM_BASE + NRF52_CCM_INTENSET_OFFSET)
#define NRF52_CCM_INTENCLR                  (NRF52_CCM_BASE + NRF52_CCM_INTENCLR_OFFSET)
#define NRF52_CCM_MICSTATUS                 (NRF52_CCM_BASE + NRF52_CCM_MICSTATUS_OFFSET)
#define NRF52_CCM_ENABLE                    (NRF52_CCM_BASE + NRF52_CCM_ENABLE_OFFSET)
#define NRF52_CCM_MODE                      (NRF52_CCM_BASE + NRF52_CCM_MODE_OFFSET)
#define NRF52_CCM_CNFPTR                    (NRF52_CCM_BASE + NRF52_CCM_CNFPTR_OFFSET)
#define NRF52_CCM_INPTR                     (NRF52_CCM_BASE + NRF52_CCM_INPTR_OFFSET)
#define NRF52_CCM_OUTPTR                    (NRF52_CCM_BASE + NRF52_CCM_OUTPTR_OFFSET)
#define NRF52_CCM_SCRATCHPTR                (NRF52_CCM_BASE + NRF52_CCM_SCRATCHPTR_OFFSET)
#define NRF52_CCM_MAXPACKETSIZE             (NRF52_CCM_BASE + NRF52_CCM_MAXPACKETSIZE_OFFSET)
#define NRF52_CCM_RATEOVERRIDE              (NRF52_CCM_BASE + NRF52_CCM_RATEOVERRIDE_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SHORTS Register */

#define CCM_SHORTS_ENDKSGEN_CRYPT           (1 << 0) /* Bit 0: ENDKSGEN -> CRYPT */

/* INTENSET/INTENCLR Register */

#define CCM_INT_ENDKSGEN                    (1 << 0) /* Bit 0: ENDKSGEN event */
#define CCM_INT_ENDCRYPT                    (1 << 1) /* Bit 1: ENDCRYPT event */
#define CCM_INT_ERROR                       (1 << 2) /* Bit 2: ERROR event */

/* MICSTATUS Register */

#define CCM_MICSTATUS_PASSED                (1 << 0) /* Bit 0: MIC check passed */

/* ENABLE Register */

#define CCM_ENABLE_SHIFT                    (0)      /* Bits 0-1: Enable CCM */
#define CCM_ENABLE_MASK                     (0x3 << CCM_ENABLE_SHIFT)
#  define CCM_ENABLE_DISABLED               (0 << CCM_ENABLE_SHIFT)
#  define CCM_ENABLE_ENABLED                (2 << CCM_ENABLE_SHIFT)

/* MODE Register */

#define CCM_MODE_DECRYPTION                 (1 << 0) /* Bit 0: Decryption mode */
#define CCM_MODE_DATARATE_SHIFT             (16)     /* Bits 16-17: Radio data rate */
#define CCM_MODE_DATARATE_MASK              (0x3 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_1MBIT           (0 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_2MBIT           (1 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_125KBPS         (2 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_500KBPS         (3 << CCM_MODE_DATARATE_SHIFT)
#define CCM_MODE_LENGTH_EXTENDED            (1 << 24) /* Bit 24: Extended packet length */

/* MAXPACKETSIZE Register */

#define CCM_MAXPACKETSIZE_SHIFT             (0)      /* Bits 0-7: Maximum packet size */
#define CCM_MAXPACKETSIZE_MASK              (0xff << CCM_MAXPACKETSIZE_SHIFT)

/* RATEOVERRIDE Register */

#define CCM_RATEOVERRIDE_SHIFT              (0)      /* Bits 0-1: Data rate override */
#define CCM_RATEOVERRIDE_MASK               (0x3 << CCM_RATEOVERRIDE_SHIFT)
#  define CCM_RATEOVERRIDE_1MBIT            (0 << CCM_RATEOVERRIDE_SHIFT)
#  define CCM_RATEOVERRIDE_2MBIT            (1 << CCM_RATEOVERRIDE_SHIFT)
#  define CCM_RATEOVERRIDE_125KBPS          (2 << CCM_RATEOVERRIDE_SHIFT)
#  define CCM_RATEOVERRIDE_500KBPS          (3 << CCM_RATEOVERRIDE_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_CCM_H */
