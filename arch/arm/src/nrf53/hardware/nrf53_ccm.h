/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_ccm.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CCM_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets for CCM *************************************************/

#define NRF53_CCM_TASKS_KSGEN_OFFSET            0x0000 /* Start KSGEN */
#define NRF53_CCM_TASKS_CRYPT_OFFSET            0x0004 /* Start crypt */
#define NRF53_CCM_TASKS_STOP_OFFSET             0x0008 /* Stop crypt */
#define NRF53_CCM_TASKS_RATEOVERRIDE_OFFSET     0x000c /* Override DATARATE */
#define NRF53_CCM_SUBSCRIBE_KSGEN_OFFSET        0x0080 /* Subscribe KSGEN */
#define NRF53_CCM_SUBSCRIBE_CRYPT_OFFSET        0x0084 /* Subscribe CRYPT */
#define NRF53_CCM_SUBSCRIBE_STOP_OFFSET         0x0088 /* Subscribe STOP */
#define NRF53_CCM_SUBSCRIBE_RATEOVERRIDE_OFFSET 0x008c /* Subscribe RATEOVR */
#define NRF53_CCM_EVENTS_ENDKSGEN_OFFSET        0x0100 /* Key-stream done */
#define NRF53_CCM_EVENTS_ENDCRYPT_OFFSET        0x0104 /* Crypt done */
#define NRF53_CCM_EVENTS_ERROR_OFFSET           0x0108 /* Deprecated */
#define NRF53_CCM_PUBLISH_ENDKSGEN_OFFSET       0x0180 /* Publish ENDKSGEN */
#define NRF53_CCM_PUBLISH_ENDCRYPT_OFFSET       0x0184 /* Publish ENDCRYPT */
#define NRF53_CCM_PUBLISH_ERROR_OFFSET          0x0188 /* Publish ERROR */
#define NRF53_CCM_SHORTS_OFFSET                 0x0200 /* Shortcuts */
#define NRF53_CCM_INTENSET_OFFSET               0x0304 /* Enable interrupt */
#define NRF53_CCM_INTENCLR_OFFSET               0x0308 /* Disable interrupt */
#define NRF53_CCM_MICSTATUS_OFFSET              0x0400 /* MIC check result */
#define NRF53_CCM_ENABLE_OFFSET                 0x0500 /* Enable */
#define NRF53_CCM_MODE_OFFSET                   0x0504 /* Operation mode */
#define NRF53_CCM_CNFPTR_OFFSET                 0x0508 /* Config pointer */
#define NRF53_CCM_INPTR_OFFSET                  0x050c /* Input pointer */
#define NRF53_CCM_OUTPTR_OFFSET                 0x0510 /* Output pointer */
#define NRF53_CCM_SCRATCHPTR_OFFSET             0x0514 /* Scratch pointer */
#define NRF53_CCM_MAXPACKETSIZE_OFFSET          0x0518 /* Max packet size */
#define NRF53_CCM_RATEOVERRIDE_OFFSET           0x051c /* Data rate override */
#define NRF53_CCM_HEADERMASK_OFFSET             0x0520 /* Header (S0) mask */

/* Register Addresses for CCM ***********************************************/

#define NRF53_CCM_TASKS_KSGEN                   (NRF53_CCM_BASE + NRF53_CCM_TASKS_KSGEN_OFFSET)
#define NRF53_CCM_TASKS_CRYPT                   (NRF53_CCM_BASE + NRF53_CCM_TASKS_CRYPT_OFFSET)
#define NRF53_CCM_TASKS_STOP                    (NRF53_CCM_BASE + NRF53_CCM_TASKS_STOP_OFFSET)
#define NRF53_CCM_TASKS_RATEOVERRIDE            (NRF53_CCM_BASE + NRF53_CCM_TASKS_RATEOVERRIDE_OFFSET)
#define NRF53_CCM_SUBSCRIBE_KSGEN               (NRF53_CCM_BASE + NRF53_CCM_SUBSCRIBE_KSGEN_OFFSET)
#define NRF53_CCM_SUBSCRIBE_CRYPT               (NRF53_CCM_BASE + NRF53_CCM_SUBSCRIBE_CRYPT_OFFSET)
#define NRF53_CCM_SUBSCRIBE_STOP                (NRF53_CCM_BASE + NRF53_CCM_SUBSCRIBE_STOP_OFFSET)
#define NRF53_CCM_SUBSCRIBE_RATEOVERRIDE        (NRF53_CCM_BASE + NRF53_CCM_SUBSCRIBE_RATEOVERRIDE_OFFSET)
#define NRF53_CCM_EVENTS_ENDKSGEN               (NRF53_CCM_BASE + NRF53_CCM_EVENTS_ENDKSGEN_OFFSET)
#define NRF53_CCM_EVENTS_ENDCRYPT               (NRF53_CCM_BASE + NRF53_CCM_EVENTS_ENDCRYPT_OFFSET)
#define NRF53_CCM_EVENTS_ERROR                  (NRF53_CCM_BASE + NRF53_CCM_EVENTS_ERROR_OFFSET)
#define NRF53_CCM_PUBLISH_ENDKSGEN              (NRF53_CCM_BASE + NRF53_CCM_PUBLISH_ENDKSGEN_OFFSET)
#define NRF53_CCM_PUBLISH_ENDCRYPT              (NRF53_CCM_BASE + NRF53_CCM_PUBLISH_ENDCRYPT_OFFSET)
#define NRF53_CCM_PUBLISH_ERROR                 (NRF53_CCM_BASE + NRF53_CCM_PUBLISH_ERROR_OFFSET)
#define NRF53_CCM_SHORTS                        (NRF53_CCM_BASE + NRF53_CCM_SHORTS_OFFSET)
#define NRF53_CCM_INTENSET                      (NRF53_CCM_BASE + NRF53_CCM_INTENSET_OFFSET)
#define NRF53_CCM_INTENCLR                      (NRF53_CCM_BASE + NRF53_CCM_INTENCLR_OFFSET)
#define NRF53_CCM_MICSTATUS                     (NRF53_CCM_BASE + NRF53_CCM_MICSTATUS_OFFSET)
#define NRF53_CCM_ENABLE                        (NRF53_CCM_BASE + NRF53_CCM_ENABLE_OFFSET)
#define NRF53_CCM_MODE                          (NRF53_CCM_BASE + NRF53_CCM_MODE_OFFSET)
#define NRF53_CCM_CNFPTR                        (NRF53_CCM_BASE + NRF53_CCM_CNFPTR_OFFSET)
#define NRF53_CCM_INPTR                         (NRF53_CCM_BASE + NRF53_CCM_INPTR_OFFSET)
#define NRF53_CCM_OUTPTR                        (NRF53_CCM_BASE + NRF53_CCM_OUTPTR_OFFSET)
#define NRF53_CCM_SCRATCHPTR                    (NRF53_CCM_BASE + NRF53_CCM_SCRATCHPTR_OFFSET)
#define NRF53_CCM_MAXPACKETSIZE                 (NRF53_CCM_BASE + NRF53_CCM_MAXPACKETSIZE_OFFSET)
#define NRF53_CCM_RATEOVERRIDE                  (NRF53_CCM_BASE + NRF53_CCM_RATEOVERRIDE_OFFSET)
#define NRF53_CCM_HEADERMASK                    (NRF53_CCM_BASE + NRF53_CCM_HEADERMASK_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SHORTS Register */

#define CCM_SHORTS_ENDKSGEN_CRYPT   (1 << 0)  /* Bit 0: ENDKSGEN -> CRYPT */

/* INTENSET/INTENCLR Register */

#define CCM_INT_ENDKSGEN            (1 << 0)  /* Bit 0: ENDKSGEN event */
#define CCM_INT_ENDCRYPT            (1 << 1)  /* Bit 1: ENDCRYPT event */
#define CCM_INT_ERROR               (1 << 2)  /* Bit 2: ERROR event */

/* MICSTATUS Register */

#define CCM_MICSTATUS_PASSED        (1 << 0)  /* Bit 0: MIC check passed */

/* ENABLE Register */

#define CCM_ENABLE_SHIFT            (0)       /* Bits 0-1: Enable CCM */
#define CCM_ENABLE_MASK             (0x3 << CCM_ENABLE_SHIFT)
#  define CCM_ENABLE_DISABLED       (0 << CCM_ENABLE_SHIFT)
#  define CCM_ENABLE_ENABLED        (2 << CCM_ENABLE_SHIFT)

/* MODE Register */

#define CCM_MODE_DECRYPTION         (1 << 0)  /* Bit 0: Decryption mode */
#define CCM_MODE_DATARATE_SHIFT     (16)      /* Bits 16-17: Radio data rate */
#define CCM_MODE_DATARATE_MASK      (0x3 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_1MBIT   (0 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_2MBIT   (1 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_125KBPS (2 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_500KBPS (3 << CCM_MODE_DATARATE_SHIFT)
#define CCM_MODE_LENGTH_EXTENDED    (1 << 24) /* Bit 24: Extended length */

/* MAXPACKETSIZE Register */

#define CCM_MAXPACKETSIZE_SHIFT     (0)       /* Bits 0-7: Max size */
#define CCM_MAXPACKETSIZE_MASK      (0xff << CCM_MAXPACKETSIZE_SHIFT)

/* RATEOVERRIDE Register */

#define CCM_RATEOVERRIDE_SHIFT      (0)       /* Bits 0-1: Data rate */
#define CCM_RATEOVERRIDE_MASK       (0x3 << CCM_RATEOVERRIDE_SHIFT)
#  define CCM_RATEOVERRIDE_1MBIT    (0 << CCM_RATEOVERRIDE_SHIFT)
#  define CCM_RATEOVERRIDE_2MBIT    (1 << CCM_RATEOVERRIDE_SHIFT)
#  define CCM_RATEOVERRIDE_125KBPS  (2 << CCM_RATEOVERRIDE_SHIFT)
#  define CCM_RATEOVERRIDE_500KBPS  (3 << CCM_RATEOVERRIDE_SHIFT)

/* HEADERMASK Register */

#define CCM_HEADERMASK_SHIFT        (0)       /* Bits 0-7: Header (S0) mask */
#define CCM_HEADERMASK_MASK         (0xff << CCM_HEADERMASK_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CCM_H */
