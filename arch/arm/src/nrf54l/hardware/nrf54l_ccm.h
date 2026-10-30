/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_ccm.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_CCM_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for CCM *************************************************/

#define NRF54L_CCM_TASKS_START_OFFSET        0x0000               /* Start encryption/decryption */
#define NRF54L_CCM_TASKS_STOP_OFFSET         0x0004               /* Stop encryption/decryption */
#define NRF54L_CCM_TASKS_RATEOVERRIDE_OFFSET 0x0008               /* Override DATARATE */
#define NRF54L_CCM_SUBSCRIBE_START_OFFSET    0x0080               /* Subscribe to START */
#define NRF54L_CCM_SUBSCRIBE_STOP_OFFSET     0x0084               /* Subscribe to STOP */
#define NRF54L_CCM_EVENTS_END_OFFSET         0x0104               /* Encrypt/decrypt complete or ended on error */
#define NRF54L_CCM_EVENTS_ERROR_OFFSET       0x0108               /* CCM error */
#define NRF54L_CCM_INTENSET_OFFSET           0x0304               /* Enable interrupt */
#define NRF54L_CCM_INTENCLR_OFFSET           0x0308               /* Disable interrupt */
#define NRF54L_CCM_MACSTATUS_OFFSET          0x0400               /* MAC check result */
#define NRF54L_CCM_ERRORSTATUS_OFFSET        0x0404               /* Error status */
#define NRF54L_CCM_ENABLE_OFFSET             0x0500               /* Enable */
#define NRF54L_CCM_MODE_OFFSET               0x0504               /* Operation mode */
#define NRF54L_CCM_KEY_OFFSET(n)             (0x0510 + ((n) * 4)) /* AES key word n */
#define NRF54L_CCM_NONCE_OFFSET(n)           (0x0520 + ((n) * 4)) /* Nonce word n */
#define NRF54L_CCM_IN_PTR_OFFSET             0x0530               /* Input job list pointer */
#define NRF54L_CCM_OUT_PTR_OFFSET            0x0538               /* Output job list pointer */
#define NRF54L_CCM_RATEOVERRIDE_OFFSET       0x0544               /* Data rate override */
#define NRF54L_CCM_ADATAMASK_OFFSET          0x0548               /* Additional data mask */

/* Register addresses for CCM ***********************************************/

#define NRF54L_CCM_TASKS_START               (NRF54L_CCM00_BASE + NRF54L_CCM_TASKS_START_OFFSET)
#define NRF54L_CCM_TASKS_STOP                (NRF54L_CCM00_BASE + NRF54L_CCM_TASKS_STOP_OFFSET)
#define NRF54L_CCM_TASKS_RATEOVERRIDE        (NRF54L_CCM00_BASE + NRF54L_CCM_TASKS_RATEOVERRIDE_OFFSET)
#define NRF54L_CCM_SUBSCRIBE_START           (NRF54L_CCM00_BASE + NRF54L_CCM_SUBSCRIBE_START_OFFSET)
#define NRF54L_CCM_SUBSCRIBE_STOP            (NRF54L_CCM00_BASE + NRF54L_CCM_SUBSCRIBE_STOP_OFFSET)
#define NRF54L_CCM_EVENTS_END                (NRF54L_CCM00_BASE + NRF54L_CCM_EVENTS_END_OFFSET)
#define NRF54L_CCM_EVENTS_ERROR              (NRF54L_CCM00_BASE + NRF54L_CCM_EVENTS_ERROR_OFFSET)
#define NRF54L_CCM_INTENSET                  (NRF54L_CCM00_BASE + NRF54L_CCM_INTENSET_OFFSET)
#define NRF54L_CCM_INTENCLR                  (NRF54L_CCM00_BASE + NRF54L_CCM_INTENCLR_OFFSET)
#define NRF54L_CCM_MACSTATUS                 (NRF54L_CCM00_BASE + NRF54L_CCM_MACSTATUS_OFFSET)
#define NRF54L_CCM_ERRORSTATUS               (NRF54L_CCM00_BASE + NRF54L_CCM_ERRORSTATUS_OFFSET)
#define NRF54L_CCM_ENABLE                    (NRF54L_CCM00_BASE + NRF54L_CCM_ENABLE_OFFSET)
#define NRF54L_CCM_MODE                      (NRF54L_CCM00_BASE + NRF54L_CCM_MODE_OFFSET)
#define NRF54L_CCM_KEY(n)                    (NRF54L_CCM00_BASE + NRF54L_CCM_KEY_OFFSET(n))
#define NRF54L_CCM_NONCE(n)                  (NRF54L_CCM00_BASE + NRF54L_CCM_NONCE_OFFSET(n))
#define NRF54L_CCM_IN_PTR                    (NRF54L_CCM00_BASE + NRF54L_CCM_IN_PTR_OFFSET)
#define NRF54L_CCM_OUT_PTR                   (NRF54L_CCM00_BASE + NRF54L_CCM_OUT_PTR_OFFSET)
#define NRF54L_CCM_RATEOVERRIDE              (NRF54L_CCM00_BASE + NRF54L_CCM_RATEOVERRIDE_OFFSET)
#define NRF54L_CCM_ADATAMASK                 (NRF54L_CCM00_BASE + NRF54L_CCM_ADATAMASK_OFFSET)

/* Register bit definitions *************************************************/

/* SUBSCRIBE Registers ******************************************************/

#define CCM_SUBSCRIBE_CHIDX_MASK             (0xff)               /* Bits 0-7: DPPI channel */
#define CCM_SUBSCRIBE_EN                     (1 << 31)            /* Bit 31: Enable subscription */

/* INTENSET/INTENCLR Register ***********************************************/

#define CCM_INT_END                          (1 << 1)             /* Bit 1: END event */
#define CCM_INT_ERROR                        (1 << 2)             /* Bit 2: ERROR event */

/* MACSTATUS Register *******************************************************/

#define CCM_MACSTATUS_PASSED                 (1 << 0)             /* Bit 0: MAC check passed */

/* ENABLE Register **********************************************************/

#define CCM_ENABLE_SHIFT                     (0)                  /* Bits 0-1: Enable CCM */
#define CCM_ENABLE_MASK                      (0x3 << CCM_ENABLE_SHIFT)
#  define CCM_ENABLE_DISABLED                (0 << CCM_ENABLE_SHIFT)
#  define CCM_ENABLE_ENABLED                 (2 << CCM_ENABLE_SHIFT)

/* MODE Register ************************************************************/

#define CCM_MODE_MODE_SHIFT                  (0)                  /* Bits 0-1: Operation mode */
#define CCM_MODE_MODE_MASK                   (0x3 << CCM_MODE_MODE_SHIFT)
#  define CCM_MODE_ENCRYPTION                (0 << CCM_MODE_MODE_SHIFT)
#  define CCM_MODE_DECRYPTION                (1 << CCM_MODE_MODE_SHIFT)
#  define CCM_MODE_FASTDECRYPTION            (2 << CCM_MODE_MODE_SHIFT)
#define CCM_MODE_PROTOCOL_SHIFT              (8)                  /* Bits 8-9: Packet format */
#define CCM_MODE_PROTOCOL_MASK               (0x3 << CCM_MODE_PROTOCOL_SHIFT)
#  define CCM_MODE_PROTOCOL_BLE              (0 << CCM_MODE_PROTOCOL_SHIFT)
#  define CCM_MODE_PROTOCOL_IEEE802154       (1 << CCM_MODE_PROTOCOL_SHIFT)
#define CCM_MODE_DATARATE_SHIFT              (16)                 /* Bits 16-18: Radio data rate */
#define CCM_MODE_DATARATE_MASK               (0x7 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_125KBIT          (0 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_250KBIT          (1 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_500KBIT          (2 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_1MBIT            (3 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_2MBIT            (4 << CCM_MODE_DATARATE_SHIFT)
#  define CCM_MODE_DATARATE_4MBIT            (5 << CCM_MODE_DATARATE_SHIFT)
#define CCM_MODE_MACLEN_SHIFT                (24)                 /* Bits 24-26: MAC length */
#define CCM_MODE_MACLEN_MASK                 (0x7 << CCM_MODE_MACLEN_SHIFT)
#  define CCM_MODE_MACLEN_M0                 (0 << CCM_MODE_MACLEN_SHIFT)
#  define CCM_MODE_MACLEN_M4                 (1 << CCM_MODE_MACLEN_SHIFT)

/* ADATAMASK Register *******************************************************/

#define CCM_ADATAMASK_MASK                   (0xff)               /* Bits 0-7: Additional data mask */

/* Job list entry attributes ************************************************/

#define CCM_JOB_ATTR_ALEN                    (11)                 /* Additional data length */
#define CCM_JOB_ATTR_MLEN                    (12)                 /* Message length */
#define CCM_JOB_ATTR_ADATA                   (13)                 /* Additional data */
#define CCM_JOB_ATTR_MDATA                   (14)                 /* Message data */

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_CCM_H */
