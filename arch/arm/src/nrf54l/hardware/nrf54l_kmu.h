/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_kmu.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_KMU_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_KMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_KMU_TASKS_PROVISION_OFFSET     0x0000 /* Provision a key slot */
#define NRF54L_KMU_TASKS_PUSH_OFFSET          0x0004 /* Push a key slot */
#define NRF54L_KMU_TASKS_REVOKE_OFFSET        0x0008 /* Revoke a key slot */
#define NRF54L_KMU_TASKS_READMETADATA_OFFSET  0x000c /* Read key metadata */
#define NRF54L_KMU_TASKS_PUSHBLOCK_OFFSET     0x0010 /* Block key push */
#define NRF54L_KMU_TASKS_BLOCK_OFFSET         0x0014 /* Block key operations */
#define NRF54L_KMU_EVENTS_PROVISIONED_OFFSET  0x0100 /* Key provisioned */
#define NRF54L_KMU_EVENTS_PUSHED_OFFSET       0x0104 /* Key pushed */
#define NRF54L_KMU_EVENTS_REVOKED_OFFSET      0x0108 /* Key revoked */
#define NRF54L_KMU_EVENTS_ERROR_OFFSET        0x010c /* Operation failed */
#define NRF54L_KMU_EVENTS_METADATAREAD_OFFSET 0x0110 /* Metadata available */
#define NRF54L_KMU_EVENTS_PUSHBLOCKED_OFFSET  0x0114 /* Key push blocked */
#define NRF54L_KMU_EVENTS_BLOCKED_OFFSET      0x0118 /* Key operations blocked */
#define NRF54L_KMU_STATUS_OFFSET              0x0400 /* Preparation status */
#define NRF54L_KMU_KEYSLOT_OFFSET             0x0500 /* Selected key slot */
#define NRF54L_KMU_SRC_OFFSET                 0x0504 /* Provisioning source */
#define NRF54L_KMU_METADATA_OFFSET            0x0508 /* Key metadata */

/* Register addresses *******************************************************/

#define NRF54L_KMU_TASKS_PROVISION     (NRF54L_KMU_BASE + NRF54L_KMU_TASKS_PROVISION_OFFSET)
#define NRF54L_KMU_TASKS_PUSH          (NRF54L_KMU_BASE + NRF54L_KMU_TASKS_PUSH_OFFSET)
#define NRF54L_KMU_TASKS_REVOKE        (NRF54L_KMU_BASE + NRF54L_KMU_TASKS_REVOKE_OFFSET)
#define NRF54L_KMU_TASKS_READMETADATA  (NRF54L_KMU_BASE + NRF54L_KMU_TASKS_READMETADATA_OFFSET)
#define NRF54L_KMU_TASKS_PUSHBLOCK     (NRF54L_KMU_BASE + NRF54L_KMU_TASKS_PUSHBLOCK_OFFSET)
#define NRF54L_KMU_TASKS_BLOCK         (NRF54L_KMU_BASE + NRF54L_KMU_TASKS_BLOCK_OFFSET)
#define NRF54L_KMU_EVENTS_PROVISIONED  (NRF54L_KMU_BASE + NRF54L_KMU_EVENTS_PROVISIONED_OFFSET)
#define NRF54L_KMU_EVENTS_PUSHED       (NRF54L_KMU_BASE + NRF54L_KMU_EVENTS_PUSHED_OFFSET)
#define NRF54L_KMU_EVENTS_REVOKED      (NRF54L_KMU_BASE + NRF54L_KMU_EVENTS_REVOKED_OFFSET)
#define NRF54L_KMU_EVENTS_ERROR        (NRF54L_KMU_BASE + NRF54L_KMU_EVENTS_ERROR_OFFSET)
#define NRF54L_KMU_EVENTS_METADATAREAD (NRF54L_KMU_BASE + NRF54L_KMU_EVENTS_METADATAREAD_OFFSET)
#define NRF54L_KMU_EVENTS_PUSHBLOCKED  (NRF54L_KMU_BASE + NRF54L_KMU_EVENTS_PUSHBLOCKED_OFFSET)
#define NRF54L_KMU_EVENTS_BLOCKED      (NRF54L_KMU_BASE + NRF54L_KMU_EVENTS_BLOCKED_OFFSET)
#define NRF54L_KMU_STATUS              (NRF54L_KMU_BASE + NRF54L_KMU_STATUS_OFFSET)
#define NRF54L_KMU_KEYSLOT             (NRF54L_KMU_BASE + NRF54L_KMU_KEYSLOT_OFFSET)
#define NRF54L_KMU_SRC                 (NRF54L_KMU_BASE + NRF54L_KMU_SRC_OFFSET)
#define NRF54L_KMU_METADATA            (NRF54L_KMU_BASE + NRF54L_KMU_METADATA_OFFSET)

/* Register bit definitions *************************************************/

#define KMU_STATUS_BUSY      (1 << 0)
#define KMU_KEYSLOT_ID_SHIFT (0)
#define KMU_KEYSLOT_ID_MASK  (0xff << KMU_KEYSLOT_ID_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_KMU_H */
