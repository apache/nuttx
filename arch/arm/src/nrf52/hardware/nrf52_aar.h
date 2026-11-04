/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_aar.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_AAR_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_AAR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets for AAR *************************************************/

#define NRF52_AAR_TASKS_START_OFFSET        0x0000 /* Start resolving addresses */
#define NRF52_AAR_TASKS_STOP_OFFSET         0x0008 /* Stop resolving addresses */
#define NRF52_AAR_EVENTS_END_OFFSET         0x0100 /* Address resolution procedure complete */
#define NRF52_AAR_EVENTS_RESOLVED_OFFSET    0x0104 /* Address resolved */
#define NRF52_AAR_EVENTS_NOTRESOLVED_OFFSET 0x0108 /* Address not resolved */
#define NRF52_AAR_INTENSET_OFFSET           0x0304 /* Enable interrupt */
#define NRF52_AAR_INTENCLR_OFFSET           0x0308 /* Disable interrupt */
#define NRF52_AAR_STATUS_OFFSET             0x0400 /* Resolution status */
#define NRF52_AAR_ENABLE_OFFSET             0x0500 /* Enable AAR */
#define NRF52_AAR_NIRK_OFFSET               0x0504 /* Number of IRKs */
#define NRF52_AAR_IRKPTR_OFFSET             0x0508 /* Pointer to IRK data structure */
#define NRF52_AAR_ADDRPTR_OFFSET            0x0510 /* Pointer to the resolvable address */
#define NRF52_AAR_SCRATCHPTR_OFFSET         0x0514 /* Pointer to scratch area */

/* Register Addresses for AAR ***********************************************/

#define NRF52_AAR_TASKS_START               (NRF52_AAR_BASE + NRF52_AAR_TASKS_START_OFFSET)
#define NRF52_AAR_TASKS_STOP                (NRF52_AAR_BASE + NRF52_AAR_TASKS_STOP_OFFSET)
#define NRF52_AAR_EVENTS_END                (NRF52_AAR_BASE + NRF52_AAR_EVENTS_END_OFFSET)
#define NRF52_AAR_EVENTS_RESOLVED           (NRF52_AAR_BASE + NRF52_AAR_EVENTS_RESOLVED_OFFSET)
#define NRF52_AAR_EVENTS_NOTRESOLVED        (NRF52_AAR_BASE + NRF52_AAR_EVENTS_NOTRESOLVED_OFFSET)
#define NRF52_AAR_INTENSET                  (NRF52_AAR_BASE + NRF52_AAR_INTENSET_OFFSET)
#define NRF52_AAR_INTENCLR                  (NRF52_AAR_BASE + NRF52_AAR_INTENCLR_OFFSET)
#define NRF52_AAR_STATUS                    (NRF52_AAR_BASE + NRF52_AAR_STATUS_OFFSET)
#define NRF52_AAR_ENABLE                    (NRF52_AAR_BASE + NRF52_AAR_ENABLE_OFFSET)
#define NRF52_AAR_NIRK                      (NRF52_AAR_BASE + NRF52_AAR_NIRK_OFFSET)
#define NRF52_AAR_IRKPTR                    (NRF52_AAR_BASE + NRF52_AAR_IRKPTR_OFFSET)
#define NRF52_AAR_ADDRPTR                   (NRF52_AAR_BASE + NRF52_AAR_ADDRPTR_OFFSET)
#define NRF52_AAR_SCRATCHPTR                (NRF52_AAR_BASE + NRF52_AAR_SCRATCHPTR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* INTENSET/INTENCLR Register */

#define AAR_INT_END                         (1 << 0) /* Bit 0: END event */
#define AAR_INT_RESOLVED                    (1 << 1) /* Bit 1: RESOLVED event */
#define AAR_INT_NOTRESOLVED                 (1 << 2) /* Bit 2: NOTRESOLVED event */

/* STATUS Register */

#define AAR_STATUS_SHIFT                    (0)      /* Bits 0-3: Index of resolving IRK */
#define AAR_STATUS_MASK                     (0xf << AAR_STATUS_SHIFT)

/* ENABLE Register */

#define AAR_ENABLE_SHIFT                    (0)      /* Bits 0-1: Enable AAR */
#define AAR_ENABLE_MASK                     (0x3 << AAR_ENABLE_SHIFT)
#  define AAR_ENABLE_DISABLED               (0 << AAR_ENABLE_SHIFT)
#  define AAR_ENABLE_ENABLED                (3 << AAR_ENABLE_SHIFT)

/* NIRK Register */

#define AAR_NIRK_SHIFT                      (0)      /* Bits 0-4: Number of IRKs */
#define AAR_NIRK_MASK                       (0x1f << AAR_NIRK_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_AAR_H */
