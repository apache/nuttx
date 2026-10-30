/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_aar.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_AAR_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_AAR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for AAR *************************************************/

#define NRF54L_AAR_TASKS_START_OFFSET        0x0000               /* Start address resolution */
#define NRF54L_AAR_TASKS_STOP_OFFSET         0x0004               /* Stop address resolution */
#define NRF54L_AAR_SUBSCRIBE_START_OFFSET    0x0080               /* Subscribe to START */
#define NRF54L_AAR_SUBSCRIBE_STOP_OFFSET     0x0084               /* Subscribe to STOP */
#define NRF54L_AAR_EVENTS_END_OFFSET         0x0100               /* Resolution complete or ended on error */
#define NRF54L_AAR_EVENTS_RESOLVED_OFFSET    0x0104               /* Address resolved */
#define NRF54L_AAR_EVENTS_NOTRESOLVED_OFFSET 0x0108               /* Address not resolved */
#define NRF54L_AAR_EVENTS_ERROR_OFFSET       0x010c               /* Operation aborted */
#define NRF54L_AAR_INTENSET_OFFSET           0x0304               /* Enable interrupt */
#define NRF54L_AAR_INTENCLR_OFFSET           0x0308               /* Disable interrupt */
#define NRF54L_AAR_ERRORSTATUS_OFFSET        0x0404               /* Error status */
#define NRF54L_AAR_ENABLE_OFFSET             0x0500               /* Enable */
#define NRF54L_AAR_MAXRESOLVED_OFFSET        0x0508               /* Maximum number of IRKs to resolve */
#define NRF54L_AAR_IN_PTR_OFFSET             0x0530               /* Input job list pointer */
#define NRF54L_AAR_OUT_PTR_OFFSET            0x0538               /* Output job list pointer */
#define NRF54L_AAR_OUT_AMOUNT_OFFSET         0x053c               /* Bytes written in the last job */

/* Register addresses for AAR ***********************************************/

#define NRF54L_AAR_TASKS_START               (NRF54L_AAR00_BASE + NRF54L_AAR_TASKS_START_OFFSET)
#define NRF54L_AAR_TASKS_STOP                (NRF54L_AAR00_BASE + NRF54L_AAR_TASKS_STOP_OFFSET)
#define NRF54L_AAR_SUBSCRIBE_START           (NRF54L_AAR00_BASE + NRF54L_AAR_SUBSCRIBE_START_OFFSET)
#define NRF54L_AAR_SUBSCRIBE_STOP            (NRF54L_AAR00_BASE + NRF54L_AAR_SUBSCRIBE_STOP_OFFSET)
#define NRF54L_AAR_EVENTS_END                (NRF54L_AAR00_BASE + NRF54L_AAR_EVENTS_END_OFFSET)
#define NRF54L_AAR_EVENTS_RESOLVED           (NRF54L_AAR00_BASE + NRF54L_AAR_EVENTS_RESOLVED_OFFSET)
#define NRF54L_AAR_EVENTS_NOTRESOLVED        (NRF54L_AAR00_BASE + NRF54L_AAR_EVENTS_NOTRESOLVED_OFFSET)
#define NRF54L_AAR_EVENTS_ERROR              (NRF54L_AAR00_BASE + NRF54L_AAR_EVENTS_ERROR_OFFSET)
#define NRF54L_AAR_INTENSET                  (NRF54L_AAR00_BASE + NRF54L_AAR_INTENSET_OFFSET)
#define NRF54L_AAR_INTENCLR                  (NRF54L_AAR00_BASE + NRF54L_AAR_INTENCLR_OFFSET)
#define NRF54L_AAR_ERRORSTATUS               (NRF54L_AAR00_BASE + NRF54L_AAR_ERRORSTATUS_OFFSET)
#define NRF54L_AAR_ENABLE                    (NRF54L_AAR00_BASE + NRF54L_AAR_ENABLE_OFFSET)
#define NRF54L_AAR_MAXRESOLVED               (NRF54L_AAR00_BASE + NRF54L_AAR_MAXRESOLVED_OFFSET)
#define NRF54L_AAR_IN_PTR                    (NRF54L_AAR00_BASE + NRF54L_AAR_IN_PTR_OFFSET)
#define NRF54L_AAR_OUT_PTR                   (NRF54L_AAR00_BASE + NRF54L_AAR_OUT_PTR_OFFSET)
#define NRF54L_AAR_OUT_AMOUNT                (NRF54L_AAR00_BASE + NRF54L_AAR_OUT_AMOUNT_OFFSET)

/* Register bit definitions *************************************************/

/* SUBSCRIBE Registers ******************************************************/

#define AAR_SUBSCRIBE_CHIDX_MASK             (0xff)               /* Bits 0-7: DPPI channel */
#define AAR_SUBSCRIBE_EN                     (1 << 31)            /* Bit 31: Enable subscription */

/* ENABLE Register **********************************************************/

#define AAR_ENABLE_SHIFT                     (0)                  /* Bits 0-1: Enable AAR */
#define AAR_ENABLE_MASK                      (0x3 << AAR_ENABLE_SHIFT)
#  define AAR_ENABLE_DISABLED                (0 << AAR_ENABLE_SHIFT)
#  define AAR_ENABLE_ENABLED                 (3 << AAR_ENABLE_SHIFT)

/* Job list entry attributes ************************************************/

#define AAR_JOB_ATTR_HASH                    (11)                 /* Address hash */
#define AAR_JOB_ATTR_PRAND                   (12)                 /* Address prand */
#define AAR_JOB_ATTR_IRK                     (13)                 /* IRK */
#define AAR_JOB_ATTR_INDEX                   (11)                 /* Output IRK index */

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_AAR_H */
