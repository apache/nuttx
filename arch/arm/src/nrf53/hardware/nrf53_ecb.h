/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_ecb.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_ECB_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_ECB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets for ECB *************************************************/

#define NRF53_ECB_TASKS_STARTECB_OFFSET     0x0000 /* Start ECB */
#define NRF53_ECB_TASKS_STOPECB_OFFSET      0x0004 /* Abort ECB */
#define NRF53_ECB_SUBSCRIBE_STARTECB_OFFSET 0x0080 /* Subscribe STARTECB */
#define NRF53_ECB_SUBSCRIBE_STOPECB_OFFSET  0x0084 /* Subscribe STOPECB */
#define NRF53_ECB_EVENTS_ENDECB_OFFSET      0x0100 /* ECB done */
#define NRF53_ECB_EVENTS_ERRORECB_OFFSET    0x0104 /* ECB aborted */
#define NRF53_ECB_PUBLISH_ENDECB_OFFSET     0x0180 /* Publish ENDECB */
#define NRF53_ECB_PUBLISH_ERRORECB_OFFSET   0x0184 /* Publish ERRORECB */
#define NRF53_ECB_INTENSET_OFFSET           0x0304 /* Enable interrupt */
#define NRF53_ECB_INTENCLR_OFFSET           0x0308 /* Disable interrupt */
#define NRF53_ECB_ECBDATAPTR_OFFSET         0x0504 /* Data pointer */

/* Register Addresses for ECB ***********************************************/

#define NRF53_ECB_TASKS_STARTECB            (NRF53_ECB_BASE + NRF53_ECB_TASKS_STARTECB_OFFSET)
#define NRF53_ECB_TASKS_STOPECB             (NRF53_ECB_BASE + NRF53_ECB_TASKS_STOPECB_OFFSET)
#define NRF53_ECB_SUBSCRIBE_STARTECB        (NRF53_ECB_BASE + NRF53_ECB_SUBSCRIBE_STARTECB_OFFSET)
#define NRF53_ECB_SUBSCRIBE_STOPECB         (NRF53_ECB_BASE + NRF53_ECB_SUBSCRIBE_STOPECB_OFFSET)
#define NRF53_ECB_EVENTS_ENDECB             (NRF53_ECB_BASE + NRF53_ECB_EVENTS_ENDECB_OFFSET)
#define NRF53_ECB_EVENTS_ERRORECB           (NRF53_ECB_BASE + NRF53_ECB_EVENTS_ERRORECB_OFFSET)
#define NRF53_ECB_PUBLISH_ENDECB            (NRF53_ECB_BASE + NRF53_ECB_PUBLISH_ENDECB_OFFSET)
#define NRF53_ECB_PUBLISH_ERRORECB          (NRF53_ECB_BASE + NRF53_ECB_PUBLISH_ERRORECB_OFFSET)
#define NRF53_ECB_INTENSET                  (NRF53_ECB_BASE + NRF53_ECB_INTENSET_OFFSET)
#define NRF53_ECB_INTENCLR                  (NRF53_ECB_BASE + NRF53_ECB_INTENCLR_OFFSET)
#define NRF53_ECB_ECBDATAPTR                (NRF53_ECB_BASE + NRF53_ECB_ECBDATAPTR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* INTENSET/INTENCLR Register */

#define ECB_INT_ENDECB   (1 << 0) /* Bit 0: ENDECB event */
#define ECB_INT_ERRORECB (1 << 1) /* Bit 1: ERRORECB event */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_ECB_H */
