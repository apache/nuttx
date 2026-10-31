/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_ecb.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_ECB_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_ECB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets for ECB *************************************************/

#define NRF52_ECB_TASKS_STARTECB_OFFSET   0x0000 /* Start ECB block encrypt */
#define NRF52_ECB_TASKS_STOPECB_OFFSET    0x0004 /* Abort a possible executing ECB operation */
#define NRF52_ECB_EVENTS_ENDECB_OFFSET    0x0100 /* ECB block encrypt complete */
#define NRF52_ECB_EVENTS_ERRORECB_OFFSET  0x0104 /* ECB block encrypt aborted */
#define NRF52_ECB_INTENSET_OFFSET         0x0304 /* Enable interrupt */
#define NRF52_ECB_INTENCLR_OFFSET         0x0308 /* Disable interrupt */
#define NRF52_ECB_ECBDATAPTR_OFFSET       0x0504 /* ECB block encrypt memory pointers */

/* Register Addresses for ECB ***********************************************/

#define NRF52_ECB_TASKS_STARTECB          (NRF52_ECB_BASE + NRF52_ECB_TASKS_STARTECB_OFFSET)
#define NRF52_ECB_TASKS_STOPECB           (NRF52_ECB_BASE + NRF52_ECB_TASKS_STOPECB_OFFSET)
#define NRF52_ECB_EVENTS_ENDECB           (NRF52_ECB_BASE + NRF52_ECB_EVENTS_ENDECB_OFFSET)
#define NRF52_ECB_EVENTS_ERRORECB         (NRF52_ECB_BASE + NRF52_ECB_EVENTS_ERRORECB_OFFSET)
#define NRF52_ECB_INTENSET                (NRF52_ECB_BASE + NRF52_ECB_INTENSET_OFFSET)
#define NRF52_ECB_INTENCLR                (NRF52_ECB_BASE + NRF52_ECB_INTENCLR_OFFSET)
#define NRF52_ECB_ECBDATAPTR              (NRF52_ECB_BASE + NRF52_ECB_ECBDATAPTR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* INTENSET/INTENCLR Register */

#define ECB_INT_ENDECB                    (1 << 0) /* Bit 0: ENDECB event */
#define ECB_INT_ERRORECB                  (1 << 1) /* Bit 1: ERRORECB event */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_ECB_H */
