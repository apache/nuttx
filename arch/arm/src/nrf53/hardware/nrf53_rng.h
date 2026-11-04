/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_rng.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_RNG_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_RNG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets for RNG *************************************************/

#define NRF53_RNG_TASKS_START_OFFSET     0x0000 /* Start RNG */
#define NRF53_RNG_TASKS_STOP_OFFSET      0x0004 /* Stop RNG */
#define NRF53_RNG_SUBSCRIBE_START_OFFSET 0x0080 /* Subscribe START */
#define NRF53_RNG_SUBSCRIBE_STOP_OFFSET  0x0084 /* Subscribe STOP */
#define NRF53_RNG_EVENTS_VALRDY_OFFSET   0x0100 /* Value ready */
#define NRF53_RNG_PUBLISH_VALRDY_OFFSET  0x0180 /* Publish VALRDY */
#define NRF53_RNG_SHORTS_OFFSET          0x0200 /* Shortcuts */
#define NRF53_RNG_INTENSET_OFFSET        0x0304 /* Enable interrupt */
#define NRF53_RNG_INTENCLR_OFFSET        0x0308 /* Disable interrupt */
#define NRF53_RNG_CONFIG_OFFSET          0x0504 /* Configuration */
#define NRF53_RNG_VALUE_OFFSET           0x0508 /* Random number */

/* Register Addresses for RNG ***********************************************/

#define NRF53_RNG_TASKS_START            (NRF53_RNG_BASE + NRF53_RNG_TASKS_START_OFFSET)
#define NRF53_RNG_TASKS_STOP             (NRF53_RNG_BASE + NRF53_RNG_TASKS_STOP_OFFSET)
#define NRF53_RNG_SUBSCRIBE_START        (NRF53_RNG_BASE + NRF53_RNG_SUBSCRIBE_START_OFFSET)
#define NRF53_RNG_SUBSCRIBE_STOP         (NRF53_RNG_BASE + NRF53_RNG_SUBSCRIBE_STOP_OFFSET)
#define NRF53_RNG_EVENTS_VALRDY          (NRF53_RNG_BASE + NRF53_RNG_EVENTS_VALRDY_OFFSET)
#define NRF53_RNG_PUBLISH_VALRDY         (NRF53_RNG_BASE + NRF53_RNG_PUBLISH_VALRDY_OFFSET)
#define NRF53_RNG_SHORTS                 (NRF53_RNG_BASE + NRF53_RNG_SHORTS_OFFSET)
#define NRF53_RNG_INTENSET               (NRF53_RNG_BASE + NRF53_RNG_INTENSET_OFFSET)
#define NRF53_RNG_INTENCLR               (NRF53_RNG_BASE + NRF53_RNG_INTENCLR_OFFSET)
#define NRF53_RNG_CONFIG                 (NRF53_RNG_BASE + NRF53_RNG_CONFIG_OFFSET)
#define NRF53_RNG_VALUE                  (NRF53_RNG_BASE + NRF53_RNG_VALUE_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SHORTS Register */

#define RNG_SHORTS_VALRDY_STOP (1 << 0) /* Bit 0: VALRDY -> STOP */

/* INTENSET/INTENCLR Register */

#define RNG_INT_VALRDY         (1 << 0) /* Bit 0: VALRDY event */

/* CONFIG Register */

#define RNG_CONFIG_DERCEN      (1 << 0) /* Bit 0: Bias correction */

/* VALUE Register */

#define RNG_VALUE_SHIFT        (0)      /* Bits 0-7: Random number */
#define RNG_VALUE_MASK         (0xff << RNG_VALUE_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_RNG_H */
