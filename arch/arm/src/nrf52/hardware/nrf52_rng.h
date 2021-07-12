/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_rng.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_RNG_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_RNG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets for RNG *************************************************/

#define NRF52_RNG_TASKS_START_OFFSET   0x0000  /* RNG Task Start */
#define NRF52_RNG_TASKS_STOP_OFFSET    0x0004  /* RNG Task Stop */
#define NRF52_RNG_EVENTS_RDY_OFFSET    0x0100  /* RNG Eevent ValRDY */
#define NRF52_RNG_SHORTS_OFFSET        0x0200  /* RNG Short Register */
#define NRF52_RNG_INTSET_OFFSET        0x0304  /* RNG INT SET Register */
#define NRF52_RNG_INTCLR_OFFSET        0x0308  /* RNG INT CLR Register */
#define NRF52_RNG_CONFIG_OFFSET        0x0504  /* RNG CONFIG Register */
#define NRF52_RNG_VALUE_OFFSET         0x0508  /* RNG Value Register */

/* Register Addresses for RNG ***********************************************/

#define NRF52_RNG_TASKS_START          (NRF52_RNG_BASE + NRF52_RNG_TASKS_START_OFFSET)
#define NRF52_RNG_TASKS_STOP           (NRF52_RNG_BASE + NRF52_RNG_TASKS_STOP_OFFSET)
#define NRF52_RNG_EVENTS_RDY           (NRF52_RNG_BASE + NRF52_RNG_EVENTS_RDY_OFFSET)

#define NRF52_RNG_SHORTS               (NRF52_RNG_BASE + NRF52_RNG_SHORTS_OFFSET)
#define NRF52_RNG_INTSET               (NRF52_RNG_BASE + NRF52_RNG_INTSET_OFFSET)
#define NRF52_RNG_INTCLR               (NRF52_RNG_BASE + NRF52_RNG_INTCLR_OFFSET)

#define NRF52_RNG_CONFIG               (NRF52_RNG_BASE + NRF52_RNG_CONFIG_OFFSET)
#define NRF52_RNG_VALUE                (NRF52_RNG_BASE + NRF52_RNG_VALUE_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* INTSET/INTCLR Register */

#define RNG_INT_RDY                    (1 << 0) /* Bit 0: VALRDY event */

/* CONFIG Register */

#define RNG_CONFIG_DERCEN              (1 << 0) /* Bit 0: Bias correction */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_STM32_RNG_H */
