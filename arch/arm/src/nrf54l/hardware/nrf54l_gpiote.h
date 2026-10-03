/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_gpiote.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_GPIOTE_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_GPIOTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for GPIOTE **********************************************/

#define NRF54L_GPIOTE_TASKS_OUT_OFFSET(x)           (0x0000 + (0x04 * (x))) /* TASKS_OUT[x] */
#define NRF54L_GPIOTE_TASKS_SET_OFFSET(x)           (0x0030 + (0x04 * (x))) /* TASKS_SET[x] */
#define NRF54L_GPIOTE_TASKS_CLR_OFFSET(x)           (0x0060 + (0x04 * (x))) /* TASKS_CLR[x] */
#define NRF54L_GPIOTE_SUBSCRIBE_OUT_OFFSET(x)       (0x0080 + (0x04 * (x))) /* SUBSCRIBE_OUT[x] */
#define NRF54L_GPIOTE_SUBSCRIBE_SET_OFFSET(x)       (0x00b0 + (0x04 * (x))) /* SUBSCRIBE_SET[x] */
#define NRF54L_GPIOTE_SUBSCRIBE_CLR_OFFSET(x)       (0x00e0 + (0x04 * (x))) /* SUBSCRIBE_CLR[x] */
#define NRF54L_GPIOTE_EVENTS_IN_OFFSET(x)           (0x0100 + (0x04 * (x))) /* EVENTS_IN[x] */
#define NRF54L_GPIOTE_EVENTS_PORT_NONSECURE_OFFSET  0x0140                  /* EVENTS_PORT.NONSECURE */
#define NRF54L_GPIOTE_EVENTS_PORT_SECURE_OFFSET     0x0144                  /* EVENTS_PORT.SECURE */
#define NRF54L_GPIOTE_PUBLISH_IN_OFFSET(x)          (0x0180 + (0x04 * (x))) /* PUBLISH_IN[x] */
#define NRF54L_GPIOTE_PUBLISH_PORT_NONSECURE_OFFSET 0x01c0                  /* PUBLISH_PORT.NONSECURE */
#define NRF54L_GPIOTE_PUBLISH_PORT_SECURE_OFFSET    0x01c4                  /* PUBLISH_PORT.SECURE */
#define NRF54L_GPIOTE_INTENSET0_OFFSET              0x0304                  /* INTENSET0 */
#define NRF54L_GPIOTE_INTENCLR0_OFFSET              0x0308                  /* INTENCLR0 */
#define NRF54L_GPIOTE_INTENSET1_OFFSET              0x0314                  /* INTENSET1 */
#define NRF54L_GPIOTE_INTENCLR1_OFFSET              0x0318                  /* INTENCLR1 */
#define NRF54L_GPIOTE_CONFIG_OFFSET(x)              (0x0510 + (0x04 * (x))) /* CONFIG[x] */

/* Register bitfield definitions ********************************************/

/* SUBSCRIBE and PUBLISH Registers */

#define GPIOTE_SUBSCRIBE_CHIDX_SHIFT (0)
#define GPIOTE_SUBSCRIBE_CHIDX_MASK  (0xff << GPIOTE_SUBSCRIBE_CHIDX_SHIFT)
#define GPIOTE_SUBSCRIBE_EN          (1 << 31)
#define GPIOTE_PUBLISH_CHIDX_SHIFT   (0)
#define GPIOTE_PUBLISH_CHIDX_MASK    (0xff << GPIOTE_PUBLISH_CHIDX_SHIFT)
#define GPIOTE_PUBLISH_EN            (1 << 31)

/* EVENT_IN Register */

#define GPIOTE_EVENT_IN_EVENT       (1 << 0) /* Bit 0: Event generated from pin */

/* INTENSET/INTENCLR Register */

#define GPIOTE_INT_IN_SHIFT         0    /* Bits 0-7: Enable interrupt for event IN[i] */

#define GPIOTE_INT_IN_MASK          (0xff << GPIOTE_INT_IN_SHIFT)
#  define GPIOTE_INT_IN(i)          ((1 << (i + GPIOTE_INT_IN_SHIFT)) & GPIOTE_INT_IN_MASK)

#define GPIOTE_INT_PORT_NONSECURE   (1 << 16) /* Non-secure PORT event */
#define GPIOTE_INT_PORT_SECURE      (1 << 17) /* Secure PORT event */

/* CONFIG Register */

#define GPIOTE_CONFIG_MODE_SHIFT    0    /* Bits 0-1: Mode */
#define GPIOTE_CONFIG_MODE_MASK     (0x3 << GPIOTE_CONFIG_MODE_SHIFT)
#  define GPIOTE_CONFIG_MODE_DIS    (0x0 << GPIOTE_CONFIG_MODE_SHIFT) /* 0: Disabled */
#  define GPIOTE_CONFIG_MODE_EV     (0x1 << GPIOTE_CONFIG_MODE_SHIFT) /* 1: Event */
#  define GPIOTE_CONFIG_MODE_TS     (0x3 << GPIOTE_CONFIG_MODE_SHIFT) /* 2: Task */

#define GPIOTE_CONFIG_PSEL_SHIFT    (4)  /* Bits 4-8: GPIO number */
#define GPIOTE_CONFIG_PSEL_MASK     (0x1f << GPIOTE_CONFIG_PSEL_SHIFT)
#define GPIOTE_CONFIG_PORT_SHIFT    (9)  /* Bits 9-12: GPIO port */
#define GPIOTE_CONFIG_PORT_MASK     (0xf << GPIOTE_CONFIG_PORT_SHIFT)
#define GPIOTE_CONFIG_POL_SHIFT     (16) /* Bits 16-17: Polarity */
#define GPIOTE_CONFIG_POL_MASK      (0x3 << GPIOTE_CONFIG_POL_SHIFT)
#  define GPIOTE_CONFIG_POL_NONE    (0x0 << GPIOTE_CONFIG_POL_SHIFT) /* 0: None */
#  define GPIOTE_CONFIG_POL_LTH     (0x1 << GPIOTE_CONFIG_POL_SHIFT) /* 1: LoToHi */
#  define GPIOTE_CONFIG_POL_HTL     (0x2 << GPIOTE_CONFIG_POL_SHIFT) /* 2: HiToLo */
#  define GPIOTE_CONFIG_POL_TG      (0x3 << GPIOTE_CONFIG_POL_SHIFT) /* 3: Toggle */

#define GPIOTE_CONFIG_OUTINIT_SHIFT (20) /* Bit 20: Initial value */

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_GPIOTE_H */
