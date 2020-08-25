/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_gpiote.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_GPIOTE_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_GPIOTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for GPIOTE **********************************************/

#define NRF52_GPIOTE_TASKS_OUT_OFFSET(x)    (0x0000 + (0x04 * x)) /* TASKS_OUT[x] */
#define NRF52_GPIOTE_TASKS_SET_OFFSET(x)    (0x0030 + (0x04 * x)) /* TASKS_SET[x] */
#define NRF52_GPIOTE_TASKS_CLR_OFFSET(x)    (0x0060 + (0x04 * x)) /* TASKS_CLR[x] */
#define NRF52_GPIOTE_EVENTS_IN_OFFSET(x)    (0x0100 + (0x04 * x)) /* EVENTS_IN[x] */
#define NRF52_GPIOTE_EVENTS_PORT_OFFSET     0x017c                /* EVENTS_PORT */
#define NRF52_GPIOTE_INTENSET_OFFSET        0x0304                /* INTENSET */
#define NRF52_GPIOTE_INTENCLR_OFFSET        0x0308                /* INTENCLR */
#define NRF52_GPIOTE_CONFIG_OFFSET(x)       (0x0510 + (0x04 * x)) /* CONFIG[x] */

/* Register addresses for GPIOTE ********************************************/

#define NRF52_GPIOTE_TASKS_OUT(x)           (NRF52_GPIOTE_BASE + NRF52_GPIOTE_TASKS_OUT_OFFSET(x))
#define NRF52_GPIOTE_TASKS_SET(x)           (NRF52_GPIOTE_BASE + NRF52_GPIOTE_TASKS_SET_OFFSET(x))
#define NRF52_GPIOTE_TASKS_CLR(x)           (NRF52_GPIOTE_BASE + NRF52_GPIOTE_TASKS_CLR_OFFSET(x))
#define NRF52_GPIOTE_EVENTS_IN(x)           (NRF52_GPIOTE_BASE + NRF52_GPIOTE_EVENTS_IN_OFFSET(x))
#define NRF52_GPIOTE_EVENTS_PORT            (NRF52_GPIOTE_BASE + NRF52_GPIOTE_EVENTS_PORT_OFFSET)
#define NRF52_GPIOTE_INTENSET               (NRF52_GPIOTE_BASE + NRF52_GPIOTE_INTENSET_OFFSET)
#define NRF52_GPIOTE_INTENCLR               (NRF52_GPIOTE_BASE + NRF52_GPIOTE_INTENCLR_OFFSET)
#define NRF52_GPIOTE_CONFIG(x)              (NRF52_GPIOTE_BASE + NRF52_GPIOTE_CONFIG_OFFSET(x))

/* Register offsets for GPIOTE **********************************************/

/* EVENT_IN Register */

#define GPIOTE_EVENT_IN_EVENT       (1 << 0) /* Bit 0: Event generated from pin */

/* INTENSET/INTENCLR Register */

#define GPIOTE_INT_IN_SHIFT         0    /* Bits 0-7: Enable interrupt for event IN[i] */

#define GPIOTE_INT_IN_MASK          (0xff << GPIOTE_INT_IN_SHIFT)
#  define GPIOTE_INT_IN(i)          ((1 << (i + GPIOTE_INT_IN_SHIFT)) & GPIOTE_INT_IN_MASK)

#define GPIOTE_INT_PORT             31   /* Bit 31: Enable interrupt for event PORT */

/* CONFIG Register */

#define GPIOTE_CONFIG_MODE_SHIFT    0    /* Bits 0-1: Mode */
#define GPIOTE_CONFIG_MODE_MASK     (0x3 << GPIOTE_CONFIG_MODE_SHIFT)
#  define GPIOTE_CONFIG_MODE_DIS    (0x0 << GPIOTE_CONFIG_MODE_SHIFT) /* 0: Disabled */
#  define GPIOTE_CONFIG_MODE_EV     (0x1 << GPIOTE_CONFIG_MODE_SHIFT) /* 1: Event */
#  define GPIOTE_CONFIG_MODE_TS     (0x3 << GPIOTE_CONFIG_MODE_SHIFT) /* 2: Task */

#define GPIOTE_CONFIG_PSEL_SHIFT    (8)  /* Bits 8-12: GPIO number */
#define GPIOTE_CONFIG_PSEL_MASK     (0x1f << GPIOTE_CONFIG_PSEL_SHIFT)
#define GPIOTE_CONFIG_PORT_SHIFT    (13) /* Bit 13: GPIO port */
#define GPIOTE_CONFIG_POL_SHIFT     (16) /* Bits 16-17: Polarity */
#define GPIOTE_CONFIG_POL_MASK      (0x3 << GPIOTE_CONFIG_POL_SHIFT)
#  define GPIOTE_CONFIG_POL_NONE    (0x0 << GPIOTE_CONFIG_POL_SHIFT) /* 0: None */
#  define GPIOTE_CONFIG_POL_LTH     (0x1 << GPIOTE_CONFIG_POL_SHIFT) /* 1: LoToHi */
#  define GPIOTE_CONFIG_POL_HTL     (0x2 << GPIOTE_CONFIG_POL_SHIFT) /* 2: HiToLo */
#  define GPIOTE_CONFIG_POL_TG      (0x3 << GPIOTE_CONFIG_POL_SHIFT) /* 3: Toggle */

#define GPIOTE_CONFIG_OUTINIT_SHIFT (20) /* Bit 20: Initial value */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_GPIOTE_H */
