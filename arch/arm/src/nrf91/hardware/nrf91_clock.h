/****************************************************************************
 * arch/arm/src/nrf91/hardware/nrf91_clock.h
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

#ifndef __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_CLOCK_H
#define __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf91_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF91_CLOCK_TASKS_HFCLKSTART_OFFSET         0x0000 /* Start HFCLK crystal oscillator */
#define NRF91_CLOCK_TASKS_HFCLKSTOP_OFFSET          0x0004 /* Stop HFCLK crystal oscillator */
#define NRF91_CLOCK_TASKS_LFCLKSTART_OFFSET         0x0008 /* Start LFCLK source */
#define NRF91_CLOCK_TASKS_LFCLKSTOP_OFFSET          0x000c /* Stop LFCLK source */
                                                           /* TODO: */
#define NRF91_CLOCK_EVENTS_HFCLKSTARTED_OFFSET      0x0100 /* HFCLK oscillator started */
#define NRF91_CLOCK_EVENTS_LFCLKSTARTED_OFFSET      0x0104 /* LFCLK started */
                                                           /* TODO: */
#define NRF91_CLOCK_INTENSET_OFFSET                 0x0304 /* Enable interrupt */
#define NRF91_CLOCK_INTENCLR_OFFSET                 0x0308 /* Disable interrupt */
                                                           /* TODO: */
#define NRF91_CLOCK_HFCLKRUN_OFFSET                 0x0408 /* Status indicating that HFCLKSTART task has been triggered */
#define NRF91_CLOCK_HFCLKSTAT_OFFSET                0x040c /* HFCLK status */
#define NRF91_CLOCK_LFCLKRUN_OFFSET                 0x0414 /* Status indicating that LFCLKSTART task has been triggered */
#define NRF91_CLOCK_LFCLKSTAT_OFFSET                0x0418 /* LFCLK status */
#define NRF91_CLOCK_LFCLKSRCCOPY_OFFSET             0x041c /* Copy of LFCLKSRC register, set when LFCLKSTART task was triggered */
#define NRF91_CLOCK_LFCLKSRC_OFFSET                 0x0518 /* Clock source for the LFCLK */

/* Register definitions *****************************************************/

#define NRF91_CLOCK_TASKS_HFCLKSTART         (NRF91_CLOCK_BASE + NRF91_CLOCK_TASKS_HFCLKSTART_OFFSET)
#define NRF91_CLOCK_TASKS_HFCLKSTOP          (NRF91_CLOCK_BASE + NRF91_CLOCK_TASKS_HFCLKSTOP_OFFSET)
#define NRF91_CLOCK_TASKS_LFCLKSTART         (NRF91_CLOCK_BASE + NRF91_CLOCK_TASKS_LFCLKSTART_OFFSET)
#define NRF91_CLOCK_TASKS_LFCLKSTOP          (NRF91_CLOCK_BASE + NRF91_CLOCK_TASKS_LFCLKSTOP_OFFSET)
/* TODO */
#define NRF91_CLOCK_EVENTS_HFCLKSTARTED      (NRF91_CLOCK_BASE + NRF91_CLOCK_EVENTS_HFCLKSTARTED_OFFSET)
#define NRF91_CLOCK_EVENTS_LFCLKSTARTED      (NRF91_CLOCK_BASE + NRF91_CLOCK_EVENTS_LFCLKSTARTED_OFFSET)
/* TODO */
#define NRF91_CLOCK_INTENSET                 (NRF91_CLOCK_BASE + NRF91_CLOCK_INTENSET_OFFSET)
#define NRF91_CLOCK_INTENCLR                 (NRF91_CLOCK_BASE + NRF91_CLOCK_INTENCLR_OFFSET)
/* TODO */
#define NRF91_CLOCK_HFCLKRUN                 (NRF91_CLOCK_BASE + NRF91_CLOCK_HFCLKRUN_OFFSET)
#define NRF91_CLOCK_HFCLKSTAT                (NRF91_CLOCK_BASE + NRF91_CLOCK_HFCLKSTAT_OFFSET)
#define NRF91_CLOCK_LFCLKRUN                 (NRF91_CLOCK_BASE + NRF91_CLOCK_LFCLKRUN_OFFSET)
#define NRF91_CLOCK_LFCLKSTAT                (NRF91_CLOCK_BASE + NRF91_CLOCK_LFCLKSTAT_OFFSET)
#define NRF91_CLOCK_LFCLKSRCCOPY             (NRF91_CLOCK_BASE + NRF91_CLOCK_LFCLKSRCCOPY_OFFSET)
#define NRF91_CLOCK_LFCLKSRC                 (NRF91_CLOCK_BASE + NRF91_CLOCK_LFCLKSRC_OFFSET)

/* Register bit definitions *************************************************/

/* HFCLKRUN Register */

#define CLOCK_HFCLKRUN_STATUS           (1 << 0)  /* Bit 0: HFCLSTART task triggered status */

/* HFCLKSTAT Register */

#define CLOCK_HFCLKSTAT_SRC_SHIFT       (0)                              /* Bit 0: Source of HFCLK */
#define CLOCK_HFCLKSTAT_SRC_MASK        (1 << CLOCK_HFCLKSTAT_SRC_SHIFT)
#  define CLOCK_HFCLKSTAT_SRC_HFINT     (0 << CLOCK_HFCLKSTAT_SRC_SHIFT) /* 0b0: 128 MHz internal oscillator (HFINT) */
#  define CLOCK_HFCLKSTAT_SRC_HFXO      (1 << CLOCK_HFCLKSTAT_SRC_SHIFT) /* 0b1: 128 MHz crystal oscilator (HFXO) */
#define CLOCK_HFCLKSTAT_STATE           (1 << 16)                        /* Bit 16: HFCLK state */

/* LFCLKRUN Register */

#define CLOCK_LFCLKRUN_STATUS           (1 << 0)  /* Bit 0: LFCLKSTART task triggered status */

/* LFCLKSTAT Register */

#define CLOCK_LFCLKSTAT_SRC_SHIFT       (0)                              /* Bits 0-1: Source of LFCLK */
#define CLOCK_LFCLKSTAT_SRC_MASK        (3 << CLOCK_LFCLKSTAT_SRC_SHIFT)
#  define CLOCK_LFCLKSTAT_SRC_LFRC      (1 << CLOCK_LFCLKSTAT_SRC_SHIFT) /* 0b0: 32.768 kHz RC oscillator (LFRC) */
#  define CLOCK_LFCLKSTAT_SRC_LFXO      (2 << CLOCK_LFCLKSTAT_SRC_SHIFT) /* 0b1: 32.768 kHz crystal oscillator (LFXO) */
#define CLOCK_LFCLKSTAT_STATE           (1 << 16)                        /* Bit 16: LFCLKSTAT state */

/* LFCLKSRC Register */

#define CLOCK_LFCLKSRC_SRC_SHIFT       (0)       /* Bits 0-1: LFRC clock source */
#define CLOCK_LFCLKSRC_SRC_MASK        (3 << CLOCK_LFCLKSRC_SRC_SHIFT)
#  define CLOCK_LFCLKSRC_SRC_RFU       (0 << CLOCK_LFCLKSTAT_SRC_SHIFT)
#  define CLOCK_LFCLKSRC_SRC_LFRC      (1 << CLOCK_LFCLKSTAT_SRC_SHIFT)
#  define CLOCK_LFCLKSRC_SRC_LFXO      (2 << CLOCK_LFCLKSTAT_SRC_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_CLOCK_H */
