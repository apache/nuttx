/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_clock.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_CLOCK_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(CONFIG_ARCH_CHIP_NRF52832)
#  undef HAVE_CLOCK_CALTIMER_EVENTS
#  undef HAVE_CLOCK_HFXODEBOUNCE
#elif defined(CONFIG_ARCH_CHIP_NRF52833)
#  define HAVE_CLOCK_CALTIMER_EVENTS
#  define HAVE_CLOCK_HFXODEBOUNCE
#elif defined(CONFIG_ARCH_CHIP_NRF52840)
#  define HAVE_CLOCK_CALTIMER_EVENTS
#  define HAVE_CLOCK_HFXODEBOUNCE
#else
#  error Unknown NRF52 chip !
#endif

/* Register offsets *********************************************************/

#define NRF52_CLOCK_TASKS_HFCLKSTART_OFFSET    0x0000 /* Start HFCLK crystal oscillator */
#define NRF52_CLOCK_TASKS_HFCLKSTOP_OFFSET     0x0004 /* Stop HFCLK crystal oscillator */
#define NRF52_CLOCK_TASKS_LFCLKSTART_OFFSET    0x0008 /* Start LFCLK source */
#define NRF52_CLOCK_TASKS_LFCLKSTOP_OFFSET     0x000c /* Stop LFCLK source */
#define NRF52_CLOCK_TASKS_CAL_OFFSET           0x0010 /* Start calibration of LFRC oscillator */
#define NRF52_CLOCK_TASKS_CTSTART_OFFSET       0x0014 /* Start calibration timer */
#define NRF52_CLOCK_TASKS_CTSTOP_OFFSET        0x0018 /* Stop calibration timer */
#define NRF52_CLOCK_EVENTS_HFCLKSTARTED_OFFSET 0x0100 /* HFCLK oscillator started */
#define NRF52_CLOCK_EVENTS_LFCLKSTARTED_OFFSET 0x0104 /* LFCLK started */
#define NRF52_CLOCK_EVENTS_DONE_OFFSET         0x010c /* Calibration of LFCLK RC oscillator complete event */
#define NRF52_CLOCK_EVENTS_CTTO_OFFSET         0x0110 /* Calibration timer timeout */
#ifdef HAVE_CLOCK_CALTIMER_EVENTS
#  define NRF52_CLOCK_EVENTS_CSTARTED_OFFSET   0x0128 /* Calibration timer has been started */
#  define NRF52_CLOCK_EVENTS_CTSTOPPED_OFFSET  0x012c /* Calibration timer has been stopped */
#endif
#define NRF52_CLOCK_INTENSET_OFFSET            0x0304 /* Enable interrupt */
#define NRF52_CLOCK_INTENCLR_OFFSET            0x0308 /* Disable interrupt */
#define NRF52_CLOCK_HFCLKRUN_OFFSET            0x0408 /* Status indicating that HFCLKSTART task has been triggered */
#define NRF52_CLOCK_HFCLKSTAT_OFFSET           0x040c /* HFCLK status */
#define NRF52_CLOCK_LFCLKRUN_OFFSET            0x0414 /* Status indicating that LFCLKSTART task has been triggered */
#define NRF52_CLOCK_LFCLKSTAT_OFFSET           0x0418 /* LFCLK status */
#define NRF52_CLOCK_LFCLKSRCCOPY_OFFSET        0x041c /* Copy of LFCLKSRC register, set when LFCLKSTART task was triggered */
#define NRF52_CLOCK_LFCLKSRC_OFFSET            0x0518 /* Clock source for the LFCLK */
#ifdef HAVE_CLOCK_HFXODEBOUNCE
#  define NRF52_CLOCK_HFXODEBOUNCE_OFFSET      0x0528 /* HFXO debounce time */
#endif
#define NRF52_CLOCK_CTIV_OFFSET                0x0538 /* Calibration timer interval */
#define NRF52_CLOCK_TRACECONFIG_OFFSET         0x055c /* Clocking options for the Trace Port debug interface */

/* Register Addresses *******************************************************/

#define NRF52_CLOCK_TASKS_HFCLKSTART    (NRF52_CLOCK_BASE + NRF52_CLOCK_TASKS_HFCLKSTART_OFFSET)
#define NRF52_CLOCK_TASKS_HFCLKSTOP     (NRF52_CLOCK_BASE + NRF52_CLOCK_TASKS_HFCLKSTOP_OFFSET)
#define NRF52_CLOCK_TASKS_LFCLKSTART    (NRF52_CLOCK_BASE + NRF52_CLOCK_TASKS_LFCLKSTART_OFFSET)
#define NRF52_CLOCK_TASKS_LFCLKSTOP     (NRF52_CLOCK_BASE + NRF52_CLOCK_TASKS_LFCLKSTOP_OFFSET)
#define NRF52_CLOCK_TASKS_CAL           (NRF52_CLOCK_BASE + NRF52_CLOCK_TASKS_CAL_OFFSET)
#define NRF52_CLOCK_TASKS_CTSTART       (NRF52_CLOCK_BASE + NRF52_CLOCK_TASKS_CTSTART_OFFSET)
#define NRF52_CLOCK_TASKS_CTSTOP        (NRF52_CLOCK_BASE + NRF52_CLOCK_TASKS_CTSTOP_OFFSET)
#define NRF52_CLOCK_EVENTS_HFCLKSTARTED (NRF52_CLOCK_BASE + NRF52_CLOCK_EVENTS_HFCLKSTARTED_OFFSET)
#define NRF52_CLOCK_EVENTS_LFCLKSTARTED (NRF52_CLOCK_BASE + NRF52_CLOCK_EVENTS_LFCLKSTARTED_OFFSET)
#define NRF52_CLOCK_EVENTS_DONE         (NRF52_CLOCK_BASE + NRF52_CLOCK_EVENTS_DONE_OFFSET)
#define NRF52_CLOCK_EVENTS_CTTO         (NRF52_CLOCK_BASE + NRF52_CLOCK_EVENTS_CTTO_OFFSET)
#ifdef HAVE_CLOCK_CALTIMER_EVENTS
#  define NRF52_CLOCK_EVENTS_CSTARTED   (NRF52_CLOCK_BASE + NRF52_CLOCK_EVENTS_CSTARTED_OFFSET)
#  define NRF52_CLOCK_EVENTS_CSTOPPED   (NRF52_CLOCK_BASE + NRF52_CLOCK_EVENTS_CSTOPPED_OFFSET)
#endif
#define NRF52_CLOCK_INTENSET            (NRF52_CLOCK_BASE + NRF52_CLOCK_INTENSET_OFFSET)
#define NRF52_CLOCK_INTENCLR            (NRF52_CLOCK_BASE + NRF52_CLOCK_INTENCLR_OFFSET)
#define NRF52_CLOCK_HFCLKRUN            (NRF52_CLOCK_BASE + NRF52_CLOCK_HFCLKRUN_OFFSET)
#define NRF52_CLOCK_HFCLKSTAT           (NRF52_CLOCK_BASE + NRF52_CLOCK_HFCLKSTAT_OFFSET)
#define NRF52_CLOCK_LFCLKRUN            (NRF52_CLOCK_BASE + NRF52_CLOCK_LFCLKRUN_OFFSET)
#define NRF52_CLOCK_LFCLKSTAT           (NRF52_CLOCK_BASE + NRF52_CLOCK_LFCLKSTAT_OFFSET)
#define NRF52_CLOCK_LFCLKSRCCOPY        (NRF52_CLOCK_BASE + NRF52_CLOCK_LFCLKSRCCOPY_OFFSET)
#define NRF52_CLOCK_LFCLKSRC            (NRF52_CLOCK_BASE + NRF52_CLOCK_LFCLKSRC_OFFSET)
#ifdef HAVE_CLOCK_HFXODEBOUNCE
#  define NRF52_CLOCK_HFXODEBOUNCE      (NRF52_CLOCK_BASE + NRF52_CLOCK_HFXODEBOUNCE_OFFSET)
#endif
#define NRF52_CLOCK_CTIV                (NRF52_CLOCK_BASE + NRF52_CLOCK_CTIV_OFFSET)
#define NRF52_CLOCK_TRACECONFIG         (NRF52_CLOCK_BASE + NRF52_CLOCK_TRACECONFIG_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* HFCLKRUN Register */

#define CLOCK_HFCLKRUN_STATUS           (1 << 0)  /* Bit 0: HFCLSTART task triggered status */

/* HFCLKSTAT Register */

#define CLOCK_HFCLKSTAT_SRC_SHIFT       (0)                              /* Bit 0: Source of HFCLK */
#define CLOCK_HFCLKSTAT_SRC_MASK        (1 << CLOCK_HFCLKSTAT_SRC_SHIFT)
#  define CLOCK_HFCLKSTAT_SRC_RC        (0 << CLOCK_HFCLKSTAT_SRC_SHIFT) /* 0b0: 64 MHz internal oscillator (HFINT) */
#  define CLOCK_HFCLKSTAT_SRC_XTAL      (1 << CLOCK_HFCLKSTAT_SRC_SHIFT) /* 0b1: 64 MHz crystal oscilator (HFXO) */
#define CLOCK_HFCLKSTAT_STATE           (1 << 16)                        /* Bit 16: HFCLK state */

/* LFCLKRUN Register */

#define CLOCK_LFCLKRUN_STATUS           (1 << 0)  /* Bit 0: LFCLKSTART task triggered status */

/* LFCLKSTAT Register */

#define CLOCK_LFCLKSTAT_SRC_SHIFT       (0)                              /* Bits 0-1: Source of LFCLK */
#define CLOCK_LFCLKSTAT_SRC_MASK        (3 << CLOCK_LFCLKSTAT_SRC_SHIFT)
#  define CLOCK_LFCLKSTAT_SRC_RC        (0 << CLOCK_LFCLKSTAT_SRC_SHIFT) /* 0b0: RC oscillator (LFRC) */
#  define CLOCK_LFCLKSTAT_SRC_XTAL      (1 << CLOCK_LFCLKSTAT_SRC_SHIFT) /* 0b1: crystal oscillator (LFXO) */
#  define CLOCK_LFCLKSTAT_SRC_SYNTH     (2 << CLOCK_LFCLKSTAT_SRC_SHIFT) /* 0b2: synthesized from HFCLK (LFSYNT) */
#define CLOCK_LFCLKSTAT_STATE           (1 << 16)                        /* Bit 16: LFCLKSTAT state */

/* LFCLKSRC Register */

#define CLOCK_LFCLKSRC_SRC_SHIFT        (0)       /* Bits 0-1: LFRC clock source */
#define CLOCK_LFCLKSRC_SRC_MASK         (3 << CLOCK_LFCLKSRC_SRC_SHIFT)
#  define CLOCK_LFCLKSRC_SRC_RC         (0 << CLOCK_LFCLKSRC_SRC_SHIFT)
#  define CLOCK_LFCLKSRC_SRC_XTAL       (1 << CLOCK_LFCLKSRC_SRC_SHIFT)
#  define CLOCK_LFCLKSRC_SRC_SYNTH      (2 << CLOCK_LFCLKSRC_SRC_SHIFT)
#define CLOCK_LFCLKSRC_BYPASS           (1 << 16) /* Bit 16: Enable/disable bypass of LFCLK crystal oscillator */
#define CLOCK_LFCLKSRC_EXTERNAL         (1 << 17) /* Bit 17: Enable/disable external source for LFCLK */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_CLOCK_H */
