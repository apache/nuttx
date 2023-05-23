/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_clock.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CLOCK_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF53_CLOCK_TASKS_HFCLKSTART_OFFSET         0x0000 /* Start HFCLK crystal oscillator */
#define NRF53_CLOCK_TASKS_HFCLKSTOP_OFFSET          0x0004 /* Stop HFCLK crystal oscillator */
#define NRF53_CLOCK_TASKS_LFCLKSTART_OFFSET         0x0008 /* Start LFCLK source */
#define NRF53_CLOCK_TASKS_LFCLKSTOP_OFFSET          0x000c /* Stop LFCLK source */
#define NRF53_CLOCK_TASKS_CAL_OFFSET                0x0010 /* Start calibration of LFRC oscillator */
#define NRF53_CLOCK_TASKS_HFCLKAUDIOSTART_OFFSET    0x0018 /* Start HFCLKAUDIO source */
#define NRF53_CLOCK_TASKS_HFCLKAUDIOSTOP_OFFSET     0x001c /* Stop HFCLKAUDIO source */
#define NRF53_CLOCK_TASKS_HFCLK192MSTART_OFFSET     0x0020 /* Start HFCLK192M source as selected in HFCLK192MSRC */
#define NRF53_CLOCK_TASKS_HFCLK192MSTOP_OFFSET      0x0024 /* Stop HFCLK192M source */
                                                           /* TODO: */
#define NRF53_CLOCK_EVENTS_HFCLKSTARTED_OFFSET      0x0100 /* HFCLK oscillator started */
#define NRF53_CLOCK_EVENTS_LFCLKSTARTED_OFFSET      0x0104 /* LFCLK started */
#define NRF53_CLOCK_EVENTS_DONE_OFFSET              0x011c /* Calibration of LFCLK RC oscillator complete event */
#define NRF53_CLOCK_EVENTS_HFCLKAUDIOSTARTED_OFFSET 0x0120 /* HFCLKAUDIO source started */
#define NRF53_CLOCK_EVENTS_HFCLK192MSTARTED_OFFSET  0x0124 /* HFCLK192M source started */
                                                           /* TODO: */
#define NRF53_CLOCK_INTENSET_OFFSET                 0x0304 /* Enable interrupt */
#define NRF53_CLOCK_INTENCLR_OFFSET                 0x0308 /* Disable interrupt */
                                                           /* TODO: */
#define NRF53_CLOCK_HFCLKRUN_OFFSET                 0x0408 /* Status indicating that HFCLKSTART task has been triggered */
#define NRF53_CLOCK_HFCLKSTAT_OFFSET                0x040c /* HFCLK status */
#define NRF53_CLOCK_LFCLKRUN_OFFSET                 0x0414 /* Status indicating that LFCLKSTART task has been triggered */
#define NRF53_CLOCK_LFCLKSTAT_OFFSET                0x0418 /* LFCLK status */
#define NRF53_CLOCK_LFCLKSRCCOPY_OFFSET             0x041c /* Copy of LFCLKSRC register, set when LFCLKSTART task was triggered */
#define NRF53_CLOCK_HFCLKAUDIORUN_OFFSET            0x0450 /* Status indicating that HFCLKAUDIOSTART task has been triggered */
#define NRF53_CLOCK_HFCLKAUDIOSTAT_OFFSET           0x0454 /* Status indicating which HFCLKAUDIO source is running */
#define NRF53_CLOCK_HFCLK192MRUN_OFFSET             0x0458 /* Status indicating that HFCLK192MSTART task has been triggered */
#define NRF53_CLOCK_HFCLK192MSTAT_OFFSET            0x045c /* Status indicating which HFCLK192M source is running */
#define NRF53_CLOCK_HFCLKSRC_OFFSET                 0x0514 /* Clock source for HFCLK128M/HFCLK64M */
#define NRF53_CLOCK_LFCLKSRC_OFFSET                 0x0518 /* Clock source for the LFCLK */
#define NRF53_CLOCK_HFCLKCTRL_OFFSET                0x0558 /* HFCLK128M frequency configuration */
#define NRF53_CLOCK_HFCLKAUDIOFREQUENCY_OFFSET      0x055c /* Audio PLL frequency */
#define NRF53_CLOCK_HFCLKALWAYSRUN_OFFSET           0x0570 /* Automatic or manual control of HFCLK128M/HFCLK64M */
#define NRF53_CLOCK_LFCLKALWAYSRUN_OFFSET           0x0574 /* Automatic or manual control of LFCLK */
#define NRF53_CLOCK_HFCLKAUDIOALWAYSRUN_OFFSET      0x057c /* Automatic or manual control of HFCLKAUDIO */
#define NRF53_CLOCK_HFCLK192MSRC_OFFSET             0x0580 /* Clock source for HFCLK192M */
#define NRF53_CLOCK_HFCLK192MALWAYSRUN_OFFSET       0x0584 /* Automatic or manual control of HFCLK192M */
#define NRF53_CLOCK_HFCLK192MCTRL_OFFSET            0x05b8 /* HFCLK192M frequency configuration */

/* Register definitions *****************************************************/

#define NRF53_CLOCK_TASKS_HFCLKSTART         (NRF53_CLOCK_BASE + NRF53_CLOCK_TASKS_HFCLKSTART_OFFSET)
#define NRF53_CLOCK_TASKS_HFCLKSTOP          (NRF53_CLOCK_BASE + NRF53_CLOCK_TASKS_HFCLKSTOP_OFFSET)
#define NRF53_CLOCK_TASKS_LFCLKSTART         (NRF53_CLOCK_BASE + NRF53_CLOCK_TASKS_LFCLKSTART_OFFSET)
#define NRF53_CLOCK_TASKS_LFCLKSTOP          (NRF53_CLOCK_BASE + NRF53_CLOCK_TASKS_LFCLKSTOP_OFFSET)
#define NRF53_CLOCK_TASKS_CAL                (NRF53_CLOCK_BASE + NRF53_CLOCK_TASKS_CAL_OFFSET)
#define NRF53_CLOCK_TASKS_HFCLKAUDIOSTART    (NRF53_CLOCK_BASE + NRF53_CLOCK_TASKS_HFCLKAUDIOSTART_OFFSET)
#define NRF53_CLOCK_TASKS_HFCLKAUDIOSTOP     (NRF53_CLOCK_BASE + NRF53_CLOCK_TASKS_HFCLKAUDIOSTOP_OFFSET)
#define NRF53_CLOCK_TASKS_HFCLK192MSTART     (NRF53_CLOCK_BASE + NRF53_CLOCK_TASKS_HFCLK192MSTART_OFFSET)
#define NRF53_CLOCK_TASKS_HFCLK192MSTOP      (NRF53_CLOCK_BASE + NRF53_CLOCK_TASKS_HFCLK192MSTOP_OFFSET)
/* TODO */
#define NRF53_CLOCK_EVENTS_HFCLKSTARTED      (NRF53_CLOCK_BASE + NRF53_CLOCK_EVENTS_HFCLKSTARTED_OFFSET)
#define NRF53_CLOCK_EVENTS_LFCLKSTARTED      (NRF53_CLOCK_BASE + NRF53_CLOCK_EVENTS_LFCLKSTARTED_OFFSET)
#define NRF53_CLOCK_EVENTS_DONE              (NRF53_CLOCK_BASE + NRF53_CLOCK_EVENTS_DONE_OFFSET)
#define NRF53_CLOCK_EVENTS_HFCLKAUDIOSTARTED (NRF53_CLOCK_BASE + NRF53_CLOCK_EVENTS_HFCLKAUDIOSTARTED_OFFSET)
#define NRF53_CLOCK_EVENTS_HFCLK192MSTARTED  (NRF53_CLOCK_BASE + NRF53_CLOCK_EVENTS_HFCLK192MSTARTED_OFFSET)
/* TODO */
#define NRF53_CLOCK_INTENSET                 (NRF53_CLOCK_BASE + NRF53_CLOCK_INTENSET_OFFSET)
#define NRF53_CLOCK_INTENCLR                 (NRF53_CLOCK_BASE + NRF53_CLOCK_INTENCLR_OFFSET)
/* TODO */
#define NRF53_CLOCK_HFCLKRUN                 (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLKRUN_OFFSET)
#define NRF53_CLOCK_HFCLKSTAT                (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLKSTAT_OFFSET)
#define NRF53_CLOCK_LFCLKRUN                 (NRF53_CLOCK_BASE + NRF53_CLOCK_LFCLKRUN_OFFSET)
#define NRF53_CLOCK_LFCLKSTAT                (NRF53_CLOCK_BASE + NRF53_CLOCK_LFCLKSTAT_OFFSET)
#define NRF53_CLOCK_LFCLKSRCCOPY             (NRF53_CLOCK_BASE + NRF53_CLOCK_LFCLKSRCCOPY_OFFSET)
#define NRF53_CLOCK_HFCLKAUDIORUN            (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLKAUDIORUN_OFFSET)
#define NRF53_CLOCK_HFCLKAUDIOSTAT           (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLKAUDIOSTAT_OFFSET)
#define NRF53_CLOCK_HFCLK192MRUN             (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLK192MRUN_OFFSET)
#define NRF53_CLOCK_HFCLK192MSTAT            (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLK192MSTAT_OFFSET)
#define NRF53_CLOCK_HFCLKSRC                 (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLKSRC_OFFSET)
#define NRF53_CLOCK_LFCLKSRC                 (NRF53_CLOCK_BASE + NRF53_CLOCK_LFCLKSRC_OFFSET)
#define NRF53_CLOCK_HFCLKCTRL                (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLKCTRL_OFFSET)
#define NRF53_CLOCK_HFCLKAUDIOFREQUENCY      (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLKAUDIOFREQUENCY_OFFSET)
#define NRF53_CLOCK_HFCLKALWAYSRUN           (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLKALWAYSRUN_OFFSET)
#define NRF53_CLOCK_LFCLKALWAYSRUN           (NRF53_CLOCK_BASE + NRF53_CLOCK_LFCLKALWAYSRUN_OFFSET)
#define NRF53_CLOCK_HFCLKAUDIOALWAYSRUN      (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLKAUDIOALWAYSRUN_OFFSET)
#define NRF53_CLOCK_HFCLK192MSRC             (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLK192MSRC_OFFSET)
#define NRF53_CLOCK_HFCLK192MALWAYSRUN       (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLK192MALWAYSRUN_OFFSET)
#define NRF53_CLOCK_HFCLK192MCTRL            (NRF53_CLOCK_BASE + NRF53_CLOCK_HFCLK192MCTRL_OFFSET)

/* Register bit definitions *************************************************/

/* HFCLKRUN Register */

#define CLOCK_HFCLKRUN_STATUS           (1 << 0)  /* Bit 0: HFCLSTART task triggered status */

/* HFCLKSTAT Register */

#define CLOCK_HFCLKSTAT_SRC_SHIFT       (0)                              /* Bit 0: Source of HFCLK */
#define CLOCK_HFCLKSTAT_SRC_MASK        (1 << CLOCK_HFCLKSTAT_SRC_SHIFT)
#  define CLOCK_HFCLKSTAT_SRC_HFINT     (0 << CLOCK_HFCLKSTAT_SRC_SHIFT) /* 0b0: 128 MHz internal oscillator (HFINT) */
#  define CLOCK_HFCLKSTAT_SRC_HFXO      (1 << CLOCK_HFCLKSTAT_SRC_SHIFT) /* 0b1: 128 MHz crystal oscilator (HFXO) */
#define CLOCK_HFCLKSTAT_ALWAYSRUNNING   (1 << 4)                         /* Bit 4: Oscillator is always running */
#define CLOCK_HFCLKSTAT_STATE           (1 << 16)                        /* Bit 16: HFCLK state */

/* LFCLKRUN Register */

#define CLOCK_LFCLKRUN_STATUS           (1 << 0)  /* Bit 0: LFCLKSTART task triggered status */

/* LFCLKSTAT Register */

#define CLOCK_LFCLKSTAT_SRC_SHIFT       (0)                              /* Bits 0-1: Source of LFCLK */
#define CLOCK_LFCLKSTAT_SRC_MASK        (3 << CLOCK_LFCLKSTAT_SRC_SHIFT)
#  define CLOCK_LFCLKSTAT_SRC_LFRC      (1 << CLOCK_LFCLKSTAT_SRC_SHIFT) /* 0b0: 32.768 kHz RC oscillator (LFRC) */
#  define CLOCK_LFCLKSTAT_SRC_LFXO      (2 << CLOCK_LFCLKSTAT_SRC_SHIFT) /* 0b1: 32.768 kHz crystal oscillator (LFXO) */
#  define CLOCK_LFCLKSTAT_SRC_LFSYNT    (3 << CLOCK_LFCLKSTAT_SRC_SHIFT) /* 0b2: 32.768 kHz synthesized from HFCLK (LFSYNT) */
#define CLOCK_LFCLKSTAT_ALWAYSRUNNING   (1 << 4)                         /* Bit 4: Oscillator is always running */
#define CLOCK_LFCLKSTAT_STATE           (1 << 16)                        /* Bit 16: LFCLKSTAT state */

/* LFCLKSRC Register */

#define CLOCK_LFCLKSRC_SRC_SHIFT       (0)       /* Bits 0-1: LFRC clock source */
#define CLOCK_LFCLKSRC_SRC_MASK        (3 << CLOCK_LFCLKSRC_SRC_SHIFT)
#  define CLOCK_LFCLKSRC_SRC_LFRC      (1 << CLOCK_LFCLKSTAT_SRC_SHIFT)
#  define CLOCK_LFCLKSRC_SRC_LFXO      (2 << CLOCK_LFCLKSTAT_SRC_SHIFT)
#  define CLOCK_LFCLKSRC_SRC_LFSYNT    (3 << CLOCK_LFCLKSTAT_SRC_SHIFT)

/* HFCLK192MSRC Register */

#define CLOCK_HFCLK192MSRC_SRC_SHIFT       (0)       /* Bits 0-1: HFCLK192M clock source */
#define CLOCK_HFCLK192MSRC_SRC_MASK        (3 << CLOCK_HFCLK192MSRC_SRC_SHIFT)
#  define CLOCK_HFCLK192MSRC_DIV1          (0 << CLOCK_HFCLK192MSRC_SRC_SHIFT)
#  define CLOCK_HFCLK192MSRC_DIV2          (1 << CLOCK_HFCLK192MSRC_SRC_SHIFT)
#  define CLOCK_HFCLK192MSRC_DIV4          (2 << CLOCK_HFCLK192MSRC_SRC_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CLOCK_H */
