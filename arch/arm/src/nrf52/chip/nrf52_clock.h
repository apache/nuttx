/************************************************************************************************
 * arch/arm/src/nrf52/chip/nrf52_clock.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_CHIP_NRF52_CLOCK_H
#define __ARCH_ARM_SRC_NRF52_CHIP_NRF52_CLOCK_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "chip/nrf52_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* Register offsets *****************************************************************************/

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
#define NRF52_CLOCK_INTENSET_OFFSET            0x0304 /* Enable interrupt */
#define NRF52_CLOCK_INTENCLR_OFFSET            0x0308 /* Disable interrupt */
#define NRF52_CLOCK_HFCLKRUN_OFFSET            0x0408 /* Status indicating that HFCLKSTART task has been triggered */
#define NRF52_CLOCK_HFCLKSTAT_OFFSET           0x040c /* HFCLK status */
#define NRF52_CLOCK_LFCLKRUN_OFFSET            0x0414 /* Status indicating that LFCLKSTART task has been triggered */
#define NRF52_CLOCK_LFCLKSTAT_OFFSET           0x0418 /* LFCLK status */
#define NRF52_CLOCK_LFCLKSRCCOPY_OFFSET        0x041c /* Copy of LFCLKSRC register, set when LFCLKSTART task was triggered */
#define NRF52_CLOCK_LFCLKSRC_OFFSET            0x0518 /* Clock source for the LFCLK */
#define NRF52_CLOCK_CTIV_OFFSET                0x0538 /* Calibration timer interval */
#define NRF52_CLOCK_TRACECONFIG_OFFSET         0x055c /* Clocking options for the Trace Port debug interface */

/* Register Addresses ***************************************************************************/

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
#define NRF52_CLOCK_INTENSET            (NRF52_CLOCK_BASE + NRF52_CLOCK_INTENSET_OFFSET)
#define NRF52_CLOCK_INTENCLR            (NRF52_CLOCK_BASE + NRF52_CLOCK_INTENCLR_OFFSET)
#define NRF52_CLOCK_HFCLKRUN            (NRF52_CLOCK_BASE + NRF52_CLOCK_HFCLKRUN_OFFSET)
#define NRF52_CLOCK_HFCLKSTAT           (NRF52_CLOCK_BASE + NRF52_CLOCK_HFCLKSTAT_OFFSET)
#define NRF52_CLOCK_LFCLKRUN            (NRF52_CLOCK_BASE + NRF52_CLOCK_LFCLKRUN_OFFSET)
#define NRF52_CLOCK_LFCLKSTAT           (NRF52_CLOCK_BASE + NRF52_CLOCK_LFCLKSTAT_OFFSET)
#define NRF52_CLOCK_LFCLKSRCCOPY        (NRF52_CLOCK_BASE + NRF52_CLOCK_LFCLKSRCCOPY_OFFSET)
#define NRF52_CLOCK_LFCLKSRC            (NRF52_CLOCK_BASE + NRF52_CLOCK_LFCLKSRC_OFFSET )
#define NRF52_CLOCK_CTIV                (NRF52_CLOCK_BASE + NRF52_CLOCK_CTIV_OFFSET)
#define NRF52_CLOCK_TRACECONFIG         (NRF52_CLOCK_BASE + NRF52_CLOCK_TRACECONFIG_OFFSET)

/* Register Bitfield Definitions ****************************************************************/

#endif /* __ARCH_ARM_SRC_NRF52_CHIP_NRF52_LCD_H */
