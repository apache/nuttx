/**************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_tim.h
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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
 ***************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_TIM_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_TIM_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Register offsets for TIM ************************************************/

#define NRF52_TIM_TASKS_START_OFFSET       0x0000 /* Start Timer */
#define NRF52_TIM_TASKS_STOP_OFFSET        0x0004 /* Stop Timer */
#define NRF52_TIM_TASKS_COUNT_OFFSET       0x0008 /* Increment Timer*/
#define NRF52_TIM_TASKS_CLEAR_OFFSET       0x000c /* Clear time */
#define NRF52_TIM_TASKS_SHUTDOWN_OFFSET    0x0010 /* Shutdown Timer */
#define NRF52_TIM_TASKS_CAPTURE_OFFSET(x)  (0x0040 + ((x) * 0x04)) /* Capture Timer value to CC[x] */
#define NRF52_TIM_EVENTS_COMPARE_OFFSET(x) (0x0140 + ((x) * 0x04)) /* Compare event on CC[x] */
#define NRF52_TIM_SHORTS_OFFSET            0x0200 /* Shortcuts between local events and tasks */
#define NRF52_TIM_INTENSET_OFFSET          0x0304 /* Enable interrupt */
#define NRF52_TIM_MODE_OFFSET              0x0504 /* Timer mode selection */
#define NRF52_TIM_BITMODE_OFFSET           0x0508 /* Configure the number of bits used by the Timer */
#define NRF52_TIM_PRESCALER_OFFSET         0x0510 /* Timer prescaler register */
#define NRF52_TIM_CC_OFFSET(x)             (0x0540 + ((x) * 0x04)) /* Capture/Compare register x */

/* Register offsets for TIM ************************************************/

/* SHORTS Register */

#define TIM_SHORTS_COMPARE_CLEAR(x)        (1 << (x))        /* Bits 0-5: */
#define TIM_SHORTS_COMPARE_STOP(x)         (1 << (x + 0x8))  /* Bits 8-13 */

/* INTENSET/INTENCLR Register */

#define TIM_INT_COMPARE(x)                 (1 << (x + 0x16)) /* Bits 16-21 */

/* MODE Register */

#define TIM_MODE_SHIFT                     (0)               /* Bits 0-1: Timer mode */
#define TIM_MODE_MASK                      (0x3 << TIM_MODE_SHIFT)
#  define TIM_MODE_TIMER                   (0x0 << TIM_MODE_SHIFT) /* 0: Timer mode */
#  define TIM_MODE_COUNTER                 (0x1 << TIM_MODE_SHIFT) /* 1: Counter mode */
#  define TIM_MODE_LPCONUTER               (0x2 << TIM_MODE_SHIFT) /* 2: Low Power Counter mode */

/* BITMODE Register */

#define TIM_BITMODE_SHIFT                  (0)               /* Bits 0-1: Timer bit width */
#define TIM_BITMODE_MASK                   (0x3 << TIM_BITMODE_SHIFT)
#  define TIM_BITMODE_16B                  (0x0 << TIM_BITMODE_SHIFT) /* 0: 16 bit */
#  define TIM_BITMODE_8B                   (0x1 << TIM_BITMODE_SHIFT) /* 1: 8 bit */
#  define TIM_BITMODE_24B                  (0x2 << TIM_BITMODE_SHIFT) /* 2: 24 bit */
#  define TIM_BITMODE_32B                  (0x3 << TIM_BITMODE_SHIFT) /* 3: 32 bit */

/* PRESCALER Register */

#define TIM_PRESCALER_SHIFT                (0)               /* Bits 0-3: Prescaler value */
#define TIM_PRESCALER_MASK                 (0xf << TIM_PRESCALER_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_TIM_H */
