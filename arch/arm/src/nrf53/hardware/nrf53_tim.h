/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_tim.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_TIM_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TIMER constants **********************************************************/

#define TIMER_BASE_FERQUENCY              (16000000)

/* Register offsets for TIM *************************************************/

#define NRF53_TIM_TASKS_START_OFFSET       0x0000                  /* Start Timer */
#define NRF53_TIM_TASKS_STOP_OFFSET        0x0004                  /* Stop Timer */
#define NRF53_TIM_TASKS_COUNT_OFFSET       0x0008                  /* Increment Timer */
#define NRF53_TIM_TASKS_CLEAR_OFFSET       0x000c                  /* Clear time */
#define NRF53_TIM_TASKS_SHUTDOWN_OFFSET    0x0010                  /* Shutdown Timer */
#define NRF53_TIM_TASKS_CAPTURE_OFFSET(x)  (0x0040 + ((x) * 4))    /* Capture Timer value to CC[x] */
#define NRF53_TIM_EVENTS_COMPARE_OFFSET(x) (0x0140 + ((x) * 4))    /* Compare event on CC[x] */
                                                                   /* TODO: 0x080-0x1c0 */
#define NRF53_TIM_SHORTS_OFFSET            0x0200                  /* Shortcuts between local events and tasks */
#define NRF53_TIM_INTEN_OFFSET             0x0300                  /* Enable or disable interrupt */
#define NRF53_TIM_INTENSET_OFFSET          0x0304                  /* Enable interrupt */
#define NRF53_TIM_INTCLR_OFFSET            0x0308                  /* Disable interrupt */
#define NRF53_TIM_MODE_OFFSET              0x0504                  /* Timer mode selection */
#define NRF53_TIM_BITMODE_OFFSET           0x0508                  /* Configure the number of bits used by the Timer */
#define NRF53_TIM_PRESCALER_OFFSET         0x0510                  /* Timer prescaler register */
#define NRF53_TIM_CC_OFFSET(x)             (0x0540 + ((x) * 4))    /* Capture/Compare register x */
#define NRF53_TIM_ONESHOT_OFFSET(x)        (0x0580 + ((x) * 4))    /* Enable one-shot operation for Capture/Compare channel x */

/* Register offsets for TIM *************************************************/

/* TASKS_START Register */

#define TIM_TASKS_START                    (1 << 0)                /* Bit 0: Start Timer */

/* TASKS_STOP Register */

#define TIM_TASKS_STOP                     (1 << 0)                /* Bit 0: Stop Timer */

/* TASKS_COUNT Register */

#define TIM_TASKS_COUNT                    (1 << 0)                /* Bit 0: Increment Timer */

/* TASKS_CLEAR Register */

#define TIM_TASKS_CLEAR                    (1 << 0)                /* Bit 0: Clear Timer */

/* SHORTS Register */

#define TIM_SHORTS_COMPARE_CLEAR(x)        (1 << (x))              /* Bits 0-5: */
#define TIM_SHORTS_COMPARE_STOP(x)         (1 << (x + 8))          /* Bits 8-13 */

/* INTENSET/INTENCLR Register */

#define TIM_INT_COMPARE(x)                 (1 << (x + 16))         /* Bits 16-21 */

/* MODE Register */

#define TIM_MODE_SHIFT                     (0)                     /* Bits 0-1: Timer mode */
#define TIM_MODE_MASK                      (0x3 << TIM_MODE_SHIFT)
#  define TIM_MODE_TIMER                   (0x0 << TIM_MODE_SHIFT) /* 0: Timer mode */
#  define TIM_MODE_COUNTER                 (0x1 << TIM_MODE_SHIFT) /* 1: Counter mode */
#  define TIM_MODE_LPCOUNTER               (0x2 << TIM_MODE_SHIFT) /* 2: Low Power Counter mode */

/* BITMODE Register */

#define TIM_BITMODE_SHIFT                  (0)                        /* Bits 0-1: Timer bit width */
#define TIM_BITMODE_MASK                   (0x3 << TIM_BITMODE_SHIFT)
#  define TIM_BITMODE_16B                  (0x0 << TIM_BITMODE_SHIFT) /* 0: 16 bit */
#  define TIM_BITMODE_8B                   (0x1 << TIM_BITMODE_SHIFT) /* 1: 8 bit */
#  define TIM_BITMODE_24B                  (0x2 << TIM_BITMODE_SHIFT) /* 2: 24 bit */
#  define TIM_BITMODE_32B                  (0x3 << TIM_BITMODE_SHIFT) /* 3: 32 bit */

/* PRESCALER Register */

#define TIM_PRESCALER_SHIFT                (0)                        /* Bits 0-3: Prescaler value */
#define TIM_PRESCALER_MAX                  (9)
#define TIM_PRESCALER_MASK                 (TIM_PRESCALER_MAX << TIM_PRESCALER_SHIFT)

/* ONESHOT Register */

#define TIM_ONESHOT_EN                     (1 << 0)                   /* Bit 0: Enable one-shot operation */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_TIM_H */
