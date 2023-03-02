/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_rtc.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_RTC_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for RTC *************************************************/

#define NRF53_RTC_TASKS_START_OFFSET       0x0000                  /* Start RTC counter */
#define NRF53_RTC_TASKS_STOP_OFFSET        0x0004                  /* Stop RTC counter */
#define NRF53_RTC_TASKS_CLEAR_OFFSET       0x0008                  /* Clear RTC counter */
#define NRF53_RTC_TASKS_TRIGOVRFLW_OFFSET  0x000c                  /* Clear Set counter to 0xfffff0 */
                                                                   /* TODO: 0x040 - 0x0c */
#define NRF53_RTC_EVENTS_TICK_OFFSET       0x0100                  /* Event on counter increment */
#define NRF53_RTC_EVENTS_OVRFLW_OFFSET     0x0104                  /* Event on counter overflow */
#define NRF53_RTC_EVENTS_COMPARE_OFFSET(x) (0x0140 + ((x) * 0x04)) /* Compare event on CC[x] match */
                                                                   /* TODO: 0x180 - 0x200 */
#define NRF53_RTC_INTENSET_OFFSET          0x0304                  /* Enable interrupt */
#define NRF53_RTC_INTENCLR_OFFSET          0x0308                  /* Disable interrupt */
#define NRF53_RTC_EVTEN_OFFSET             0x0340                  /* Enable or disable event routing */
#define NRF53_RTC_EVTENSET_OFFSET          0x0344                  /* Enable event routing */
#define NRF53_RTC_EVTENCLR_OFFSET          0x0348                  /* Disable event routing */
#define NRF53_RTC_COUNTER_OFFSET           0x0504                  /* Current counter value */
#define NRF53_RTC_PRESCALER_OFFSET         0x0508                  /* 12 bit prescaler for counter frequency */
#define NRF53_RTC_CC_OFFSET(x)             (0x0540 + ((x) * 0x04)) /* Compare register x */

/* Register offsets for RTC *************************************************/

/* TASKS_START Register */

#define RTC_TASKS_START            (1 << 0)                        /* Bit 0: Start RTC counter */

/* TASKS_STOP Register */

#define RTC_TASKS_STOP             (1 << 0)                        /* Bit 0: Stop RTC counter */

/* TASKS_CLEAR Register */

#define RTC_TASKS_CLEAR            (1 << 0)                        /* Bit 0: Clear RTC counter */

/* TASKS_TRIGOVRFLW Register */

#define RTC_TASKS_TRIGOVRFLW       (1 << 0)                        /* Bit 0: Set counter to 0xfffff0 */

/* EVENTS_TICK Register */

#define RTC_EVENTS_TICK            (1 << 0)                        /* Bit 0: Event on counter increment */

/* EVENTS_OVRFLW Register */

#define RTC_EVENTS_OVRFLW          (1 << 0)                        /* Bit 0: Event on counter overflow */

/* EVENTS_COMPARE Register */

#define RTC_EVENTS_COMPARE         (1 << 0)                        /* Bit 0: Eompare event on CC[x] match */

/* INTENSET/INTENCLR Register */

#define RTC_INT_TICK               (1 << 0)                        /* Bit 0: TICK interrupt*/
#define RTC_INT_OVRFLW             (1 << 1)                        /* Bit 1: OVRFLW interrupt */
#define RTC_INT_COMPARE(x)         (1 << (16 + (x)))               /* Bit 16-19: COMPARE[x] interrupt */

/* EVTEN/EVTENSET/EVTSENCLR Register */

#define RTC_EVTEN_TICK             (1 << 0)                        /* Bit 0: TICK event */
#define RTC_EVTEN_OVRFLW           (1 << 1)                        /* Bit 1: OVRFLW event */
#define RTC_EVTEN_COMPARE(x)       (1 << (16 + (x)))               /* Bit 16-19: COMPARE[x] event */

/* COUNTER Register */

#define RTC_COUNTER_MASK           (0x00ffffff)                    /* Bits 0-23: Counter value */

/* PRESCALER Register */

#define RTC_PRESCALER_MASK         (0x00000fff)                    /* Bits 0-11: Prescaler value */
#define RTC_PRESCALER_MAX          (0x00000fff)

/* CC Register */

#define RTC_CC_MASK                (0x00ffffff)                    /* Bits 0-23: Compare register */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_RTC_H */
