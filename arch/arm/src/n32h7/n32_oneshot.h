/****************************************************************************
 * arch/arm/src/n32h7/n32_oneshot.h
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

#ifndef __ARCH_ARM_SRC_N32H7_N32_ONESHOT_H
#define __ARCH_ARM_SRC_N32H7_N32_ONESHOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include <nuttx/irq.h>

#include "n32_tim.h"

#ifdef CONFIG_N32H7_ONESHOT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_N32H7_ONESHOT_MAXCHANNELS) || \
    CONFIG_N32H7_ONESHOT_MAXCHANNELS < 1
#  undef CONFIG_N32H7_ONESHOT_MAXCHANNELS
#  define CONFIG_N32H7_ONESHOT_MAXCHANNELS 1
#endif

#if CONFIG_N32H7_ONESHOT_MAXCHANNELS > 8
#  warning Additional logic required to handle more than 8 timers
#  undef CONFIG_N32H7_ONESHOT_MAXCHANNELS
#  define CONFIG_N32H7_ONESHOT_MAXCHANNELS 8
#endif

/* Interrupt source for oneshot timer (capture/compare interrupt) */

#define N32_ONESHOT_CCIE(ch) \
    (N32_ATIM_DINTEN_CC1IEN << ((ch) - 1))

#define N32_ONESHOT_CCIF(ch) \
    (N32_ATIM_STS_CC1ITF << ((ch) - 1))

#define N32_ONESHOT_UIE    N32_ATIM_DINTEN_UIEN
#define N32_ONESHOT_UIF    N32_ATIM_STS_UDITF

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the callback function that will be invoked when the oneshot
 * timer expires.  The oneshot fires, the client will receive:
 *
 *   arg - The opaque argument provided when the interrupt was registered
 */

typedef void (*n32_oneshot_handler_t)(void *arg);

/* The oneshot client must allocate an instance of this structure and call
 * n32_oneshot_initialize() before using the oneshot facilities. The client
 * should not access the contents of this structure directly since the
 * contents are subject to change.
 */

struct n32_oneshot_s
{
  uint8_t timer;                          /* Timer number (1 for ATIM1, 3 for GTIMA1, etc.) */
  uint8_t channel;                        /* CC channel (1-4) */
#if CONFIG_N32H7_ONESHOT_MAXCHANNELS > 1
  uint8_t cbndx;                          /* Timer callback handler index */
#endif
  volatile bool running;                  /* True: the timer is running */
  struct n32_tim_dev_s *tch;              /* Pointer returned by n32_tim_init() */
  volatile n32_oneshot_handler_t handler; /* Oneshot expiration callback */
  volatile void *arg;                     /* The argument that will accompany the callback */
  uint32_t frequency;                     /* Timer clock frequency */
  uint32_t period;                        /* Current period in ticks */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: n32_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper (internal use)
 *
 * Input Parameters:
 *   oneshot    Caller allocated instance of the oneshot state structure
 *   timer      Timer number (1-18, see n32_tim_init mapping)
 *   channel    Capture/compare channel (1-4)
 *   resolution The required resolution of the timer in units of
 *              microseconds.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int n32_oneshot_initialize(struct n32_oneshot_s *oneshot, int timer,
                           int channel, uint16_t resolution);

/****************************************************************************
 * Name: n32_oneshot_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 ****************************************************************************/

int n32_oneshot_max_delay(struct n32_oneshot_s *oneshot, uint64_t *usec);

/****************************************************************************
 * Name: n32_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 ****************************************************************************/

int n32_oneshot_start(struct n32_oneshot_s *oneshot,
                      n32_oneshot_handler_t handler, void *arg,
                      const struct timespec *ts);

/****************************************************************************
 * Name: n32_oneshot_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 ****************************************************************************/

int n32_oneshot_cancel(struct n32_oneshot_s *oneshot,
                       struct timespec *ts);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_N32H7_ONESHOT */
#endif /* __ARCH_ARM_SRC_N32H7_N32_ONESHOT_H */
