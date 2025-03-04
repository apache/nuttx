/****************************************************************************
 * arch/x86_64/src/intel64/intel64_oneshot.h
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

#ifndef __ARCH_X86_64_SRC_INTEL64_INTEL64_ONESHOT_H
#define __ARCH_X86_64_SRC_INTEL64_INTEL64_ONESHOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>
#include <nuttx/clock.h>

#include <intel64_hpet.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the callback function that will be invoked when the oneshot
 * timer expires.  The oneshot fires, the client will receive:
 *
 *   arg - The opaque argument provided when the interrupt was registered
 */

typedef void (*oneshot_handler_t)(void *arg);

/* The oneshot client must allocate an instance of this structure and called
 * intel64_oneshot_initialize() before using the oneshot facilities.  The
 * client should not access the contents of this structure directly since
 * the contents are subject to change.
 */

struct intel64_oneshot_s
{
  uint8_t                     chan;
  uint8_t                     cbndx;
  volatile bool               running;
  struct intel64_tim_dev_s   *tch;
  volatile oneshot_handler_t  handler;
  volatile void              *arg;
  uint32_t                    frequency;
  uint32_t                    period;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: intel64_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper
 *
 * Input Parameters:
 *   oneshot    Caller allocated instance of the oneshot state structure
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int intel64_oneshot_initialize(struct intel64_oneshot_s *oneshot, int chan,
                               uint16_t resolution);

/****************************************************************************
 * Name: intel64_oneshot_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 ****************************************************************************/

int intel64_oneshot_max_delay(struct intel64_oneshot_s *oneshot,
                              uint64_t *usec);

/****************************************************************************
 * Name: intel64_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           intel64_oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int intel64_oneshot_start(struct intel64_oneshot_s *oneshot,
                          oneshot_handler_t handler, void *arg,
                          const struct timespec *ts);

/****************************************************************************
 * Name: intel64_oneshot_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           intel64_oneshot_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.  A time of zero is returned if the timer is
 *           not running.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int intel64_oneshot_cancel(struct intel64_oneshot_s *oneshot,
                           struct timespec *ts);

/****************************************************************************
 * Name: intel64_oneshot_current
 *
 * Description:
 *   Get the current time.
 *
 ****************************************************************************/

int intel64_oneshot_current(struct intel64_oneshot_s *oneshot,
                            uint64_t *usec);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_64_SRC_INTEL64_INTEL64_ONESHOT_H */
