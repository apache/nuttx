/****************************************************************************
 * include/nuttx/timers/hrtimer.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_HRTIMER_H
#define __INCLUDE_NUTTX_TIMERS_HRTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/clock.h>
#include <stdbool.h>
#include <sys/types.h>
#include <assert.h>
#include <errno.h>

#ifdef CONFIG_HRTIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Method access helper macros **********************************************/

/* Set the expiration time (in absolute clock ticks).
 * Argument: expiration time
 */

#define HRTIMER_SETEXPIRE(l,t) \
  ((l)->ops->setexpire ? (l)->ops->setexpire(l,t) : -ENOTSUP)

/* Get the current time (in clock ticks).
 * Argument: Ignored
 */

#define HRTIMER_CURRENT(l) \
  ((l)->ops->current ? (l)->ops->current(l) : -ENOTSUP)

/* Start the high-resolution timer.
 * Argument: Ignored
 */

#define HRTIMER_START(l) \
  ((l)->ops->start ? (l)->ops->start(l) : -ENOTSUP)

/* Trigger the timer immediately (force expiration).
 * Argument: Ignored
 */

#define HRTIMER_TRIGGER(l) \
  ((l)->ops->trigger ? (l)->ops->trigger(l) : -ENOTSUP)

#define HRTIMER_USEC2TIME(l, us) \
  (((clock_t)(us)) * ((l)->freq) / USEC_PER_SEC)

#define HRTIMER_NSEC2TIME(l, ns) \
  (((clock_t)(ns)) * ((l)->freq) / NSEC_PER_SEC)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Forward declaration */

struct hrtimer_lowerhalf_s;

/* This structure defines the operations provided by the "lower-half" HRTIMER
 * driver to the "upper-half" driver.
 */

struct hrtimer_ops_s
{
  /* Set the timer expiration time (absolute time in clock ticks). */

  CODE int (*setexpire)(FAR struct hrtimer_lowerhalf_s *lower,
                        clock_t expiration_time);

  /* Get the current time (in clock ticks). */

  CODE clock_t (*current)(FAR struct hrtimer_lowerhalf_s *lower);

  /* Start the timer counting. */

  CODE int (*start)(FAR struct hrtimer_lowerhalf_s *lower);

  /* Trigger the timer immediately, simulating expiration. */

  CODE int (*trigger)(FAR struct hrtimer_lowerhalf_s *lower);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  The real lower half driver will
 * extend this structure with its own private data.
 */

struct hrtimer_lowerhalf_s
{
  /* Publicly visible portion of the "lower-half" driver state structure. */

  FAR const struct hrtimer_ops_s *ops; /* Lower half operations */

  /* Frequency of the high-resolution timer in Hz. */

  uint64_t freq;

  /* The remainder of the structure is private to the lower half driver. */
};

#endif /* CONFIG_HRTIMER */
#endif /* __INCLUDE_NUTTX_TIMERS_HRTIMER_H */
