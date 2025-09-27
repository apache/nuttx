/****************************************************************************
 * include/nuttx/hrtimer.h
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

#ifndef __INCLUDE_NUTTX_HRTIMER_H
#define __INCLUDE_NUTTX_HRTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/hrtimer.h>

#include <sys/tree.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_TIME64
#  define HRTIMER_MAX_VALUE UINT64_MAX
#else
#  define HRTIMER_MAX_VALUE UINT32_MAX
#endif

/* The maximum increment that can be safely distinguished without overflow */

#define HRTIMER_MAX_INCREMENT     (HRTIMER_MAX_VALUE >> 1)

/* Default expiration increment if no timer is active */

#define HRTIMER_DEFAULT_INCREMENT ((clock_t)UINT32_MAX >> 2)

/* Maximum number of hrtimer process handlers */

#define MAX_HRTIMER_PROCESS_CNT   4u

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Timer mode: absolute or relative */

enum hrtimer_mode
{
  HRTIMER_MODE_ABS = 0x0, /* Absolute expiration time */
  HRTIMER_MODE_REL = 0x1  /* Relative delay from now  */
};

/* Forward declarations */

struct hrtimer_s;
struct hrtimer_node;
struct hrtimer_upperhalf_s;

/* Callback type for high-resolution timer expiration */

typedef void (*hrtimer_callback_t)(FAR struct hrtimer_s *);

/* High-resolution timer tree node */

struct hrtimer_node
{
  clock_t   expiration_time;          /* Expiration time */
  RB_ENTRY(hrtimer_node) entry;       /* Red-black tree link */
};

/* Red-black tree root for hrtimers */

RB_HEAD(hrtimer_tree, hrtimer_node);

/* High-resolution timer instance */

struct hrtimer_s
{
  struct hrtimer_node        node;       /* Expiration time node */
  FAR struct hrtimer_upperhalf_s *upper; /* Associated timer queue */
  hrtimer_callback_t         callback;   /* Expiration callback */
  FAR void                   *args;      /* Optional callback argument */
};

/* High-resolution timer upper-half instance */

struct hrtimer_upperhalf_s
{
  FAR struct hrtimer_tree        tree;    /* Timer tree */
  FAR struct hrtimer_lowerhalf_s *lower;  /* Lower-half hardware driver */
  bool                           started; /* True if timer queue started */
  spinlock_t                     lock;    /* Protection spinlock */
};

extern struct hrtimer_upperhalf_s g_default_hrtimer_upperhalf;
extern struct hrtimer_s g_hrtimer_systick_timer;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Name: hrtimer_reload
 *
 * Description:
 *   Reload an existing high-resolution timer with a new expiration time.
 *
 * Input Parameters:
 *   hrtimer - Timer instance to reload
 *   time    - Expiration time in ticks
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_reload(FAR struct hrtimer_s *hrtimer, clock_t time);

/****************************************************************************
 * Name: hrtimer_upper_start
 *
 * Description:
 *   Start the high-resolution timer queue.
 *
 * Input Parameters:
 *   upper - Timer upper-half instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_upper_start(FAR struct hrtimer_upperhalf_s *upper);

/****************************************************************************
 * Name: hrtimer_upper_process
 *
 * Description:
 *   Process expired timers in the queue.
 *
 * Input Parameters:
 *   upper - Timer upper-half instance
 *
 ****************************************************************************/

void hrtimer_upper_process(FAR struct hrtimer_upperhalf_s *upper);

/****************************************************************************
 * Name: hrtimer_upper_add_abs
 *
 * Description:
 *   Add a timer to the queue with an absolute expiration time.
 *
 * Input Parameters:
 *   upper   - Timer upper-half instance
 *   hrtimer - Timer instance
 *   start   - Absolute start time
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_upper_add_abs(FAR struct hrtimer_upperhalf_s *upper,
                          FAR struct hrtimer_s *hrtimer,
                          clock_t start);

/****************************************************************************
 * Name: hrtimer_upper_add_rel
 *
 * Description:
 *   Add a timer to the queue with a relative delay.
 *
 * Input Parameters:
 *   upper     - Timer upper-half instance
 *   hrtimer   - Timer instance
 *   increment - Delay in ticks
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_upper_add_rel(FAR struct hrtimer_upperhalf_s *upper,
                          FAR struct hrtimer_s *hrtimer,
                          clock_t increment);

/****************************************************************************
 * Name: hrtimer_upper_set_lower
 *
 * Description:
 *   Set the lower-half driver for the given timer queue.
 *
 * Input Parameters:
 *   upper - Timer upper-half instance
 *   lower - Hardware lower-half driver
 *
 ****************************************************************************/

static inline void
hrtimer_upper_set_lower(FAR struct hrtimer_upperhalf_s *upper,
                        FAR struct hrtimer_lowerhalf_s *lower)
{
  upper->lower = lower;
}

/****************************************************************************
 * Name: hrtimer_init
 *
 * Description:
 *   Initialize a high-resolution timer instance with callback and argument.
 *
 * Input Parameters:
 *   hrtimer  - Timer instance
 *   callback - Expiration callback function
 *   args     - Callback argument
 *
 ****************************************************************************/

static inline void
hrtimer_init(FAR struct hrtimer_s *hrtimer,
             hrtimer_callback_t callback,
             FAR void *args)
{
  hrtimer->node.expiration_time = 0;
  hrtimer->upper = NULL;
  hrtimer->callback = callback;
  hrtimer->args = args;
}

/****************************************************************************
 * Name: hrtimer_cancel
 *
 * Description:
 *   Cancel a currently running high-resolution timer.
 *
 * Input Parameters:
 *   hrtimer - Timer instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_cancel(FAR struct hrtimer_s *hrtimer);

/****************************************************************************
 * Name: hrtimer_start
 *
 * Description:
 *   Start a high-resolution timer, either with absolute or relative time.
 *
 * Input Parameters:
 *   hrtimer - Timer instance
 *   ns      - Expiration time in nanoseconds
 *   mode    - Expiration mode (absolute/relative)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value on failure.
 *
 ****************************************************************************/

static inline int
hrtimer_start(FAR struct hrtimer_s *hrtimer,
              uint64_t ns,
              const enum hrtimer_mode mode)
{
  FAR struct hrtimer_upperhalf_s *upper = &g_default_hrtimer_upperhalf;
  clock_t time = HRTIMER_NSEC2TIME(upper->lower, ns);

  if (mode == HRTIMER_MODE_ABS)
    {
      return hrtimer_upper_add_abs(upper, hrtimer, time);
    }
  else
    {
      return hrtimer_upper_add_rel(upper, hrtimer, time);
    }
}

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_has_expired
 *
 * Description:
 *   Check whether a timer has expired.
 *
 * Input Parameters:
 *   expiration_time - Expiration time
 *   current_time    - Current time
 *
 * Returned Value:
 *   True if expired; false otherwise.
 *
 ****************************************************************************/

static inline bool
hrtimer_has_expired(clock_t expiration_time,
                    clock_t current_time)
{
  if (expiration_time > current_time)
    {
      return (expiration_time - current_time) > HRTIMER_MAX_INCREMENT;
    }
  else if (expiration_time < current_time)
    {
      return (current_time - expiration_time) <= HRTIMER_MAX_INCREMENT;
    }
  else
    {
      return true;
    }
}

/****************************************************************************
 * Name: hrtimer_setexpire
 *
 * Description:
 *   Program the hardware to expire at the specified time.
 *
 * Input Parameters:
 *   upper           - Timer upper-half instance
 *   expiration_time - Expiration time
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value on failure.
 *
 ****************************************************************************/

static inline int
hrtimer_setexpire(FAR struct hrtimer_upperhalf_s *upper,
                        clock_t expiration_time)
{
  FAR struct hrtimer_lowerhalf_s *lower = upper->lower;
  clock_t now;
  int ret = OK;

  HRTIMER_SETEXPIRE(lower, expiration_time);
  now = HRTIMER_CURRENT(lower);

  /* If expiration already passed, trigger immediately */

  if (hrtimer_has_expired(expiration_time, now))
    {
      ret = HRTIMER_TRIGGER(lower);
    }

  return ret;
}

/****************************************************************************
 * Name: hrtimer_cmp
 *
 * Description:
 *   Compare function for RB tree ordering.
 *
 * Input Parameters:
 *   a - First node
 *   b - Second node
 *
 * Returned Value:
 *   Comparison result for RB tree insertion.
 *
 ****************************************************************************/

static inline int
hrtimer_cmp(struct hrtimer_node *a, struct hrtimer_node *b)
{
  return hrtimer_has_expired(b->expiration_time, a->expiration_time);
}

RB_PROTOTYPE(hrtimer_tree, hrtimer_node, entry, hrtimer_cmp)

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_HRTIMER_H */
