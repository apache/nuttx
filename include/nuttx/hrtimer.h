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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

#include <sys/tree.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum representable timer value */

#define HRTIMER_MAX_VALUE UINT64_MAX

/* The maximum increment that can be safely distinguished without overflow */

#define HRTIMER_MAX_INCREMENT     (HRTIMER_MAX_VALUE >> 1)

/* Default expiration increment if no timer is active */

#define HRTIMER_DEFAULT_INCREMENT ((uint64_t)UINT32_MAX >> 2)

/* Maximum number of hrtimer process handlers invoked in one batch */

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

/* Clock source for hrtimers */

typedef enum
{
  HRTIMER_CLOCK_ALARM = 0x0, /* Alarm-based hardware clock */
  HRTIMER_CLOCK_TIMER = 0x1  /* Timer-based hardware clock */
} htimer_clockid_t;

/* Forward declarations */

struct hrtimer_s;
struct hrtimer_node;
struct hrtimer_upperhalf_s;

/* Callback type for high-resolution timer expiration */

typedef void (*hrtimer_callback_t)(FAR struct hrtimer_s *);

/* High-resolution timer tree node */

struct hrtimer_node
{
  uint64_t expiration_time;         /* Expiration time in nanoseconds */
  RB_ENTRY(hrtimer_node) entry;     /* Red-black tree linkage */
};

/* Red-black tree root for hrtimers */

RB_HEAD(hrtimer_tree, hrtimer_node);

/* High-resolution timer instance */

struct hrtimer_s
{
  struct hrtimer_node         node;      /* Expiration time node */
  FAR struct hrtimer_upperhalf_s *upper; /* Associated timer queue */
  hrtimer_callback_t          callback;  /* Expiration callback */
  FAR void                   *args;      /* Optional callback argument */
};

/* High-resolution timer upper-half instance */

struct hrtimer_upperhalf_s
{
  FAR struct hrtimer_tree tree;     /* Timer tree (RB tree of active timers) */
  htimer_clockid_t        clockid;  /* Clock source type */
  bool                    started;  /* True if timer queue has started */
  spinlock_t              lock;     /* Spinlock protecting the queue */
};

/* Global default hrtimer queue and systick timer */

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
 *   Reload (reschedule) a high-resolution timer by advancing its expiration
 *   time with the specified interval in nanoseconds.
 *
 * Input Parameters:
 *   hrtimer - Timer instance to reload
 *   ns      - Interval to add in nanoseconds
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_reload(FAR struct hrtimer_s *hrtimer, uint64_t ns);

/****************************************************************************
 * Name: hrtimer_upper_start
 *
 * Description:
 *   Start the high-resolution timer queue.
 *
 * Input Parameters:
 *   upper   - Timer upper-half instance
 *   clockid - Clock source identifier
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_upper_start(FAR struct hrtimer_upperhalf_s *upper,
                        htimer_clockid_t clockid);

/****************************************************************************
 * Name: hrtimer_upper_process
 *
 * Description:
 *   Process and handle expired timers in the queue.
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
 *   Add a timer with an absolute expiration time.
 *
 * Input Parameters:
 *   upper   - Timer upper-half instance
 *   hrtimer - Timer instance
 *   start   - Absolute expiration time
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_upper_add_abs(FAR struct hrtimer_upperhalf_s *upper,
                          FAR struct hrtimer_s *hrtimer,
                          uint64_t start);

/****************************************************************************
 * Name: hrtimer_upper_add_rel
 *
 * Description:
 *   Add a timer with a relative delay from now.
 *
 * Input Parameters:
 *   upper     - Timer upper-half instance
 *   hrtimer   - Timer instance
 *   increment - Relative delay in nanoseconds
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_upper_add_rel(FAR struct hrtimer_upperhalf_s *upper,
                          FAR struct hrtimer_s *hrtimer,
                          uint64_t increment);

/****************************************************************************
 * Name: hrtimer_init
 *
 * Description:
 *   Initialize a high-resolution timer instance.
 *
 * Input Parameters:
 *   hrtimer  - Timer instance
 *   callback - Expiration callback
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
 *   Cancel a running high-resolution timer.
 *
 * Input Parameters:
 *   hrtimer - Timer instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_cancel(FAR struct hrtimer_s *hrtimer);

/****************************************************************************
 * Name: hrtimer_start
 *
 * Description:
 *   Start a high-resolution timer in either absolute or relative mode.
 *
 * Input Parameters:
 *   hrtimer - Timer instance
 *   ns      - Expiration time in nanoseconds
 *   mode    - Expiration mode (absolute or relative)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static inline int
hrtimer_start(FAR struct hrtimer_s *hrtimer,
              uint64_t ns,
              const enum hrtimer_mode mode)
{
  FAR struct hrtimer_upperhalf_s *upper = &g_default_hrtimer_upperhalf;

  if (mode == HRTIMER_MODE_ABS)
    {
      return hrtimer_upper_add_abs(upper, hrtimer, ns);
    }
  else
    {
      return hrtimer_upper_add_rel(upper, hrtimer, ns);
    }
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_HRTIMER_H */
