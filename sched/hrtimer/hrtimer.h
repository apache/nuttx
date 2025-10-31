/****************************************************************************
 * sched/hrtimer/hrtimer.h
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

#ifndef __SCHED_HRTIMER_H
#define __SCHED_HRTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/hrtimer.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_clock_setexpire
 *
 * Description:
 *   Program the hardware to expire at the specified time.
 *
 * Input Parameters:
 *   upper - Timer upper-half instance
 *   ns    - Expiration time in nanoseconds
 *
 * Returned Value:
 *   OK on success; -ENOTSUP if not supported.
 *
 ****************************************************************************/

static inline int
hrtimer_clock_setexpire(FAR struct hrtimer_upperhalf_s *upper, uint64_t ns)
{
  int ret = -ENOTSUP;
  struct timespec ts;
  clock_nsec2time(&ts, ns);

  switch (upper->clockid)
    {
#ifdef CONFIG_ALARM_ARCH
      case HRTIMER_CLOCK_ALARM:
        ret = up_alarm_start(&ts);
        break;
#endif
#ifdef CONFIG_TIMER_ARCH
      case HRTIMER_CLOCK_TIMER:
        ret = up_timer_start(&ts);
        break;
#endif
      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: hrtimer_clock_current
 *
 * Description:
 *   Get the current time of the underlying hardware clock.
 *
 * Input Parameters:
 *   upper - Timer upper-half instance
 *   ns    - Location to return current time (nanoseconds)
 *
 * Returned Value:
 *   OK on success; hardware-dependent error code on failure.
 *
 ****************************************************************************/

static inline int
hrtimer_clock_current(FAR struct hrtimer_upperhalf_s *upper, uint64_t *ns)
{
  int ret = -ENOTSUP;
  struct timespec ts;

  switch (upper->clockid)
    {
#ifdef CONFIG_ALARM_ARCH
      case HRTIMER_CLOCK_ALARM:
        ret = up_alarm_gettime(&ts);
        break;
#endif
#ifdef CONFIG_TIMER_ARCH
      case HRTIMER_CLOCK_TIMER:
        ret = up_timer_gettime(&ts);
        break;
#endif
      default:
        break;
    }

  *ns = clock_time2nsec(&ts);
  return ret;
}

/****************************************************************************
 * Name: hrtimer_clock_start
 *
 * Description:
 *   Start the hardware clock associated with the given upper-half timer
 *   queue.
 *
 * Input Parameters:
 *   upper - Timer upper-half instance
 *
 * Returned Value:
 *   OK on success; -ENOTSUP if not supported.
 *
 ****************************************************************************/

static inline int
hrtimer_clock_start(FAR struct hrtimer_upperhalf_s *upper)
{
  int ret = -ENOTSUP;

  switch (upper->clockid)
    {
#ifdef CONFIG_ALARM_ARCH
      case HRTIMER_CLOCK_ALARM:
        ret = up_alarm_trigger();
        break;
#endif
#ifdef CONFIG_TIMER_ARCH
      case HRTIMER_CLOCK_TIMER:
        ret = up_timer_trigger();
        break;
#endif
      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: hrtimer_clock_trigger
 *
 * Description:
 *   Force trigger of the hardware clock associated with the upper-half
 *   timer queue.
 *
 * Input Parameters:
 *   upper - Timer upper-half instance
 *
 * Returned Value:
 *   OK on success; -ENOTSUP if not supported.
 *
 ****************************************************************************/

static inline int
hrtimer_clock_trigger(FAR struct hrtimer_upperhalf_s *upper)
{
  int ret = -ENOTSUP;

  switch (upper->clockid)
    {
#ifdef CONFIG_ALARM_ARCH
      case HRTIMER_CLOCK_ALARM:
        ret = up_alarm_trigger();
        break;
#endif
#ifdef CONFIG_TIMER_ARCH
      case HRTIMER_CLOCK_TIMER:
        ret = up_timer_trigger();
        break;
#endif
      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: hrtimer_has_expired
 *
 * Description:
 *   Check whether a timer has already expired.
 *
 * Input Parameters:
 *   expiration_time - Expiration timestamp
 *   current_time    - Current timestamp
 *
 * Returned Value:
 *   True if expired; false otherwise.
 *
 ****************************************************************************/

static inline bool
hrtimer_has_expired(uint64_t expiration_time,
                    uint64_t current_time)
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
 *   Program the hardware to expire at the specified time. If the time has
 *   already passed, trigger immediately.
 *
 * Input Parameters:
 *   upper           - Timer upper-half instance
 *   expiration_time - Expiration timestamp
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static inline int
hrtimer_setexpire(FAR struct hrtimer_upperhalf_s *upper,
                  uint64_t expiration_time)
{
  uint64_t now;
  int ret = OK;

  hrtimer_clock_setexpire(upper, expiration_time);
  hrtimer_clock_current(upper, &now);

  /* If expiration already passed, trigger immediately */

  if (hrtimer_has_expired(expiration_time, now))
    {
      ret = hrtimer_clock_trigger(upper);
    }

  return ret;
}

/****************************************************************************
 * Name: hrtimer_cmp
 *
 * Description:
 *   Compare two hrtimer nodes for ordering in the RB tree.
 *
 * Input Parameters:
 *   a - First node
 *   b - Second node
 *
 * Returned Value:
 *   <0 if a < b, 0 if equal, >0 if a > b
 *
 ****************************************************************************/

static inline int
hrtimer_cmp(struct hrtimer_node *a, struct hrtimer_node *b)
{
  return hrtimer_has_expired(b->expiration_time, a->expiration_time);
}
RB_PROTOTYPE(hrtimer_tree, hrtimer_node, entry, hrtimer_cmp);

#endif /* __SCHED_HRTIMER_H */
