/****************************************************************************
 * sched/hrtimer/hrtimer.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>

#include "sched/sched.h"
#include "hrtimer.h"
#include "hrtimer_internal.h"

#include <debug.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static hrtimer_queue_t g_hrtimer_queue;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* The reprogramming function can be fully inlined. */

static inline_function void hrtimer_reprogram(uint64_t next_expired)
{
#ifdef CONFIG_SCHED_TICKLESS
  struct timespec ts;
#  ifdef CONFIG_SCHED_TICKLESS_ALARM
  clock_nsec2time(&ts, next_expired);
  up_alarm_start(&ts);
#  else
  struct timespec current;
  up_timer_gettime(&current);
  clock_nsec2time(&ts, next_expired);
  clock_timespec_subtract(&ts, &current, &ts);
  up_timer_start(&ts);
#  endif
#else
  UNUSED(next_expired);
#endif
}

/****************************************************************************
 * Name: hrtimer_process
 *
 * Description:
 *   Process the time expiration of the hrtimer queue.
 *
 * Input Parameters:
 *   queue   - The hrtimer queue.
 *   current - The current time.
 *   reprogram - The timer reprogram function.
 *
 * Returned Value:
 *   The next expiration time of the hrtimer queue.
 *
 * Assumption:
 *   The caller should be in the interrupt context, where the cpu core id
 *   should not change.
 *
 ****************************************************************************/

static inline_function
uint64_t hrtimer_process(FAR hrtimer_queue_t *queue, uint64_t current)
{
  FAR hrtimer_t    *timer;
  hrtimer_callback_t func;
  uint64_t   next_expired;
  uint64_t        expired;
  int                 cpu = this_cpu();
  irqstate_t        flags = write_seqlock_irqsave(&queue->lock);

  /* Check if we need processing the expiration. */

  while (HRTIMER_TIME_BEFORE_EQ(queue->next_expired, current))
    {
      timer   = hrtimer_queue_peek(queue);

      /* Remove the hrtimer from the head of the queue
       * and update the next_expired.
       */

      func    = timer->func;
      expired = timer->expired;

      /* Clear the pending state */

      timer->func = NULL;
      hrtimer_queue_del(queue, timer);

      /* Mark the hrtimer in the running state. In this case,
       * User can not reclaim (deallocate or modify) the hrtimer.
       */

      hrtimer_queue_mark_running(queue, timer, cpu);
      write_sequnlock_irqrestore(&queue->lock, flags);

      /* Execute the hrtimer function. Here we can not modify the hrtimer
       * anymore because the hrtimer might be modified by other users.
       * The expired time is passing as the callback version to help users
       * to avoid race conditions.
       */

      next_expired = expired + func(timer, expired);
      DEBUGASSERT(HRTIMER_TIME_AFTER_EQ(next_expired, expired));

      flags = write_seqlock_irqsave(&queue->lock);

      /* In the interrupt context, the cpu core id should not change.
       * NuttX currently does not support kernel preemption, thus interrupts
       * cannot be nested within interrupt context.
       * Even if kernel preemption and interrupt nesting are supported in the
       * future, we can avoid nested calls to `hrtimer_expiry` by an
       * interrupt nesting counter.
       * Besides, in SMP mode, during interrupt handling, we use per-core
       * `ARCH_INTERRUPTSTACK`, ensuring that the core ID remains unchanged
       * while in interrupt context.
       */

      DEBUGASSERT(cpu == this_cpu());

      /* If the running timer has not changed, we can re-enqueue the
       * timer or release the ownership. Otherwise, the hrtimer should
       * have been cancelled. In this case, we lose the ownership of the
       * hrtimer and can not modify this hrtimer anymore.
       */

      if (next_expired != expired &&
          hrtimer_queue_has_ownership(queue, timer, cpu))
        {
          /* Re-enqueue the hrtimer if it is periodic. */

          timer->func    = func;
          timer->expired = next_expired;
          hrtimer_queue_add(queue, timer);
        }

      /* Clear the running state and release the hrtimer reference. */

      hrtimer_queue_mark_running(queue, NULL, cpu);
    }

  next_expired = queue->next_expired;
  hrtimer_reprogram(next_expired);

  write_sequnlock_irqrestore(&queue->lock, flags);

  return next_expired;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_initialize
 *
 * Description:
 *   Initialize the high-resolution timer queue for timing subsystem.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void hrtimer_initialize(void)
{
  FAR hrtimer_queue_t *queue = &g_hrtimer_queue;
  FAR hrtimer_t *guard_timer = &queue->guard_timer;

#if !defined(CONFIG_SCHED_TICKLESS)
  swarn("WARNING: The system hrtimer is running in \
         low-resolution (tick) mode.");
#endif

  /* We must ensure the clock_t is 64-bit. */

  DEBUGASSERT(sizeof(clock_t) >= sizeof(uint64_t));

  seqlock_init(&queue->lock);
  hrtimer_queue_clear(queue);

  /* The guard timer is designed to ensure the system will enter the
   * safe state if the timing system is not working properly.
   * It can be customized by the user after the hrtimer_queue_init
   * and before the first user hrtimer is added.
   * If the guard timer fired, it means 292-years have passed, which is
   * impossible. In this case we should trigger the kernel panic and
   * reboot the system. Here we assume the system will reboot after
   * jumping to NULL.
   */

  guard_timer->func    = NULL;
  guard_timer->expired = INT64_MAX;
  queue->next_expired  = UINT64_MAX;
  hrtimer_queue_add(queue, guard_timer);

  memset(queue->running, 0, sizeof(queue->running));
}

/****************************************************************************
 * Name: hrtimer_expiry
 *
 * Description:
 *   This function is called by the timer interrupt handler to handle
 *   if a hrtimer has expired.
 *
 * Input Parameters:
 *   nsec - The expiration time in nanoseconds.
 *   noswitches - True: Disable context switches.
 *
 * Returned Value:
 *   The next expiration time in nanoseconds.
 *
 * Assumption:
 *   The caller should be in the interrupt context.
 *
 ****************************************************************************/

uint64_t hrtimer_expiry(uint64_t nsec, bool noswitches)
{
  FAR hrtimer_queue_t *queue = &g_hrtimer_queue;
  return noswitches ? hrtimer_queue_read(queue, &queue->next_expired) :
                      hrtimer_process(queue, nsec);
}

/****************************************************************************
 * Name: hrtimer_async_restart
 *
 * Description:
 *   Restart the hrtimer with relative or absolute time in nanoseconds.
 *   These functions allow restarting the hrtimer that has been set to
 *   cancelled state via `hrtimer_async_cancel`.
 *   Please be aware of concurrency issues. Concurrency errors are prone to
 *   occur in this use case.
 *
 * Input Parameters:
 *   timer - The hrtimer to be started.
 *   func  - The callback function to be called when the timer expires.
 *   arg   - The argument to be passed to the callback function.
 *   time  - The relative or absolute expiration time in nanoseconds.
 *   mode  - The timer mode, relative or absolute.
 *
 * Returned Value:
 *   Zero on success.
 *   -EINVAL on if one of the parameter is invalid.
 *
 * Assumption:
 *   The timer and func should be not NULL.
 *   The timer should be cancelled or completed.
 *
 ****************************************************************************/

int hrtimer_async_restart(FAR hrtimer_t *timer)
{
  FAR hrtimer_queue_t *queue = &g_hrtimer_queue;
  irqstate_t           flags = write_seqlock_irqsave(&queue->lock);

  /* The queue should not have the timer ownership before. */

  DEBUGASSERT(hrtimer_queue_count_ownership(queue, (uintptr_t)timer) == 0);

  /* Reprogram the timer when the queue head has changed. */

  if (hrtimer_queue_add(queue, timer))
    {
      hrtimer_reprogram(queue->next_expired);
    }

  write_sequnlock_irqrestore(&queue->lock, flags);

  return OK;
}

int hrtimer_async_start(FAR hrtimer_t *timer)
{
  /* This timer should never be executing. */

  DEBUGASSERT(hrtimer_queue_count_ownership(&g_hrtimer_queue,
                                            (uintptr_t)timer) == 0);
  DEBUGASSERT(hrtimer_queue_count_ownership(&g_hrtimer_queue,
                                     (uintptr_t)timer | 0x1u) == 0);
  hrtimer_async_restart(timer);
  return OK;
}

/****************************************************************************
 * Name: hrtimer_cancel
 *
 * Description:
 *   Cancel the hrtimer asynchronously. This function set the timer to the
 *   cancelled state. The caller will acquire the limited ownership of the
 *   hrtimer, which allow the caller restart the hrtimer, but the callback
 *   function may still be executing on another CPU, which prevent the caller
 *   from freeing the hrtimer. The caller must call `hrtimer_cancel` to wait
 *   for the callback to be finished. Please use the function with care.
 *   Concurrency errors are prone to occur in this use case.
 *
 * Input Parameters:
 *   timer - The hrtimer to be cancelled.
 *
 * Returned Value
 *   OK on success. > 0 on if the timer callback is running.
 *
 * Assumption:
 *   The timer should not be NULL.
 *
 ****************************************************************************/

int hrtimer_cancel(FAR hrtimer_t *timer)
{
  int                    ret;
#ifdef CONFIG_SMP
  uintptr_t        cancelled = (uintptr_t)timer | 0x1u;
#endif
  FAR hrtimer_queue_t *queue = &g_hrtimer_queue;
  irqstate_t           flags = write_seqlock_irqsave(&queue->lock);

  /* The timer pointer should be at least 2-bytes aligned. */

  DEBUGASSERT(timer);
  DEBUGASSERT(((uintptr_t)timer & 0x1u) == 0u);

  /* Revoke the hrtimer ownership. */

  ret = hrtimer_queue_revoke_ownership(queue, cancelled);

  /* Remove the hrtimer from the queue. */

  if (HRTIMER_ISPENDING(timer))
    {
      /* Clear the pending state. */

      timer->func = NULL;

      /* Reprogram the timer when the queue head has changed. */

      if (hrtimer_queue_del(queue, timer))
        {
          hrtimer_reprogram(queue->next_expired);
        }
    }

  write_sequnlock_irqrestore(&queue->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: hrtimer_cancel_sync
 *
 * Description:
 *   Cancel the hrtimer and synchronously wait the callback to be finished.
 *   This function set the timer to the cancelled state and wait for all
 *   references to be released. The caller will then acquire full ownership
 *   of the hrtimer. After the function returns, the caller can safely
 *   deallocate the hrtimer.
 *
 * Input Parameters:
 *   timer - The hrtimer to be cancelled.
 *
 * Returned Value
 *   OK on success.
 *
 * Assumption:
 *   The timer should not be NULL.
 *
 ****************************************************************************/

int hrtimer_cancel_sync(FAR hrtimer_t *timer)
{
  int ret = hrtimer_cancel(timer);

  if (ret > 0)
    {
      /* Wait the timer to finish and reclaim the ownership. */

      hrtimer_queue_wait(&g_hrtimer_queue, timer);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: hrtimer_gettime
 *
 * Description:
 *   Get the rest of the delay time of the hrtimer in nanoseconds.
 *
 * Input Parameters:
 *   timer - The hrtimer to be queried.
 *
 * Returned Value
 *   The time until next expiration in nanoseconds.
 *
 * Assumption:
 *   The timer should not be NULL.
 *
 ****************************************************************************/

uint64_t hrtimer_gettime(FAR hrtimer_t *timer)
{
  uint64_t expire = hrtimer_queue_read_64(&g_hrtimer_queue, &timer->expired);
  int64_t  remain = expire - clock_systime_nsec();

  return remain < 0 ? 0u : remain;
}
