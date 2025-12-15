/****************************************************************************
 * include/nuttx/hrtimer/hrtimer_queue.h
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

#ifndef __INCLUDE_HRTIMER_QUEUE_H
#define __INCLUDE_HRTIMER_QUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>
#include <nuttx/clock.h>
#include <nuttx/seqlock.h>

#include <nuttx/hrtimer_queue_type.h>

#include <stdint.h>

/* This header file should be only included for internal use,
 * DO NOT EXPOSE IT TO USERS.
 *
 * Before including this file, Please provide the following inputs:
 *
 * Include the hrtimer_type_xxx.h header file. This header file should
 * provide the implementation of the queue operations and definition for
 * internal hrtimer.
 *
 * Function implementation: static inline_function
 * void hrtimer_reprogram(FAR USER_HRTIMER_QUEUE_TYPE *queue,
 *                        uint64_t next_expired);
 *   - Reprogram the timer hardware to the next expired time.
 */

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline function
 ****************************************************************************/

/* The relied function `hrtimer_reprogram` and `hrtimer_wait_policy` must be
 * implemented:
 *
 * void hrtimer_reprogram(FAR hrtimer_queue_internal_t *queue,
 *                        uint64_t next_expired);
 *   - This function abstracts how to reprogram the timer hardware.
 *
 * void hrtimer_wait_policy(void);
 *   - This function abstracts how to wait for the callback finished.
 *
 * Please note the forward declaration and reverse dependency here.
 * We instead of the function pointers because most functional-safety
 * compilers (E.g. GHC, Tasking and CompCert C) do not support inlining
 * function pointers, which would introduce additional memory and
 * performance overhead.
 */

static inline_function
void hrtimer_reprogram(FAR hrtimer_queue_internal_t *queue,
                       uint64_t next_expired);

static inline_function
void hrtimer_wait_policy(void);

/* Queue dependent functions should be provided by `hrtimer_type_xxx.h`. */

static inline_function
FAR hrtimer_internal_t *hrtimer_queue_peek(
                        FAR hrtimer_queue_internal_t *queue);

static inline_function
bool hrtimer_queue_add(FAR hrtimer_queue_t *queue, FAR hrtimer_t *timer);

static inline_function
bool hrtimer_queue_del(FAR hrtimer_queue_t *queue, FAR hrtimer_t *timer);

static inline_function
void hrtimer_queue_clear(FAR hrtimer_queue_t *queue);

/* Reusable library code for user-defined high-resolution timer queue. */

/****************************************************************************
 * Name: hrtimer_queue_lock/unlock
 *
 * Description:
 *   Lock/Unlock the hrtimer queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *
 * Returned Value:
 *   The previous interrupt state.
 *
 ****************************************************************************/

static inline_function
void hrtimer_queue_lock_init(FAR hrtimer_queue_internal_t *queue)
{
  seqlock_init(&queue->lock);
}

static inline_function
irqstate_t hrtimer_queue_lock(FAR hrtimer_queue_internal_t *queue)
{
  return write_seqlock_irqsave(&queue->lock);
}

static inline_function
void hrtimer_queue_unlock(FAR hrtimer_queue_internal_t *queue,
                          irqstate_t flags)
{
  write_sequnlock_irqrestore(&queue->lock, flags);
}

/****************************************************************************
 * Name: hrtimer_queue_locked_read_64/32
 *
 * Description:
 *   Read the value in the queue atomically.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *
 * Returned Value:
 *   The value in the queue.
 *
 ****************************************************************************/

static inline_function
uint64_t hrtimer_queue_locked_read_64(FAR hrtimer_queue_internal_t *queue,
                                      FAR const uint64_t *ptr)
{
  uint64_t val;
  uint32_t seq;

  do
    {
      seq = read_seqbegin(&queue->lock);
      val = *ptr;
    }
  while (read_seqretry(&queue->lock, seq));

  return val;
}

static inline_function
uint32_t hrtimer_queue_locked_read_32(FAR hrtimer_queue_internal_t *queue,
                                      FAR const uint32_t *ptr)
{
  uint32_t val;
  uint32_t seq;

  do
    {
      seq = read_seqbegin(&queue->lock);
      val = *ptr;
    }
  while (read_seqretry(&queue->lock, seq));

  return val;
}

/****************************************************************************
 * Name: hrtimer_queue_read_64/32
 *
 * Description:
 *   Internal function to read the value in the queue atomically.
 *   Do not use this function if you are not sure about the thread-safe
 *   of the value you are reading.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *   ptr   - The pointer to be read.
 *
 * Returned Value:
 *   The value in the queue.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_64BIT
/* On 64-bit architectures, read/write uint64_t is atomic. */
#  define hrtimer_queue_read_64(queue, ptr) (*(FAR volatile uint64_t *)(ptr))
#else
#  define hrtimer_queue_read_64(queue, ptr) \
  hrtimer_queue_locked_read_64(queue, ptr)
#endif

#if UINT_MAX >= UINT32_MAX
/* On 32/64-bit architectures, read/write uint32_t is atomic. */
#  define hrtimer_queue_read_32(queue, ptr) (*(FAR volatile uint32_t *)(ptr))
#else
#  define hrtimer_queue_read_32(queue, ptr) \
  hrtimer_queue_locked_read_32(queue, ptr)
#endif

/* Generic function to read the value in the queue atomically. */

#define hrtimer_queue_read(queue, ptr) \
  (sizeof(*(ptr)) == 8u ? \
   hrtimer_queue_read_64(queue, (FAR const uint64_t *)(ptr)) : \
   (sizeof(*(ptr)) == 4u ? \
    hrtimer_queue_read_32(queue, (FAR const uint32_t *)(ptr)) : 0u))

/****************************************************************************
 * Name: hrtimer_queue_mark_running
 *
 * Description:
 *   Mark the timer as running.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *   timer - The timer to be marked.
 *   cpu   - The CPU core Id.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static inline_function
void hrtimer_queue_mark_running(FAR hrtimer_queue_internal_t *queue,
                                FAR hrtimer_internal_t *timer, int cpu)
{
  queue->running[cpu] = (uintptr_t)timer;
}
#else
#  define hrtimer_queue_mark_running(queue, timer, cpu)
#endif

/****************************************************************************
 * Name: hrtimer_queue_has_ownership
 *
 * Description:
 *   Check if the CPU core has ownership of the timer.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *   timer - The timer to be marked.
 *   cpu   - The CPU core Id.
 *
 * Returned Value:
 *   true if the CPU core has ownership of the timer, false otherwise.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
#  define hrtimer_queue_has_ownership(queue, timer, cpu) \
  ((queue)->running[cpu] == (uintptr_t)(timer))
#else
#  define hrtimer_queue_has_ownership(queue, timer, cpu) (true)
#endif

/* Helper function for debugging usage.
 * If the ownership count > 0, it indicates that users might forget to call
 * async_cancel to seize the timer ownership before restart the timer.
 */

#ifdef CONFIG_SMP
unused_code static inline_function
unsigned hrtimer_queue_count_ownership(FAR hrtimer_queue_internal_t *queue,
                                       FAR hrtimer_internal_t *timer)
{
  int      cpu;
  unsigned owner = 0u;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      if (hrtimer_queue_has_ownership(queue, timer, cpu))
        {
          owner++;
        }
    }

  return owner;
}
#else
#  define hrtimer_queue_count_ownership(queue, timer) 0u
#endif

/****************************************************************************
 * Name: hrtimer_queue_is_running
 *
 * Description:
 *   Check if the CPU core is running the timer. This can be called
 *   without the queue lock.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *   timer - The timer to be marked.
 *   cpu   - The CPU core Id.
 *
 * Returned Value:
 *   true if the CPU core has ownership of the timer, false otherwise.
 *
 ****************************************************************************/

#define hrtimer_queue_is_running(queue, timer, cpu) \
  (hrtimer_queue_read(queue, &(queue)->running[cpu]) == (uintptr_t)(timer))

/****************************************************************************
 * Name: hrtimer_queue_init
 *
 * Description:
 *   Initialize the hrtimer queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *
 * Returned Value:
 *   0 on OK, -EINVAL on error.
 *
 ****************************************************************************/

static inline_function
int hrtimer_queue_init(FAR hrtimer_queue_internal_t *queue)
{
  int ret = -EINVAL;
  int cpu;

  if (queue)
    {
      FAR hrtimer_internal_t *guard_timer = &queue->guard_timer;

      hrtimer_queue_lock_init(&queue->lock);
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

      hrtimer_fill(guard_timer, NULL, NULL, INT64_MAX);

      queue->next_expired = UINT64_MAX;
      hrtimer_queue_add(queue, guard_timer);

      memset(queue->running, 0, sizeof(queue->running));
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: hrtimer_queue_expiry
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
uint64_t hrtimer_queue_expiry(FAR hrtimer_queue_internal_t *queue,
                              uint64_t current)
{
  FAR hrtimer_internal_t *timer;
  FAR void                 *arg;
  hrtimer_callback_t       func;
  uint64_t         next_expired;
  uint64_t              expired;
  int                       cpu = this_cpu();
  irqstate_t              flags = hrtimer_queue_lock(&queue->lock);

  /* Check if we need processing the expiration. */

  while (HRTIMER_TIME_BEFORE_EQ(queue->next_expired, current))
    {
      timer   = hrtimer_queue_peek(queue);

      /* Remove the hrtimer from the head of the queue
       * and update the next_expired.
       */

      func    = timer->func;
      arg     = timer->arg;
      expired = timer->expired;

      /* Clear the pending state */

      timer->func = NULL;
      hrtimer_queue_del(queue, timer);

      /* Mark the hrtimer in the running state. In this case,
       * User can not reclaim (deallocate or modify) the hrtimer.
       */

      hrtimer_queue_mark_running(queue, timer, cpu);
      hrtimer_queue_unlock(&queue->lock, flags);

      /* Execute the hrtimer function. Here we can not modify the hrtimer
       * anymore because the hrtimer might be modified by other users.
       * The expired time is passing as the callback version to help users
       * to avoid race conditions.
       */

      next_expired = expired + func(arg, expired);
      DEBUGASSERT(HRTIMER_TIME_AFTER_EQ(next_expired, expired));

      flags = hrtimer_queue_lock(&queue->lock);

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

      hrtimer_queue_mark_running(&queue->lock, NULL, cpu);
    }

  next_expired = queue->next_expired;
  hrtimer_reprogram(queue, next_expired);

  hrtimer_queue_unlock(&queue->lock, flags);

  return next_expired;
}

/****************************************************************************
 * Name: hrtimer_queue_start
 *
 * Description:
 *   Start the hrtimer asynchronously.
 *
 * Input Parameters:
 *   queue   - The hrtimer queue.
 *   timer   - The hrtimer to be set.
 *   reprogram - The timer reprogram function.
 *
 * Returned Value:
 *   None.
 *
 * Assumption:
 *   The timer is not in the queue.
 *
 ****************************************************************************/

static inline_function
void hrtimer_queue_start(FAR hrtimer_queue_internal_t *queue,
                         FAR hrtimer_internal_t *timer)
{
  irqstate_t flags;

  DEBUGASSERT(queue && timer && timer->func);

  flags = hrtimer_queue_lock(&queue->lock);

  /* The queue should not have the timer ownership before. */

  DEBUGASSERT(hrtimer_queue_count_ownership(queue, timer) == 0u);

  /* Reprogram the timer when the queue head has changed. */

  if (hrtimer_queue_add(queue, timer))
    {
      hrtimer_reprogram(queue, queue->next_expired);
    }

  hrtimer_queue_unlock(&queue->lock, flags);
}

/****************************************************************************
 * Name: hrtimer_queue_async_cancel
 *
 * Description:
 *   Dequeue and set the timer to the cancelled state asynchronously.
 *   The write ownership of the timer is removed from the queue expiration.
 *   However, the callback function may still be executing on another CPU.
 *
 * Input Parameters:
 *   queue - The hrtimer queue.
 *   timer - The hrtimer to be canceled.
 *   reprogram - The timer reprogram function.
 *
 * Returned Value:
 *   The count of the timer references. Zero means the timer is not
 *   referenced by any core. Generally, only one reference to a timer
 *   can exist at the same time. However, when a timer may be restarted
 *   at the pending state (i.e., by calling hrtimer_restart), more references
 *   to the timer may exist.
 *   -EINVAL on if the timer is not in the pending state.
 *
 ****************************************************************************/

static inline_function
int hrtimer_queue_async_cancel(FAR hrtimer_queue_internal_t *queue,
                               FAR hrtimer_internal_t *timer)
{
  uint64_t        next_expired;
  irqstate_t             flags;
  int                      cpu;
  bool                reassess = false;
  uintptr_t         timer_uptr = (uintptr_t)timer;
  int                      ret = 0;

  DEBUGASSERT(queue && timer);

  flags = hrtimer_queue_lock(&queue->lock);

#ifdef CONFIG_SMP
  DEBUGASSERT((timer_uptr & 0x1u) == 0u);

  /* Check if the timer is referenced by any CPU core.
   * Generally, only one reference to a timer can exist at the same time.
   * However, when a timer may be restarted at the cancelled state (i.e.,
   * by calling hrtimer_restart), more references to the timer may exist.
   */

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      /* This is a faster implementation equivalent to
       * (queue->running[cpu] & (~(uintptr_t)0x1u)) == timer,
       * Assuming the pointer of the timer is at least 2-bytes aligned
       * (the last bit must be zero).
       */

      if ((queue->running[cpu] ^ timer_uptr) <= 0x1u)
        {
          /* Set the timer to the cancelled state and revoke the write
           * ownership of the timer from the queue.
           */

          queue->running[cpu] = timer_uptr | 0x1u;
          ret++;
        }
    }
#endif

  if (HRTIMER_ISPENDING(timer))
    {
      /* Clear the pending state. */

      timer->func = NULL;

      /* Reprogram the timer when the queue head has changed. */

      if (hrtimer_queue_del(queue, timer))
        {
          hrtimer_reprogram(queue, queue->next_expired);
        }
    }

  hrtimer_queue_unlock(&queue->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: hrtimer_queue_wait
 *
 * Description:
 *   Wait for all references to be released before the timer can be freed.
 *
 * Input Parameters:
 *   queue - The hrtimer queue.
 *   timer - The hrtimer to be waited.
 *
 * Returned Value:
 *   None.
 *
 * Assumption:
 *   The periodical hrtimer should be cancelled by hrtimer_queue_asyn_cancel
 *   before calling this function.
 *
 ****************************************************************************/

static inline_function
void hrtimer_queue_wait(FAR hrtimer_queue_internal_t *queue,
                        FAR hrtimer_internal_t *timer)
{
  DEBUGASSERT(queue && timer && !HRTIMER_ISPENDING(timer));

#ifdef CONFIG_SMP
  int cpu;

  /* Wait until all references have been released. */

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      uintptr_t cancelled_uptr = (uintptr_t)timer | 0x1u;

      /* The timer should not be restarted again.
       * Or there will be a ownership invariant violation.
       */

      DEBUGASSERT(!hrtimer_queue_is_running(queue, timer, cpu));

      while (hrtimer_queue_is_running(queue, cancelled_uptr, cpu))
        {
          /* Customed wait policy. */

          hrtimer_wait_policy();
        }
    }
#endif

  /* Finally, we acquire the ownership of this hrtimer. */
}

/****************************************************************************
 * Name: hrtimer_queue_gettime
 *
 * Description:
 *   Get the rest of the delay time of the hrtimer.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *   timer - The hrtimer to be queried.
 *   current - The current time.
 *
 * Returned Value:
 *   The time until next expiration.
 *
 ****************************************************************************/

static inline_function
uint64_t hrtimer_queue_gettime(FAR hrtimer_queue_internal_t *queue,
                               FAR hrtimer_internal_t *timer,
                               uint64_t current)
{
  uint64_t expired = hrtimer_queue_read_64(timer, &timer->expired);
  int64_t  remain  = expired - current;

  return remain < 0 ? 0u : remain;
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_HRTIMER_QUEUE_H */
