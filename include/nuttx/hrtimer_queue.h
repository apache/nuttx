/****************************************************************************
 * include/nuttx/hrtimer_queue.h
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This header file should be only included for internal use,
 * DO NOT EXPOSE IT TO USERS.
 *
 * Before including this file, Please provide the following inputs:
 * Bool macro: HRTIMER_QUEUE_USE_LIST.
 *   - Tell the library to use list implementation or red-black tree
 *     implementation.
 *
 * Function implementation: static inline_function
 * void hrtimer_reprogram(FAR USER_HRTIMER_QUEUE_TYPE *queue,
 *                        uint64_t next_expired);
 *   - Reprogram the timer hardware to the next expired time.
 */

#ifdef HRTIMER_QUEUE_USE_LIST
typedef struct hrtimer_list_s hrtimer_internal_t;
typedef struct hrtimer_list_queue_s hrtimer_queue_internal_t;
#  define GENERATE_QUEUE_OPS()
#else
typedef struct hrtimer_rb_s hrtimer_internal_t;
typedef struct hrtimer_rb_queue_s hrtimer_queue_internal_t;

/* Compare function for the rb-tree. */

static inline_function
int hrtimer_compare(FAR const hrtimer_rb_t *a, FAR const hrtimer_rb_t *b)
{
  /* This branchless compare is equivalent to:
   * (int64_t)(a->expired - b->expired) > 0 ? 1 : -1;
   */

  return 1 - (HRTIMER_TIME_BEFORE_EQ(a->expired, b->expired) << 1u);
}

/* Generate red-black tree implementation for high-resolution timers */

RB_PROTOTYPE_STATIC(hrtimer_tree_s, hrtimer_rb_s, node, hrtimer_compare)
#define GENERATE_QUEUE_OPS() RB_GENERATE_STATIC(hrtimer_tree_s, hrtimer_rb_s, node, hrtimer_compare)
#endif

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

/* The relied function hrtimer_reprogram must be implemented. */

static inline_function
void hrtimer_reprogram(FAR hrtimer_queue_internal_t *queue,
                       uint64_t next_expired);

/* Reusable library code for user-defined high-resolution timer queue. */

/****************************************************************************
 * Name: hrtimer_in_queue
 *
 * Description:
 *   Whether the hrtimer is in the queue.
 *
 * Input Parameters:
 *   timer - The timer to be checked.
 *
 * Returned Value:
 *   true if the timer is in the queue, otherwise false.
 *
 ****************************************************************************/

#define hrtimer_in_queue(timer) \
  ((uintptr_t)(timer)->func != UINTPTR_MAX)

#define hrtimer_mark_dequeued(timer) \
  ((timer)->func = (hrtimer_callback_t)UINTPTR_MAX)

/****************************************************************************
 * Name: hrtimer_queue_peek
 *
 * Description:
 *   Get the head hrtimer in the queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *
 * Returned Value:
 *   The head timer in the queue.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *
 ****************************************************************************/

static inline_function
FAR hrtimer_internal_t *hrtimer_queue_peek(
                                         FAR hrtimer_queue_internal_t *queue)
{
#ifdef HRTIMER_QUEUE_USE_LIST
  return list_first_entry(&queue->queue, hrtimer_t, node);
#else
  return queue->first;
#endif
}

/****************************************************************************
 * Name: hrtimer_queue_add
 *
 * Description:
 *   Add the hrtimer to the queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *   timer - The timer to be added.
 *
 * Returned Value:
 *   true if the timer is added to the head of the queue, otherwise false.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *   The caller must ensure that the timer is not in the queue.
 *
 ****************************************************************************/

static inline_function
bool hrtimer_queue_add(FAR hrtimer_queue_internal_t *queue,
                       FAR hrtimer_internal_t *timer)
{
  FAR hrtimer_internal_t *curr;
  bool           is_head = false;
  uint64_t       expired = timer->expired;

#ifdef HRTIMER_QUEUE_USE_LIST
  list_for_every_entry(&queue->queue, curr, hrtimer_list_t, node)
    {
      /* Until curr->expired has not timed out. */

      if (HRTIMER_TIME_AFTER(curr->expired, expired))
        {
          break;
        }
    }

  list_add_before(&curr->node, &timer->node);
#else
  curr = RB_INSERT(hrtimer_tree_s, &queue->root, timer);
  DEBUGASSERT(curr == NULL);
#endif

  if (HRTIMER_TIME_AFTER(queue->next_expired, expired))
    {
#ifndef HRTIMER_QUEUE_USE_LIST
      /* Update the cached first node. */

      queue->first = timer;
#endif
      queue->next_expired = expired;
      is_head = true;
    }

  DEBUGASSERT(hrtimer_in_queue(timer));

  return is_head;
}

/****************************************************************************
 * Name: hrtimer_queue_del
 *
 * Description:
 *   Delete the hrtimer from the queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *   timer - The timer to be deleted.
 *
 * Returned Value:
 *   true if the timer is deleted from the head of the
 *   queue, otherwise false.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *   The caller must ensure that the timer is in the queue.
 *
 ****************************************************************************/

static inline_function
bool hrtimer_queue_del(FAR hrtimer_queue_internal_t *queue,
                       FAR hrtimer_internal_t *timer)
{
  bool is_head = false;

  DEBUGASSERT(hrtimer_in_queue(timer));

  /* Indicate the timer is not in the queue anymore. */

  hrtimer_mark_dequeued(timer);

#ifdef HRTIMER_QUEUE_USE_LIST
  is_head = list_first_entry(&queue->queue, hrtimer_list_t, node) == timer;
  list_delete(&timer->node);

  /* Update the next_expired if the queue head changes. */

  if (is_head)
    {
      queue->next_expired = hrtimer_queue_peek(queue)->expired;
    }
#else
  RB_REMOVE(hrtimer_tree_s, &queue->root, timer);

  /* Update the first node. */

  if (timer == queue->first)
    {
      queue->first = RB_MIN(hrtimer_tree_s, &queue->root);
      queue->next_expired = queue->first->expired;
      is_head = true;
    }
#endif

  return is_head;
}

/****************************************************************************
 * Name: hrtimer_queue_init
 *
 * Description:
 *   Initialize the hrtimer queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *   guard_timer - The guard timer.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function
void hrtimer_queue_init(FAR hrtimer_queue_internal_t *queue)
{
  FAR hrtimer_internal_t *guard_timer = &queue->guard_timer;
  int cpu;

  DEBUGASSERT(queue);

  /* The guard timer is designed to ensure the system will enter the safe
   * state if the timing system is not working properly.
   * It can be customized by the user after the hrtimer_queue_init and
   * before the first user hrtimer is added.
   * If the guard timer fired, it means 292-years have passed, which is
   * impossible. In this case we should trigger the kernel panic and reboot
   * the system. Here we assume the system will reboot after jumping to NULL.
   */

  hrtimer_fill(guard_timer, NULL, NULL, INT64_MAX);

  seqlock_init(&queue->lock);

#ifdef HRTIMER_QUEUE_USE_LIST
  list_initialize(&queue->queue);
#else
  RB_INIT(&queue->root);
#endif

  queue->next_expired = UINT64_MAX;
  hrtimer_queue_add(queue, guard_timer);

  memset(queue->running, 0, sizeof(queue->running));
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

static inline_function
uint64_t hrtimer_queue_read_64(FAR hrtimer_queue_internal_t *queue,
                               FAR const uint64_t *ptr)
{
  uint64_t val;

  DEBUGASSERT(queue && ptr);

#ifdef CONFIG_ARCH_64BIT
  /* On 64-bit architectures, read/write uint64_t is atomic. */

  val = *((FAR volatile uint64_t *)ptr);
#else
  uint32_t seq;

  do
    {
      seq = read_seqbegin(&queue->lock);
      val = *ptr;
    }
  while (read_seqretry(&queue->lock, seq));
#endif

  return val;
}

static inline_function
uint32_t hrtimer_queue_read_32(FAR hrtimer_queue_internal_t *queue,
                               FAR const uint32_t *ptr)
{
  uint32_t val;

  DEBUGASSERT(queue && ptr);

#if (UINT_MAX >= UINT32_MAX)
  /* On 32/64-bit architectures, read/write uint32_t is atomic. */

  val = *((FAR volatile uint32_t *)ptr);
#else
  uint32_t seq;

  do
    {
      seq = read_seqbegin(&queue->lock);
      val = *ptr;
    }
  while (read_seqretry(&queue->lock, seq));
#endif

  return val;
}

/* Generic function to read the value in the queue atomically. */

#define hrtimer_queue_read(queue, ptr) \
  (sizeof(*(ptr)) == 8u ? \
   hrtimer_queue_read_64(queue, (FAR const uint64_t *)(ptr)) : \
   (sizeof(*(ptr)) == 4u ? \
    hrtimer_queue_read_32(queue, (FAR const uint32_t *)(ptr)) : 0u))

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
  irqstate_t              flags = write_seqlock_irqsave(&queue->lock);

  /* Check if we need processing the expiration. */

  while (HRTIMER_TIME_BEFORE_EQ(queue->next_expired, current))
    {
      timer     = hrtimer_queue_peek(queue);

      /* Remove the hrtimer from the head of the queue
       * and update the next_expired.
       */

      func    = timer->func;
      arg     = timer->arg;
      expired = timer->expired;

      hrtimer_queue_del(queue, timer);

      /* Mark the hrtimer via the hazard potiner.
       * In this case, User can not reclaim (deallocate or modify) the
       * hrtimer.
       */

      queue->running[cpu] = (uintptr_t)timer;

      write_sequnlock_irqrestore(&queue->lock, flags);

      /* Execute the hrtimer function. */

      next_expired = expired + func(arg, expired);

      flags = write_seqlock_irqsave(&queue->lock);

      /* In the interrupt context, the cpu core id should not change. */

      DEBUGASSERT(cpu == this_cpu());

      /* If the hazard-pointer has not changed, we can re-enqueue the
       * timer or release the ownership. Otherwise, the hrtimer should
       * have been cancelled. In this case, we lose the ownership of the
       * hrtimer and can not modify this hrtimer anymore.
       */

      if (queue->running[cpu] == timer)
        {
          if (next_expired != expired)
            {
              /* Re-enqueue the hrtimer if it is periodic. */

              timer->func    = func;
              timer->expired = next_expired;
              hrtimer_queue_add(queue, timer);
            }
          else
            {
              /* Clear the pending state to release the hrtimer ownership. */

              timer->func = NULL;
            }
        }

      /* Clear the hazard pointer. */

      queue->running[cpu] = 0u;
    }

  hrtimer_reprogram(queue, queue->next_expired);
  next_expired = queue->next_expired;

  write_sequnlock_irqrestore(&queue->lock, flags);

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

/* Helper function for debugging usage.
 * If the ownership count > 0, it indicates that users might forget to call
 * async_cancel to seize the timer ownership before restart the timer.
 */

unused_code static inline_function
unsigned hrtimer_queue_count_ownership(FAR hrtimer_queue_internal_t *queue,
                                       FAR hrtimer_internal_t *timer)
{
  int       cpu;
  unsigned  owner     = 0u;
  uintptr_t time_uptr = (uintptr_t)timer;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      if (queue->running[cpu] == time_uptr)
        {
          owner++;
        }
    }

  return owner;
}

static inline_function
void hrtimer_queue_start(FAR hrtimer_queue_internal_t *queue,
                         FAR hrtimer_internal_t *timer)
{
  irqstate_t flags;

  DEBUGASSERT(queue && timer && timer->func);

  flags = write_seqlock_irqsave(&queue->lock);

  /* The queue should not have the timer ownership before. */

  DEBUGASSERT(hrtimer_queue_count_ownership(queue, timer) == 0u);

  /* Reprogram the timer when the queue head has changed. */

  if (hrtimer_queue_add(queue, timer))
    {
      hrtimer_reprogram(queue, queue->next_expired);
    }

  write_sequnlock_irqrestore(&queue->lock, flags);
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
  uint64_t   next_expired;
  irqstate_t        flags;
  int                 cpu;
  uintptr_t    timer_uptr = (uintptr_t)timer;
  int                 ret = -EINVAL;
  bool          reprogram = false;

  DEBUGASSERT(queue && timer);

  flags = write_seqlock_irqsave(&queue->lock);

  if (HRTIMER_ISPENDING(timer))
    {
      if (hrtimer_in_queue(timer))
        {
          /* Reprogram the timer when the queue head has changed. */

          reprogram = hrtimer_queue_del(queue, timer);
        }

      ret = 0;

      DEBUGASSERT((timer_uptr & 0x1u) == 0u);

      /* Check if the timer is referenced by any core.
       * Generally, only one reference to a timer can exist at the same time.
       * However, when a timer may be restarted at the pending state (i.e.,
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
              queue->running[cpu] = timer_uptr | 0x1u;
              ret++;
            }
        }
    }

  /* Reprograming the timer if needed. */

  if (reprogram)
    {
      hrtimer_reprogram(queue, queue->next_expired);
    }

  write_sequnlock_irqrestore(&queue->lock, flags);

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
  int              cpu;
  uintptr_t timer_uptr = (uintptr_t)timer | 0x1u;

  DEBUGASSERT(queue && timer);

  /* Wait until all references have been released. */

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      DEBUGASSERT(hrtimer_queue_read(queue, &queue->running[cpu]) !=
                  (uintptr_t)timer);

      while (hrtimer_queue_read(queue, &queue->running[cpu]) == timer_uptr)
        {
          /* CPU Relax */
        }
    }

  /* Finally, we acquire the ownership of this hrtimer. */

  timer->func = NULL;
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
  uint64_t expired = hrtimer_queue_read(timer, &timer->expired);
  int64_t  remain  = expired - current;

  return remain < 0 ? 0u : remain;
}

/****************************************************************************
 * Public Template Macros
 ****************************************************************************/

/* Generate the hrtimer queue implementation. */

#define HRTIMER_QUEUE_GENERATE(name, queue, current) \
GENERATE_QUEUE_OPS() /* Generate queue operations if needed */ \
int name##_restart_absolute(FAR hrtimer_internal_t *timer, hrtimer_callback_t func, \
                            FAR void *arg, uint64_t expected) \
{ \
  hrtimer_fill(timer, func, arg, expected); \
  hrtimer_queue_start(queue, timer); \
  return OK; \
} \
 \
int name##_async_cancel(FAR hrtimer_internal_t *timer) \
{ \
  return hrtimer_queue_async_cancel(queue, timer); \
} \
 \
int name##_cancel(FAR hrtimer_internal_t *timer) \
{ \
  int ret = name##_async_cancel(timer); \
 \
  if (ret == 0) \
    { \
      /* Nothing to wait, reclaim the ownership directly. */ \
 \
      timer->func = NULL; \
    } \
  else if (ret > 0) \
    { \
      /* Wait the timer to finish and reclaim the ownership. */ \
 \
      hrtimer_queue_wait(queue, timer); \
      ret = OK; \
    } \
 \
  return ret; \
} \
 \
uint64_t name##_gettime(FAR hrtimer_internal_t *timer) \
{ \
  return hrtimer_queue_gettime(queue, timer, current()); \
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_HRTIMER_QUEUE_H */
