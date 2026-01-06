/****************************************************************************
 * sched/hrtimer/hrtimer_internal.h
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

#ifndef __INCLUDE_SCHED_HRTIMER_HRTIMER_INTERNAL_H
#define __INCLUDE_SCHED_HRTIMER_HRTIMER_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/hrtimer.h>
#include <nuttx/seqlock.h>

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef CONFIG_HRTIMER_LIST
/* Compare function for the rb-tree. */

static inline_function
int hrtimer_compare(FAR const hrtimer_t *a, FAR const hrtimer_t *b)
{
  /* This branchless compare is equivalent to:
   * (int64_t)(a->expired - b->expired) >= 0 ? 1 : -1;
   */

  return 1 - (HRTIMER_TIME_BEFORE(a->expired, b->expired) << 1u);
}

/* Generate red-black tree implementation for high-resolution timers
 * Do not inline the RB-Tree related functions to reduce code.
 */

RB_GENERATE_INTERNAL(hrtimer_tree_s, hrtimer_s, node, hrtimer_compare,
                     static inline)
#endif

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
FAR hrtimer_t *hrtimer_queue_peek(FAR hrtimer_queue_t *queue)
{
#ifdef CONFIG_HRTIMER_LIST
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
bool hrtimer_queue_add(FAR hrtimer_queue_t *queue, FAR hrtimer_t *timer)
{
  FAR hrtimer_t *curr;
  bool           is_head;

#ifdef CONFIG_HRTIMER_LIST
  list_for_every_entry(&queue->queue, curr, hrtimer_t, node)
    {
      /* Until curr->expired has not timed out. */

      if (HRTIMER_TIME_AFTER(curr->expired, timer->expired))
        {
          break;
        }
    }

  list_add_before(&curr->node, &timer->node);
#else
  curr = RB_INSERT(hrtimer_tree_s, &queue->root, timer);
  DEBUGASSERT(curr == NULL);
#endif

  is_head = HRTIMER_TIME_AFTER(queue->next_expired, timer->expired);

  if (is_head)
    {
#ifndef CONFIG_HRTIMER_LIST
      queue->first        = timer;
#endif
      queue->next_expired = timer->expired;
    }

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
 *   true if the timer is the head of the queue, otherwise false.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *   The caller must ensure that the timer is in the queue.
 *
 ****************************************************************************/

static inline_function
bool hrtimer_queue_del(FAR hrtimer_queue_t *queue,
                       FAR hrtimer_t *timer)
{
  bool is_head;
#ifdef CONFIG_HRTIMER_LIST
  FAR hrtimer_t *head = list_first_entry(&queue->queue, hrtimer_t, node);
  list_delete(&timer->node);

  is_head = head == timer;
#else
  RB_REMOVE(hrtimer_tree_s, &queue->root, timer);

  is_head = queue->first == timer;
#endif

  /* Update the first node. */

  if (is_head)
    {
#ifndef CONFIG_HRTIMER_LIST
      queue->first = RB_MIN(hrtimer_tree_s, &queue->root);
#endif
      queue->next_expired = hrtimer_queue_peek(queue)->expired;
    }

  return is_head;
}

/****************************************************************************
 * Name: hrtimer_queue_clear
 *
 * Description:
 *   Clear the hrtimer queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *
 * Returned Value:
 *   None.
 *
 * Assumption:
 *   The caller must hold the queue lock or ownership of the queue.
 *
 ****************************************************************************/

static inline_function
void hrtimer_queue_clear(FAR hrtimer_queue_t *queue)
{
#ifdef CONFIG_HRTIMER_LIST
  list_initialize(&queue->queue);
#else
  RB_INIT(&queue->root);
#endif
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
static inline_function
uint64_t hrtimer_queue_read_64(FAR hrtimer_queue_t *queue,
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
#endif

#if UINT_MAX >= UINT32_MAX
/* On 32/64-bit architectures, read/write uint32_t is atomic. */
#  define hrtimer_queue_read_32(queue, ptr) (*(FAR volatile uint32_t *)(ptr))
#else
static inline_function
uint32_t hrtimer_queue_read_32(FAR hrtimer_queue_t *queue,
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
 * Assumption:
 *   The caller must hold the queue lock.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static inline_function
void hrtimer_queue_mark_running(FAR hrtimer_queue_t *queue,
                                FAR hrtimer_t *timer, int cpu)
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
  (hrtimer_queue_read(queue, &(queue)->running[cpu]) == (uintptr_t)(timer))
#else
#  define hrtimer_queue_has_ownership(queue, timer, cpu) (true)
#endif

/****************************************************************************
 * Name: hrtimer_queue_revoke_ownership
 *
 * Description:
 *   Revoke the ownership of the cancelled timer and return the references
 *   count to the timer.
 *
 * Input Parameters:
 *   queue     - The timer queue.
 *   cancelled - The cancelled timer upt
 *
 * Returned Value:
 *   The references count to the timer.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static inline_function
int hrtimer_queue_revoke_ownership(FAR hrtimer_queue_t *queue,
                                   uintptr_t cancelled)
{
  int cpu;
  int refs = 0;

  /* Check if the timer is referenced by any CPU core.
   * Generally, only one reference to a timer can exist at the same time.
   * However, when a timer may be restarted at the cancelled state (i.e.,
   * by calling hrtimer_restart), more references to the timer may exist.
   */

  unroll_loop(CONFIG_SMP_NCPUS) /* Tell the compiler to unroll the loop. */

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      /* This is a faster implementation equivalent to
       * (queue->running[cpu] & (~(uintptr_t)0x1u)) == timer,
       * Assuming the pointer of the timer is at least 2-bytes aligned
       * (the last bit must be zero).
       */

      if ((queue->running[cpu] ^ cancelled) <= 0x1u)
        {
          /* Set the timer to the cancelled state and revoke the write
           * ownership of the timer from the queue.
           */

          queue->running[cpu] = cancelled;
          refs++;
        }
    }

  return refs;
}

/* Helper function for debugging.
 * If the ownership count > 0, it indicates that users might forget to call
 * hrtimer_cancel to seize the timer ownership before restart the timer.
 */

static inline_function
int hrtimer_queue_count_ownership(FAR hrtimer_queue_t *queue,
                                  uintptr_t timer)
{
  int cpu;
  int owners = 0;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      if (hrtimer_queue_has_ownership(queue, timer, cpu))
        {
          owners++;
        }
    }

  return owners;
}
#else
#  define hrtimer_queue_revoke_ownership(queue, cancelled) (0)
#  define hrtimer_queue_count_ownership(queue, timer)      (0)
#endif

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

#ifdef CONFIG_SMP
static inline_function
void hrtimer_queue_wait(FAR hrtimer_queue_t *queue, FAR hrtimer_t *timer)
{
  int             cpu;
  uintptr_t cancelled = (uintptr_t)timer | 0x1u;

  DEBUGASSERT(queue && timer && !HRTIMER_ISPENDING(timer));

  /* Wait until all references have been released. */

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      /* The timer should not be restarted again.
       * Or there will be ownership invariant violation.
       */

      DEBUGASSERT(hrtimer_queue_read(queue, &queue->running[cpu]) !=
                  (uintptr_t)timer);

      while (hrtimer_queue_read(queue, &queue->running[cpu]) == cancelled)
        {
          /* Spin-wait is enough */
        }
    }

  /* Finally, we acquire the ownership of this hrtimer. */
}
#else
#  define hrtimer_queue_wait(queue, timer) /* Nothing to wait in non-SMP. */
#endif

#endif  /* __INCLUDE_SCHED_HRTIMER_HRTIMER_INTERNAL_H */
