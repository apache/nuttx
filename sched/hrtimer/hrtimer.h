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
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __SCHED_HRTIMER_HRTIMER_H
#define __SCHED_HRTIMER_HRTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/hrtimer.h>
#include <nuttx/seqlock.h>

#include "sched/sched.h"

#ifdef CONFIG_HRTIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Delay used while waiting for a running hrtimer callback to complete */

#define HRTIMER_CANCEL_SYNC_DELAY_US CONFIG_USEC_PER_TICK

/* The pending state indicates the timer belongs to the shared hrtimer queue
 * and is waiting for the next hrtimer expiry.
 */

#define hrtimer_is_pending(hrtimer)    ((hrtimer)->func != NULL)

/* Compare functions for the expired time. Since some compilers may not
 * optimize the compare.
 */

#define HRTIMER_TIME_BEFORE(t1, t2)    ((int64_t)((t2) - (t1)) > 0)
#define HRTIMER_TIME_BEFORE_EQ(t1, t2) ((int64_t)((t2) - (t1)) >= 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Red-black tree head for managing active high-resolution timers.
 *
 * Timers are ordered by expiration time, the earliest expiring timer
 * being the left-most (minimum) node in the tree.
 */

#ifdef CONFIG_HRTIMER_TREE
RB_HEAD(hrtimer_tree_s, hrtimer_s);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Seqcount protecting access to the hrtimer queue and timer state */

extern seqcount_t g_hrtimer_lock;

#ifdef CONFIG_HRTIMER_TREE
/* Red-Black tree containing all active high-resolution timers */

extern struct hrtimer_tree_s g_hrtimer_tree;
extern struct hrtimer_s      g_hrtimer_guard;
extern struct FAR hrtimer_s *g_hrtimer_head;
#else
/* List containing all active high-resolution timers */

extern struct list_node g_hrtimer_list;
#endif

/* Array of pointers to currently running high-resolution timers
 * for each CPU in SMP configurations. Index corresponds to CPU ID.
 */

#ifdef CONFIG_SMP
extern uintptr_t g_hrtimer_running[CONFIG_SMP_NCPUS];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_process
 *
 * Description:
 *   Called from the timer interrupt handler to process expired
 *   high-resolution timers. If a timer has expired, its callback
 *   function will be executed in the context of the timer interrupt.
 *
 * Input Parameters:
 *   now - The current time (nsecs).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void hrtimer_process(uint64_t now);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_reprogram
 *
 * Description:
 *   Reprogram the hardware timer to expire at a specified nanosecond time.
 *   Converts the nanosecond time to timespec and calls the platform-specific
 *   timer start function.
 *
 * Input Parameters:
 *   ns - Expiration time in nanoseconds.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   The underlying timer start function returns 0 on success.
 *
 ****************************************************************************/

static inline_function void hrtimer_reprogram(uint64_t next_expired)
{
  /* `hrtimer_reprogram` relies on the underlying timer being a non-periodic
   * timer. If the underlying timer hardware is a periodic timer like
   * systick, we cannot set the next expiration time.
   */

#ifdef CONFIG_SCHED_TICKLESS
  int ret = 0;
  struct timespec ts;

  clock_nsec2time(&ts, next_expired);

#  ifdef CONFIG_ALARM_ARCH
  ret = up_alarm_start(&ts);
#  else
  struct timespec current;
  up_timer_gettime(&current);
  clock_timespec_subtract(&ts, &current, &ts);
  ret = up_timer_start(&ts);
#  endif

  DEBUGASSERT(ret == 0);
#endif
}

/****************************************************************************
 * Name: hrtimer_compare
 *
 * Description:
 *   Compare two high-resolution timer nodes to determine their ordering
 *   in the hrtimer queue. Used internally by the RB-tree macros.
 *
 *   Note that RB-tree can not guarantee that timers with the same expired
 *   time will be processed in a FIFO order. However, this violation
 *   of the sorted queue invariant is acceptable and can not affect the
 *   functional correctness for the hrtimer.
 *
 * Input Parameters:
 *   a - Pointer to the first hrtimer node.
 *   b - Pointer to the second hrtimer node.
 *
 * Returned Value:
 *   <0 if a expires before b.
 *   >=0 if a expires after b.
 *
 ****************************************************************************/

#ifdef CONFIG_HRTIMER_TREE
static inline_function
int hrtimer_compare(FAR const hrtimer_t *a, FAR const hrtimer_t *b)
{
  /* This branchless compare is equivalent to:
   * (int64_t)(a->expired - b->expired) >= 0 ? 1 : -1;
   */

  return 1 - (HRTIMER_TIME_BEFORE(a->expired, b->expired) << 1u);
}
#endif

/****************************************************************************
 * Red-Black Tree Prototype for high-resolution timers
 *
 * Description:
 *   Declare the RB-tree prototype that manages all active high-resolution
 *   timers. This tree provides efficient insertion, removal, and lookup
 *   operations based on timer expiration time.
 ****************************************************************************/

#ifdef CONFIG_HRTIMER_TREE
RB_PROTOTYPE(hrtimer_tree_s, hrtimer_s, node, hrtimer_compare);
#endif

/****************************************************************************
 * Name: hrtimer_remove
 *
 * Description:
 *   Remove a timer from the queue and mark it as dequeued.
 *
 * Returned Value:
 *   true if the timer is the head of the queue, otherwise false.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *   The caller must ensure that the timer is in the queue.
 *
 ****************************************************************************/

static inline_function bool hrtimer_remove(FAR hrtimer_t *hrtimer)
{
  bool is_head;
#ifdef CONFIG_HRTIMER_TREE
  is_head = g_hrtimer_head == hrtimer;

  RB_REMOVE(hrtimer_tree_s, &g_hrtimer_tree, hrtimer);

  if (is_head)
    {
      g_hrtimer_head = RB_MIN(hrtimer_tree_s, &g_hrtimer_tree);
    }
#else
  is_head = list_is_head(&g_hrtimer_list, &hrtimer->node);
  list_delete_fast(&hrtimer->node);
#endif

  /* Explicitly mark the timer as dequeued. */

  hrtimer->func = NULL;

  return is_head;
}

/****************************************************************************
 * Name: hrtimer_insert
 *
 * Description:
 *   Insert a timer into the hrtimer queue according to its expiration time.
 *
 * Returned Value:
 *   true if the timer is added to the head of the queue, otherwise false.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *   The caller must ensure that the timer is in the queue.
 *
 ****************************************************************************/

static inline_function bool hrtimer_insert(FAR hrtimer_t *hrtimer)
{
#ifdef CONFIG_HRTIMER_TREE
  bool is_head = false;
  RB_INSERT(hrtimer_tree_s, &g_hrtimer_tree, hrtimer);

  if (HRTIMER_TIME_BEFORE(hrtimer->expired, g_hrtimer_head->expired))
    {
      g_hrtimer_head = hrtimer;
      is_head        = true;
    }

  return is_head;
#else
  FAR hrtimer_t *curr;
  uint64_t expired = hrtimer->expired;

  list_for_every_entry(&g_hrtimer_list, curr, hrtimer_t, node)
    {
      /* Until curr->expired has not timed out relative to expired */

      if (!clock_compare(curr->expired, expired))
        {
          break;
        }
    }

  list_add_before(&curr->node, &hrtimer->node);
  return list_is_head(&g_hrtimer_list, &hrtimer->node);
#endif
}

/****************************************************************************
 * Name: hrtimer_get_first
 *
 * Description:
 *   Return the earliest expiring pending timer.
 *
 * Returned Value:
 *   Pointer to the earliest timer, or NULL if none are pending.
 *
 ****************************************************************************/

static inline_function FAR hrtimer_t *hrtimer_get_first(void)
{
#ifdef CONFIG_HRTIMER_TREE
  return g_hrtimer_head;
#else
  return list_first_entry(&g_hrtimer_list, FAR hrtimer_t, node);
#endif
}

/****************************************************************************
 * Name: hrtimer_read_64/32
 *
 * Description:
 *   Internal function to read the value in the queue atomically.
 *   Do not use this function if you are not sure about the thread-safe
 *   of the value you are reading.
 *
 * Input Parameters:
 *   ptr - The pointer to be read.
 *
 * Returned Value:
 *   The value in the queue.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_64BIT
/* On 64-bit architectures, read/write uint64_t is atomic. */
#  define hrtimer_read_64(ptr) (*(FAR volatile uint64_t *)(ptr))
#else
static inline_function
uint64_t hrtimer_read_64(FAR const uint64_t *ptr)
{
  uint64_t val;
  uint32_t seq;

  do
    {
      seq = read_seqbegin(&g_hrtimer_lock);
      val = *ptr;
    }
  while (read_seqretry(&g_hrtimer_lock, seq));

  return val;
}
#endif

#if UINT_MAX >= UINT32_MAX
/* On 32/64-bit architectures, read/write uint32_t is atomic. */
#  define hrtimer_read_32(ptr) (*(FAR volatile uint32_t *)(ptr))
#else
static inline_function
uint32_t hrtimer_read_32(FAR const uint32_t *ptr)
{
  uint32_t val;
  uint32_t seq;

  do
    {
      seq = read_seqbegin(&g_hrtimer_lock);
      val = *ptr;
    }
  while (read_seqretry(&g_hrtimer_lock, seq));

  return val;
}
#endif

/* Generic function to read the value in the queue atomically. */

#define hrtimer_read(ptr) \
  (sizeof(*(ptr)) == 8u ? \
   hrtimer_read_64((FAR const uint64_t *)(ptr)) : \
   (sizeof(*(ptr)) == 4u ? \
    hrtimer_read_32((FAR const uint32_t *)(ptr)) : 0u))

/****************************************************************************
 * Name: hrtimer_mark_running
 *
 * Description:
 *   Mark the timer as running.
 *
 * Input Parameters:
 *   timer - The timer to be marked.
 *   cpu   - The CPU core Id.
 *
 * Returned Value:
 *   None.
 *
 * Assumption:
 *   The caller must hold the lock.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
#  define hrtimer_mark_running(timer, cpu) \
  (g_hrtimer_running[cpu] = (uintptr_t)(timer))
#else
#  define hrtimer_mark_running(timer, cpu) UNUSED(cpu)
#endif
#define hrtimer_unmark_running(cpu) hrtimer_mark_running(NULL, cpu)

/****************************************************************************
 * Name: hrtimer_is_running
 *
 * Description:
 *   Check if the CPU core is running the timer.
 *
 * Input Parameters:
 *   timer - The timer to be marked.
 *   cpu   - The CPU core Id.
 *
 * Returned Value:
 *   true if the CPU core is running the timer, false otherwise.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
#  define hrtimer_is_running(timer, cpu) \
  (hrtimer_read(&g_hrtimer_running[cpu]) == (uintptr_t)(timer))
#else
#  define hrtimer_is_running(timer, cpu) (true)
#endif
#define hrtimer_is_cancelling(timer, cpu) \
  hrtimer_is_running((uintptr_t)(timer) | 0x1u, cpu)

/****************************************************************************
 * Name: hrtimer_cancel_running
 *
 * Description:
 *   Cancel the timer, revoke the ownership of the cancelled timer, and
 *   return the references count to the timer.
 *
 * Input Parameters:
 *   hrtimer - The cancelled timer.
 *
 * Returned Value:
 *   The references count to the timer.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *
 ****************************************************************************/

static inline_function
int hrtimer_cancel_running(FAR hrtimer_t *timer)
{
  int refs            = 0;
#ifdef CONFIG_SMP
  uintptr_t cancelled = (uintptr_t)timer | 0x1u;
  int cpu;

  /* Check if the timer is referenced by any CPU core.
   * Generally, only one reference to a timer can exist at the same time.
   * However, when a timer may be restarted at the cancelled state,
   * more references to the timer may exist.
   */

  unroll_loop(CONFIG_SMP_NCPUS) /* Tell the compiler to unroll the loop. */

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      /* This is a faster implementation equivalent to
       * (g_hrtimer_running[cpu] & (~(uintptr_t)0x1u)) == timer,
       * Assuming the pointer of the timer is at least 2-bytes aligned
       * (the last bit must be zero).
       */

      if ((g_hrtimer_running[cpu] ^ cancelled) <= 0x1u)
        {
          /* Set the timer to the cancelled state and revoke the write
           * ownership of the timer from the queue.
           */

          g_hrtimer_running[cpu] = cancelled;
          refs++;
        }
    }
#endif

  return refs;
}

/****************************************************************************
 * Name: hrtimer_wait
 *
 * Description:
 *   Wait for all references to be released before the timer can be freed.
 *
 * Input Parameters:
 *   timer - The hrtimer to be waited.
 *
 * Returned Value:
 *   None.
 *
 * Assumption:
 *   The periodical hrtimer should be cancelled before calling this function.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static inline_function void hrtimer_wait(FAR hrtimer_t *timer)
{
  int cpu;

  DEBUGASSERT(timer && timer->func == NULL);

  /* Wait until all references have been released. */

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      /* The timer should not be restarted again.
       * Or there will be ownership invariant violation.
       */

      DEBUGASSERT(!hrtimer_is_running(timer, cpu));

      /* If sleeping is permitted, yield the CPU briefly to avoid
       * busy-waiting.  Otherwise, spin until the callback completes
       * and the state becomes inactive.
       */

      while (hrtimer_is_cancelling(timer, cpu))
        {
          if (!up_interrupt_context() && !is_idle_task(this_task()))
            {
              nxsched_usleep(HRTIMER_CANCEL_SYNC_DELAY_US);
            }

          /* Otherwise, spin-wait is enough. */
        }
    }

  /* Finally, we acquire the ownership of this hrtimer. */
}
#else
#  define hrtimer_wait(timer) /* Nothing to wait in non-SMP. */
#endif

#endif /* CONFIG_HRTIMER */
#endif /* __SCHED_HRTIMER_HRTIMER_H */
