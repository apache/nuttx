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
#include <nuttx/list.h>
#include <nuttx/seqlock.h>

#include <sys/tree.h>

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

/* Precision mask for reducing hrtimer resolution.
 * When CONFIG_HRTIMER_PRECISION_SHIFT > 0, the programmed expiration
 * time is ceiling-aligned to (1 << SHIFT) nanosecond boundaries.
 */

#define HRTIMER_PRECISION_MASK \
  ((1ull << CONFIG_HRTIMER_PRECISION_SHIFT) - 1u)

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

extern struct hrtimer_s g_hrtimer_guard;

#ifdef CONFIG_HRTIMER_TREE
/* Red-Black tree containing all active high-resolution timers */

extern struct hrtimer_tree_s g_hrtimer_tree;
extern struct FAR hrtimer_s *g_hrtimer_head;
#else
/* List containing all active high-resolution timers */

extern struct list_node g_hrtimer_list;
#endif

#ifdef CONFIG_SCHED_TICKLESS
extern uint64_t g_hrtimer_expected;
#endif

#ifdef CONFIG_SMP
/* Array of pointers to currently running high-resolution timers
 * for each CPU in SMP configurations. Index corresponds to CPU ID.
 */

extern uintptr_t g_hrtimer_running[CONFIG_SMP_NCPUS];
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_lock
 *
 * Description:
 *   Acquire the hrtimer queue lock. In SMP mode, this acquires the seqlock
 *   and disables interrupts. In non-SMP mode, only interrupts are disabled
 *   since no concurrent writers exist, saving the seqlock overhead.
 *
 * Input Parameters:
 *  None.
 *
 * Returned Value:
 *   The saved interrupt state.
 *
 ****************************************************************************/

static inline_function irqstate_t hrtimer_lock(void)
{
#ifdef CONFIG_SMP
  return write_seqlock_irqsave(&g_hrtimer_lock);
#else
  return up_irq_save();
#endif
}

/****************************************************************************
 * Name: hrtimer_unlock
 *
 * Description:
 *   Release the hrtimer queue lock. In SMP mode, this releases the seqlock
 *   and restores interrupts. In non-SMP mode, only interrupts are restored.
 *
 * Input Parameters:
 *   flags - The saved interrupt state from hrtimer_lock().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function
void hrtimer_unlock(irqstate_t flags)
{
#ifdef CONFIG_SMP
  write_sequnlock_irqrestore(&g_hrtimer_lock, flags);
#else
  up_irq_restore(flags);
#endif
}

/****************************************************************************
 * Name: hrtimer_last_expired
 *
 * Description:
 *   Return the last expired time of the hrtimer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The last expired time in nanoseconds.
 *
 ****************************************************************************/

static inline_function uint64_t hrtimer_last_expired(void)
{
  return g_hrtimer_guard.expired - HRTIMER_GUARD_DELAY;
}

/****************************************************************************
 * Name: hrtimer_reprogram
 *
 * Description:
 *   Reprogram the hardware timer to expire at a specified nanosecond time.
 *   Converts the nanosecond time to timespec and calls the platform-specific
 *   timer start function. If CONFIG_TIMER_ADJUST_NSEC is set, the expiration
 *   time is adjusted earlier by that amount to compensate for interrupt
 *   latency.
 *
 * Input Parameters:
 *   next_expired - Expiration time in nanoseconds.
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
  int             ret;
  struct timespec ts;
  uint64_t        delay;
  uint64_t        expired = hrtimer_last_expired();

  if (g_hrtimer_expected != next_expired)
    {
      g_hrtimer_expected = next_expired;
#  if CONFIG_HRTIMER_PRECISION_SHIFT > 0
      /* Ceiling-align the target expiration time to the precision boundary.
       * This coarsens the timer resolution to (1 << SHIFT) nanoseconds,
       * reducing the frequency of hardware timer reprogramming.
       */

      next_expired = (next_expired + HRTIMER_PRECISION_MASK) &
                     ~HRTIMER_PRECISION_MASK;
#  endif

      delay = HRTIMER_TIME_BEFORE_EQ(expired, next_expired) ?
              next_expired - expired : 0u;

#  ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
      delay = delay < TICK2NSEC((uint64_t)g_oneshot_maxticks) ?
              delay : TICK2NSEC((uint64_t)g_oneshot_maxticks);
#  endif

      /* Even if overflow occurs here, the functional correctness
       * should still be fine.
       */

      delay -= CONFIG_TIMER_ADJUST_NSEC;

      clock_nsec2time(&ts, expired + delay);

#  ifdef CONFIG_SCHED_TICKLESS_ALARM
      ret = up_alarm_start(&ts);
#  else
      struct timespec current;
      up_timer_gettime(&current);
      clock_timespec_subtract(&ts, &current, &ts);
      ret = up_timer_start(&ts);
#  endif

      DEBUGASSERT(ret == 0);
    }
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

#ifdef CONFIG_SMP
#  define DEFINE_HRTIMER_READ(suffix, type)                     \
static inline_function type                                     \
hrtimer_read_##suffix(FAR const type *ptr)                      \
{                                                               \
  type val;                                                     \
  uint32_t seq;                                                 \
  do                                                            \
    {                                                           \
      seq = read_seqbegin(&g_hrtimer_lock);                     \
      val = *ptr;                                               \
    }                                                           \
  while (read_seqretry(&g_hrtimer_lock, seq));                  \
  return val;                                                   \
}
#else
#  define DEFINE_HRTIMER_READ(suffix, type)                     \
static inline_function type                                     \
hrtimer_read_##suffix(FAR const type *ptr)                      \
{                                                               \
  irqstate_t flags = up_irq_save();                                \
  type val = *ptr;                                              \
  up_irq_restore(flags);                                           \
  return val;                                                   \
}
#endif

#ifdef CONFIG_ARCH_64BIT
/* On 64-bit architectures, read/write uint64_t is atomic. */
#  define hrtimer_read_64(ptr) (*(FAR volatile uint64_t *)(ptr))
#else
DEFINE_HRTIMER_READ(64, uint64_t)
#endif

#if UINT_MAX >= UINT32_MAX
/* On 32/64-bit architectures, read/write uint32_t is atomic. */
#  define hrtimer_read_32(ptr) (*(FAR volatile uint32_t *)(ptr))
#else
DEFINE_HRTIMER_READ(32, uint32_t)
#endif

/* Generic function to read the pointer in the queue atomically. */

#if UINTPTR_MAX <= UINT32_MAX
#  define hrtimer_read_ptr(ptr) hrtimer_read_32((FAR const uint32_t *)(ptr))
#else
#  define hrtimer_read_ptr(ptr) hrtimer_read_64((FAR const uint64_t *)(ptr))
#endif

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
  (hrtimer_read_ptr(&g_hrtimer_running[cpu]) == (uintptr_t)(timer))
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

static inline_function void hrtimer_wait(FAR hrtimer_t *timer)
{
#ifdef CONFIG_SMP
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
#endif
}

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
 * Assumption:
 *   This function can only be called in interrupt context.
 *
 ****************************************************************************/

static inline_function void hrtimer_process(uint64_t now)
{
  uint64_t          delay;
  int                 cpu = this_cpu();
  irqstate_t        flags = hrtimer_lock();
  FAR hrtimer_t  *hrtimer = hrtimer_get_first();
  uint64_t        expired = hrtimer->expired;
  hrtimer_entry_t    func = hrtimer->func;

#ifdef CONFIG_SCHED_TICKLESS
  /* Invalidate the g_hrtimer_expected */

  g_hrtimer_expected = now;
#endif

  /* Acquire the lock and seize the ownership of the hrtimer queue.
   * Then check if the timer has expired.
   */

  while (HRTIMER_TIME_BEFORE_EQ(expired, now))
    {
      /* Remove the expired timer from the timer queue */

      hrtimer_remove(hrtimer);

      hrtimer_mark_running(hrtimer, cpu);

      /* Leave critical section before invoking the callback */

      hrtimer_unlock(flags);

      /* Invoke the timer callback */

      delay = func(hrtimer, expired);

      /* Ensure the delay is valid. */

      DEBUGASSERT(HRTIMER_TIME_BEFORE_EQ(expired + delay,
                                         now + HRTIMER_MAX_DELAY));

      /* Re-enter critical section to update timer state */

      flags = hrtimer_lock();

      /* If the timer is periodic and has not been rearmed or
       * cancelled concurrently, calculate next expiration and
       * re-insert into the timer queue.
       */

      if (delay != 0u && hrtimer_is_running(hrtimer, cpu))
        {
          hrtimer->expired = expired + delay;
          hrtimer->func    = func;
          hrtimer_insert(hrtimer);
        }

      /* Fetch the earliest active timer */

      hrtimer = hrtimer_get_first();
      expired = hrtimer->expired;
      func    = hrtimer->func;
    }

  hrtimer_unmark_running(cpu);

  /* Note that the g_hrtimer_guard.expired is not monotonic. */

  g_hrtimer_guard.expired = now + HRTIMER_GUARD_DELAY;

  /* Schedule the next timer expiration */

  hrtimer_reprogram(expired);

  /* Release the lock and give up the ownership of the hrtimer queue. */

  hrtimer_unlock(flags);
}

#endif /* CONFIG_HRTIMER */
#endif /* __SCHED_HRTIMER_HRTIMER_H */
