/****************************************************************************
 * include/nuttx/mpsc.h
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

/* Multi-producer/single-consumer intrusive lock-free linked-list queue.
 *
 * Algorithm: Dmitry Vyukov's MPSC (stub-sentinel with recycle). The queue
 * is initialised with a single sentinel node (`stub`) so that `head` and
 * `tail` are never NULL. Producers serialise via a single atomic exchange
 * on `tail`; the consumer walks the linked list from `head`, skipping or
 * re-inserting the stub as needed to keep the list well-formed.
 *
 *   Producer-shared state : q->tail (atomic, xchg)
 *   Consumer-private state: q->head
 *   Cross-thread sync     : release-store on prev->next (producer)
 *                           acquire-load  on head->next (consumer)
 *
 * Hot-path cost (per round-trip, low contention):
 *   enqueue : 1 xchg-release + 1 release-store, wrapped in
 *             irq_save()/irq_restore() to bound the same-core
 *             "transient inconsistent" window if a producer is
 *             preempted between the xchg and the prev->next publish.
 *   dequeue : 1 acquire-load (best case). When the queue is about to
 *             become empty, dequeue re-enqueues the stub to keep the
 *             list non-empty - that path adds one xchg + release-store
 *             call, so worst-case dequeue cost is roughly equal to
 *             enqueue.
 *   Both sides are lock-free; enqueue is wait-free.
 *
 * Transient inconsistent state:
 *   Between a producer's xchg on `tail` and its store on `prev->next`,
 *   the consumer can observe `q->tail != q->head` while `head->next`
 *   is still NULL. This is intrinsic to any two-step intrusive MPSC
 *   publish and cannot be removed without losing lock-freedom.
 *   mpsc_dequeue() returns NULL in this window. Callers that need to
 *   distinguish "really empty" from "transient" can use mpsc_empty(),
 *   or use one of the helper wrappers below:
 *     mpsc_dequeue       - lock-free, returns NULL on transient/empty
 *     mpsc_dequeue_spin  - busy-spin transient, return NULL on empty
 *     mpsc_dequeue_yield - sched_yield() through transient, NULL on empty
 *   For "block until a node arrives", combine mpsc_dequeue() +
 *   mpsc_empty() with a semaphore the producer posts after enqueue.
 *
 * Node lifetime:
 *   The node returned by mpsc_dequeue() has been fully detached from
 *   the linked list before return; the caller may free or re-enqueue
 *   it immediately. The stub (q->stub) is owned by the queue and
 *   must never be freed or re-enqueued by the caller.
 *
 * IRQ handling:
 *   mpsc_enqueue() disables interrupts on the calling core for the
 *   duration of the xchg + release-store window only. This keeps the
 *   per-core preemption-induced transient bounded. Cross-core
 *   producers serialise via the atomic exchange and do not disable
 *   each other's interrupts. Do not extend the IRQ-disabled window by
 *   performing unrelated work between manual irq_save and
 *   mpsc_enqueue - prepare the node *before* calling mpsc_enqueue().
 *
 * Multi-core placement:
 *   When producers and consumer run on different cores, the queue
 *   descriptor and all enqueued nodes (including the embedded stub)
 *   must reside in cache-coherent memory. See the multi-core notes
 *   in include/nuttx/ring.h.
 */

#ifndef __INCLUDE_NUTTX_MPSC_H
#define __INCLUDE_NUTTX_MPSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/atomic.h>
#include <nuttx/irq.h>

#include <sched.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CACHELINE_SIZE
#  define CACHELINE_SIZE 64
#endif

/* MPSC_INIT - designated initialiser for a queue object.
 *
 * Uses a self-referencing initialiser to point head/tail at the
 * embedded stub. This relies on the GCC/Clang extension that allows
 * taking the address of the object being initialised; both compilers
 * supported by NuttX accept this form.
 *
 * Usage:
 *   struct mpsc_queue_s g_q = MPSC_INIT(g_q);
 */

#define MPSC_INIT(q)                       \
  {                                        \
    .head      = &(q).stub,                \
    .tail      = (intptr_t)&(q).stub,      \
    .stub.next = 0,                        \
  }

/* MPSC_DEFINE - define and initialise a queue.
 *
 * Usage:
 *   MPSC_DEFINE(g_my_queue);
 *   mpsc_enqueue(&g_my_queue, &node);
 */

#define MPSC_DEFINE(name) \
  struct mpsc_queue_s name = MPSC_INIT(name)

/* MPSC_DECLARE - declare a queue without initialisation.
 *
 * Caller must invoke mpsc_init() before any enqueue/dequeue. Use
 * this for queues placed in shared/uncached memory or allocated
 * dynamically.
 */

#define MPSC_DECLARE(name) \
  struct mpsc_queue_s name

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Intrusive queue node. Embed inside the user data structure and convert
 * with container_of(). The `next` field is the cross-thread sync point.
 */

struct mpsc_node_s
{
  atomic_ptr_t next;
};

/* MPSC queue descriptor. Head (consumer-private) and tail (producer-
 * shared) sit on separate cache lines to avoid false sharing.
 */

struct aligned_data(CACHELINE_SIZE) mpsc_queue_s
{
  /* Consumer side - written only by the consumer */

  FAR struct mpsc_node_s *head;
  uint8_t pad0[CACHELINE_SIZE - sizeof(FAR struct mpsc_node_s *)];

  /* Producer side - all producers contend on this atomic */

  atomic_ptr_t tail;
  uint8_t pad1[CACHELINE_SIZE - sizeof(atomic_ptr_t)];

  /* Embedded sentinel. Always present in the linked list; recycled
   * by the consumer when the queue is about to drain.
   */

  struct mpsc_node_s stub;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpsc_init
 *
 * Description:
 *   Initialise a queue. Required for MPSC_DECLARE() and for queues
 *   placed in shared memory; MPSC_DEFINE() initialises statically and
 *   does not need this. In multi-core setups, ensure a memory barrier
 *   (and any required IPI handshake) between init and first use on
 *   remote cores.
 *
 ****************************************************************************/

static inline_function void mpsc_init(FAR struct mpsc_queue_s *q)
{
  atomic_ptr_set(&q->stub.next, 0);
  q->head = &q->stub;
  atomic_ptr_set(&q->tail, (intptr_t)&q->stub);
}

/****************************************************************************
 * Name: mpsc_empty
 *
 * Description:
 *   Return true when the queue is genuinely empty - no linked
 *   successor *and* no producer in flight. Distinguishes "really
 *   empty" from "transient inconsistent" (where a producer has won
 *   the xchg but not yet completed the prev->next store).
 *
 *   Only meaningful when called from the consumer side.
 *
 ****************************************************************************/

static inline_function bool mpsc_empty(FAR struct mpsc_queue_s *q)
{
  FAR struct mpsc_node_s *head = q->head;
  FAR struct mpsc_node_s *next;
  FAR struct mpsc_node_s *tail;

  next = (FAR struct mpsc_node_s *)
         (intptr_t)atomic_ptr_read_acquire(&head->next);
  if (next != NULL)
    {
      return false;
    }

  /* No linked successor; verify no producer is in flight. */

  tail = (FAR struct mpsc_node_s *)
         (intptr_t)atomic_ptr_read(&q->tail);
  return head == tail;
}

/****************************************************************************
 * Name: mpsc_enqueue
 *
 * Description:
 *   Append a node. Wait-free per producer; safe from any number of
 *   concurrent producers (and from the consumer's stub recycle path).
 *
 *   The caller must populate the user data of `node` before this call.
 *   Once enqueued, the node belongs to the queue and must not be
 *   touched until returned by mpsc_dequeue().
 *
 ****************************************************************************/

static inline_function void
mpsc_enqueue(FAR struct mpsc_queue_s *q, FAR struct mpsc_node_s *node)
{
  FAR struct mpsc_node_s *prev;
  irqstate_t flags;

  /* Mark the new tail terminal. Relaxed is fine: the consumer reaches
   * `node` only through the release-store on prev->next below.
   */

  atomic_ptr_set(&node->next, 0);

  /* Bound the per-core preemption window: between the xchg and the
   * prev->next store, the queue is "transiently inconsistent" - tail
   * points at node but prev->next is still NULL. Disabling local IRQs
   * keeps another producer on the same core from extending this gap.
   */

  flags = irq_save();
  prev  = (FAR struct mpsc_node_s *)
          (intptr_t)atomic_ptr_xchg_release(&q->tail, (intptr_t)node);
  atomic_ptr_set_release(&prev->next, (intptr_t)node);
  irq_restore(flags);
}

/****************************************************************************
 * Name: mpsc_dequeue
 *
 * Description:
 *   Remove and return the next node, or NULL when the queue is empty
 *   or transiently inconsistent. Single consumer only. Lock-free.
 *
 *   The returned node is fully detached from the queue; the caller
 *   may free or re-enqueue it. Returning NULL is not an error - it
 *   may simply mean a producer is in the middle of an enqueue and
 *   the caller should retry shortly. See mpsc_dequeue_spin() and
 *   mpsc_dequeue_yield() for wrappers that hide the transient.
 *
 ****************************************************************************/

static inline_function FAR struct mpsc_node_s *
mpsc_dequeue(FAR struct mpsc_queue_s *q)
{
  FAR struct mpsc_node_s *head = q->head;
  FAR struct mpsc_node_s *next;
  FAR struct mpsc_node_s *tail;

  next = (FAR struct mpsc_node_s *)
         (intptr_t)atomic_ptr_read_acquire(&head->next);

  /* Skip the stub if we are sitting on it. */

  if (head == &q->stub)
    {
      if (next == NULL)
        {
          return NULL;
        }

      q->head = next;
      head    = next;
      next    = (FAR struct mpsc_node_s *)
                (intptr_t)atomic_ptr_read_acquire(&head->next);
    }

  /* Common case: another node is already linked after `head`. Advance
   * past `head` and return it - it is now detached from the list.
   */

  if (next != NULL)
    {
      q->head = next;
      return head;
    }

  /* `head` looks like the last node. If a producer has won the xchg
   * but not yet completed the prev->next store, treat as empty -
   * caller retries.
   */

  tail = (FAR struct mpsc_node_s *)
         (intptr_t)atomic_ptr_read(&q->tail);
  if (head != tail)
    {
      return NULL;
    }

  /* Re-enqueue the stub so the queue remains non-empty after we
   * detach `head`. Producers see this exactly like any other enqueue.
   */

  mpsc_enqueue(q, &q->stub);

  next = (FAR struct mpsc_node_s *)
         (intptr_t)atomic_ptr_read_acquire(&head->next);
  if (next == NULL)
    {
      return NULL;
    }

  q->head = next;
  return head;
}

/****************************************************************************
 * Name: mpsc_dequeue_spin
 *
 * Description:
 *   Wrapper that hides the transient-inconsistent window from the
 *   caller. Busy-spins while a producer is in the middle of an
 *   enqueue, returns NULL only when the queue is genuinely empty.
 *
 *   Use when transient windows are expected to be very short
 *   (hand-off between cores, no producer preemption) and the caller
 *   prefers low-latency to giving up the CPU. Not suitable when a
 *   producer may be preempted while in mpsc_enqueue() - the spin
 *   would last as long as that preemption.
 *
 *   Loses lock-freedom of the consumer side: progress depends on
 *   the in-flight producer making progress.
 *
 ****************************************************************************/

static inline_function FAR struct mpsc_node_s *
mpsc_dequeue_spin(FAR struct mpsc_queue_s *q)
{
  FAR struct mpsc_node_s *node;

  while ((node = mpsc_dequeue(q)) == NULL)
    {
      if (mpsc_empty(q))
        {
          return NULL;
        }

      /* Transient: producer between xchg and prev->next store. */
    }

  return node;
}

/****************************************************************************
 * Name: mpsc_dequeue_yield
 *
 * Description:
 *   Wrapper that hides the transient-inconsistent window by yielding
 *   the CPU instead of busy-spinning. Returns NULL only when the
 *   queue is genuinely empty.
 *
 *   Use in cooperative or user-space contexts where giving up the
 *   CPU is preferable to spinning - especially when a producer may
 *   be preempted (yielding gives it a chance to run).
 *
 *   Must not be called from interrupt context.
 *
 ****************************************************************************/

static inline_function FAR struct mpsc_node_s *
mpsc_dequeue_yield(FAR struct mpsc_queue_s *q)
{
  FAR struct mpsc_node_s *node;

  while ((node = mpsc_dequeue(q)) == NULL)
    {
      if (mpsc_empty(q))
        {
          return NULL;
        }

      sched_yield();
    }

  return node;
}

#endif /* __INCLUDE_NUTTX_MPSC_H */
