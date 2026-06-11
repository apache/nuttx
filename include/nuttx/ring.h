/****************************************************************************
 * include/nuttx/ring.h
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

/* Unified lock-free ring queue supporting SPSC/MPSC/SPMC/MPMC modes.
 *
 * Inspired by DPDK rte_ring, uses 32-bit RTS (Relaxed Tail Sync)
 * algorithm packing {cnt:8, idx:24} into a single atomic_t for
 * multi-producer/multi-consumer coordination via single CAS,
 * eliminating tail-wait spin without disabling interrupts.
 *
 * Usage Guide:
 *
 * 1. Declaration and Initialization:
 *
 *   Static (compile-time):
 *     RING_DEFINE(g_ring, uint32_t, 64);
 *
 *   Dynamic (runtime):
 *     struct ring_s *r = ring_alloc(64, sizeof(uint32_t));
 *     ring_free(r);
 *
 *   Multi-core safe init (SMP, uses atomic CAS):
 *     ring_init(&g_ring, 64);
 *
 *   Single-side init (AMP/SPSC, no atomic needed):
 *     ring_spsc_init(&g_ring, 64);
 *     // Other side polls: while (!ring_is_ready(&g_ring));
 *
 * 2. Mode Selection:
 *
 *   SPSC - Single Producer, Single Consumer (volatile + barrier):
 *     ring_spsc_enqueue(&g_ring, &val, 1);
 *     ring_spsc_dequeue(&g_ring, &out, 1);
 *
 *   MPSC - Multiple Producers, Single Consumer:
 *     ring_mpsc_enqueue(&g_ring, &val, 1);
 *     ring_mpsc_dequeue(&g_ring, &out, 1);
 *
 *   SPMC - Single Producer, Multiple Consumers:
 *     ring_spmc_enqueue(&g_ring, &val, 1);
 *     ring_spmc_dequeue(&g_ring, &out, 1);
 *
 *   MPMC - Multiple Producers, Multiple Consumers:
 *     ring_mpmc_enqueue(&g_ring, &val, 1);
 *     ring_mpmc_dequeue(&g_ring, &out, 1);
 *
 * 3. Skip (advance consumer index without reading data):
 *
 *     ring_spsc_skip(&g_ring, n);
 *     ring_mpsc_skip(&g_ring, n);
 *     ring_spmc_skip(&g_ring, n);
 *     ring_mpmc_skip(&g_ring, n);
 *
 *    Skip-to (advance cons_tail to a specific target index):
 *
 *     ring_spsc_skipto(&g_ring, target_index);
 *     ring_mpsc_skipto(&g_ring, target_index);
 *     ring_spmc_skipto(&g_ring, target_index);
 *     ring_mpmc_skipto(&g_ring, target_index);
 *
 * 4. Overwrite Mode (SPMC/MPMC only, discards oldest on full):
 *
 *     ring_spmc_overwrite(&g_ring, &val, 1);
 *     ring_mpmc_overwrite(&g_ring, &val, 1);
 *     // Consumer MUST use MC dequeue (ring_spmc/mpmc_dequeue)
 *
 * 5. Zero-copy enqueue/dequeue (start/finish pattern):
 *
 *     struct ring_buf_s buf;
 *
 *     // Enqueue (all modes: spsc/mpsc/spmc/mpmc):
 *     ring_spsc_enqueue_start(&g_ring, count, elem_size, &buf);
 *     memcpy(buf.ptr1, src, buf.len1 * elem_size);
 *     if (buf.ptr2)
 *       memcpy(buf.ptr2, src + buf.len1 * elem_size,
 *              buf.len2 * elem_size);
 *     ring_spsc_enqueue_finish(&g_ring, &buf);
 *
 *     // Dequeue (all modes):
 *     ring_spsc_dequeue_start(&g_ring, count, elem_size, &buf);
 *     // Read from buf.ptr1 (len1) and buf.ptr2 (len2)
 *     ring_spsc_dequeue_finish(&g_ring, &buf);
 *
 * 6. Zero-copy overwrite (start/finish pattern):
 *
 *     struct ring_buf_s buf;
 *     ring_spmc_overwrite_start(&g_ring, count,
 *                               elem_size, &buf);
 *     memcpy(buf.ptr1, src, buf.len1 * elem_size);
 *     if (buf.ptr2)
 *       memcpy(buf.ptr2, src + buf.len1 * elem_size,
 *              buf.len2 * elem_size);
 *     ring_spmc_overwrite_finish(&g_ring, &buf);
 *
 * 7. Query:
 *
 *     ring_count(&g_ring);     // Elements currently in ring
 *     ring_capacity(&g_ring);  // Max elements
 *     ring_space(&g_ring);     // Free slots
 *     ring_is_ready(&g_ring);  // true if initialized
 */

#ifndef __INCLUDE_NUTTX_RING_H
#define __INCLUDE_NUTTX_RING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/arch.h>
#include <nuttx/atomic.h>
#include <nuttx/irq.h>
#include <nuttx/lib/lib.h>

#include <assert.h>
#include <string.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 24-bit index mask */

#define RING_IDX_MASK    0xffffff

/* Maximum ring capacity (fits in 24-bit index space) */

#define RING_SIZE_MAX    RING_IDX_MASK

/* Extract data capacity from total buffer size (total - header) */

#define RING_CAPACITY(l) ((l) - sizeof(struct ring_s))

/* 32-bit RTS packed type: {cnt:8, idx:24} in a single atomic_t.
 *
 * Layout (little-endian and big-endian safe via shift/mask):
 *   bits [31:24] = cnt  (8-bit thread counter)
 *   bits [23:0]  = idx  (24-bit ring index)
 *
 * Never access cnt/idx via struct bitfields - use the macros below
 * to pack/unpack from the raw 32-bit atomic value.
 */

#define RTS_CNT_SHIFT    24
#define RTS_CNT_MASK     0xff000000u
#define RTS_IDX_MASK     0x00ffffffu

#define RTS_CNT(raw)     ((uint8_t)(((uint32_t)(raw) >> RTS_CNT_SHIFT) & 0xff))
#define RTS_IDX(raw)     ((uint32_t)(raw) & RTS_IDX_MASK)
#define RTS_PACK(cnt, idx) \
  ((((uint32_t)(cnt) & 0xff) << RTS_CNT_SHIFT) | ((uint32_t)(idx) & RTS_IDX_MASK))

#ifndef CACHELINE_SIZE
#  define CACHELINE_SIZE 64
#endif

/* Magic values for warm-restart detection.
 *
 * When a ring resides in memory that survives reboot (e.g. shared
 * SRAM), ring_init() uses an atomic magic field to decide whether
 * to perform a fresh initialization or recover the previous state.
 *
 * Three-state protocol (multi-core safe):
 *   0                -> uninitialized (or garbage != READY/INIT)
 *   RING_MAGIC_INIT  -> one core is initializing; others spin-wait
 *   RING_MAGIC_READY -> init complete; ring is usable
 */

#define RING_MAGIC_INIT   ((int32_t)0x52494e49)   /* "RINI" */
#define RING_MAGIC_READY  ((int32_t)0x52494e47)   /* "RING" */

/* RING_INIT - static initialize a ring
 *
 * buflen - total buffer size in bytes (header + data)
 *
 * Usage:
 *   RING_DECLARE(g_my_ring, uint32_t, 64) = RING_INIT(64);
 */

#define RING_INIT(buflen)                                \
  {                                                      \
    .base.magic    = RING_MAGIC_READY,                   \
    .base.capacity = RING_CAPACITY(buflen),              \
    .base.mask     = (RING_CAPACITY(buflen) &            \
                     (RING_CAPACITY(buflen) - 1))        \
                     ? 0 : RING_CAPACITY(buflen) - 1,    \
  }

/* RING_DECLARE - declare a ring variable (uninitialized).
 * Must call ring_init() or RING_INIT() before use.
 *
 * Usage:
 *   RING_DECLARE(g_my_ring, uint32_t, 64);
 *   ring_init(g_my_ring, 64);
 *   or
 *   RING_DECLARE(g_my_ring, uint32_t, 64) = RING_INIT(64);
 *
 *   then:
 *   ring_spsc_enqueue(&g_my_ring, &val, 1);
 */

#define RING_DECLARE(name, type, buflen)                 \
  struct ring_##name##_s                                 \
  {                                                      \
    static_assert(RING_CAPACITY(buflen) <= RING_SIZE_MAX,\
                  "ring capacity exceeds RING_SIZE_MAX");\
    struct ring_s base;                                  \
    type          buf[RING_CAPACITY(buflen)];            \
  } name aligned_data(CACHELINE_SIZE)

/* RING_DEFINE - define and initialise a ring (no init call needed).
 *
 * name   - variable name
 * type   - element type
 * buflen - total buffer size in bytes (header + data)
 *
 * Usage:
 *   RING_DEFINE(g_my_ring, uint32_t, 64);
 *   ring_spsc_enqueue(&my_ring, &val, 1);
 */

#define RING_DEFINE(name, type, buflen)                  \
  RING_DECLARE(name, type, buflen) = RING_INIT(buflen)

/* RING_BASE - convert a RING_DECLARE/RING_DEFINE variable to
 * struct ring_s pointer for use with internal APIs.
 */

#define RING_BASE(ring)       ((FAR struct ring_s *)(ring))

#define RING_FIELD(ptr)       (*(FAR volatile int32_t *)(ptr))

/* Public API macros - accept RING_DECLARE/RING_DEFINE structs */

/* ring_reset - clear all indices and return the count of elements that
 * were present before the reset (i.e. the number of elements discarded
 * by the reset operation).
 *
 * Callers may either use the return value (e.g. to bump a "dropped"
 * counter) or discard it.  Safe to call on a freshly-zeroed ring whose
 * indices have not been written yet: reading the volatile fields yields
 * whatever happens to be in memory and the caller is expected to discard
 * the value in that case.
 */

#define ring_reset(r)       ring_reset_indices(RING_BASE(r))

/* ring_init - runtime initialization with warm-restart recovery.
 *
 * If the ring resides in memory that survives reboot (shared SRAM,
 * battery-backed RAM), ring_init detects prior initialization via
 * an atomic magic field and attempts to recover the previous state
 * instead of zeroing everything.
 *
 * Multi-core safe: concurrent ring_init calls from different cores
 * are serialized via CAS on the magic field. Cores that lose the
 * race spin-wait until the winner finishes, so no core can use the
 * ring before initialization is complete.
 *
 * Protocol:
 *   magic == 0 / garbage  -> fresh init (zero all indices)
 *   magic == RING_MAGIC_INIT  -> another core is initializing; spin
 *   magic == RING_MAGIC_READY -> warm restart; validate & recover
 */

#define ring_init(buf, buflen)   ring_init_magic(buf, buflen)

/* ring_data - return pointer to the embedded data buffer that immediately
 * follows the ring_s header in memory.
 */

#define ring_data(r)  ((FAR char *)((FAR struct ring_s *)(r) + 1))

/* ring_spsc_init - lightweight initialization for SPSC mode.
 *
 * Unlike ring_init (which uses atomic CAS for multi-core safe
 * concurrent initialization), ring_spsc_init uses only volatile
 * writes. It can be called by either the producer or consumer
 * side, but NOT concurrently from both — the caller must ensure
 * only one side calls init. The other side polls
 * ring_is_ready() before accessing the ring.
 *
 * Use ring_spsc_init for AMP/SHM scenarios where atomic
 * instructions are unavailable on shared memory.
 * Use ring_init for SMP/BMP scenarios where multiple cores may
 * race to initialize the same ring.
 */

#define ring_spsc_init(r, cap)                           \
  do                                                     \
    {                                                    \
      FAR struct ring_s *ring = RING_BASE(r);            \
      irqstate_t flags = irq_save();                     \
      if (RING_FIELD(&ring->magic) != RING_MAGIC_READY)  \
        {                                                \
          RING_FIELD(&ring->magic) = RING_MAGIC_INIT;    \
          UP_WMB();                                      \
          ring->capacity = (cap);                        \
          ring->mask = ((cap) & ((cap) - 1)) ? 0 :       \
                       (cap) - 1;                        \
          ring_reset(r);                                 \
          UP_WMB();                                      \
          RING_FIELD(&ring->magic) = RING_MAGIC_READY;   \
        }                                                \
      irq_restore(flags);                                \
    }                                                    \
  while (0)

#define ring_is_ready(r) \
  (RING_FIELD(&RING_BASE(r)->magic) == RING_MAGIC_READY)

#define ring_count(r)                                    \
  ((RTS_IDX(RING_FIELD(&RING_BASE(r)->prod_tail)) -      \
    RTS_IDX(RING_FIELD(&RING_BASE(r)->cons_tail))) &     \
   RING_IDX_MASK)

#define ring_capacity(r)    (RING_BASE(r)->capacity)
#define ring_space(r)       (ring_capacity(r) - ring_count(r))

/* Check whether cons_index is still valid (not overwritten).
 * Returns true if cons_index >= current cons_tail (data still valid).
 */

#define ring_is_valid_cons(cons_index, cons_tail) \
  ((int32_t)((cons_index) - (cons_tail)) >= 0)

/* Clamp an external index if it fell behind cons_tail (overwritten).
 * Returns the effective read position.
 */

#define ring_clamp_cons(cons_index, cons_tail) \
  (ring_is_valid_cons(cons_index, cons_tail) ? (cons_index) : (cons_tail))

#define ring_spsc_enqueue(r, items, count)               \
  (ring_is_ready(r) ?                                    \
   ring_sp_enqueue(RING_BASE(r), items, count, sizeof(*(items))) : 0)

#define ring_spsc_dequeue(r, items, count)               \
  (ring_is_ready(r) ?                                    \
   ring_sc_dequeue(RING_BASE(r), items, count, sizeof(*(items))) : 0)

#define ring_spsc_skip(r, count)                         \
  (ring_is_ready(r) ?                                    \
   ring_sc_skip(RING_BASE(r), count) : 0)

#define ring_mpmc_enqueue(r, items, count) \
  ring_mp_enqueue(RING_BASE(r), items, count, sizeof(*(items)))

#define ring_mpmc_dequeue(r, items, count) \
  ring_mc_dequeue(RING_BASE(r), items, count, sizeof(*(items)))

#define ring_spmc_enqueue(r, items, count) \
  ring_sp_enqueue(RING_BASE(r), items, count, sizeof(*(items)))

#define ring_spmc_dequeue(r, items, count) \
  ring_mc_dequeue(RING_BASE(r), items, count, sizeof(*(items)))

#define ring_mpsc_enqueue(r, items, count) \
  ring_mp_enqueue(RING_BASE(r), items, count, sizeof(*(items)))

#define ring_mpsc_dequeue(r, items, count) \
  ring_sc_dequeue(RING_BASE(r), items, count, sizeof(*(items)))

#define ring_mpmc_skip(r, count) \
  ring_mc_skip(RING_BASE(r), count)

#define ring_spmc_skip(r, count) \
  ring_mc_skip(RING_BASE(r), count)

#define ring_mpsc_skip(r, count) \
  ring_sc_skip(RING_BASE(r), count)

#define ring_mpsc_skipto(r, target) \
  ring_sc_skipto(RING_BASE(r), target)

#define ring_spsc_skipto(r, target) \
  ring_sc_skipto(RING_BASE(r), target)

#define ring_spmc_skipto(r, target) \
  ring_mc_skipto(RING_BASE(r), target)

#define ring_mpmc_skipto(r, target) \
  ring_mc_skipto(RING_BASE(r), target)

/* Overwrite: when the ring is full, discard oldest data to
 * make room.  The discard path acts as a virtual multi-consumer,
 * so the dequeue side MUST use MC protocol (ring_spmc_dequeue or
 * ring_mpmc_dequeue) regardless of the number of real consumers.
 *
 * Only available for MC queues (SPMC/MPMC). SC queues cannot
 * support overwrite because the producer must CAS cons_head to
 * discard, which conflicts with SC's plain-store protocol.
 */

#define ring_spmc_overwrite(r, items, count) \
  ring_sp_overwrite(RING_BASE(r), items, count, sizeof(*(items)))

#define ring_mpmc_overwrite(r, items, count) \
  ring_mp_overwrite(RING_BASE(r), items, count, sizeof(*(items)))

#define ring_spmc_overwrite_start(r, count, elem_size, b) \
  ring_sp_overwrite_start(RING_BASE(r), count, elem_size, b)

#define ring_spmc_overwrite_finish(r, b) \
  ring_sp_finish(RING_BASE(r), b)

#define ring_mpmc_overwrite_start(r, count, elem_size, b) \
  ring_mp_overwrite_start(RING_BASE(r), count, elem_size, b)

#define ring_mpmc_overwrite_finish(r, b) \
  ring_mp_finish(RING_BASE(r), b)

#define ring_spsc_enqueue_start(r, count, elem_size, b) \
  ring_sp_start(RING_BASE(r), count, elem_size, b)

#define ring_spsc_enqueue_finish(r, b) \
  ring_sp_finish(RING_BASE(r), b)

#define ring_spsc_dequeue_start(r, count, elem_size, b) \
  ring_sc_start(RING_BASE(r), count, elem_size, b)

#define ring_spsc_dequeue_finish(r, b) \
  ring_sc_finish(RING_BASE(r), b)

#define ring_mpmc_enqueue_start(r, count, elem_size, b) \
  ring_mp_start(RING_BASE(r), count, elem_size, b)

#define ring_mpmc_enqueue_finish(r, b) \
  ring_mp_finish(RING_BASE(r), b)

#define ring_mpmc_dequeue_start(r, count, elem_size, b) \
  ring_mc_start(RING_BASE(r), count, elem_size, b)

#define ring_mpmc_dequeue_finish(r, b) \
  ring_mc_finish(RING_BASE(r), b)

#define ring_spmc_enqueue_start(r, count, elem_size, b) \
  ring_sp_start(RING_BASE(r), count, elem_size, b)

#define ring_spmc_enqueue_finish(r, b) \
  ring_sp_finish(RING_BASE(r), b)

#define ring_spmc_dequeue_start(r, count, elem_size, b) \
  ring_mc_start(RING_BASE(r), count, elem_size, b)

#define ring_spmc_dequeue_finish(r, b) \
  ring_mc_finish(RING_BASE(r), b)

#define ring_mpsc_enqueue_start(r, count, elem_size, b) \
  ring_mp_start(RING_BASE(r), count, elem_size, b)

#define ring_mpsc_enqueue_finish(r, b) \
  ring_mp_finish(RING_BASE(r), b)

#define ring_mpsc_dequeue_start(r, count, elem_size, b) \
  ring_sc_start(RING_BASE(r), count, elem_size, b)

#define ring_mpsc_dequeue_finish(r, b) \
  ring_sc_finish(RING_BASE(r), b)

#define ring_mpsc_peek(r, items, count, index) \
  ring_sc_peek(RING_BASE(r), items, count, sizeof(*(items)), index)

#define ring_spsc_peek(r, items, count, index) \
  ring_sc_peek(RING_BASE(r), items, count, sizeof(*(items)), index)

#define ring_mpmc_peek(r, items, count, index) \
  ring_mc_peek(RING_BASE(r), items, count, sizeof(*(items)), index)

#define ring_spmc_peek(r, items, count, index) \
  ring_mc_peek(RING_BASE(r), items, count, sizeof(*(items)), index)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* NOTE: The cache-line isolation between the metadata / producer /
 * consumer regions is provided by the explicit pad0/pad1/pad2 arrays,
 * not by a type-level aligned attribute.  The aligned_data(CACHELINE_SIZE)
 * attribute is applied at each declaration/definition site instead (see
 * RING_DECLARE and ring_alloc), so that it binds to the
 * variable rather than the type.  Binding it to the type makes some
 * compilers (e.g. TASKING) propagate the alignment to every member,
 * inflating sizeof(struct ring_s) and breaking the cross-core shared
 * memory layout.
 */

struct ring_s
{
  /* Read-only metadata - cache-line aligned */

  atomic_t     magic;             /* Warm-restart detection */
  uint32_t     capacity;          /* Max 16777215 */
  uint32_t     mask;              /* capacity-1 if power-of-2, else 0 (use modulo) */
  uint8_t      pad0[CACHELINE_SIZE - sizeof(atomic_t) -
                    sizeof(uint32_t) * 2];

  /* Producer side - cache-line aligned */

  atomic_t     prod_head;         /* SP: unused; MP: RTS CAS */
  atomic_t     prod_tail;         /* SP: store_release; MP: RTS CAS */
  uint8_t      pad1[CACHELINE_SIZE - sizeof(atomic_t) * 2];

  /* Consumer side - cache-line aligned */

  atomic_t     cons_head;         /* SC: unused; MC: RTS CAS */
  atomic_t     cons_tail;         /* SC: store_release; MC: RTS CAS */
  uint8_t      pad2[CACHELINE_SIZE - sizeof(atomic_t) * 2];
};

struct ring_buf_s
{
  FAR void    *ptr1;              /* First contiguous segment */
  uint32_t     len1;              /* Element count in first segment */
  FAR void    *ptr2;              /* Second segment (wrap), NULL if none */
  uint32_t     len2;              /* Element count in second segment */
  uint32_t     idx;               /* Internal: start index */
  uint32_t     cnt;               /* Internal: total slots reserved */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ring_idx
 *
 * Description:
 *   Convert a logical index to a physical buffer position. Uses
 *   bitmask when capacity is power-of-2, modulo otherwise.
 *
 ****************************************************************************/

static inline_function uint32_t ring_idx(FAR struct ring_s *r, uint32_t idx)
{
  return r->mask ? (idx & r->mask) : (idx % r->capacity);
}

/****************************************************************************
 * Name: ring_reset_indices
 *
 * Description:
 *   Clear all indices and return the count of elements that were present
 *   before the reset (i.e. the number of elements discarded by the reset
 *   operation).  Called by the ring_reset() macro after RING_BASE()
 *   conversion, and may also be invoked directly when the caller already
 *   holds a FAR struct ring_s * (e.g. from inside other inline helpers
 *   such as ring_init_magic).
 *
 *   Callers may either use the return value (e.g. to bump a "dropped"
 *   counter) or discard it.  Safe to call on a freshly-zeroed ring whose
 *   indices have not been written yet: reading the volatile fields yields
 *   whatever happens to be in memory and the caller is expected to discard
 *   the value in that case.
 *
 ****************************************************************************/

static inline_function uint32_t ring_reset_indices(FAR struct ring_s *r)
{
  uint32_t dropped = ring_count(r);

  if (dropped > r->capacity)
    {
      dropped = r->capacity;
    }

  RING_FIELD(&r->prod_head) = 0;
  RING_FIELD(&r->prod_tail) = 0;
  RING_FIELD(&r->cons_head) = 0;
  RING_FIELD(&r->cons_tail) = 0;

  return dropped;
}

/****************************************************************************
 * Name: ring_init_magic
 *
 * Description:
 *   Runtime initialization with warm-restart recovery and multi-core
 *   safety.  Called by the ring_init() macro after capacity is set.
 *
 *   Three outcomes based on the magic field:
 *     1. Uninitialized (magic != READY and != INIT):
 *        Winner CAS to INIT, zeroes indices, sets READY.
 *        Losers spin until READY.
 *     2. Already READY (warm restart):
 *        Winner CAS to INIT, validates/recovers indices, sets READY.
 *        Losers spin until READY.
 *     3. INIT in progress:
 *        Another core is initializing; spin until READY.
 *
 ****************************************************************************/

static inline_function FAR struct ring_s *
ring_init_magic(FAR void *buf, uint32_t buflen)
{
  FAR struct ring_s *r = (FAR struct ring_s *)buf;
  irqstate_t flags;
  int32_t old_magic;

  DEBUGASSERT(buf != NULL && buflen > sizeof(struct ring_s) &&
              RING_CAPACITY(buflen) <= RING_SIZE_MAX);

  buflen = RING_CAPACITY(buflen);

  flags = irq_save();

  do
    {
      old_magic = atomic_read(&r->magic);

      if (old_magic == RING_MAGIC_INIT)
        {
          irq_restore(flags);

          /* Another core is initializing - spin until READY */

          while (atomic_read_acquire(&r->magic) != RING_MAGIC_READY)
            {
              UP_WFE();
            }

          return r;
        }
    }
  while (!atomic_cmpxchg(&r->magic, &old_magic, RING_MAGIC_INIT));

  r->capacity = buflen;
  r->mask = (buflen & (buflen - 1)) ? 0 : buflen - 1;

  if (old_magic == RING_MAGIC_READY)
    {
      /* Warm restart: validate indices.
       *
       * Invariant: count = (prod_idx - cons_idx) must be in
       * [0, capacity].  If violated, the indices are corrupt
       * and we fall back to a fresh reset.
       */

      uint32_t prod_index = RTS_IDX(atomic_read(&r->prod_tail));
      uint32_t cons_index = RTS_IDX(atomic_read(&r->cons_tail));
      uint32_t count = (prod_index - cons_index) & RING_IDX_MASK;

      if (count > r->capacity)
        {
          ring_reset(r);
        }
      else
        {
          /* Indices are valid - rebuild with cnt=0.
           *
           * A reboot mid-enqueue/dequeue can leave head.cnt != tail.cnt,
           * which deadlocks the RTS commit (tail can never catch head).
           * Reset cnt to 0 on all four atomics, keeping only the idx.
           */

          uint32_t prod_tail = RTS_PACK(0, prod_index);
          uint32_t cons_tail = RTS_PACK(0, cons_index);

          atomic_set(&r->prod_tail, prod_tail);
          atomic_set(&r->prod_head, prod_tail);
          atomic_set(&r->cons_tail, cons_tail);
          atomic_set(&r->cons_head, cons_tail);
        }
    }
  else
    {
      /* First-time init: zero everything */

      ring_reset(r);
    }

  /* Publish: all stores above are visible before magic becomes READY */

  atomic_set_release(&r->magic, RING_MAGIC_READY);
  UP_SEV();
  irq_restore(flags);
  return r;
}

/****************************************************************************
 * Name: ring_alloc
 *
 * Description:
 *   Dynamically allocate a ring descriptor + buffer.
 *
 ****************************************************************************/

static inline_function FAR struct ring_s *ring_alloc(uint32_t cap,
                                                     size_t elem_size)
{
  FAR struct ring_s *r;

  if (cap == 0)
    {
      return NULL;
    }

  r = lib_memalign(CACHELINE_SIZE, sizeof(*r) + cap * elem_size);
  if (r == NULL)
    {
      return NULL;
    }

  ring_init(r, cap);
  return r;
}

/****************************************************************************
 * Name: ring_free
 *
 * Description:
 *   Free a dynamically allocated ring.
 *
 ****************************************************************************/

static inline_function void ring_free(FAR struct ring_s *r)
{
  lib_free(r);
}

/****************************************************************************
 * Name: ring_buf_init
 *
 * Description:
 *   Initialize a ring_buf_s with pointers to the ring buffer region
 *   starting at idx for cnt elements, handling wrap-around.
 *   Sets cnt to 0 if the requested count exceeds available/free space.
 *
 ****************************************************************************/

static inline_function void
ring_buf_init(FAR struct ring_buf_s *buf, FAR struct ring_s *r,
              uint32_t idx, uint32_t count, size_t elem_size)
{
  uint32_t first;

  buf->cnt = count;

  if (count == 0)
    {
      buf->ptr1 = NULL;
      buf->len1 = 0;
      buf->ptr2 = NULL;
      buf->len2 = 0;
      buf->idx  = 0;
      return;
    }

  buf->idx = idx;

  idx   = ring_idx(r, idx);
  first = r->capacity - idx;

  if (first >= count)
    {
      buf->ptr1 = ring_data(r) + idx * elem_size;
      buf->len1 = count;
      buf->ptr2 = NULL;
      buf->len2 = 0;
    }
  else
    {
      buf->ptr1 = ring_data(r) + idx * elem_size;
      buf->len1 = first;
      buf->ptr2 = ring_data(r);
      buf->len2 = count - first;
    }
}

/****************************************************************************
 * Name: ring_copy_from
 *
 * Description:
 *   Copy elements from a linear buffer into ring_buf_s segments.
 *
 ****************************************************************************/

static inline_function void
ring_copy_from(FAR struct ring_buf_s *buf, FAR const void *items,
               size_t elem_size)
{
  if (buf->cnt == 1)
    {
      if (elem_size == sizeof(uint8_t))
        {
          *(FAR uint8_t *)buf->ptr1 = *(FAR const uint8_t *)items;
        }
      else if (elem_size == sizeof(uint16_t))
        {
          *(FAR uint16_t *)buf->ptr1 = *(FAR const uint16_t *)items;
        }
      else if (elem_size == sizeof(uint32_t))
        {
          *(FAR uint32_t *)buf->ptr1 = *(FAR const uint32_t *)items;
        }
      else if (elem_size == sizeof(uint64_t))
        {
          *(FAR uint64_t *)buf->ptr1 = *(FAR const uint64_t *)items;
        }
      else
        {
          memcpy(buf->ptr1, items, elem_size);
        }
    }
  else if (buf->cnt != 0)
    {
      memcpy(buf->ptr1, items, buf->len1 * elem_size);
      if (buf->len2 != 0)
        {
          memcpy(buf->ptr2, (FAR const char *)items + buf->len1 * elem_size,
                 buf->len2 * elem_size);
        }
    }
}

/****************************************************************************
 * Name: ring_copy_to
 *
 * Description:
 *   Copy elements from ring_buf_s segments to a linear buffer.
 *
 ****************************************************************************/

static inline_function void
ring_copy_to(FAR struct ring_buf_s *buf, FAR void *items, size_t elem_size)
{
  if (buf->cnt == 1)
    {
      if (elem_size == sizeof(uint8_t))
        {
          *(FAR uint8_t *)items = *(FAR const uint8_t *)buf->ptr1;
        }
      else if (elem_size == sizeof(uint16_t))
        {
          *(FAR uint16_t *)items = *(FAR const uint16_t *)buf->ptr1;
        }
      else if (elem_size == sizeof(uint32_t))
        {
          *(FAR uint32_t *)items = *(FAR const uint32_t *)buf->ptr1;
        }
      else if (elem_size == sizeof(uint64_t))
        {
          *(FAR uint64_t *)items = *(FAR const uint64_t *)buf->ptr1;
        }
      else
        {
          memcpy(items, buf->ptr1, elem_size);
        }
    }
  else if (buf->cnt != 0)
    {
      memcpy(items, buf->ptr1, buf->len1 * elem_size);
      if (buf->len2 != 0)
        {
          memcpy((FAR char *)items + buf->len1 * elem_size,
                 buf->ptr2, buf->len2 * elem_size);
        }
    }
}

/****************************************************************************
 * Name: ring_sp_start
 *
 * Description:
 *   Single-producer start. Calculate free space, initialize ring_buf_s.
 *
 ****************************************************************************/

static inline_function void
ring_sp_start(FAR struct ring_s *r, uint32_t count,
              size_t elem_size, FAR struct ring_buf_s *buf)
{
  uint32_t prod_index = RTS_IDX(RING_FIELD(&r->prod_tail));
  uint32_t cons_index;
  uint32_t free_space;
  uint32_t n;

  UP_RMB();
  cons_index = RTS_IDX(RING_FIELD(&r->cons_tail));
  free_space = r->capacity -
               ((prod_index - cons_index) & RING_IDX_MASK);
  n = count < free_space ? count : free_space;

  ring_buf_init(buf, r, prod_index, n, elem_size);
}

/****************************************************************************
 * Name: ring_mp_start
 *
 * Description:
 *   Multi-producer start. CAS on prod_head to reserve up to count
 *   slots, initialize ring_buf_s.
 *
 ****************************************************************************/

static inline_function void
ring_mp_start(FAR struct ring_s *r, uint32_t count,
              size_t elem_size, FAR struct ring_buf_s *buf)
{
  uint32_t old_head;
  uint32_t new_head;
  uint32_t n;

  old_head = atomic_read(&r->prod_head);
  do
    {
      uint32_t cons_index = RTS_IDX(atomic_read_acquire(&r->cons_tail));
      uint32_t free_space = r->capacity -
        ((RTS_IDX(old_head) - cons_index) & RING_IDX_MASK);

      n = count < free_space ? count : free_space;
      if (n == 0)
        {
          break;
        }

      new_head = RTS_PACK(RTS_CNT(old_head) + 1, RTS_IDX(old_head) + n);
    }
  while (!atomic_cmpxchg_relaxed(&r->prod_head, &old_head, new_head));

  ring_buf_init(buf, r, RTS_IDX(old_head), n, elem_size);
}

/****************************************************************************
 * Name: ring_sc_start
 *
 * Description:
 *   Single-consumer start. Calculate available items, initialize
 *   ring_buf_s.
 *
 ****************************************************************************/

static inline_function void
ring_sc_start(FAR struct ring_s *r, uint32_t count,
              size_t elem_size, FAR struct ring_buf_s *buf)
{
  uint32_t cons_index = RTS_IDX(RING_FIELD(&r->cons_tail));
  uint32_t prod_index;
  uint32_t available;
  uint32_t n;

  UP_RMB();
  prod_index = RTS_IDX(RING_FIELD(&r->prod_tail));
  available = (prod_index - cons_index) & RING_IDX_MASK;
  n = count < available ? count : available;

  ring_buf_init(buf, r, cons_index, n, elem_size);
}

/****************************************************************************
 * Name: ring_mc_start
 *
 * Description:
 *   Multi-consumer start. CAS on cons_head to reserve up to count
 *   slots, initialize ring_buf_s.
 *
 ****************************************************************************/

static inline_function void
ring_mc_start(FAR struct ring_s *r, uint32_t count,
              size_t elem_size, FAR struct ring_buf_s *buf)
{
  uint32_t old_head;
  uint32_t new_head;
  uint32_t n;

  old_head = atomic_read(&r->cons_head);
  do
    {
      uint32_t prod_index = RTS_IDX(atomic_read_acquire(&r->prod_tail));
      uint32_t available =
        (prod_index - RTS_IDX(old_head)) & RING_IDX_MASK;

      n = count < available ? count : available;
      if (n == 0)
        {
          break;
        }

      new_head = RTS_PACK(RTS_CNT(old_head) + 1, RTS_IDX(old_head) + n);
    }
  while (!atomic_cmpxchg_relaxed(&r->cons_head, &old_head, new_head));

  ring_buf_init(buf, r, RTS_IDX(old_head), n, elem_size);
}

/****************************************************************************
 * Name: ring_sp_finish
 *
 * Description:
 *   Single-producer finish. Advance prod_tail from ring_buf_s state.
 *
 ****************************************************************************/

static inline_function void
ring_sp_finish(FAR struct ring_s *r, FAR struct ring_buf_s *buf)
{
  if (buf->cnt != 0)
    {
      uint32_t prod_index = (buf->idx + buf->cnt) & RING_IDX_MASK;

      UP_WMB();
      RING_FIELD(&r->prod_tail) = RTS_PACK(0, prod_index);
    }
}

/****************************************************************************
 * Name: ring_mp_finish
 *
 * Description:
 *   Multi-producer tail finish. Increment cnt, and when tail.cnt
 *   catches up to head.cnt, advance tail.idx to head.idx.
 *
 ****************************************************************************/

static inline_function void
ring_mp_finish(FAR struct ring_s *r, FAR struct ring_buf_s *buf)
{
  uint32_t old_tail;
  uint32_t new_head;
  uint32_t new_tail;

  if (buf->cnt == 0)
    {
      return;
    }

  old_tail = atomic_read_acquire(&r->prod_tail);
  do
    {
      uint8_t cnt = RTS_CNT(old_tail) + 1;
      uint32_t idx = RTS_IDX(old_tail);

      new_head = atomic_read(&r->prod_head);
      if (cnt == RTS_CNT(new_head))
        {
          idx = RTS_IDX(new_head);
        }

      new_tail = RTS_PACK(cnt, idx);
    }
  while (!atomic_cmpxchg_release_acquire(&r->prod_tail, &old_tail,
                                         new_tail));
}

/****************************************************************************
 * Name: ring_sc_finish
 *
 * Description:
 *   Single-consumer finish. Advance cons_tail from ring_buf_s state.
 *
 ****************************************************************************/

static inline_function void
ring_sc_finish(FAR struct ring_s *r, FAR struct ring_buf_s *buf)
{
  if (buf->cnt != 0)
    {
      uint32_t cons_index = (buf->idx + buf->cnt) & RING_IDX_MASK;

      UP_WMB();
      RING_FIELD(&r->cons_tail) = RTS_PACK(0, cons_index);
    }
}

/****************************************************************************
 * Name: ring_sc_peek
 *
 * Description:
 *   Single-consumer peek with external index. Read data from ring
 *   without advancing cons_tail. The caller's index tracks the read
 *   position independently.
 *
 * Returned Value:
 *   Number of elements peeked.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sc_peek(FAR struct ring_s *r, FAR void *items, uint32_t count,
             size_t elem_size, FAR uint32_t *index)
{
  uint32_t cons_index;
  uint32_t prod_index;
  uint32_t available;
  uint32_t n;

  cons_index = RTS_IDX(RING_FIELD(&r->cons_tail));
  if (index)
    {
      cons_index = ring_clamp_cons(*index, cons_index);
    }

  UP_RMB();
  prod_index = RTS_IDX(RING_FIELD(&r->prod_tail));
  available = RTS_IDX(prod_index - cons_index);
  n = count < available ? count : available;

  if (items)
    {
      struct ring_buf_s buf;

      ring_buf_init(&buf, r, cons_index, n, elem_size);
      ring_copy_to(&buf, items, elem_size);
      cons_index = RTS_IDX(cons_index + n);
    }

  if (index)
    {
      *index = cons_index;
    }

  return n;
}

/****************************************************************************
 * Name: ring_mc_peek
 *
 * Description:
 *   Multi-consumer peek with external index and overwrite detection.
 *   Uses optimistic-read-then-verify to ensure data consistency when
 *   producers may overwrite the region being read.
 *
 * Returned Value:
 *   Number of elements peeked.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_mc_peek(FAR struct ring_s *r, FAR void *items, uint32_t count,
             size_t elem_size, FAR uint32_t *index)
{
  uint32_t cons_index;
  uint32_t n;

  do
    {
      struct ring_buf_s buf;
      uint32_t prod_index;
      uint32_t available;

      cons_index = RTS_IDX(atomic_read_acquire(&r->cons_tail));
      if (index)
        {
          cons_index = ring_clamp_cons(*index, cons_index);
        }

      prod_index = RTS_IDX(atomic_read_acquire(&r->prod_tail));
      available = RTS_IDX(prod_index - cons_index);
      n = count < available ? count : available;

      if (n == 0 || !items)
        {
          break;
        }

      ring_buf_init(&buf, r, cons_index, n, elem_size);
      ring_copy_to(&buf, items, elem_size);

      /* Verify data wasn't overwritten during copy:
       * If cons_tail advanced past our read position, retry.
       */
    }
  while (!ring_is_valid_cons(cons_index,
                             RTS_IDX(atomic_read_acquire(&r->cons_tail))));

  if (items)
    {
      cons_index = RTS_IDX(cons_index + n);
    }

  if (index)
    {
      *index = cons_index;
    }

  return n;
}

/****************************************************************************
 * Name: ring_mc_finish
 *
 * Description:
 *   Multi-consumer tail finish. Symmetric to ring_mp_finish,
 *   operating on cons_head/cons_tail.
 *
 ****************************************************************************/

static inline_function void
ring_mc_finish(FAR struct ring_s *r, FAR struct ring_buf_s *buf)
{
  uint32_t old_tail;
  uint32_t new_head;
  uint32_t new_tail;

  if (buf->cnt == 0)
    {
      return;
    }

  old_tail = atomic_read_acquire(&r->cons_tail);
  do
    {
      uint8_t cnt = RTS_CNT(old_tail) + 1;
      uint32_t idx = RTS_IDX(old_tail);

      new_head = atomic_read(&r->cons_head);
      if (cnt == RTS_CNT(new_head))
        {
          idx = RTS_IDX(new_head);
        }

      new_tail = RTS_PACK(cnt, idx);
    }
  while (!atomic_cmpxchg_release_acquire(&r->cons_tail, &old_tail,
                                         new_tail));
}

/****************************************************************************
 * Name: ring_sp_enqueue
 *
 * Description:
 *   Single-producer enqueue. No CAS, direct idx read/write.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sp_enqueue(FAR struct ring_s *r, FAR const void *items,
                uint32_t count, size_t elem_size)
{
  struct ring_buf_s buf;

  ring_sp_start(r, count, elem_size, &buf);
  ring_copy_from(&buf, items, elem_size);
  ring_sp_finish(r, &buf);

  return buf.cnt;
}

/****************************************************************************
 * Name: ring_mp_enqueue
 *
 * Description:
 *   Multi-producer enqueue. Uses RTS CAS on prod_head + prod_tail.
 *
 *   cnt overflow note: cnt is 8-bit (0~255). A false-positive
 *   match requires 256 concurrent in-flight producers - impossible
 *   on systems with < 256 cores. uint8_t wraps naturally and
 *   symmetrically on both head and tail sides, so no special
 *   overflow handling is needed.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_mp_enqueue(FAR struct ring_s *r, FAR const void *items,
                uint32_t count, size_t elem_size)
{
  struct ring_buf_s buf;

  ring_mp_start(r, count, elem_size, &buf);
  ring_copy_from(&buf, items, elem_size);
  ring_mp_finish(r, &buf);

  return buf.cnt;
}

/****************************************************************************
 * Name: ring_sc_dequeue
 *
 * Description:
 *   Single-consumer dequeue. No CAS, direct idx read/write.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sc_dequeue(FAR struct ring_s *r, FAR void *items,
                uint32_t count, size_t elem_size)
{
  struct ring_buf_s buf;

  ring_sc_start(r, count, elem_size, &buf);
  ring_copy_to(&buf, items, elem_size);
  ring_sc_finish(r, &buf);

  return buf.cnt;
}

/****************************************************************************
 * Name: ring_mc_dequeue
 *
 * Description:
 *   Multi-consumer dequeue. Uses RTS CAS on cons_head + cons_tail.
 *
 *   cnt overflow note: same as ring_mp_enqueue - 8-bit cnt wraps
 *   naturally, safe as long as concurrent consumers < 256.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_mc_dequeue(FAR struct ring_s *r, FAR void *items,
                uint32_t count, size_t elem_size)
{
  struct ring_buf_s buf;

  ring_mc_start(r, count, elem_size, &buf);
  ring_copy_to(&buf, items, elem_size);
  ring_mc_finish(r, &buf);

  return buf.cnt;
}

/****************************************************************************
 * Name: ring_sc_skip
 *
 * Description:
 *   Single-consumer skip. Advance cons_tail by count without
 *   reading data. Returns actual count skipped.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sc_skip(FAR struct ring_s *r, uint32_t count)
{
  struct ring_buf_s buf;

  ring_sc_start(r, count, 0, &buf);
  ring_sc_finish(r, &buf);

  return buf.cnt;
}

/****************************************************************************
 * Name: ring_sc_skipto
 *
 * Description:
 *   Single-consumer skip-to. Advance cons_tail to the specified target
 *   index. Computes the skip count from current cons_tail internally.
 *   If target is behind cons_tail (already consumed), does nothing.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sc_skipto(FAR struct ring_s *r, uint32_t target)
{
  uint32_t cons_index = RING_FIELD(&r->cons_tail);
  uint32_t count = RTS_IDX(target - cons_index);

  if (count != 0)
    {
      return ring_sc_skip(r, count);
    }

  return 0;
}

/****************************************************************************
 * Name: ring_mc_skip
 *
 * Description:
 *   Multi-consumer skip. Reserve and finish slots on
 *   cons_head and cons_tail without reading data.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_mc_skip(FAR struct ring_s *r, uint32_t count)
{
  struct ring_buf_s buf;

  ring_mc_start(r, count, 0, &buf);
  ring_mc_finish(r, &buf);

  return buf.cnt;
}

/****************************************************************************
 * Name: ring_mc_skipto
 *
 * Description:
 *   Multi-consumer skip-to. Advance cons_tail to the specified target
 *   index. This is a convenience wrapper that computes the skip count
 *   from the current cons_tail and then calls ring_mc_skip.
 *   If target is behind cons_tail (already consumed), does nothing.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_mc_skipto(FAR struct ring_s *r, uint32_t target)
{
  uint32_t cons_index = RTS_IDX(atomic_read(&r->cons_tail));
  uint32_t count = RTS_IDX(target - cons_index);

  if (count != 0)
    {
      return ring_mc_skip(r, count);
    }

  return 0;
}

/****************************************************************************
 * Name: ring_sp_overwrite_start
 *
 * Description:
 *   Single-producer overwrite start. Discard oldest committed data
 *   if needed, then initialize ring_buf_s for zero-copy write.
 *   Caller writes data into buf, then calls ring_sp_finish.
 *
 *   Returns the number of elements evicted from the ring to make room
 *   for the request; zero if the ring had enough free space.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sp_overwrite_start(FAR struct ring_s *r, uint32_t count,
                        size_t elem_size, FAR struct ring_buf_s *buf)
{
  uint32_t prod_idx;
  uint32_t cons_idx;
  uint32_t free_space;
  uint32_t discard = 0;
  uint32_t old_cons;
  uint32_t new_cons;

  if (count > r->capacity)
    {
      count = r->capacity;
    }

  prod_idx = RTS_IDX(atomic_read(&r->prod_tail));

  for (; ; )
    {
      old_cons = atomic_read_acquire(&r->cons_head);
      cons_idx = RTS_IDX(old_cons);
      free_space = r->capacity -
                   ((prod_idx - cons_idx) & RING_IDX_MASK);

      if (free_space >= count)
        {
          break;
        }

      discard = count - free_space;
      new_cons = RTS_PACK(RTS_CNT(old_cons) + 1, cons_idx + discard);
      if (atomic_cmpxchg_relaxed(&r->cons_head, &old_cons, new_cons))
        {
          ring_buf_init(buf, r, cons_idx, discard, elem_size);
          ring_mc_finish(r, buf);
          break;
        }
    }

  ring_buf_init(buf, r, prod_idx, count, elem_size);
  return discard;
}

/****************************************************************************
 * Name: ring_sp_overwrite
 *
 * Description:
 *   Single-producer overwrite (copy variant).  Returns the net
 *   number of elements added to the ring (count - discarded);
 *   when the ring was full, the count of evicted elements is subtracted
 *   from the inserted count so the caller sees the true delta in
 *   ring occupancy.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sp_overwrite(FAR struct ring_s *r, FAR const void *items,
                  uint32_t count, size_t elem_size)
{
  struct ring_buf_s buf;
  uint32_t discard;

  discard = ring_sp_overwrite_start(r, count, elem_size, &buf);
  ring_copy_from(&buf,
                 (FAR const char *)items + (count - buf.cnt) * elem_size,
                 elem_size);
  ring_sp_finish(r, &buf);

  return buf.cnt - discard;
}

/****************************************************************************
 * Name: ring_mp_overwrite_start
 *
 * Description:
 *   Multi-producer overwrite start. Discard oldest committed data
 *   if needed, reserve producer slots via CAS, then initialize
 *   ring_buf_s for zero-copy write. Caller writes data into buf,
 *   then calls ring_mp_finish.
 *
 *   Returns the total number of elements evicted from the ring across
 *   all retry rounds; zero if the ring had enough free space.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_mp_overwrite_start(FAR struct ring_s *r, uint32_t count,
                        size_t elem_size, FAR struct ring_buf_s *buf)
{
  uint32_t old_prod;
  uint32_t new_prod;
  uint32_t prod_idx;
  uint32_t cons_idx;
  uint32_t free_space;
  uint32_t discard;
  uint32_t all_discard = 0;
  uint32_t old_cons;
  uint32_t new_cons;

  if (count > r->capacity)
    {
      count = r->capacity;
    }

  old_prod = atomic_read(&r->prod_head);
  for (; ; )
    {
      uint32_t prod_tail_idx;

      old_cons = atomic_read_acquire(&r->cons_head);
      prod_tail_idx = RTS_IDX(atomic_read_acquire(&r->prod_tail));
      prod_idx = RTS_IDX(old_prod);
      cons_idx = RTS_IDX(old_cons);

      free_space = r->capacity -
                   ((prod_idx - cons_idx) & RING_IDX_MASK);

      if (free_space < count)
        {
          uint32_t committed;

          committed = (prod_tail_idx - cons_idx) & RING_IDX_MASK;
          discard = count - free_space;
          if (discard > committed)
            {
              old_prod = atomic_read(&r->prod_head);
              continue;
            }

          new_cons = RTS_PACK(RTS_CNT(old_cons) + 1, cons_idx + discard);
          if (!atomic_cmpxchg_relaxed(&r->cons_head, &old_cons, new_cons))
            {
              old_prod = atomic_read(&r->prod_head);
              continue;
            }

          ring_buf_init(buf, r, cons_idx, discard, elem_size);
          ring_mc_finish(r, buf);
          all_discard += discard;

          old_prod = atomic_read(&r->prod_head);
          prod_idx = RTS_IDX(old_prod);
        }

      new_prod = RTS_PACK(RTS_CNT(old_prod) + 1, prod_idx + count);
      if (atomic_cmpxchg_relaxed(&r->prod_head, &old_prod, new_prod))
        {
          break;
        }
    }

  ring_buf_init(buf, r, prod_idx, count, elem_size);
  return all_discard;
}

/****************************************************************************
 * Name: ring_mp_overwrite
 *
 * Description:
 *   Multi-producer overwrite (copy variant).  Returns the net
 *   number of elements added to the ring (count - discarded);
 *   when the ring was full, the count of evicted elements is subtracted
 *   from the inserted count so the caller sees the true delta in
 *   ring occupancy.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_mp_overwrite(FAR struct ring_s *r, FAR const void *items,
                  uint32_t count, size_t elem_size)
{
  struct ring_buf_s buf;
  uint32_t discard;

  discard = ring_mp_overwrite_start(r, count, elem_size, &buf);
  ring_copy_from(&buf,
                 (FAR const char *)items + (count - buf.cnt) * elem_size,
                 elem_size);
  ring_mp_finish(r, &buf);

  return buf.cnt - discard;
}

#endif /* __INCLUDE_NUTTX_RING_H */
