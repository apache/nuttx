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
 */

#ifndef __INCLUDE_NUTTX_RING_H
#define __INCLUDE_NUTTX_RING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
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
 * cap   - capacity (must be power-of-2)
 *
 * Usage:
 *   RING_DECLARE(g_my_ring, uint32_t, 64) = RING_INIT(64);
 */

#define RING_INIT(cap)                                   \
  {                                                      \
    .base.magic    = RING_MAGIC_READY,                   \
    .base.capacity = cap,                                \
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

#define RING_DECLARE(name, type, cap)                    \
  static_assert((cap) > 0 && ((cap) & ((cap) - 1)) == 0, \
                "ring cap must be power-of-2");          \
  struct ring_##name##_s                                 \
  {                                                      \
    struct ring_s base;                                  \
    type          buf[cap];                              \
  } name

/* RING_DEFINE - define and initialise a ring (no init call needed).
 *
 * name  - variable name
 * type  - element type
 * cap   - capacity (must be power-of-2)
 *
 * Usage:
 *   RING_DEFINE(g_my_ring, uint32_t, 64);
 *   ring_spsc_enqueue(&my_ring, &val, 1);
 */

#define RING_DEFINE(name, type, cap)                     \
  RING_DECLARE(name, type, cap) = RING_INIT(cap)

/* RING_BASE - convert a RING_DECLARE/RING_DEFINE variable to
 * struct ring_s pointer for use with internal APIs.
 */

#define RING_BASE(ring)       ((FAR struct ring_s *)(ring))

/* Public API macros - accept RING_DECLARE/RING_DEFINE structs */

#define ring_reset(r)                                    \
  do                                                     \
    {                                                    \
      atomic_set(&RING_BASE(r)->prod_head, 0);           \
      atomic_set(&RING_BASE(r)->prod_tail, 0);           \
      atomic_set(&RING_BASE(r)->cons_head, 0);           \
      atomic_set(&RING_BASE(r)->cons_tail, 0);           \
    }                                                    \
  while (0)

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

#define ring_init(r, cap)   ring_init_magic(RING_BASE(r), cap)

#define ring_count(r)                                    \
  ((RTS_IDX(atomic_read(&RING_BASE(r)->prod_tail)) -     \
    RTS_IDX(atomic_read(&RING_BASE(r)->cons_tail))) &    \
   RING_IDX_MASK)

#define ring_capacity(r)    (RING_BASE(r)->capacity)
#define ring_space(r)       (ring_capacity(r) - ring_count(r))

#define ring_spsc_enqueue(r, items, count) \
  ring_sp_enqueue(RING_BASE(r), items, count, sizeof(*(items)))

#define ring_spsc_dequeue(r, items, count) \
  ring_sc_dequeue(RING_BASE(r), items, count, sizeof(*(items)))

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

#define ring_spsc_dequeue_skip(r, count) \
  ring_sc_dequeue_skip(RING_BASE(r), count)

#define ring_mpmc_dequeue_skip(r, count) \
  ring_mc_dequeue_skip(RING_BASE(r), count)

#define ring_spmc_dequeue_skip(r, count) \
  ring_mc_dequeue_skip(RING_BASE(r), count)

#define ring_mpsc_dequeue_skip(r, count) \
  ring_sc_dequeue_skip(RING_BASE(r), count)

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

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aligned_data(CACHELINE_SIZE) ring_s
{
  /* Read-only metadata - cache-line aligned */

  atomic_t     magic;             /* Warm-restart detection */
  uint32_t     capacity;          /* Power-of-2, max 16777215 */
  uint8_t      pad0[CACHELINE_SIZE - sizeof(atomic_t) -
                    sizeof(uint32_t) - sizeof(size_t)];

  /* Producer side - cache-line aligned */

  atomic_t     prod_head;         /* SP: unused; MP: RTS CAS */
  atomic_t     prod_tail;         /* SP: store_release; MP: RTS CAS */
  uint8_t      pad1[CACHELINE_SIZE - sizeof(atomic_t) * 2];

  /* Consumer side - cache-line aligned */

  atomic_t     cons_head;         /* SC: unused; MC: RTS CAS */
  atomic_t     cons_tail;         /* SC: store_release; MC: RTS CAS */
  uint8_t      pad2[CACHELINE_SIZE - sizeof(atomic_t) * 2];

  /* Flexible array for embedded buffer */

  char         ring[];            /* Flexible array for embedded buffer */
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

static inline_function void ring_init_magic(FAR struct ring_s *r,
                                            uint32_t cap)
{
  irqstate_t flags;
  int32_t old_magic;

  flags = irq_save();

  do
    {
      old_magic = atomic_read(&r->magic);

      if (old_magic == RING_MAGIC_INIT)
        {
          irq_restore(flags);

          /* Another core is initializing - spin until READY */

          while (atomic_read_acquire(&r->magic) != RING_MAGIC_READY);

          return;
        }
    }
  while (!atomic_cmpxchg(&r->magic, &old_magic, RING_MAGIC_INIT));

  r->capacity = cap;

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
  irq_restore(flags);
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

  if (cap == 0 || (cap & (cap - 1)) != 0)
    {
      return NULL;
    }

  r = lib_malloc(sizeof(*r) + cap * elem_size);
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
      return;
    }

  buf->idx = idx;

  idx   = idx & (r->capacity - 1);
  first = r->capacity - idx;

  if (first >= count)
    {
      buf->ptr1 = (FAR char *)r->ring + idx * elem_size;
      buf->len1 = count;
      buf->ptr2 = NULL;
      buf->len2 = 0;
    }
  else
    {
      buf->ptr1 = (FAR char *)r->ring + idx * elem_size;
      buf->len1 = first;
      buf->ptr2 = r->ring;
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
  else
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
  else
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
  uint32_t prod_index = RTS_IDX(atomic_read(&r->prod_tail));
  uint32_t cons_index = RTS_IDX(atomic_read_acquire(&r->cons_tail));
  uint32_t free_space = r->capacity -
                        ((prod_index - cons_index) & RING_IDX_MASK);
  uint32_t n = count < free_space ? count : free_space;

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
  uint32_t cons_index = RTS_IDX(atomic_read(&r->cons_tail));
  uint32_t prod_index = RTS_IDX(atomic_read_acquire(&r->prod_tail));
  uint32_t available = (prod_index - cons_index) & RING_IDX_MASK;
  uint32_t n = count < available ? count : available;

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
      atomic_set_release(&r->prod_tail, RTS_PACK(0, prod_index));
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
      atomic_set_release(&r->cons_tail, RTS_PACK(0, cons_index));
    }
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
 * Name: ring_sc_dequeue_skip
 *
 * Description:
 *   Single-consumer dequeue skip. Advance cons_tail by count without
 *   reading data. Returns actual count skipped.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sc_dequeue_skip(FAR struct ring_s *r, uint32_t count)
{
  struct ring_buf_s buf;

  ring_sc_start(r, count, 0, &buf);
  ring_sc_finish(r, &buf);

  return buf.cnt;
}

/****************************************************************************
 * Name: ring_mc_dequeue_skip
 *
 * Description:
 *   Multi-consumer dequeue skip. Reserve and finish slots on
 *   cons_head and cons_tail without reading data.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_mc_dequeue_skip(FAR struct ring_s *r, uint32_t count)
{
  struct ring_buf_s buf;

  ring_mc_start(r, count, 0, &buf);
  ring_mc_finish(r, &buf);

  return buf.cnt;
}

#endif /* __INCLUDE_NUTTX_RING_H */
