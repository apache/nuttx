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

/* RING_INIT - static initialize a ring
 *
 * cap   - capacity (must be power-of-2)
 *
 * Usage:
 *   RING_DECLARE(g_my_ring, uint32_t, 64) = RING_INIT(64);
 */

#define RING_INIT(cap)                                   \
  {                                                      \
    .base.capacity  = cap,                               \
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

#define ring_init(r, cap)                                \
  do                                                     \
    {                                                    \
      RING_BASE(r)->capacity = (cap);                    \
      ring_reset(r);                                     \
    }                                                    \
  while (0)

#define ring_count(r)                                    \
  ((RTS_IDX(atomic_read(&RING_BASE(r)->prod_tail)) -     \
    RTS_IDX(atomic_read(&RING_BASE(r)->cons_tail))) &    \
   RING_IDX_MASK)

#define ring_capacity(r)    (RING_BASE(r)->capacity)

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

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aligned_data(CACHELINE_SIZE) ring_s
{
  /* Producer side - cache-line aligned */

  atomic_t     prod_head;         /* SP: unused; MP: RTS CAS */
  atomic_t     prod_tail;         /* SP: store_release; MP: RTS CAS */
  uint8_t      pad0[CACHELINE_SIZE - sizeof(atomic_t) * 2];

  /* Consumer side - cache-line aligned */

  atomic_t     cons_head;         /* SC: unused; MC: RTS CAS */
  atomic_t     cons_tail;         /* SC: store_release; MC: RTS CAS */
  uint8_t      pad1[CACHELINE_SIZE - sizeof(atomic_t) * 2];

  /* Read-only metadata - cache-line aligned */

  uint32_t     capacity;          /* Power-of-2, max 16777215 */
  char         ring[];            /* Flexible array for embedded buffer */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

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
 * Name: ring_copy_from
 *
 * Description:
 *   Copy elements into the ring buffer, handling wrap-around.
 *
 ****************************************************************************/

static inline_function void
ring_copy_from(FAR struct ring_s *r, uint32_t idx, FAR const void *items,
               uint32_t n, size_t elem_size)
{
  FAR char *dst;

  idx &= r->capacity - 1;
  dst = (FAR char *)r->ring + idx * elem_size;

  /* Fast path: single element. When elem_size is a compile-time
   * constant, the compiler emits a single store.
   */

  if (n == 1)
    {
      if (elem_size == sizeof(uint8_t))
        {
          *(FAR uint8_t *)dst = *(FAR const uint8_t *)items;
        }
      else if (elem_size == sizeof(uint16_t))
        {
          *(FAR uint16_t *)dst = *(FAR const uint16_t *)items;
        }
      else if (elem_size == sizeof(uint32_t))
        {
          *(FAR uint32_t *)dst = *(FAR const uint32_t *)items;
        }
      else if (elem_size == sizeof(uint64_t))
        {
          *(FAR uint64_t *)dst = *(FAR const uint64_t *)items;
        }
      else
        {
          memcpy(dst, items, elem_size);
        }
    }
  else
    {
      uint32_t first = r->capacity - idx;

      if (first > n)
        {
          first = n;
        }

      memcpy(dst, items, first * elem_size);
      if (n > first)
        {
          memcpy(r->ring, (FAR const char *)items + first * elem_size,
                 (n - first) * elem_size);
        }
    }
}

/****************************************************************************
 * Name: ring_copy_to
 *
 * Description:
 *   Copy elements from the ring buffer to items, handling wrap-around.
 *
 ****************************************************************************/

static inline_function void
ring_copy_to(FAR struct ring_s *r, uint32_t idx, FAR void *items,
             uint32_t n, size_t elem_size)
{
  FAR const char *src;

  idx &= r->capacity - 1;
  src = (FAR const char *)r->ring + idx * elem_size;

  /* Fast path: single element copy */

  if (n == 1)
    {
      if (elem_size == sizeof(uint8_t))
        {
          *(FAR uint8_t *)items = *(FAR const uint8_t *)src;
        }
      else if (elem_size == sizeof(uint16_t))
        {
          *(FAR uint16_t *)items = *(FAR const uint16_t *)src;
        }
      else if (elem_size == sizeof(uint32_t))
        {
          *(FAR uint32_t *)items = *(FAR const uint32_t *)src;
        }
      else if (elem_size == sizeof(uint64_t))
        {
          *(FAR uint64_t *)items = *(FAR const uint64_t *)src;
        }
      else
        {
          memcpy(items, src, elem_size);
        }
    }
  else
    {
      uint32_t first = r->capacity - idx;

      if (first > n)
        {
          first = n;
        }

      memcpy(items, src, first * elem_size);
      if (n > first)
        {
          memcpy((FAR char *)items + first * elem_size, r->ring,
                 (n - first) * elem_size);
        }
    }
}

/****************************************************************************
 * Name: ring_sp_enqueue
 *
 * Description:
 *   Single-producer enqueue. No CAS, direct idx read/write.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sp_enqueue(FAR struct ring_s *r, FAR const void *items, uint32_t count,
                size_t elem_size)
{
  uint32_t prod_index = RTS_IDX(atomic_read(&r->prod_tail));
  uint32_t cons_index = RTS_IDX(atomic_read_acquire(&r->cons_tail));
  uint32_t free_space = r->capacity -
                        ((prod_index - cons_index) & RING_IDX_MASK);
  uint32_t n = count < free_space ? count : free_space;

  if (n != 0)
    {
      ring_copy_from(r, prod_index, items, n, elem_size);

      prod_index = (prod_index + n) & RING_IDX_MASK;
      atomic_set_release(&r->prod_tail, RTS_PACK(0, prod_index));
    }

  return n;
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
ring_mp_enqueue(FAR struct ring_s *r, FAR const void *items, uint32_t count,
                size_t elem_size)
{
  uint32_t old_head;
  uint32_t new_head;
  uint32_t old_tail;
  uint32_t new_tail;
  uint32_t n;

  /* RTS CAS on prod_head to reserve slots */

  old_head = atomic_read(&r->prod_head);
  do
    {
      uint32_t cons_index = RTS_IDX(atomic_read_acquire(&r->cons_tail));
      uint32_t free_space = r->capacity -
        ((RTS_IDX(old_head) - cons_index) & RING_IDX_MASK);

      n = count < free_space ? count : free_space;
      if (n == 0)
        {
          return 0;
        }

      new_head = RTS_PACK(RTS_CNT(old_head) + 1, RTS_IDX(old_head) + n);
    }
  while (!atomic_cmpxchg_relaxed(&r->prod_head, &old_head, new_head));

  ring_copy_from(r, RTS_IDX(old_head), items, n, elem_size);

  /* RTS tail commit: increment cnt, and when tail.cnt catches
   * up to the *current* head.cnt (all in-flight threads done),
   * advance tail.idx to head.idx so consumers can see the
   * committed data. Re-read prod_head each iteration - other
   * threads may have reserved new slots since our CAS on head.
   */

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

  return n;
}

/****************************************************************************
 * Name: ring_sc_dequeue
 *
 * Description:
 *   Single-consumer dequeue. No CAS, direct idx read/write.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sc_dequeue(FAR struct ring_s *r, FAR void *items, uint32_t count,
                size_t elem_size)
{
  uint32_t cons_index = RTS_IDX(atomic_read(&r->cons_tail));
  uint32_t prod_index = RTS_IDX(atomic_read_acquire(&r->prod_tail));
  uint32_t available = (prod_index - cons_index) & RING_IDX_MASK;
  uint32_t n = count < available ? count : available;

  if (n != 0)
    {
      ring_copy_to(r, cons_index, items, n, elem_size);

      cons_index = (cons_index + n) & RING_IDX_MASK;
      atomic_set_release(&r->cons_tail, RTS_PACK(0, cons_index));
    }

  return n;
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
ring_mc_dequeue(FAR struct ring_s *r, FAR void *items, uint32_t count,
                size_t elem_size)
{
  uint32_t old_head;
  uint32_t new_head;
  uint32_t old_tail;
  uint32_t new_tail;
  uint32_t n;

  /* RTS CAS on cons_head to reserve slots */

  old_head = atomic_read(&r->cons_head);
  do
    {
      uint32_t prod_index = RTS_IDX(atomic_read_acquire(&r->prod_tail));
      uint32_t available = (prod_index - RTS_IDX(old_head)) & RING_IDX_MASK;

      n = count < available ? count : available;
      if (n == 0)
        {
          return 0;
        }

      new_head = RTS_PACK(RTS_CNT(old_head) + 1, RTS_IDX(old_head) + n);
    }
  while (!atomic_cmpxchg_relaxed(&r->cons_head, &old_head, new_head));

  ring_copy_to(r, RTS_IDX(old_head), items, n, elem_size);

  /* RTS tail commit: symmetric to ring_mp_enqueue.
   * Re-read cons_head each iteration.
   */

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

  return n;
}

#endif /* __INCLUDE_NUTTX_RING_H */
