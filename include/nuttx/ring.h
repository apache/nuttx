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
 *   Declare the blob and the handle as plain variables; storage, alignment
 *   and section are the caller's to choose.  Neither takes an initializer -
 *   init fills the handle in, and a located or shared blob would not get one
 *   loaded anyway.  The size given is the whole blob, its header included,
 *   so the usable capacity is what is left after that header:
 *     static char g_blob[4096] aligned_data(RING_BLOB_ALIGN);
 *     static struct ring_s g_ring;
 *
 *   Init takes the blob and its total size, and derives the geometry.  The
 *   multi-core calls also take a rewind count, replaying that many
 *   already-consumed elements across a warm restart, 0 to disable:
 *     ring_mpmc_init(&g_ring, g_blob, sizeof(g_blob), 0);  // SMP/BMP, CAS
 *     ring_mpsc_init(&g_ring, g_blob, sizeof(g_blob), 0);  // alias of mpmc
 *     ring_spmc_init(&g_ring, g_blob, sizeof(g_blob), 0);  // alias of mpmc
 *     ring_spsc_init(&g_ring, g_blob, sizeof(g_blob), 0);  // AMP, no atomic
 *
 *   Under SPSC each side inits its own handle on the shared blob; the one
 *   that did not reset it polls ring_is_ready(&g_ring) before using it.
 *
 *   Dynamic - one allocation holds handle and blob, already initialized:
 *     struct ring_s *r = ring_alloc(256);
 *     ring_free(r);
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
#include <nuttx/nuttx.h>

#include <assert.h>
#include <string.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 24-bit index mask */

#define RING_IDX_MASK    0xffffff

/* Sentinel *index value for the peek helpers: a real index never exceeds
 * RTS_IDX_MASK (24 bits), so this out-of-range value asks the peek to start
 * from the oldest readable position (the live cons_tail) instead of clamping
 * a caller-supplied index.
 */

#define RING_IDX_OLDEST  UINT32_MAX

/* Maximum ring capacity (fits in 24-bit index space) */

#define RING_SIZE_MAX    0x1000000

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

#ifndef RING_BLOB_ALIGN
#  define RING_BLOB_ALIGN 64
#endif

/* Magic values for warm-restart detection.
 *
 * When a ring resides in memory that survives reboot (e.g. shared
 * SRAM), ring_mpmc_init() uses an atomic magic field to decide whether
 * to perform a fresh initialization or recover the previous state.
 *
 * Three-state protocol (multi-core safe):
 *   0                     -> uninitialized (or garbage != READY/INIT)
 *   RING_BLOB_MAGIC_INIT  -> one core is initializing; others spin-wait
 *   RING_BLOB_MAGIC_READY -> init complete; ring is usable
 */

#define RING_BLOB_MAGIC_INIT   ((int32_t)0x52494e49)   /* "RINI" */
#define RING_BLOB_MAGIC_READY  ((int32_t)0x52494e47)   /* "RING" */

/* Handle magic values, stored in struct ring_s::magic.  Separate from the
 * blob magic above because the handle lives in ordinary RAM cleared on every
 * reboot - hence "boot-scoped", with RING_MAGIC_UNINIT being that zero.
 */

#define RING_MAGIC_UNINIT     0   /* Zero-init: no caller yet this boot   */
#define RING_MAGIC_INIT       1   /* A caller is running the init body    */
#define RING_MAGIC_READY      2   /* Init complete; ring usable this boot */

/* Public API macros - accept a struct ring_s * handle */

#define ring_mpsc_init(r, buf, buflen, rewind) ring_mpmc_init(r, buf, buflen, rewind)
#define ring_spmc_init(r, buf, buflen, rewind) ring_mpmc_init(r, buf, buflen, rewind)

#define ring_is_ready(r) ((r)->magic == RING_MAGIC_READY)
#define ring_count(r)    ring_dist(r, RTS_IDX((r)->blob->prod_tail), \
                                   RTS_IDX((r)->blob->cons_tail))
#define ring_capacity(r) ((r)->blob->capacity)
#define ring_space(r)    (ring_capacity(r) - ring_count(r))

#define ring_spsc_enqueue(r, items, count) \
  (ring_is_ready(r) ? \
   ring_sp_enqueue(r, items, count, sizeof(*(items))) : 0)

#define ring_spsc_dequeue(r, items, count) \
  (ring_is_ready(r) ? \
   ring_sc_dequeue(r, items, count, sizeof(*(items))) : 0)

#define ring_spsc_skip(r, count) \
  (ring_is_ready(r) ? ring_sc_skip(r, count) : 0)

#define ring_mpmc_enqueue(r, items, count) \
  ring_mp_enqueue(r, items, count, sizeof(*(items)))

#define ring_mpmc_dequeue(r, items, count) \
  ring_mc_dequeue(r, items, count, sizeof(*(items)))

#define ring_spmc_enqueue(r, items, count) \
  ring_sp_enqueue(r, items, count, sizeof(*(items)))

#define ring_spmc_dequeue(r, items, count) \
  ring_mc_dequeue(r, items, count, sizeof(*(items)))

#define ring_mpsc_enqueue(r, items, count) \
  ring_mp_enqueue(r, items, count, sizeof(*(items)))

#define ring_mpsc_dequeue(r, items, count) \
  ring_sc_dequeue(r, items, count, sizeof(*(items)))

#define ring_mpmc_skip(r, count) ring_mc_skip(r, count)
#define ring_spmc_skip(r, count) ring_mc_skip(r, count)
#define ring_mpsc_skip(r, count) ring_sc_skip(r, count)

#define ring_mpsc_skipto(r, target) ring_sc_skipto(r, target)
#define ring_spsc_skipto(r, target) ring_sc_skipto(r, target)
#define ring_spmc_skipto(r, target) ring_mc_skipto(r, target)
#define ring_mpmc_skipto(r, target) ring_mc_skipto(r, target)

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
  ring_sp_overwrite(r, items, count, sizeof(*(items)))

#define ring_mpmc_overwrite(r, items, count) \
  ring_mp_overwrite(r, items, count, sizeof(*(items)))

#define ring_spmc_overwrite_start(r, count, elem_size, b) \
  ring_sp_overwrite_start(r, count, elem_size, b)

#define ring_spmc_overwrite_finish(r, b) ring_sp_finish(r, b)

#define ring_mpmc_overwrite_start(r, count, elem_size, b) \
  ring_mp_overwrite_start(r, count, elem_size, b)

#define ring_mpmc_overwrite_finish(r, b) ring_mp_finish(r, b)

#define ring_spsc_enqueue_start(r, count, elem_size, b) \
  ring_sp_start(r, count, elem_size, b)

#define ring_spsc_enqueue_finish(r, b) ring_sp_finish(r, b)

#define ring_spsc_dequeue_start(r, count, elem_size, b) \
  ring_sc_start(r, count, elem_size, b)

#define ring_spsc_dequeue_finish(r, b) ring_sc_finish(r, b)

#define ring_mpmc_enqueue_start(r, count, elem_size, b) \
  ring_mp_start(r, count, elem_size, b)

#define ring_mpmc_enqueue_finish(r, b) ring_mp_finish(r, b)

#define ring_mpmc_dequeue_start(r, count, elem_size, b) \
  ring_mc_start(r, count, elem_size, b)

#define ring_mpmc_dequeue_finish(r, b) ring_mc_finish(r, b)

#define ring_spmc_enqueue_start(r, count, elem_size, b) \
  ring_sp_start(r, count, elem_size, b)

#define ring_spmc_enqueue_finish(r, b) ring_sp_finish(r, b)

#define ring_spmc_dequeue_start(r, count, elem_size, b) \
  ring_mc_start(r, count, elem_size, b)

#define ring_spmc_dequeue_finish(r, b) ring_mc_finish(r, b)

#define ring_mpsc_enqueue_start(r, count, elem_size, b) \
  ring_mp_start(r, count, elem_size, b)

#define ring_mpsc_enqueue_finish(r, b) ring_mp_finish(r, b)

#define ring_mpsc_dequeue_start(r, count, elem_size, b) \
  ring_sc_start(r, count, elem_size, b)

#define ring_mpsc_dequeue_finish(r, b) ring_sc_finish(r, b)

#define ring_mpsc_peek(r, items, count, index) \
  ring_sc_peek(r, items, count, sizeof(*(items)), index)

#define ring_spsc_peek(r, items, count, index) \
  ring_sc_peek(r, items, count, sizeof(*(items)), index)

#define ring_mpmc_peek(r, items, count, index) \
  ring_mc_peek(r, items, count, sizeof(*(items)), index)

#define ring_spmc_peek(r, items, count, index) \
  ring_mc_peek(r, items, count, sizeof(*(items)), index)

/* Peek at a fixed offset from the oldest element (cons_tail), without
 * advancing.  Unlike the external-index peek above, the read position is
 * expressed relative to the consumer tail, so the caller never handles
 * absolute ring indices.
 */

#define ring_mpsc_peek_at(r, items, count, offset) \
  ring_sc_peek_at(r, items, count, sizeof(*(items)), offset)

#define ring_spsc_peek_at(r, items, count, offset) \
  ring_sc_peek_at(r, items, count, sizeof(*(items)), offset)

#define ring_mpmc_peek_at(r, items, count, offset) \
  ring_mc_peek_at(r, items, count, sizeof(*(items)), offset)

#define ring_spmc_peek_at(r, items, count, offset) \
  ring_mc_peek_at(r, items, count, sizeof(*(items)), offset)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* struct ring_blob_s - the blob header.
 *
 * The state living in the (possibly cross-core) ring memory: magic and
 * geometry, plus the producer and consumer indices, each on its own cache
 * line, followed immediately by the element data region, the data[] member.
 * Callers reach it only through a struct ring_s handle.
 *
 * The geometry is kept here rather than in the handle so that the ring
 * length can be recovered from the shared memory alone - by a peer image, a
 * debugger or a crash dump - without access to any handle.  The init call
 * writes it, deriving it from buflen.
 *
 * NOTE: cache-line isolation comes from the explicit pad0/pad1/pad2 arrays,
 * and aligned_data(RING_BLOB_ALIGN) is applied at each definition site
 * rather than to the type: binding it to the type makes some compilers (e.g.
 * TASKING) propagate the alignment to every member, inflating sizeof() and
 * breaking the shared memory layout.
 */

struct ring_blob_s
{
  /* Read-only metadata - cache-line aligned */

  atomic_t     magic;             /* Warm-restart detection */
  uint32_t     capacity;          /* Data capacity in bytes (<= 16777215) */
  uint32_t     wrappoint;         /* Largest multiple of capacity <= 2^24 */
  uint8_t      pad0[RING_BLOB_ALIGN - sizeof(atomic_t) -
                    sizeof(uint32_t) * 2];

  /* Producer side - cache-line aligned */

  atomic_t     prod_head;         /* SP: unused; MP: RTS CAS */
  atomic_t     prod_tail;         /* SP: store_release; MP: RTS CAS */
  uint8_t      pad1[RING_BLOB_ALIGN - sizeof(atomic_t) * 2];

  /* Consumer side - cache-line aligned */

  atomic_t     cons_head;         /* SC: unused; MC: RTS CAS */
  atomic_t     cons_tail;         /* SC: store_release; MC: RTS CAS */
  uint8_t      pad2[RING_BLOB_ALIGN - sizeof(atomic_t) * 2];

  /* Element data region, capacity bytes of it. */

  uint8_t      data[];
};

/* struct ring_s - lightweight ring handle.
 *
 * Callers hold this handle rather than the blob header itself.  Keeping it
 * separate lets one blob be reached from several images / owners, each with
 * its own handle in ordinary memory.
 *
 *   magic    - boot-scoped tri-state init guard, also the "initialized this
 *              boot" flag.  Living in reboot-cleared memory is what makes
 *              it boot-scoped, unlike the blob magic which may survive a
 *              reboot.
 *   blob     - the blob header this handle operates on.  The geometry lives
 *              there, not here, so every owner of the blob sees one copy.
 */

struct ring_s
{
  atomic_t                magic;    /* Boot-scoped init guard (tri-state) */
  FAR struct ring_blob_s *blob;     /* Ring blob (header + data) */
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
 *   Convert a logical index to a physical position in the blob data. Uses
 *   bitmask when capacity is power-of-2, modulo otherwise.
 *
 ****************************************************************************/

static inline_function uint32_t ring_idx(FAR struct ring_s *r,
                                         uint32_t idx)
{
  FAR struct ring_blob_s *blob = r->blob;

  return blob->wrappoint == RING_SIZE_MAX ?
         (idx & (blob->capacity - 1)) : (idx % blob->capacity);
}

/****************************************************************************
 * Name: ring_advance
 *
 * Description:
 *   Advance a logical index by n, wrapping at the ring's wrap period.
 *   Power-of-2 rings use the fast 24-bit mask; others wrap modulo the
 *   wrap period so the physical mapping stays continuous.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_advance(FAR struct ring_s *r, uint32_t idx, uint32_t n)
{
  uint32_t wrappoint = r->blob->wrappoint;

  return wrappoint == RING_SIZE_MAX ? ((idx + n) & RING_IDX_MASK) :
                                      ((idx + n) % wrappoint);
}

/****************************************************************************
 * Name: ring_dist
 *
 * Description:
 *   Compute the forward distance from b to a in logical index space,
 *   respecting the ring's wrap period. Power-of-2 rings use the fast
 *   24-bit mask; others reduce modulo the wrap period.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_dist(FAR struct ring_s *r, uint32_t a, uint32_t b)
{
  uint32_t wrappoint = r->blob->wrappoint;

  return wrappoint == RING_SIZE_MAX ? ((a - b) & RING_IDX_MASK) :
                                      ((a + wrappoint - b) % wrappoint);
}

/****************************************************************************
 * Name: ring_is_valid_cons
 *
 * Description:
 *   Check whether cons_index is still live (not lapped by the writer).
 *   Power-of-2 rings shift bit 23 to the 32-bit sign bit so the signed
 *   compare works modulo 2^24. Non-power-of-2 rings treat an index within
 *   the first half of the wrap window (measured from cons_tail) as live.
 *
 ****************************************************************************/

static inline_function bool
ring_is_valid_cons(FAR struct ring_s *r, uint32_t cons_index,
                   uint32_t cons_tail)
{
  uint32_t wrappoint = r->blob->wrappoint;

  return wrappoint == RING_SIZE_MAX ?
         ((int32_t)((cons_index - cons_tail) << 8) >= 0) :
         (ring_dist(r, cons_index, cons_tail) < (wrappoint >> 1));
}

/****************************************************************************
 * Name: ring_clamp_cons
 *
 * Description:
 *   Clamp an external index if it fell behind cons_tail (overwritten).
 *   Returns the effective read position.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_clamp_cons(FAR struct ring_s *r, uint32_t cons_index,
                uint32_t cons_tail)
{
  return ring_is_valid_cons(r, cons_index, cons_tail) ?
         cons_index : cons_tail;
}

/****************************************************************************
 * Name: ring_reset
 *
 * Description:
 *   Clear all indices and return the count of elements that were present
 *   before the reset (i.e. the number of elements discarded by the reset
 *   operation).
 *
 *   Callers may either use the return value (e.g. to bump a "dropped"
 *   counter) or discard it.  Safe to call on a freshly-zeroed ring whose
 *   indices have not been written yet: reading the volatile fields yields
 *   whatever happens to be in memory and the caller is expected to discard
 *   the value in that case.
 *
 ****************************************************************************/

static inline_function uint32_t ring_reset(FAR struct ring_s *r)
{
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t dropped = ring_count(r);

  if (dropped > blob->capacity)
    {
      dropped = blob->capacity;
    }

  blob->prod_head = 0;
  blob->prod_tail = 0;
  blob->cons_head = 0;
  blob->cons_tail = 0;

  return dropped;
}

/****************************************************************************
 * Name: ring_spsc_init
 *
 * Description:
 *   Lightweight initialization for SPSC mode.
 *
 *   Unlike ring_mpmc_init (which uses atomic CAS for multi-core safe
 *   concurrent initialization), ring_spsc_init uses only volatile writes.
 *   It can be called by either the producer or consumer side, but NOT
 *   concurrently from both - the caller must ensure only one side calls it
 *   first.  The other side then calls it too, to bind its own handle, and
 *   polls ring_is_ready() before touching the ring: the reset is guarded by
 *   the blob magic, so a later call only binds and stamps the same geometry.
 *
 *   Use ring_spsc_init for AMP/SHM scenarios where atomic instructions are
 *   unavailable on shared memory.  Use ring_mpmc_init for SMP/BMP scenarios
 *   where multiple cores may race to initialize the same ring.
 *
 *   buf    - the blob to operate on
 *   buflen - total blob size in bytes (header + data)
 *   rewind - elements to replay on a warm restart, 0 for none
 *
 * Returned Value:
 *   true if this caller ran the init body (the first one this boot), false
 *   if the blob was already initialized by another caller.  The ring is
 *   usable either way.
 *
 ****************************************************************************/

static inline_function bool
ring_spsc_init(FAR struct ring_s *r, FAR void *buf, uint32_t buflen,
               uint32_t rewind)
{
  FAR struct ring_blob_s *blob = buf;
  uint32_t cap = buflen - sizeof(*blob);
  irqstate_t flags;
  bool first;

  DEBUGASSERT(buf != NULL && cap <= RING_SIZE_MAX &&
              buflen > sizeof(*blob) &&
              IS_ALIGNED((uintptr_t)buf, RING_BLOB_ALIGN));

  if (ring_is_ready(r))
    {
      return false;
    }

  flags = irq_save();

  r->blob = blob;

  first = blob->magic != RING_BLOB_MAGIC_READY || blob->capacity != cap;
  if (first)
    {
      blob->magic = RING_BLOB_MAGIC_INIT;
      blob->capacity = cap;
      blob->wrappoint = ALIGN_DOWN(RING_SIZE_MAX, cap);
      ring_reset(r);
      UP_WMB();
      blob->magic = RING_BLOB_MAGIC_READY;
    }
  else if (rewind != 0)
    {
      uint32_t prod_index = RTS_IDX(blob->prod_tail);
      uint32_t cons_index = RTS_IDX(blob->cons_tail);
      uint32_t count = ring_dist(r, prod_index, cons_index);

      if (count > cap)
        {
          ring_reset(r);
        }
      else
        {
          if (rewind > cap - count)
            {
              rewind = cap - count;
            }

          cons_index = ring_dist(r, cons_index, rewind);
          blob->cons_tail = RTS_PACK(0, cons_index);
        }
    }

  UP_WMB();
  r->magic = RING_MAGIC_READY;
  irq_restore(flags);
  return first;
}

/****************************************************************************
 * Name: ring_mpmc_init
 *
 * Description:
 *   Runtime initialization with warm-restart recovery, safe against
 *   concurrent callers.  The unified entry point for all multi-core rings;
 *   ring_mpsc_init / ring_spmc_init alias it, SPSC rings use ring_spsc_init.
 *
 *   Binds the handle to the blob, stamps the blob geometry derived from
 *   buflen, then seeds the indices - recovering the previous contents on a
 *   warm restart - and marks the handle initialized for this boot.
 *
 *   Warm restart (blob magic already READY) keeps the indices where the
 *   previous run left them, so whatever had been produced but not consumed
 *   is still readable.  Only the RTS thread counters are cleared: no thread
 *   from the previous run is still in flight, and a stale non-zero counter
 *   would make every later CAS wait for a producer/consumer that no longer
 *   exists.
 *
 *   rewind additionally replays elements the previous run had already
 *   consumed, by pulling cons_tail back that many elements.  It is a count
 *   of elements, not bytes, and it is clamped to the free space
 *   (capacity - count): the live data occupies count slots and the whole
 *   remaining area is exactly the window of already-consumed-but-still-
 *   present data.  It only makes sense for a blob that survives reboot (a
 *   located .noinit section or shared SRAM); pass 0 otherwise.  A cold
 *   start has nothing to replay, and a blob with corrupted indices
 *   (count > capacity) is reset - neither path applies rewind.
 *
 *   buf    - the blob to operate on
 *   buflen - total blob size in bytes (header + data)
 *   rewind - elements of already-consumed data to replay on a warm restart,
 *            0 to disable
 *
 * Returned Value:
 *   true if this caller ran the init body (the first one this boot), false
 *   if another caller had already done it and this one only waited for
 *   completion.  The ring is usable either way.
 *
 ****************************************************************************/

static inline_function bool
ring_mpmc_init(FAR struct ring_s *r, FAR void *buf, uint32_t buflen,
               uint32_t rewind)
{
  FAR struct ring_blob_s *blob = buf;
  uint32_t cap = buflen - sizeof(*blob);
  int32_t old_magic;
  irqstate_t flags;
  bool first;

  DEBUGASSERT(buf != NULL && cap <= RING_SIZE_MAX &&
              buflen > sizeof(*blob) &&
              IS_ALIGNED((uintptr_t)buf, RING_BLOB_ALIGN));

  if (atomic_read(&r->magic) == RING_MAGIC_READY)
    {
      return false;
    }

  flags = irq_save();

  do
    {
      old_magic = atomic_read(&blob->magic);

      if (old_magic == RING_BLOB_MAGIC_INIT)
        {
          irq_restore(flags);

          /* Another core is initializing - spin until READY */

          while (atomic_read_acquire(&blob->magic) != RING_BLOB_MAGIC_READY)
            {
              UP_WFE();
            }

          r->blob = blob;
          atomic_set(&r->magic, RING_MAGIC_READY);
          return false;
        }
    }
  while (!atomic_cmpxchg(&blob->magic, &old_magic, RING_BLOB_MAGIC_INIT));

  /* Stamp the geometry before anything that depends on it (ring_dist,
   * ring_reset).  A persisted blob is only reusable for a warm restart
   * when it was left READY by a build with identical geometry; stale
   * residue from another firmware or a resized buffer is discarded.
   */

  r->blob = blob;

  first = old_magic != RING_BLOB_MAGIC_READY || blob->capacity != cap;
  if (first)
    {
      blob->capacity = cap;
      blob->wrappoint = ALIGN_DOWN(RING_SIZE_MAX, cap);
      ring_reset(r);
    }
  else
    {
      uint32_t prod_index = RTS_IDX(atomic_read(&blob->prod_tail));
      uint32_t cons_index = RTS_IDX(atomic_read(&blob->cons_tail));
      uint32_t count = ring_dist(r, prod_index, cons_index);

      if (count > cap)
        {
          ring_reset(r);
        }
      else
        {
          uint32_t prod_tail;
          uint32_t cons_tail;

          if (rewind > cap - count)
            {
              rewind = cap - count;
            }

          cons_index = ring_dist(r, cons_index, rewind);

          prod_tail = RTS_PACK(0, prod_index);
          cons_tail = RTS_PACK(0, cons_index);

          atomic_set(&blob->prod_tail, prod_tail);
          atomic_set(&blob->prod_head, prod_tail);
          atomic_set(&blob->cons_tail, cons_tail);
          atomic_set(&blob->cons_head, cons_tail);
        }
    }

  atomic_set_release(&blob->magic, RING_BLOB_MAGIC_READY);
  atomic_set(&r->magic, RING_MAGIC_READY);
  UP_SEV();
  irq_restore(flags);
  return first;
}

/****************************************************************************
 * Name: ring_alloc
 *
 * Description:
 *   Dynamically allocate a ring descriptor + buffer.
 *
 ****************************************************************************/

static inline_function FAR struct ring_s *ring_alloc(size_t buflen)
{
  FAR struct ring_blob_s *blob;
  FAR struct ring_s *r;

  if (buflen == 0)
    {
      return NULL;
    }

  r = lib_memalign(RING_BLOB_ALIGN, RING_BLOB_ALIGN + buflen);
  if (r == NULL)
    {
      return NULL;
    }

  blob = (FAR struct ring_blob_s *)((FAR char *)r + RING_BLOB_ALIGN);

  /* Fresh memory holds garbage, and init reads the blob magic to tell a cold
   * start from a warm restart; clear it so this one is seen as cold.
   */

  blob->magic = ~RING_BLOB_MAGIC_INIT;
  r->magic = RING_MAGIC_UNINIT;
  ring_spsc_init(r, blob, buflen, 0);
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
  FAR struct ring_blob_s *blob = r->blob;
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
  first = blob->capacity - idx;

  if (first >= count)
    {
      buf->ptr1 = blob->data + idx * elem_size;
      buf->len1 = count;
      buf->ptr2 = NULL;
      buf->len2 = 0;
    }
  else
    {
      buf->ptr1 = blob->data + idx * elem_size;
      buf->len1 = first;
      buf->ptr2 = blob->data;
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
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t prod_index = RTS_IDX(blob->prod_tail);
  uint32_t cons_index;
  uint32_t free_space;
  uint32_t n;

  UP_RMB();
  cons_index = RTS_IDX(blob->cons_tail);
  free_space = blob->capacity - ring_dist(r, prod_index, cons_index);
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
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t old_head;
  uint32_t new_head;
  uint32_t n;

  old_head = atomic_read(&blob->prod_head);
  do
    {
      uint32_t cons_index = RTS_IDX(atomic_read_acquire(&blob->cons_tail));
      uint32_t free_space = blob->capacity -
        ring_dist(r, RTS_IDX(old_head), cons_index);

      n = count < free_space ? count : free_space;
      if (n == 0)
        {
          break;
        }

      new_head = RTS_PACK(RTS_CNT(old_head) + 1,
                          ring_advance(r, RTS_IDX(old_head), n));
    }
  while (!atomic_cmpxchg_relaxed(&blob->prod_head, &old_head, new_head));

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
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t cons_index = RTS_IDX(blob->cons_tail);
  uint32_t prod_index;
  uint32_t available;
  uint32_t n = 0;

  UP_RMB();
  prod_index = RTS_IDX(blob->prod_tail);
  available = ring_dist(r, prod_index, cons_index);

  /* available > capacity is physically impossible for a consistent ring;
   * it can only result from a torn snapshot where prod_tail is stale and
   * the consumer indices were advanced past it by an overwrite eviction.
   * Treat it as no data this round; the next pass re-reads a coherent
   * snapshot without losing entries.
   */

  if (available <= blob->capacity)
    {
      n = count < available ? count : available;
    }

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
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t old_head;
  uint32_t new_head;
  uint32_t n;

  old_head = atomic_read(&blob->cons_head);
  do
    {
      uint32_t prod_index = RTS_IDX(atomic_read_acquire(&blob->prod_tail));
      uint32_t available = ring_dist(r, prod_index, RTS_IDX(old_head));

      /* available > capacity is physically impossible for a consistent
       * ring; it can only result from a torn snapshot where prod_tail is
       * stale and the consumer indices were advanced past it by an
       * overwrite eviction. Treat it as no data this round; the next pass
       * re-reads a coherent snapshot without losing entries.
       */

      if (available > blob->capacity)
        {
          n = 0;
          break;
        }

      n = count < available ? count : available;
      if (n == 0)
        {
          break;
        }

      new_head = RTS_PACK(RTS_CNT(old_head) + 1,
                          ring_advance(r, RTS_IDX(old_head), n));
    }
  while (!atomic_cmpxchg_relaxed(&blob->cons_head, &old_head, new_head));

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
  FAR struct ring_blob_s *blob = r->blob;

  if (buf->cnt != 0)
    {
      uint32_t prod_index = ring_advance(r, buf->idx, buf->cnt);

      UP_WMB();
      blob->prod_tail = RTS_PACK(0, prod_index);
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
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t old_tail;
  uint32_t new_head;
  uint32_t new_tail;

  if (buf->cnt == 0)
    {
      return;
    }

  old_tail = atomic_read_acquire(&blob->prod_tail);
  do
    {
      uint8_t cnt = RTS_CNT(old_tail) + 1;
      uint32_t idx = RTS_IDX(old_tail);

      new_head = atomic_read(&blob->prod_head);
      if (cnt == RTS_CNT(new_head))
        {
          idx = RTS_IDX(new_head);
        }

      new_tail = RTS_PACK(cnt, idx);
    }
  while (!atomic_cmpxchg_release_acquire(&blob->prod_tail, &old_tail,
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
  FAR struct ring_blob_s *blob = r->blob;

  if (buf->cnt != 0)
    {
      uint32_t cons_index = ring_advance(r, buf->idx, buf->cnt);

      UP_WMB();
      blob->cons_tail = RTS_PACK(0, cons_index);
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
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t cons_index;
  uint32_t prod_index;
  uint32_t available;
  uint32_t n;

  cons_index = RTS_IDX(blob->cons_tail);
  if (index)
    {
      cons_index = ring_clamp_cons(r, *index, cons_index);
    }

  UP_RMB();
  prod_index = RTS_IDX(blob->prod_tail);
  available = ring_dist(r, prod_index, cons_index);
  n = count < available ? count : available;

  if (items)
    {
      struct ring_buf_s buf;

      ring_buf_init(&buf, r, cons_index, n, elem_size);
      ring_copy_to(&buf, items, elem_size);
      cons_index = ring_advance(r, cons_index, n);
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
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t cons_index;
  uint32_t n;

  do
    {
      struct ring_buf_s buf;
      uint32_t prod_index;
      uint32_t available;

      cons_index = RTS_IDX(atomic_read_acquire(&blob->cons_tail));
      if (index && *index != RING_IDX_OLDEST)
        {
          cons_index = ring_clamp_cons(r, *index, cons_index);
        }

      prod_index = RTS_IDX(atomic_read_acquire(&blob->prod_tail));
      available = ring_dist(r, prod_index, cons_index);
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
  while (!ring_is_valid_cons(r, cons_index,
             RTS_IDX(atomic_read_acquire(&blob->cons_tail))));

  if (items)
    {
      cons_index = ring_advance(r, cons_index, n);
    }

  if (index)
    {
      *index = cons_index;
    }

  return n;
}

/****************************************************************************
 * Name: ring_sc_peek_at
 *
 * Description:
 *   Single-consumer peek at a byte/element offset measured from the oldest
 *   element (cons_tail), without advancing.  The caller works purely in the
 *   relative offset space and never sees an absolute ring index.
 *
 * Returned Value:
 *   Number of elements peeked (0 if offset is out of range).
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sc_peek_at(FAR struct ring_s *r, FAR void *items, uint32_t count,
                size_t elem_size, uint32_t offset)
{
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t index;

  if (offset >= ring_count(r))
    {
      return 0;
    }

  index = ring_advance(r, offset, RTS_IDX(blob->cons_tail));
  return ring_sc_peek(r, items, count, elem_size, &index);
}

/****************************************************************************
 * Name: ring_mc_peek_at
 *
 * Description:
 *   Multi-consumer counterpart of ring_sc_peek_at.  Peeks at an offset from
 *   cons_tail with the same overwrite-safe semantics as ring_mc_peek.
 *
 * Returned Value:
 *   Number of elements peeked (0 if offset is out of range).
 *
 ****************************************************************************/

static inline_function uint32_t
ring_mc_peek_at(FAR struct ring_s *r, FAR void *items, uint32_t count,
                size_t elem_size, uint32_t offset)
{
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t index;

  if (offset >= ring_count(r))
    {
      return 0;
    }

  index = ring_advance(r, offset,
                       RTS_IDX(atomic_read_acquire(&blob->cons_tail)));
  return ring_mc_peek(r, items, count, elem_size, &index);
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
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t old_tail;
  uint32_t new_head;
  uint32_t new_tail;

  if (buf->cnt == 0)
    {
      return;
    }

  old_tail = atomic_read_acquire(&blob->cons_tail);
  do
    {
      uint8_t cnt = RTS_CNT(old_tail) + 1;
      uint32_t idx = RTS_IDX(old_tail);

      new_head = atomic_read(&blob->cons_head);
      if (cnt == RTS_CNT(new_head))
        {
          idx = RTS_IDX(new_head);
        }

      new_tail = RTS_PACK(cnt, idx);
    }
  while (!atomic_cmpxchg_release_acquire(&blob->cons_tail, &old_tail,
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
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t cons_index = RTS_IDX(blob->cons_tail);
  uint32_t count = ring_dist(r, target, cons_index);

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
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t cons_index = RTS_IDX(atomic_read(&blob->cons_tail));
  uint32_t count = ring_dist(r, target, cons_index);

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
 *   Returns the number of elements that did not stay in the ring (the
 *   request beyond capacity plus evicted old data); zero otherwise.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_sp_overwrite_start(FAR struct ring_s *r, uint32_t count,
                        size_t elem_size, FAR struct ring_buf_s *buf)
{
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t prod_idx;
  uint32_t cons_idx;
  uint32_t free_space;
  uint32_t discard;
  uint32_t all_discard = 0;
  uint32_t old_cons;
  uint32_t new_cons;

  if (count > blob->capacity)
    {
      all_discard = count - blob->capacity;
      count = blob->capacity;
    }

  prod_idx = RTS_IDX(atomic_read(&blob->prod_tail));

  for (; ; )
    {
      old_cons = atomic_read_acquire(&blob->cons_head);
      cons_idx = RTS_IDX(old_cons);
      free_space = blob->capacity - ring_dist(r, prod_idx, cons_idx);

      if (free_space >= count)
        {
          break;
        }

      discard = count - free_space;
      new_cons = RTS_PACK(RTS_CNT(old_cons) + 1,
                          ring_advance(r, cons_idx, discard));
      if (atomic_cmpxchg_relaxed(&blob->cons_head, &old_cons, new_cons))
        {
          ring_buf_init(buf, r, cons_idx, discard, elem_size);
          ring_mc_finish(r, buf);
          all_discard += discard;
          break;
        }
    }

  ring_buf_init(buf, r, prod_idx, count, elem_size);
  return all_discard;
}

/****************************************************************************
 * Name: ring_sp_overwrite
 *
 * Description:
 *   Single-producer overwrite (copy variant).  Returns the number of
 *   elements that did not stay in the ring (the request beyond capacity
 *   plus evicted old data); zero when the request fit.
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

  return discard;
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
 *   Returns the number of elements that did not stay in the ring (the
 *   request beyond capacity plus evicted old data); zero otherwise.
 *
 ****************************************************************************/

static inline_function uint32_t
ring_mp_overwrite_start(FAR struct ring_s *r, uint32_t count,
                        size_t elem_size, FAR struct ring_buf_s *buf)
{
  FAR struct ring_blob_s *blob = r->blob;
  uint32_t old_prod;
  uint32_t new_prod;
  uint32_t prod_idx;
  uint32_t cons_idx;
  uint32_t free_space;
  uint32_t discard;
  uint32_t all_discard = 0;
  uint32_t old_cons;
  uint32_t new_cons;

  if (count > blob->capacity)
    {
      all_discard = count - blob->capacity;
      count = blob->capacity;
    }

  if (count == 0)
    {
      /* Nothing to reserve.  Bumping prod_head.cnt here without a
       * matching prod_tail.cnt bump (ring_mp_finish() returns early on
       * buf->cnt == 0) would leave a permanent cnt mismatch that
       * deadlocks the RTS tail sync.  Return an empty buf instead.
       */

      ring_buf_init(buf, r, 0, 0, elem_size);
      return all_discard;
    }

  old_prod = atomic_read(&blob->prod_head);
  for (; ; )
    {
      uint32_t prod_tail_idx;

      old_cons = atomic_read_acquire(&blob->cons_head);
      prod_tail_idx = RTS_IDX(atomic_read_acquire(&blob->prod_tail));
      prod_idx = RTS_IDX(old_prod);
      cons_idx = RTS_IDX(old_cons);

      free_space = ring_dist(r, prod_idx, cons_idx);
      free_space = free_space < blob->capacity ?
                   blob->capacity - free_space : 0;

      if (free_space < count)
        {
          uint32_t committed;

          committed = ring_dist(r, prod_tail_idx, cons_idx);
          discard = count - free_space;
          if (discard > committed)
            {
              old_prod = atomic_read(&blob->prod_head);
              continue;
            }

          new_cons = RTS_PACK(RTS_CNT(old_cons) + 1,
                              ring_advance(r, cons_idx, discard));
          if (!atomic_cmpxchg_relaxed(&blob->cons_head, &old_cons, new_cons))
            {
              old_prod = atomic_read(&blob->prod_head);
              continue;
            }

          ring_buf_init(buf, r, cons_idx, discard, elem_size);
          ring_mc_finish(r, buf);
          all_discard += discard;
        }

      new_prod = RTS_PACK(RTS_CNT(old_prod) + 1,
                          ring_advance(r, prod_idx, count));
      if (atomic_cmpxchg_relaxed(&blob->prod_head, &old_prod, new_prod))
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
 *   Multi-producer overwrite (copy variant).  Returns the number of
 *   elements that did not stay in the ring (the request beyond capacity
 *   plus evicted old data); zero when the request fit.
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

  return discard;
}

#endif /* __INCLUDE_NUTTX_RING_H */
