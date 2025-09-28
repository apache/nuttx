/****************************************************************************
 * libs/libc/misc/lib_backtrace.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>
#include <debug.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdbool.h>
#include <syslog.h>

#include <nuttx/init.h>
#include <nuttx/nuttx.h>
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
#  define backtrace_lock(bp) spin_lock_irqsave_notrace(&(bp)->lock)
#  define backtrace_unlock(bp, flags) spin_unlock_irqrestore_notrace(&(bp)->lock, flags)
#else
#  define backtrace_lock(bp) ((irqstate_t)nxmutex_lock(&(bp)->lock))
#  define backtrace_unlock(bp, flags) \
          ((void)flags, nxmutex_unlock(&(bp)->lock))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if CONFIG_LIBC_BACKTRACE_DEPTH > 0
struct backtrace_entry_s
{
  union
  {
    FAR void *stack[CONFIG_LIBC_BACKTRACE_DEPTH];
    struct sq_entry_s freenode;
  };

  FAR void *next;  /* Next index in the hash chain */
  uint32_t  depth; /* Depth of the backtrace */
  uint32_t  ref;   /* Count of the backtrace */
};

struct backtrace_pool_s
{
  FAR struct backtrace_entry_s **bucket;
  struct sq_queue_s freelist;
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  spinlock_t lock;
#else
  mutex_t lock;
#endif
  size_t capacity;
  size_t used;
  size_t overflow;
  bool expanding;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct backtrace_entry_s
g_backtrace_entries[CONFIG_LIBC_BACKTRACE_INIT_SIZE];
static FAR struct backtrace_entry_s *
g_backtrace_bucket[CONFIG_LIBC_BACKTRACE_INIT_SIZE];

static struct backtrace_pool_s g_backtrace_pool =
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  .lock = SP_UNLOCKED,
#else
  .lock = NXMUTEX_INITIALIZER,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR char **backtrace_malloc(FAR void *const *buffer, int size)
{
  size_t length = 0;

  if (size <= 0)
    {
      return NULL;
    }

  while (size-- > 0)
    {
      int ret = snprintf(NULL, 0, "%pS", *buffer++);

      if (ret < 0)
        {
          return NULL;
        }

      length += sizeof(FAR char *) + ret + 1;
    }

  return lib_malloc(length);
}

#if CONFIG_LIBC_BACKTRACE_DEPTH > 0

/****************************************************************************
 * Name: backtrace_hash
 ****************************************************************************/

/* nosanitize_address: backtrace_hash is on the hot path of every recorded
 * allocation/free (called by backtrace_record and backtrace_remove when
 * CONFIG_MM_RECORD_STACK is on).  On KASAN-enabled debug builds each
 * data[i] load would otherwise trip a per-access __asan_load4_noabort
 * shadow check; multiplied across a heavy allocation phase (e.g. C++
 * static initialization) this stretches the wall time past the test
 * framework's busy-loop threshold and surfaces as a no-output hang.  The
 * same reasoning was applied to backtrace_find(); this covers the
 * remaining uninstrumented reader on the same path.  The buffer is either
 * the caller's stack array (filled by sched_backtrace) or a pool entry's
 * stack array, both always valid, so skipping the shadow check is safe.
 */

nosanitize_address
static uint32_t backtrace_hash(FAR const void *backtrace, int depth)
{
  FAR const uintptr_t *data = backtrace;
  uint32_t hash = 5381;
  int i;

  for (i = 0; i < depth; i++)
    {
      hash = ((hash << 5) + hash) + data[i];
    }

  return hash;
}

/****************************************************************************
 * Name: backtrace_rehash
 ****************************************************************************/

#if CONFIG_LIBC_BACKTRACE_LOAD_FACTOR < 100
static void backtrace_rehash(FAR struct backtrace_pool_s *bp,
                             FAR struct backtrace_entry_s **bucket)
{
  FAR struct backtrace_entry_s *entry;
  size_t capacity = bp->capacity;
  size_t i;

  /* Rehash the backtrace record, and move the backtrace record to the new
   * bucket
   */

  for (i = 0; i < capacity; i++)
    {
      entry = bp->bucket[i];
      while (entry)
        {
          uint32_t slot = backtrace_hash(entry->stack, entry->depth);
          FAR struct backtrace_entry_s *next = entry->next;

          slot = slot % (capacity * 2);
          entry->next = bucket[slot];
          bucket[slot] = entry;
          entry = next;
        }
    }
}
#endif /* CONFIG_LIBC_BACKTRACE_LOAD_FACTOR < 100 */

/****************************************************************************
 * Name: backtrace_expand
 *
 * Description:
 *   Expand the backtrace pool. The pool lock is held on entry and is held on
 *   return. The lock is released around the sleeping allocation path so that
 *   a second thread recording a backtrace can still make progress: it skips
 *   expansion because bp->expanding is true and falls back to the remaining
 *   headroom in the current pool.
 *
 * Input Parameters:
 *   bp    - The backtrace pool
 *   flags - The irqstate_t saved by the caller when the lock was acquired;
 *           updated in place whenever the lock is released and reacquired
 *           inside this function.
 ****************************************************************************/

static int backtrace_expand(FAR struct backtrace_pool_s *bp,
                            FAR irqstate_t *flags)
{
  size_t i;

  /* If the capacity is 0, it means that the backtrace record is not
   * initialized, we will initialize it with the static pool.
   */

  if (bp->capacity == 0)
    {
      /* No allocation is needed for the static pool, so the lock is held
       * throughout the initialization to make bp->capacity visible to any
       * waiting thread before it observes the pool.
       */

      sq_init(&bp->freelist);
      for (i = 0; i < CONFIG_LIBC_BACKTRACE_INIT_SIZE; i++)
        {
          sq_addlast(&g_backtrace_entries[i].freenode, &bp->freelist);
        }

      bp->bucket = g_backtrace_bucket;
      bp->capacity = CONFIG_LIBC_BACKTRACE_INIT_SIZE;
      return 0;
    }

#if CONFIG_LIBC_BACKTRACE_LOAD_FACTOR < 100
  {
    /* When expanding, double the capacity of the bucket and release
     * the old bucket.
     */

    FAR struct backtrace_entry_s **oldbucket;
    FAR struct backtrace_entry_s **bucket;
    FAR struct backtrace_entry_s *pool;
    size_t poolsize = bp->capacity;
    size_t capacity = bp->capacity * 2;

    /* Drop the lock before the sleeping allocation so we neither deadlock
     * on the pool spinlock nor block IRQs across heap allocation.
     */

    backtrace_unlock(bp, *flags);
    bucket = lib_zalloc(sizeof(FAR struct backtrace_entry_s *) * capacity);
    pool = lib_zalloc(sizeof(struct backtrace_entry_s) * poolsize);
    if (bucket == NULL || pool == NULL)
      {
        lib_free(bucket);
        lib_free(pool);
        *flags = backtrace_lock(bp);
        return -ENOMEM;
      }

    *flags = backtrace_lock(bp);
    backtrace_rehash(bp, bucket);

    for (i = 0; i < poolsize; i++)
      {
        sq_addlast(&pool[i].freenode, &bp->freelist);
      }

    oldbucket = bp->bucket;
    bp->bucket = bucket;
    bp->capacity = capacity;
    backtrace_unlock(bp, *flags);

    if (oldbucket != g_backtrace_bucket)
      {
        lib_free(oldbucket);
      }

    *flags = backtrace_lock(bp);
  }
#else
  (void)flags;
#endif /* CONFIG_LIBC_BACKTRACE_LOAD_FACTOR < 100 */

  return 0;
}

/****************************************************************************
 * Name: backtrace_exist
 ****************************************************************************/

nosanitize_address no_builtin("memcmp")
static FAR struct backtrace_entry_s *
backtrace_find(FAR struct backtrace_pool_s *bp, uint32_t slot,
               FAR const void *stack, int depth)
{
  FAR struct backtrace_entry_s *entry = bp->bucket[slot];
  FAR const uintptr_t *s1 = (FAR const uintptr_t *)stack;
  int i;

  while (entry != NULL)
    {
      if (entry->depth == depth)
        {
          FAR const uintptr_t *s2 = (FAR const uintptr_t *)entry->stack;

          /* Inline word-by-word compare to avoid the per-byte KASAN shadow
           * checks inside memcmp().  This function is on the hot path of
           * every recorded allocation when CONFIG_MM_RECORD_STACK is on; on
           * KASAN-enabled debug builds the per-byte instrumentation in
           * memcmp() multiplies the cost of each lookup and can stretch
           * heavy allocation phases (e.g. romfs_cachenode during mount of a
           * large partition) past the test framework's busy-loop threshold.
           * The stack array stores word-aligned function pointers, so word
           * comparison is semantically equivalent to memcmp().
           *
           * The no_builtin("memcmp") attribute is required because GCC's
           * -ftree-loop-distribute-patterns optimization (enabled at -O2)
           * recognizes this loop as a memcmp pattern and rewrites it back
           * into a memcmp() call, defeating the nosanitize_address intent
           * (the called memcmp is still KASAN-instrumented per byte read).
           */

          for (i = 0; i < depth; i++)
            {
              if (s1[i] != s2[i])
                {
                  break;
                }
            }

          if (i == depth)
            {
              /* If the backtrace record already exists, just increase the
               * reference count
               */

              entry->ref++;
              return entry;
            }
        }

      entry = entry->next;
    }

  return NULL;
}

/****************************************************************************
 * Name: backtrace_alloc
 ****************************************************************************/

nosanitize_address no_builtin("memcpy")
static FAR struct backtrace_entry_s *
backtrace_alloc(FAR struct backtrace_pool_s *bp, uint32_t slot,
                const FAR void *stack, int depth)
{
  FAR struct backtrace_entry_s *entry;
  FAR const uintptr_t *src = (FAR const uintptr_t *)stack;
  FAR uintptr_t *dst;
  int i;

  /* Get the first entry from the free list */

  entry = (FAR struct backtrace_entry_s *)sq_remfirst(&bp->freelist);
  if (entry == NULL)
    {
      bp->overflow++;
      return NULL;
    }

  entry = container_of(entry, struct backtrace_entry_s, freenode);
  DEBUGASSERT(entry->ref == 0);

  /* Copy the backtrace to the entry */

  bp->used++;

  /* Inline word-by-word copy to avoid the per-byte KASAN shadow checks
   * inside memcpy().  This function is on the hot path of every recorded
   * allocation when CONFIG_MM_RECORD_STACK is on; on KASAN-enabled debug
   * builds the per-byte instrumentation in memcpy() multiplies the cost
   * of each insertion and can stretch heavy allocation phases (e.g. WAMR
   * AOT module loading) past the test framework's busy-loop threshold.
   * The stack array stores word-aligned function pointers, so word
   * copying is semantically equivalent to memcpy().
   */

  dst = (FAR uintptr_t *)entry->stack;
  for (i = 0; i < depth; i++)
    {
      dst[i] = src[i];
    }

  entry->depth = depth;
  entry->ref = 1;

  /* Insert backtrace entry to the head of the linked list */

  entry->next = bp->bucket[slot];
  bp->bucket[slot] = entry;
  return entry;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: backtrace_record
 *
 * Description:
 *   Record the backtrace of the current task
 *
 * Returned Value:
 *   Return the index of the backtrace record if success, otherwise return
 *   a negative value.
 ****************************************************************************/

FAR void *backtrace_record(int skip)
{
  FAR void *buffer[CONFIG_LIBC_BACKTRACE_DEPTH];
  FAR struct backtrace_pool_s *bp = &g_backtrace_pool;
  FAR struct backtrace_entry_s *entry = NULL;
  irqstate_t flags;
  uint32_t slot;
  int depth;

  depth = sched_backtrace(_SCHED_GETTID(), buffer,
                          CONFIG_LIBC_BACKTRACE_DEPTH, skip);
  if (depth <= 0)
    {
      return NULL;
    }

  slot = backtrace_hash(buffer, depth);
  flags = backtrace_lock(bp);

  /* If the used amount reaches the threshold, we will expand it */

  if (bp->used >= bp->capacity * CONFIG_LIBC_BACKTRACE_LOAD_FACTOR / 100 &&
      !bp->expanding
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
      && OSINIT_MM_READY() && !up_interrupt_context() &&
      (!is_idle_task(nxsched_self()) || !OSINIT_IDLELOOP())
#endif
     )
    {
      bp->expanding = true;
      backtrace_expand(bp, &flags);
      bp->expanding = false;
    }

  /* Backtrace can only be recorded after initialization */

  if (bp->capacity)
    {
      slot = slot % bp->capacity;
      entry = backtrace_find(bp, slot, buffer, depth);
      if (entry == NULL)
        {
          /* If the backtrace record does not exist, allocate a new one */

          entry = backtrace_alloc(bp, slot, buffer, depth);
        }
    }

  backtrace_unlock(bp, flags);
  return entry;
}

/****************************************************************************
 * Name: backtrace_remove
 *
 * Description:
 *   Remove the backtrace record
 *
 * Input Parameters:
 *   index - The index of the backtrace record
 *
 * Returned Value:
 *   Return 0 if success, otherwise return a negative value.
 ****************************************************************************/

/* nosanitize_address: backtrace_remove is on the hot path of every
 * allocation/free (mempool_record calls it for each when
 * CONFIG_MM_RECORD_STACK is on).  On KASAN-enabled debug builds the loads
 * of entry->ref, the bucket slot and the collision chain each trip a
 * per-access shadow check; across a heavy allocation phase (e.g. recursive
 * directory scan at boot) this pushes wall time past the test framework's
 * busy-loop threshold.  All pointers touched are pool-internal, so skipping
 * the check is safe -- same reasoning as backtrace_hash()/backtrace_find();
 * this is the remaining uninstrumented accessor on the same path.
 */

nosanitize_address
void backtrace_remove(FAR void *index)
{
  FAR struct backtrace_pool_s *bp = &g_backtrace_pool;
  FAR struct backtrace_entry_s *entry = index;
  irqstate_t flags;
  uint32_t slot;

  if (entry == NULL)
    {
      return;
    }

  flags = backtrace_lock(bp);
  DEBUGASSERT(entry->ref > 0);
  entry->ref--;
  if (entry->ref > 0)
    {
      backtrace_unlock(bp, flags);
      return;
    }

  slot = backtrace_hash(entry->stack, entry->depth);
  slot = slot % bp->capacity;
  if (bp->bucket[slot] == entry)
    {
      bp->bucket[slot] = entry->next;
    }
  else
    {
      /* Remove the backtrace record from the linked list */

      FAR struct backtrace_entry_s *prev = bp->bucket[slot];

      while (prev)
        {
          if (prev->next == entry)
            {
              prev->next = entry->next;
              break;
            }

          prev = prev->next;
        }

      /* If the backtrace record is not found, it means that the backtrace
       * record entry is an invalid value.
       */

      DEBUGASSERT(prev != NULL);
    }

  bp->used--;
  sq_addlast(&entry->freenode, &bp->freelist);
  backtrace_unlock(bp, flags);
}

/****************************************************************************
 * Name: backtrace_get
 *
 * Description:
 *   Find the backtrace record by index
 *
 * Input Parameters:
 *   index - The index of the backtrace record
 *   size  - The size of the backtrace record
 *
 * Returned Value:
 *   Return the backtrace record if success, otherwise return NULL
 ****************************************************************************/

FAR void **backtrace_get(FAR void *index, FAR int *size)
{
  FAR struct backtrace_entry_s *entry = index;

  if (size == NULL || entry == NULL)
    {
      return NULL;
    }

  *size = entry->depth;
  return entry->stack;
}

void backtrace_dump(void)
{
  char buf[BACKTRACE_BUFFER_SIZE(CONFIG_LIBC_BACKTRACE_DEPTH)];
  FAR struct backtrace_pool_s *bp = &g_backtrace_pool;
  FAR struct backtrace_entry_s *entry;
  size_t conflict = 0;
  size_t slot;

  syslog(LOG_INFO, "%-8s %-8s %s", "slot", "refcount", "backtrace");
  for (slot = 0; slot < bp->capacity; slot++)
    {
      entry = bp->bucket[slot];
      while (entry)
        {
          backtrace_format(buf, sizeof(buf), entry->stack, entry->depth);
          syslog(LOG_INFO, "%-8zu %-8d %s\n", slot, (int)entry->ref, buf);
          conflict += entry->next != NULL;
          entry = entry->next;
        }
    }

  syslog(LOG_INFO,
         "capacity: %zu, used: %zu, conflict: %zu, overflow: %zu\n",
         bp->capacity, bp->used, conflict, bp->overflow);
}
#endif /* CONFIG_LIBC_BACKTRACE_DEPTH > 0 */

FAR char **backtrace_symbols(FAR void *const *buffer, int size)
{
  FAR char **syms;
  FAR char *buf;
  int i;

  syms = backtrace_malloc(buffer, size);
  if (syms != NULL)
    {
      buf = (FAR char *)&syms[size];
      for (i = 0; i < size; i++)
        {
          syms[i] = buf;
          buf += sprintf(buf, "%pS", buffer[i]);
          buf += 1;
        }
    }

  return syms;
}

void backtrace_symbols_fd(FAR void *const *buffer, int size, int fd)
{
  int i;

  for (i = 0; i < size; i++)
    {
      dprintf(fd, "%pS\n", buffer[i]);
    }
}

/****************************************************************************
 * Name: backtrace_format
 *
 * Description:
 *  Format a backtrace into a buffer for dumping.
 *
 ****************************************************************************/

nosanitize_address
int backtrace_format(FAR char *buffer, int size,
                     FAR void *backtrace[], int depth)
{
  FAR const char *format = "%0*p ";
  int len = 0;
  int i;

  if (buffer == NULL || backtrace == NULL || size < 1)
    {
      return 0;
    }

  buffer[0] = '\0';
  for (i = 0; i < depth && backtrace[i]; i++)
    {
      if ((i + 1) * BACKTRACE_PTR_FMT_WIDTH >= size)
        {
          break;
        }

      len += snprintf(buffer + i * BACKTRACE_PTR_FMT_WIDTH,
                      size - i * BACKTRACE_PTR_FMT_WIDTH,
                      format, BACKTRACE_PTR_FMT_WIDTH - 1, backtrace[i]);
    }

  return len;
}
