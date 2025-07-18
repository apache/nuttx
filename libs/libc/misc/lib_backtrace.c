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
#  define backtrace_lock(bp) spin_lock_irqsave(&(bp)->lock)
#  define backtrace_unlock(bp, flags) spin_unlock_irqrestore(&(bp)->lock, flags)
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
  uint16_t  depth; /* Depth of the backtrace */
  uint16_t  ref;   /* Count of the backtrace */
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
  bool expanding;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct backtrace_pool_s g_backtrace_pool;
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

/****************************************************************************
 * Name: backtrace_expand
 ****************************************************************************/

static int backtrace_expand(FAR struct backtrace_pool_s *bp)
{
  FAR struct backtrace_entry_s **oldbucket;
  FAR struct backtrace_entry_s **bucket;
  FAR struct backtrace_entry_s *pool;
  size_t capacity = bp->capacity;
  size_t poolsize;
  irqstate_t flags;
  size_t i;

  /* If the capacity is 0, it means that the backtrace record is not
   * initialized, we will initialize it with the default size
   */

  if (capacity == 0)
    {
      poolsize = CONFIG_LIBC_BACKTRACE_INIT_SIZE;
      capacity = CONFIG_LIBC_BACKTRACE_INIT_SIZE;
    }
  else
    {
      /* When expanding, double the capacity of the bunket and release
       * the old bunket.
       */

      poolsize = capacity;
      capacity = capacity * 2;
    }

  bucket = lib_zalloc(sizeof(FAR struct backtrace_entry_s *) * capacity);
  pool = lib_zalloc(sizeof(struct backtrace_entry_s) * poolsize);
  if (bucket == NULL || pool == NULL)
    {
      lib_free(bucket);
      lib_free(pool);
      return -ENOMEM;
    }

  /* Rehash the backtrace record, and move the backtrace record to the new
   * bucket
   */

  flags = backtrace_lock(bp);
  if (bp->capacity)
    {
      backtrace_rehash(bp, bucket);
    }
  else
    {
      sq_init(&bp->freelist);
    }

  /* Expand the backtrace entry pool, and add the new backtrace entry
   * to the free list
   */

  for (i = 0; i < poolsize; i++)
    {
      sq_addlast(&pool[i].freenode, &bp->freelist);
    }

  oldbucket = bp->bucket;
  bp->bucket = bucket;
  bp->capacity = capacity;
  backtrace_unlock(bp, flags);
  lib_free(oldbucket);
  return 0;
}

/****************************************************************************
 * Name: backtrace_exist
 ****************************************************************************/

static FAR struct backtrace_entry_s *
backtrace_find(FAR struct backtrace_pool_s *bp, uint32_t slot,
               FAR const void *stack, int depth)
{
  FAR struct backtrace_entry_s *entry = bp->bucket[slot];

  while (entry != NULL)
    {
      if (entry->depth == depth &&
          memcmp(stack, entry->stack, depth * sizeof(FAR void *)) == 0)
        {
          /* If the backtrace record already exists, just increase the
           * reference count
           */

          entry->ref++;
          return entry;
        }

      entry = entry->next;
    }

  return NULL;
}

/****************************************************************************
 * Name: backtrace_alloc
 ****************************************************************************/

static FAR struct backtrace_entry_s *
backtrace_alloc(FAR struct backtrace_pool_s *bp, uint32_t slot,
                const FAR void *stack, int depth)
{
  FAR struct backtrace_entry_s *entry;

  /* Get the first entry from the free list */

  entry = (FAR struct backtrace_entry_s *)sq_remfirst(&bp->freelist);
  if (entry == NULL)
    {
      return NULL;
    }

  entry = container_of(entry, struct backtrace_entry_s, freenode);
  DEBUGASSERT(entry->ref == 0);

  /* Copy the backtrace to the entry */

  bp->used++;
  memcpy(entry->stack, stack, depth * sizeof(FAR void *));
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
      backtrace_unlock(bp, flags);
      backtrace_expand(bp);
      flags = backtrace_lock(bp);
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

  syslog(LOG_INFO, "capacity: %zu, used: %zu, conflict: %zu\n",
         bp->capacity, bp->used, conflict);
}
#endif

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
