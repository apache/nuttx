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

#include <execinfo.h>
#include <stdio.h>
#include <stdbool.h>
#include <syslog.h>
#include <sys/param.h>

#include <nuttx/spinlock.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LIBC_BACKTRACE_BUFFSIZE
#  define CONFIG_LIBC_BACKTRACE_BUFFSIZE 0
#endif

/* Calculate the number of backtrace that can be saved based on
 * LIBC_BACKTRACE_BUFFSIZE. Each backtrace entry corresponds to a hash
 * table entry. The maximum number of entries that can be recorded is
 * BACKTRACE_BUFFSIZE / (backtrace entry size + hash entry size).
 */

#define BACKTRACE_NUMBER (CONFIG_LIBC_BACKTRACE_BUFFSIZE / \
                         (sizeof(struct backtrace_entry_s) + \
                          sizeof(int)))

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

#if CONFIG_LIBC_BACKTRACE_BUFFSIZE > 0
struct backtrace_entry_s
{
  union
  {
    FAR void *stack[CONFIG_LIBC_BACKTRACE_DEPTH];
    struct sq_entry_s freenode;
  };

  uint16_t depth; /* Depth of the backtrace */
  uint16_t count; /* Count of the backtrace */
  int      next;  /* Next index in the hash chain */
};

struct backtrace_pool_s
{
  /* Pool to store the backtrace record */

  struct backtrace_entry_s pool[BACKTRACE_NUMBER];

  /* Hash table to store the index of the backtrace record */

  int bucket[BACKTRACE_NUMBER];
  struct sq_queue_s freelist;
  spinlock_t lock;
  bool init;
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

#if CONFIG_LIBC_BACKTRACE_BUFFSIZE > 0

/****************************************************************************
 * Name: backtrace_init
 *
 * Description:
 *   Initialize the backtrace record
 *
 ****************************************************************************/

static void backtrace_init(void)
{
  FAR struct backtrace_pool_s *bp = &g_backtrace_pool;
  size_t i;

  sq_init(&bp->freelist);
  memset(bp->bucket, -1, sizeof(bp->bucket));
  for (i = 0; i < nitems(bp->pool); i++)
    {
      sq_addlast(&bp->pool[i].freenode, &bp->freelist);
    }
}

/****************************************************************************
 * Name: backtrace_hash
 ****************************************************************************/

static int backtrace_hash(FAR struct backtrace_pool_s *bp,
                          FAR const void *backtrace, int depth)
{
  FAR const uint8_t *data = backtrace;
  uint32_t hash = 5381;
  int i;

  for (i = 0; i < depth * sizeof(uintptr_t); i++)
    {
      hash = ((hash << 5) + hash) + data[i];
    }

  return hash % nitems(bp->bucket);
}

/****************************************************************************
 * Name: backtrace_exist
 ****************************************************************************/

static int backtrace_exist(FAR struct backtrace_pool_s *bp, int slot,
                           FAR void *backtrace, int depth)
{
  FAR struct backtrace_entry_s *entry;
  int index;

  if (bp->init == false)
    {
      bp->init = true;
      backtrace_init();
      return -ENOENT;
    }

  index = bp->bucket[slot];

  while (index >= 0)
    {
      entry = &bp->pool[index];
      if (entry->depth == depth &&
          memcmp(backtrace, entry->stack, depth * sizeof(FAR void *)) == 0)
        {
          return index;
        }

      index = entry->next;
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: backtrace_alloc
 ****************************************************************************/

static int backtrace_alloc(FAR struct backtrace_pool_s *bp)
{
  FAR struct backtrace_entry_s *entry;
  FAR struct sq_entry_s *sq;

  /* Get the first entry from the free list */

  sq = sq_remfirst(&bp->freelist);
  if (!sq)
    {
      return -ENOMEM;
    }

  entry = (FAR struct backtrace_entry_s *)sq;
  return entry - bp->pool;
}

/****************************************************************************
 * Name: backtrace_free
 ****************************************************************************/

static void backtrace_free(FAR struct backtrace_pool_s *bp, int index)
{
  FAR struct backtrace_entry_s *entry = &bp->pool[index];
  sq_addlast(&entry->freenode, &bp->freelist);
  entry->depth = 0;
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
 *   a negtive value.
 ****************************************************************************/

int backtrace_record(int skip)
{
  FAR void *buffer[CONFIG_LIBC_BACKTRACE_DEPTH];
  FAR struct backtrace_pool_s *bp = &g_backtrace_pool;
  FAR struct backtrace_entry_s *entry;
  irqstate_t flags;
  int depth;
  int index;
  int slot;

  depth = sched_backtrace(_SCHED_GETTID(), buffer,
                          CONFIG_LIBC_BACKTRACE_DEPTH, skip);
  if (depth <= 0)
    {
      return -ENOENT;
    }

  slot = backtrace_hash(bp, buffer, depth);
  flags = spin_lock_irqsave(&bp->lock);
  index = backtrace_exist(bp, slot, buffer, depth);
  if (index >= 0)
    {
      /* If the backtrace record already exists, just increase the count */

      entry = &bp->pool[index];
      entry->count++;
      spin_unlock_irqrestore(&pool->lock, flags);
      return index;
    }

  index = backtrace_alloc(bp);
  if (index < 0)
    {
      spin_unlock_irqrestore(&pool->lock, flags);
      return index;
    }

  entry = &bp->pool[index];
  memcpy(entry->stack, buffer, depth * sizeof(FAR void *));
  entry->depth = depth;
  entry->count = 1;

  /* Insert backtrace to the head of the linked list of the
   * current hash value
   */

  entry->next = bp->bucket[slot];
  bp->bucket[slot] = index;
  spin_unlock_irqrestore(&pool->lock, flags);
  return index;
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
 *   Return 0 if success, otherwise return a negtive value.
 ****************************************************************************/

int backtrace_remove(int index)
{
  FAR struct backtrace_pool_s *bp = &g_backtrace_pool;
  FAR struct backtrace_entry_s *entry;
  irqstate_t flags;
  int slot;

  if (index < 0 || index >= nitems(bp->pool))
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&bp->lock);
  entry = &bp->pool[index];
  if (entry->count > 1)
    {
      entry->count--;
      spin_unlock_irqrestore(&pool->lock, flags);
      return OK;
    }

  slot = backtrace_hash(bp, entry->stack, entry->depth);

  /* Remove the backtrace record from the linked list */

  if (bp->bucket[slot] == index)
    {
      /* Remove the head of the singly linked list */

      bp->bucket[slot] = entry->next;
    }
  else if (entry->next >= 0)
    {
      /* Copy and delete the next node of the singly linked list
       * to achieve O(1) deletion
       */

      FAR struct backtrace_entry_s *next = &bp->pool[entry->next];
      index = entry->next;
      memcpy(entry, next, sizeof(struct backtrace_entry_s));
    }
  else
    {
      /* If it is the last node in the linked list, we need to traverse
       * and find the previous node to delete it.
       */

      int prev = bp->bucket[slot];
      while (prev >= 0)
        {
          if (bp->pool[prev].next == index)
            {
              bp->pool[prev].next = -1;
              break;
            }

          prev = bp->pool[prev].next;
        }
    }

  backtrace_free(bp, index);
  spin_unlock_irqrestore(&pool->lock, flags);
  return OK;
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

FAR void **backtrace_get(int index, FAR int *size)
{
  FAR struct backtrace_pool_s *bp = &g_backtrace_pool;
  FAR struct backtrace_entry_s *entry;

  if (size == NULL || index < 0 || index >= nitems(bp->pool))
    {
      return NULL;
    }

  entry = &bp->pool[index];
  *size = entry->depth;
  return entry->stack;
}

void backtrace_dump(void)
{
  char buf[BACKTRACE_BUFFER_SIZE(CONFIG_LIBC_BACKTRACE_DEPTH)];
  FAR struct backtrace_pool_s *bp = &g_backtrace_pool;
  FAR struct backtrace_entry_s *entry;
  size_t i;

  syslog(LOG_INFO, "%-6s %-6s %s", "index", "count", "backtrace");
  for (i = 0; i < nitems(bp->pool); i++)
    {
      entry = &bp->pool[i];
      if (entry->depth)
        {
          backtrace_format(buf, sizeof(buf), entry->stack, entry->depth);
          syslog(LOG_INFO, "%-6zu %-6u %s\n", i, entry->count, buf);
        }
    }
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

  if (size < 1)
    {
      return 0;
    }

  buffer[0] = '\0';
  for (i = 0; i < depth && backtrace[i]; i++)
    {
      if (i * BACKTRACE_PTR_FMT_WIDTH >= size)
        {
          break;
        }

      len += snprintf(buffer + i * BACKTRACE_PTR_FMT_WIDTH,
                      size - i * BACKTRACE_PTR_FMT_WIDTH,
                      format, BACKTRACE_PTR_FMT_WIDTH - 1, backtrace[i]);
    }

  return len;
}
