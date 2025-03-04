/****************************************************************************
 * mm/mempool/mempool.c
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

#include <assert.h>
#include <execinfo.h>
#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/kasan.h>
#include <nuttx/mm/mempool.h>
#include <nuttx/nuttx.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_MM_BACKTRACE >= 0
#define MEMPOOL_MAGIC_FREE  0xAAAAAAAA
#define MEMPOOL_MAGIC_ALLOC 0x55555555

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*mempool_callback_t)(FAR struct mempool_s *pool,
                                   FAR struct mempool_backtrace_s *buf,
                                   FAR const void *input, FAR void *output);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline FAR sq_entry_t *
mempool_remove_queue(FAR struct mempool_s *pool, FAR sq_queue_t *queue)
{
  FAR sq_entry_t *ret = queue->head;

  if (ret)
    {
      queue->head = ret->flink;
      if (!queue->head)
        {
          queue->tail = NULL;
        }
      else
        {
          pool->check(pool, queue->head);
        }

      ret->flink = NULL;
    }

  return ret;
}

static inline void mempool_add_queue(FAR struct mempool_s *pool,
                                     FAR sq_queue_t *queue,
                                     FAR char *base, size_t nblks,
                                     size_t blocksize)
{
  while (nblks-- > 0)
    {
#if CONFIG_MM_BACKTRACE >= 0
      FAR struct mempool_backtrace_s *buf =
       (FAR struct mempool_backtrace_s *)
       (base + nblks * blocksize + pool->blocksize);

      buf->magic = MEMPOOL_MAGIC_FREE;
#endif
      sq_addlast((FAR sq_entry_t *)(base + blocksize * nblks), queue);
    }
}

#if CONFIG_MM_BACKTRACE >= 0
static inline void mempool_add_backtrace(FAR struct mempool_s *pool,
                                         FAR struct mempool_backtrace_s *buf)
{
  DEBUGASSERT(buf->magic == MEMPOOL_MAGIC_FREE);
  buf->magic = MEMPOOL_MAGIC_ALLOC;
  buf->pid = _SCHED_GETTID();
  buf->seqno = g_mm_seqno++;
#  if CONFIG_MM_BACKTRACE > 0
  if (pool->procfs.backtrace)
    {
      int result = sched_backtrace(buf->pid, buf->backtrace,
                                   CONFIG_MM_BACKTRACE,
                                   CONFIG_MM_HEAP_MEMPOOL_BACKTRACE_SKIP);
      if (result < CONFIG_MM_BACKTRACE)
        {
          buf->backtrace[result] = NULL;
        }
    }
  else
    {
      buf->backtrace[0] = NULL;
    }
#  endif
}

static void mempool_foreach(FAR struct mempool_s *pool,
                            mempool_callback_t callback,
                            FAR const void *input, FAR void *output)
{
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
  FAR struct mempool_backtrace_s *buf;
  FAR sq_entry_t *entry;
  FAR char *base ;
  size_t nblks;

  if (pool->ibase != NULL)
    {
      nblks = pool->interruptsize / blocksize;
      while (nblks--)
        {
          buf = (FAR struct mempool_backtrace_s *)
                  pool->ibase + nblks * blocksize + pool->blocksize;

          callback(pool, buf, input, output);
        }
    }

  sq_for_every(&pool->equeue, entry)
    {
      nblks = (pool->expandsize - sizeof(sq_entry_t)) / blocksize;
      base = (FAR char *)entry - (nblks * blocksize);

      while (nblks--)
        {
          buf = (FAR struct mempool_backtrace_s *)
                  (base + nblks * blocksize + pool->blocksize);
          callback(pool, buf, input, output);
        }
    }
}

static void mempool_info_task_callback(FAR struct mempool_s *pool,
                                       FAR struct mempool_backtrace_s *buf,
                                       FAR const void *input,
                                       FAR void *output)
{
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
  FAR const struct malltask *task = input;
  FAR struct mallinfo_task *info = output;

  if (buf->magic == MEMPOOL_MAGIC_FREE)
    {
      return;
    }

  if ((MM_DUMP_ASSIGN(task, buf) || MM_DUMP_ALLOC(task, buf) ||
       MM_DUMP_LEAK(task, buf)) && MM_DUMP_SEQNO(task, buf))
    {
      info->aordblks++;
      info->uordblks += blocksize;
    }
}

static void mempool_memdump_callback(FAR struct mempool_s *pool,
                                     FAR struct mempool_backtrace_s *buf,
                                     FAR const void *input, FAR void *output)
{
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
  size_t overhead = blocksize - pool->blocksize;
  FAR const struct mm_memdump_s *dump = input;

  if (buf->magic == MEMPOOL_MAGIC_FREE)
    {
      return;
    }

  if ((MM_DUMP_ASSIGN(dump, buf) || MM_DUMP_ALLOC(dump, buf) ||
       MM_DUMP_LEAK(dump, buf)) && MM_DUMP_SEQNO(dump, buf))
    {
#  if CONFIG_MM_BACKTRACE > 0
      char tmp[BACKTRACE_BUFFER_SIZE(CONFIG_MM_BACKTRACE)];

      backtrace_format(tmp, sizeof(tmp), buf->backtrace,
                       CONFIG_MM_BACKTRACE);
#  else
      FAR const char *tmp = "";
#  endif

      syslog(LOG_INFO, "%6d%12zu%9zu%12lu%*p %s\n",
             buf->pid, blocksize, overhead, buf->seqno,
             BACKTRACE_PTR_FMT_WIDTH,
             ((FAR char *)buf - pool->blocksize), tmp);
    }
}

static void
mempool_memdump_free_callback(FAR struct mempool_s *pool,
                              FAR struct mempool_backtrace_s *buf,
                              FAR const void *input, FAR void *output)
{
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
  size_t overhead = blocksize - pool->blocksize;

  if (buf->magic == MEMPOOL_MAGIC_FREE)
    {
      syslog(LOG_INFO, "%12zu%9zu%*p\n",
             blocksize, overhead, BACKTRACE_PTR_FMT_WIDTH,
             ((FAR char *)buf - pool->blocksize));
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mempool_init
 *
 * Description:
 *   Initialize a memory pool.
 *   The user needs to specify the initialization information of mempool
 *   including blocksize, initialsize, expandsize, interruptsize.
 *
 * Input Parameters:
 *   pool - Address of the memory pool to be used.
 *   name - The name of memory pool.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int mempool_init(FAR struct mempool_s *pool, FAR const char *name)
{
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);

  sq_init(&pool->queue);
  sq_init(&pool->iqueue);
  sq_init(&pool->equeue);
  pool->nalloc = 0;
  if (pool->interruptsize >= blocksize)
    {
      size_t ninterrupt = pool->interruptsize / blocksize;
      size_t size = ninterrupt * blocksize;

      pool->ibase = pool->alloc(pool, size);
      if (pool->ibase == NULL)
        {
          return -ENOMEM;
        }

      mempool_add_queue(pool, &pool->iqueue,
                        pool->ibase, ninterrupt, blocksize);
      kasan_poison(pool->ibase, size);
    }
  else
    {
      pool->ibase = NULL;
    }

  if (pool->initialsize >= blocksize + sizeof(sq_entry_t))
    {
      size_t ninitial = (pool->initialsize - sizeof(sq_entry_t)) / blocksize;
      size_t size = ninitial * blocksize + sizeof(sq_entry_t);
      FAR char *base;

      base = pool->alloc(pool, size);
      if (base == NULL)
        {
          if (pool->ibase)
            {
              pool->free(pool, pool->ibase);
            }

          return -ENOMEM;
        }

      mempool_add_queue(pool, &pool->queue,
                        base, ninitial, blocksize);
      sq_addlast((FAR sq_entry_t *)(base + ninitial * blocksize),
                  &pool->equeue);
      kasan_poison(base, size);
    }

  spin_lock_init(&pool->lock);
  if (pool->wait && pool->expandsize == 0)
    {
      nxsem_init(&pool->waitsem, 0, 0);
    }

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL)
  mempool_procfs_register(&pool->procfs, name);
#  ifdef CONFIG_MM_BACKTRACE_DEFAULT
  pool->procfs.backtrace = true;
#  elif CONFIG_MM_BACKTRACE > 0
  pool->procfs.backtrace = false;
#  endif
#endif

  return 0;
}

/****************************************************************************
 * Name: mempool_allocate
 *
 * Description:
 *   Allocate an block from a specific memory pool.
 *
 *   If there isn't enough memory blocks, This function will expand memory
 *   pool if expandsize isn't zero.
 *
 * Input Parameters:
 *   pool - Address of the memory pool to be used.
 *
 * Returned Value:
 *   The pointer to the allocated block on success; NULL on any failure.
 *
 ****************************************************************************/

FAR void *mempool_allocate(FAR struct mempool_s *pool)
{
  FAR sq_entry_t *blk;
  irqstate_t flags;

retry:
  flags = spin_lock_irqsave(&pool->lock);
  blk = mempool_remove_queue(pool, &pool->queue);
  if (blk == NULL)
    {
      if (up_interrupt_context())
        {
          blk = mempool_remove_queue(pool, &pool->iqueue);
          if (blk == NULL)
            {
              spin_unlock_irqrestore(&pool->lock, flags);
              return blk;
            }
        }
      else
        {
          size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);

          spin_unlock_irqrestore(&pool->lock, flags);
          if (pool->expandsize >= blocksize + sizeof(sq_entry_t))
            {
              size_t nexpand = (pool->expandsize - sizeof(sq_entry_t)) /
                               blocksize;
              size_t size = nexpand * blocksize + sizeof(sq_entry_t);
              FAR char *base = pool->alloc(pool, size);

              if (base == NULL)
                {
                  return NULL;
                }

              kasan_poison(base, size);
              flags = spin_lock_irqsave(&pool->lock);
              mempool_add_queue(pool, &pool->queue,
                                base, nexpand, blocksize);
              sq_addlast((FAR sq_entry_t *)(base + nexpand * blocksize),
                         &pool->equeue);
              blk = mempool_remove_queue(pool, &pool->queue);
            }
          else if (!pool->wait ||
                   nxsem_wait_uninterruptible(&pool->waitsem) < 0)
            {
              return NULL;
            }
          else
            {
              goto retry;
            }
        }
    }

  pool->nalloc++;
  spin_unlock_irqrestore(&pool->lock, flags);

#if CONFIG_MM_BACKTRACE >= 0
  mempool_add_backtrace(pool, (FAR struct mempool_backtrace_s *)
                              ((FAR char *)blk + pool->blocksize));
#endif

  blk = kasan_unpoison(blk, pool->blocksize);
#ifdef CONFIG_MM_FILL_ALLOCATIONS
  memset(blk, MM_ALLOC_MAGIC, pool->blocksize);
#endif

  return blk;
}

/****************************************************************************
 * Name: mempool_release
 *
 * Description:
 *   Release a memory block to the pool.
 *
 * Input Parameters:
 *   pool - Address of the memory pool to be used.
 *   blk  - The pointer of memory block.
 ****************************************************************************/

void mempool_release(FAR struct mempool_s *pool, FAR void *blk)
{
  irqstate_t flags = spin_lock_irqsave(&pool->lock);
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
#if CONFIG_MM_BACKTRACE >= 0
  FAR struct mempool_backtrace_s *buf =
    (FAR struct mempool_backtrace_s *)((FAR char *)blk + pool->blocksize);

  /* Check double free or out of out of bounds */

  DEBUGASSERT(buf->magic == MEMPOOL_MAGIC_ALLOC);
  buf->magic = MEMPOOL_MAGIC_FREE;

#endif

  pool->nalloc--;

#ifdef CONFIG_MM_FILL_ALLOCATIONS
  memset(blk, MM_FREE_MAGIC, pool->blocksize);
#endif

  if (pool->interruptsize > blocksize)
    {
      if ((FAR char *)blk >= pool->ibase &&
          (FAR char *)blk < pool->ibase + pool->interruptsize - blocksize)
        {
          sq_addlast(blk, &pool->iqueue);
        }
      else
        {
          sq_addlast(blk, &pool->queue);
        }
    }
  else
    {
      sq_addlast(blk, &pool->queue);
    }

  kasan_poison(blk, pool->blocksize);
  spin_unlock_irqrestore(&pool->lock, flags);
  if (pool->wait && pool->expandsize == 0)
    {
      int semcount;

      nxsem_get_value(&pool->waitsem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&pool->waitsem);
        }
    }
}

/****************************************************************************
 * Name: mempool_info
 *
 * Description:
 *   mempool_info returns a copy of updated current mempool information.
 *
 * Input Parameters:
 *   pool    - Address of the memory pool to be used.
 *   info    - The pointer of mempoolinfo.
 *
 * Returned Value:
 *   OK on success; A negated errno value on any failure.
 ****************************************************************************/

int mempool_info(FAR struct mempool_s *pool, FAR struct mempoolinfo_s *info)
{
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
  irqstate_t flags;

  DEBUGASSERT(pool != NULL && info != NULL);

  flags = spin_lock_irqsave(&pool->lock);
  info->ordblks = sq_count(&pool->queue);
  info->iordblks = sq_count(&pool->iqueue);
  info->aordblks = pool->nalloc;
  info->arena = sq_count(&pool->equeue) * sizeof(sq_entry_t) +
    (info->aordblks + info->ordblks + info->iordblks) * blocksize;
  spin_unlock_irqrestore(&pool->lock, flags);
  info->sizeblks = blocksize;
  if (pool->wait && pool->expandsize == 0)
    {
      int semcount;

      nxsem_get_value(&pool->waitsem, &semcount);
      info->nwaiter = -semcount;
    }
  else
    {
      info->nwaiter = 0;
    }

  return 0;
}

/****************************************************************************
 * Name: mempool_info_task
 ****************************************************************************/

struct mallinfo_task
mempool_info_task(FAR struct mempool_s *pool,
                  FAR const struct malltask *task)
{
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
  struct mallinfo_task info =
    {
      0, 0
    };

  if (task->pid == PID_MM_FREE)
    {
      irqstate_t flags = spin_lock_irqsave(&pool->lock);
      size_t count = sq_count(&pool->queue) +
                     sq_count(&pool->iqueue);

      spin_unlock_irqrestore(&pool->lock, flags);
      info.aordblks += count;
      info.uordblks += count * blocksize;
    }
  else if (task->pid == PID_MM_ALLOC)
    {
      info.aordblks += pool->nalloc;
      info.uordblks += pool->nalloc * blocksize;
    }
#if CONFIG_MM_BACKTRACE >= 0
  else
    {
      mempool_foreach(pool, mempool_info_task_callback, task, &info);
    }
#endif

  return info;
}

/****************************************************************************
 * Name: mempool_memdump
 *
 * Description:
 *   mempool_memdump returns a memory info about specified pid of
 *   task/thread. if pid equals -1, this function will dump all allocated
 *   node and output backtrace for every allocated node for this mempool,
 *   if pid equals -2, this function will dump all free node for this
 *   mempool, and if pid is greater than or equal to 0, will dump pid
 *   allocated node and output backtrace.
 *
 * Input Parameters:
 *   pool    - Address of the memory pool to be used.
 *   dump    - The information of what need dump.
 *
 * Returned Value:
 *   OK on success; A negated errno value on any failure.
 ****************************************************************************/

void mempool_memdump(FAR struct mempool_s *pool,
                     FAR const struct mm_memdump_s *dump)
{
#if CONFIG_MM_BACKTRACE >= 0
  if (dump->pid == PID_MM_FREE)
    {
      mempool_foreach(pool, mempool_memdump_free_callback, NULL, NULL);
    }
  else
    {
      mempool_foreach(pool, mempool_memdump_callback, dump, NULL);
    }
#else
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
  size_t overhead = blocksize - pool->blocksize;

  /* Avoid race condition */

  syslog(LOG_INFO, "%12zu%9zu%*p skip block dump\n",
         blocksize, overhead, BACKTRACE_PTR_FMT_WIDTH, pool);
#endif
}

/****************************************************************************
 * Name: mempool_deinit
 *
 * Description:
 *   Deallocate a memory pool.
 *
 * Input Parameters:
 *   pool    - Address of the memory pool to be used.
 ****************************************************************************/

int mempool_deinit(FAR struct mempool_s *pool)
{
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
  FAR sq_entry_t *blk;
  size_t count = 0;

  if (pool->nalloc != 0)
    {
      return -EBUSY;
    }

  if (pool->initialsize >= blocksize + sizeof(sq_entry_t))
    {
      count = (pool->initialsize - sizeof(sq_entry_t)) / blocksize;
    }

  if (count == 0)
    {
      if (pool->expandsize >= blocksize + sizeof(sq_entry_t))
        {
          count = (pool->expandsize - sizeof(sq_entry_t)) / blocksize;
        }
    }

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL)
  mempool_procfs_unregister(&pool->procfs);
#endif

  while ((blk = mempool_remove_queue(pool, &pool->equeue)) != NULL)
    {
      blk = (FAR sq_entry_t *)((FAR char *)blk - count * blocksize);

      blk = kasan_unpoison(blk, count * blocksize + sizeof(sq_entry_t));
      pool->free(pool, blk);
      if (pool->expandsize >= blocksize + sizeof(sq_entry_t))
        {
          count = (pool->expandsize - sizeof(sq_entry_t)) / blocksize;
        }
    }

  if (pool->ibase)
    {
      pool->ibase = kasan_unpoison(pool->ibase,
                      pool->interruptsize / blocksize * blocksize);
      pool->free(pool, pool->ibase);
    }

  if (pool->wait && pool->expandsize == 0)
    {
      nxsem_destroy(&pool->waitsem);
    }

  return 0;
}
