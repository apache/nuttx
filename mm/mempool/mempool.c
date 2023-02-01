/****************************************************************************
 * mm/mempool/mempool.c
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
#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/mempool.h>

#include "kasan/kasan.h"

#if UINTPTR_MAX <= UINT32_MAX
#  define MM_PTR_FMT_WIDTH 11
#elif UINTPTR_MAX <= UINT64_MAX
#  define MM_PTR_FMT_WIDTH 19
#endif

#undef  ALIGN_UP
#define ALIGN_UP(x, a) (((x) + ((a) - 1)) & (~((a) - 1)))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline FAR sq_entry_t *mempool_remove_queue(FAR sq_queue_t *queue)
{
  if (!sq_empty(queue))
    {
      FAR sq_entry_t *entry = queue->head;

      queue->head = entry->flink;
      return entry;
    }
  else
    {
      return NULL;
    }
}

static inline size_t mempool_queue_lenth(FAR sq_queue_t *queue)
{
  FAR sq_entry_t *node;
  size_t count;

  for (node = queue->head, count = 0;
       node != NULL;
       node = node->flink, count++);

  return count;
}

static inline void mempool_add_queue(FAR sq_queue_t *queue,
                                     FAR char *base, size_t nblks,
                                     size_t blocksize)
{
  while (nblks-- > 0)
    {
      sq_addfirst((FAR sq_entry_t *)(base + blocksize * nblks), queue);
    }
}

#if CONFIG_MM_BACKTRACE >= 0
static inline void mempool_add_backtrace(FAR struct mempool_s *pool,
                                         FAR struct mempool_backtrace_s *buf)
{
  list_add_head(&pool->alist, &buf->node);
  buf->pid = _SCHED_GETTID();
#  if CONFIG_MM_BACKTRACE > 0
  if (pool->procfs.backtrace)
    {
      int result = backtrace(buf->backtrace, CONFIG_MM_BACKTRACE);
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
  pool->nexpend = 0;
  pool->totalsize = 0;

#if CONFIG_MM_BACKTRACE >= 0
  list_initialize(&pool->alist);
#else
  pool->nalloc = 0;
#endif

  if (pool->interruptsize > sizeof(sq_entry_t))
    {
      size_t ninterrupt = (pool->interruptsize - sizeof(sq_entry_t)) /
                          blocksize;
      size_t size = ninterrupt * blocksize + sizeof(sq_entry_t);

      pool->ibase = pool->alloc(pool, size);
      if (pool->ibase == NULL)
        {
          return -ENOMEM;
        }

      pool->nexpend++;
      pool->totalsize += size;
      mempool_add_queue(&pool->iqueue, pool->ibase, ninterrupt, blocksize);
      kasan_poison(pool->ibase, size);
    }
  else
    {
      pool->ibase = NULL;
    }

  if (pool->initialsize > sizeof(sq_entry_t))
    {
      size_t ninitial = (pool->initialsize - sizeof(sq_entry_t)) / blocksize;
      size_t size = ninitial * blocksize + sizeof(sq_entry_t);
      FAR char *base;

      base = pool->alloc(pool, size);
      if (base == NULL)
        {
          mempool_free(pool, pool->ibase);
          return -ENOMEM;
        }

      pool->nexpend++;
      pool->totalsize += size;
      mempool_add_queue(&pool->queue, base, ninitial, blocksize);
      sq_addlast((FAR sq_entry_t *)(base + ninitial * blocksize),
                  &pool->equeue);
      kasan_poison(base, size);
    }

  spin_initialize(&pool->lock, 0);
  if (pool->wait && pool->expandsize == 0)
    {
      nxsem_init(&pool->waitsem, 0, 0);
    }

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL)
  mempool_procfs_register(&pool->procfs, name);
#  ifdef CONFIG_MM_BACKTRACE_DEFAULT
  pool->procfs.backtrace = true;
#  endif
#endif

  return 0;
}

/****************************************************************************
 * Name: mempool_alloc
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

FAR void *mempool_alloc(FAR struct mempool_s *pool)
{
  FAR sq_entry_t *blk;
  irqstate_t flags;

retry:
  flags = spin_lock_irqsave(&pool->lock);
  blk = mempool_remove_queue(&pool->queue);
  if (blk == NULL)
    {
      if (up_interrupt_context())
        {
          blk = mempool_remove_queue(&pool->iqueue);
          if (blk == NULL)
            {
              goto out_with_lock;
            }
        }
      else
        {
          spin_unlock_irqrestore(&pool->lock, flags);
          if (pool->expandsize > sizeof(sq_entry_t))
            {
              size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
              size_t nexpand = (pool->expandsize - sizeof(sq_entry_t)) /
                               blocksize;
              size_t size = nexpand * blocksize + sizeof(sq_entry_t);
              FAR char *base = pool->alloc(pool, size);

              if (base == NULL)
                {
                  return NULL;
                }

              pool->nexpend++;
              pool->totalsize += size;
              kasan_poison(base, size);
              flags = spin_lock_irqsave(&pool->lock);
              mempool_add_queue(&pool->queue, base, nexpand, blocksize);
              sq_addlast((FAR sq_entry_t *)(base + nexpand * blocksize),
                         &pool->equeue);
              blk = mempool_remove_queue(&pool->queue);
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

#if CONFIG_MM_BACKTRACE >= 0
  mempool_add_backtrace(pool, (FAR struct mempool_backtrace_s *)
                              ((FAR char *)blk + pool->blocksize));
#else
  pool->nalloc++;
#endif
  kasan_unpoison(blk, pool->blocksize);
out_with_lock:
  spin_unlock_irqrestore(&pool->lock, flags);
  return blk;
}

/****************************************************************************
 * Name: mempool_free
 *
 * Description:
 *   Release an memory block to the pool.
 *
 * Input Parameters:
 *   pool - Address of the memory pool to be used.
 *   blk  - The pointer of memory block.
 ****************************************************************************/

void mempool_free(FAR struct mempool_s *pool, FAR void *blk)
{
  irqstate_t flags = spin_lock_irqsave(&pool->lock);
  size_t blocksize = MEMPOOL_REALBLOCKSIZE(pool);
#if CONFIG_MM_BACKTRACE >= 0
  FAR struct mempool_backtrace_s *buf =
    (FAR struct mempool_backtrace_s *)((FAR char *)blk + pool->blocksize);

  list_delete(&buf->node);
#else
  pool->nalloc--;
#endif

  if (pool->interruptsize > blocksize)
    {
      if ((FAR char *)blk >= pool->ibase &&
          (FAR char *)blk < pool->ibase + pool->interruptsize - blocksize)
        {
          sq_addfirst(blk, &pool->iqueue);
        }
      else
        {
          sq_addfirst(blk, &pool->queue);
        }
    }
  else
    {
      sq_addfirst(blk, &pool->queue);
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
  irqstate_t flags;

  DEBUGASSERT(pool != NULL && info != NULL);

  flags = spin_lock_irqsave(&pool->lock);
  info->ordblks = mempool_queue_lenth(&pool->queue);
  info->iordblks = mempool_queue_lenth(&pool->iqueue);
#if CONFIG_MM_BACKTRACE >= 0
  info->aordblks = list_length(&pool->alist);
#else
  info->aordblks = pool->nalloc;
#endif
  info->arena = (info->aordblks + info->ordblks + info->iordblks) *
                pool->blocksize;
  spin_unlock_irqrestore(&pool->lock, flags);
  info->sizeblks = pool->blocksize;
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

int mempool_info_task(FAR struct mempool_s *pool,
                      FAR struct mempoolinfo_task *info)
{
  DEBUGASSERT(info);
  if (info->pid == -2)
    {
      size_t count = mempool_queue_lenth(&pool->queue) +
                     mempool_queue_lenth(&pool->iqueue);

      info->aordblks += count;
      info->uordblks += count * pool->blocksize;
      if (pool->calibrate)
        {
          info->aordblks -= pool->nexpend;
          info->uordblks -= pool->totalsize;
        }
    }
  else if (info->pid == -1)
    {
#if CONFIG_MM_BACKTRACE >= 0
      size_t count = list_length(&pool->alist);
#else
      size_t count = pool->nalloc;
#endif

      info->aordblks += count;
      info->uordblks += count * pool->blocksize;
      info->aordblks -= pool->nexpend;
      info->uordblks -= pool->totalsize;
    }
#if CONFIG_MM_BACKTRACE >= 0
  else
    {
      FAR struct mempool_backtrace_s *buf;

      list_for_every_entry(&pool->alist, buf, struct mempool_backtrace_s,
                           node)
        {
          if (buf->pid == info->pid)
            {
              info->aordblks++;
              info->uordblks += pool->blocksize;
            }
        }
    }
#endif

  return OK;
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
 *   pid     - The task of pid.
 *
 * Returned Value:
 *   OK on success; A negated errno value on any failure.
 ****************************************************************************/

void mempool_memdump(FAR struct mempool_s *pool, pid_t pid)
{
  if (pid == -2)
    {
      FAR sq_entry_t *entry;

      sq_for_every(&pool->queue, entry)
        {
          syslog(LOG_INFO, "%12zu%*p\n",
                 pool->blocksize, MM_PTR_FMT_WIDTH,
                 (FAR char *)entry);
        }

      sq_for_every(&pool->iqueue, entry)
        {
          syslog(LOG_INFO, "%12zu%*p\n",
                 pool->blocksize, MM_PTR_FMT_WIDTH,
                 (FAR char *)entry);
        }
    }
#if CONFIG_MM_BACKTRACE >= 0
  else
    {
      FAR struct mempool_backtrace_s *buf;

      list_for_every_entry(&pool->alist, buf, struct mempool_backtrace_s,
                           node)
        {
          if (buf->pid == pid || pid == -1)
            {
#  if CONFIG_MM_BACKTRACE > 0
              int i;
              FAR const char *format = " %0*p";
#  endif
              char bt[CONFIG_MM_BACKTRACE * MM_PTR_FMT_WIDTH + 1];

              bt[0] = '\0';
#  if CONFIG_MM_BACKTRACE > 0
              for (i = 0; i < CONFIG_MM_BACKTRACE && buf->backtrace[i]; i++)
                {
                  sprintf(bt + i * MM_PTR_FMT_WIDTH, format,
                          MM_PTR_FMT_WIDTH - 1, buf->backtrace[i]);
                }
#  endif

              syslog(LOG_INFO, "%6d%12zu%*p%s\n",
                     (int)buf->pid, pool->blocksize, MM_PTR_FMT_WIDTH,
                     ((FAR char *)buf - pool->blocksize), bt);
            }
        }
    }
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

#if CONFIG_MM_BACKTRACE >= 0
  if (!list_is_empty(&pool->alist))
#else
  if (pool->nalloc != 0)
#endif
    {
      return -EBUSY;
    }

  if (pool->initialsize > sizeof(sq_entry_t))
    {
      count = (pool->initialsize - sizeof(sq_entry_t)) / blocksize;
    }

  if (count == 0)
    {
      if (pool->expandsize > sizeof(sq_entry_t))
        {
          count = (pool->expandsize - sizeof(sq_entry_t)) / blocksize;
        }
    }

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL)
  mempool_procfs_unregister(&pool->procfs);
#endif

  while ((blk = mempool_remove_queue(&pool->equeue)) != NULL)
    {
      blk = (FAR sq_entry_t *)((FAR char *)blk - count * blocksize);
      pool->free(pool, blk);
      if (pool->expandsize > sizeof(sq_entry_t))
        {
          count = (pool->expandsize - sizeof(sq_entry_t)) / blocksize;
        }
    }

  if (pool->ibase)
    {
      pool->free(pool, pool->ibase);
    }

  if (pool->wait && pool->expandsize == 0)
    {
      nxsem_destroy(&pool->waitsem);
    }

  return 0;
}
