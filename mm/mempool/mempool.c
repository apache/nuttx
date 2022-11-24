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

#define ALIGN_UP(x, a) ((((x) + (a) - 1) / (a)) * (a))

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if CONFIG_MM_BACKTRACE >= 0
struct mempool_backtrace_s
{
  FAR struct list_node node;
  pid_t pid;
#  if CONFIG_MM_BACKTRACE > 0
  FAR void *backtrace[CONFIG_MM_BACKTRACE];
#  endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void mempool_add_list(FAR struct list_node *list,
                                    FAR void *base, size_t nblks,
                                    size_t blocksize)
{
  while (nblks--)
    {
      list_add_head(list, ((FAR struct list_node *)((FAR char *)base +
                                                    blocksize * nblks)));
    }
}

static inline FAR void *mempool_malloc(FAR struct mempool_s *pool,
                                       size_t size)
{
  if (pool->alloc != NULL)
    {
      return pool->alloc(pool, size);
    }
  else
    {
      return kmm_malloc(size);
    }
}

static inline void mempool_mfree(FAR struct mempool_s *pool, FAR void *addr)
{
  if (pool->free != NULL)
    {
      return pool->free(pool, addr);
    }
  else
    {
      return kmm_free(addr);
    }
}

#if CONFIG_MM_BACKTRACE >= 0
static inline void mempool_add_backtrace(FAR struct mempool_s *pool,
                                         FAR struct mempool_backtrace_s *buf)
{
  list_add_head(&pool->alist, &buf->node);
  buf->pid = gettid();
#  if CONFIG_MM_BACKTRACE > 0
  if (pool->procfs.backtrace)
    {
      int result = backtrace(buf->backtrace, CONFIG_MM_BACKTRACE);
      while (result < CONFIG_MM_BACKTRACE)
        {
          buf->backtrace[result++] = NULL;
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
#if CONFIG_MM_BACKTRACE >= 0
  size_t blocksize = pool->blocksize + sizeof(struct mempool_backtrace_s);
#else
  size_t blocksize = pool->blocksize;
#endif
  size_t ninterrupt;
  size_t ninitial;
  size_t count;

  DEBUGASSERT(pool->blocksize != 0);

  pool->nused = 0;
  list_initialize(&pool->list);
  list_initialize(&pool->ilist);
  list_initialize(&pool->elist);

#if CONFIG_MM_BACKTRACE >= 0
  list_initialize(&pool->alist);
#endif

  blocksize = ALIGN_UP(blocksize, pool->blocksize);
  ninitial = pool->initialsize / blocksize;
  ninterrupt = pool->interruptsize / blocksize;
  count = ninitial + ninterrupt;
  if (count != 0)
    {
      FAR char *base;

      base = mempool_malloc(pool, blocksize * count +
                            sizeof(struct list_node));
      if (base == NULL)
        {
          return -ENOMEM;
        }

      mempool_add_list(&pool->ilist, base, ninterrupt, blocksize);
      mempool_add_list(&pool->list, base + ninterrupt * blocksize,
                       ninitial, blocksize);
      list_add_head(&pool->elist, (FAR struct list_node *)
                                  (base + count * blocksize));
      kasan_poison(base, blocksize * count);
    }

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
  FAR struct list_node *blk;
  irqstate_t flags;

retry:
  flags = spin_lock_irqsave(&pool->lock);
  blk = list_remove_head(&pool->list);
  if (blk == NULL)
    {
      if (up_interrupt_context())
        {
          blk = list_remove_head(&pool->ilist);
          if (blk == NULL)
            {
              goto out_with_lock;
            }
        }
      else
        {
          spin_unlock_irqrestore(&pool->lock, flags);
          if (pool->expandsize != 0)
            {
#if CONFIG_MM_BACKTRACE >= 0
              size_t blocksize = pool->blocksize +
                                 sizeof(struct mempool_backtrace_s);
#else
              size_t blocksize = pool->blocksize;
#endif
              size_t nexpand;

              blocksize = ALIGN_UP(blocksize, pool->blocksize);
              nexpand = pool->expandsize / blocksize;
              blk = mempool_malloc(pool, blocksize * nexpand + sizeof(*blk));
              if (blk == NULL)
                {
                  return NULL;
                }

              kasan_poison(blk, blocksize * nexpand);
              flags = spin_lock_irqsave(&pool->lock);
              mempool_add_list(&pool->list, blk, nexpand, blocksize);
              list_add_head(&pool->elist, (FAR struct list_node *)
                            ((FAR char *)blk + nexpand * blocksize));
              blk = list_remove_head(&pool->list);
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

  pool->nused++;
#if CONFIG_MM_BACKTRACE >= 0
  mempool_add_backtrace(pool, (FAR struct mempool_backtrace_s *)
                              ((FAR char *)blk + pool->blocksize));
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
  irqstate_t flags;
#if CONFIG_MM_BACKTRACE >= 0
  size_t blocksize = pool->blocksize + sizeof(struct mempool_backtrace_s);
  FAR struct mempool_backtrace_s *buf =
  (FAR struct mempool_backtrace_s *)((FAR char *)blk + pool->blocksize);

  list_delete(&buf->node);
#else
  size_t blocksize = pool->blocksize;
#endif

  flags = spin_lock_irqsave(&pool->lock);
  if ((pool->blocksize & (pool->blocksize - 1)) == 0)
    {
      blocksize = ALIGN_UP(blocksize, pool->blocksize);
    }

  if (pool->interruptsize != 0)
    {
      FAR char *base;
      size_t ninterrupt;

      base = (FAR char *)(list_peek_head(&pool->elist) + 1);
      ninterrupt = pool->interruptsize / blocksize;
      if ((FAR char *)blk >= base &&
          (FAR char *)blk < base + ninterrupt * blocksize)
        {
          list_add_head(&pool->ilist, blk);
        }
      else
        {
          list_add_head(&pool->list, blk);
        }
    }
  else
    {
      list_add_head(&pool->list, blk);
    }

  pool->nused--;
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

int mempool_info(FAR struct mempool_s *pool, struct mempoolinfo_s *info)
{
  irqstate_t flags;

  DEBUGASSERT(pool != NULL && info != NULL);

  flags = spin_lock_irqsave(&pool->lock);
  info->ordblks = list_length(&pool->list);
  info->iordblks = list_length(&pool->ilist);
  info->aordblks = pool->nused;
  info->arena = (pool->nused + info->ordblks + info->iordblks) *
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
      size_t count = list_length(&pool->list);

      info->aordblks += count;
      info->uordblks += count * pool->blocksize;
    }
  else if (info->pid == -1)
    {
#if CONFIG_MM_BACKTRACE >= 0
      size_t blocksize = pool->blocksize +
             sizeof(struct mempool_backtrace_s);
#else
      size_t blocksize = pool->blocksize;
#endif
      size_t count;

      if ((pool->blocksize & (pool->blocksize - 1)) == 0)
        {
          blocksize = ALIGN_UP(blocksize, pool->blocksize);
        }

      count = (pool->initialsize + pool->interruptsize) / blocksize +
              (list_length(&pool->elist) - 1) - list_length(&pool->list);

      info->aordblks += count;
      info->uordblks += count * pool->blocksize;
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
      FAR struct list_node *node;
      list_for_every(&pool->list, node)
        {
          syslog(LOG_INFO, "%12zu%*p\n",
                 pool->blocksize, MM_PTR_FMT_WIDTH,
                 (FAR char *)node);
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
#if CONFIG_MM_BACKTRACE >= 0
  size_t blocksize = pool->blocksize + sizeof(struct mempool_backtrace_s);
#else
  size_t blocksize = pool->blocksize;
#endif
  FAR struct list_node *blk;
  size_t ninterrupt;
  size_t ninitial;
  size_t count;

  if (pool->nused != 0)
    {
      return -EBUSY;
    }

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMPOOL)
  mempool_procfs_unregister(&pool->procfs);
#endif

  if ((pool->blocksize & (pool->blocksize - 1)) == 0)
    {
      blocksize = ALIGN_UP(blocksize, pool->blocksize);
    }

  ninitial = pool->initialsize / blocksize;
  ninterrupt = pool->interruptsize / blocksize;
  count = ninitial + ninterrupt;
  if (count == 0)
    {
      count = pool->expandsize / blocksize;
    }

  while ((blk = list_remove_head(&pool->elist)) != NULL)
    {
      blk = (FAR struct list_node *)((FAR char *)blk -
                                     count * blocksize);
      kasan_unpoison(blk, mm_malloc_size(blk));
      mempool_mfree(pool, blk);
      count = pool->expandsize / blocksize;
    }

  if (pool->wait && pool->expandsize == 0)
    {
      nxsem_destroy(&pool->waitsem);
    }

  return 0;
}
