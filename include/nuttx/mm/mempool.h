/****************************************************************************
 * include/nuttx/mm/mempool.h
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

#ifndef __INCLUDE_NUTTX_MM_MEMPOOL_H
#define __INCLUDE_NUTTX_MM_MEMPOOL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <queue.h>
#include <sys/types.h>

#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes memory buffer pool */

struct mempool_s
{
  sq_queue_t list;       /* The free block list in normal mempool */
  sq_queue_t ilist;      /* The free block list in interrupt mempool */
  sq_queue_t elist;      /* The expand block list for normal mempool */
  size_t     bsize;      /* The size for every block in mempool */
  size_t     ninterrupt; /* The number of block in interrupt mempool */
  size_t     nexpand;    /* The number of expand block every time for mempool */
  size_t     nused;      /* The number of used block in mempool */
  spinlock_t lock;       /* The protect lock to mempool */
  sem_t      wait;       /* The semaphore of waiter get free block */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mempool_init
 *
 * Description:
 *   Initialize a memory pool.
 *
 * Input Parameters:
 *   pool       - Address of the memory pool to be used.
 *   bsize      - The block size of memory blocks in pool.
 *   ninitial   - The initial count of memory blocks in pool.
 *   nexpand    - The increment count of memory blocks in pool.
 *                If there is not enough memory blocks and it isn't zero,
 *                mempool_alloc will alloc nexpand memory blocks.
 *   ninterrupt - The block count of memory blocks in pool for interrupt
 *                context. These blocks only can use in interrupt context.
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int mempool_init(FAR struct mempool_s *pool, size_t bsize, size_t ninitial,
                 size_t nexpand, size_t ninterrupt);

/****************************************************************************
 * Name: mempool_alloc
 *
 * Description:
 *   Allocate an block from a specific memory pool.
 *
 *   If there isn't enough memory blocks, This function will expand memory
 *   pool if nexpand isn't zero.
 *
 * Input Parameters:
 *   pool - Address of the memory pool to be used.
 *
 * Returned Value:
 *   The pointer to the allocated block on success; NULL on any failure.
 *
 ****************************************************************************/

FAR void *mempool_alloc(FAR struct mempool_s *pool);

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

void mempool_free(FAR struct mempool_s *pool, FAR void *blk);

/****************************************************************************
 * Name: mempool_deinit
 *
 * Description:
 *   Deallocate a memory pool.
 *
 * Input Parameters:
 *   pool    - Address of the memory pool to be used.
 ****************************************************************************/

int mempool_deinit(FAR struct mempool_s *pool);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif
