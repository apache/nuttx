/****************************************************************************
 * net/utils/net_bufpool.c
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

#include <nuttx/config.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/semaphore.h>

#include "utils/utils.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The node to store in the pool */

struct net_bufnode_s
{
  sq_entry_t node;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_bufpool_init
 *
 * Description:
 *   Initialize a network buffer pool.
 *
 * Input Parameters:
 *   pool - The pool to be initialized
 *
 ****************************************************************************/

void net_bufpool_init(FAR struct net_bufpool_s *pool)
{
  int i;
  unsigned int maxalloc;

  if (pool->dynalloc > 0)
    {
      maxalloc = pool->u.maxalloc > 0 ? pool->u.maxalloc : INT16_MAX;
    }
  else
    {
      maxalloc = pool->prealloc;
    }

  nxsem_init(&pool->u.sem, 0, maxalloc);

  sq_init(&pool->freebuffers);
  for (i = 0; i < pool->prealloc; i++)
    {
      FAR struct net_bufnode_s *node = (FAR struct net_bufnode_s *)
                                      (pool->pool + i * pool->nodesize);
      sq_addlast(&node->node, &pool->freebuffers);
    }
}

/****************************************************************************
 * Name: net_bufpool_timedalloc
 *
 * Description:
 *   Allocate a buffer from the pool.  If no buffer is available, then wait
 *   for the specified timeout.
 *
 * Input Parameters:
 *   pool    - The pool from which to allocate the buffer
 *   timeout - The maximum time to wait for a buffer to become available.
 *
 * Returned Value:
 *   A reference to the allocated buffer, which is guaranteed to be zeroed.
 *   NULL is returned on a timeout.
 *
 ****************************************************************************/

FAR void *net_bufpool_timedalloc(FAR struct net_bufpool_s *pool,
                                 unsigned int timeout)
{
  FAR struct net_bufnode_s *node;
  int ret;
  int i;

  ret = net_sem_timedwait_uninterruptible(&pool->u.sem, timeout);
  if (ret != OK)
    {
      return NULL;
    }

  /* If we get here, then we didn't exceed maxalloc. */

  if (pool->dynalloc > 0 && sq_peek(&pool->freebuffers) == NULL)
    {
      node = kmm_zalloc(pool->nodesize * pool->dynalloc);
      if (node == NULL)
        {
          return NULL;
        }

      /* Now initialize each connection structure */

      for (i = 0; i < pool->dynalloc; i++)
        {
          sq_addlast(&node->node, &pool->freebuffers);
          node = (FAR struct net_bufnode_s *)
                                      ((FAR char *)node + pool->nodesize);
        }
    }

  return sq_remfirst(&pool->freebuffers);
}

/****************************************************************************
 * Name: net_bufpool_free
 *
 * Description:
 *   Free a buffer from the pool.
 *
 * Input Parameters:
 *   pool - The pool from which to allocate the buffer
 *   node - The buffer to be freed
 *
 ****************************************************************************/

void net_bufpool_free(FAR struct net_bufpool_s *pool, FAR void *node)
{
  if (pool->dynalloc == 1 &&
      ((FAR char *)node < pool->pool ||
       (FAR char *)node >= pool->pool + pool->prealloc * pool->nodesize))
    {
      kmm_free(node);
    }
  else
    {
      FAR struct net_bufnode_s *net_bufnode = node;

      /* Set the buffer to zero, to make sure all nodes in the free buffer
       * pool are zeroed.
       */

      memset(net_bufnode, 0, pool->nodesize);
      sq_addlast(&net_bufnode->node, &pool->freebuffers);
    }

  nxsem_post(&pool->u.sem);
}
