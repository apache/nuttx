/****************************************************************************
 * libs/libc/misc/lib_idr.c
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

#include <nuttx/lib/lib.h>
#include <nuttx/idr.h>
#include <sys/tree.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

RB_HEAD(idr_tree_s, idr_node_s);

struct idr_node_s
{
  RB_ENTRY(idr_node_s) link;
  unsigned int id;
  FAR void *data;
};

struct idr_s
{
  mutex_t lock;
  unsigned int lastid;
  unsigned int base;
  struct idr_tree_s alloced;
  struct idr_tree_s removed;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: idr_compare
 ****************************************************************************/

static int idr_compare(FAR struct idr_node_s *node,
                       FAR struct idr_node_s *root)
{
  /* When node < root, it should be placed on the left side of the tree */

  return node->id - root->id;
}

/****************************************************************************
 * Name: idr_create_node
 ****************************************************************************/

FAR static struct idr_node_s *idr_create_node(FAR void *ptr, unsigned int id)
{
  FAR struct idr_node_s *node;

  node = lib_malloc(sizeof(struct idr_node_s));
  if (node == NULL)
    {
      return NULL;
    }

  node->id = id;
  node->data = ptr;
  return node;
}

/****************************************************************************
 * Name: RB_GENERATE_STATIC
 ****************************************************************************/

RB_GENERATE_STATIC(idr_tree_s, idr_node_s, link, idr_compare)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: idr_find
 ****************************************************************************/

FAR void *idr_find(FAR struct idr_s *idr, unsigned int id)
{
  FAR struct idr_node_s *node;
  struct idr_node_s search;

  search.id = id;
  nxmutex_lock(&idr->lock);
  node = RB_FIND(idr_tree_s, &idr->alloced, &search);
  if (node == NULL)
    {
      nxmutex_unlock(&idr->lock);
      return NULL;
    }

  nxmutex_unlock(&idr->lock);
  return node->data;
}

/****************************************************************************
 * Name: idr_get_next
 ****************************************************************************/

FAR void *idr_get_next(FAR struct idr_s *idr, FAR int *id)
{
  FAR struct idr_node_s *node;
  struct idr_node_s search;

  search.id = *id;
  nxmutex_lock(&idr->lock);
  node = RB_NFIND(idr_tree_s, &idr->alloced, &search);
  if (node == NULL)
    {
      nxmutex_unlock(&idr->lock);
      return NULL;
    }

  /* Update id */

  *id = node->id;
  nxmutex_unlock(&idr->lock);
  return node->data;
}

/****************************************************************************
 * Name: idr_remove
 ****************************************************************************/

FAR void *idr_remove(FAR struct idr_s *idr, unsigned int id)
{
  FAR struct idr_node_s *node;
  struct idr_node_s search;
  FAR void *ptr;

  search.id = id;
  nxmutex_lock(&idr->lock);
  node = RB_FIND(idr_tree_s, &idr->alloced, &search);
  if (node == NULL)
    {
      /* We did not find the corresponding node from the allocated tree */

      nxmutex_unlock(&idr->lock);
      return NULL;
    }

  ptr = node->data;
  node->data = NULL;

  /* We move the node from the allocated tree to the removed tree so that
   * it can be used the next time it is allocated to avoid multiple
   * allocations and reduce efficiency.
   */

  RB_REMOVE(idr_tree_s, &idr->alloced, node);
  RB_INSERT(idr_tree_s, &idr->removed, node);
  nxmutex_unlock(&idr->lock);
  return ptr;
}

/****************************************************************************
 * Name: idr_alloc_u32
 ****************************************************************************/

int idr_alloc_u32(FAR struct idr_s *idr, FAR void *ptr, FAR uint32_t *result,
                  uint32_t max)
{
  FAR struct idr_node_s *node;
  FAR struct idr_node_s *temp;
  struct idr_node_s search;

  nxmutex_lock(&idr->lock);

  /* Check ID Value */

  if (*result < idr->base)
    {
      /* If it is less than the set base, then the search starts from
       * the base.
       */

      *result = idr->base;
    }

  /* Check if there are available blocks from the removed tree */

  RB_FOREACH_SAFE(node, idr_tree_s, &idr->removed, temp)
    {
      if (node->id >= *result && node->id <= max)
        {
          RB_REMOVE(idr_tree_s, &idr->removed, node);
          node->data = ptr;
          *result = node->id;
          RB_INSERT(idr_tree_s, &idr->alloced, node);
          goto out;
        }
    }

  /* Branch prediction: do we check if the saved lastid is in range?
   * This can improve performance.
   */

  if (idr->lastid >= *result && idr->lastid < max)
    {
      search.id = ++idr->lastid;
      temp = RB_FIND(idr_tree_s, &idr->alloced, &search);
      if (temp == NULL)
        {
          /* We can use this value, reduced the traversal search
           * available id
           */

          *result = search.id;
          goto create;
        }
    }

  /* Cache miss, then we need to traverse the tree */

  for (; *result <= max; (*result)++)
    {
      search.id = *result;
      temp = RB_FIND(idr_tree_s, &idr->alloced, &search);
      if (temp == NULL)
        {
          /* We found it */

          goto create;
        }
    }

  /* There are no available IDs in this range. */

  nxmutex_unlock(&idr->lock);
  return -ENOSPC;

create:
  node = idr_create_node(ptr, *result);
  if (node == NULL)
    {
      nxmutex_unlock(&idr->lock);
      return -ENOMEM;
    }

  RB_INSERT(idr_tree_s, &idr->alloced, node);

  /* Update lastid */

  idr->lastid = *result;
out:
  nxmutex_unlock(&idr->lock);
  return OK;
}

/****************************************************************************
 * Name: idr_alloc
 ****************************************************************************/

int idr_alloc(FAR struct idr_s *idr, FAR void *ptr, int start, int end)
{
  uint32_t id = start;
  int ret;

  if (start < 0)
    {
      return -EINVAL;
    }

  ret = idr_alloc_u32(idr, ptr, &id, end > 0 ? end - 1 : INT32_MAX);
  if (ret < 0)
    {
      return ret;
    }

  return id;
}

/****************************************************************************
 * Name: idr_init_base
 ****************************************************************************/

FAR struct idr_s *idr_init_base(int base)
{
  FAR struct idr_s *idr;

  idr = lib_zalloc(sizeof(struct idr_s));
  if (idr == NULL)
    {
      return NULL;
    }

  idr->base = base;
  RB_INIT(&idr->alloced);
  RB_INIT(&idr->removed);
  nxmutex_init(&idr->lock);
  return idr;
}

/****************************************************************************
 * Name: idr_destroy
 ****************************************************************************/

void idr_destroy(FAR struct idr_s *idr)
{
  FAR struct idr_node_s *node;
  FAR struct idr_node_s *temp;

  /* Release the space occupied by each node in the RB tree */

  nxmutex_lock(&idr->lock);
  RB_FOREACH_SAFE(node, idr_tree_s, &idr->removed, temp)
    {
      lib_free(node);
    }

  RB_FOREACH_SAFE(node, idr_tree_s, &idr->alloced, temp)
    {
      lib_free(node);
    }

  nxmutex_unlock(&idr->lock);
  nxmutex_destroy(&idr->lock);
  lib_free(idr);
}
