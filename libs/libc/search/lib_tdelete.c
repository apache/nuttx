/****************************************************************************
 * libs/libc/search/lib_tdelete.c
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

#include <search.h>

#include "libc.h"
#include "search/tsearch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdelete
 *
 * Description:
 *   Remove the node of key from the tree at *rootp.  The other nodes stay
 *   where they are in memory: a node with two children is replaced by its
 *   in-order predecessor node, the keys do not move.
 *
 * Returned Value:
 *   A pointer to the parent of the removed node, rootp if it was the
 *   root, or NULL if the tree has no such key.
 *
 ****************************************************************************/

FAR void *tdelete(FAR const void *key, FAR void **rootp,
                  CODE int (*compar)(FAR const void *, FAR const void *))
{
  FAR struct tnode_s **path[TSEARCH_MAXHEIGHT + 1];
  FAR struct tnode_s **link;
  FAR struct tnode_s *node;
  FAR struct tnode_s *pred;
  FAR void *parent;
  int n = 0;
  int k;
  int c;

  if (rootp == NULL)
    {
      return NULL;
    }

  link = (FAR struct tnode_s **)rootp;
  path[n++] = link;
  while (*link != NULL && (c = compar(key, (*link)->key)) != 0)
    {
      link = &(*link)->child[c > 0];
      path[n++] = link;
    }

  node = *link;
  if (node == NULL)
    {
      return NULL;
    }

  k = n - 1;
  parent = k > 0 ? (FAR void *)*path[k - 1] : (FAR void *)rootp;

  if (node->child[0] != NULL && node->child[1] != NULL)
    {
      /* Find the predecessor: the rightmost node of the left subtree */

      link = &node->child[0];
      path[n++] = link;
      while ((*link)->child[1] != NULL)
        {
          link = &(*link)->child[1];
          path[n++] = link;
        }

      /* Unlink it and put it where the node was */

      pred           = *link;
      *link          = pred->child[0];
      pred->child[0] = node->child[0];
      pred->child[1] = node->child[1];
      pred->height   = node->height;
      *path[k]       = pred;

      /* The path went through the removed node */

      path[k + 1] = &pred->child[0];
    }
  else
    {
      *link = node->child[node->child[0] == NULL];
    }

  lib_free(node);

  /* Rebalance the ancestors of the changed link, while their height
   * changes.
   */

  for (n -= 2; n >= 0 && tsearch_balance(path[n]); n--)
    {
    }

  return parent;
}
