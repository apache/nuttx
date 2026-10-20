/****************************************************************************
 * libs/libc/search/lib_tsearch.c
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
 * Private Functions
 ****************************************************************************/

static int tsearch_height(FAR const struct tnode_s *node)
{
  return node != NULL ? node->height : 0;
}

static void tsearch_update(FAR struct tnode_s *node)
{
  int h0 = tsearch_height(node->child[0]);
  int h1 = tsearch_height(node->child[1]);

  node->height = (h0 > h1 ? h0 : h1) + 1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tsearch_balance
 ****************************************************************************/

int tsearch_balance(FAR struct tnode_s **link)
{
  FAR struct tnode_s *node = *link;
  FAR struct tnode_s *heavy;
  FAR struct tnode_s *inner;
  int old = node->height;
  int h0 = tsearch_height(node->child[0]);
  int h1 = tsearch_height(node->child[1]);
  int other;
  int dir;

  if (h0 - h1 <= 1 && h1 - h0 <= 1)
    {
      tsearch_update(node);
      return node->height != old;
    }

  /* dir is the heavy side.  An inner grandchild that is higher than the
   * outer one needs a double rotation.
   */

  dir   = h1 > h0;
  other = !dir;
  heavy = node->child[dir];
  inner = heavy->child[other];

  if (tsearch_height(inner) > tsearch_height(heavy->child[dir]))
    {
      heavy->child[other] = inner->child[dir];
      node->child[dir]    = inner->child[other];
      inner->child[dir]   = heavy;
      inner->child[other] = node;
      tsearch_update(heavy);
      tsearch_update(node);
      tsearch_update(inner);
      *link = inner;
    }
  else
    {
      node->child[dir]    = inner;
      heavy->child[other] = node;
      tsearch_update(node);
      tsearch_update(heavy);
      *link = heavy;
    }

  return (*link)->height != old;
}

/****************************************************************************
 * Name: tsearch
 *
 * Description:
 *   Find the node of key in the tree at *rootp; add one if there is none.
 *   The tree is an AVL tree.
 *
 * Returned Value:
 *   A pointer to the node (its first member is the key pointer), or NULL
 *   if there is no memory or rootp is NULL.
 *
 ****************************************************************************/

FAR void *tsearch(FAR const void *key, FAR void **rootp,
                  CODE int (*compar)(FAR const void *, FAR const void *))
{
  FAR struct tnode_s **path[TSEARCH_MAXHEIGHT + 1];
  FAR struct tnode_s **link;
  FAR struct tnode_s *node;
  int n = 0;
  int c;

  if (rootp == NULL)
    {
      return NULL;
    }

  link = (FAR struct tnode_s **)rootp;
  path[n++] = link;
  while (*link != NULL)
    {
      c = compar(key, (*link)->key);
      if (c == 0)
        {
          return *link;
        }

      link = &(*link)->child[c > 0];
      path[n++] = link;
    }

  node = lib_malloc(sizeof(*node));
  if (node == NULL)
    {
      return NULL;
    }

  node->key      = key;
  node->child[0] = NULL;
  node->child[1] = NULL;
  node->height   = 1;
  *link          = node;

  /* The ancestors, from the parent up, while their height changes */

  for (n -= 2; n >= 0 && tsearch_balance(path[n]); n--)
    {
    }

  return node;
}
