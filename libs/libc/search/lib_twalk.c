/****************************************************************************
 * libs/libc/search/lib_twalk.c
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

#include "search/tsearch.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void twalk_node(FAR const struct tnode_s *node,
                       CODE void (*action)(FAR const void *, VISIT, int),
                       int depth)
{
  if (node->child[0] == NULL && node->child[1] == NULL)
    {
      action(node, leaf, depth);
      return;
    }

  action(node, preorder, depth);
  if (node->child[0] != NULL)
    {
      twalk_node(node->child[0], action, depth + 1);
    }

  action(node, postorder, depth);
  if (node->child[1] != NULL)
    {
      twalk_node(node->child[1], action, depth + 1);
    }

  action(node, endorder, depth);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: twalk
 *
 * Description:
 *   Call action for each node of the tree, depth first, left to right:
 *   preorder, postorder and endorder for a node with children, leaf for a
 *   node without.  The depth of the root is 0.
 *
 ****************************************************************************/

void twalk(FAR const void *root,
           CODE void (*action)(FAR const void *, VISIT, int))
{
  if (root != NULL && action != NULL)
    {
      twalk_node(root, action, 0);
    }
}
