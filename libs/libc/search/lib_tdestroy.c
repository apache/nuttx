/****************************************************************************
 * libs/libc/search/lib_tdestroy.c
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
 * Name: tdestroy
 *
 * Description:
 *   Free all nodes of the tree, and call free_node for the key of each
 *   (a GNU extension).
 *
 ****************************************************************************/

void tdestroy(FAR void *root, CODE void (*free_node)(FAR void *))
{
  FAR struct tnode_s *node = root;

  if (node == NULL)
    {
      return;
    }

  tdestroy(node->child[0], free_node);
  tdestroy(node->child[1], free_node);
  if (free_node != NULL)
    {
      free_node((FAR void *)node->key);
    }

  lib_free(node);
}
