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

#include <nuttx/config.h>

#include <search.h>

#ifdef CONFIG_FDPIC
#  include <nuttx/fdpic.h>
#endif

#include "libc.h"
#include "search/tsearch.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tdestroy_node(FAR struct tnode_s *node,
                          CODE void (*free_node)(FAR void *))
{
  if (node == NULL)
    {
      return;
    }

  tdestroy_node(node->child[0], free_node);
  tdestroy_node(node->child[1], free_node);
  if (free_node != NULL)
    {
      free_node((FAR void *)node->key);
    }

  lib_free(node);
}

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
#ifdef CONFIG_FDPIC
  /* See qsort(): an FDPIC caller passes a descriptor, not a code address.
   * tdestroy_node() gets the resolved free_node.
   */

  free_node = (CODE void (*)(FAR void *))
              fdpic_callback((FAR void *)free_node);
#endif

  tdestroy_node(root, free_node);
}
