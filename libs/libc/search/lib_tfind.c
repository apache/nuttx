/****************************************************************************
 * libs/libc/search/lib_tfind.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tfind
 *
 * Description:
 *   Find the node of key in the tree at *rootp.
 *
 * Returned Value:
 *   A pointer to the node, or NULL if the tree has no such key.
 *
 ****************************************************************************/

FAR void *tfind(FAR const void *key, FAR void * const *rootp,
                CODE int (*compar)(FAR const void *, FAR const void *))
{
  FAR struct tnode_s *node;
  int c;

  if (rootp == NULL)
    {
      return NULL;
    }

  for (node = *rootp; node != NULL; node = node->child[c > 0])
    {
      c = compar(key, node->key);
      if (c == 0)
        {
          return node;
        }
    }

  return NULL;
}
