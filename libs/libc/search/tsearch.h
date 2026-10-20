/****************************************************************************
 * libs/libc/search/tsearch.h
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

#ifndef __LIBS_LIBC_SEARCH_TSEARCH_H
#define __LIBS_LIBC_SEARCH_TSEARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <search.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The height of an AVL tree is less than 1.45 * log2(n + 2) */

#define TSEARCH_MAXHEIGHT (sizeof(FAR void *) * 8 * 3 / 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A node of the tree.  POSIX: the key pointer is the first member, so a
 * pointer to a node is also a pointer to its key pointer.
 */

struct tnode_s
{
  FAR const void *key;
  FAR struct tnode_s *child[2];
  int height;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tsearch_balance
 *
 * Description:
 *   Update the height of the subtree at *link and rotate it if its two
 *   sides differ by more than one.  Return nonzero if the height of the
 *   subtree changed.
 *
 ****************************************************************************/

int tsearch_balance(FAR struct tnode_s **link);

#endif /* __LIBS_LIBC_SEARCH_TSEARCH_H */
