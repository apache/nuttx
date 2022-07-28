/****************************************************************************
 * libs/libc/queue/sq_addafter.c
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

#include <queue.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sq_addafter.c
 *
 * Description:
 *  The sq_addafter function adds 'node' after 'prev' in the 'queue.'
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_INLINE_QUEUE
static inline
#endif
void sq_addafter(FAR sq_entry_t *prev, FAR sq_entry_t *node,
                 sq_queue_t *queue)
{
  if (!queue->head || prev == queue->tail)
    {
      sq_addlast(node, queue);
    }
  else
    {
      node->flink = prev->flink;
      prev->flink = node;
    }
}
