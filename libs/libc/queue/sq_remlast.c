/****************************************************************************
 * libs/libc/queue/sq_remlast.c
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
 * Name: sq_remlast
 *
 * Description:
 *   Removes the last entry in a singly-linked queue.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_INLINE_QUEUE
static inline
#endif
FAR sq_entry_t *sq_remlast(sq_queue_t *queue)
{
  FAR sq_entry_t *ret = queue->tail;

  if (ret)
    {
      if (queue->head == queue->tail)
        {
          queue->head = NULL;
          queue->tail = NULL;
        }
      else
        {
          FAR sq_entry_t *prev;
          for (prev = queue->head;
              prev && prev->flink != ret;
              prev = prev->flink);

          if (prev)
            {
              prev->flink = NULL;
              queue->tail = prev;
            }
        }

      ret->flink = NULL;
    }

  return ret;
}
