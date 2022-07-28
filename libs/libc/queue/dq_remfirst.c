/****************************************************************************
 * libs/libc/queue/dq_remfirst.c
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
 * Name: dq_remfirst
 *
 * Description:
 *   dq_remfirst removes 'node' from the head of 'queue'
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_INLINE_QUEUE
static inline
#endif
FAR dq_entry_t *dq_remfirst(dq_queue_t *queue)
{
  FAR dq_entry_t *ret = queue->head;

  if (ret)
    {
      FAR dq_entry_t *next = ret->flink;
      if (!next)
        {
          queue->head = NULL;
          queue->tail = NULL;
        }
      else
        {
          queue->head = next;
          next->blink = NULL;
        }

      ret->flink = NULL;
      ret->blink = NULL;
    }

  return ret;
}
