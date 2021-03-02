/****************************************************************************
 * libs/libc/queue/sq_cat.c
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

#include <assert.h>
#include <queue.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sq_cat
 *
 * Description:
 *   Move the content of queue1 to the end of queue2.
 *
 ****************************************************************************/

void sq_cat(FAR sq_queue_t *queue1, FAR sq_queue_t *queue2)
{
  DEBUGASSERT(queue1 != NULL && queue2 != NULL);

  /* If queue2 is empty, then just move queue1 to queue2 */

  if (sq_empty(queue2))
    {
      sq_move(queue1, queue2);
    }

  /* Do nothing if queue1 is empty */

  else if (!sq_empty(queue1))
    {
      /* Attach the head of queue1 to the final entry of queue2 */

      queue2->tail->flink = queue1->head;

      /* The tail of queue1 is the new tail of queue2 */

      queue2->tail = queue1->tail;
      sq_init(queue1);
    }
}
