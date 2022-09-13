/****************************************************************************
 * libs/libc/queue/dq_remafter.c
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
 * Name: dq_remafter
 *
 * Description:
 *   dq_remafter removes the entry following 'node' from the 'queue'.
 *   Returns a reference to the removed entry.
 *
 ****************************************************************************/

FAR dq_entry_t *dq_remafter(FAR dq_entry_t *node, FAR dq_queue_t *queue)
{
  FAR dq_entry_t *ret = node->flink;

  if (queue->head != NULL && ret != NULL)
    {
      dq_rem(ret, queue);
    }

  return ret;
}
