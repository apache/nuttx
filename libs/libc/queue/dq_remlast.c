/****************************************************************************
 * libs/libc/queue/dq_remlast.c
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

#include <nuttx/queue.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/***************************************************(************************
 * Name: dq_remlast
 *
 * Description:
 *   dq_remlast removes the last entry from 'queue'
 *
 ****************************************************************************/

FAR dq_entry_t *dq_remlast(dq_queue_t *queue)
{
  FAR dq_entry_t *ret = queue->tail;

  if (ret)
    {
      FAR dq_entry_t *prev = ret->blink;
      if (!prev)
        {
          queue->head = NULL;
          queue->tail = NULL;
        }
      else
        {
          queue->tail = prev;
          prev->flink = NULL;
        }

      ret->flink = NULL;
      ret->blink = NULL;
    }

  return ret;
}
