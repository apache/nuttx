/****************************************************************************
 * include/nuttx/hrtimer/hrtimer_type_list.h
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

#ifndef __INCLUDE_HRTIMER_TYPE_LIST_H
#define __INCLUDE_HRTIMER_TYPE_LIST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/hrtimer_queue_type.h>

/* This header file should be only included for internal use,
 * DO NOT EXPOSE IT TO USERS.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

typedef struct hrtimer_list_s hrtimer_internal_t;
typedef struct hrtimer_list_queue_s hrtimer_queue_internal_t;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline function
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_queue_peek
 *
 * Description:
 *   Get the head hrtimer in the queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *
 * Returned Value:
 *   The head timer in the queue.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *
 ****************************************************************************/

static inline_function
FAR hrtimer_list_t *hrtimer_queue_peek(FAR hrtimer_list_queue_t *queue)
{
  return list_first_entry(&queue->queue, hrtimer_list_t, node);
}

/****************************************************************************
 * Name: hrtimer_queue_add
 *
 * Description:
 *   Add the hrtimer to the queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *   timer - The timer to be added.
 *
 * Returned Value:
 *   true if the timer is added to the head of the queue, otherwise false.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *   The caller must ensure that the timer is not in the queue.
 *
 ****************************************************************************/

static inline_function
bool hrtimer_queue_add(FAR hrtimer_list_queue_t *queue,
                       FAR hrtimer_list_t *timer)
{
  FAR hrtimer_list_t *curr;
  bool             is_head = false;
  uint64_t         expired = timer->expired;

  list_for_every_entry(&queue->queue, curr, hrtimer_list_t, node)
    {
      /* Until curr->expired has not timed out. */

      if (HRTIMER_TIME_AFTER(curr->expired, expired))
        {
          break;
        }
    }

  list_add_before(&curr->node, &timer->node);

  if (HRTIMER_TIME_AFTER(queue->next_expired, expired))
    {
      queue->next_expired = expired;
      is_head = true;
    }

  DEBUGASSERT(hrtimer_in_queue(timer));

  return is_head;
}

/****************************************************************************
 * Name: hrtimer_queue_del
 *
 * Description:
 *   Delete the hrtimer from the queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *   timer - The timer to be deleted.
 *
 * Returned Value:
 *   The previous head of the queue.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *   The caller must ensure that the timer is in the queue.
 *
 ****************************************************************************/

static inline_function
FAR hrtimer_list_t *hrtimer_queue_del(FAR hrtimer_list_queue_t *queue,
                                      FAR hrtimer_list_t *timer)
{
  FAR hrtimer_list_t *head;

  DEBUGASSERT(hrtimer_in_queue(timer));

  head = list_first_entry(&queue->queue, hrtimer_list_t, node);
  list_delete(&timer->node);

  /* Update the next_expired if the queue head changes. */

  if (head == timer)
    {
      queue->next_expired = hrtimer_queue_peek(queue)->expired;
    }

  return head;
}

/****************************************************************************
 * Name: hrtimer_queue_clear
 *
 * Description:
 *   Clear the hrtimer queue.
 *
 * Input Parameters:
 *   queue - The timer queue.
 *
 * Returned Value:
 *   None.
 *
 * Assumption:
 *   The caller must hold the queue lock.
 *
 ****************************************************************************/

static inline_function
void hrtimer_queue_clear(FAR hrtimer_list_queue_t *queue)
{
  list_initialize(&queue->queue);
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_HRTIMER_TYPE_LIST_H */
