/****************************************************************************
 * include/nuttx/hrtimer/hrtimer_type_rb.h
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

#ifndef __INCLUDE_HRTIMER_TYPE_RB_H
#define __INCLUDE_HRTIMER_TYPE_RB_H

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

typedef struct hrtimer_rb_s hrtimer_internal_t;
typedef struct hrtimer_rb_queue_s hrtimer_queue_internal_t;

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

/* Compare function for the rb-tree. */

static inline_function
int hrtimer_compare(FAR const hrtimer_rb_t *a, FAR const hrtimer_rb_t *b)
{
  /* This branchless compare is equivalent to:
   * (int64_t)(a->expired - b->expired) > 0 ? 1 : -1;
   */

  return 1 - (HRTIMER_TIME_BEFORE_EQ(a->expired, b->expired) << 1u);
}

/* Generate red-black tree implementation for high-resolution timers
 * Do not inline the RB-Tree related functions to reduce code.
 */

RB_GENERATE_INTERNAL(hrtimer_tree_s, hrtimer_rb_s, node, hrtimer_compare,
                     noinline_function unused_code static inline)

/****************************************************************************
 * Name: hrtimer_rb_queue_peek
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
FAR hrtimer_rb_t *hrtimer_queue_peek(FAR hrtimer_rb_queue_t *queue)
{
  return queue->first;
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
bool hrtimer_queue_add(FAR hrtimer_rb_queue_t *queue,
                       FAR hrtimer_rb_t *timer)
{
  FAR hrtimer_rb_t *curr;
  bool           is_head = false;
  uint64_t       expired = timer->expired;

  curr = RB_INSERT(hrtimer_tree_s, &queue->root, timer);
  DEBUGASSERT(curr == NULL);

  if (HRTIMER_TIME_AFTER(queue->next_expired, expired))
    {
      queue->first        = timer;
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
FAR hrtimer_rb_t *hrtimer_queue_del(FAR hrtimer_rb_queue_t *queue,
                                    FAR hrtimer_rb_t *timer)
{
  FAR hrtimer_rb_t *head;

  DEBUGASSERT(hrtimer_in_queue(timer));

  RB_REMOVE(hrtimer_tree_s, &queue->root, timer);

  head = queue->first;

  /* Update the first node. */

  if (timer == head)
    {
      queue->first = RB_MIN(hrtimer_tree_s, &queue->root);
      queue->next_expired = queue->first->expired;
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
 *   The caller must hold the queue lock or ownership of the queue.
 *
 ****************************************************************************/

static inline_function
void hrtimer_queue_clear(FAR hrtimer_rb_queue_t *queue)
{
  RB_INIT(&queue->root);
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_HRTIMER_TYPE_RB_H */
