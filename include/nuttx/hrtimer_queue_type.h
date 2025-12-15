/****************************************************************************
 * include/nuttx/hrtimer_queue_type.h
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

#ifndef __INCLUDE_HRTIMER_QUEUE_TYPE_H
#define __INCLUDE_HRTIMER_QUEUE_TYPE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>
#include <nuttx/clock.h>
#include <nuttx/seqlock.h>

#include <nuttx/list.h>
#include <sys/tree.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The pending state indicates the timer belongs to the shared hrtimer queue
 * and is waiting for the next hrtimer expiry.
 */

#define HRTIMER_ISPENDING(timer)       ((timer)->func != NULL)

/* The maximum delay tick should be INT64_MAX. However, if there are expired
 * hrtimer in the queue, HRTIMER_TIME_BEFORE/AFTER might be incorrect, so we
 * limited the delay to INT64_MAX >> 1, assuming all expired hrtimer can be
 * processed within HRTIMER_MAX_DELAY.
 */

#define HRTIMER_MAX_DELAY              (INT64_MAX >> 1)

#define HRTIMER_TIME_BEFORE(t1, t2)    ((int64_t)((t2) - (t1)) > 0)
#define HRTIMER_TIME_BEFORE_EQ(t1, t2) ((int64_t)((t2) - (t1)) >= 0)
#define HRTIMER_TIME_AFTER(t1, t2)     ((int64_t)((t2) - (t1)) < 0)
#define HRTIMER_TIME_AFTER_EQ(t1, t2)  ((int64_t)((t2) - (t1)) <= 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the form of the callback function that is called when the
 * hrtimer function expires. The return value is next delay time.
 * If the return value is not equal 0 indicates it is periodic timer.
 */

typedef CODE uint64_t (*hrtimer_callback_t)(FAR void *arg, uint64_t expired);

/* Type template for customed hrtimer. */

#define HRTIMER_TYPE_DECLARE(name, node_def, queue_def, lock_def) \
typedef struct name##_s \
{ \
  node_def \
  uint64_t           expired; /* Expired time */ \
  hrtimer_callback_t func;    /* Callback function */ \
  FAR void          *arg;     /* Callback argument */ \
} name##_t; \
typedef struct name##_queue_s \
{ \
  lock_def \
  queue_def \
  uint64_t  next_expired;              /* Next expired time */ \
  name##_t  guard_timer;               /* Guard timer for functional-safety. */ \
  uintptr_t running[CONFIG_SMP_NCPUS]; /* Hazard pointers for memory reclamation. */ \
} name##_queue_t;

/* Definition of the hrtimer_list_t. */

HRTIMER_TYPE_DECLARE(hrtimer_list,
                     struct list_node node;  /* Supports a doubly linked list, 16-bytes for 64-bit architectures */,
                     struct list_node queue; /* HRTimer doubly linked-list queue */,
                     seqcount_t lock;        /* The RW-lock */)

/* Definition of the hrtimer_rb_t. */

RB_HEAD(hrtimer_tree_s, hrtimer_rb_s);
HRTIMER_TYPE_DECLARE(hrtimer_rb,
                     RB_ENTRY(hrtimer_rb_s)   node;  /* Supports a RB-tree node, 32-bytes for 64-bit architectures */,
                     FAR struct hrtimer_rb_s *first; /* Cached left-most timer in the queue */
                     struct hrtimer_tree_s    root;  /* HRTimer red-black-tree-based queue */,
                     seqcount_t               lock;  /* The RW-lock */)

/****************************************************************************
 * Name: hrtimer_fill
 *
 * Description:
 *   Internal function to fill the timer.
 *
 * Input Parameters:
 *   timer    - The timer to be set.
 *   function - The callback function of the timer.
 *   argument - The argument of the callback function.
 *   time     - The expired time.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#define hrtimer_fill(timer, function, argument, time) \
  do { \
    DEBUGASSERT(timer); \
    (timer)->func    = (function); \
    (timer)->arg     = (argument); \
    (timer)->expired = (time); \
  } while(0)

#define hrtimer_init(timer) memset(timer, 0, sizeof(timer))

#endif /* __INCLUDE_HRTIMER_QUEUE_TYPE_H */
