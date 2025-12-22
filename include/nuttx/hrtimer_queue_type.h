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
 * and it can not be reclaimed until the timer is cancelled or completed.
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

#define HRTIMER_MODE_ABS               (0)
#define HRTIMER_MODE_REL               (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the form of the callback function that is called when the
 * hrtimer function expires. The return value is next delay time.
 * If the return value is not equal 0 indicates it is periodic timer.
 */

typedef CODE uint64_t (*hrtimer_callback_t)(FAR void *arg, uint64_t expired);

#define HRTIMER_TYPE_DECLARE(name, node_def, queue_def) \
typedef struct name##_s \
{ \
  node_def \
  uint64_t           expired; /* Expired time */ \
  hrtimer_callback_t func;    /* Callback function */ \
  FAR void          *arg;     /* Callback argument */ \
} name##_t; \
typedef struct name##_queue_s \
{ \
  seqcount_t         lock;                      /* The RW-lock */ \
  queue_def \
  uint64_t           next_expired;              /* Next expired time */ \
  name##_t           guard_timer;               /* Guard timer for functional-safety. */ \
  volatile uintptr_t running[CONFIG_SMP_NCPUS]; /* Hazard pointers for memory reclamation. */ \
} name##_queue_t;

/* Definition of the hrtimer_list_t. */

HRTIMER_TYPE_DECLARE(hrtimer_list,
                     struct list_node node;  /* Supports a doubly linked list, 16-bytes for 64-bit architectures */,
                     struct list_node queue; /* HRTimer doubly linked-list queue */)

/* Definition of the hrtimer_rb_t. */

RB_HEAD(hrtimer_tree_s, hrtimer_rb_s);
HRTIMER_TYPE_DECLARE(hrtimer_rb,
                     RB_ENTRY(hrtimer_rb_s)   node;  /* Supports a RB-tree node, 32-bytes for 64-bit architectures */,
                     FAR struct hrtimer_rb_s *first; /* Cached left-most timer in the queue */
                     struct hrtimer_tree_s    root;  /* HRTimer red-black-tree-based queue */)

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

/****************************************************************************
 * Public Template Macros
 ****************************************************************************/

/* These templates are designed for generating user-customizable instances.
 * We use templates instead of the function pointers because most
 * functional-safety compilers (E.g. GHC, Tasking and CompCert C) do not
 * support inlining function pointers, which would introduce additional
 * performance overhead.
 * Although the macro-based implementation reduces readability, but it is
 * worth for performance.
 *
 * Note that the template macros may violate MISRA-C Rule 20.10 Advisory.
 * To avoid this, we can expand the template macros to the user code and
 * remove the macro definitions.
 */

/* Declare the hrtimer queue prototype. */

#define HRTIMER_QUEUE_PROTOTYPE(name, hrtimer_type, current) \
int name##_restart_absolute(FAR hrtimer_type *timer, hrtimer_callback_t func, \
                            FAR void *arg, uint64_t expected); \
static inline_function \
int name##_restart(hrtimer_type *timer, hrtimer_callback_t func, \
                   FAR void *arg, uint64_t time, uint32_t mode) \
{ \
  uint64_t expired; \
  if (mode == HRTIMER_MODE_REL) \
    { \
      expired  = time <= HRTIMER_MAX_DELAY ? time : HRTIMER_MAX_DELAY; \
      expired += current(); \
    } \
  DEBUGASSERT(mode <= 1u); \
  return name##_restart_absolute(timer, func, arg, expired); \
} \
static inline_function \
int name##_start(FAR hrtimer_type *timer, hrtimer_callback_t func, \
                 FAR void *arg, uint64_t time, uint32_t mode) \
{ \
  int ret = -EINVAL; \
  DEBUGASSERT(mode <= 1u); \
  if (timer && func && !HRTIMER_ISPENDING(timer)) \
    { \
      ret = name##_restart(timer, func, arg, time, mode); \
    } \
  return ret; \
} \
int name##_async_cancel(FAR hrtimer_type *timer); \
int name##_cancel(FAR hrtimer_type *timer); \
uint64_t name##_gettime(FAR hrtimer_type *timer);

#define HRTIMER_QUEUE_PROTOTYPE_LIST(name, current) \
  typedef struct hrtimer_list_s name##_t; \
  typedef struct hrtimer_list_queue_s name##_queue_t; \
  HRTIMER_QUEUE_PROTOTYPE(name, hrtimer_list_t, current)

#define HRTIMER_QUEUE_PROTOTYPE_RB(name, current) \
  typedef struct hrtimer_rb_s name##_t; \
  typedef struct hrtimer_rb_queue_s name##_queue_t; \
  HRTIMER_QUEUE_PROTOTYPE(name, hrtimer_rb_t, current)

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_HRTIMER_QUEUE_TYPE_H */
