/****************************************************************************
 * sched/sched/queue.h
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

#ifndef __INCLUDE_SCHED_SCHED_NUTTX_QUEUE_H
#define __INCLUDE_SCHED_SCHED_NUTTX_QUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/queue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define dq_addfirst_nonempty(p, q) \
  do \
    { \
      FAR dq_entry_t *tmp_node = (p); \
      tmp_node->blink = NULL; \
      tmp_node->flink = (q)->head; \
      (q)->head->blink = tmp_node; \
      (q)->head = tmp_node; \
    } \
  while (0)

#define dq_rem_head(p, q) \
  do \
    { \
      FAR dq_entry_t *tmp_node = (p); \
      FAR dq_entry_t *tmp_next = tmp_node->flink; \
      (q)->head = tmp_next; \
      tmp_next->blink = NULL; \
      tmp_node->flink = NULL; \
    } \
  while (0)

#define dq_rem_mid(p) \
  do \
    { \
      FAR dq_entry_t *tmp_prev = (FAR dq_entry_t *)p->blink; \
      FAR dq_entry_t *tmp_next = (FAR dq_entry_t *)p->flink; \
      tmp_prev->flink = tmp_next; \
      tmp_next->blink = tmp_prev; \
    } \
  while (0)

#define dq_insert_mid(pre, mid, next) \
  do \
    { \
      mid->flink = next; \
      mid->blink = prev; \
      pre->flink = mid; \
      next->blink = mid; \
    } \
  while (0)

#endif /* __INCLUDE_NUTTX_QUEUE_H_ */
