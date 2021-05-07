/****************************************************************************
 * include/queue.h
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

#ifndef __INCLUDE_QUEUE_H
#define __INCLUDE_QUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define sq_init(q) \
  do \
    { \
      (q)->head = NULL; \
      (q)->tail = NULL; \
    } \
  while (0)

#define dq_init(q) \
  do \
    { \
      (q)->head = NULL; \
      (q)->tail = NULL; \
    } \
  while (0)

#define sq_move(q1,q2) \
  do \
    { \
      (q2)->head = (q1)->head; \
      (q2)->tail = (q1)->tail; \
      (q1)->head = NULL; \
      (q1)->tail = NULL; \
    } \
  while (0)

#define dq_move(q1,q2) \
  do \
    { \
      (q2)->head = (q1)->head; \
      (q2)->tail = (q1)->tail; \
      (q1)->head = NULL; \
      (q1)->tail = NULL; \
    } \
  while (0)

#define sq_next(p)  ((p)->flink)
#define dq_next(p)  ((p)->flink)
#define dq_prev(p)  ((p)->blink)

#define sq_empty(q) ((q)->head == NULL)
#define dq_empty(q) ((q)->head == NULL)

#define sq_peek(q)  ((q)->head)
#define dq_peek(q)  ((q)->head)

#define sq_tail(q)  ((q)->tail)
#define dq_tail(q)  ((q)->tail)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct sq_entry_s
{
  FAR struct sq_entry_s *flink;
};
typedef struct sq_entry_s sq_entry_t;

struct dq_entry_s
{
  FAR struct dq_entry_s *flink;
  FAR struct dq_entry_s *blink;
};
typedef struct dq_entry_s dq_entry_t;

struct sq_queue_s
{
  FAR sq_entry_t *head;
  FAR sq_entry_t *tail;
};
typedef struct sq_queue_s  sq_queue_t;

struct dq_queue_s
{
  FAR dq_entry_t *head;
  FAR dq_entry_t *tail;
};
typedef struct dq_queue_s dq_queue_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Add nodes to queues */

void sq_addfirst(FAR sq_entry_t *node, FAR sq_queue_t *queue);
void dq_addfirst(FAR dq_entry_t *node, FAR dq_queue_t *queue);
void sq_addlast(FAR sq_entry_t *node, FAR sq_queue_t *queue);
void dq_addlast(FAR dq_entry_t *node, FAR dq_queue_t *queue);
void sq_addafter(FAR sq_entry_t *prev, FAR sq_entry_t *node,
                 FAR sq_queue_t *queue);
void dq_addafter(FAR dq_entry_t *prev, FAR dq_entry_t *node,
                 FAR dq_queue_t *queue);
void dq_addbefore(FAR dq_entry_t *next, FAR dq_entry_t *node,
                  FAR dq_queue_t *queue);

/* Combine queues */

void sq_cat(FAR sq_queue_t *queue1, FAR sq_queue_t *queue2);
void dq_cat(FAR dq_queue_t *queue1, FAR dq_queue_t *queue2);

/* Remove nodes from queues */

FAR  sq_entry_t *sq_remafter(FAR sq_entry_t *node, FAR sq_queue_t *queue);
void sq_rem(FAR sq_entry_t *node, FAR sq_queue_t *queue);
void dq_rem(FAR dq_entry_t *node, FAR dq_queue_t *queue);
FAR  sq_entry_t *sq_remlast(FAR sq_queue_t *queue);
FAR  dq_entry_t *dq_remlast(FAR dq_queue_t *queue);
FAR  sq_entry_t *sq_remfirst(FAR sq_queue_t *queue);
FAR  dq_entry_t *dq_remfirst(FAR dq_queue_t *queue);

/* Count nodes in queues */

size_t sq_count(FAR sq_queue_t *queue);
size_t dq_count(FAR dq_queue_t *queue);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_QUEUE_H_ */
