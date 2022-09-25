/****************************************************************************
 * include/nuttx/queue.h
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

#ifndef __INCLUDE_NUTTX_QUEUE_H
#define __INCLUDE_NUTTX_QUEUE_H

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

#define sq_addfirst(p, q) \
  do \
    { \
      FAR sq_entry_t *tmp_node = (p); \
      tmp_node->flink = (q)->head; \
      if (!(q)->head) \
        { \
          (q)->tail = tmp_node; \
        } \
      (q)->head = tmp_node; \
    } \
  while (0)

#define dq_addfirst(p, q) \
  do \
    { \
      FAR dq_entry_t *tmp_node = (p); \
      tmp_node->blink = NULL; \
      tmp_node->flink = (q)->head; \
      if (!(q)->head) \
        { \
          (q)->head = tmp_node; \
          (q)->tail = tmp_node; \
        } \
      else \
        { \
          (q)->head->blink = tmp_node; \
          (q)->head = tmp_node; \
        } \
    } \
  while (0)

#define sq_addlast(p, q) \
  do \
    { \
      FAR sq_entry_t *tmp_node = (p); \
      tmp_node->flink = NULL; \
      if (!(q)->head) \
        { \
          (q)->head = tmp_node; \
          (q)->tail = tmp_node; \
        } \
      else \
        { \
          (q)->tail->flink = tmp_node; \
          (q)->tail        = tmp_node; \
        } \
    } \
  while (0)

#define dq_addlast(p, q) \
  do \
    { \
      FAR dq_entry_t *tmp_node = (p); \
      tmp_node->flink = NULL; \
      tmp_node->blink = (q)->tail; \
      if (!(q)->head) \
        { \
          (q)->head = tmp_node; \
          (q)->tail = tmp_node; \
        } \
      else \
        { \
          (q)->tail->flink = tmp_node; \
          (q)->tail        = tmp_node; \
        } \
    } \
  while (0)

#define dq_addbefore(n, p, q) \
  do \
    { \
      FAR dq_entry_t *_tmp_node = (p); \
      if (!(q)->head || (n) == (q)->head) \
        { \
          dq_addfirst(_tmp_node, q); \
        } \
      else \
        { \
          FAR dq_entry_t *tmp_prev = (n)->blink; \
          _tmp_node->flink = (n); \
          _tmp_node->blink = tmp_prev; \
          tmp_prev->flink = _tmp_node; \
          (n)->blink = _tmp_node; \
        } \
    } \
  while (0)

#define sq_for_every(q, p) \
  for((p) = (q)->head; (p) != NULL; (p) = (p)->flink)

#define sq_rem(p, q) \
  do \
    { \
      FAR sq_entry_t *tmp_node = (p); \
      if ((q)->head && tmp_node) \
        { \
          if (tmp_node == (q)->head) \
            { \
              (q)->head = tmp_node->flink; \
              if (tmp_node == (q)->tail) \
                { \
                  (q)->tail = NULL; \
                } \
            } \
          else \
            { \
              FAR sq_entry_t *tmp_prev; \
              sq_for_every(q, tmp_prev) \
                { \
                  if (tmp_prev->flink == tmp_node) \
                    { \
                      sq_remafter(tmp_prev, q); \
                    } \
                } \
            } \
        } \
    } \
  while (0)

#define dq_rem(p, q) \
  do \
    { \
      FAR dq_entry_t *tmp_node = (p); \
      FAR dq_entry_t *tmp_prev = tmp_node->blink; \
      FAR dq_entry_t *tmp_next = tmp_node->flink; \
      if (!tmp_prev) \
        { \
          (q)->head = tmp_next; \
        } \
      else \
        { \
          tmp_prev->flink = tmp_next; \
        } \
      if (!tmp_next) \
        { \
          (q)->tail = tmp_prev; \
        } \
      else \
        { \
          tmp_next->blink = tmp_prev; \
        } \
      tmp_node->flink = NULL; \
      tmp_node->blink = NULL; \
    } \
  while (0)

#define sq_cat(q1, q2) \
  do \
    { \
      if (sq_empty(q2)) \
        { \
          sq_move(q1, q2); \
        } \
      else if (!sq_empty(q1)) \
        { \
          (q2)->tail->flink = (q1)->head; \
          (q2)->tail = (q1)->tail; \
          sq_init(q1); \
        } \
    } \
  while (0)

#define dq_cat(q1, q2) \
  do \
    { \
      if (dq_empty(q2)) \
        { \
          dq_move(q1, q2); \
        } \
      else if (!dq_empty(q1)) \
        { \
          (q2)->tail->flink = (q1)->head; \
          (q1)->head->blink = (q2)->tail; \
          (q2)->tail = (q1)->tail; \
          dq_init(q1); \
        } \
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

void sq_addafter(FAR sq_entry_t *prev, FAR sq_entry_t *node,
                 FAR sq_queue_t *queue);
void dq_addafter(FAR dq_entry_t *prev, FAR dq_entry_t *node,
                 FAR dq_queue_t *queue);

/* Remove nodes from queues */

FAR sq_entry_t *sq_remafter(FAR sq_entry_t *node, FAR sq_queue_t *queue);
FAR dq_entry_t *dq_remafter(FAR dq_entry_t *node, FAR dq_queue_t *queue);
FAR sq_entry_t *sq_remlast(FAR sq_queue_t *queue);
FAR dq_entry_t *dq_remlast(FAR dq_queue_t *queue);
FAR sq_entry_t *sq_remfirst(FAR sq_queue_t *queue);
FAR dq_entry_t *dq_remfirst(FAR dq_queue_t *queue);

/* Count nodes in queues */

size_t sq_count(FAR sq_queue_t *queue);
size_t dq_count(FAR dq_queue_t *queue);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_QUEUE_H_ */
