/****************************************************************************
 * include/nuttx/notifier.h
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

#ifndef __INCLUDE_NUTTX_NOTIFIER_H
#define __INCLUDE_NUTTX_NOTIFIER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include <debug.h>
#include <errno.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ATOMIC_NOTIFIER_INIT(name) {NULL}

#define ATOMIC_NOTIFIER_HEAD(name) \
  struct atomic_notifier_head name = ATOMIC_NOTIFIER_INIT(name)

#define BLOCKING_NOTIFIER_INIT(name) { \
    NXMUTEX_INITIALIZER, \
    NULL \
  }

#define BLOCKING_NOTIFIER_HEAD(name) \
  struct blocking_notifier_head name = BLOCKING_NOTIFIER_INIT(name)

#define BLOCKING_INIT_NOTIFIER_HEAD(name) \
  do \
    { \
      nxmutex_init(&(name)->mutex); \
      (name)->head = NULL; \
    } \
  while (0)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct notifier_block;

typedef CODE int (*notifier_fn_t)(FAR struct notifier_block *nb,
                                  unsigned long action, FAR void *data);

struct notifier_block
{
  notifier_fn_t              notifier_call;
  FAR struct notifier_block *next;
  int                        priority;
};

struct atomic_notifier_head
{
  FAR struct notifier_block *head;
};

struct blocking_notifier_head
{
  mutex_t mutex;
  FAR struct notifier_block *head;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#define notifier_chain_register(nhead, node, unique_priority) \
  do \
    { \
      FAR struct notifier_block **nl = &(nhead); \
      FAR struct notifier_block *n = (node); \
      int flag = 0; \
      while (*nl != NULL) \
        { \
          if (*nl == n) \
            { \
              flag = -EEXIST; \
              break; \
            } \
          if (n->priority > (*nl)->priority)  \
            { \
              break; \
            } \
          if (n->priority == (*nl)->priority && (unique_priority)) \
            { \
              flag = -EBUSY; \
              break; \
            } \
          nl = &((*nl)->next); \
        } \
      if (flag == 0) \
        { \
          n->next = *nl; \
          *nl = n; \
        } \
    } \
  while (0)

#define notifier_chain_unregister(nhead, node) \
  do \
    { \
      FAR struct notifier_block **nl = &(nhead); \
      FAR struct notifier_block *n = (node); \
      while (*nl != NULL) \
        { \
          if (*nl == n) \
            { \
              *nl = n->next; \
              break; \
            } \
          nl = &((*nl)->next); \
        } \
    } \
  while(0)

#define notifier_call_chain(nhead, val, v, num_to_call, num_calls) \
  do \
    { \
      FAR struct notifier_block *nb; \
      FAR struct notifier_block *next_nb; \
      int nr_to_call = (num_to_call); \
      FAR int *nr_calls = (num_calls); \
      nb = (nhead); \
      while (nb && nr_to_call) \
        { \
          next_nb = nb->next; \
          nb->notifier_call(nb, (val), (v)); \
          if (nr_calls) \
            { \
              (*nr_calls)++; \
            } \
          nb = next_nb; \
          nr_to_call--; \
        } \
    } \
  while(0)

#define atomic_notifier_chain_register(nhead, nb) \
  do \
    { \
      FAR struct atomic_notifier_head *nh = (nhead); \
      irqstate_t flags; \
      flags = enter_critical_section(); \
      notifier_chain_register(nh->head, (nb), false); \
      leave_critical_section(flags); \
    } \
  while(0)

#define atomic_notifier_chain_register_uniqueprio(nhead, nb) \
  do \
    { \
      FAR struct atomic_notifier_head *nh = (nhead); \
      irqstate_t flags; \
      flags = enter_critical_section(); \
      notifier_chain_register(nh->head, (nb), true); \
      leave_critical_section(flags); \
    } \
  while(0)

#define atomic_notifier_chain_unregister(nhead, nb) \
  do \
    { \
      FAR struct atomic_notifier_head *nh = (nhead); \
      irqstate_t flags; \
      flags = enter_critical_section(); \
      notifier_chain_unregister(nh->head, (nb)); \
      leave_critical_section(flags); \
    } \
  while(0)

#define atomic_notifier_call_chain(nhead, val, v) \
  do \
    { \
      FAR struct atomic_notifier_head *nh = (nhead); \
      irqstate_t flags; \
      flags = enter_critical_section(); \
      notifier_call_chain(nh->head, (val), (v), -1, NULL); \
      leave_critical_section(flags); \
    } \
  while(0)

#define blocking_notifier_chain_register(nhead, nb) \
  do \
    { \
      FAR struct blocking_notifier_head *nh = (nhead); \
      if (nxmutex_lock(&nh->mutex) < 0) \
        { \
          break; \
        } \
      notifier_chain_register(nh->head, (nb), false); \
      nxmutex_unlock(&nh->mutex);\
    } \
  while(0)

#define blocking_notifier_chain_register_uniqueprio(nhead, nb) \
  do \
    { \
      FAR struct blocking_notifier_head *nh = (nhead); \
      if (nxmutex_lock(&nh->mutex) < 0) \
        { \
          break; \
        } \
      notifier_chain_register(nh->head, (nb), true); \
      nxmutex_unlock(&nh->mutex);\
    } \
  while(0)

#define blocking_notifier_chain_unregister(nhead, nb) \
  do \
    { \
      FAR struct blocking_notifier_head *nh = (nhead); \
      if (nxmutex_lock(&nh->mutex) < 0) \
        { \
          break; \
        } \
      notifier_chain_unregister(nh->head, (nb)); \
      nxmutex_unlock(&nh->mutex);\
    } \
  while(0)

#define blocking_notifier_call_chain(nhead, val, v) \
  do \
    { \
      FAR struct blocking_notifier_head *nh = (nhead); \
      if (nxmutex_lock(&nh->mutex) < 0) \
        { \
          break; \
        } \
      notifier_call_chain(nh->head, (val), (v), -1, NULL); \
      nxmutex_unlock(&nh->mutex);\
    } \
  while(0)

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __INCLUDE_NUTTX_NOTIFIER_H */
