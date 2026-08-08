/****************************************************************************
 * include/nuttx/plist.h
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

#ifndef __INCLUDE_NUTTX_PLIST_H
#define __INCLUDE_NUTTX_PLIST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>

#include <nuttx/nuttx.h>
#include <nuttx/list.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct plist_head
{
  struct list_node node_list;
};

struct plist_node
{
  uint32_t prio;
  struct list_node prio_list;
  struct list_node node_list;
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PLIST_HEAD_INIT - static struct plist_head initializer
 * head: struct plist_head variable name
 */

#define PLIST_HEAD_INIT(head)                     \
  {                                               \
    .node_list = LIST_HEAD_INIT((head).node_list) \
  }

/* PLIST_HEAD - declare and init plist_head
 * head: name for struct plist_head variable
 */

#define PLIST_HEAD(head) \
  struct plist_head head = PLIST_HEAD_INIT(head)

/* PLIST_NODE_INIT - static struct plist_node initializer
 * node: struct plist_node variable name
 * prio: initial node priority
 */

#define PLIST_NODE_INIT(node, val)                 \
  {                                                \
    .prio      = (val),                            \
    .prio_list = LIST_HEAD_INIT((node).prio_list), \
    .node_list = LIST_HEAD_INIT((node).node_list), \
  }

/* plist_head_init - dynamic struct plist_head initializer
 * head: struct plist_head pointer
 */

#define plist_head_init(head) \
  list_initialize(&(head)->node_list)

/* plist_node_init - Dynamic struct plist_node initializer
 * node: struct plist_node pointer
 * prio: initial node priority
 */

#define plist_node_init(node, val)         \
  do                                       \
    {                                      \
      (node)->prio = (val);                \
      list_initialize(&(node)->prio_list); \
      list_initialize(&(node)->node_list); \
    }                                      \
  while(0)

/* plist_for_each - iterate over the plist
 * pos:  the type * to use as a loop counter
 * head: the head for your list
 */

#define plist_for_each(pos, head) \
  list_for_every_entry(&(head)->node_list, pos, typeof(*pos), node_list)

/* plist_for_each_continue - continue iteration over the plist
 * pos:  the type * to use as a loop cursor
 * head: the head for your list
 *
 * Continue to iterate over plist, continuing after the current position.
 */

#define plist_for_each_continue(pos, head) \
  list_for_every_entry_continue(pos, &(head)->node_list, typeof(*pos), node_list)

/* plist_for_each_safe - iterate safely over a plist of given type
 * pos:  the type * to use as a loop counter
 * n:    another type * to use as temporary storage
 * head: the head for your list
 *
 * Iterate over a plist of given type, safe against removal of list entry.
 */

#define plist_for_each_safe(pos, n, head) \
  list_for_every_entry_safe(&(head)->node_list, pos, n, typeof(*pos), node_list)

/* plist_for_each_entry - iterate over list of given type
 * pos:  the type * to use as a loop counter
 * head: the head for your list
 * mem:  the name of the list_node within the struct
 */

#define plist_for_each_entry(pos, head, mem) \
  list_for_every_entry(&(head)->node_list, pos, typeof(*pos), mem.node_list)

/* plist_for_each_entry_continue - continue iteration over list of given type
 * pos:  the type * to use as a loop cursor
 * head: the head for your list
 * m:    the name of the list_node within the struct
 *
 * Continue to iterate over list of given type, continuing after
 * the current position.
 */
#define plist_for_each_entry_continue(pos, head, m) \
  list_for_every_entry_continue(pos, &(head)->node_list, typeof(*pos), m.node_list)

/* plist_for_each_entry_safe - iterate safely over list of given type
 * pos:  the type * to use as a loop counter
 * n:    another type * to use as temporary storage
 * head: the head for your list
 * m:    the name of the list_node within the struct
 *
 * Iterate over list of given type, safe against removal of list entry.
 */
#define plist_for_each_entry_safe(pos, n, head, m) \
  list_for_every_entry_safe(&(head)->node_list, pos, n, typeof(*pos), m.node_list)

/* All functions below assume the plist_head is not empty. */

/* plist_first_entry - get the struct for the first entry
 * head:   the struct plist_head pointer
 * type:   the type of the struct this is embedded in
 * member: the name of the list_node within the struct
 */

#define plist_first_entry(head, type, member) \
  container_of(plist_first(head), type, member)

/* plist_last_entry - get the struct for the last entry
 * head:   the struct plist_head pointer
 * type:   the type of the struct this is embedded in
 * member: the name of the list_node within the struct
 */

#define plist_last_entry(head, type, member) \
  container_of(plist_last(head), type, member)

/* plist_next - get the next entry in list
 * pos: the type * to cursor
 */

#define plist_next(pos) list_next_entry(pos, typeof(*(pos)), node_list)

/* plist_prev - get the prev entry in list
 * pos: the type * to cursor
 */

#define plist_prev(pos) list_prev_entry(pos, typeof(*(pos)), node_list)

/* plist_head_empty - return !0 if a plist_head is empty
 * head: struct plist_head pointer
 */

#define plist_head_empty(head) list_is_empty(&(head)->node_list)

/* plist_node_empty - return !0 if plist_node is not on a list
 * node: struct plist_node pointer
 */

#define plist_node_empty(node) list_is_empty(&(node)->node_list)

/* plist_first - return the first node (and thus, highest priority)
 * head: the struct plist_head pointer
 *
 * Assumes the plist is _not_ empty.
 */

#define plist_first(head) \
  list_entry((head)->node_list.next, struct plist_node, node_list)

/* plist_last - return the last node (and thus, lowest priority)
 * head: the struct plist_head pointer
 *
 * Assumes the plist is _not_ empty.
 */

#define plist_last(head) \
    list_entry((head)->node_list.prev, struct plist_node, node_list)

/* plist_add - add node to head
 * node: struct plist_node pointer
 * head: struct plist_head pointer
 */

#define plist_add(node, head)                                      \
  do                                                               \
    {                                                              \
      FAR struct plist_head *head_ = (head);                       \
      FAR struct plist_node *node_ = (node);                       \
      FAR struct list_node *node_next_ = &head_->node_list;        \
                                                                   \
      DEBUGASSERT(plist_node_empty(node_));                        \
      DEBUGASSERT(list_is_empty(&node_->prio_list));               \
                                                                   \
      if (!plist_head_empty(head_))                                \
        {                                                          \
          FAR struct plist_node *first_ = plist_first(head_);      \
          FAR struct plist_node *iter_ = first_;                   \
          FAR struct plist_node *prev_ = NULL;                     \
                                                                   \
          do                                                       \
            {                                                      \
              if (node_->prio < iter_->prio)                       \
                {                                                  \
                  node_next_ = &iter_->node_list;                  \
                  break;                                           \
                }                                                  \
                                                                   \
                prev_ = iter_;                                     \
                iter_ = list_entry(iter_->prio_list.next,          \
                                   struct plist_node, prio_list);  \
            }                                                      \
          while (iter_ != first_);                                 \
                                                                   \
          if (!prev_ || prev_->prio != node_->prio)                \
            {                                                      \
              list_add_tail(&iter_->prio_list, &node_->prio_list); \
            }                                                      \
        }                                                          \
                                                                   \
      list_add_tail(node_next_, &node_->node_list);                \
    }                                                              \
  while (0)

/* plist_del - Remove a node from plist.
 * node: struct plist_node pointer - entry to be removed
 * head: struct plist_head pointer - list head
 */

#define plist_del(node, head)                                          \
  do                                                                   \
    {                                                                  \
      FAR struct plist_head *head_ = (head);                           \
      FAR struct plist_node *node_ = (node);                           \
                                                                       \
      if (!list_is_empty(&node_->prio_list))                           \
        {                                                              \
          if (node_->node_list.next != &head_->node_list)              \
            {                                                          \
              FAR struct plist_node *next_ =                           \
                list_entry(node_->node_list.next,                      \
                           struct plist_node, node_list);              \
                                                                       \
              /* Add the next plist_node into prio_list */             \
                                                                       \
              if (list_is_empty(&next_->prio_list))                    \
                {                                                      \
                  list_add_head(&node_->prio_list, &next_->prio_list); \
                }                                                      \
            }                                                          \
                                                                       \
          list_delete_init(&node_->prio_list);                         \
        }                                                              \
                                                                       \
      list_delete_init(&node_->node_list);                             \
    }                                                                  \
  while (0)

/* plist_requeue - Requeue node at end of same-prio entries.
 *
 * This is essentially an optimized plist_del() followed by
 * plist_add().  It moves an entry already in the plist to
 * after any other same-priority entries.
 * node: struct plist_node pointer - entry to be moved
 * head: struct plist_head pointer - list head
 */

#define plist_requeue(node, head)                                   \
  do                                                                \
    {                                                               \
      FAR struct plist_head *head_ = (head);                        \
      FAR struct plist_node *node_ = (node);                        \
                                                                    \
      DEBUGASSERT(!plist_head_empty(head_));                        \
      DEBUGASSERT(!plist_node_empty(node_));                        \
                                                                    \
      if (node_ != plist_last(head_))                               \
        {                                                           \
          FAR struct plist_node *iter_ = plist_next(node_);         \
                                                                    \
          if (node_->prio == iter_->prio)                           \
            {                                                       \
              FAR struct list_node *node_next_ = &head_->node_list; \
                                                                    \
              plist_del(node_, head_);                              \
                                                                    \
              plist_for_each_continue(iter_, head_)                 \
                {                                                   \
                  if (node_->prio != iter_->prio)                   \
                    {                                               \
                      node_next_ = &iter_->node_list;               \
                      break;                                        \
                    }                                               \
                }                                                   \
                                                                    \
              list_add_tail(node_next_, &node_->node_list);         \
            }                                                       \
        }                                                           \
    }                                                               \
  while (0)

#endif /* __INCLUDE_NUTTX_PLIST_H */
