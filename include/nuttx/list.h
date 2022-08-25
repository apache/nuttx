/****************************************************************************
 * include/nuttx/list.h
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

#ifndef __INCLUDE_NUTTX_LIST_H
#define __INCLUDE_NUTTX_LIST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/nuttx.h>

#include <stddef.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LIST_INITIAL_VALUE(list) { &(list), &(list) }
#define LIST_INITIAL_CLEARED_VALUE { NULL, NULL }

#define list_in_list(item) ((item)->prev || (item)->next)
#define list_is_empty(list) ((list)->next == list)
#define list_is_clear(list) ((list)->next == NULL)

#define list_initialize(list) \
  do \
    { \
      (list)->prev = (list)->next = (list); \
    } \
  while(0)

#define list_clear_node(item) \
  do \
    { \
      (item)->prev = (item)->next = NULL; \
    } \
  while (0)

#define list_peek_head(list) ((list)->next != (list) ? (list)->next : NULL)
#define list_peek_tail(list) ((list)->prev != (list) ? (list)->prev : NULL)

#define list_prev(list, item) ((item)->prev != (list) ? (item)->prev : NULL)
#define list_prev_wrap(list, item) \
  ((item)->prev != (list) ? (item)->prev : \
   (item)->prev->prev != (list) ? (item)->prev->prev : NULL)

#define list_next(list, item) ((item)->next != (list) ? (item)->next : NULL)
#define list_next_wrap(list, item) \
  ((item)->next != (list) ? (item)->next : \
   (item)->next->next != (list) ? (item)->next->next : NULL)

/**
 * list_entry - get the struct for this entry
 * @ptr: the &struct list_head pointer.
 * @type: the type of the struct this is embedded in.
 * @member: the name of the list_head within the struct.
 */
#define list_entry(ptr, type, member) \
           container_of(ptr, type, member)

/**
 * list_first_entry - get the first element from a list
 * @ptr: the list head to take the element from.
 * @type: the type of the struct this is embedded in.
 * @member: the name of the list_head within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define list_first_entry(ptr, type, member) \
             list_entry((ptr)->next, type, member)

/**
 * list_last_entry - get the last element from a list
 * @ptr: the list head to take the element from.
 * @type: the type of the struct this is embedded in.
 * @member: the name of the list_head within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define list_last_entry(ptr, type, member) \
           list_entry((ptr)->prev, type, member)

#define list_add_after(entry, new_entry) list_add_head(entry, new_entry)
#define list_add_head(list, item) \
  do \
    { \
      (item)->next       = (list)->next; \
      (item)->prev       = (list); \
      (list)->next->prev = (item); \
      (list)->next       = (item); \
    } \
  while (0)

#define list_add_before(entry, new_entry) list_add_tail(entry, new_entry)
#define list_add_tail(list, item) \
  do \
    { \
      (item)->prev       = (list)->prev; \
      (item)->next       = (list); \
      (list)->prev->next = (item); \
      (list)->prev       = (item); \
    } \
  while (0)

#define list_delete(item) \
  do \
    { \
      (item)->next->prev = (item)->prev; \
      (item)->prev->next = (item)->next; \
      (item)->prev       = (item)->next = NULL; \
    } \
  while (0)

#define list_remove_head_type(list, type, element) ( \
{\
  FAR struct list_node *__nod = list_remove_head(list); \
  FAR type *__t; \
\
  if(__nod) \
    { \
      __t = container_of(__nod, type, element); \
    } \
  else \
    {\
      __t = (FAR type *)NULL; \
    } \
\
  __t; \
})

#define list_remove_tail_type(list, type, element) ( \
{\
  FAR struct list_node *__nod = list_remove_tail(list); \
  FAR type *__t; \
\
  if(__nod) \
    { \
      __t = container_of(__nod, type, element); \
    } \
  else\
    { \
      __t = (FAR type *)NULL; \
    } \
\
  __t; \
})

#define list_peek_head_type(list, type, element) ( \
{\
  FAR struct list_node *__nod = list_peek_head(list); \
  FAR type *__t; \
\
  if(__nod) \
    { \
      __t = container_of(__nod, type, element); \
    } \
  else \
    { \
      __t = (FAR type *)NULL; \
    } \
\
  __t; \
})

#define list_peek_tail_type(list, type, element) ( \
{ \
  FAR struct list_node *__nod = list_peek_tail(list); \
  FAR type *__t; \
\
  if(__nod) \
    { \
      __t = container_of(__nod, type, element); \
    } \
  else\
    { \
      __t = (FAR type *)NULL; \
    } \
\
  __t; \
})

#define list_prev_type(list, item, type, element) ( \
{ \
  FAR struct list_node *__nod = list_prev(list, item); \
  FAR type *__t; \
\
  if(__nod) \
    { \
      __t = container_of(__nod, type, element); \
    } \
  else \
    { \
      __t = (FAR type *)NULL; \
    } \
\
  __t; \
})

#define list_prev_wrap_type(list, item, type, element) ( \
{ \
  FAR struct list_node *__nod = list_prev_wrap(list, item); \
  FAR type *__t; \
\
  if(__nod) \
    { \
      __t = container_of(__nod, type, element); \
    } \
  else \
    { \
      __t = (FAR type *)NULL; \
    } \
\
  __t; \
})

#define list_next_type(list, item, type, element) ( \
{ \
  FAR struct list_node *__nod = list_next(list, item); \
  FAR type *__t; \
\
  if(__nod) \
    { \
      __t = container_of(__nod, type, element); \
    } \
  else \
    { \
      __t = (FAR type *)NULL; \
    } \
\
  __t; \
})

#define list_next_wrap_type(list, item, type, element) ( \
{\
  FAR struct list_node *__nod = list_next_wrap(list, item); \
  FAR type *__t; \
\
  if(__nod) \
    { \
      __t = container_of(__nod, type, element); \
    } \
  else \
    { \
      __t = (FAR type *)NULL; \
    } \
\
  __t; \
})

/* iterates over the list, node should be struct list_node* */

#define list_for_every(list, node) \
  for(node = (list)->next; node != (list); node = node->next)

/* iterates over the list in a safe way for deletion of current node
 * node and temp_node should be struct list_node*
 */

#define list_for_every_safe(list, node, temp_node) \
  for(node = (list)->next, temp_node = (node)->next; \
      node != (list); \
      node = temp_node, temp_node = (node)->next)

/* iterates over the list, entry should be the container structure type */

#define list_for_every_entry(list, entry, type, member) \
  for((entry) = container_of((list)->next, type, member); \
      &(entry)->member != (list); \
      (entry) = container_of((entry)->member.next, type, member))

/* iterates over the list in a safe way for deletion of current node
 * entry and temp_entry should be the container structure type *
 */

#define list_for_every_entry_safe(list, entry, temp_entry, type, member) \
  for(entry = container_of((list)->next, type, member), \
      temp_entry = container_of((entry)->member.next, type, member); \
      &(entry)->member != (list); \
      entry = temp_entry, \
      temp_entry = container_of((temp_entry)->member.next, type, member))

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct list_node
{
  FAR struct list_node *prev;
  FAR struct list_node *next;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline FAR struct list_node *
list_remove_head(FAR struct list_node *list)
{
  if (list->next != list)
    {
      FAR struct list_node *item = list->next;
      list_delete(item);
      return item;
    }
  else
    {
      return NULL;
    }
}

static inline FAR struct list_node *
list_remove_tail(FAR struct list_node *list)
{
  if (list->prev != list)
    {
      FAR struct list_node *item = list->prev;
      list_delete(item);
      return item;
    }
  else
    {
      return NULL;
    }
}

static inline size_t list_length(FAR struct list_node *list)
{
  FAR struct list_node *node = list;
  size_t cnt = 0;

  list_for_every(list, node)
    {
      cnt++;
    }

  return cnt;
}

#endif /* __INCLUDE_NUTTX_LIST_H */
