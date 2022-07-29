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

#define list_add_after(entry, new_entry) list_add_head(entry, new_entry)
#define list_add_before(entry, new_entry) list_add_tail(entry, new_entry)

#define list_remove_head_type(list, type, element) (\
{\
  FAR struct list_node *__nod = list_remove_head(list);\
  FAR type *__t;\
\
  if(__nod)\
    {\
      __t = container_of(__nod, type, element);\
    }\
  else\
    {\
      __t = (type *)0;\
    }\
\
  __t;\
})

#define list_remove_tail_type(list, type, element) (\
{\
  FAR struct list_node *__nod = list_remove_tail(list);\
  FAR type *__t;\
\
  if(__nod)\
    {\
      __t = container_of(__nod, type, element);\
    }\
  else\
    {\
      __t = (type *)0;\
    }\
\
  __t;\
})

#define list_peek_head_type(list, type, element) (\
{\
  FAR struct list_node *__nod = list_peek_head(list);\
  FAR type *__t;\
\
  if(__nod)\
    {\
      __t = container_of(__nod, type, element);\
    }\
  else\
    {\
      __t = (type *)0;\
    }\
\
  __t;\
})

#define list_peek_tail_type(list, type, element) (\
{\
  FAR struct list_node *__nod = list_peek_tail(list);\
  FAR type *__t;\
\
  if(__nod)\
    {\
      __t = container_of(__nod, type, element);\
    }\
  else\
    {\
      __t = (type *)0;\
    }\
\
  __t;\
})

#define list_prev_type(list, item, type, element) (\
{\
  FAR struct list_node *__nod = list_prev(list, item);\
  FAR type *__t;\
\
  if(__nod)\
    {\
      __t = container_of(__nod, type, element);\
    }\
  else\
    {\
      __t = (type *)0;\
    }\
\
  __t;\
})

#define list_prev_wrap_type(list, item, type, element) (\
{\
  FAR struct list_node *__nod = list_prev_wrap(list, item);\
  FAR type *__t;\
\
  if(__nod)\
    {\
      __t = container_of(__nod, type, element);\
    }\
  else\
    {\
      __t = (type *)0;\
    }\
\
  __t;\
})

#define list_next_type(list, item, type, element) (\
{\
  FAR struct list_node *__nod = list_next(list, item);\
  FAR type *__t;\
\
  if(__nod)\
    {\
      __t = container_of(__nod, type, element);\
    }\
  else\
    {\
      __t = (type *)0;\
    }\
\
  __t;\
})

#define list_next_wrap_type(list, item, type, element) (\
{\
  FAR struct list_node *__nod = list_next_wrap(list, item);\
  FAR type *__t;\
\
  if(__nod)\
    {\
      __t = container_of(__nod, type, element);\
    }\
  else\
    {\
      __t = (type *)0;\
    }\
\
  __t;\
})

/* iterates over the list, node should be struct list_node* */

#define list_for_every(list, node) \
  for(node = (list)->next; node != (list); node = node->next)

/* iterates over the list in a safe way for deletion of current node
 * node and temp_node should be struct list_node*
 */

#define list_for_every_safe(list, node, temp_node) \
  for(node = (list)->next, temp_node = (node)->next;\
      node != (list);\
      node = temp_node, temp_node = (node)->next)

/* iterates over the list, entry should be the container structure type */

#define list_for_every_entry(list, entry, type, member) \
  for((entry) = container_of((list)->next, type, member);\
      &(entry)->member != (list);\
      (entry) = container_of((entry)->member.next, type, member))

/* iterates over the list in a safe way for deletion of current node
 * entry and temp_entry should be the container structure type *
 */

#define list_for_every_entry_safe(list, entry, temp_entry, type, member) \
  for(entry = container_of((list)->next, type, member),\
      temp_entry = container_of((entry)->member.next, type, member);\
      &(entry)->member != (list);\
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

static inline void list_initialize(FAR struct list_node *list)
{
  list->prev = list->next = list;
}

static inline void list_clear_node(FAR struct list_node *item)
{
  item->prev = item->next = 0;
}

static inline bool list_in_list(FAR struct list_node *item)
{
  if (item->prev == 0 && item->next == 0)
    {
      return false;
    }
  else
    {
      return true;
    }
}

static inline void list_add_head(FAR struct list_node *list,
                                 FAR struct list_node *item)
{
  item->next       = list->next;
  item->prev       = list;
  list->next->prev = item;
  list->next       = item;
}

static inline void list_add_tail(FAR struct list_node *list,
                                 FAR struct list_node *item)
{
  item->prev       = list->prev;
  item->next       = list;
  list->prev->next = item;
  list->prev       = item;
}

static inline void list_delete(struct list_node *item)
{
  item->next->prev = item->prev;
  item->prev->next = item->next;
  item->prev       = item->next = 0;
}

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

static inline FAR struct list_node *
list_peek_head(FAR struct list_node *list)
{
  if (list->next != list)
    {
      return list->next;
    }
  else
    {
      return NULL;
    }
}

static inline FAR struct list_node *
list_peek_tail(FAR struct list_node *list)
{
  if (list->prev != list)
    {
      return list->prev;
    }
  else
    {
      return NULL;
    }
}

static inline FAR struct list_node *list_prev(FAR struct list_node *list,
                                              FAR struct list_node *item)
{
  if (item->prev != list)
    {
      return item->prev;
    }
  else
    {
      return NULL;
    }
}

static inline FAR struct list_node *
list_prev_wrap(FAR struct list_node *list, FAR struct list_node *item)
{
  if (item->prev != list)
    {
      return item->prev;
    }
  else if (item->prev->prev != list)
    {
      return item->prev->prev;
    }
  else
    {
      return NULL;
    }
}

static inline FAR struct list_node *list_next(FAR struct list_node *list,
                                              FAR struct list_node *item)
{
  if (item->next != list)
    {
      return item->next;
    }
  else
    {
      return NULL;
    }
}

static inline FAR struct list_node *
list_next_wrap(FAR struct list_node *list, FAR struct list_node *item)
{
  if (item->next != list)
    {
      return item->next;
    }
  else if (item->next->next != list)
    {
      return item->next->next;
    }
  else
    {
      return NULL;
    }
}

static inline bool list_is_empty(FAR struct list_node *list)
{
  return list->next == list;
}

static inline bool list_is_clear(FAR struct list_node *list)
{
  return list->next == NULL;
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
