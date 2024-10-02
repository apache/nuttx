/****************************************************************************
 * include/nuttx/list.h
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * SPDX-FileCopyrightText: 2008 Travis Geiselbrecht. All rights reserved.
 * SPDX-FileContributor: Travis Geiselbrecht <geist@foobox.com>
 *
 * Extracted from logic originally written by Travis Geiselbrecht and
 * released under a public domain license.  Re-released here under the 3-
 * clause BSD license by Pinecone, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LIST_H
#define __INCLUDE_NUTTX_LIST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Name: list_container_of
 *
 * Description:
 *   Cast a member of a structure out to get the address of the containing
 *   structure
 *
 * Arguments:
 *   ptr    - The pointer to the member.
 *   type   - The type of the container struct this is embedded in.
 *   member - The name of the member within the struct.
 */

#define list_container_of(ptr, type, member) \
  ((type *)((uintptr_t)(ptr) - offsetof(type, member)))

#define LIST_INITIAL_VALUE(list) { &(list), &(list) }
#define LIST_INITIAL_CLEARED_VALUE { NULL, NULL }

#define list_in_list(item)     ((item)->prev != NULL)
#define list_is_empty(list)    ((list)->next == list)
#define list_is_clear(list)    ((list)->next == NULL)
#define list_is_singular(list) ((list)->next == (list)->prev)

#define list_initialize(list) \
  do \
    { \
      FAR struct list_node *__list = (list); \
      __list->prev = __list->next = __list; \
    } \
  while(0)

#define list_clear_node(item) \
  do \
    { \
      FAR struct list_node *__item = (item); \
      __item->prev = __item->next = NULL; \
    } \
  while (0)

#define list_is_head(list, item) ((list)->next == (item))
#define list_is_tail(list, item) ((list)->prev == (item))
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

#define list_entry(ptr, type, member) list_container_of(ptr, type, member)
#define list_first_entry(list, type, member) list_container_of((list)->next, type, member)
#define list_last_entry(list, type, member) list_container_of((list)->prev, type, member)
#define list_next_entry(list, type, member) list_container_of((list)->member.next, type, member)
#define list_prev_entry(list, type, member) list_container_of((list)->member.prev, type, member)

#define list_add_after(entry, new_entry) list_add_head(entry, new_entry)
#define list_add_head(list, item) \
  do \
    { \
      FAR struct list_node *__list = (list); \
      FAR struct list_node *__item = (item); \
      __item->next       = __list->next; \
      __item->prev       = __list; \
      __list->next->prev = __item; \
      __list->next       = __item; \
    } \
  while (0)

#define list_add_before(entry, new_entry) list_add_tail(entry, new_entry)
#define list_add_tail(list, item) \
  do \
    { \
      FAR struct list_node *__list = (list); \
      FAR struct list_node *__item = (item); \
      __item->prev       = __list->prev; \
      __item->next       = __list; \
      __list->prev->next = __item; \
      __list->prev       = __item; \
    } \
  while (0)

#define list_delete(item) \
  do \
    { \
      FAR struct list_node *__item = (item); \
      __item->next->prev = __item->prev; \
      __item->prev->next = __item->next; \
      __item->prev = __item->next = NULL; \
    } \
  while (0)

#define list_delete_init(item) \
  do \
    { \
      list_delete(item); \
      list_initialize(item); \
    } \
  while (0)

#define list_remove_head_type(list, type, member) \
  ({ \
    FAR struct list_node *__node = list_remove_head(list); \
    FAR type *__t = NULL; \
    if(__node) \
      { \
        __t = list_container_of(__node, type, member); \
      } \
    __t; \
  })

#define list_remove_tail_type(list, type, member) \
  ({ \
    FAR struct list_node *__node = list_remove_tail(list); \
    FAR type *__t = NULL; \
    if(__node) \
      { \
        __t = list_container_of(__node, type, member); \
      } \
    __t; \
  })

#define list_peek_head_type(list, type, member) \
  ({ \
    FAR struct list_node *__node = list_peek_head(list); \
    FAR type *__t = NULL; \
    if(__node) \
      { \
        __t = list_container_of(__node, type, member); \
      } \
    __t; \
  })

#define list_peek_tail_type(list, type, member) \
  ({ \
    FAR struct list_node *__node = list_peek_tail(list); \
    FAR type *__t = NULL; \
    if(__node) \
      { \
        __t = list_container_of(__node, type, member); \
      } \
    __t; \
  })

#define list_prev_type(list, item, type, member) \
  ({ \
    FAR struct list_node *__node = list_prev(list, item); \
    FAR type *__t = NULL; \
    if(__node) \
      { \
        __t = list_container_of(__node, type, member); \
      } \
    __t; \
  })

#define list_prev_wrap_type(list, item, type, member) \
  ({ \
    FAR struct list_node *__node = list_prev_wrap(list, item); \
    FAR type *__t = NULL; \
    if(__node) \
      { \
        __t = list_container_of(__node, type, member); \
      } \
    __t; \
  })

#define list_next_type(list, item, type, member) \
  ({ \
    FAR struct list_node *__node = list_next(list, item); \
    FAR type *__t = NULL; \
    if(__node) \
      { \
        __t = list_container_of(__node, type, member); \
      } \
    __t; \
  })

#define list_next_wrap_type(list, item, type, member) \
  ({ \
    FAR struct list_node *__node = list_next_wrap(list, item); \
    FAR type *__t = NULL; \
    if(__node) \
      { \
        __t = list_container_of(__node, type, member); \
      } \
    __t; \
  })

/* iterates over the list, node should be struct list_node* */

#define list_for_every(list, node) \
  for(node = (list)->next; node != (list); node = node->next)

/* iterates over the list in a safe way for deletion of current node
 * node and temp_node should be struct list_node*
 */

#define list_for_every_safe(list, node, temp) \
  for(node = (list)->next, temp = node->next; \
      node != (list); node = temp, temp = node->next)

/* iterates over the list, entry should be the container structure type */

#define list_for_every_entry(list, entry, type, member) \
  for(entry = list_container_of((list)->next, type, member); \
      &entry->member != (list); \
      entry = list_container_of(entry->member.next, type, member))

/* iterates over the list in a safe way for deletion of current node
 * entry and temp_entry should be the container structure type *
 */

#define list_for_every_entry_safe(list, entry, temp, type, member) \
  for(entry = list_container_of((list)->next, type, member), \
      temp = list_container_of(entry->member.next, type, member); \
      &entry->member != (list); entry = temp, \
      temp = list_container_of(temp->member.next, type, member))

/* Iterate from a given entry node in a safe way */

#define list_for_every_entry_safe_from(list, cur, temp, type, member) \
  for ((temp) = list_next_entry(cur, type, member); \
       &(cur)->member != (list); \
       (cur) = (temp), (temp) = list_next_entry(temp, type, member))

#define list_for_every_entry_continue(list, head, type, member)    \
  for ((list) = list_next_entry(list, type, member); \
       &(list)->member != (head); \
       (list) = list_next_entry(list, type, member))

/* iterates over the list in reverse order, entry should be the container
 * structure type
 */

#define list_for_every_entry_reverse(list, entry, type, member) \
  for(entry = list_container_of((list)->prev, type, member); \
      &entry->member != (list); \
      entry = list_container_of(entry->member.prev, type, member))

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
