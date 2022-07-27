/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_mbox_list.h
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

#ifndef __ARCH_ARM_SRC_STM32WB_STM32WB_MBOX_LIST_H
#define __ARCH_ARM_SRC_STM32WB_STM32WB_MBOX_LIST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This list implementation is similar to list_node, but prev and next
 * fields are at opposite places and the struct is packed.  Also there is
 * a new list_moveall function.
 */

begin_packed_struct struct stm32wb_mbox_list_s
{
  struct stm32wb_mbox_list_s *next;
  struct stm32wb_mbox_list_s *prev;
} end_packed_struct;

typedef struct stm32wb_mbox_list_s stm32wb_mbox_list_t;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_mbox_list_initialize
 *
 * Description:
 *   Initialize internal fields.
 *
 ****************************************************************************/

static inline void stm32wb_mbox_list_initialize(stm32wb_mbox_list_t *list)
{
  list->prev = list;
  list->next = list;
}

/****************************************************************************
 * Name: stm32wb_mbox_list_add_tail
 *
 * Description:
 *   Add new node at the end of the list.
 *
 ****************************************************************************/

static inline void stm32wb_mbox_list_add_tail(stm32wb_mbox_list_t *list,
                                              stm32wb_mbox_list_t *item)
{
  item->prev       = list->prev;
  item->next       = list;
  list->prev->next = item;
  list->prev       = item;
}

/****************************************************************************
 * Name: stm32wb_mbox_list_remove_head
 *
 * Description:
 *   Remove and return first node from the list head (if any).
 *
 ****************************************************************************/

static inline stm32wb_mbox_list_t *
stm32wb_mbox_list_remove_head(stm32wb_mbox_list_t *list)
{
  if (list->next != list)
  {
    stm32wb_mbox_list_t *item = list->next;
    item->next->prev = item->prev;
    item->prev->next = item->next;
    item->prev       = NULL;
    item->next       = NULL;
    return item;
  }
  else
  {
    return NULL;
  }
}

/****************************************************************************
 * Name: stm32wb_mbox_list_is_empty
 *
 * Description:
 *   Check if the list is empty.
 *
 ****************************************************************************/

static inline bool stm32wb_mbox_list_is_empty(stm32wb_mbox_list_t *list)
{
  return (list->next == list);
}

/****************************************************************************
 * Name: stm32wb_mbox_list_moveall
 *
 * Description:
 *   Remove all nodes from source list and add them to the end of the
 *   destination list.
 *
 ****************************************************************************/

static inline void stm32wb_mbox_list_moveall(stm32wb_mbox_list_t *src,
                                             stm32wb_mbox_list_t *dst)
{
  if (src->next != src)
    {
      src->next->prev = dst->prev;
      src->prev->next = dst;
      dst->prev->next = src->next;
      dst->prev       = src->prev;
      src->prev       = src;
      src->next       = src;
    }
}

#endif /* __ARCH_ARM_SRC_STM32WB_STM32WB_MBOX_LIST_H */
