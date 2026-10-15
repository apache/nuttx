/****************************************************************************
 * net/bt/services/ots/ots_obj_manager.c
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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

/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <nuttx/list.h>
#include <nuttx/net/bluetooth/compat/sys/byteorder.h>

#include <nuttx/net/bluetooth/services/ots.h>
#include "ots_internal.h"

#include <nuttx/net/bluetooth/compat/logging/log.h>

LOG_MODULE_DECLARE(bt_ots, CONFIG_BT_OTS_LOG_LEVEL);

struct bt_gatt_ots_pool_item
{
  struct list_node dnode;
  struct bt_gatt_ots_object val;
  bool is_allocated;
};

struct bt_gatt_ots_obj_manager
{
  struct list_node list;
  struct bt_gatt_ots_pool_item pool[CONFIG_BT_OTS_MAX_OBJ_CNT];
  bool is_assigned;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t obj_id_to_index(uint64_t id)
{
  if (IS_ENABLED(CONFIG_BT_OTS_DIR_LIST_OBJ))
    {
      if (id == OTS_OBJ_ID_DIR_LIST)
        {
          return id;
        }
      else
        {
          return id - BT_OTS_OBJ_ID_MIN + 1;
        }
    }
  else
    {
      return id - BT_OTS_OBJ_ID_MIN;
    }
}

static uint64_t obj_index_to_id(uint64_t index)
{
  if (IS_ENABLED(CONFIG_BT_OTS_DIR_LIST_OBJ))
    {
      if (index == 0)
        {
          return OTS_OBJ_ID_DIR_LIST;
        }
      else
        {
          return BT_OTS_OBJ_ID_MIN + index - 1;
        }
    }
  else
    {
      return BT_OTS_OBJ_ID_MIN + index;
    }
}

int bt_gatt_ots_obj_manager_first_obj_get(
  struct bt_gatt_ots_obj_manager *obj_manager,
  struct bt_gatt_ots_object **obj)
{
  struct list_node *obj_dnode;
  struct bt_gatt_ots_pool_item *first_item;

  if (list_is_empty(&obj_manager->list))
    {
      return -ENOENT;
    }

  obj_dnode = list_peek_head(&obj_manager->list);
  first_item = CONTAINER_OF(obj_dnode, struct bt_gatt_ots_pool_item, dnode);
  *obj = &first_item->val;

  return 0;
}

int bt_gatt_ots_obj_manager_last_obj_get(
  struct bt_gatt_ots_obj_manager *obj_manager,
  struct bt_gatt_ots_object **obj)
{
  struct list_node *obj_dnode;
  struct bt_gatt_ots_pool_item *last_item;

  if (list_is_empty(&obj_manager->list))
    {
      return -ENOENT;
    }

  obj_dnode = list_peek_tail(&obj_manager->list);
  last_item = CONTAINER_OF(obj_dnode, struct bt_gatt_ots_pool_item, dnode);
  *obj = &last_item->val;

  return 0;
}

int bt_gatt_ots_obj_manager_prev_obj_get(
  struct bt_gatt_ots_obj_manager *obj_manager,
  const struct bt_gatt_ots_object *cur_obj,
  struct bt_gatt_ots_object **prev_obj)
{
  struct list_node *prev_obj_dnode;
  struct bt_gatt_ots_pool_item *cur_item;
  struct bt_gatt_ots_pool_item *prev_item;

  if (list_is_empty(&obj_manager->list))
    {
      return -ENOENT;
    }

  cur_item = CONTAINER_OF(cur_obj, struct bt_gatt_ots_pool_item, val);
  prev_obj_dnode = list_prev(&obj_manager->list, &cur_item->dnode);
  if (!prev_obj_dnode)
    {
      return -ENFILE;
    }

  prev_item = CONTAINER_OF(prev_obj_dnode, struct bt_gatt_ots_pool_item,
                           dnode);
  *prev_obj = &prev_item->val;

  return 0;
}

int bt_gatt_ots_obj_manager_next_obj_get(
  struct bt_gatt_ots_obj_manager *obj_manager,
  const struct bt_gatt_ots_object *cur_obj,
  struct bt_gatt_ots_object **next_obj)
{
  struct list_node *next_obj_dnode;
  struct bt_gatt_ots_pool_item *cur_item;
  struct bt_gatt_ots_pool_item *next_item;

  if (list_is_empty(&obj_manager->list))
    {
      return -ENOENT;
    }

  cur_item = CONTAINER_OF(cur_obj, struct bt_gatt_ots_pool_item, val);
  next_obj_dnode = list_next(&obj_manager->list, &cur_item->dnode);
  if (!next_obj_dnode)
    {
      return -ENFILE;
    }

  next_item = CONTAINER_OF(next_obj_dnode, struct bt_gatt_ots_pool_item,
                           dnode);
  *next_obj = &next_item->val;

  return 0;
}

int bt_gatt_ots_obj_manager_obj_get(
  struct bt_gatt_ots_obj_manager *obj_manager,
  uint64_t id,
  struct bt_gatt_ots_object **object)
{
  uint64_t index;

  if (list_is_empty(&obj_manager->list))
    {
      return -ENOENT;
    }

  if (id == OTS_OBJ_ID_DIR_LIST && !IS_ENABLED(CONFIG_BT_OTS_DIR_LIST_OBJ))
    {
      return -EINVAL;
    }

  index = obj_id_to_index(id);

  if (index >= ARRAY_SIZE(obj_manager->pool))
    {
      return -EINVAL;
    }

  if (!obj_manager->pool[index].is_allocated)
    {
      return -EINVAL;
    }

  *object = &obj_manager->pool[index].val;

  return 0;
}

int bt_gatt_ots_obj_manager_obj_add(
  struct bt_gatt_ots_obj_manager *obj_manager,
  struct bt_gatt_ots_object **object)
{
  for (uint64_t i = 0; i < ARRAY_SIZE(obj_manager->pool); i++)
    {
      struct bt_gatt_ots_pool_item *cur_obj = &obj_manager->pool[i];

      if (!cur_obj->is_allocated)
        {
          cur_obj->is_allocated = true;
          cur_obj->val.id = obj_index_to_id(i);
          list_add_tail(&obj_manager->list, &cur_obj->dnode);

          *object = &cur_obj->val;
          return 0;
        }
    }

  return -ENOMEM;
}

int bt_gatt_ots_obj_manager_obj_delete(struct bt_gatt_ots_object *obj)
{
  struct bt_gatt_ots_pool_item *item;

  item = CONTAINER_OF(obj, struct bt_gatt_ots_pool_item, val);

  if (!item->is_allocated)
    {
      return -EINVAL;
    }

  if (IS_ENABLED(CONFIG_BT_OTS_DIR_LIST_OBJ) &&
      obj->id == OTS_OBJ_ID_DIR_LIST)
    {
      return -EINVAL;
    }

  item->is_allocated = false;
  list_delete(&item->dnode);

  return 0;
}

bool bt_gatt_ots_obj_manager_obj_contains(
  struct bt_gatt_ots_obj_manager *obj_manager,
  struct bt_gatt_ots_object *obj)
{
  struct bt_gatt_ots_pool_item *item;

  item = CONTAINER_OF(obj, struct bt_gatt_ots_pool_item, val);

  return PART_OF_ARRAY(obj_manager->pool, item);
}

void *bt_gatt_ots_obj_manager_assign(void)
{
  static struct bt_gatt_ots_obj_manager
          obj_manager[CONFIG_BT_OTS_MAX_INST_CNT];
  struct bt_gatt_ots_obj_manager *cur_manager;

  for (cur_manager = obj_manager; cur_manager != obj_manager +
          CONFIG_BT_OTS_MAX_INST_CNT;
       cur_manager++)
    {
      if (!cur_manager->is_assigned)
        {
          break;
        }
    }

  if (cur_manager->is_assigned)
    {
      return NULL;
    }

  cur_manager->is_assigned = true;
  list_initialize(&cur_manager->list);

  return cur_manager;
}
