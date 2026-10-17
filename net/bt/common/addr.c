/****************************************************************************
 * net/bt/common/addr.c
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

/* Copyright (c) 2022 Stephanos Ioannidis <root@stephanos.io>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/addr.h>

const bt_addr_t bt_addr_any =
{
  {
      0, 0, 0, 0, 0, 0
  }
};
const bt_addr_t bt_addr_none =
{
  {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff
  }
};
const bt_addr_le_t bt_addr_le_any =
{
  0,
  {
      {
          0, 0, 0, 0, 0, 0
      }
  }
};
const bt_addr_le_t bt_addr_le_none =
{
  0,
  {
      {
          0xff, 0xff, 0xff, 0xff, 0xff, 0xff
      }
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct bt_addr_tmp_str bt_addr_tmp_str(const bt_addr_t *addr)
{
  struct bt_addr_tmp_str val;

  (void)bt_addr_to_str(addr, val.str, sizeof(val.str));

  return val;
}

struct bt_addr_le_tmp_str bt_addr_le_tmp_str(const bt_addr_le_t *addr)
{
  struct bt_addr_le_tmp_str val;

  (void)bt_addr_le_to_str(addr, val.str, sizeof(val.str));

  return val;
}
