/****************************************************************************
 * net/bt/host/addr.c
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

/* Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <nuttx/net/bluetooth/addr.h>
#include <nuttx/net/bluetooth/crypto.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>

#include "keys.h"

#define ADDR_RESOLVED_BITMASK (0x02)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int create_random_addr(bt_addr_le_t *addr)
{
  addr->type = BT_ADDR_LE_RANDOM;

  return bt_rand(addr->a.val, 6);
}

int bt_addr_le_create_nrpa(bt_addr_le_t *addr)
{
  int err;

  err = create_random_addr(addr);
  if (err)
    {
      return err;
    }

  BT_ADDR_SET_NRPA(&addr->a);

  return 0;
}

int bt_addr_le_create_static(bt_addr_le_t *addr)
{
  int err;

  err = create_random_addr(addr);
  if (err)
    {
      return err;
    }

  BT_ADDR_SET_STATIC(&addr->a);

  return 0;
}

int bt_addr_from_str(const char *str, bt_addr_t *addr)
{
  /* Parse a null-terminated string with a Bluetooth address in
   * canonical "XX:XX:XX:XX:XX:XX" format.
   */

  const size_t len = strlen(str);

  /* Verify length. */

  if (len != BT_ADDR_STR_LEN - 1)
    {
      return -EINVAL;
    }

  /* Verify that all the colons are present. */

  for (size_t i = 2; i < len; i += 3)
    {
      if (str[i] != ':')
        {
          return -EINVAL;
        }
    }

  /* Parse each octet as hex and populate `addr->val`. It must be
   * reversed since `bt_addr_t` is in 'on-air' format.
   */

  for (size_t i = 0; i < ARRAY_SIZE(addr->val); i++)
    {
      const size_t reverse_i = ARRAY_SIZE(addr->val) - 1 - i;

      if (!hex2bin(&str[i * 3], 2, &addr->val[reverse_i], 1))
        {
          return -EINVAL;
        }
    }

  return 0;
}

int bt_addr_le_from_str(const char *str, bt_addr_le_t *addr)
{
  uint8_t type;
  int err;

  /* Parse a null-terminated string in the "P:XX:XX:XX:XX:XX:XX" (public)
   * or "R:XX:XX:XX:XX:XX:XX" (random) format produced by
   * bt_addr_le_to_str().
   */

  switch (str[0])
    {
      case 'P':
      case 'p':
        type = BT_ADDR_LE_PUBLIC;
        break;
      case 'R':
      case 'r':
        type = BT_ADDR_LE_RANDOM;
        break;
      default:
        return -EINVAL;
    }

  if (str[1] != ':')
    {
      return -EINVAL;
    }

  err = bt_addr_from_str(&str[2], &addr->a);
  if (err < 0)
    {
      return err;
    }

  addr->type = type;

  return 0;
}

void bt_addr_le_copy_resolved(bt_addr_le_t *dst, const bt_addr_le_t *src)
{
  bt_addr_le_copy(dst, src);

  /* translate to "regular" address type */

  dst->type &= ~ADDR_RESOLVED_BITMASK;
}

bool bt_addr_le_is_resolved(const bt_addr_le_t *addr)
{
  /* Bit 1 alone is not enough: the anonymous (0xff) and unresolved (0xfe)
   * sentinels have it set too.
   */

  return addr->type == BT_ADDR_LE_PUBLIC_ID ||
      addr->type == BT_ADDR_LE_RANDOM_ID;
}

#if defined(CONFIG_BT_PRIVACY)
int bt_addr_le_rpa_resolve(uint8_t id,
                           const bt_addr_le_t *addr,
                           bt_addr_le_t *resolved_addr)
{
  struct bt_keys *keys;

  if (addr == NULL || resolved_addr == NULL)
    {
      return -EINVAL;
    }

  if (id >= CONFIG_BT_ID_MAX)
    {
      return -EINVAL;
    }

  if (!bt_addr_le_is_rpa(addr))
    {
      return -EINVAL;
    }

  keys = bt_keys_find_irk(id, addr);
  if (keys != NULL)
    {
      /* Found a matching IRK, return the identity address */

      bt_addr_le_copy(resolved_addr, &keys->addr);
      return 0;
    }

  /* No matching IRK found for this RPA */

  return -ENOENT;
}
#endif /* CONFIG_BT_PRIVACY */
