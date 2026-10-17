/****************************************************************************
 * net/bt/host/uuid.c
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

/* uuid.c - Bluetooth UUID handling */

/* Copyright (c) 2025 Xiaomi Corporation
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <uuid.h>

#include <nuttx/net/bluetooth/uuid.h>
#include <assert.h>
#include <nuttx/net/bluetooth/compat/sys/byteorder.h>

#define BT_UUID_STR_LEN_16 4U
#define BT_UUID_STR_LEN_32 8U
#define BT_UUID_STR_LEN_128 UUID_STR_LEN
#define BT_UUID_SIZE 16U

/* Bluetooth Base UUID (big-endian / RFC 9562):
 *   00000000-0000-1000-8000-00805F9B34FB
 */

static const uint8_t bt_uuid_base_be[BT_UUID_SIZE] =
{
  0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
                                                      0x10u, 0x00u, 0x80u,
                                                          0x00u, 0x00u,
                                                            0x80u,
                                                        0x5fu,
                                                            0x9bu,
                                                            0x34u,
                                                            0xfbu
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief Expand a Bluetooth UUID to the generic UUID representation.
 *
 * 16-bit and 32-bit short UUIDs are expanded using the Bluetooth Base UUID.
 * 128-bit UUIDs are byte-swapped from BT wire order (LE) to RFC 9562 order
 * (BE).
 */

static void bt_uuid_to_uuid(const struct bt_uuid *src, uuid_t *dst)
{
  uint8_t uuid_be[BT_UUID_SIZE];

  switch (src->type)
    {
      case BT_UUID_TYPE_16:
        memcpy(uuid_be, bt_uuid_base_be, sizeof(uuid_be));
        sys_put_be16(BT_UUID_16(src)->val, &uuid_be[2]);
        uuid_dec_be(uuid_be, dst);
        return;
      case BT_UUID_TYPE_32:
        memcpy(uuid_be, bt_uuid_base_be, sizeof(uuid_be));
        sys_put_be32(BT_UUID_32(src)->val, uuid_be);
        uuid_dec_be(uuid_be, dst);
        return;
      case BT_UUID_TYPE_128:
        sys_memcpy_swap(uuid_be, BT_UUID_128(src)->val, sizeof(uuid_be));
        uuid_dec_be(uuid_be, dst);
        return;
      default:
        DEBUGASSERT(false);
    }
}

/**
 * @brief Offset of the Bluetooth Base UUID tail (bytes [4..15]).
 */
#define BT_UUID_BASE_TAIL_OFFSET 4U

/**
 * @brief Check whether a generic UUID matches the Bluetooth Base UUID.
 *
 * Compares bytes [4..15] against the Bluetooth Base UUID tail.
 *
 * @return true if the UUID matches the Bluetooth Base UUID pattern.
 */

static bool is_bt_base_uuid(const uuid_t *u)
{
  uint8_t uuid_be[BT_UUID_SIZE];

  uuid_enc_be(uuid_be, u);
  return memcmp(&uuid_be[BT_UUID_BASE_TAIL_OFFSET],
          &bt_uuid_base_be[BT_UUID_BASE_TAIL_OFFSET],
                sizeof(uuid_be) - BT_UUID_BASE_TAIL_OFFSET) == 0;
}

/**
 * @brief Determine the compact Bluetooth UUID type for a base-matching UUID.
 *
 * @pre is_bt_base_uuid(u) must be true.
 *
 * @return BT_UUID_TYPE_16 or BT_UUID_TYPE_32.
 */

static uint8_t bt_base_uuid_type(const uuid_t *u)
{
  uint8_t uuid_be[BT_UUID_SIZE];

  uuid_enc_be(uuid_be, u);
  if (uuid_be[0] == 0 && uuid_be[1] == 0)
    {
      return BT_UUID_TYPE_16;
    }

  return BT_UUID_TYPE_32;
}

int bt_uuid_cmp(const struct bt_uuid *u1, const struct bt_uuid *u2)
{
  if (u1->type != u2->type)
    {
      uuid_t uu1;
      uuid_t uu2;
      uint8_t uu1_be[BT_UUID_SIZE];
      uint8_t uu2_be[BT_UUID_SIZE];

      bt_uuid_to_uuid(u1, &uu1);
      bt_uuid_to_uuid(u2, &uu2);
      uuid_enc_be(uu1_be, &uu1);
      uuid_enc_be(uu2_be, &uu2);
      return memcmp(uu1_be, uu2_be, sizeof(uu1_be));
    }

  switch (u1->type)
    {
      case BT_UUID_TYPE_16:
        return (int)BT_UUID_16(u1)->val - (int)BT_UUID_16(u2)->val;
      case BT_UUID_TYPE_32:
        return (int)BT_UUID_32(u1)->val - (int)BT_UUID_32(u2)->val;
      case BT_UUID_TYPE_128:
        return memcmp(BT_UUID_128(u1)->val, BT_UUID_128(u2)->val, 16);
    }

  return -EINVAL;
}

bool bt_uuid_create(struct bt_uuid *uuid,
                    const uint8_t *data, uint8_t data_len)
{
  /* Copy UUID from packet data/internal variable to internal bt_uuid */

  switch (data_len)
    {
      case BT_UUID_SIZE_16:
        uuid->type = BT_UUID_TYPE_16;
        BT_UUID_16(uuid)->val = sys_get_le16(data);
        break;
      case BT_UUID_SIZE_32:
        uuid->type = BT_UUID_TYPE_32;
        BT_UUID_32(uuid)->val = sys_get_le32(data);
        break;
      case BT_UUID_SIZE_128:
        uuid->type = BT_UUID_TYPE_128;
        memcpy(&BT_UUID_128(uuid)->val, data, 16);
        break;
      default:
        return false;
    }

  return true;
}

void bt_uuid_to_str(const struct bt_uuid *uuid, char *str, size_t len)
{
  __maybe_unused int err;

  switch (uuid->type)
    {
      case BT_UUID_TYPE_16:
        err = snprintf(str, len, "%04x", BT_UUID_16(uuid)->val);
        DEBUGASSERT(err == BT_UUID_STR_LEN_16);
        break;
      case BT_UUID_TYPE_32:
        err = snprintf(str, len, "%08x", BT_UUID_32(uuid)->val);
        DEBUGASSERT(err == BT_UUID_STR_LEN_32);
        break;
      case BT_UUID_TYPE_128:
        {
          uuid_t gen_uuid;
          uint32_t status;
          char *full = NULL;

          bt_uuid_to_uuid(uuid, &gen_uuid);
          uuid_to_string(&gen_uuid, &full, &status);
          if (status != uuid_s_ok || full == NULL)
            {
              (void)memset(str, 0, len);
              return;
            }

          err = snprintf(str, len, "%s", full);
          free(full);
          DEBUGASSERT(err == BT_UUID_STR_LEN_128);
          break;
        }

      default:
        (void)memset(str, 0, len);
        return;
    }
}

int bt_uuid_from_str(const char *str, struct bt_uuid_any *uuid)
{
  uuid_t gen_uuid;
  uint8_t uuid_be[BT_UUID_SIZE];
  uint32_t status;

  if (str == NULL || uuid == NULL)
    {
      return -EINVAL;
    }

  switch (strlen(str))
    {
      case BT_UUID_STR_LEN_16:
        {
          char full[UUID_STR_LEN + 1U];

          snprintf(full, sizeof(full),
            "00000000-0000-1000-8000-00805f9b34fb");
          if (strlen(full) != UUID_STR_LEN)
            {
              return -EINVAL;
            }

          (void)memcpy(&full[4], str, BT_UUID_STR_LEN_16);

          uuid_from_string(full, &gen_uuid, &status);
          if (status != uuid_s_ok)
            {
              return -EINVAL;
            }

          /* Enforce 16-bit UUID type for 4-hex-digit input strings. */

          uuid->uuid.type = BT_UUID_TYPE_16;
          uuid_enc_be(uuid_be, &gen_uuid);
          BT_UUID_16(&uuid->uuid)->val = sys_get_be16(&uuid_be[2]);
          return 0;
        }

      case BT_UUID_STR_LEN_32:
        {
          char full[UUID_STR_LEN + 1U];

          snprintf(full, sizeof(full),
            "00000000-0000-1000-8000-00805f9b34fb");
          if (strlen(full) != UUID_STR_LEN)
            {
              return -EINVAL;
            }

          (void)memcpy(full, str, BT_UUID_STR_LEN_32);
          uuid_from_string(full, &gen_uuid, &status);
          if (status != uuid_s_ok)
            {
              return -EINVAL;
            }

          /* Enforce 32-bit UUID type for 8-hex-digit input strings. */

          uuid->uuid.type = BT_UUID_TYPE_32;
          uuid_enc_be(uuid_be, &gen_uuid);
          BT_UUID_32(&uuid->uuid)->val = sys_get_be32(uuid_be);
          return 0;
        }

      case BT_UUID_STR_LEN_128: /* full string -> always 128-bit */
        uuid_from_string(str, &gen_uuid, &status);
        if (status != uuid_s_ok)
          {
            return -EINVAL;
          }

        uuid->uuid.type = BT_UUID_TYPE_128;
        uuid_enc_be(uuid_be, &gen_uuid);
        sys_memcpy_swap(BT_UUID_128(&uuid->uuid)->val, uuid_be,
                sizeof(uuid_be));
        return 0;
      default:
        return -EINVAL;
    }
}

int bt_uuid_compress(const struct bt_uuid *src, struct bt_uuid_any *dst)
{
  uuid_t u;
  uint8_t uuid_be[BT_UUID_SIZE];

  if (src == NULL || dst == NULL)
    {
      return -EINVAL;
    }

  switch (src->type)
    {
      case BT_UUID_TYPE_16:
        dst->uuid.type = BT_UUID_TYPE_16;
        BT_UUID_16(&dst->uuid)->val = BT_UUID_16(src)->val;
        return 0;
      case BT_UUID_TYPE_32:
        dst->uuid.type = BT_UUID_TYPE_32;
        BT_UUID_32(&dst->uuid)->val = BT_UUID_32(src)->val;
        return 0;
      case BT_UUID_TYPE_128:
        bt_uuid_to_uuid(src, &u);

        if (!is_bt_base_uuid(&u))
          {
            return -ENOTSUP;
          }

        dst->uuid.type = bt_base_uuid_type(&u);
        uuid_enc_be(uuid_be, &u);

        if (dst->uuid.type == BT_UUID_TYPE_16)
          {
            BT_UUID_16(&dst->uuid)->val = sys_get_be16(&uuid_be[2]);
          }
        else
          {
            BT_UUID_32(&dst->uuid)->val = sys_get_be32(uuid_be);
          }

        return 0;
      default:
        return -EINVAL;
    }
}
