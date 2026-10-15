/****************************************************************************
 * net/bt/common/bt_str.h
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

/* Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __NET_BT_COMMON_BT_STR_H
#define __NET_BT_COMMON_BT_STR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <string.h>
#include <wchar.h>

#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/uuid.h>

/* NOTE: These helper functions always encodes into the same buffer storage.
 * It is the responsibility of the user of this function to copy the
 * information in this string if needed.
 *
 * NOTE: These functions are not thread-safe!
 */

const char *bt_hex(const void *buf, size_t len);
const char *bt_uuid_str(const struct bt_uuid *uuid);

static inline char *utf8_trunc(char *utf8_str)
{
  mbstate_t state =
    {
        0
    };
  char *p = utf8_str;

  if (utf8_str == NULL)
    {
      return NULL;
    }

  while (*p != '\0')
    {
      size_t len = mbrtowc(NULL, p, strlen(p), &state);

      if (len == (size_t)-1 || len == (size_t)-2)
        {
          *p = '\0';
          break;
        }

      if (len == 0)
        {
          break;
        }

      p += len;
    }

  return utf8_str;
}

static inline char *utf8_lcpy(char *dst, const char *src, size_t n)
{
  if (n == 0)
    {
      return dst;
    }

  strncpy(dst, src, n - 1);
  dst[n - 1] = '\0';

  return utf8_trunc(dst);
}

static inline int utf8_count_chars(const char *s)
{
  mbstate_t state =
    {
        0
    };
  int count = 0;

  if (s == NULL)
    {
      return 0;
    }

  while (*s != '\0')
    {
      size_t len = mbrtowc(NULL, s, strlen(s), &state);

      if (len == (size_t)-1 || len == (size_t)-2)
        {
          return -1;
        }

      if (len == 0)
        {
          break;
        }

      s += len;
      count++;
    }

  return count;
}

#endif /* __NET_BT_COMMON_BT_STR_H */
