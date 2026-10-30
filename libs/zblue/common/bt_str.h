/****************************************************************************
 * libs/zblue/common/bt_str.h
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/* Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LIBS_ZBLUE_COMMON_BT_STR_H
#define __LIBS_ZBLUE_COMMON_BT_STR_H

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

#endif /* __LIBS_ZBLUE_COMMON_BT_STR_H */
