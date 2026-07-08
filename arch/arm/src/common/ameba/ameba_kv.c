/****************************************************************************
 * arch/arm/src/common/ameba/ameba_kv.c
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

/****************************************************************************
 * Realtek SDK key-value store (rt_kv_*) backed by the NuttX filesystem.
 *
 * The WHC host WiFi libraries persist fast-connect / PMK-cache data through
 * the SDK's rt_kv_set/get API.  In the vendor SDK these store one file per
 * key on a littlefs partition (component/file_system/kv/kv.c does
 * fopen("<prefix>:KV/<key>") + fwrite/fread).  Here we provide the same
 * symbols on top of NuttX's own littlefs mount (AMEBA_KV_DIR), keeping one
 * file per key.
 *
 * This file lives on the NuttX side of the WiFi port (full NuttX headers,
 * POSIX file API) -- the lwIP-colliding SDK headers are NOT pulled in, so it
 * is compiled through CHIP_CSRCS rather than the SDK include path.  The
 * symbols are always defined when WiFi is built; if no flash filesystem is
 * configured they degrade to "store nothing / always miss", which simply
 * disables fast-connect (a full scan/connect still works).
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#include "ameba_flash_mtd.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RTL8721DX_FLASH_FS

/****************************************************************************
 * Name: ameba_kv_path
 *
 * Description:
 *   Build the "<AMEBA_KV_DIR>/<key>" path.  Returns 0 on success.
 *
 ****************************************************************************/

static int ameba_kv_path(char *path, size_t size, const char *key)
{
  int n = snprintf(path, size, "%s/%s", AMEBA_KV_DIR, key);

  return (n > 0 && (size_t)n < size) ? 0 : -1;
}

/****************************************************************************
 * Name: rt_kv_set
 *
 * Description:
 *   Store len bytes of val under key (overwriting any previous value).
 *
 * Returned Value:
 *   The number of bytes stored on success, or a negative value on error.
 *
 ****************************************************************************/

int32_t rt_kv_set(const char *key, const void *val, int32_t len)
{
  char path[64];
  int fd;
  ssize_t nw;

  if (key == NULL || val == NULL || len < 0 ||
      ameba_kv_path(path, sizeof(path), key) < 0)
    {
      return -1;
    }

  /* The mount point is created by the board bring-up; make sure the KV
   * sub-directory exists (ignore "already exists").
   */

  mkdir(AMEBA_KV_DIR, 0777);

  fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (fd < 0)
    {
      return -1;
    }

  nw = write(fd, val, (size_t)len);
  close(fd);

  return (nw == (ssize_t)len) ? len : -1;
}

/****************************************************************************
 * Name: rt_kv_get
 *
 * Description:
 *   Read up to len bytes of the value stored under key into buffer.
 *
 * Returned Value:
 *   The number of bytes read on success, or a negative value if the key
 *   does not exist / cannot be read (treated as a cache miss by callers).
 *
 ****************************************************************************/

int32_t rt_kv_get(const char *key, void *buffer, int32_t len)
{
  char path[64];
  int fd;
  ssize_t nr;

  if (key == NULL || buffer == NULL || len < 0 ||
      ameba_kv_path(path, sizeof(path), key) < 0)
    {
      return -1;
    }

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      return -1;
    }

  nr = read(fd, buffer, (size_t)len);
  close(fd);

  return (nr < 0) ? -1 : (int32_t)nr;
}

/****************************************************************************
 * Name: rt_kv_delete
 *
 * Description:
 *   Remove the value stored under key.  Returns 0 on success.
 *
 ****************************************************************************/

int32_t rt_kv_delete(const char *key)
{
  char path[64];

  if (key == NULL || ameba_kv_path(path, sizeof(path), key) < 0)
    {
      return -1;
    }

  return (unlink(path) == 0) ? 0 : -1;
}

#else /* CONFIG_RTL8721DX_FLASH_FS */

/* No persistent filesystem configured: provide the symbols so the WiFi
 * libraries link, but store nothing.  rt_kv_get always misses, so the host
 * just performs a normal (full) connect instead of a fast-connect.
 */

int32_t rt_kv_set(const char *key, const void *val, int32_t len)
{
  UNUSED(key);
  UNUSED(val);
  UNUSED(len);
  return 0;
}

int32_t rt_kv_get(const char *key, void *buffer, int32_t len)
{
  UNUSED(key);
  UNUSED(buffer);
  UNUSED(len);
  return -1;
}

int32_t rt_kv_delete(const char *key)
{
  UNUSED(key);
  return -1;
}

#endif /* CONFIG_RTL8721DX_FLASH_FS */
