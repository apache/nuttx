/****************************************************************************
 * mm/net_buf/net_buf_simple.c
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

/****************************************************************************
 * mm/net_buf/net_buf_simple.c
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
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <assert.h>
#include <endian.h>
#include <nuttx/mm/net_buf.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_buf_simple_init_with_data
 ****************************************************************************/

void net_buf_simple_init_with_data(struct net_buf_simple *buf,
                                   void *data, size_t size)
{
  buf->__buf = data;
  buf->data  = data;
  buf->size  = size;
  buf->len   = size;
}

/****************************************************************************
 * Name: net_buf_simple_reserve
 ****************************************************************************/

void net_buf_simple_reserve(struct net_buf_simple *buf, size_t reserve)
{
  DEBUGASSERT(buf != NULL);
  DEBUGASSERT(buf->len == 0U);

  buf->data = buf->__buf + reserve;
}

/****************************************************************************
 * Name: net_buf_simple_clone
 ****************************************************************************/

void net_buf_simple_clone(const struct net_buf_simple *original,
                          struct net_buf_simple *clone)
{
  *clone = *original;
}

/****************************************************************************
 * Name: net_buf_simple_add
 ****************************************************************************/

void *net_buf_simple_add(struct net_buf_simple *buf, size_t len)
{
  uint8_t *tail;

  DEBUGASSERT(buf != NULL);
  DEBUGASSERT(len <= net_buf_simple_tailroom(buf));

  tail = net_buf_simple_tail(buf);
  buf->len += len;
  return tail;
}

/****************************************************************************
 * Name: net_buf_simple_add_mem
 ****************************************************************************/

void *net_buf_simple_add_mem(struct net_buf_simple *buf, const void *mem,
                             size_t len)
{
  void *tail;

  tail = net_buf_simple_add(buf, len);

  return memcpy(tail, mem, len);
}

/****************************************************************************
 * Name: net_buf_simple_add_u8
 ****************************************************************************/

uint8_t *net_buf_simple_add_u8(struct net_buf_simple *buf, uint8_t val)
{
  uint8_t *u8;

  u8 = net_buf_simple_add(buf, 1);

  *u8 = val;
  return u8;
}

/****************************************************************************
 * Name: net_buf_simple_add_le16/be16
 ****************************************************************************/

void net_buf_simple_add_le16(struct net_buf_simple *buf, uint16_t val)
{
  *(uint16_t *)net_buf_simple_add(buf, sizeof(val)) = htole16(val);
}

void net_buf_simple_add_be16(struct net_buf_simple *buf, uint16_t val)
{
  *(uint16_t *)net_buf_simple_add(buf, sizeof(val)) = htobe16(val);
}

/****************************************************************************
 * Name: net_buf_simple_add_le32/be32
 ****************************************************************************/

void net_buf_simple_add_le32(struct net_buf_simple *buf, uint32_t val)
{
  *(uint32_t *)net_buf_simple_add(buf, sizeof(val)) = htole32(val);
}

void net_buf_simple_add_be32(struct net_buf_simple *buf, uint32_t val)
{
  *(uint32_t *)net_buf_simple_add(buf, sizeof(val)) = htobe32(val);
}

/****************************************************************************
 * Name: net_buf_simple_add_le64/be64
 ****************************************************************************/

void net_buf_simple_add_le64(struct net_buf_simple *buf, uint64_t val)
{
  *(uint64_t *)net_buf_simple_add(buf, sizeof(val)) = htole64(val);
}

void net_buf_simple_add_be64(struct net_buf_simple *buf, uint64_t val)
{
  *(uint64_t *)net_buf_simple_add(buf, sizeof(val)) = htobe64(val);
}

/****************************************************************************
 * Name: net_buf_simple_remove_mem
 ****************************************************************************/

void *net_buf_simple_remove_mem(struct net_buf_simple *buf, size_t len)
{
  DEBUGASSERT(buf != NULL);
  DEBUGASSERT(buf->len >= len);

  buf->len -= len;
  return buf->data + buf->len;
}

/****************************************************************************
 * Name: net_buf_simple_remove_u8
 ****************************************************************************/

uint8_t net_buf_simple_remove_u8(struct net_buf_simple *buf)
{
  return *(uint8_t *)net_buf_simple_remove_mem(buf, sizeof(uint8_t));
}

/****************************************************************************
 * Name: net_buf_simple_remove_le16/be16
 ****************************************************************************/

uint16_t net_buf_simple_remove_le16(struct net_buf_simple *buf)
{
  return le16toh(*(uint16_t *)net_buf_simple_remove_mem(buf,
                          sizeof(uint16_t)));
}

uint16_t net_buf_simple_remove_be16(struct net_buf_simple *buf)
{
  return be16toh(*(uint16_t *)net_buf_simple_remove_mem(buf,
                          sizeof(uint16_t)));
}

/****************************************************************************
 * Name: net_buf_simple_remove_le32/be32
 ****************************************************************************/

uint32_t net_buf_simple_remove_le32(struct net_buf_simple *buf)
{
  return le32toh(*(uint32_t *)net_buf_simple_remove_mem(buf,
                          sizeof(uint32_t)));
}

uint32_t net_buf_simple_remove_be32(struct net_buf_simple *buf)
{
  return be32toh(*(uint32_t *)net_buf_simple_remove_mem(buf,
                          sizeof(uint32_t)));
}

/****************************************************************************
 * Name: net_buf_simple_remove_le64/be64
 ****************************************************************************/

uint64_t net_buf_simple_remove_le64(struct net_buf_simple *buf)
{
  return le64toh(*(uint64_t *)net_buf_simple_remove_mem(buf,
                          sizeof(uint64_t)));
}

uint64_t net_buf_simple_remove_be64(struct net_buf_simple *buf)
{
  return be64toh(*(uint64_t *)net_buf_simple_remove_mem(buf,
                          sizeof(uint64_t)));
}

/****************************************************************************
 * Name: net_buf_simple_push
 ****************************************************************************/

void *net_buf_simple_push(struct net_buf_simple *buf, size_t len)
{
  DEBUGASSERT(buf != NULL);
  DEBUGASSERT(len <= net_buf_simple_headroom(buf));

  buf->data -= len;
  buf->len += len;
  return buf->data;
}

/****************************************************************************
 * Name: net_buf_simple_push_mem
 ****************************************************************************/

void *net_buf_simple_push_mem(struct net_buf_simple *buf, const void *mem,
                              size_t len)
{
  void *data;

  data = net_buf_simple_push(buf, len);

  return memcpy(data, mem, len);
}

/****************************************************************************
 * Name: net_buf_simple_push_u8
 ****************************************************************************/

void net_buf_simple_push_u8(struct net_buf_simple *buf, uint8_t val)
{
  uint8_t *data = net_buf_simple_push(buf, 1);

  *data = val;
}

/****************************************************************************
 * Name: net_buf_simple_push_le16/be16
 ****************************************************************************/

void net_buf_simple_push_le16(struct net_buf_simple *buf, uint16_t val)
{
  *(uint16_t *)net_buf_simple_push(buf, sizeof(val)) = htole16(val);
}

void net_buf_simple_push_be16(struct net_buf_simple *buf, uint16_t val)
{
  *(uint16_t *)net_buf_simple_push(buf, sizeof(val)) = htobe16(val);
}

/****************************************************************************
 * Name: net_buf_simple_push_le32/be32
 ****************************************************************************/

void net_buf_simple_push_le32(struct net_buf_simple *buf, uint32_t val)
{
  *(uint32_t *)net_buf_simple_push(buf, sizeof(val)) = htole32(val);
}

void net_buf_simple_push_be32(struct net_buf_simple *buf, uint32_t val)
{
  *(uint32_t *)net_buf_simple_push(buf, sizeof(val)) = htobe32(val);
}

/****************************************************************************
 * Name: net_buf_simple_push_le64/be64
 ****************************************************************************/

void net_buf_simple_push_le64(struct net_buf_simple *buf, uint64_t val)
{
  *(uint64_t *)net_buf_simple_push(buf, sizeof(val)) = htole64(val);
}

void net_buf_simple_push_be64(struct net_buf_simple *buf, uint64_t val)
{
  *(uint64_t *)net_buf_simple_push(buf, sizeof(val)) = htobe64(val);
}

/****************************************************************************
 * Name: net_buf_simple_pull
 ****************************************************************************/

void *net_buf_simple_pull(struct net_buf_simple *buf, size_t len)
{
  DEBUGASSERT(buf != NULL);
  DEBUGASSERT(buf->len >= len);

  buf->len -= len;
  return buf->data += len;
}

/****************************************************************************
 * Name: net_buf_simple_pull_mem
 ****************************************************************************/

void *net_buf_simple_pull_mem(struct net_buf_simple *buf, size_t len)
{
  void *data = buf->data;

  DEBUGASSERT(buf != NULL);
  DEBUGASSERT(buf->len >= len);

  buf->len -= len;
  buf->data += len;

  return data;
}

/****************************************************************************
 * Name: net_buf_simple_pull_u8
 ****************************************************************************/

uint8_t net_buf_simple_pull_u8(struct net_buf_simple *buf)
{
  return *(uint8_t *)net_buf_simple_pull_mem(buf, sizeof(uint8_t));
}

/****************************************************************************
 * Name: net_buf_simple_pull_le16/be16
 ****************************************************************************/

uint16_t net_buf_simple_pull_le16(struct net_buf_simple *buf)
{
  return le16toh(*(uint16_t *)net_buf_simple_pull_mem(buf,
                          sizeof(uint16_t)));
}

uint16_t net_buf_simple_pull_be16(struct net_buf_simple *buf)
{
  return be16toh(*(uint16_t *)net_buf_simple_pull_mem(buf,
                          sizeof(uint16_t)));
}

/****************************************************************************
 * Name: net_buf_simple_pull_le32/be32
 ****************************************************************************/

uint32_t net_buf_simple_pull_le32(struct net_buf_simple *buf)
{
  return le32toh(*(uint32_t *)net_buf_simple_pull_mem(buf,
                          sizeof(uint32_t)));
}

uint32_t net_buf_simple_pull_be32(struct net_buf_simple *buf)
{
  return be32toh(*(uint32_t *)net_buf_simple_pull_mem(buf,
                          sizeof(uint32_t)));
}

/****************************************************************************
 * Name: net_buf_simple_pull_le64/be64
 ****************************************************************************/

uint64_t net_buf_simple_pull_le64(struct net_buf_simple *buf)
{
  return le64toh(*(uint64_t *)net_buf_simple_pull_mem(buf,
                          sizeof(uint64_t)));
}

uint64_t net_buf_simple_pull_be64(struct net_buf_simple *buf)
{
  return be64toh(*(uint64_t *)net_buf_simple_pull_mem(buf,
                          sizeof(uint64_t)));
}
