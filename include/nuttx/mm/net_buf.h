/****************************************************************************
 * include/nuttx/mm/net_buf.h
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
 * include/nuttx/mm/net_buf.h
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

#ifndef __INCLUDE_NUTTX_MM_NET_BUF_H
#define __INCLUDE_NUTTX_MM_NET_BUF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/list.h>
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alignment for buffer definitions */

#if CONFIG_NET_BUF_ALIGNMENT == 0
#  define __net_buf_align __attribute__((aligned(sizeof(void *))))
#else
#  define __net_buf_align \
  __attribute__((aligned(CONFIG_NET_BUF_ALIGNMENT)))
#endif

/* Flag indicating external data ownership */

#define NET_BUF_EXTERNAL_DATA  (1 << 0)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/**
 * Simple network buffer representation.
 *
 * This is a simpler variant of the net_buf object (in fact net_buf uses
 * net_buf_simple internally). It doesn't provide any kind of reference
 * counting, user data, dynamic allocation, or in general the ability to
 * pass through kernel objects such as FIFOs.
 */

struct net_buf_simple
{
  uint8_t *data;      /* Pointer to the start of data in the buffer */
  uint16_t len;       /* Length of the data behind the data pointer */
  uint16_t size;      /* Amount of data that __buf can store */
  uint8_t *__buf;     /* Start of the data storage (do not access directly) */
};

/**
 * Parsing state of a buffer.
 */

struct net_buf_simple_state
{
  uint16_t offset;    /* Offset of data pointer from the storage start */
  uint16_t len;       /* Length of data */
};

/* Forward declarations */

struct net_buf_pool;
struct net_buf;

/* Data allocator callback vtable */

struct net_buf_data_cb
{
  uint8_t *(*alloc)(struct net_buf *buf, size_t *size,
                    bool block);
  uint8_t *(*ref)(struct net_buf *buf, uint8_t *data);
  void     (*unref)(struct net_buf *buf, uint8_t *data);
};

/* Data allocator configuration */

struct net_buf_data_alloc
{
  const struct net_buf_data_cb *cb;
  void *alloc_data;
  size_t max_alloc_size;
};

/* Fixed data pool helper: maps buffer index to its data region */

struct net_buf_pool_fixed
{
  uint8_t *data_pool;
};

/**
 * Network buffer pool representation.
 */

struct net_buf_pool
{
  struct list_node free_list;              /* LIFO free list */
  spinlock_t lock;                         /* Prevents concurrent access */
  const uint16_t buf_count;                /* Number of buffers in pool */
  uint16_t uninit_count;                   /* Number of uninitialized bufs */
  uint8_t user_data_size;                  /* User data size per buffer */
  void (*const destroy)(struct net_buf *buf);
  const struct net_buf_data_alloc *alloc;  /* Data allocation handlers */
  struct net_buf * const __bufs;           /* Start of buffer storage array */
  sem_t sem;                               /* Semaphore for blocking alloc */
};

/**
 * Network buffer representation.
 */

struct net_buf
{
  struct list_node node;        /* Linked list node */
  struct net_buf *frags;        /* Fragment chain */
  struct net_buf_pool *pool;    /* Owning pool */
  uint8_t ref;                  /* Reference count */
  uint8_t flags;                /* Buffer flags (e.g. NET_BUF_EXTERNAL_DATA) */
  uint8_t user_data_size;       /* Size of user data on this buffer */

  /* Union for ABI compatibility with net_buf_simple */

  union
  {
    struct
    {
      uint8_t *data;
      uint16_t len;
      uint16_t size;
      uint8_t *__buf;
    };

    struct net_buf_simple b;
  };

  uint8_t user_data[] __net_buf_align;  /* Variable-size trailing metadata */
};

/**
 * LIFO stack for inter-thread buffer passing.
 *
 * Provides spinlock-protected LIFO (Last In First Out) semantics for
 * passing net_buf instances between threads, equivalent to Zephyr's
 * k_lifo used with net_buf.
 */

struct net_buf_lifo
{
  struct list_node list;   /* LIFO list */
  spinlock_t lock;         /* Prevents concurrent put/get */
  sem_t sem;               /* Counting semaphore for blocking get */
};

/****************************************************************************
 * net_buf_simple stack variable macros
 ****************************************************************************/

#define NET_BUF_SIMPLE_DEFINE(_name, _size)                    \
  uint8_t net_buf_data_##_name[_size];                         \
  struct net_buf_simple _name =                                \
    {                                                          \
      .data  = net_buf_data_##_name,                           \
      .len   = 0,                                              \
      .size  = _size,                                          \
      .__buf = net_buf_data_##_name,                           \
    }

#define NET_BUF_SIMPLE_DEFINE_STATIC(_name, _size)             \
  static uint8_t net_buf_data_##_name[_size];                  \
  static struct net_buf_simple _name =                         \
    {                                                          \
      .data  = net_buf_data_##_name,                           \
      .len   = 0,                                              \
      .size  = _size,                                          \
      .__buf = net_buf_data_##_name,                           \
    }

#define NET_BUF_SIMPLE(_size)                                  \
  ((struct net_buf_simple *)(&(struct                          \
    {                                                          \
      struct net_buf_simple buf;                               \
      uint8_t data[_size];                                     \
    })                                                         \
    {                                                          \
      .buf.size = _size,                                       \
    }))

/****************************************************************************
 * net_buf_simple Pure Inline Functions
 *
 * (These only access struct members or call other pure inline functions.)
 ****************************************************************************/

static inline void net_buf_simple_init(struct net_buf_simple *buf,
                                       size_t reserve_head)
{
  if (!buf->__buf)
    {
      buf->__buf = (uint8_t *)buf + sizeof(*buf);
    }

  buf->data = buf->__buf + reserve_head;
  buf->len = 0U;
}

static inline void net_buf_simple_reset(struct net_buf_simple *buf)
{
  buf->len  = 0U;
  buf->data = buf->__buf;
}

static inline uint8_t *net_buf_simple_tail(const struct net_buf_simple *buf)
{
  return buf->data + buf->len;
}

static inline size_t net_buf_simple_headroom(
  const struct net_buf_simple *buf)
{
  return buf->data - buf->__buf;
}

static inline size_t net_buf_simple_tailroom(
  const struct net_buf_simple *buf)
{
  return buf->size - net_buf_simple_headroom(buf) - buf->len;
}

static inline uint16_t net_buf_simple_max_len(
  const struct net_buf_simple *buf)
{
  return buf->size - net_buf_simple_headroom(buf);
}

static inline bool net_buf_simple_is_valid(const struct net_buf_simple *buf)
{
  size_t headroom;

  if (buf == NULL || buf->__buf == NULL || buf->data < buf->__buf)
    {
      return false;
    }

  headroom = net_buf_simple_headroom(buf);
  if (headroom > buf->size)
    {
      return false;
    }

  return (size_t)buf->len <= (size_t)(buf->size - headroom);
}

static inline void net_buf_simple_save(const struct net_buf_simple *buf,
                                       struct net_buf_simple_state *state)
{
  state->offset = (uint16_t)net_buf_simple_headroom(buf);
  state->len = buf->len;
}

static inline void net_buf_simple_restore(struct net_buf_simple *buf,
                                          struct net_buf_simple_state *state)
{
  buf->data = buf->__buf + state->offset;
  buf->len = state->len;
}

/****************************************************************************
 * net_buf_simple Function Declarations
 *
 * (These are declared BEFORE the net_buf inline wrappers that call them.)
 ****************************************************************************/

void net_buf_simple_init_with_data(struct net_buf_simple *buf,
                                   void *data, size_t size);

void net_buf_simple_clone(const struct net_buf_simple *original,
                          struct net_buf_simple *clone);

void *net_buf_simple_add(struct net_buf_simple *buf, size_t len);

void *net_buf_simple_add_mem(struct net_buf_simple *buf, const void *mem,
                             size_t len);

uint8_t *net_buf_simple_add_u8(struct net_buf_simple *buf, uint8_t val);

void net_buf_simple_add_le16(struct net_buf_simple *buf, uint16_t val);
void net_buf_simple_add_be16(struct net_buf_simple *buf, uint16_t val);
void net_buf_simple_add_le32(struct net_buf_simple *buf, uint32_t val);
void net_buf_simple_add_be32(struct net_buf_simple *buf, uint32_t val);
void net_buf_simple_add_le64(struct net_buf_simple *buf, uint64_t val);
void net_buf_simple_add_be64(struct net_buf_simple *buf, uint64_t val);

void *net_buf_simple_remove_mem(struct net_buf_simple *buf, size_t len);

uint8_t  net_buf_simple_remove_u8(struct net_buf_simple *buf);
uint16_t net_buf_simple_remove_le16(struct net_buf_simple *buf);
uint16_t net_buf_simple_remove_be16(struct net_buf_simple *buf);
uint32_t net_buf_simple_remove_le32(struct net_buf_simple *buf);
uint32_t net_buf_simple_remove_be32(struct net_buf_simple *buf);
uint64_t net_buf_simple_remove_le64(struct net_buf_simple *buf);
uint64_t net_buf_simple_remove_be64(struct net_buf_simple *buf);

void *net_buf_simple_push(struct net_buf_simple *buf, size_t len);

void *net_buf_simple_push_mem(struct net_buf_simple *buf, const void *mem,
                              size_t len);

void net_buf_simple_push_u8(struct net_buf_simple *buf, uint8_t val);
void net_buf_simple_push_le16(struct net_buf_simple *buf, uint16_t val);
void net_buf_simple_push_be16(struct net_buf_simple *buf, uint16_t val);
void net_buf_simple_push_le32(struct net_buf_simple *buf, uint32_t val);
void net_buf_simple_push_be32(struct net_buf_simple *buf, uint32_t val);
void net_buf_simple_push_le64(struct net_buf_simple *buf, uint64_t val);
void net_buf_simple_push_be64(struct net_buf_simple *buf, uint64_t val);

void *net_buf_simple_pull(struct net_buf_simple *buf, size_t len);

void *net_buf_simple_pull_mem(struct net_buf_simple *buf, size_t len);

uint8_t  net_buf_simple_pull_u8(struct net_buf_simple *buf);
uint16_t net_buf_simple_pull_le16(struct net_buf_simple *buf);
uint16_t net_buf_simple_pull_be16(struct net_buf_simple *buf);
uint32_t net_buf_simple_pull_le32(struct net_buf_simple *buf);
uint32_t net_buf_simple_pull_be32(struct net_buf_simple *buf);
uint64_t net_buf_simple_pull_le64(struct net_buf_simple *buf);
uint64_t net_buf_simple_pull_be64(struct net_buf_simple *buf);

void net_buf_simple_reserve(struct net_buf_simple *buf, size_t reserve);

/****************************************************************************
 * net_buf Core Function Declarations
 *
 * (Declared BEFORE the inline wrappers below.)
 ****************************************************************************/

extern const struct net_buf_data_cb g_net_buf_fixed_cb;

struct net_buf *net_buf_alloc_fixed(struct net_buf_pool *pool,
                                    int32_t timeout);

struct net_buf *net_buf_alloc_len(struct net_buf_pool *pool,
                                  size_t size, int32_t timeout);

struct net_buf *net_buf_alloc_with_data(struct net_buf_pool *pool,
                                        void *data, size_t size,
                                        int32_t timeout);

void net_buf_reset(struct net_buf *buf);
void net_buf_unref(struct net_buf *buf);
struct net_buf *net_buf_ref(struct net_buf *buf);
struct net_buf *net_buf_clone(struct net_buf *buf, int32_t timeout);
int net_buf_id(const struct net_buf *buf);
int net_buf_user_data_copy(struct net_buf *dst,
                           const struct net_buf *src);

void net_buf_slist_put(struct list_node *list, struct net_buf *buf);
struct net_buf *net_buf_slist_get(struct list_node *list);

/* LIFO queue operations (inter-thread buffer passing) */

void net_buf_lifo_init(struct net_buf_lifo *lifo);
void net_buf_lifo_put(struct net_buf_lifo *lifo, struct net_buf *buf);
struct net_buf *net_buf_lifo_get(struct net_buf_lifo *lifo, int32_t timeout);

/* Fragments */

struct net_buf *net_buf_frag_last(struct net_buf *buf);
void net_buf_frag_insert(struct net_buf *parent, struct net_buf *frag);
struct net_buf *net_buf_frag_add(struct net_buf *head,
                                 struct net_buf *frag);
struct net_buf *net_buf_frag_del(struct net_buf *parent,
                                 struct net_buf *frag);

/* Utilities */

size_t net_buf_linearize(void *dst, size_t dst_len,
                         const struct net_buf *src,
                         size_t offset, size_t len);

typedef struct net_buf *(*net_buf_allocator_cb)(int32_t timeout,
                                                void *user_data);

size_t net_buf_append_bytes(struct net_buf *buf, size_t len,
                            const void *value, int32_t timeout,
                            net_buf_allocator_cb allocate_cb,
                            void *user_data);

size_t net_buf_data_match(const struct net_buf *buf, size_t offset,
                          const void *data, size_t len);

size_t net_buf_frags_len(struct net_buf *buf);

/****************************************************************************
 * Pool Definition Macros
 ****************************************************************************/

#define _NET_BUF_ARRAY_DEFINE(_name, _count, _ud_size)            \
  struct _net_buf_##_name                                         \
  {                                                               \
    uint8_t b[sizeof(struct net_buf)];                            \
    uint8_t ud[_ud_size];                                         \
  } __net_buf_align;                                              \
  static struct _net_buf_##_name _net_buf_##_name[_count]

#define NET_BUF_POOL_INITIALIZER(_pool, _alloc, _bufs, _count,    \
                                 _ud_size, _destroy)              \
  {                                                               \
    .free_list = LIST_INITIAL_VALUE(_pool.free_list),             \
    .lock = SP_UNLOCKED,                                          \
    .buf_count = _count,                                          \
    .uninit_count = _count,                                       \
    .user_data_size = _ud_size,                                   \
    .destroy = _destroy,                                          \
    .alloc = _alloc,                                              \
    .__bufs = (struct net_buf *)_bufs,                            \
    .sem = NXSEM_INITIALIZER(0, 0),                               \
  }

#define NET_BUF_POOL_FIXED_DEFINE(_name, _count, _data_size,      \
                                  _ud_size, _destroy)             \
  _NET_BUF_ARRAY_DEFINE(_name, _count, _ud_size);                 \
  static uint8_t net_buf_data_##_name[_count][_data_size]         \
    __net_buf_align;                                               \
  static const struct net_buf_pool_fixed net_buf_fixed_##_name =  \
  {                                                               \
    .data_pool = (uint8_t *)net_buf_data_##_name,                 \
  };                                                              \
  static const struct net_buf_data_alloc                          \
    net_buf_fixed_alloc_##_name =                                 \
  {                                                               \
    .cb = &g_net_buf_fixed_cb,                                    \
    .alloc_data = (void *)&net_buf_fixed_##_name,                 \
    .max_alloc_size = _data_size,                                 \
  };                                                              \
  static struct net_buf_pool _name =                              \
    NET_BUF_POOL_INITIALIZER(_name, &net_buf_fixed_alloc_##_name, \
                             _net_buf_##_name, _count,            \
                             _ud_size, _destroy)

#define NET_BUF_POOL_DEFINE(_name, _count, _size, _ud_size,       \
                            _destroy)                             \
  NET_BUF_POOL_FIXED_DEFINE(_name, _count, _size, _ud_size,       \
                            _destroy)

/****************************************************************************
 * net_buf Inline Functions
 *
 * (All declarations used by these inlines are now visible above.)
 ****************************************************************************/

static inline struct net_buf *net_buf_alloc(struct net_buf_pool *pool,
                                            int32_t timeout)
{
  return net_buf_alloc_fixed(pool, timeout);
}

static inline void net_buf_destroy(struct net_buf *buf)
{
  struct net_buf_pool *pool = buf->pool;

  if (buf->__buf)
    {
      if (!(buf->flags & NET_BUF_EXTERNAL_DATA))
        {
          pool->alloc->cb->unref(buf, buf->__buf);
        }

      buf->__buf = NULL;
    }

  list_add_head(&pool->free_list, &buf->node);

  /* Wake up a thread blocked in net_buf_alloc_len */

  nxsem_post(&pool->sem);
}

static inline void *net_buf_user_data(const struct net_buf *buf)
{
  return (void *)buf->user_data;
}

static inline struct net_buf *net_buf_take(struct net_buf **orig)
{
  struct net_buf *buf = *orig;

  *orig = NULL;
  return buf;
}

static inline void net_buf_drop(struct net_buf **orig)
{
  struct net_buf *buf = net_buf_take(orig);

  if (buf != NULL)
    {
      net_buf_unref(buf);
    }
}

/* Convenience wrappers that delegate to net_buf_simple */

static inline void net_buf_reserve(struct net_buf *buf, size_t reserve)
{
  net_buf_simple_reserve(&buf->b, reserve);
}

static inline void *net_buf_add(struct net_buf *buf, size_t len)
{
  return net_buf_simple_add(&buf->b, len);
}

static inline void *net_buf_add_mem(struct net_buf *buf, const void *mem,
                                    size_t len)
{
  return net_buf_simple_add_mem(&buf->b, mem, len);
}

static inline uint8_t *net_buf_add_u8(struct net_buf *buf, uint8_t val)
{
  return net_buf_simple_add_u8(&buf->b, val);
}

static inline void net_buf_add_le16(struct net_buf *buf, uint16_t val)
{
  net_buf_simple_add_le16(&buf->b, val);
}

static inline void net_buf_add_le24(struct net_buf *buf, uint32_t val)
{
  uint8_t *data = net_buf_add(buf, 3);

  data[0] = (uint8_t)val;
  data[1] = (uint8_t)(val >> 8);
  data[2] = (uint8_t)(val >> 16);
}

static inline void net_buf_add_be24(struct net_buf *buf, uint32_t val)
{
  uint8_t *data = net_buf_add(buf, 3);

  data[0] = (uint8_t)(val >> 16);
  data[1] = (uint8_t)(val >> 8);
  data[2] = (uint8_t)val;
}

static inline void net_buf_simple_add_le24(struct net_buf_simple *buf,
                                           uint32_t val)
{
  uint8_t *data = net_buf_simple_add(buf, 3);

  data[0] = (uint8_t)val;
  data[1] = (uint8_t)(val >> 8);
  data[2] = (uint8_t)(val >> 16);
}

static inline uint32_t net_buf_simple_pull_le24(struct net_buf_simple *buf)
{
  uint8_t *data = net_buf_simple_pull(buf, 3);

  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16);
}

static inline void net_buf_simple_push_be24(struct net_buf_simple *buf,
                                            uint32_t val)
{
  uint8_t *data = net_buf_simple_push(buf, 3);

  data[0] = (uint8_t)(val >> 16);
  data[1] = (uint8_t)(val >> 8);
  data[2] = (uint8_t)val;
}

static inline uint32_t net_buf_simple_pull_be24(struct net_buf_simple *buf)
{
  uint8_t *data = net_buf_simple_pull(buf, 3);

  return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) |
         (uint32_t)data[2];
}

static inline void net_buf_add_be16(struct net_buf *buf, uint16_t val)
{
  net_buf_simple_add_be16(&buf->b, val);
}

static inline void net_buf_add_le32(struct net_buf *buf, uint32_t val)
{
  net_buf_simple_add_le32(&buf->b, val);
}

static inline void net_buf_add_be32(struct net_buf *buf, uint32_t val)
{
  net_buf_simple_add_be32(&buf->b, val);
}

static inline void net_buf_add_le64(struct net_buf *buf, uint64_t val)
{
  net_buf_simple_add_le64(&buf->b, val);
}

static inline void net_buf_add_be64(struct net_buf *buf, uint64_t val)
{
  net_buf_simple_add_be64(&buf->b, val);
}

static inline void *net_buf_push(struct net_buf *buf, size_t len)
{
  return net_buf_simple_push(&buf->b, len);
}

static inline void *net_buf_push_mem(struct net_buf *buf, const void *mem,
                                     size_t len)
{
  return net_buf_simple_push_mem(&buf->b, mem, len);
}

static inline void net_buf_push_u8(struct net_buf *buf, uint8_t val)
{
  net_buf_simple_push_u8(&buf->b, val);
}

static inline void net_buf_push_le16(struct net_buf *buf, uint16_t val)
{
  net_buf_simple_push_le16(&buf->b, val);
}

static inline void net_buf_push_be16(struct net_buf *buf, uint16_t val)
{
  net_buf_simple_push_be16(&buf->b, val);
}

static inline void net_buf_push_le32(struct net_buf *buf, uint32_t val)
{
  net_buf_simple_push_le32(&buf->b, val);
}

static inline void net_buf_push_be32(struct net_buf *buf, uint32_t val)
{
  net_buf_simple_push_be32(&buf->b, val);
}

static inline void net_buf_push_le64(struct net_buf *buf, uint64_t val)
{
  net_buf_simple_push_le64(&buf->b, val);
}

static inline void net_buf_push_be64(struct net_buf *buf, uint64_t val)
{
  net_buf_simple_push_be64(&buf->b, val);
}

static inline void *net_buf_pull(struct net_buf *buf, size_t len)
{
  return net_buf_simple_pull(&buf->b, len);
}

static inline void *net_buf_pull_mem(struct net_buf *buf, size_t len)
{
  return net_buf_simple_pull_mem(&buf->b, len);
}

static inline uint8_t net_buf_pull_u8(struct net_buf *buf)
{
  return net_buf_simple_pull_u8(&buf->b);
}

static inline uint16_t net_buf_pull_le16(struct net_buf *buf)
{
  return net_buf_simple_pull_le16(&buf->b);
}

static inline uint16_t net_buf_pull_be16(struct net_buf *buf)
{
  return net_buf_simple_pull_be16(&buf->b);
}

static inline uint32_t net_buf_pull_be24(struct net_buf *buf)
{
  return net_buf_simple_pull_be24(&buf->b);
}

static inline uint32_t net_buf_pull_le32(struct net_buf *buf)
{
  return net_buf_simple_pull_le32(&buf->b);
}

static inline uint32_t net_buf_pull_be32(struct net_buf *buf)
{
  return net_buf_simple_pull_be32(&buf->b);
}

static inline uint64_t net_buf_pull_le64(struct net_buf *buf)
{
  return net_buf_simple_pull_le64(&buf->b);
}

static inline uint64_t net_buf_pull_be64(struct net_buf *buf)
{
  return net_buf_simple_pull_be64(&buf->b);
}

static inline void *net_buf_remove_mem(struct net_buf *buf, size_t len)
{
  return net_buf_simple_remove_mem(&buf->b, len);
}

static inline uint8_t net_buf_remove_u8(struct net_buf *buf)
{
  return net_buf_simple_remove_u8(&buf->b);
}

static inline uint16_t net_buf_remove_le16(struct net_buf *buf)
{
  return net_buf_simple_remove_le16(&buf->b);
}

static inline uint16_t net_buf_remove_be16(struct net_buf *buf)
{
  return net_buf_simple_remove_be16(&buf->b);
}

static inline uint32_t net_buf_remove_le32(struct net_buf *buf)
{
  return net_buf_simple_remove_le32(&buf->b);
}

static inline uint32_t net_buf_remove_be32(struct net_buf *buf)
{
  return net_buf_simple_remove_be32(&buf->b);
}

static inline uint64_t net_buf_remove_le64(struct net_buf *buf)
{
  return net_buf_simple_remove_le64(&buf->b);
}

static inline uint64_t net_buf_remove_be64(struct net_buf *buf)
{
  return net_buf_simple_remove_be64(&buf->b);
}

static inline uint8_t *net_buf_tail(const struct net_buf *buf)
{
  return net_buf_simple_tail(&buf->b);
}

static inline size_t net_buf_headroom(const struct net_buf *buf)
{
  return net_buf_simple_headroom(&buf->b);
}

static inline size_t net_buf_tailroom(const struct net_buf *buf)
{
  return net_buf_simple_tailroom(&buf->b);
}

/****************************************************************************
 * Name: net_buf_skip
 *
 * Description:
 *   Skip len bytes across the fragment chain.  Fully consumed fragments
 *   are removed from the chain.
 *
 *   Warning: this function will not check whether remaining data in the
 *   chain is at least len bytes.  Callers must validate beforehand.
 *
 * Returned Value:
 *   Pointer to the fragment that contains the remaining data, or NULL if
 *   all data was consumed.
 ****************************************************************************/

static inline struct net_buf *net_buf_skip(struct net_buf *buf,
                                           size_t len)
{
  while (buf != NULL && len > 0)
    {
      net_buf_pull_u8(buf);
      len--;
      if (buf->len == 0)
        {
          buf = net_buf_frag_del(NULL, buf);
        }
    }

  return buf;
}

/****************************************************************************
 * Name: net_buf_is_valid
 *
 * Description:
 *   Validate the internal consistency of a net_buf instance and its
 *   fragment chain.  Useful for DEBUGASSERT-style sanity checks.
 *
 * Returned Value:
 *   true if the buffer and fragment chain are valid, false otherwise.
 ****************************************************************************/

static inline bool net_buf_is_valid(const struct net_buf *buf)
{
  if (buf == NULL || buf->__buf == NULL || buf->pool == NULL)
    {
      return false;
    }

  if (buf->ref == 0 || buf->ref == 0xff)
    {
      return false;
    }

  if (!net_buf_simple_is_valid(&buf->b))
    {
      return false;
    }

  return (buf->frags == NULL) ? true : net_buf_is_valid(buf->frags);
}

#endif /* __INCLUDE_NUTTX_MM_NET_BUF_H */
