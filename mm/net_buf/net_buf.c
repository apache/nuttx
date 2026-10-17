/****************************************************************************
 * mm/net_buf/net_buf.c
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
 * mm/net_buf/net_buf.c
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

#include <errno.h>
#include <string.h>
#include <stddef.h>
#include <assert.h>
#include <nuttx/semaphore.h>
#include <nuttx/clock.h>
#include <nuttx/mm/net_buf.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct_size
 *
 * Description:
 *   Calculate the stride between consecutive buffers in the pool based on
 *   alignment requirements.
 ****************************************************************************/

static inline size_t struct_size(struct net_buf_pool *pool)
{
  return ((sizeof(struct net_buf) + pool->user_data_size +
           (sizeof(void *) - 1)) & ~(sizeof(void *) - 1));
}

/****************************************************************************
 * Name: pool_get_uninit
 *
 * Description:
 *   Get a pointer to one of the uninitialized buffers from the pool.
 ****************************************************************************/

static inline struct net_buf *pool_get_uninit(struct net_buf_pool *pool,
                                              uint16_t uninit_count)
{
  size_t stride = struct_size(pool);
  size_t byte_offset = (pool->buf_count - uninit_count) * stride;
  struct net_buf *buf;

  buf = (struct net_buf *)(((uint8_t *)pool->__bufs) + byte_offset);

  buf->pool = pool;
  buf->user_data_size = pool->user_data_size;

  return buf;
}

/****************************************************************************
 * Name: data_alloc
 *
 * Description:
 *   Allocate data storage for a buffer.
 ****************************************************************************/

static uint8_t *data_alloc(struct net_buf *buf, size_t *size)
{
  struct net_buf_pool *pool = buf->pool;

  return pool->alloc->cb->alloc(buf, size, false);
}

/****************************************************************************
 * Name: data_ref
 *
 * Description:
 *   Add a reference to the data region.
 ****************************************************************************/

static uint8_t *data_ref(struct net_buf *buf, uint8_t *data)
{
  struct net_buf_pool *pool = buf->pool;

  return pool->alloc->cb->ref(buf, data);
}

/****************************************************************************
 * Data allocator callbacks -- Fixed
 ****************************************************************************/

/****************************************************************************
 * Name: fixed_data_alloc
 ****************************************************************************/

static uint8_t *fixed_data_alloc(struct net_buf *buf, size_t *size,
                                 bool block)
{
  struct net_buf_pool *pool = buf->pool;
  const struct net_buf_pool_fixed *fixed = pool->alloc->alloc_data;

  *size = pool->alloc->max_alloc_size;

  return fixed->data_pool + *size * net_buf_id(buf);
}

/****************************************************************************
 * Name: fixed_data_unref
 ****************************************************************************/

static void fixed_data_unref(struct net_buf *buf, uint8_t *data)
{
  /* Nothing needed for fixed-size data pools */
}

/****************************************************************************
 * Name: fixed_data_ref
 *
 * Description:
 *   Fixed pools don't support data referencing (data is per-buffer).
 *   Returns NULL so net_buf_clone falls back to full copy.
 ****************************************************************************/

static uint8_t *fixed_data_ref(struct net_buf *buf, uint8_t *data)
{
  return NULL;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct net_buf_data_cb g_net_buf_fixed_cb =
{
  .alloc = fixed_data_alloc,
  .ref   = fixed_data_ref,
  .unref = fixed_data_unref,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_buf_alloc_len
 *
 * Description:
 *   Allocate a new buffer from a pool with a specific data size.
 *   For fixed pools, size is ignored (max_alloc_size is used).
 *
 *   timeout < 0  : wait forever until a buffer is available
 *   timeout == 0 : non-blocking, return NULL immediately if pool empty
 *   timeout > 0  : wait up to timeout milliseconds
 ****************************************************************************/

struct net_buf *net_buf_alloc_len(struct net_buf_pool *pool, size_t size,
                                  int32_t timeout)
{
  struct net_buf *buf;
  irqstate_t flags;

  DEBUGASSERT(pool != NULL);

  /* Prevent race conditions when accessing pool->uninit_count */

  flags = spin_lock_irqsave(&pool->lock);

  /* If there are uninitialized buffers we're guaranteed to succeed */

  if (pool->uninit_count > 0)
    {
      uint16_t uninit_count;

      /* If this is not the first access to the pool, we can be
       * opportunistic and try to fetch a previously used buffer
       * from the free list.
       */

      if (pool->uninit_count < pool->buf_count)
        {
          struct list_node *node;

          node = list_remove_head(&pool->free_list);
          if (node != NULL)
            {
              buf = list_container_of(node, struct net_buf, node);
              spin_unlock_irqrestore(&pool->lock, flags);
              goto success;
            }
        }

      uninit_count = pool->uninit_count--;
      spin_unlock_irqrestore(&pool->lock, flags);

      buf = pool_get_uninit(pool, uninit_count);
      goto success;
    }

  /* All buffers initialized.  Try to get from free list once. */

  buf = list_remove_head_type(&pool->free_list, struct net_buf, node);

  spin_unlock_irqrestore(&pool->lock, flags);

  if (buf != NULL)
    {
      goto success;
    }

  /* Non-blocking: no buffer available */

  if (timeout == 0)
    {
      return NULL;
    }

  /* Blocking path: wait for a buffer to be freed via pool->sem.
   * Loop to handle spurious wakeups.
   */

  while (1)
    {
      int ret;

      if (timeout < 0)
        {
          ret = nxsem_wait(&pool->sem);
        }
      else
        {
          ret = nxsem_tickwait(&pool->sem, MSEC2TICK(timeout));
        }

      if (ret < 0)
        {
          return NULL;
        }

      flags = spin_lock_irqsave(&pool->lock);

      buf = list_remove_head_type(&pool->free_list, struct net_buf, node);
      if (buf != NULL)
        {
          spin_unlock_irqrestore(&pool->lock, flags);
          goto success;
        }

      spin_unlock_irqrestore(&pool->lock, flags);
    }

success:
  if (pool->alloc->max_alloc_size > 0)
    {
      size_t data_size = size;

      buf->__buf = data_alloc(buf, &data_size);
      if (buf->__buf == NULL)
        {
          net_buf_destroy(buf);
          return NULL;
        }
    }
  else
    {
      buf->__buf = NULL;
    }

  buf->ref   = 1U;
  buf->flags = 0U;
  buf->frags = NULL;
  list_clear_node(&buf->node);
  buf->size  = pool->alloc->max_alloc_size > 0 ?
    (uint16_t)pool->alloc->max_alloc_size : 0;
  memset(buf->user_data, 0, buf->user_data_size);
  net_buf_reset(buf);

  return buf;
}

/****************************************************************************
 * Name: net_buf_alloc_fixed
 *
 * Description:
 *   Allocate a buffer from a pool with the pool's default data size.
 ****************************************************************************/

struct net_buf *net_buf_alloc_fixed(struct net_buf_pool *pool,
                                    int32_t timeout)
{
  return net_buf_alloc_len(pool, pool->alloc->max_alloc_size, timeout);
}

/****************************************************************************
 * Name: net_buf_alloc_with_data
 *
 * Description:
 *   Allocate a new buffer from a pool but with external data pointer.
 ****************************************************************************/

struct net_buf *net_buf_alloc_with_data(struct net_buf_pool *pool,
                                        void *data, size_t size,
                                        int32_t timeout)
{
  struct net_buf *buf;

  buf = net_buf_alloc_len(pool, 0, timeout);
  if (buf == NULL)
    {
      return NULL;
    }

  net_buf_simple_init_with_data(&buf->b, data, size);
  buf->flags = NET_BUF_EXTERNAL_DATA;

  return buf;
}

/****************************************************************************
 * Name: net_buf_reset
 *
 * Description:
 *   Reset buffer data and flags so it can be reused for other purposes.
 ****************************************************************************/

void net_buf_reset(struct net_buf *buf)
{
  DEBUGASSERT(buf != NULL);
  DEBUGASSERT(buf->frags == NULL);

  net_buf_simple_reset(&buf->b);
}

/****************************************************************************
 * Name: net_buf_slist_put
 *
 * Description:
 *   Append a buffer to a singly-linked list (thread-safe with spinlock).
 ****************************************************************************/

static spinlock_t g_net_buf_slist_lock = SP_UNLOCKED;

void net_buf_slist_put(struct list_node *list, struct net_buf *buf)
{
  irqstate_t flags;

  DEBUGASSERT(list != NULL);
  DEBUGASSERT(buf != NULL);

  flags = spin_lock_irqsave(&g_net_buf_slist_lock);
  list_add_tail(list, &buf->node);
  spin_unlock_irqrestore(&g_net_buf_slist_lock, flags);
}

/****************************************************************************
 * Name: net_buf_slist_get
 *
 * Description:
 *   Get a buffer from a singly-linked list (thread-safe with spinlock).
 ****************************************************************************/

struct net_buf *net_buf_slist_get(struct list_node *list)
{
  struct list_node *node;
  irqstate_t flags;

  DEBUGASSERT(list != NULL);

  flags = spin_lock_irqsave(&g_net_buf_slist_lock);
  node = list_remove_head(list);
  spin_unlock_irqrestore(&g_net_buf_slist_lock, flags);

  return node != NULL ? list_container_of(node, struct net_buf, node) : NULL;
}

/****************************************************************************
 * Name: net_buf_lifo_init
 *
 * Description:
 *   Initialize a LIFO stack for inter-thread buffer passing.
 ****************************************************************************/

void net_buf_lifo_init(struct net_buf_lifo *lifo)
{
  DEBUGASSERT(lifo != NULL);

  list_initialize(&lifo->list);
  lifo->lock = SP_UNLOCKED;
  nxsem_init(&lifo->sem, 0, 0);
}

/****************************************************************************
 * Name: net_buf_lifo_put
 *
 * Description:
 *   Push a buffer onto the LIFO stack (producer side).  The caller
 *   transfers ownership of the buffer reference to the LIFO.  This
 *   function is safe to call from ISR context.
 ****************************************************************************/

void net_buf_lifo_put(struct net_buf_lifo *lifo, struct net_buf *buf)
{
  irqstate_t flags;

  DEBUGASSERT(lifo != NULL);
  DEBUGASSERT(buf != NULL);

  flags = spin_lock_irqsave(&lifo->lock);

  list_add_head(&lifo->list, &buf->node);

  spin_unlock_irqrestore(&lifo->lock, flags);

  /* Wake up one blocked consumer */

  nxsem_post(&lifo->sem);
}

/****************************************************************************
 * Name: net_buf_lifo_get
 *
 * Description:
 *   Pop a buffer from the LIFO stack (consumer side).
 *
 *   timeout < 0  : wait forever until a buffer is available
 *   timeout == 0 : non-blocking, return NULL immediately if empty
 *   timeout > 0  : wait up to timeout milliseconds
 *
 *   The caller takes ownership of the returned buffer reference.
 *
 *   Note: do NOT call with timeout != 0 from ISR context.
 ****************************************************************************/

struct net_buf *net_buf_lifo_get(struct net_buf_lifo *lifo, int32_t timeout)
{
  struct net_buf *buf;
  irqstate_t flags;

  DEBUGASSERT(lifo != NULL);

  /* Non-blocking path */

  if (timeout == 0)
    {
      flags = spin_lock_irqsave(&lifo->lock);

      buf = list_remove_head_type(&lifo->list, struct net_buf, node);

      spin_unlock_irqrestore(&lifo->lock, flags);

      return buf;
    }

  /* Blocking path (timeout < 0: forever, timeout > 0: ms timeout).
   * Loop to handle spurious wakeups and timeout restarts.
   */

  while (1)
    {
      int ret;

      if (timeout < 0)
        {
          ret = nxsem_wait(&lifo->sem);
        }
      else
        {
          ret = nxsem_tickwait(&lifo->sem, MSEC2TICK(timeout));
        }

      /* Timed out or interrupted */

      if (ret < 0)
        {
          return NULL;
        }

      flags = spin_lock_irqsave(&lifo->lock);

      buf = list_remove_head_type(&lifo->list, struct net_buf, node);
      if (buf != NULL)
        {
          spin_unlock_irqrestore(&lifo->lock, flags);
          return buf;
        }

      spin_unlock_irqrestore(&lifo->lock, flags);
    }
}

/****************************************************************************
 * Name: net_buf_unref
 *
 * Description:
 *   Decrement the reference count of a buffer and free it back to the
 *   pool when the count reaches zero. Also unreferences the fragment chain.
 ****************************************************************************/

void net_buf_unref(struct net_buf *buf)
{
  DEBUGASSERT(buf != NULL);

  while (buf != NULL)
    {
      struct net_buf *frags = buf->frags;
      struct net_buf_pool *pool;
      irqstate_t flags;
      uint8_t old_ref;

      old_ref = buf->ref;
      DEBUGASSERT(old_ref != 0);

      if (old_ref == 1)
        {
          buf->ref = 0;
        }
      else
        {
          buf->ref = old_ref - 1;
          return;
        }

      buf->data = NULL;
      buf->frags = NULL;

      pool = buf->pool;

      if (pool->destroy != NULL)
        {
          pool->destroy(buf);
        }
      else
        {
          /* Return to pool free list (LIFO) */

          flags = spin_lock_irqsave(&pool->lock);
          list_add_head(&pool->free_list, &buf->node);
          spin_unlock_irqrestore(&pool->lock, flags);
        }

      buf = frags;
    }
}

/****************************************************************************
 * Name: net_buf_ref
 *
 * Description:
 *   Increment the reference count of a buffer.
 ****************************************************************************/

struct net_buf *net_buf_ref(struct net_buf *buf)
{
  DEBUGASSERT(buf != NULL);
  DEBUGASSERT(buf->ref < 0xff);

  buf->ref++;

  return buf;
}

/****************************************************************************
 * Name: net_buf_clone
 ****************************************************************************/

struct net_buf *net_buf_clone(struct net_buf *buf, int32_t timeout)
{
  struct net_buf_pool *pool;
  struct net_buf *clone;

  DEBUGASSERT(buf != NULL);

  pool = buf->pool;

  clone = net_buf_alloc_len(pool, 0, timeout);
  if (clone == NULL)
    {
      return NULL;
    }

  /* If the pool supports data referencing use that. Otherwise
   * we need to allocate new data and make a copy.
   */

  if (buf->__buf != NULL &&
      !(buf->flags & NET_BUF_EXTERNAL_DATA))
    {
      uint8_t *ref_data = data_ref(clone, buf->__buf);

      if (ref_data != NULL)
        {
          clone->__buf = ref_data;
          clone->data = clone->__buf + net_buf_headroom(buf);
          clone->len = buf->len;
          clone->size = buf->size;
        }
      else
        {
          /* Fall back to full copy */

          size_t data_size = buf->size;

          clone->__buf = data_alloc(clone, &data_size);
          if (clone->__buf == NULL ||
              data_size < buf->size)
            {
              net_buf_destroy(clone);
              return NULL;
            }

          clone->size = data_size;
          clone->data = clone->__buf + net_buf_headroom(buf);
          net_buf_add_mem(clone, buf->data, buf->len);
        }
    }
  else
    {
      if (buf->__buf != NULL)
        {
          size_t data_size = buf->size;

          clone->__buf = data_alloc(clone, &data_size);
          if (clone->__buf == NULL ||
              data_size < buf->size)
            {
              net_buf_destroy(clone);
              return NULL;
            }

          clone->size = data_size;
          clone->data = clone->__buf + net_buf_headroom(buf);
          net_buf_add_mem(clone, buf->data, buf->len);
        }
      else
        {
          clone->data = NULL;
          clone->len = 0;
          clone->size = 0;
        }
    }

  memcpy(clone->user_data, buf->user_data, clone->user_data_size);

  return clone;
}

/****************************************************************************
 * Name: net_buf_id
 *
 * Description:
 *   Get a zero-based index for a buffer, based on its placement in its
 *   buffer pool.
 ****************************************************************************/

int net_buf_id(const struct net_buf *buf)
{
  struct net_buf_pool *pool = buf->pool;
  size_t stride = struct_size(pool);
  ptrdiff_t offset = (uint8_t *)buf - (uint8_t *)pool->__bufs;

  return offset / stride;
}

/****************************************************************************
 * Name: net_buf_user_data_copy
 ****************************************************************************/

int net_buf_user_data_copy(struct net_buf *dst, const struct net_buf *src)
{
  DEBUGASSERT(dst != NULL);
  DEBUGASSERT(src != NULL);

  if (dst == src)
    {
      return 0;
    }

  if (dst->user_data_size < src->user_data_size)
    {
      return -EINVAL;
    }

  memcpy(dst->user_data, src->user_data, src->user_data_size);

  return 0;
}

/****************************************************************************
 * Fragment Operations
 ****************************************************************************/

/****************************************************************************
 * Name: net_buf_frag_last
 ****************************************************************************/

struct net_buf *net_buf_frag_last(struct net_buf *buf)
{
  DEBUGASSERT(buf != NULL);

  while (buf->frags != NULL)
    {
      buf = buf->frags;
    }

  return buf;
}

/****************************************************************************
 * Name: net_buf_frag_insert
 ****************************************************************************/

void net_buf_frag_insert(struct net_buf *parent, struct net_buf *frag)
{
  DEBUGASSERT(parent != NULL);
  DEBUGASSERT(frag != NULL);

  if (parent->frags != NULL)
    {
      net_buf_frag_last(frag)->frags = parent->frags;
    }

  /* Take ownership of the fragment reference */

  parent->frags = frag;
}

/****************************************************************************
 * Name: net_buf_frag_add
 ****************************************************************************/

struct net_buf *net_buf_frag_add(struct net_buf *head, struct net_buf *frag)
{
  DEBUGASSERT(frag != NULL);

  if (head == NULL)
    {
      return net_buf_ref(frag);
    }

  net_buf_frag_insert(net_buf_frag_last(head), frag);

  return head;
}

/****************************************************************************
 * Name: net_buf_frag_del
 ****************************************************************************/

struct net_buf *net_buf_frag_del(struct net_buf *parent,
                                 struct net_buf *frag)
{
  struct net_buf *next_frag;

  DEBUGASSERT(frag != NULL);

  if (parent != NULL)
    {
      DEBUGASSERT(parent->frags != NULL);
      DEBUGASSERT(parent->frags == frag);
      parent->frags = frag->frags;
    }

  next_frag = frag->frags;

  frag->frags = NULL;

  net_buf_unref(frag);

  return next_frag;
}

/****************************************************************************
 * Utility Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_buf_linearize
 *
 * Description:
 *   Copy data from a fragment chain into a linear destination buffer.
 ****************************************************************************/

size_t net_buf_linearize(void *dst, size_t dst_len,
                         const struct net_buf *src,
                         size_t offset, size_t len)
{
  const struct net_buf *frag;
  size_t to_copy;
  size_t copied;

  if (len > dst_len)
    {
      len = dst_len;
    }

  frag = src;

  /* Find the right fragment to start copying from */

  while (frag != NULL && offset >= frag->len)
    {
      offset -= frag->len;
      frag = frag->frags;
    }

  /* Traverse the fragment chain until len bytes are copied */

  copied = 0;
  while (frag != NULL && len > 0)
    {
      if (len < frag->len - offset)
        {
          to_copy = len;
        }
      else
        {
          to_copy = frag->len - offset;
        }

      memcpy((uint8_t *)dst + copied, frag->data + offset, to_copy);

      copied += to_copy;

      /* to_copy is always <= len */

      len -= to_copy;
      frag = frag->frags;

      /* After the first iteration, this value will be 0 */

      offset = 0;
    }

  return copied;
}

/****************************************************************************
 * Name: net_buf_append_bytes
 *
 * Description:
 *   Append multiple bytes to a buffer chain, creating new fragments as
 *   needed.
 ****************************************************************************/

size_t net_buf_append_bytes(struct net_buf *buf, size_t len,
                            const void *value, int32_t timeout,
                            net_buf_allocator_cb allocate_cb,
                            void *user_data)
{
  struct net_buf *tail = net_buf_frag_last(buf);
  struct net_buf *frag = tail;
  size_t added_len = 0;
  const uint8_t *value8 = value;

  do
    {
      uint16_t count;

      if (len < net_buf_tailroom(frag))
        {
          count = len;
        }
      else
        {
          count = net_buf_tailroom(frag);
        }

      net_buf_add_mem(frag, value8, count);
      len -= count;
      added_len += count;
      value8 += count;

      if (len == 0)
        {
          return added_len;
        }

      if (allocate_cb != NULL)
        {
          frag = allocate_cb(timeout, user_data);
        }
      else
        {
          /* Allocate from the same pool */

          struct net_buf_pool *pool = buf->pool;
          size_t max_size = pool->alloc->max_alloc_size;

          frag = net_buf_alloc_len(pool,
                  max_size > 0 ? (len < max_size ? len : max_size) : len,
                  timeout);
        }

      if (frag == NULL)
        {
          return added_len;
        }

      net_buf_frag_insert(tail, frag);
      tail = net_buf_frag_last(frag);
    }
  while (1);

  /* Unreachable */

  return 0;
}

/****************************************************************************
 * Name: net_buf_data_match
 *
 * Description:
 *   Match data in a fragment chain against a byte sequence.
 ****************************************************************************/

size_t net_buf_data_match(const struct net_buf *buf, size_t offset,
                          const void *data, size_t len)
{
  const uint8_t *dptr = data;
  const uint8_t *bptr;
  size_t compared = 0;
  size_t to_compare;
  size_t i;

  if (buf == NULL || data == NULL)
    {
      return compared;
    }

  /* Find the right fragment to start comparison */

  while (buf != NULL && offset >= buf->len)
    {
      offset -= buf->len;
      buf = buf->frags;
    }

  while (buf != NULL && len > 0)
    {
      bptr = buf->data + offset;

      if (len < buf->len - offset)
        {
          to_compare = len;
        }
      else
        {
          to_compare = buf->len - offset;
        }

      for (i = 0; i < to_compare; ++i)
        {
          if (dptr[compared] != bptr[i])
            {
              return compared;
            }

          compared++;
        }

      len -= to_compare;
      buf = buf->frags;
      offset = 0;
    }

  return compared;
}

/****************************************************************************
 * Name: net_buf_frags_len
 *
 * Description:
 *   Calculate the total length of a fragment chain.
 ****************************************************************************/

size_t net_buf_frags_len(struct net_buf *buf)
{
  size_t bytes = 0;

  while (buf != NULL)
    {
      bytes += buf->len;
      buf = buf->frags;
    }

  return bytes;
}
