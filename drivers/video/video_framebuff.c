/****************************************************************************
 * drivers/video/video_framebuff.c
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
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include "video_framebuff.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void init_buf_chain(video_framebuff_t *fbuf)
{
  vbuf_container_t *tmp;
  int i;

  fbuf->vbuf_empty = fbuf->vbuf_alloced;
  fbuf->vbuf_next  = NULL;
  fbuf->vbuf_curr  = NULL;
  fbuf->vbuf_top   = NULL;
  fbuf->vbuf_tail  = NULL;

  tmp = fbuf->vbuf_alloced;
  for (i = 1; i < fbuf->container_size; i++)
    {
      tmp->next = &tmp[1];
      tmp++;
    }
}

static inline bool is_last_one(video_framebuff_t *fbuf)
{
  return fbuf->vbuf_top == fbuf->vbuf_tail;
}

static inline vbuf_container_t *dequeue_vbuf_unsafe(video_framebuff_t *fbuf)
{
  vbuf_container_t *ret = fbuf->vbuf_top;

  if (is_last_one(fbuf))
    {
      fbuf->vbuf_top  = NULL;
      fbuf->vbuf_tail = NULL;
      fbuf->vbuf_next = NULL;
    }
  else
    {
      fbuf->vbuf_top = fbuf->vbuf_top->next;
    }

  ret->next = NULL;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void video_framebuff_init(video_framebuff_t *fbuf)
{
  memset(fbuf, 0, sizeof(video_framebuff_t));
  fbuf->mode = V4L2_BUF_MODE_RING;
  nxmutex_init(&fbuf->lock_empty);
}

void video_framebuff_uninit(video_framebuff_t *fbuf)
{
  video_framebuff_realloc_container(fbuf, 0);
  nxmutex_destroy(&fbuf->lock_empty);
}

int video_framebuff_realloc_container(video_framebuff_t *fbuf, int sz)
{
  vbuf_container_t *vbuf;

  if (fbuf->container_size == sz)
    {
      return OK;
    }

  vbuf = kmm_realloc(fbuf->vbuf_alloced, sizeof(vbuf_container_t) * sz);
  if (vbuf != NULL)
    {
      memset(vbuf, 0, sizeof(vbuf_container_t) * sz);
    }
  else if (sz != 0)
    {
      return -ENOMEM;
    }

  fbuf->vbuf_alloced = vbuf;
  fbuf->container_size = sz;

  init_buf_chain(fbuf);
  return OK;
}

vbuf_container_t *video_framebuff_get_container(video_framebuff_t *fbuf)
{
  vbuf_container_t *ret;

  nxmutex_lock(&fbuf->lock_empty);
  ret = fbuf->vbuf_empty;
  if (ret != NULL)
    {
      fbuf->vbuf_empty = ret->next;
      ret->next        = NULL;
    }

  nxmutex_unlock(&fbuf->lock_empty);
  return ret;
}

void video_framebuff_free_container(video_framebuff_t *fbuf,
                                    vbuf_container_t  *cnt)
{
  nxmutex_lock(&fbuf->lock_empty);
  cnt->next = fbuf->vbuf_empty;
  fbuf->vbuf_empty = cnt;
  nxmutex_unlock(&fbuf->lock_empty);
}

void video_framebuff_queue_container(video_framebuff_t *fbuf,
                                     vbuf_container_t  *tgt)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (fbuf->vbuf_top != NULL)
    {
      fbuf->vbuf_tail->next = tgt;
      fbuf->vbuf_tail = tgt;
    }
  else
    {
      fbuf->vbuf_top = fbuf->vbuf_tail = tgt;
    }

  if (fbuf->vbuf_next == NULL)
    {
      fbuf->vbuf_next = tgt;
    }

  if (fbuf->mode == V4L2_BUF_MODE_RING)
    {
      tgt->next = fbuf->vbuf_top;
    }
  else  /* Case of V4L2_BUF_MODE_FIFO */
    {
      tgt->next = NULL;
    }

  leave_critical_section(flags);
}

vbuf_container_t *video_framebuff_dq_valid_container(video_framebuff_t *fbuf)
{
  vbuf_container_t *ret = NULL;
  irqstate_t flags;

  flags = enter_critical_section();
  if (fbuf->vbuf_top != NULL && fbuf->vbuf_top != fbuf->vbuf_next)
    {
      ret = dequeue_vbuf_unsafe(fbuf);
    }

  leave_critical_section(flags);
  return ret;
}

vbuf_container_t *
video_framebuff_get_vacant_container(video_framebuff_t *fbuf)
{
  vbuf_container_t *ret;
  irqstate_t flags;

  flags = enter_critical_section();
  ret = fbuf->vbuf_curr = fbuf->vbuf_next;
  leave_critical_section(flags);

  return ret;
}

void video_framebuff_capture_done(video_framebuff_t *fbuf)
{
  fbuf->vbuf_curr = NULL;
  if (fbuf->vbuf_next != NULL)
    {
      fbuf->vbuf_next = fbuf->vbuf_next->next;
      if (fbuf->vbuf_next == fbuf->vbuf_top)  /* RING mode case. */
        {
          fbuf->vbuf_top  = fbuf->vbuf_top->next;
          fbuf->vbuf_tail = fbuf->vbuf_tail->next;
        }
    }
}

void video_framebuff_change_mode(video_framebuff_t  *fbuf,
                                 enum v4l2_buf_mode mode)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (fbuf->mode != mode)
    {
      if (fbuf->vbuf_tail)
        {
          if (mode == V4L2_BUF_MODE_RING)
            {
              fbuf->vbuf_tail->next = fbuf->vbuf_top;
            }
          else
            {
              fbuf->vbuf_tail->next = NULL;
            }
        }

      fbuf->vbuf_next = fbuf->vbuf_top;
      fbuf->mode = mode;
    }

  leave_critical_section(flags);
}

vbuf_container_t *video_framebuff_pop_curr_container(video_framebuff_t *fbuf)
{
  vbuf_container_t *ret = NULL;
  irqstate_t flags;

  flags = enter_critical_section();
  if (fbuf->vbuf_top != NULL)
    {
      ret = dequeue_vbuf_unsafe(fbuf);
    }

  leave_critical_section(flags);
  return ret;
}
