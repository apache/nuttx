/****************************************************************************
 * drivers/video/video_framebuff.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include "video_framebuff.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void init_buf_chain(video_framebuff_t *fbuf)
{
  int i;
  vbuf_container_t *tmp;

  fbuf->vbuf_empty    = fbuf->vbuf_alloced;
  fbuf->vbuf_next_dma = NULL;
  fbuf->vbuf_dma      = NULL;
  fbuf->vbuf_top      = NULL;
  fbuf->vbuf_tail     = NULL;

  tmp = fbuf->vbuf_alloced;
  for (i = 0; i < fbuf->container_size - 1; i++)
    {
      tmp->next = &tmp[1];
      tmp++;
    }
}

static void cleanup_container(video_framebuff_t *fbuf)
{
  if (fbuf->vbuf_alloced)
    {
      memset(fbuf->vbuf_alloced,
             0,
             sizeof(vbuf_container_t)*fbuf->container_size);
      init_buf_chain(fbuf);
    }
}

static inline int is_last_one(video_framebuff_t *fbuf)
{
  return fbuf->vbuf_top == fbuf->vbuf_tail ? 1 : 0;
}

static inline vbuf_container_t *dequeue_vbuf_unsafe(video_framebuff_t *fbuf)
{
  vbuf_container_t *ret = fbuf->vbuf_top;
  if (is_last_one(fbuf))
    {
      fbuf->vbuf_top      = NULL;
      fbuf->vbuf_tail     = NULL;
      fbuf->vbuf_next_dma = NULL;
    }
  else
    {
      if (fbuf->mode == V4L2_BUF_MODE_RING)
        {
          fbuf->vbuf_tail->next = fbuf->vbuf_top->next;
        }
      fbuf->vbuf_top = fbuf->vbuf_top->next;
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void video_framebuff_init(video_framebuff_t *fbuf)
{
  fbuf->mode = V4L2_BUF_MODE_RING;
  fbuf->vbuf_empty    = NULL;
  fbuf->vbuf_top      = NULL;
  fbuf->vbuf_tail     = NULL;
  fbuf->vbuf_next_dma = NULL;

  nxsem_init(&fbuf->lock_empty, 0, 1);
}

void video_framebuff_uninit(video_framebuff_t *fbuf)
{
  video_framebuff_realloc_container(fbuf, 0);
  nxsem_destroy(&fbuf->lock_empty);
}

int video_framebuff_realloc_container(video_framebuff_t *fbuf, int sz)
{
  if (sz > V4L2_REQBUFS_COUNT_MAX)
    {
      return -EINVAL;
    }

  if (fbuf->vbuf_alloced == NULL || fbuf->container_size != sz)
    {
        if (fbuf->container_size != sz)
          {
            if (fbuf->vbuf_alloced != NULL)
              {
                kmm_free(fbuf->vbuf_alloced);
              }
            fbuf->vbuf_alloced   = NULL;
            fbuf->container_size = 0;
          }
        if (sz > 0)
          {
            fbuf->vbuf_alloced
             = (vbuf_container_t *)kmm_malloc(sizeof(vbuf_container_t)*sz);
            if (fbuf->vbuf_alloced == NULL)
              {
                return -ENOMEM;
              }
          }
        fbuf->container_size = sz;
    }

  cleanup_container(fbuf);

  return OK;
}

vbuf_container_t *video_framebuff_get_container(video_framebuff_t *fbuf)
{
  vbuf_container_t *ret;

  nxsem_wait(&fbuf->lock_empty);
  ret = fbuf->vbuf_empty;
  if (ret)
    {
      fbuf->vbuf_empty = ret->next;
      ret->next        = NULL;
    }
  nxsem_post(&fbuf->lock_empty);

  return ret;
}

void video_framebuff_free_container(video_framebuff_t *fbuf,
                                    vbuf_container_t  *cnt)
{
  nxsem_wait(&fbuf->lock_empty);
  cnt->next = fbuf->vbuf_empty;
  fbuf->vbuf_empty = cnt;
  nxsem_post(&fbuf->lock_empty);
}

void video_framebuff_queue_container(video_framebuff_t *fbuf,
                                     vbuf_container_t  *tgt)
{
  irqstate_t flags;

  flags = enter_critical_section();
  if (fbuf->vbuf_top)
    {
      fbuf->vbuf_tail->next = tgt;
      fbuf->vbuf_tail = tgt;
      if (fbuf->vbuf_next_dma == NULL)
        {
          fbuf->vbuf_next_dma = tgt;
        }
    }
  else
    {
      fbuf->vbuf_top = fbuf->vbuf_tail = tgt;
      fbuf->vbuf_next_dma = tgt;
    }

  if (fbuf->mode == V4L2_BUF_MODE_RING)
    {
      fbuf->vbuf_tail->next = fbuf->vbuf_top;
    }
  else  /* Case of V4L2_BUF_MODE_FIFO */
    {
      fbuf->vbuf_tail->next = NULL;
    }
  leave_critical_section(flags);
}

vbuf_container_t *video_framebuff_dq_valid_container(video_framebuff_t *fbuf)
{
  irqstate_t flags;
  vbuf_container_t *ret = NULL;

  flags = enter_critical_section();
  if (fbuf->vbuf_top != NULL && fbuf->vbuf_top != fbuf->vbuf_next_dma)
    {
      ret = dequeue_vbuf_unsafe(fbuf);
    }
  leave_critical_section(flags);

  return ret;
}

vbuf_container_t *video_framebuff_get_dma_container(video_framebuff_t *fbuf)
{
  irqstate_t flags;
  vbuf_container_t *ret;

  flags = enter_critical_section();
  ret = fbuf->vbuf_dma = fbuf->vbuf_next_dma;
  leave_critical_section(flags);

  return ret;
}

void video_framebuff_dma_done(video_framebuff_t *fbuf)
{
  fbuf->vbuf_dma = NULL;
  if (fbuf->vbuf_next_dma)
    {
      fbuf->vbuf_next_dma = fbuf->vbuf_next_dma->next;
      if (fbuf->vbuf_next_dma == fbuf->vbuf_top)  /* RING mode case. */
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
              fbuf->vbuf_next_dma = fbuf->vbuf_top;
            }
          else
            {
              fbuf->vbuf_tail->next = NULL;
              fbuf->vbuf_next_dma = fbuf->vbuf_top;
            }
        }
      fbuf->mode = mode;
    }
  leave_critical_section(flags);
}

vbuf_container_t *video_framebuff_pop_curr_container(video_framebuff_t *fbuf)
{
  irqstate_t flags;
  vbuf_container_t *ret = NULL;

  flags = enter_critical_section();
  if (fbuf->vbuf_top != NULL)
    {
      ret = dequeue_vbuf_unsafe(fbuf);
    }

  leave_critical_section(flags);

  return ret;
}
