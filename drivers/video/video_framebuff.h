/****************************************************************************
 * drivers/video/video_framebuff.h
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

#ifndef __VIDEO_VIDEO_FRAMEBUFF_H__
#define __VIDEO_VIDEO_FRAMEBUFF_H__

#include <nuttx/video/video.h>
#include <nuttx/semaphore.h>

struct vbuf_container_s
{
  struct v4l2_buffer       buf;    /* Buffer information */
  struct vbuf_container_s *next;  /* pointer to next buffer */
};

typedef struct vbuf_container_s vbuf_container_t;

struct video_framebuff_s
{
  enum v4l2_buf_mode  mode;
  sem_t lock_empty;
  int container_size;
  vbuf_container_t *vbuf_alloced;
  vbuf_container_t *vbuf_empty;
  vbuf_container_t *vbuf_top;
  vbuf_container_t *vbuf_tail;
  vbuf_container_t *vbuf_dma;
  vbuf_container_t *vbuf_next_dma;
};

typedef struct video_framebuff_s video_framebuff_t;

/* Buffer access interface. */

void              video_framebuff_init
                       (video_framebuff_t *fbuf);
void              video_framebuff_uninit
                       (video_framebuff_t *fbuf);
int               video_framebuff_realloc_container
                       (video_framebuff_t *fbuf, int sz);
vbuf_container_t *video_framebuff_get_container
                       (video_framebuff_t *fbuf);
void              video_framebuff_free_container
                       (video_framebuff_t *fbuf, vbuf_container_t *cnt);
void              video_framebuff_queue_container
                       (video_framebuff_t *fbuf, vbuf_container_t *tgt);
vbuf_container_t *video_framebuff_dq_valid_container
                       (video_framebuff_t *fbuf);
vbuf_container_t *video_framebuff_get_dma_container
                       (video_framebuff_t *fbuf);
vbuf_container_t *video_framebuff_pop_curr_container
                       (video_framebuff_t *fbuf);
void              video_framebuff_dma_done
                       (video_framebuff_t *fbuf);
void              video_framebuff_change_mode
                       (video_framebuff_t *fbuf, enum v4l2_buf_mode mode);

#endif // __VIDEO_VIDEO_FRAMEBUFF_H__
