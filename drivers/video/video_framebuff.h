/****************************************************************************
 * drivers/video/video_framebuff.h
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

#ifndef __VIDEO_VIDEO_FRAMEBUFF_H__
#define __VIDEO_VIDEO_FRAMEBUFF_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/video/video.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Functions Definistions
 ****************************************************************************/

struct vbuf_container_s
{
  struct v4l2_buffer       buf;   /* Buffer information */
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
