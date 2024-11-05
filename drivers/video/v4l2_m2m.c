/****************************************************************************
 * drivers/video/v4l2_m2m.c
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

#include <debug.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>

#include <nuttx/sched.h>
#include <nuttx/video/v4l2_m2m.h>
#include <nuttx/video/video.h>

#include "video_framebuff.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Offset base for buffers on the destination queue - used to distinguish
 * between source and destination buffers when mmapping - they receive the
 * same offsets but for different queues.
 */

#define CAPTURE_BUF_OFFSET (1 << 30)
#define CODEC_EVENT_COUNT  6

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct codec_event_s
{
  sq_entry_t        entry;
  struct v4l2_event event;
};

typedef struct codec_event_s codec_event_t;

struct codec_type_inf_s
{
  video_framebuff_t bufinf;
  FAR uint8_t       *bufheap;   /* for V4L2_MEMORY_MMAP buffers */
  bool              buflast;
};

typedef struct codec_type_inf_s codec_type_inf_t;

struct codec_file_s
{
  codec_type_inf_t  capture_inf;
  codec_type_inf_t  output_inf;
  sq_queue_t        event_avail;
  sq_queue_t        event_free;
  codec_event_t     event_pool[CODEC_EVENT_COUNT];
  FAR struct pollfd *fds;
  FAR void          *priv;
};

typedef struct codec_file_s codec_file_t;

struct codec_mng_s
{
  struct v4l2_s v4l2;
  FAR struct codec_s *codec;
};

typedef struct codec_mng_s codec_mng_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods. */

static int codec_open(FAR struct file *filep);
static int codec_close(FAR struct file *filep);
static int codec_mmap(FAR struct file *filep,
                      FAR struct mm_map_entry_s *map);
static int codec_poll(FAR struct file *filep,
                      FAR struct pollfd *fds, bool setup);

/* Common function */

static FAR codec_type_inf_t *
codec_get_type_inf(FAR struct codec_file_s *cfile, int type);

/* ioctl function for each cmds of ioctl */

static int codec_querycap(FAR struct file *filep,
                          FAR struct v4l2_capability *cap);
static int codec_reqbufs(FAR struct file *filep,
                         FAR struct v4l2_requestbuffers *reqbufs);
static int codec_querybuf(FAR struct file *filep,
                          FAR struct v4l2_buffer *buf);
static int codec_qbuf(FAR struct file *filep,
                      FAR struct v4l2_buffer *buf);
static int codec_dqbuf(FAR struct file *filep,
                       FAR struct v4l2_buffer *buf);
static int codec_g_fmt(FAR struct file *filep,
                       FAR struct v4l2_format *fmt);
static int codec_s_fmt(FAR struct file *filep,
                       FAR struct v4l2_format *fmt);
static int codec_try_fmt(FAR struct file *filep,
                         FAR struct v4l2_format *fmt);
static int codec_g_parm(FAR struct file *filep,
                        FAR struct v4l2_streamparm *parm);
static int codec_s_parm(FAR struct file *filep,
                        FAR struct v4l2_streamparm *parm);
static int codec_streamon(FAR struct file *filep,
                          FAR enum v4l2_buf_type *type);
static int codec_streamoff(FAR struct file *filep,
                           FAR enum v4l2_buf_type *type);
static int codec_g_selection(FAR struct file *filep,
                             FAR struct v4l2_selection *clip);
static int codec_s_selection(FAR struct file *filep,
                             FAR struct v4l2_selection *clip);
static int codec_g_ext_ctrls(FAR struct file *filep,
                             FAR struct v4l2_ext_controls *ctrls);
static int codec_s_ext_ctrls(FAR struct file *filep,
                             FAR struct v4l2_ext_controls *ctrls);
static int codec_enum_fmt(FAR struct file *filep,
                          FAR struct v4l2_fmtdesc *fmt);
static int codec_cropcap(FAR struct file *filep,
                         FAR struct v4l2_cropcap *cropcap);
static int codec_dqevent(FAR struct file *filep,
                         FAR struct v4l2_event *event);
static int codec_subscribe_event(FAR struct file *filep,
                                 FAR struct v4l2_event_subscription *sub);
static int codec_decoder_cmd(FAR struct file *filep,
                             FAR struct v4l2_decoder_cmd *cmd);
static int codec_encoder_cmd(FAR struct file *filep,
                             FAR struct v4l2_encoder_cmd *cmd);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct v4l2_ops_s g_codec_vops =
{
  codec_querycap,        /* querycap */
  NULL,                  /* g_input */
  NULL,                  /* enum_input */
  codec_reqbufs,         /* reqbufs */
  codec_querybuf,        /* querybuf */
  codec_qbuf,            /* qbuf */
  codec_dqbuf,           /* dqbuf */
  NULL,                  /* cancel_dqbuf */
  codec_g_fmt,           /* g_fmt */
  codec_s_fmt,           /* s_fmt */
  codec_try_fmt,         /* try_fmt */
  codec_g_parm,          /* g_parm */
  codec_s_parm,          /* s_parm */
  codec_streamon,        /* streamon */
  codec_streamoff,       /* streamoff */
  NULL,                  /* do_halfpush */
  NULL,                  /* takepict_start */
  NULL,                  /* takepict_stop */
  codec_s_selection,     /* s_selection */
  codec_g_selection,     /* g_selection */
  NULL,                  /* queryctrl */
  NULL,                  /* query_ext_ctrl */
  NULL,                  /* querymenu */
  NULL,                  /* g_ctrl */
  NULL,                  /* s_ctrl */
  codec_g_ext_ctrls,     /* g_ext_ctrls */
  codec_s_ext_ctrls,     /* s_ext_ctrls */
  NULL,                  /* query_ext_ctrl_scene */
  NULL,                  /* querymenu_scene */
  NULL,                  /* g_ext_ctrls_scene */
  NULL,                  /* s_ext_ctrls_scene */
  codec_enum_fmt,        /* enum_fmt */
  NULL,                  /* enum_frminterval */
  NULL,                  /* enum_frmsize */
  codec_cropcap,         /* cropcap */
  codec_dqevent,         /* dqevent */
  codec_subscribe_event, /* subscribe_event */
  codec_decoder_cmd,     /* decoder_cmd */
  codec_encoder_cmd      /* encoder_cmd */
};

static const struct file_operations g_codec_fops =
{
  codec_open,            /* open */
  codec_close,           /* close */
  NULL,                  /* read */
  NULL,                  /* write */
  NULL,                  /* seek */
  NULL,                  /* ioctl */
  codec_mmap,            /* mmap */
  NULL,                  /* truncate */
  codec_poll,            /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR codec_type_inf_t *
codec_get_type_inf(FAR struct codec_file_s *cfile, int type)
{
  if (V4L2_TYPE_IS_OUTPUT(type))
    {
      return &cfile->output_inf;
    }
  else
    {
      return &cfile->capture_inf;
    }
}

static int codec_querycap(FAR struct file *filep,
                          FAR struct v4l2_capability *cap)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  return CODEC_QUERYCAP(cmng->codec, cfile->priv, cap);
}

static int codec_enum_fmt(FAR struct file *filep,
                          FAR struct v4l2_fmtdesc *fmt)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (fmt == NULL)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(fmt->type))
    {
      return CODEC_OUTPUT_ENUM_FMT(cmng->codec, cfile->priv, fmt);
    }
  else
    {
      return CODEC_CAPTURE_ENUM_FMT(cmng->codec, cfile->priv, fmt);
    }
}

static int codec_reqbufs(FAR struct file *filep,
                         FAR struct v4l2_requestbuffers *reqbufs)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;
  FAR codec_type_inf_t *type_inf;
  irqstate_t flags;
  size_t buf_size;
  int ret = OK;

  if (reqbufs == NULL)
    {
      return -EINVAL;
    }

  reqbufs->mode = V4L2_BUF_MODE_FIFO;
  if (reqbufs->count > V4L2_REQBUFS_COUNT_MAX)
    {
      reqbufs->count = V4L2_REQBUFS_COUNT_MAX;
    }

  if (V4L2_TYPE_IS_OUTPUT(reqbufs->type))
    {
      buf_size = CODEC_OUTPUT_G_BUFSIZE(cmng->codec, cfile->priv);
    }
  else
    {
      buf_size = CODEC_CAPTURE_G_BUFSIZE(cmng->codec, cfile->priv);
    }

  if (buf_size == 0)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  type_inf = codec_get_type_inf(cfile, reqbufs->type);
  video_framebuff_change_mode(&type_inf->bufinf, reqbufs->mode);
  ret = video_framebuff_realloc_container(&type_inf->bufinf,
                                          reqbufs->count);
  if (ret == 0 && reqbufs->memory == V4L2_MEMORY_MMAP)
    {
      kumm_free(type_inf->bufheap);
      type_inf->bufheap = kumm_memalign(32, reqbufs->count * buf_size);
      if (type_inf->bufheap == NULL)
        {
          ret = -ENOMEM;
        }
    }

  leave_critical_section(flags);
  return ret;
}

static int codec_querybuf(FAR struct file *filep,
                          FAR struct v4l2_buffer *buf)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;
  FAR codec_type_inf_t *type_inf;

  if (buf == NULL || buf->memory != V4L2_MEMORY_MMAP)
    {
      return -EINVAL;
    }

  type_inf = codec_get_type_inf(cfile, buf->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (buf->index >= type_inf->bufinf.container_size)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(buf->type))
    {
      buf->length   = CODEC_OUTPUT_G_BUFSIZE(cmng->codec, cfile->priv);
      buf->m.offset = buf->length * buf->index;
    }
  else
    {
      buf->length   = CODEC_CAPTURE_G_BUFSIZE(cmng->codec, cfile->priv);
      buf->m.offset = buf->length * buf->index + CAPTURE_BUF_OFFSET;
    }

  if (buf->length == 0)
    {
      return -EINVAL;
    }

  return OK;
}

static int codec_qbuf(FAR struct file *filep,
                      FAR struct v4l2_buffer *buf)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;
  FAR codec_type_inf_t *type_inf;
  FAR vbuf_container_t *container;
  size_t buf_size;

  if (buf == NULL)
    {
      return -EINVAL;
    }

  type_inf = codec_get_type_inf(cfile, buf->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  container = video_framebuff_get_container(&type_inf->bufinf);
  if (container == NULL)
    {
      vwarn("get container failed\n");
      return -EAGAIN;
    }

  memcpy(&container->buf, buf, sizeof(struct v4l2_buffer));
  if (buf->memory == V4L2_MEMORY_MMAP)
    {
      /* only use userptr inside the container */

      if (V4L2_TYPE_IS_OUTPUT(buf->type))
        {
          buf_size = CODEC_OUTPUT_G_BUFSIZE(cmng->codec, cfile->priv);
        }
      else
        {
          buf_size = CODEC_CAPTURE_G_BUFSIZE(cmng->codec, cfile->priv);
        }

      if (buf_size == 0)
        {
          return -EINVAL;
        }

      container->buf.length    = buf_size;
      container->buf.m.userptr = (unsigned long)(type_inf->bufheap +
                                 container->buf.length * buf->index);
    }

  video_framebuff_queue_container(&type_inf->bufinf, container);

  if (V4L2_TYPE_IS_OUTPUT(buf->type))
    {
      return CODEC_OUTPUT_AVAILABLE(cmng->codec, cfile->priv);
    }
  else
    {
      return CODEC_CAPTURE_AVAILABLE(cmng->codec, cfile->priv);
    }
}

static int codec_dqbuf(FAR struct file *filep,
                       FAR struct v4l2_buffer *buf)
{
  FAR codec_file_t *cfile = filep->f_priv;
  FAR codec_type_inf_t *type_inf;
  FAR vbuf_container_t *container;
  irqstate_t flags;

  if (buf == NULL)
    {
      return -EINVAL;
    }

  type_inf = codec_get_type_inf(cfile, buf->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  if (video_framebuff_is_empty(&type_inf->bufinf))
    {
      leave_critical_section(flags);
      return -EAGAIN;
    }

  container = video_framebuff_dq_valid_container(&type_inf->bufinf);
  if (container == NULL)
    {
      leave_critical_section(flags);
      return -EAGAIN;
    }

  memcpy(buf, &container->buf, sizeof(struct v4l2_buffer));
  video_framebuff_free_container(&type_inf->bufinf, container);

  vinfo("%s dequeue done\n", V4L2_TYPE_IS_OUTPUT(buf->type) ?
                             "output" : "capture");

  leave_critical_section(flags);
  return OK;
}

static int codec_s_selection(FAR struct file *filep,
                             FAR struct v4l2_selection *clip)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (clip == NULL)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(clip->type))
    {
      return CODEC_OUTPUT_S_SELECTION(cmng->codec, cfile->priv, clip);
    }
  else
    {
      return CODEC_CAPTURE_S_SELECTION(cmng->codec, cfile->priv, clip);
    }
}

static int codec_g_selection(FAR struct file *filep,
                             FAR struct v4l2_selection *clip)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (clip == NULL)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(clip->type))
    {
      return CODEC_OUTPUT_G_SELECTION(cmng->codec, cfile->priv, clip);
    }
  else
    {
      return CODEC_CAPTURE_G_SELECTION(cmng->codec, cfile->priv, clip);
    }
}

static int codec_g_ext_ctrls(FAR struct file *filep,
                             FAR struct v4l2_ext_controls *ctrls)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (ctrls == NULL)
    {
      return -EINVAL;
    }

  return CODEC_G_EXT_CTRLS(cmng->codec, cfile->priv, ctrls);
}

static int codec_s_ext_ctrls(FAR struct file *filep,
                             FAR struct v4l2_ext_controls *ctrls)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (ctrls == NULL)
    {
      return -EINVAL;
    }

  return CODEC_S_EXT_CTRLS(cmng->codec, cfile->priv, ctrls);
}

static int codec_try_fmt(FAR struct file *filep,
                         FAR struct v4l2_format *fmt)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (fmt == NULL)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(fmt->type))
    {
      return CODEC_OUTPUT_TRY_FMT(cmng->codec, cfile->priv, fmt);
    }
  else
    {
      return CODEC_CAPTURE_TRY_FMT(cmng->codec, cfile->priv, fmt);
    }
}

static int codec_g_fmt(FAR struct file *filep,
                       FAR struct v4l2_format *fmt)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (fmt == NULL)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(fmt->type))
    {
      return CODEC_OUTPUT_G_FMT(cmng->codec, cfile->priv, fmt);
    }
  else
    {
      return CODEC_CAPTURE_G_FMT(cmng->codec, cfile->priv, fmt);
    }
}

static int codec_s_fmt(FAR struct file *filep,
                       FAR struct v4l2_format *fmt)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (fmt == NULL)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(fmt->type))
    {
      return CODEC_OUTPUT_S_FMT(cmng->codec, cfile->priv, fmt);
    }
  else
    {
      return CODEC_CAPTURE_S_FMT(cmng->codec, cfile->priv, fmt);
    }
}

static int codec_g_parm(FAR struct file *filep,
                        FAR struct v4l2_streamparm *parm)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (parm == NULL)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(parm->type))
    {
      return CODEC_OUTPUT_G_PARM(cmng->codec, cfile->priv, parm);
    }
  else
    {
      return CODEC_CAPTURE_G_PARM(cmng->codec, cfile->priv, parm);
    }
}

static int codec_s_parm(FAR struct file *filep,
                          FAR struct v4l2_streamparm *parm)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (parm == NULL)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(parm->type))
    {
      return CODEC_OUTPUT_S_PARM(cmng->codec, cfile->priv, parm);
    }
  else
    {
      return CODEC_CAPTURE_S_PARM(cmng->codec, cfile->priv, parm);
    }
}

static int codec_streamon(FAR struct file *filep,
                          FAR enum v4l2_buf_type *type)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;
  FAR codec_type_inf_t *type_inf;

  if (type == NULL)
    {
      return -EINVAL;
    }

  type_inf = codec_get_type_inf(cfile, *type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  type_inf->buflast = false;

  if (V4L2_TYPE_IS_OUTPUT(*type))
    {
      return CODEC_OUTPUT_STREAMON(cmng->codec, cfile->priv);
    }
  else
    {
      return  CODEC_CAPTURE_STREAMON(cmng->codec, cfile->priv);
    }
}

static int codec_streamoff(FAR struct file *filep,
                           FAR enum v4l2_buf_type *type)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;
  FAR codec_type_inf_t *type_inf;

  if (type == NULL)
    {
      return -EINVAL;
    }

  type_inf = codec_get_type_inf(cfile, *type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(*type))
    {
      return CODEC_OUTPUT_STREAMOFF(cmng->codec, cfile->priv);
    }
  else
    {
      return CODEC_CAPTURE_STREAMOFF(cmng->codec, cfile->priv);
    }
}

int codec_cropcap(FAR struct file *filep,
                  FAR struct v4l2_cropcap *cropcap)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (cropcap == NULL)
    {
      return -EINVAL;
    }

  if (V4L2_TYPE_IS_OUTPUT(cropcap->type))
    {
      return CODEC_OUTPUT_CROPCAP(cmng->codec, cfile->priv, cropcap);
    }
  else
    {
      return CODEC_CAPTURE_CROPCAP(cmng->codec, cfile->priv, cropcap);
    }
}

int codec_dqevent(FAR struct file *filep,
                  FAR struct v4l2_event *event)
{
  FAR codec_file_t *cfile = filep->f_priv;
  FAR codec_event_t *cevt;
  irqstate_t flags;

  if (event == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  if (sq_empty(&cfile->event_avail))
    {
      leave_critical_section(flags);
      return -ENOENT;
    }

  cevt = (FAR codec_event_t *)sq_remfirst(&cfile->event_avail);
  memcpy(event, &cevt->event, sizeof(struct v4l2_event));
  sq_addlast((FAR sq_entry_t *)cevt, &cfile->event_free);

  leave_critical_section(flags);
  return OK;
}

int codec_subscribe_event(FAR struct file *filep,
                          FAR struct v4l2_event_subscription *sub)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (sub == NULL)
    {
      return -EINVAL;
    }

  return CODEC_SUBSCRIBE_EVENT(cmng->codec, cfile->priv, sub);
}

int codec_decoder_cmd(FAR struct file *filep,
                      FAR struct v4l2_decoder_cmd *cmd)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (cmd == NULL)
    {
      return -EINVAL;
    }

  return CODEC_DECODER_CMD(cmng->codec, cfile->priv, cmd);
}

int codec_encoder_cmd(FAR struct file *filep,
                      FAR struct v4l2_encoder_cmd *cmd)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  if (cmd == NULL)
    {
      return -EINVAL;
    }

  return CODEC_ENCODER_CMD(cmng->codec, cfile->priv, cmd);
}

/* file operations */

static int codec_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct codec_mng_s *cmng = inode->i_private;
  FAR struct codec_file_s *cfile;
  int ret;
  int i;

  cfile = kmm_zalloc(sizeof(struct codec_file_s));
  if (cfile == NULL)
    {
      return -ENOMEM;
    }

  filep->f_priv = cfile;

  ret = CODEC_OPEN(cmng->codec, cfile, &cfile->priv);
  if (ret != OK)
    {
      kmm_free(cfile);
      return ret;
    }

  sq_init(&cfile->event_avail);
  sq_init(&cfile->event_free);

  for (i = 0; i < CODEC_EVENT_COUNT; i++)
    {
      sq_addlast((FAR sq_entry_t *)&cfile->event_pool[i],
                 &cfile->event_free);
    }

  video_framebuff_init(&cfile->capture_inf.bufinf);
  video_framebuff_init(&cfile->output_inf.bufinf);

  return OK;
}

static int codec_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;

  CODEC_CLOSE(cmng->codec, cfile->priv);

  video_framebuff_uninit(&cfile->capture_inf.bufinf);
  video_framebuff_uninit(&cfile->output_inf.bufinf);
  kumm_free(cfile->capture_inf.bufheap);
  kumm_free(cfile->output_inf.bufheap);
  kmm_free(cfile);

  return OK;
}

static int codec_munmap(FAR struct task_group_s *group,
                        FAR struct mm_map_entry_s *entry,
                        FAR void *start, size_t length)
{
  return mm_map_remove(get_group_mm(group), entry);
}

static int codec_mmap(FAR struct file *filep,
                      FAR struct mm_map_entry_s *map)
{
  FAR struct inode *inode = filep->f_inode;
  FAR codec_mng_t *cmng = inode->i_private;
  FAR codec_file_t *cfile = filep->f_priv;
  FAR codec_type_inf_t *type_inf;
  int ret = -EINVAL;
  size_t total_size;
  size_t buf_size;

  if (map == NULL)
    {
      return -EINVAL;
    }

  if (map->offset < CAPTURE_BUF_OFFSET)
    {
      type_inf = &cfile->output_inf;
      buf_size = CODEC_OUTPUT_G_BUFSIZE(cmng->codec, cfile->priv);
    }
  else
    {
      type_inf     = &cfile->capture_inf;
      map->offset -= CAPTURE_BUF_OFFSET;
      buf_size     = CODEC_CAPTURE_G_BUFSIZE(cmng->codec, cfile->priv);
    }

  if (buf_size == 0)
    {
      return -EINVAL;
    }

  total_size = type_inf->bufinf.container_size * buf_size;
  if (map->offset >= 0 && map->offset < total_size &&
      map->length && map->offset + map->length <= total_size)
    {
      map->vaddr  = type_inf->bufheap + map->offset;
      map->munmap = codec_munmap;
      ret = mm_map_add(get_current_mm(), map);
    }

  return ret;
}

static int codec_poll(FAR struct file *filep,
                      FAR struct pollfd *fds, bool setup)
{
  FAR codec_file_t *cfile = filep->f_priv;
  pollevent_t eventset = 0;
  irqstate_t flags;

  flags = enter_critical_section();

  if (setup)
    {
      if (cfile->fds == NULL)
        {
          cfile->fds = fds;
          fds->priv  = &cfile->fds;

          if (!video_framebuff_is_empty(&cfile->output_inf.bufinf))
            {
              eventset |= POLLOUT;
            }

          if (cfile->capture_inf.buflast ||
              !video_framebuff_is_empty(&cfile->capture_inf.bufinf))
            {
              eventset |= POLLIN;
            }

          if (!sq_empty(&cfile->event_avail))
            {
              eventset |= POLLPRI;
            }

          if (eventset > 0)
            {
              poll_notify(&cfile->fds, 1, eventset);
            }
        }
      else
        {
          leave_critical_section(flags);
          return -EBUSY;
        }
    }
  else if (fds->priv)
    {
      cfile->fds = NULL;
      fds->priv  = NULL;
    }

  leave_critical_section(flags);
  return OK;
}

static FAR struct v4l2_buffer *codec_get_buf(FAR codec_type_inf_t *type_inf)
{
  FAR vbuf_container_t *container;

  container = video_framebuff_get_vacant_container(&type_inf->bufinf);
  if (container == NULL)
    {
      vinfo("No buffer available\n");
      return NULL;
    }

  return &container->buf;
}

static int codec_put_buf(FAR codec_file_t *cfile,
                         FAR codec_type_inf_t *type_inf,
                         FAR struct v4l2_buffer *buf)
{
  if (cfile == NULL || type_inf == NULL || buf == NULL)
    {
      return -EINVAL;
    }

  if (buf->flags & V4L2_BUF_FLAG_LAST)
    {
      type_inf->buflast = true;
    }

  video_framebuff_capture_done(&type_inf->bufinf);
  poll_notify(&cfile->fds, 1,
              V4L2_TYPE_IS_OUTPUT(buf->type) ? POLLOUT : POLLIN);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int codec_register(FAR const char *devpath, FAR struct codec_s *codec)
{
  FAR struct codec_mng_s *cmng;
  int ret;

  if (devpath == NULL || codec == NULL)
    {
      return -EINVAL;
    }

  cmng = kmm_zalloc(sizeof(struct codec_mng_s));
  if (cmng == NULL)
    {
      verr("Failed to allocate codec instance\n");
      return -ENOMEM;
    }

  cmng->v4l2.vops = &g_codec_vops;
  cmng->v4l2.fops = &g_codec_fops;
  cmng->codec     = codec;

  /* Register the character driver */

  ret = video_register(devpath, (FAR struct v4l2_s *)cmng);
  if (ret < 0)
    {
      verr("Failed to register driver: %d\n", ret);
      kmm_free(cmng);
      return ret;
    }

  return OK;
}

int codec_unregister(FAR const char *devpath)
{
  return unregister_driver(devpath);
}

FAR struct v4l2_buffer *codec_output_get_buf(void *cookie)
{
  FAR codec_file_t *cfile = cookie;

  return codec_get_buf(&cfile->output_inf);
}

FAR struct v4l2_buffer *codec_capture_get_buf(void *cookie)
{
  FAR codec_file_t *cfile = cookie;

  return codec_get_buf(&cfile->capture_inf);
}

int codec_output_put_buf(FAR void *cookie, FAR struct v4l2_buffer *buf)
{
  FAR codec_file_t *cfile = cookie;

  return codec_put_buf(cfile, &cfile->output_inf, buf);
}

int codec_capture_put_buf(FAR void *cookie, FAR struct v4l2_buffer *buf)
{
  FAR codec_file_t *cfile = cookie;

  return codec_put_buf(cfile, &cfile->capture_inf, buf);
}

int codec_queue_event(FAR void *cookie, FAR struct v4l2_event *evt)
{
  FAR codec_file_t *cfile = cookie;
  FAR codec_event_t *cevt;
  irqstate_t flags;

  flags = enter_critical_section();

  cevt = (FAR codec_event_t *)sq_remfirst(&cfile->event_free);
  if (cevt == NULL)
    {
      leave_critical_section(flags);
      return -EINVAL;
    }

  memcpy(&cevt->event, evt, sizeof(struct v4l2_event));
  sq_addlast((FAR sq_entry_t *)cevt, &cfile->event_avail);

  poll_notify(&cfile->fds, 1, POLLPRI);
  leave_critical_section(flags);

  return OK;
}

