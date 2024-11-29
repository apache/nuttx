/****************************************************************************
 * arch/sim/src/sim/sim_encoder.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/video/v4l2_m2m.h>
#include <nuttx/wqueue.h>

#include "sim_x264encoder.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_ENCODER_NAME       "sim-x264"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_encoder_s
{
  struct x264_wrapper_s *encoder;
  struct v4l2_format    output_fmt;
  struct v4l2_format    capture_fmt;
  struct work_s         work;
  void                  *cookie;
  int                   bframe;
  int                   fps;
  bool                  capture_on;
  bool                  flushing;
};

typedef struct sim_encoder_s sim_encoder_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sim_encoder_open(void *cookie, void **priv);
static int sim_encoder_close(void *priv);
static int sim_encoder_capture_streamon(void *priv);
static int sim_encoder_output_streamon(void *priv);
static int sim_encoder_capture_streamoff(void *priv);
static int sim_encoder_output_streamoff(void *priv);
static int sim_encoder_capture_available(void *priv);
static int sim_encoder_output_available(void *priv);
static int sim_encoder_querycap(void *priv,
                                struct v4l2_capability *cap);
static int sim_encoder_capture_enum_fmt(void *priv,
                                        struct v4l2_fmtdesc *fmt);
static int sim_encoder_output_enum_fmt(void *priv,
                                       struct v4l2_fmtdesc *fmt);
static int sim_encoder_capture_g_fmt(void *priv,
                                     struct v4l2_format *fmt);
static int sim_encoder_output_g_fmt(void *priv,
                                    struct v4l2_format *fmt);
static int sim_encoder_capture_s_fmt(void *priv,
                                     struct v4l2_format *fmt);
static int sim_encoder_output_s_fmt(void *priv,
                                    struct v4l2_format *fmt);
static int sim_encoder_capture_try_fmt(void *priv,
                                       struct v4l2_format *fmt);
static int sim_encoder_output_try_fmt(void *priv,
                                      struct v4l2_format *fmt);
static size_t sim_encoder_capture_g_bufsize(void *priv);
static size_t sim_encoder_output_g_bufsize(void *priv);
static int sim_encoder_capture_s_parm(void *priv,
                                      struct v4l2_streamparm *parm);
static int sim_encoder_capture_g_ext_ctrls(void *priv,
                                           struct v4l2_ext_controls *ctrls);
static int sim_encoder_capture_s_ext_ctrls(void *priv,
                                           struct v4l2_ext_controls *ctrls);
static int sim_encoder_subscribe_event(void *priv,
                                       struct v4l2_event_subscription *sub);
static void sim_encoder_work(void *cookie);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct codec_ops_s g_sim_encoder_ops =
{
  .open              = sim_encoder_open,
  .close             = sim_encoder_close,
  .capture_streamon  = sim_encoder_capture_streamon,
  .output_streamon   = sim_encoder_output_streamon,
  .capture_streamoff = sim_encoder_capture_streamoff,
  .output_streamoff  = sim_encoder_output_streamoff,
  .capture_available = sim_encoder_capture_available,
  .output_available  = sim_encoder_output_available,
  .querycap          = sim_encoder_querycap,
  .capture_enum_fmt  = sim_encoder_capture_enum_fmt,
  .output_enum_fmt   = sim_encoder_output_enum_fmt,
  .capture_g_fmt     = sim_encoder_capture_g_fmt,
  .output_g_fmt      = sim_encoder_output_g_fmt,
  .capture_s_fmt     = sim_encoder_capture_s_fmt,
  .output_s_fmt      = sim_encoder_output_s_fmt,
  .capture_try_fmt   = sim_encoder_capture_try_fmt,
  .output_try_fmt    = sim_encoder_output_try_fmt,
  .capture_g_bufsize = sim_encoder_capture_g_bufsize,
  .output_g_bufsize  = sim_encoder_output_g_bufsize,
  .capture_s_parm    = sim_encoder_capture_s_parm,
  .g_ext_ctrls       = sim_encoder_capture_g_ext_ctrls,
  .s_ext_ctrls       = sim_encoder_capture_s_ext_ctrls,
  .subscribe_event   = sim_encoder_subscribe_event,
};

static struct codec_s g_sim_codec_encoder =
{
  .ops = &g_sim_encoder_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sim_encoder_open(void *cookie, void **priv)
{
  sim_encoder_t *sim_encoder;
  struct x264_wrapper_s *encoder;

  sim_encoder = kmm_zalloc(sizeof(struct sim_encoder_s));
  if (sim_encoder == NULL)
    {
      return -ENOMEM;
    }

  encoder = x264_wrapper_open();
  if (encoder == NULL)
    {
      kmm_free(sim_encoder);
      return -ENOMEM;
    }

  sim_encoder->encoder = encoder;
  sim_encoder->cookie  = cookie;
  *priv                = sim_encoder;

  return 0;
}

static int sim_encoder_close(void *priv)
{
  sim_encoder_t *sim_encoder = priv;

  x264_wrapper_close(sim_encoder->encoder);
  kmm_free(sim_encoder);

  return 0;
}

static int sim_encoder_capture_streamon(void *priv)
{
  sim_encoder_t *sim_encoder = priv;

  sim_encoder->capture_on = true;
  work_queue(HPWORK, &sim_encoder->work,
             sim_encoder_work, sim_encoder, 0);

  return 0;
}

static int sim_encoder_output_streamon(void *priv)
{
  sim_encoder_t *sim_encoder = priv;

  return x264_wrapper_streamon(sim_encoder->encoder,
                               sim_encoder->output_fmt.fmt.pix.width,
                               sim_encoder->output_fmt.fmt.pix.height,
                               sim_encoder->fps ? sim_encoder->fps : 30,
                               sim_encoder->bframe);
}

static int sim_encoder_capture_available(void *priv)
{
  sim_encoder_t *sim_encoder = priv;

  if (sim_encoder->capture_on == false)
    {
      return 0;
    }

  work_queue(HPWORK, &sim_encoder->work,
             sim_encoder_work, sim_encoder, 0);
  return 0;
}

static int sim_encoder_output_available(void *priv)
{
  sim_encoder_t *sim_encoder = priv;

  if (sim_encoder->capture_on == false)
    {
      return 0;
    }

  work_queue(HPWORK, &sim_encoder->work,
             sim_encoder_work, sim_encoder, 0);
  return 0;
}

static int sim_encoder_capture_streamoff(void *priv)
{
  sim_encoder_t *sim_encoder = priv;

  sim_encoder->capture_on = false;
  return x264_wrapper_streamoff(sim_encoder->encoder);
}

static int sim_encoder_output_streamoff(void *priv)
{
  sim_encoder_t *sim_encoder = priv;

  if (!sim_encoder->capture_on)
    {
      return 0;
    }

  sim_encoder->flushing = true;
  work_queue(HPWORK, &sim_encoder->work,
             sim_encoder_work, sim_encoder, 0);

  return 0;
}

static int sim_encoder_querycap(void *priv,
                                struct v4l2_capability *cap)
{
  strncpy((char *)cap->driver, SIM_ENCODER_NAME, sizeof(cap->driver));
  strncpy((char *)cap->card, SIM_ENCODER_NAME, sizeof(cap->card));
  cap->capabilities = V4L2_CAP_VIDEO_M2M;

  return 0;
}

static int sim_encoder_capture_enum_fmt(void *priv,
                                        struct v4l2_fmtdesc *fmt)
{
  if (fmt->index >= 1)
    {
      return -EINVAL;
    }

  fmt->pixelformat = V4L2_PIX_FMT_H264;
  return 0;
}

static int sim_encoder_output_enum_fmt(void *priv,
                                       struct v4l2_fmtdesc *fmt)
{
  if (fmt->index >= 1)
    {
      return -EINVAL;
    }

  fmt->pixelformat = V4L2_PIX_FMT_YUV420;
  return 0;
}

static int sim_encoder_capture_g_fmt(void *priv,
                                     struct v4l2_format *fmt)
{
  fmt->fmt.pix.field       = V4L2_FIELD_NONE;
  fmt->fmt.pix.pixelformat = V4L2_PIX_FMT_H264;

  return 0;
}

static int sim_encoder_output_g_fmt(void *priv,
                                    struct v4l2_format *fmt)
{
  fmt->fmt.pix.field       = V4L2_FIELD_NONE;
  fmt->fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;

  return 0;
}

static int sim_encoder_capture_try_fmt(void *priv,
                                       struct v4l2_format *fmt)
{
  if (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_H264)
    {
      return 0;
    }

  return -EINVAL;
}

static int sim_encoder_output_try_fmt(void *priv,
                                      struct v4l2_format *fmt)
{
  if (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
    {
      return 0;
    }

  return -EINVAL;
}

static int sim_encoder_capture_s_fmt(void *priv,
                                     struct v4l2_format *fmt)
{
  sim_encoder_t *sim_encoder = priv;
  size_t sizeimage;

  if (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_H264)
    {
      sim_encoder->capture_fmt = *fmt;

      sizeimage = fmt->fmt.pix.width * fmt->fmt.pix.height;
      sizeimage = (sizeimage * 3 / 2) / 2 + 128;
      sim_encoder->capture_fmt.fmt.pix.sizeimage = sizeimage;

      return 0;
    }

  return -EINVAL;
}

static int sim_encoder_output_s_fmt(void *priv,
                                    struct v4l2_format *fmt)
{
  sim_encoder_t *sim_encoder = priv;
  size_t sizeimage;

  if (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
    {
      sim_encoder->output_fmt = *fmt;
      sizeimage = fmt->fmt.pix.width * fmt->fmt.pix.height * 3 / 2;
      sim_encoder->output_fmt.fmt.pix.sizeimage = sizeimage;

      return 0;
    }

  return -EINVAL;
}

static size_t sim_encoder_capture_g_bufsize(void *priv)
{
  sim_encoder_t *sim_encoder = priv;

  if (sim_encoder->capture_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_H264)
    {
      return sim_encoder->capture_fmt.fmt.pix.sizeimage;
    }

  return 0;
}

static size_t sim_encoder_output_g_bufsize(void *priv)
{
  sim_encoder_t *sim_encoder = priv;

  if (sim_encoder->output_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
    {
      return sim_encoder->output_fmt.fmt.pix.sizeimage;
    }

  return 0;
}

static int sim_encoder_capture_s_parm(void *priv,
                                      struct v4l2_streamparm *parm)
{
  sim_encoder_t *sim_encoder = priv;
  struct v4l2_fract fract;

  fract = parm->parm.capture.timeperframe;
  if (fract.numerator > 0 && fract.denominator > 0)
    {
      sim_encoder->fps = fract.denominator / fract.numerator;
    }

  return 0;
}

static int sim_encoder_capture_s_ext_ctrls(void *priv,
                                           struct v4l2_ext_controls *ctrls)
{
  sim_encoder_t *sim_encoder = priv;
  struct v4l2_ext_control *ctrl;

  if (ctrls->count != 1)
    {
      return -EINVAL;
    }

  ctrl = ctrls->controls;
  switch (ctrl->id)
    {
      case V4L2_CID_MPEG_VIDEO_B_FRAMES:
        sim_encoder->bframe = ctrl->value;
        return 0;

      default:
        return -EINVAL;
    }
}

static int sim_encoder_capture_g_ext_ctrls(void *priv,
                                           struct v4l2_ext_controls *ctrls)
{
  sim_encoder_t *sim_encoder = priv;
  struct v4l2_ext_control *ctrl;

  if (ctrls->count != 1)
    {
      return -EINVAL;
    }

  ctrl = ctrls->controls;
  switch (ctrl->id)
    {
      case V4L2_CID_MPEG_VIDEO_B_FRAMES:
        ctrl->value = sim_encoder->bframe;
        return 0;

      default:
        return -EINVAL;
    }
}

static int sim_encoder_subscribe_event(void *priv,
                                       struct v4l2_event_subscription *sub)
{
  switch (sub->type)
    {
      case V4L2_EVENT_EOS:
        return OK;

      default:
        return -EINVAL;
    }
}

static int sim_encoder_process(sim_encoder_t *sim_encoder,
                               struct v4l2_buffer *dst_buf,
                               struct v4l2_buffer *src_buf)
{
  struct v4l2_event event;
  uint8_t *src_data = NULL;
  uint32_t src_size = 0;
  int64_t src_pts = 0;
  int64_t dst_pts = 0;
  int ret;

  if (src_buf != NULL)
    {
      src_data = (uint8_t *)src_buf->m.userptr;
      src_size = src_buf->bytesused;
      src_pts  = src_buf->timestamp.tv_sec * 1000000 +
                 src_buf->timestamp.tv_usec;
    }

  ret = x264_wrapper_enqueue(sim_encoder->encoder,
                             src_data, src_size, src_pts);
  if (ret >= 0 && src_buf != NULL)
    {
      codec_output_put_buf(sim_encoder->cookie, src_buf);
    }

  if (ret < 1)
    {
      return ret;
    }

  ret = x264_wrapper_dequeue(sim_encoder->encoder,
                             (uint8_t *)dst_buf->m.userptr,
                             &dst_buf->bytesused,
                             &dst_pts,
                             &dst_buf->flags);
  if (ret == 0 && src_buf == NULL)
    {
      sim_encoder->flushing = false;
      dst_buf->flags |= V4L2_BUF_FLAG_LAST;

      memset(&event, 0, sizeof(event));
      event.type = V4L2_EVENT_EOS;
      codec_queue_event(sim_encoder->cookie, &event);
    }

  dst_buf->timestamp.tv_usec = dst_pts % 1000000;
  dst_buf->timestamp.tv_sec  = dst_pts / 1000000;

  codec_capture_put_buf(sim_encoder->cookie, dst_buf);
  return ret;
}

static void sim_encoder_work(void *encoder)
{
  sim_encoder_t *sim_encoder = encoder;
  struct v4l2_buffer *src_buf;
  struct v4l2_buffer *dst_buf;
  int ret;

  src_buf = codec_output_get_buf(sim_encoder->cookie);
  if (src_buf == NULL && !sim_encoder->flushing)
    {
      return;
    }

  dst_buf = codec_capture_get_buf(sim_encoder->cookie);
  if (dst_buf == NULL)
    {
      return;
    }

  ret = sim_encoder_process(encoder, dst_buf, src_buf);
  if (ret > 0)
    {
      work_queue(HPWORK, &sim_encoder->work,
                 sim_encoder_work, sim_encoder, 0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_encoder_initialize()
{
  return codec_register(CONFIG_SIM_VIDEO_ENCODER_DEV_PATH,
                        &g_sim_codec_encoder);
}
