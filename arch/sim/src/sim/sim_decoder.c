/****************************************************************************
 * arch/sim/src/sim/sim_decoder.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/video/v4l2_m2m.h>
#include <nuttx/wqueue.h>

#include "sim_openh264dec.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_DECODER_NAME "sim-h264"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_decoder_s
{
  struct openh264_decoder_s *decoder;
  struct v4l2_format        output_fmt;
  struct v4l2_format        capture_fmt;
  struct work_s             work;
  void                      *cookie;
  bool                      capture_on;
  bool                      flushing;
};

typedef struct sim_decoder_s sim_decoder_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sim_decoder_open(void *cookie, void **priv);
static int sim_decoder_close(void *priv);
static int sim_decoder_capture_streamon(void *priv);
static int sim_decoder_output_streamon(void *priv);
static int sim_decoder_capture_streamoff(void *priv);
static int sim_decoder_output_streamoff(void *priv);
static int sim_decoder_output_available(void *priv);
static int sim_decoder_capture_available(void *priv);
static int sim_decoder_querycap(void *priv,
                                struct v4l2_capability *cap);
static int sim_decoder_capture_enum_fmt(void *priv,
                                        struct v4l2_fmtdesc *fmt);
static int sim_decoder_output_enum_fmt(void *priv,
                                       struct v4l2_fmtdesc *fmt);
static int sim_decoder_capture_g_fmt(void *priv,
                                     struct v4l2_format *fmt);
static int sim_decoder_output_g_fmt(void *priv,
                                    struct v4l2_format *fmt);
static int sim_decoder_capture_s_fmt(void *priv,
                                     struct v4l2_format *fmt);
static int sim_decoder_output_s_fmt(void *priv,
                                    struct v4l2_format *fmt);
static int sim_decoder_capture_try_fmt(void *priv,
                                       struct v4l2_format *fmt);
static int sim_decoder_output_try_fmt(void *priv,
                                      struct v4l2_format *fmt);
static int sim_decoder_subscribe_event(void *priv,
                                       struct v4l2_event_subscription *sub);
static uint32_t sim_decoder_capture_g_bufsize(void *priv);
static uint32_t sim_decoder_output_g_bufsize(void *priv);
static int sim_decoder_process(sim_decoder_t *sim_decoder,
                               struct v4l2_buffer *dst_buf,
                               struct v4l2_buffer *src_buf);
static void sim_decoder_work(void *cookie);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct codec_ops_s g_sim_decoder_ops =
{
  .open              = sim_decoder_open,
  .close             = sim_decoder_close,
  .capture_streamon  = sim_decoder_capture_streamon,
  .output_streamon   = sim_decoder_output_streamon,
  .capture_streamoff = sim_decoder_capture_streamoff,
  .output_streamoff  = sim_decoder_output_streamoff,
  .output_available  = sim_decoder_output_available,
  .capture_available = sim_decoder_capture_available,
  .querycap          = sim_decoder_querycap,
  .capture_enum_fmt  = sim_decoder_capture_enum_fmt,
  .output_enum_fmt   = sim_decoder_output_enum_fmt,
  .capture_g_fmt     = sim_decoder_capture_g_fmt,
  .output_g_fmt      = sim_decoder_output_g_fmt,
  .capture_s_fmt     = sim_decoder_capture_s_fmt,
  .output_s_fmt      = sim_decoder_output_s_fmt,
  .capture_try_fmt   = sim_decoder_capture_try_fmt,
  .output_try_fmt    = sim_decoder_output_try_fmt,
  .subscribe_event   = sim_decoder_subscribe_event,
  .capture_g_bufsize = sim_decoder_capture_g_bufsize,
  .output_g_bufsize  = sim_decoder_output_g_bufsize,
};

static struct codec_s g_sim_codec_decoder =
{
  .ops = &g_sim_decoder_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sim_decoder_open(void *cookie, void **priv)
{
  sim_decoder_t *sim_decoder;
  struct openh264_decoder_s *decoder;

  sim_decoder = kmm_zalloc(sizeof(sim_decoder_t));
  if (sim_decoder == NULL)
    {
      return -ENOMEM;
    }

  decoder = openh264_decoder_open();
  if (decoder == NULL)
    {
      kmm_free(sim_decoder);
      return -ENOMEM;
    }

  sim_decoder->decoder = decoder;
  sim_decoder->cookie  = cookie;
  *priv                = sim_decoder;

  return 0;
}

static int sim_decoder_close(void *priv)
{
  sim_decoder_t *sim_decoder = priv;

  openh264_decoder_close(sim_decoder->decoder);
  kmm_free(sim_decoder);

  return 0;
}

static int sim_decoder_capture_streamon(void *priv)
{
  sim_decoder_t *sim_decoder = priv;

  sim_decoder->capture_on = true;
  work_queue(HPWORK, &sim_decoder->work,
             sim_decoder_work, sim_decoder, 0);

  return 0;
}

static int sim_decoder_output_streamon(void *priv)
{
  sim_decoder_t *sim_decoder = priv;

  return openh264_decoder_streamon(sim_decoder->decoder);
}

static int sim_decoder_capture_streamoff(void *priv)
{
  sim_decoder_t *sim_decoder = priv;

  sim_decoder->capture_on = false;
  return openh264_decoder_streamoff(sim_decoder->decoder);
}

static int sim_decoder_output_streamoff(void *priv)
{
  sim_decoder_t *sim_decoder = priv;

  if (!sim_decoder->capture_on)
    {
      return 0;
    }

  sim_decoder->flushing = true;
  work_queue(HPWORK, &sim_decoder->work,
             sim_decoder_work, sim_decoder, 0);

  return 0;
}

static int sim_decoder_output_available(void *priv)
{
  sim_decoder_t *sim_decoder = priv;

  if (sim_decoder->capture_on == false)
    {
      return 0;
    }

  work_queue(HPWORK, &sim_decoder->work,
             sim_decoder_work, sim_decoder, 0);

  return 0;
}

static int sim_decoder_capture_available(void *priv)
{
  sim_decoder_t *sim_decoder = priv;

  if (sim_decoder->capture_on == false)
    {
      return 0;
    }

  work_queue(HPWORK, &sim_decoder->work,
             sim_decoder_work, sim_decoder, 0);

  return 0;
}

static int sim_decoder_querycap(void *priv,
                                struct v4l2_capability *cap)
{
  strlcpy((char *)cap->driver, SIM_DECODER_NAME, sizeof(cap->driver));
  strlcpy((char *)cap->card, SIM_DECODER_NAME, sizeof(cap->card));
  cap->capabilities = V4L2_CAP_VIDEO_M2M;

  return 0;
}

static int sim_decoder_capture_enum_fmt(void *priv,
                                        struct v4l2_fmtdesc *f)
{
  if (f->index >= 1)
    {
      return -EINVAL;
    }

  f->pixelformat = V4L2_PIX_FMT_YUV420;
  return 0;
}

static int sim_decoder_output_enum_fmt(void *priv,
                                       struct v4l2_fmtdesc *fmt)
{
  if (fmt->index >= 1)
    {
      return -EINVAL;
    }

  fmt->pixelformat = V4L2_PIX_FMT_H264;
  return 0;
}

static int sim_decoder_capture_g_fmt(void *priv,
                                     struct v4l2_format *fmt)
{
  sim_decoder_t *sim_decoder = priv;

  *fmt = sim_decoder->capture_fmt;
  return 0;
}

static int sim_decoder_output_g_fmt(void *priv,
                                    struct v4l2_format *fmt)
{
  sim_decoder_t *sim_decoder = priv;

  *fmt = sim_decoder->output_fmt;
  return 0;
}

static int sim_decoder_capture_s_fmt(void *priv,
                                     struct v4l2_format *fmt)
{
  sim_decoder_t *sim_decoder = priv;
  size_t sizeimage;

  if (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
    {
      sim_decoder->capture_fmt = *fmt;

      sizeimage = fmt->fmt.pix.width * fmt->fmt.pix.height * 3 / 2;
      sim_decoder->capture_fmt.fmt.pix.sizeimage = sizeimage;
      sim_decoder->capture_fmt.fmt.pix.bytesperline = fmt->fmt.pix.width;

      return 0;
    }

  return -EINVAL;
}

static int sim_decoder_output_s_fmt(void *priv,
                                    struct v4l2_format *fmt)
{
  sim_decoder_t *sim_decoder = priv;
  size_t sizeimage;

  if (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_H264)
    {
      sim_decoder->output_fmt = *fmt;

      sizeimage = fmt->fmt.pix.width * fmt->fmt.pix.height;
      sizeimage = (sizeimage * 3 / 2) / 2 + 128;
      sim_decoder->output_fmt.fmt.pix.sizeimage = sizeimage;

      return 0;
    }

  return -EINVAL;
}

static int sim_decoder_capture_try_fmt(void *priv,
                                       struct v4l2_format *fmt)
{
  if (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
    {
      return 0;
    }

  return -EINVAL;
}

static int sim_decoder_output_try_fmt(void *priv,
                                      struct v4l2_format *fmt)
{
  if (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_H264)
    {
      return 0;
    }

  return -EINVAL;
}

static int sim_decoder_subscribe_event(void *priv,
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

static uint32_t sim_decoder_capture_g_bufsize(void *priv)
{
  sim_decoder_t *sim_decoder = priv;

  if (sim_decoder->capture_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
    {
      return sim_decoder->capture_fmt.fmt.pix.sizeimage;
    }

  return 0;
}

static uint32_t sim_decoder_output_g_bufsize(void *priv)
{
  sim_decoder_t *sim_decoder = priv;

  if (sim_decoder->output_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_H264)
    {
      return sim_decoder->output_fmt.fmt.pix.sizeimage;
    }

  return 0;
}

static int sim_decoder_process(sim_decoder_t *sim_decoder,
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

  ret = openh264_decoder_enqueue(sim_decoder->decoder,
                                 src_data, src_pts, src_size);
  if (ret >= 0 && src_buf != NULL)
    {
      codec_output_put_buf(sim_decoder->cookie, src_buf);
    }

  if (ret < 1)
    {
      return ret;
    }

  ret = openh264_decoder_dequeue(sim_decoder->decoder,
                                 (uint8_t *)dst_buf->m.userptr,
                                 &dst_pts,
                                 &dst_buf->bytesused);
  if (ret == 0 && src_buf == NULL)
    {
      sim_decoder->flushing = false;
      dst_buf->flags |= V4L2_BUF_FLAG_LAST;

      memset(&event, 0, sizeof(event));
      event.type = V4L2_EVENT_EOS;
      codec_queue_event(sim_decoder->cookie, &event);
    }

  dst_buf->timestamp.tv_usec = dst_pts % 1000000;
  dst_buf->timestamp.tv_sec  = dst_pts / 1000000;

  codec_capture_put_buf(sim_decoder->cookie, dst_buf);
  return ret;
}

static void sim_decoder_work(void *decoder)
{
  sim_decoder_t *sim_decoder = decoder;
  struct v4l2_buffer *src_buf;
  struct v4l2_buffer *dst_buf;
  int ret;

  src_buf = codec_output_get_buf(sim_decoder->cookie);
  if (src_buf == NULL && !sim_decoder->flushing)
    {
      return;
    }

  dst_buf = codec_capture_get_buf(sim_decoder->cookie);
  if (dst_buf == NULL)
    {
      return;
    }

  ret = sim_decoder_process(decoder, dst_buf, src_buf);
  if (ret > 0)
    {
      work_queue(HPWORK, &sim_decoder->work,
                 sim_decoder_work, sim_decoder, 0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_decoder_initialize()
{
  return codec_register(CONFIG_SIM_VIDEO_DECODER_DEV_PATH,
                        &g_sim_codec_decoder);
}
