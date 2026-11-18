/****************************************************************************
 * arch/sim/src/sim/sim_jpeg_decoder.c
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
#include <sys/param.h>
#include <sys/time.h>

#include <nuttx/kmalloc.h>
#include <nuttx/video/v4l2_m2m.h>
#include <nuttx/wqueue.h>

#include "sim_jpegdec.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_JPEG_DEC_NAME "sim-jpeg"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_jpeg_decoder_s
{
  void                 *dec;
  struct v4l2_format    output_fmt;
  struct v4l2_format    capture_fmt;
  struct work_s         work;
  void                 *cookie;
  bool                  capture_on;
};

typedef struct sim_jpeg_decoder_s sim_jpeg_decoder_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sim_jpeg_open(void *cookie, void **priv);
static int sim_jpeg_close(void *priv);
static int sim_jpeg_capture_streamon(void *priv);
static int sim_jpeg_output_streamon(void *priv);
static int sim_jpeg_capture_streamoff(void *priv);
static int sim_jpeg_output_streamoff(void *priv);
static int sim_jpeg_output_available(void *priv);
static int sim_jpeg_capture_available(void *priv);
static int sim_jpeg_querycap(void *priv, struct v4l2_capability *cap);
static int sim_jpeg_capture_enum_fmt(void *priv, struct v4l2_fmtdesc *f);
static int sim_jpeg_output_enum_fmt(void *priv, struct v4l2_fmtdesc *f);
static int sim_jpeg_capture_g_fmt(void *priv, struct v4l2_format *fmt);
static int sim_jpeg_output_g_fmt(void *priv, struct v4l2_format *fmt);
static int sim_jpeg_capture_s_fmt(void *priv, struct v4l2_format *fmt);
static int sim_jpeg_output_s_fmt(void *priv, struct v4l2_format *fmt);
static int sim_jpeg_capture_try_fmt(void *priv, struct v4l2_format *fmt);
static int sim_jpeg_output_try_fmt(void *priv, struct v4l2_format *fmt);
static int sim_jpeg_subscribe_event(void *priv,
                                    struct v4l2_event_subscription *sub);
static uint32_t sim_jpeg_capture_g_bufsize(void *priv);
static uint32_t sim_jpeg_output_g_bufsize(void *priv);
static void sim_jpeg_work(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct codec_ops_s g_sim_jpeg_dec_ops =
{
  .open              = sim_jpeg_open,
  .close             = sim_jpeg_close,
  .capture_streamon  = sim_jpeg_capture_streamon,
  .output_streamon   = sim_jpeg_output_streamon,
  .capture_streamoff = sim_jpeg_capture_streamoff,
  .output_streamoff  = sim_jpeg_output_streamoff,
  .output_available  = sim_jpeg_output_available,
  .capture_available = sim_jpeg_capture_available,
  .querycap          = sim_jpeg_querycap,
  .capture_enum_fmt  = sim_jpeg_capture_enum_fmt,
  .output_enum_fmt   = sim_jpeg_output_enum_fmt,
  .capture_g_fmt     = sim_jpeg_capture_g_fmt,
  .output_g_fmt      = sim_jpeg_output_g_fmt,
  .capture_s_fmt     = sim_jpeg_capture_s_fmt,
  .output_s_fmt      = sim_jpeg_output_s_fmt,
  .capture_try_fmt   = sim_jpeg_capture_try_fmt,
  .output_try_fmt    = sim_jpeg_output_try_fmt,
  .subscribe_event   = sim_jpeg_subscribe_event,
  .capture_g_bufsize = sim_jpeg_capture_g_bufsize,
  .output_g_bufsize  = sim_jpeg_output_g_bufsize,
};

static struct codec_s g_sim_jpeg_codec =
{
  .ops = &g_sim_jpeg_dec_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sim_jpeg_open(void *cookie, void **priv)
{
  sim_jpeg_decoder_t *ctx;

  ctx = kmm_zalloc(sizeof(sim_jpeg_decoder_t));
  if (ctx == NULL)
    {
      return -ENOMEM;
    }

  ctx->dec = sim_jpegdec_open();
  if (ctx->dec == NULL)
    {
      kmm_free(ctx);
      return -ENOMEM;
    }

  ctx->cookie = cookie;
  *priv = ctx;
  return 0;
}

static int sim_jpeg_close(void *priv)
{
  sim_jpeg_decoder_t *ctx = priv;

  work_cancel_sync(HPWORK, &ctx->work);
  sim_jpegdec_close(ctx->dec);
  kmm_free(ctx);
  return 0;
}

static int sim_jpeg_capture_streamon(void *priv)
{
  sim_jpeg_decoder_t *ctx = priv;

  ctx->capture_on = true;
  work_queue(HPWORK, &ctx->work, sim_jpeg_work, ctx, 0);
  return 0;
}

static int sim_jpeg_output_streamon(void *priv)
{
  return 0;
}

static int sim_jpeg_capture_streamoff(void *priv)
{
  sim_jpeg_decoder_t *ctx = priv;

  ctx->capture_on = false;
  return 0;
}

static int sim_jpeg_output_streamoff(void *priv)
{
  return 0;
}

static int sim_jpeg_output_available(void *priv)
{
  sim_jpeg_decoder_t *ctx = priv;

  if (ctx->capture_on)
    {
      work_queue(HPWORK, &ctx->work, sim_jpeg_work, ctx, 0);
    }

  return 0;
}

static int sim_jpeg_capture_available(void *priv)
{
  sim_jpeg_decoder_t *ctx = priv;

  if (ctx->capture_on)
    {
      work_queue(HPWORK, &ctx->work, sim_jpeg_work, ctx, 0);
    }

  return 0;
}

static int sim_jpeg_querycap(void *priv, struct v4l2_capability *cap)
{
  strlcpy((char *)cap->driver, SIM_JPEG_DEC_NAME, sizeof(cap->driver));
  strlcpy((char *)cap->card, SIM_JPEG_DEC_NAME, sizeof(cap->card));
  cap->capabilities = V4L2_CAP_VIDEO_M2M;
  return 0;
}

static int sim_jpeg_capture_enum_fmt(void *priv, struct v4l2_fmtdesc *f)
{
  static const uint32_t fmts[] =
  {
    V4L2_PIX_FMT_YUV420,
    V4L2_PIX_FMT_NV12,
    V4L2_PIX_FMT_RGB24,
    V4L2_PIX_FMT_BGR24,
    V4L2_PIX_FMT_XRGB32,
    V4L2_PIX_FMT_XBGR32,
  };

  if (f->index >= nitems(fmts))
    {
      return -EINVAL;
    }

  f->pixelformat = fmts[f->index];
  return 0;
}

static int sim_jpeg_output_enum_fmt(void *priv, struct v4l2_fmtdesc *f)
{
  if (f->index == 0)
    {
      f->pixelformat = V4L2_PIX_FMT_JPEG;
    }
  else if (f->index == 1)
    {
      f->pixelformat = V4L2_PIX_FMT_MJPEG;
    }
  else
    {
      return -EINVAL;
    }

  return 0;
}

static int sim_jpeg_capture_g_fmt(void *priv, struct v4l2_format *fmt)
{
  sim_jpeg_decoder_t *ctx = priv;

  *fmt = ctx->capture_fmt;
  return 0;
}

static int sim_jpeg_output_g_fmt(void *priv, struct v4l2_format *fmt)
{
  sim_jpeg_decoder_t *ctx = priv;

  *fmt = ctx->output_fmt;
  return 0;
}

static int sim_jpeg_capture_s_fmt(void *priv, struct v4l2_format *fmt)
{
  sim_jpeg_decoder_t *ctx = priv;
  uint32_t w = fmt->fmt.pix.width;
  uint32_t h = fmt->fmt.pix.height;

  switch (fmt->fmt.pix.pixelformat)
    {
      case V4L2_PIX_FMT_YUV420:
      case V4L2_PIX_FMT_NV12:
        fmt->fmt.pix.sizeimage = w * h * 3 / 2;
        fmt->fmt.pix.bytesperline = w;
        break;

      case V4L2_PIX_FMT_RGB24:
      case V4L2_PIX_FMT_BGR24:
        fmt->fmt.pix.sizeimage = w * h * 3;
        fmt->fmt.pix.bytesperline = w * 3;
        break;

      case V4L2_PIX_FMT_XRGB32:
      case V4L2_PIX_FMT_XBGR32:
        fmt->fmt.pix.sizeimage = w * h * 4;
        fmt->fmt.pix.bytesperline = w * 4;
        break;

      default:
        return -EINVAL;
    }

  ctx->capture_fmt = *fmt;
  return 0;
}

static int sim_jpeg_output_s_fmt(void *priv, struct v4l2_format *fmt)
{
  sim_jpeg_decoder_t *ctx = priv;

  if (fmt->fmt.pix.pixelformat != V4L2_PIX_FMT_JPEG &&
      fmt->fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
    {
      return -EINVAL;
    }

  ctx->output_fmt = *fmt;

  /* JPEG compressed size upper bound: use 2x pixel count to avoid
   * truncating high-quality JPEG bitstreams.
   */

  ctx->output_fmt.fmt.pix.sizeimage =
    fmt->fmt.pix.width * fmt->fmt.pix.height * 2;

  /* Write back sizeimage so userspace knows the buffer size */

  fmt->fmt.pix.sizeimage = ctx->output_fmt.fmt.pix.sizeimage;
  return 0;
}

static int sim_jpeg_capture_try_fmt(void *priv, struct v4l2_format *fmt)
{
  uint32_t w = fmt->fmt.pix.width;
  uint32_t h = fmt->fmt.pix.height;

  switch (fmt->fmt.pix.pixelformat)
    {
      case V4L2_PIX_FMT_YUV420:
      case V4L2_PIX_FMT_NV12:
        fmt->fmt.pix.sizeimage = w * h * 3 / 2;
        fmt->fmt.pix.bytesperline = w;
        break;

      case V4L2_PIX_FMT_RGB24:
      case V4L2_PIX_FMT_BGR24:
        fmt->fmt.pix.sizeimage = w * h * 3;
        fmt->fmt.pix.bytesperline = w * 3;
        break;

      case V4L2_PIX_FMT_XRGB32:
      case V4L2_PIX_FMT_XBGR32:
        fmt->fmt.pix.sizeimage = w * h * 4;
        fmt->fmt.pix.bytesperline = w * 4;
        break;

      default:
        return -EINVAL;
    }

  return 0;
}

static int sim_jpeg_output_try_fmt(void *priv, struct v4l2_format *fmt)
{
  if (fmt->fmt.pix.pixelformat != V4L2_PIX_FMT_JPEG &&
      fmt->fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
    {
      return -EINVAL;
    }

  fmt->fmt.pix.sizeimage =
    fmt->fmt.pix.width * fmt->fmt.pix.height * 2;
  return 0;
}

static int sim_jpeg_subscribe_event(void *priv,
                                    struct v4l2_event_subscription *sub)
{
  return sub->type == V4L2_EVENT_EOS ? 0 : -EINVAL;
}

static uint32_t sim_jpeg_capture_g_bufsize(void *priv)
{
  sim_jpeg_decoder_t *ctx = priv;

  return ctx->capture_fmt.fmt.pix.sizeimage;
}

static uint32_t sim_jpeg_output_g_bufsize(void *priv)
{
  sim_jpeg_decoder_t *ctx = priv;

  return ctx->output_fmt.fmt.pix.sizeimage;
}

static void sim_jpeg_work(void *arg)
{
  sim_jpeg_decoder_t *ctx = arg;
  struct v4l2_buffer *src_buf;
  struct v4l2_buffer *dst_buf;
  struct timeval ts;
  uint32_t dst_size;
  uint32_t w;
  uint32_t h;
  int ret;

  src_buf = codec_output_get_buf(ctx->cookie);
  if (src_buf == NULL)
    {
      return;
    }

  dst_buf = codec_capture_get_buf(ctx->cookie);
  if (dst_buf == NULL)
    {
      return;
    }

  /* Save timestamp before returning source buffer */

  ts = src_buf->timestamp;

  /* Decode JPEG to requested capture format */

  ret = sim_jpegdec_decode(ctx->dec,
                           src_buf->m.vaddr,
                           src_buf->bytesused,
                           dst_buf->m.vaddr,
                           &dst_size, &w, &h,
                           ctx->capture_fmt.fmt.pix.pixelformat);

  /* Return source buffer */

  codec_output_put_buf(ctx->cookie, src_buf);

  if (ret == 0)
    {
      dst_buf->bytesused = dst_size;
      dst_buf->timestamp = ts;
      ctx->capture_fmt.fmt.pix.width = w;
      ctx->capture_fmt.fmt.pix.height = h;
    }
  else
    {
      dst_buf->bytesused = 0;
    }

  codec_capture_put_buf(ctx->cookie, dst_buf);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_jpeg_decoder_initialize(void)
{
  return codec_register(CONFIG_SIM_JPEG_DECODER_DEV_PATH,
                        &g_sim_jpeg_codec);
}
