/****************************************************************************
 * arch/sim/src/sim/up_video.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arch/board/board.h>
#include <nuttx/config.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include "up_video_host.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HOST_VIDEO_DEV_PATH  CONFIG_HOST_VIDEO_DEV_PATH
#define HOST_VIDEO_BUF_NUM   30
#define ENQUEUE(q, addr, size) (q).addr[q.num] = addr; \
                               (q).size[q.num++] = size;

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  size_t num;
  uint8_t *addr[HOST_VIDEO_BUF_NUM];
  size_t size[HOST_VIDEO_BUF_NUM];
} buf_queue_t;

typedef struct
{
  imgdata_capture_t capture_cb;
  uint32_t capture_size;
  buf_queue_t buf_q;
} video_priv_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* video image sensor operations */

static bool sim_camera_is_available(void);
static int sim_camera_init(void);
static int sim_camera_uninit(void);
static FAR const char *sim_camera_get_driver_name(void);
static int sim_camera_validate_frame_setting(imgsensor_stream_type_t type,
                                         uint8_t nr_datafmt,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval);
static int sim_camera_start_capture(imgsensor_stream_type_t type,
                                uint8_t nr_datafmt,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval);
static int sim_camera_stop_capture(imgsensor_stream_type_t type);
static int sim_camera_get_supported_value(uint32_t id,
                                     FAR imgsensor_supported_value_t *value);
static int sim_camera_get_value(uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value);
static int sim_camera_set_value(uint32_t id, uint32_t size,
                            imgsensor_value_t value);

/* video image data operations */

static int sim_camera_data_init(void);
static int sim_camera_data_uninit(void);
static int sim_camera_data_validate_frame_setting
             (uint8_t nr_datafmt,
              imgdata_format_t *datafmt,
              imgdata_interval_t *interval);
static int sim_camera_data_start_capture
             (uint8_t nr_datafmt,
              imgdata_format_t *datafmt,
              imgdata_interval_t *interval,
              imgdata_capture_t callback);
static int sim_camera_data_stop_capture(void);
static int sim_camera_data_validate_buf(uint8_t *addr, uint32_t size);
static int sim_camera_data_set_buf(uint8_t *addr, uint32_t size);
static int sim_camera_data_enq_buf(uint8_t *addr, uint32_t size);
static int sim_camera_data_dq_buf(uint8_t **addr, struct timeval *ts);

static uint32_t imgsensor_fmt_to_v4l2(uint32_t pixelformat);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imgsensor_ops_s g_sim_camera_ops =
{
  .is_available             = sim_camera_is_available,
  .init                     = sim_camera_init,
  .uninit                   = sim_camera_uninit,
  .get_driver_name          = sim_camera_get_driver_name,
  .validate_frame_setting   = sim_camera_validate_frame_setting,
  .start_capture            = sim_camera_start_capture,
  .stop_capture             = sim_camera_stop_capture,
  .get_supported_value      = sim_camera_get_supported_value,
  .get_value                = sim_camera_get_value,
  .set_value                = sim_camera_set_value,
};

static struct imgdata_ops_s g_sim_camera_data_ops =
  {
    .init                   = sim_camera_data_init,
    .uninit                 = sim_camera_data_uninit,
    .validate_buf           = sim_camera_data_validate_buf,
    .set_buf                = sim_camera_data_set_buf,
    .dq_buf                 = sim_camera_data_dq_buf,
    .enq_buf                = sim_camera_data_enq_buf,
    .validate_frame_setting = sim_camera_data_validate_frame_setting,
    .start_capture          = sim_camera_data_start_capture,
    .stop_capture           = sim_camera_data_stop_capture,
  };

static video_priv_t priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool sim_camera_is_available()
{
  return video_host_is_available(HOST_VIDEO_DEV_PATH);
}

static int sim_camera_init()
{
  return video_host_init(HOST_VIDEO_DEV_PATH);
}

static int sim_camera_uninit()
{
  return video_host_uninit();
}

static FAR const char *sim_camera_get_driver_name(void)
{
  return "v4l2 nuttx sim driver";
}

static int sim_camera_get_supported_value(uint32_t id,
                                     FAR imgsensor_supported_value_t *value)
{
  return 0;
}

static int sim_camera_get_value(uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value)
{
  return 0;
}

static int sim_camera_set_value(uint32_t id, uint32_t size,
                            imgsensor_value_t value)
{
  return 0;
}

static int validate_format(imgsensor_stream_type_t type, int nr_fmt,
    FAR imgsensor_format_t *fmt, FAR imgsensor_interval_t *interval)
{
  if (nr_fmt > 1)
    {
      return -ENOTSUP;
    }

  uint32_t v4l2_fmt = imgsensor_fmt_to_v4l2(fmt->pixelformat);
  if (type == IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      return video_host_try_fmt(fmt->width, fmt->height,
        v4l2_fmt, interval->denominator, interval->numerator);
    }
  else
    {
      return video_host_try_fmt(fmt->width, fmt->height, v4l2_fmt, 0, 0);
    }
}

static int sim_camera_validate_frame_setting(imgsensor_stream_type_t type,
                                         uint8_t nr_fmt,
                                         FAR imgsensor_format_t *fmt,
                                         FAR imgsensor_interval_t *interval)
{
  return validate_format(type, nr_fmt, fmt, interval);
}

static int sim_camera_start_capture(imgsensor_stream_type_t type,
                                uint8_t nr_fmt,
                                FAR imgsensor_format_t *fmt,
                                FAR imgsensor_interval_t *interval)
{
  return video_host_set_fmt(fmt[IMGDATA_FMT_MAIN].width,
    fmt[IMGDATA_FMT_MAIN].height,
    imgsensor_fmt_to_v4l2(fmt[IMGDATA_FMT_MAIN].pixelformat),
    interval->denominator, interval->numerator);
}

static int sim_camera_stop_capture(imgsensor_stream_type_t type)
{
  return video_host_stop_capture();
}

static int sim_camera_data_init()
{
  memset(&priv, 0, sizeof(priv));
  return video_host_data_init();
}

static int sim_camera_data_uninit()
{
  return 0;
}

static int sim_camera_data_validate_frame_setting
             (uint8_t nr_datafmt,
              imgdata_format_t *datafmt,
              imgdata_interval_t *interval)
{
  return 0;
}

static int sim_camera_data_start_capture
             (uint8_t nr_datafmt,
              imgdata_format_t *datafmt,
              imgdata_interval_t *interval,
              imgdata_capture_t callback)
{
  int i;
  int ret;

  priv.capture_cb = callback;
  priv.capture_size = datafmt->width * datafmt->height * 2;
  ret = video_host_start_capture(priv.buf_q.num);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < priv.buf_q.num; i++)
    {
      video_host_enq_buf(priv.buf_q.addr[i], priv.buf_q.size[i]);
    }

  return 0;
}

static int sim_camera_data_stop_capture()
{
  return 0;
}

static int sim_camera_data_validate_buf(uint8_t *addr, uint32_t size)
{
  if (!addr || (uintptr_t)(addr) & 0x1f)
    {
      return -EINVAL;
    }

  return 0;
}

static int sim_camera_data_set_buf(uint8_t *addr, uint32_t size)
{
  return 0;
}

static int sim_camera_data_enq_buf(uint8_t *addr, uint32_t size)
{
  int ret = sim_camera_data_validate_buf(addr, size);
  if (ret != 0)
    {
      return ret;
    }

  if (priv.capture_cb)
    {
      video_host_enq_buf(addr, size);
    }
  else
    {
      ENQUEUE(priv.buf_q, addr, size);
    }

  return 0;
}

static int sim_camera_data_dq_buf(uint8_t **addr, struct timeval *ts)
{
  int ret = video_host_dq_buf(addr, ts);
  priv.capture_cb(0, priv.capture_size);
  return ret;
}

static uint32_t imgsensor_fmt_to_v4l2(uint32_t pixelformat)
{
  uint32_t fourcc;
  switch (pixelformat)
    {
      case IMGSENSOR_PIX_FMT_YUYV:
        fourcc = V4L2_PIX_FMT_YUYV;
        break;

      case IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG:
        fourcc = V4L2_PIX_FMT_JPEG;
        break;

      case IMGSENSOR_PIX_FMT_JPEG:
        fourcc = V4L2_PIX_FMT_JPEG;
        break;

      case IMGSENSOR_PIX_FMT_RGB565:
        fourcc = V4L2_PIX_FMT_RGB565;
        break;

      case IMGSENSOR_PIX_FMT_UYVY:
        fourcc = V4L2_PIX_FMT_UYVY;
        break;

      default:

      /* Unsupported format */

        fourcc = 0;
    }

  return fourcc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_camera_initialize(void)
{
  imgsensor_register(&g_sim_camera_ops);
  imgdata_register(&g_sim_camera_data_ops);
  return 0;
}

int sim_camera_uninitialize(void)
{
  return 0;
}
