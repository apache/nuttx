/****************************************************************************
 * arch/sim/src/sim/sim_video.c
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
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/video.h>

#include "sim_hostvideo.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  imgdata_capture_t capture_cb;
  uint32_t buf_size;
  uint8_t  *next_buf;
  struct host_video_dev_s *vdev;
} sim_video_priv_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Video image sensor operations */

static bool sim_video_is_available(void);
static int sim_video_init(void);
static int sim_video_uninit(void);
static const char *sim_video_get_driver_name(void);
static int sim_video_validate_frame_setting(imgsensor_stream_type_t type,
                                            uint8_t nr_datafmt,
                                            imgsensor_format_t *datafmts,
                                            imgsensor_interval_t *interval);
static int sim_video_start_capture(imgsensor_stream_type_t type,
                                   uint8_t nr_datafmt,
                                   imgsensor_format_t *datafmts,
                                   imgsensor_interval_t *interval);
static int sim_video_stop_capture(imgsensor_stream_type_t type);

/* Video image data operations */

static int sim_video_data_init(void);
static int sim_video_data_uninit(void);
static int sim_video_data_validate_frame_setting(uint8_t nr_datafmt,
                                                 imgdata_format_t *datafmt,
                                                 imgdata_interval_t *interv);
static int sim_video_data_start_capture(uint8_t nr_datafmt,
                                        imgdata_format_t *datafmt,
                                        imgdata_interval_t *interval,
                                        imgdata_capture_t callback);
static int sim_video_data_stop_capture(void);
static int sim_video_data_validate_buf(uint8_t *addr, uint32_t size);
static int sim_video_data_set_buf(uint8_t *addr, uint32_t size);

static uint32_t imgdata_fmt_to_v4l2(uint32_t pixelformat);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgsensor_ops_s g_sim_video_ops =
{
  .is_available             = sim_video_is_available,
  .init                     = sim_video_init,
  .uninit                   = sim_video_uninit,
  .get_driver_name          = sim_video_get_driver_name,
  .validate_frame_setting   = sim_video_validate_frame_setting,
  .start_capture            = sim_video_start_capture,
  .stop_capture             = sim_video_stop_capture,
};

static const struct imgdata_ops_s g_sim_video_data_ops =
{
  .init                     = sim_video_data_init,
  .uninit                   = sim_video_data_uninit,
  .set_buf                  = sim_video_data_set_buf,
  .validate_frame_setting   = sim_video_data_validate_frame_setting,
  .start_capture            = sim_video_data_start_capture,
  .stop_capture             = sim_video_data_stop_capture,
};

static sim_video_priv_t g_sim_video_priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Sensor op functions are mostly dummy */

static bool sim_video_is_available(void)
{
  return host_video_is_available(CONFIG_HOST_VIDEO_DEV_PATH);
}

static int sim_video_init(void)
{
  return 0;
}

static int sim_video_uninit(void)
{
  return 0;
}

static const char *sim_video_get_driver_name(void)
{
  return "V4L2 NuttX Sim Driver";
}

static int sim_video_validate_frame_setting(imgsensor_stream_type_t type,
                                            uint8_t nr_fmt,
                                            imgsensor_format_t *fmt,
                                            imgsensor_interval_t *interval)
{
  return 0;
}

static int sim_video_start_capture(imgsensor_stream_type_t type,
                                   uint8_t nr_fmt,
                                   imgsensor_format_t *fmt,
                                   imgsensor_interval_t *interval)
{
  return 0;
}

static int sim_video_stop_capture(imgsensor_stream_type_t type)
{
  return 0;
}

/* Data op functions do all the real work */

static int sim_video_data_init(void)
{
  memset(&g_sim_video_priv, 0, sizeof(g_sim_video_priv));
  g_sim_video_priv.vdev = host_video_init(CONFIG_HOST_VIDEO_DEV_PATH);
  if (g_sim_video_priv.vdev == NULL)
    {
      return -ENODEV;
    }

  return 0;
}

static int sim_video_data_uninit(void)
{
  return host_video_uninit(g_sim_video_priv.vdev);
}

static int sim_video_data_validate_buf(uint8_t *addr, uint32_t size)
{
  if (!addr || ((uintptr_t)(addr) & 0x1f))
    {
      return -EINVAL;
    }

  return 0;
}

static int sim_video_data_set_buf(uint8_t *addr, uint32_t size)
{
  if (sim_video_data_validate_buf(addr, size) < 0)
    {
      return -EINVAL;
    }

  g_sim_video_priv.next_buf = addr;
  g_sim_video_priv.buf_size = size;
  return 0;
}

static int sim_video_data_validate_frame_setting(uint8_t nr_datafmt,
                                                 imgdata_format_t *datafmt,
                                                 imgdata_interval_t *interv)
{
  uint32_t v4l2_fmt;

  if (nr_datafmt > 1)
    {
      return -ENOTSUP;
    }

  v4l2_fmt = imgdata_fmt_to_v4l2(datafmt->pixelformat);
  return host_video_try_fmt(g_sim_video_priv.vdev, datafmt->width,
                            datafmt->height, v4l2_fmt, interv->denominator,
                            interv->numerator);
}

static int sim_video_data_start_capture(uint8_t nr_datafmt,
                                        imgdata_format_t *datafmt,
                                        imgdata_interval_t *interval,
                                        imgdata_capture_t callback)
{
  int ret;

  ret = host_video_set_fmt(g_sim_video_priv.vdev,
                           datafmt[IMGDATA_FMT_MAIN].width,
                           datafmt[IMGDATA_FMT_MAIN].height,
                           imgdata_fmt_to_v4l2(
                             datafmt[IMGDATA_FMT_MAIN].pixelformat),
                           interval->denominator, interval->numerator);
  if (ret < 0)
    {
      return ret;
    }

  g_sim_video_priv.capture_cb = callback;
  return host_video_start_capture(g_sim_video_priv.vdev);
}

static int sim_video_data_stop_capture(void)
{
  g_sim_video_priv.next_buf = NULL;
  return host_video_stop_capture(g_sim_video_priv.vdev);
}

/* Helper functions */

static uint32_t imgdata_fmt_to_v4l2(uint32_t pixelformat)
{
  uint32_t fourcc;
  switch (pixelformat)
    {
      case IMGDATA_PIX_FMT_YUV420P:
        fourcc = V4L2_PIX_FMT_YUV420;
        break;

      case IMGDATA_PIX_FMT_YUYV:
        fourcc = V4L2_PIX_FMT_YUYV;
        break;

      case IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG:
        fourcc = V4L2_PIX_FMT_JPEG;
        break;

      case IMGDATA_PIX_FMT_JPEG:
        fourcc = V4L2_PIX_FMT_JPEG;
        break;

      case IMGDATA_PIX_FMT_RGB565:
        fourcc = V4L2_PIX_FMT_RGB565;
        break;

      case IMGDATA_PIX_FMT_UYVY:
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

int sim_video_initialize(void)
{
  imgsensor_register(&g_sim_video_ops);
  imgdata_register(&g_sim_video_data_ops);
  return 0;
}

int sim_video_uninitialize(void)
{
  return 0;
}

void sim_video_loop(void)
{
  int ret;

  if (g_sim_video_priv.next_buf)
    {
      ret = host_video_dqbuf(g_sim_video_priv.vdev,
                             g_sim_video_priv.next_buf,
                             g_sim_video_priv.buf_size);
      if (ret > 0)
        {
          g_sim_video_priv.capture_cb(0, ret);
        }
    }
}
