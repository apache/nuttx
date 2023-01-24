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
  struct imgdata_s data;
  struct imgsensor_s sensor;
  imgdata_capture_t capture_cb;
  uint32_t buf_size;
  uint8_t  *next_buf;
  struct host_video_dev_s *vdev;
} sim_video_priv_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Video image sensor operations */

static bool sim_video_is_available(struct imgsensor_s *sensor);
static int sim_video_init(struct imgsensor_s *sensor);
static int sim_video_uninit(struct imgsensor_s *sensor);
static const char *sim_video_get_driver_name(struct imgsensor_s *sensor);
static int sim_video_validate_frame_setting(struct imgsensor_s *sensor,
                                            imgsensor_stream_type_t type,
                                            uint8_t nr_datafmt,
                                            imgsensor_format_t *datafmts,
                                            imgsensor_interval_t *interval);
static int sim_video_start_capture(struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type,
                                   uint8_t nr_datafmt,
                                   imgsensor_format_t *datafmts,
                                   imgsensor_interval_t *interval);
static int sim_video_stop_capture(struct imgsensor_s *sensor,
                                  imgsensor_stream_type_t type);

/* Video image data operations */

static int sim_video_data_init(struct imgdata_s *data);
static int sim_video_data_uninit(struct imgdata_s *data);
static int sim_video_data_validate_frame_setting(struct imgdata_s *data,
                                                 uint8_t nr_datafmt,
                                                 imgdata_format_t *datafmt,
                                                 imgdata_interval_t *interv);
static int sim_video_data_start_capture(struct imgdata_s *data,
                                        uint8_t nr_datafmt,
                                        imgdata_format_t *datafmt,
                                        imgdata_interval_t *interval,
                                        imgdata_capture_t callback);
static int sim_video_data_stop_capture(struct imgdata_s *data);
static int sim_video_data_set_buf(struct imgdata_s *data,
                                  uint8_t *addr, uint32_t size);

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

static sim_video_priv_t g_sim_video_priv =
{
  .data =
  {
    &g_sim_video_data_ops
  },
  .sensor =
  {
    &g_sim_video_ops
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

/* Sensor op functions are mostly dummy */

static bool sim_video_is_available(struct imgsensor_s *sensor)
{
  return host_video_is_available(CONFIG_HOST_VIDEO_DEV_PATH);
}

static int sim_video_init(struct imgsensor_s *sensor)
{
  return 0;
}

static int sim_video_uninit(struct imgsensor_s *sensor)
{
  return 0;
}

static const char *sim_video_get_driver_name(struct imgsensor_s *sensor)
{
  return "V4L2 NuttX Sim Driver";
}

static int sim_video_validate_frame_setting(struct imgsensor_s *sensor,
                                            imgsensor_stream_type_t type,
                                            uint8_t nr_fmt,
                                            imgsensor_format_t *fmt,
                                            imgsensor_interval_t *interval)
{
  return 0;
}

static int sim_video_start_capture(struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type,
                                   uint8_t nr_fmt,
                                   imgsensor_format_t *fmt,
                                   imgsensor_interval_t *interval)
{
  return 0;
}

static int sim_video_stop_capture(struct imgsensor_s *sensor,
                                  imgsensor_stream_type_t type)
{
  return 0;
}

/* Data op functions do all the real work */

static int sim_video_data_init(struct imgdata_s *data)
{
  sim_video_priv_t *priv = (sim_video_priv_t *)data;

  priv->vdev = host_video_init(CONFIG_HOST_VIDEO_DEV_PATH);
  if (priv->vdev == NULL)
    {
      return -ENODEV;
    }

  return 0;
}

static int sim_video_data_uninit(struct imgdata_s *data)
{
  sim_video_priv_t *priv = (sim_video_priv_t *)data;

  return host_video_uninit(priv->vdev);
}

static int sim_video_data_validate_buf(uint8_t *addr, uint32_t size)
{
  if (!addr || ((uintptr_t)(addr) & 0x1f))
    {
      return -EINVAL;
    }

  return 0;
}

static int sim_video_data_set_buf(struct imgdata_s *data,
                                  uint8_t *addr, uint32_t size)
{
  sim_video_priv_t *priv = (sim_video_priv_t *)data;

  if (sim_video_data_validate_buf(addr, size) < 0)
    {
      return -EINVAL;
    }

  priv->next_buf = addr;
  priv->buf_size = size;
  return 0;
}

static int sim_video_data_validate_frame_setting(struct imgdata_s *data,
                                                 uint8_t nr_datafmt,
                                                 imgdata_format_t *datafmt,
                                                 imgdata_interval_t *interv)
{
  sim_video_priv_t *priv = (sim_video_priv_t *)data;
  uint32_t v4l2_fmt;

  if (nr_datafmt > 1)
    {
      return -ENOTSUP;
    }

  v4l2_fmt = imgdata_fmt_to_v4l2(datafmt->pixelformat);
  return host_video_try_fmt(priv->vdev, datafmt->width,
                            datafmt->height, v4l2_fmt, interv->denominator,
                            interv->numerator);
}

static int sim_video_data_start_capture(struct imgdata_s *data,
                                        uint8_t nr_datafmt,
                                        imgdata_format_t *datafmt,
                                        imgdata_interval_t *interval,
                                        imgdata_capture_t callback)
{
  sim_video_priv_t *priv = (sim_video_priv_t *)data;
  int ret;

  ret = host_video_set_fmt(priv->vdev,
                           datafmt[IMGDATA_FMT_MAIN].width,
                           datafmt[IMGDATA_FMT_MAIN].height,
                           imgdata_fmt_to_v4l2(
                             datafmt[IMGDATA_FMT_MAIN].pixelformat),
                           interval->denominator, interval->numerator);
  if (ret < 0 && ret != -EBUSY)
    {
      return ret;
    }

  priv->capture_cb = callback;
  return host_video_start_capture(priv->vdev);
}

static int sim_video_data_stop_capture(struct imgdata_s *data)
{
  sim_video_priv_t *priv = (sim_video_priv_t *)data;

  priv->next_buf = NULL;
  return host_video_stop_capture(priv->vdev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_video_initialize(void)
{
  sim_video_priv_t *priv = &g_sim_video_priv;

  imgsensor_register(&priv->sensor);
  imgdata_register(&priv->data);
  return 0;
}

int sim_video_uninitialize(void)
{
  return 0;
}

void sim_video_loop(void)
{
  sim_video_priv_t *priv = &g_sim_video_priv;
  int ret;

  if (priv->next_buf)
    {
      ret = host_video_dqbuf(priv->vdev, priv->next_buf, priv->buf_size);
      if (ret > 0)
        {
          priv->capture_cb(0, ret);
        }
    }
}
