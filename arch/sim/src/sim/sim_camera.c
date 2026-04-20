/****************************************************************************
 * arch/sim/src/sim/sim_camera.c
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
#include <inttypes.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/wdog.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/video.h>

#include "sim_hostvideo.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_CAMERA_PERIOD    MSEC2TICK(CONFIG_SIM_LOOP_INTERVAL)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct sim_camera_priv_s sim_camera_priv_t;

struct sim_camera_priv_s
{
  struct imgdata_s data;
  struct imgsensor_s sensor;
  imgdata_capture_t capture_cb;
  void *capture_arg;
  uint32_t buf_size;
  uint8_t *next_buf;
  struct host_video_dev_s *vdev;
  struct wdog_s wdog;
  bool capture_started;
  int index;
  char devpath[32];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Video image sensor operations */

static bool sim_camera_is_available(struct imgsensor_s *sensor);
static int sim_camera_init(struct imgsensor_s *sensor);
static int sim_camera_uninit(struct imgsensor_s *sensor);
static const char *sim_camera_get_driver_name(struct imgsensor_s *sensor);
static int sim_camera_validate_frame_setting(struct imgsensor_s *sensor,
                                             imgsensor_stream_type_t type,
                                             uint8_t nr_datafmt,
                                             imgsensor_format_t *datafmts,
                                             imgsensor_interval_t *interval);
static int sim_camera_start_capture(struct imgsensor_s *sensor,
                                    imgsensor_stream_type_t type,
                                    uint8_t nr_datafmt,
                                    imgsensor_format_t *datafmts,
                                    imgsensor_interval_t *interval);
static int sim_camera_stop_capture(struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type);

/* Video image data operations */

static int sim_camera_data_init(struct imgdata_s *data);
static int sim_camera_data_uninit(struct imgdata_s *data);
static int
sim_camera_data_validate_frame_setting(struct imgdata_s *data,
                                       uint8_t nr_datafmt,
                                       imgdata_format_t *datafmt,
                                       imgdata_interval_t *interv);
static int sim_camera_data_start_capture(struct imgdata_s *data,
                                         uint8_t nr_datafmt,
                                         imgdata_format_t *datafmt,
                                         imgdata_interval_t *interval,
                                         imgdata_capture_t callback,
                                         void *arg);
static int sim_camera_data_stop_capture(struct imgdata_s *data);
static int sim_camera_data_set_buf(struct imgdata_s *data,
                                   uint8_t nr_datafmts,
                                   imgdata_format_t *datafmts,
                                   uint8_t *addr, uint32_t size);
static void sim_camera_interrupt(wdparm_t arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgsensor_ops_s g_sim_camera_ops =
{
  .is_available           = sim_camera_is_available,
  .init                   = sim_camera_init,
  .uninit                 = sim_camera_uninit,
  .get_driver_name        = sim_camera_get_driver_name,
  .validate_frame_setting = sim_camera_validate_frame_setting,
  .start_capture          = sim_camera_start_capture,
  .stop_capture           = sim_camera_stop_capture,
};

static const struct imgdata_ops_s g_sim_camera_data_ops =
{
  .init                   = sim_camera_data_init,
  .uninit                 = sim_camera_data_uninit,
  .set_buf                = sim_camera_data_set_buf,
  .validate_frame_setting = sim_camera_data_validate_frame_setting,
  .start_capture          = sim_camera_data_start_capture,
  .stop_capture           = sim_camera_data_stop_capture,
};

static const struct v4l2_frmsizeenum g_frmsizes[] =
{
  {
    .type = V4L2_FRMSIZE_TYPE_DISCRETE,
    .discrete =
    {
      .width = 640,
      .height = 480,
    }
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
      case IMGDATA_PIX_FMT_NV12:
        fourcc = V4L2_PIX_FMT_NV12;
        break;

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

static bool sim_camera_is_available(struct imgsensor_s *sensor)
{
  sim_camera_priv_t *priv = container_of(sensor, sim_camera_priv_t, sensor);
  return host_video_is_available(priv->devpath);
}

static int sim_camera_init(struct imgsensor_s *sensor)
{
  return 0;
}

static int sim_camera_uninit(struct imgsensor_s *sensor)
{
  return 0;
}

static const char *sim_camera_get_driver_name(struct imgsensor_s *sensor)
{
  return "V4L2 NuttX Sim Driver";
}

static int sim_camera_validate_frame_setting(struct imgsensor_s *sensor,
                                             imgsensor_stream_type_t type,
                                             uint8_t nr_fmt,
                                             imgsensor_format_t *fmt,
                                             imgsensor_interval_t *interval)
{
  sim_camera_priv_t *priv = container_of(sensor, sim_camera_priv_t, sensor);
  uint32_t v4l2_fmt;

  if (nr_fmt > 1)
    {
      return -ENOTSUP;
    }

  v4l2_fmt = imgdata_fmt_to_v4l2(fmt[IMGSENSOR_FMT_MAIN].pixelformat);
  if (v4l2_fmt == 0)
    {
      verr("sim_camera[%d]: unsupported sensor pixfmt=%" PRIu32 "\n",
           priv->index, fmt[IMGSENSOR_FMT_MAIN].pixelformat);
      return -EINVAL;
    }

  return host_video_try_fmt(priv->vdev, fmt[IMGSENSOR_FMT_MAIN].width,
                            fmt[IMGSENSOR_FMT_MAIN].height, v4l2_fmt,
                            interval->denominator, interval->numerator);
}

static int sim_camera_start_capture(struct imgsensor_s *sensor,
                                    imgsensor_stream_type_t type,
                                    uint8_t nr_fmt,
                                    imgsensor_format_t *fmt,
                                    imgsensor_interval_t *interval)
{
  return 0;
}

static int sim_camera_stop_capture(struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type)
{
  return 0;
}

/* Data op functions do all the real work */

static int sim_camera_data_init(struct imgdata_s *data)
{
  sim_camera_priv_t *priv = container_of(data, sim_camera_priv_t, data);

  if (priv->vdev == NULL)
    {
      priv->vdev = host_video_init(priv->devpath);
      if (priv->vdev == NULL)
        {
          return -ENODEV;
        }
    }

  return 0;
}

static int sim_camera_data_uninit(struct imgdata_s *data)
{
  sim_camera_priv_t *priv = container_of(data, sim_camera_priv_t, data);
  int ret = 0;

  if (priv->vdev != NULL)
    {
      ret = host_video_uninit(priv->vdev);
      priv->vdev = NULL;
    }

  return ret;
}

static int sim_camera_data_validate_buf(uint8_t *addr, uint32_t size)
{
  if (!addr || ((uintptr_t)(addr) & 0x1f))
    {
      return -EINVAL;
    }

  return 0;
}

static int sim_camera_data_set_buf(struct imgdata_s *data,
                                   uint8_t nr_datafmts,
                                   imgdata_format_t *datafmts,
                                   uint8_t *addr, uint32_t size)
{
  sim_camera_priv_t *priv = container_of(data, sim_camera_priv_t, data);

  if (sim_camera_data_validate_buf(addr, size) < 0)
    {
      return -EINVAL;
    }

  priv->next_buf = addr;
  priv->buf_size = size;
  return 0;
}

static int sim_camera_data_validate_frame_setting(struct imgdata_s *data,
                                                  uint8_t nr_datafmt,
                                                  imgdata_format_t *datafmt,
                                                  imgdata_interval_t *interv)
{
  sim_camera_priv_t *priv = container_of(data, sim_camera_priv_t, data);
  uint32_t v4l2_fmt;

  if (nr_datafmt > 1)
    {
      return -ENOTSUP;
    }

  v4l2_fmt = imgdata_fmt_to_v4l2(datafmt->pixelformat);
  if (v4l2_fmt == 0)
    {
      verr("sim_camera[%d]: unsupported data pixfmt=%" PRIu32 "\n",
           priv->index, datafmt->pixelformat);
      return -EINVAL;
    }

  return host_video_try_fmt(priv->vdev, datafmt->width,
                            datafmt->height, v4l2_fmt, interv->denominator,
                            interv->numerator);
}

static int sim_camera_data_start_capture(struct imgdata_s *data,
                                         uint8_t nr_datafmt,
                                         imgdata_format_t *datafmt,
                                         imgdata_interval_t *interval,
                                         imgdata_capture_t callback,
                                         void *arg)
{
  sim_camera_priv_t *priv = container_of(data, sim_camera_priv_t, data);
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
  priv->capture_arg = arg;

  ret = host_video_start_capture(priv->vdev);
  if (ret < 0)
    {
      return ret;
    }

  priv->capture_started = true;
  wd_start(&priv->wdog, SIM_CAMERA_PERIOD, sim_camera_interrupt,
           (wdparm_t)priv);
  return 0;
}

static int sim_camera_data_stop_capture(struct imgdata_s *data)
{
  sim_camera_priv_t *priv = container_of(data, sim_camera_priv_t, data);

  priv->next_buf = NULL;
  priv->capture_started = false;
  wd_cancel(&priv->wdog);
  return host_video_stop_capture(priv->vdev);
}

static void sim_camera_interrupt(wdparm_t arg)
{
  sim_camera_priv_t *priv = (sim_camera_priv_t *)arg;
  struct timespec ts;
  struct timeval tv;
  int ret;

  if (priv == NULL)
    {
      return;
    }

  if (priv->next_buf != NULL)
    {
      ret = host_video_dqbuf(priv->vdev, priv->next_buf, priv->buf_size);
      if (ret > 0)
        {
          clock_gettime(CLOCK_MONOTONIC, &ts);
          TIMESPEC_TO_TIMEVAL(&tv, &ts);
          priv->capture_cb(0, ret, &tv, priv->capture_arg);
        }
    }

  if (priv->capture_started && priv->next_buf != NULL)
    {
      wd_start(&priv->wdog, SIM_CAMERA_PERIOD, sim_camera_interrupt,
               (wdparm_t)priv);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_camera_initialize(void)
{
  sim_camera_priv_t *privs;
  int count;
  int first_error = 0;

  count = host_video_get_device_count();
  if (count < 0)
    {
      return count;
    }

  privs = kmm_zalloc(sizeof(sim_camera_priv_t) * count);
  if (privs == NULL)
    {
      return -ENOMEM;
    }

  for (int i = 0; i < count; i++)
    {
      sim_camera_priv_t *priv = &privs[i];
      FAR struct imgsensor_s *sensor;
      int ret;

      priv->data.ops = &g_sim_camera_data_ops;
      priv->sensor.ops = &g_sim_camera_ops;
      priv->sensor.frmsizes_num = 1;
      priv->sensor.frmsizes = g_frmsizes;
      sensor = &priv->sensor;
      priv->index = i;
      snprintf(priv->devpath, sizeof(priv->devpath), "%s%d",
               CONFIG_SIM_CAMERA_DEV_PATH, i);

      ret = capture_register(priv->devpath, &priv->data, &sensor, 1);
      if (ret < 0 && first_error == 0)
        {
          first_error = ret;
        }
    }

  return first_error;
}
