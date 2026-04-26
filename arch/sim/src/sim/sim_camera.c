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
#include <string.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
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

struct sim_camera_imgdata_s
{
  struct imgdata_s data;
  sim_camera_priv_t *priv;
};

struct sim_camera_sensor_s
{
  struct imgsensor_s sensor;
  sim_camera_priv_t *priv;
};

struct sim_camera_priv_s
{
  struct sim_camera_imgdata_s data;
  struct sim_camera_sensor_s sensor;
  imgdata_capture_t capture_cb;
  void *capture_arg;
  uint32_t buf_size;
  uint8_t  *next_buf;
  struct timeval *next_ts;
  struct host_video_dev_s *vdev;
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
void sim_camera_loop(void);

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

static sim_camera_priv_t g_sim_camera_priv =
{
  .data =
  {
    .data =
    {
      .ops = &g_sim_camera_data_ops,
    },
  },
  .sensor =
  {
    .sensor =
    {
      .ops = &g_sim_camera_ops,
      .frmsizes_num = 1,
      .frmsizes = g_frmsizes,
    },
  }
};

static sim_camera_priv_t *g_sim_camera_privs;
static int g_sim_camera_priv_count;
static struct wdog_s g_camera_wdog;

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
  struct sim_camera_sensor_s *sim_sensor =
    (struct sim_camera_sensor_s *)sensor;
  sim_camera_priv_t *priv = sim_sensor->priv;
  bool available = host_video_is_available(priv->devpath);

  /* Availability probing is expected during boot; keep it at debug level. */

  vinfo("sim_camera[%d]: is_available(%s)=%d\n",
        priv->index, priv->devpath, available);
  return available;
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
  struct sim_camera_sensor_s *sim_sensor =
    (struct sim_camera_sensor_s *)sensor;
  sim_camera_priv_t *priv = sim_sensor->priv;
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
  struct sim_camera_imgdata_s *sim_data =
    (struct sim_camera_imgdata_s *)data;
  sim_camera_priv_t *priv = sim_data->priv;

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
  struct sim_camera_imgdata_s *sim_data =
    (struct sim_camera_imgdata_s *)data;
  sim_camera_priv_t *priv = sim_data->priv;
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
  struct sim_camera_imgdata_s *sim_data =
    (struct sim_camera_imgdata_s *)data;
  sim_camera_priv_t *priv = sim_data->priv;

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
  struct sim_camera_imgdata_s *sim_data =
    (struct sim_camera_imgdata_s *)data;
  sim_camera_priv_t *priv = sim_data->priv;
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
  struct sim_camera_imgdata_s *sim_data =
    (struct sim_camera_imgdata_s *)data;
  sim_camera_priv_t *priv = sim_data->priv;
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

  wd_start(&g_camera_wdog, SIM_CAMERA_PERIOD, sim_camera_interrupt, 0);
  return 0;
}

static int sim_camera_data_stop_capture(struct imgdata_s *data)
{
  struct sim_camera_imgdata_s *sim_data =
    (struct sim_camera_imgdata_s *)data;
  sim_camera_priv_t *priv = sim_data->priv;

  priv->next_buf = NULL;
  return host_video_stop_capture(priv->vdev);
}

static void sim_camera_interrupt(wdparm_t arg)
{
  bool active = false;

  sim_camera_loop();

  for (int i = 0; i < g_sim_camera_priv_count; i++)
    {
      if (g_sim_camera_privs[i].next_buf != NULL)
        {
          active = true;
          break;
        }
    }

  if (active)
    {
      wd_start(&g_camera_wdog, SIM_CAMERA_PERIOD, sim_camera_interrupt, 0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_camera_initialize(void)
{
  int count;

  count = host_video_get_device_count();
  if (count <= 0)
    {
      count = 1;
    }

  g_sim_camera_privs = kmm_zalloc(sizeof(sim_camera_priv_t) * (size_t)count);
  if (g_sim_camera_privs == NULL)
    {
      return -ENOMEM;
    }

  g_sim_camera_priv_count = count;

  for (int i = 0; i < count; i++)
    {
      sim_camera_priv_t *priv = &g_sim_camera_privs[i];

      memcpy(priv, &g_sim_camera_priv, sizeof(*priv));

      priv->data.priv = priv;
      priv->sensor.priv = priv;
      priv->index = i;
      snprintf(priv->devpath, sizeof(priv->devpath), "%s%d",
               CONFIG_SIM_CAMERA_DEV_PATH, i);

      imgdata_register(&priv->data.data);

      imgsensor_register(&priv->sensor.sensor);
    }

  return 0;
}

int sim_camera_register_capture_devices(void)
{
  int first_error = 0;

  if (g_sim_camera_privs == NULL || g_sim_camera_priv_count <= 0)
    {
      return -EINVAL;
    }

  for (int i = 0; i < g_sim_camera_priv_count; i++)
    {
      sim_camera_priv_t *priv = &g_sim_camera_privs[i];
      FAR struct imgsensor_s *sensor = &priv->sensor.sensor;
      int ret;

      ret = capture_register(priv->devpath, &priv->data.data, &sensor, 1);
      if (ret < 0 && first_error == 0)
        {
          first_error = ret;
        }
    }

  return first_error;
}

void sim_camera_loop(void)
{
  struct timespec ts;
  struct timeval tv;
  int ret;

  if (g_sim_camera_privs == NULL || g_sim_camera_priv_count <= 0)
    {
      return;
    }

  for (int i = 0; i < g_sim_camera_priv_count; i++)
    {
      sim_camera_priv_t *priv = &g_sim_camera_privs[i];

      if (priv->next_buf)
        {
          ret = host_video_dqbuf(priv->vdev, priv->next_buf, priv->buf_size);
          if (ret > 0)
            {
              clock_gettime(CLOCK_MONOTONIC, &ts);
              TIMESPEC_TO_TIMEVAL(&tv, &ts);
              priv->capture_cb(0, ret, &tv, priv->capture_arg);
            }
        }
    }
}
