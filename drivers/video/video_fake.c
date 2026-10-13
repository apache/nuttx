/****************************************************************************
 * drivers/video/video_fake.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/spinlock.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/video.h>
#include <nuttx/video/video_fake.h>
#include <nuttx/wqueue.h>

#include "video_fake.h"

struct video_fake_glyph_s
{
  char value;
  uint8_t rows[VIDEO_FAKE_FONT_HEIGHT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool video_fake_is_available(FAR struct imgsensor_s *sensor);
static int video_fake_sensor_init(FAR struct imgsensor_s *sensor);
static int
video_fake_sensor_start(FAR struct imgsensor_s *sensor,
                        imgsensor_stream_type_t type,
                        uint8_t nr_datafmts,
                        FAR imgsensor_format_t *datafmts,
                        FAR imgsensor_interval_t *interval);
static int
video_fake_sensor_stop(FAR struct imgsensor_s *sensor,
                       imgsensor_stream_type_t type);
static FAR const char *
video_fake_get_driver_name(FAR struct imgsensor_s *sensor);
static int
video_fake_sensor_validate(FAR struct imgsensor_s *sensor,
                           imgsensor_stream_type_t type,
                           uint8_t nr_datafmts,
                           FAR imgsensor_format_t *datafmts,
                           FAR imgsensor_interval_t *interval);
static int
video_fake_get_supported_value(FAR struct imgsensor_s *sensor,
                               uint32_t id,
                               FAR imgsensor_supported_value_t *value);
static int video_fake_get_value(FAR struct imgsensor_s *sensor,
                                uint32_t id, uint32_t size,
                                FAR imgsensor_value_t *value);
static int video_fake_set_value(FAR struct imgsensor_s *sensor,
                                uint32_t id, uint32_t size,
                                imgsensor_value_t value);

static int video_fake_data_init(FAR struct imgdata_s *data);
static int video_fake_data_uninit(FAR struct imgdata_s *data);
static int video_fake_set_buffer(FAR struct imgdata_s *data,
                                 uint8_t nr_datafmts,
                                 FAR imgdata_format_t *datafmts,
                                 FAR uint8_t *addr, uint32_t size);
static int video_fake_data_validate(FAR struct imgdata_s *data,
                                    uint8_t nr_datafmts,
                                    FAR imgdata_format_t *datafmts,
                                    FAR imgdata_interval_t *interval);
static int video_fake_start_capture(FAR struct imgdata_s *data,
                                    uint8_t nr_datafmts,
                                    FAR imgdata_format_t *datafmts,
                                    FAR imgdata_interval_t *interval,
                                    imgdata_capture_t callback,
                                    FAR void *arg);
static int video_fake_stop_capture(FAR struct imgdata_s *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const video_fake_params_t g_devparams[] =
{
#ifdef CONFIG_VIDEO_FAKE_DEVICE_PARAMS
  CONFIG_VIDEO_FAKE_DEVICE_PARAMS
#endif
};

static const uint8_t g_video_fake_bars_yuv[VIDEO_FAKE_BARS * 3] =
{
  235, 128, 128,  210,  16, 146,  170, 166,  16,  145,  54,  34,
  106, 202, 222,   81,  90, 240,   41, 240, 110,   16, 128, 128,
};

static const uint8_t g_video_fake_bars_rgb[VIDEO_FAKE_BARS * 3] =
{
  255, 255, 255,  255, 255,   0,    0, 255, 255,    0, 255,   0,
  255,   0, 255,  255,   0,   0,    0,   0, 255,    0,   0,   0,
};

static const struct video_fake_glyph_s g_video_fake_font[] =
{
  {
    ' ',
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    '.',
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00}
  },
  {
    '/',
    {0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x80, 0x00}
  },
  {
    '0',
    {0x7c, 0xce, 0xde, 0xf6, 0xe6, 0xc6, 0x7c, 0x00}
  },
  {
    '1',
    {0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7e, 0x00}
  },
  {
    '2',
    {0x7c, 0xc6, 0x06, 0x7c, 0xc0, 0xc0, 0xfe, 0x00}
  },
  {
    '3',
    {0x7c, 0xc6, 0x06, 0x3c, 0x06, 0xc6, 0x7c, 0x00}
  },
  {
    '4',
    {0x0c, 0x2c, 0x4c, 0x8c, 0xfe, 0x0c, 0x0c, 0x00}
  },
  {
    '5',
    {0xfe, 0xc0, 0xfc, 0x06, 0x06, 0xc6, 0x7c, 0x00}
  },
  {
    '6',
    {0x7c, 0xc6, 0xc0, 0xfc, 0xc6, 0xc6, 0x7c, 0x00}
  },
  {
    '7',
    {0xfe, 0xc6, 0x0c, 0x18, 0x30, 0x30, 0x30, 0x00}
  },
  {
    '8',
    {0x7c, 0xc6, 0xc6, 0x7c, 0xc6, 0xc6, 0x7c, 0x00}
  },
  {
    '9',
    {0x7c, 0xc6, 0xc6, 0x7e, 0x06, 0xc6, 0x7c, 0x00}
  },
  {
    ':',
    {0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00}
  },
  {
    'A',
    {0x38, 0x6c, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0x00}
  },
  {
    'B',
    {0xfc, 0xc6, 0xc6, 0xfc, 0xc6, 0xc6, 0xfc, 0x00}
  },
  {
    'C',
    {0x7c, 0xc6, 0xc0, 0xc0, 0xc0, 0xc6, 0x7c, 0x00}
  },
  {
    'D',
    {0xf8, 0xcc, 0xc6, 0xc6, 0xc6, 0xcc, 0xf8, 0x00}
  },
  {
    'E',
    {0xfe, 0xc0, 0xc0, 0xfc, 0xc0, 0xc0, 0xfe, 0x00}
  },
  {
    'F',
    {0xfe, 0xc0, 0xc0, 0xfc, 0xc0, 0xc0, 0xc0, 0x00}
  },
  {
    'G',
    {0x7c, 0xc6, 0xc0, 0xce, 0xc6, 0xc6, 0x7e, 0x00}
  },
  {
    'H',
    {0xc6, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0x00}
  },
  {
    'I',
    {0x7e, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7e, 0x00}
  },
  {
    'J',
    {0x06, 0x06, 0x06, 0x06, 0xc6, 0xc6, 0x7c, 0x00}
  },
  {
    'K',
    {0xc6, 0xcc, 0xd8, 0xf0, 0xd8, 0xcc, 0xc6, 0x00}
  },
  {
    'L',
    {0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xfe, 0x00}
  },
  {
    'M',
    {0xc6, 0xee, 0xfe, 0xd6, 0xc6, 0xc6, 0xc6, 0x00}
  },
  {
    'N',
    {0xc6, 0xe6, 0xf6, 0xde, 0xce, 0xc6, 0xc6, 0x00}
  },
  {
    'O',
    {0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00}
  },
  {
    'P',
    {0xfc, 0xc6, 0xc6, 0xfc, 0xc0, 0xc0, 0xc0, 0x00}
  },
  {
    'Q',
    {0x7c, 0xc6, 0xc6, 0xc6, 0xd6, 0xde, 0x7c, 0x06}
  },
  {
    'R',
    {0xfc, 0xc6, 0xc6, 0xfc, 0xd8, 0xcc, 0xc6, 0x00}
  },
  {
    'S',
    {0x7c, 0xc6, 0xc0, 0x7c, 0x06, 0xc6, 0x7c, 0x00}
  },
  {
    'T',
    {0xfe, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}
  },
  {
    'U',
    {0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00}
  },
  {
    'V',
    {0xc6, 0xc6, 0xc6, 0xc6, 0x6c, 0x38, 0x10, 0x00}
  },
  {
    'W',
    {0xc6, 0xc6, 0xc6, 0xd6, 0xfe, 0xee, 0xc6, 0x00}
  },
  {
    'X',
    {0xc6, 0xc6, 0x6c, 0x38, 0x6c, 0xc6, 0xc6, 0x00}
  },
  {
    'Y',
    {0xc6, 0xc6, 0x6c, 0x38, 0x18, 0x18, 0x18, 0x00}
  },
  {
    'Z',
    {0xfe, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xfe, 0x00}
  },
  {
    '_',
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe}
  },
  {
    'x',
    {0x00, 0x00, 0xc6, 0x6c, 0x38, 0x6c, 0xc6, 0x00}
  },
};

static const struct imgsensor_ops_s g_video_fake_sensor_ops =
{
  .is_available           = video_fake_is_available,
  .init                   = video_fake_sensor_init,
  .get_driver_name        = video_fake_get_driver_name,
  .validate_frame_setting = video_fake_sensor_validate,
  .start_capture          = video_fake_sensor_start,
  .stop_capture           = video_fake_sensor_stop,
  .get_supported_value    = video_fake_get_supported_value,
  .get_value              = video_fake_get_value,
  .set_value              = video_fake_set_value,
};

static const struct imgdata_ops_s g_video_fake_data_ops =
{
  .init                   = video_fake_data_init,
  .uninit                 = video_fake_data_uninit,
  .set_buf                = video_fake_set_buffer,
  .validate_frame_setting = video_fake_data_validate,
  .start_capture          = video_fake_start_capture,
  .stop_capture           = video_fake_stop_capture,
};

static const uint32_t g_video_fake_control_ids[VIDEO_FAKE_CONTROL_COUNT] =
{
  IMGSENSOR_ID_BRIGHTNESS,          IMGSENSOR_ID_CONTRAST,
  IMGSENSOR_ID_SATURATION,          IMGSENSOR_ID_HUE,
  IMGSENSOR_ID_AUTO_WHITE_BALANCE,  IMGSENSOR_ID_RED_BALANCE,
  IMGSENSOR_ID_BLUE_BALANCE,        IMGSENSOR_ID_GAMMA,
  IMGSENSOR_ID_EXPOSURE,            IMGSENSOR_ID_HFLIP_VIDEO,
  IMGSENSOR_ID_VFLIP_VIDEO,         IMGSENSOR_ID_HFLIP_STILL,
  IMGSENSOR_ID_VFLIP_STILL,         IMGSENSOR_ID_SHARPNESS,
  IMGSENSOR_ID_COLOR_KILLER,        IMGSENSOR_ID_COLORFX,
  IMGSENSOR_ID_AUTOBRIGHTNESS,      IMGSENSOR_ID_ROTATE,
  IMGSENSOR_ID_EXPOSURE_AUTO,       IMGSENSOR_ID_EXPOSURE_ABSOLUTE,
  IMGSENSOR_ID_FOCUS_ABSOLUTE,      IMGSENSOR_ID_FOCUS_RELATIVE,
  IMGSENSOR_ID_FOCUS_AUTO,          IMGSENSOR_ID_ZOOM_ABSOLUTE,
  IMGSENSOR_ID_ZOOM_RELATIVE,       IMGSENSOR_ID_ZOOM_CONTINUOUS,
  IMGSENSOR_ID_IRIS_ABSOLUTE,       IMGSENSOR_ID_IRIS_RELATIVE,
  IMGSENSOR_ID_AUTO_N_PRESET_WB,    IMGSENSOR_ID_WIDE_DYNAMIC_RANGE,
  IMGSENSOR_ID_IMG_STABILIZATION,   IMGSENSOR_ID_ISO_SENSITIVITY,
  IMGSENSOR_ID_ISO_SENSITIVITY_AUTO, IMGSENSOR_ID_EXPOSURE_METERING,
  IMGSENSOR_ID_SPOT_POSITION,       IMGSENSOR_ID_3A_LOCK,
  IMGSENSOR_ID_AUTO_FOCUS_START,    IMGSENSOR_ID_AUTO_FOCUS_STOP,
  IMGSENSOR_ID_3A_STATUS,           IMGSENSOR_ID_FLASH_LED_MODE,
  IMGSENSOR_ID_JPEG_QUALITY
};

static const int32_t g_video_fake_colorfx_values[] =
{
  V4L2_COLORFX_NONE, V4L2_COLORFX_BW, V4L2_COLORFX_SEPIA,
  V4L2_COLORFX_NEGATIVE
};

static const int32_t g_video_fake_exposure_auto_values[] =
{
  V4L2_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
  V4L2_EXPOSURE_SHUTTER_PRIORITY, V4L2_EXPOSURE_APERTURE_PRIORITY
};

static const int32_t g_video_fake_wb_values[] =
{
  V4L2_WHITE_BALANCE_AUTO, V4L2_WHITE_BALANCE_INCANDESCENT,
  V4L2_WHITE_BALANCE_FLUORESCENT, V4L2_WHITE_BALANCE_DAYLIGHT
};

static const int32_t g_video_fake_metering_values[] =
{
  V4L2_EXPOSURE_METERING_AVERAGE, V4L2_EXPOSURE_METERING_CENTER_WEIGHTED,
  V4L2_EXPOSURE_METERING_SPOT, V4L2_EXPOSURE_METERING_MATRIX
};

static const int32_t g_video_fake_iso_values[] =
{
  100, 200, 400, 800
};

static const int32_t g_video_fake_led_values[] =
{
  V4L2_FLASH_LED_MODE_NONE, V4L2_FLASH_LED_MODE_FLASH,
  V4L2_FLASH_LED_MODE_TORCH
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int video_fake_format_to_internal(uint32_t format)
{
  switch (format)
    {
      case V4L2_PIX_FMT_YUV420:
        return IMGDATA_PIX_FMT_YUV420P;

      case V4L2_PIX_FMT_NV12:
        return IMGDATA_PIX_FMT_NV12;

      case V4L2_PIX_FMT_YUYV:
        return IMGDATA_PIX_FMT_YUYV;

      case V4L2_PIX_FMT_RGB565:
        return IMGDATA_PIX_FMT_RGB565;

      case V4L2_PIX_FMT_UYVY:
        return IMGDATA_PIX_FMT_UYVY;

      case V4L2_PIX_FMT_JPEG:
        return IMGDATA_PIX_FMT_JPEG;

      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:
        return IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG;

      case V4L2_PIX_FMT_ENTROPY:
        return IMGDATA_PIX_FMT_ENTROPY;

      default:
        return -EINVAL;
    }
}

static uint32_t video_fake_internal_to_format(uint32_t format)
{
  switch (format)
    {
      case IMGDATA_PIX_FMT_YUV420P:
        return V4L2_PIX_FMT_YUV420;

      case IMGDATA_PIX_FMT_NV12:
        return V4L2_PIX_FMT_NV12;

      case IMGDATA_PIX_FMT_YUYV:
        return V4L2_PIX_FMT_YUYV;

      case IMGDATA_PIX_FMT_RGB565:
        return V4L2_PIX_FMT_RGB565;

      case IMGDATA_PIX_FMT_UYVY:
        return V4L2_PIX_FMT_UYVY;

      case IMGDATA_PIX_FMT_JPEG:
        return V4L2_PIX_FMT_JPEG;

      case IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG:
        return V4L2_PIX_FMT_JPEG_WITH_SUBIMG;

      case IMGDATA_PIX_FMT_ENTROPY:
        return V4L2_PIX_FMT_ENTROPY;

      default:
        return 0;
    }
}

static FAR const char *video_fake_format_description(uint32_t format)
{
  switch (format)
    {
      case V4L2_PIX_FMT_YUV420:
        return "YUV 4:2:0 planar";

      case V4L2_PIX_FMT_NV12:
        return "YUV 4:2:0 NV12";

      case V4L2_PIX_FMT_YUYV:
        return "YUV 4:2:2 YUYV";

      case V4L2_PIX_FMT_UYVY:
        return "YUV 4:2:2 UYVY";

      case V4L2_PIX_FMT_JPEG:
        return "JPEG stub";

      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:
        return "JPEG with subimage stub";

      case V4L2_PIX_FMT_ENTROPY:
        return "Entropy stub";

      default:
        return "RGB565";
    }
}

static FAR const char *video_fake_format_name(uint32_t format)
{
  switch (format)
    {
      case IMGDATA_PIX_FMT_YUV420P:
        return "YU12";

      case IMGDATA_PIX_FMT_NV12:
        return "NV12";

      case IMGDATA_PIX_FMT_YUYV:
        return "YUYV";

      default:
        return "RGBP";
    }
}

static int
video_fake_validate_params(FAR const video_fake_params_t *params,
                           FAR struct video_fake_counts_s *counts)
{
  bool even_width = false;
  bool even_height = false;
  uint64_t max_pixels = UINT32_MAX / VIDEO_FAKE_MAX_BYTES_PIXEL;
  size_t i;
  size_t j;

  if (params->devpath == NULL || params->devpath[0] == '\0')
    {
      return -EINVAL;
    }

  memset(counts, 0, sizeof(*counts));

  /* The lists are zero terminated and zero is not a valid entry for any of
   * them, so each walk stops exactly where the configuration does.  Every
   * array also has room for the terminator, so a list that fills the array
   * without one is reported as the configuration error it is rather than
   * being silently cut short.
   */

  for (i = 0; params->formats[i] != 0; i++)
    {
      int internal;

      if (i >= VIDEO_FAKE_FORMATS_MAX)
        {
          return -EINVAL;
        }

      internal = video_fake_format_to_internal(params->formats[i]);

      if (internal < 0)
        {
          return -EINVAL;
        }

      if (internal != IMGDATA_PIX_FMT_RGB565)
        {
          even_width = true;
        }

      if (internal == IMGDATA_PIX_FMT_YUV420P ||
          internal == IMGDATA_PIX_FMT_NV12)
        {
          even_height = true;
        }

      for (j = 0; j < i; j++)
        {
          if (params->formats[j] == params->formats[i])
            {
              return -EINVAL;
            }
        }

      counts->formats++;
    }

  if (counts->formats == 0)
    {
      return -EINVAL;
    }

  for (i = 0; params->sizes[i].width != 0 ||
              params->sizes[i].height != 0; i++)
    {
      uint16_t width = params->sizes[i].width;
      uint16_t height = params->sizes[i].height;
      uint64_t frame_bytes;
      uint64_t frame_pixels;

      if (i >= VIDEO_FAKE_SIZES_MAX)
        {
          return -EINVAL;
        }

      frame_pixels = width;
      frame_pixels *= height;
      frame_bytes = frame_pixels * VIDEO_FAKE_MAX_BYTES_PIXEL;

      if (width == 0 || height == 0 ||
          frame_pixels > max_pixels ||
          frame_bytes > UINT32_MAX ||
          (even_width && (width & 1) != 0) ||
          (even_height && (height & 1) != 0))
        {
          return -EINVAL;
        }

      for (j = 0; j < i; j++)
        {
          if (params->sizes[j].width == width &&
              params->sizes[j].height == height)
            {
              return -EINVAL;
            }
        }

      counts->sizes++;
      counts->max_width = MAX(counts->max_width, width);
      counts->max_height = MAX(counts->max_height, height);
    }

  if (counts->sizes == 0)
    {
      return -EINVAL;
    }

  for (i = 0; params->fps[i] != 0; i++)
    {
      if (i >= VIDEO_FAKE_FPS_MAX)
        {
          return -EINVAL;
        }

      for (j = 0; j < i; j++)
        {
          if (params->fps[j] == params->fps[i])
            {
              return -EINVAL;
            }
        }

      counts->rates++;
    }

  return counts->rates == 0 ? -EINVAL : OK;
}

static void
video_fake_make_name(FAR struct video_fake_s *priv,
                     FAR const char *devpath)
{
  FAR const char *end = devpath + strlen(devpath);
  FAR const char *suffix = end;

  while (suffix > devpath && *(suffix - 1) >= '0' &&
         *(suffix - 1) <= '9')
    {
      suffix--;
    }

  strlcpy(priv->name, "VIDEO_FAKE", sizeof(priv->name));
  if (suffix != end)
    {
      strlcat(priv->name, suffix, sizeof(priv->name));
    }
}

static uint32_t
video_fake_scale_value(uint32_t value, uint64_t scale)
{
  uint64_t scaled = value * scale +
                    VIDEO_FAKE_SCALE_ONE / 2;
  uint32_t result = scaled >> VIDEO_FAKE_SCALE_SHIFT;

  return MAX(result, 1u);
}

/* Size the panel to the lines this instance will actually print, so the box
 * hugs its text instead of leaving dead space.  All four lines are known
 * here: the first two come from the stream settings, and the two counters
 * are bounded by the range of their own types:
 *
 *   FRAME counts frames in a uint32   -> "FRAME 4294967295"
 *   TIME is stream uptime in hh:mm:ss:mmm -> "TIME 00:00:00:000"
 */

static void video_fake_size_panel(FAR struct video_fake_s *priv)
{
  char line[VIDEO_FAKE_LINE_SIZE];
  uint64_t scale_x;
  uint64_t scale_y;
  uint64_t panel_width;
  uint64_t panel_height;
  uint32_t i;
  size_t width = strlen(priv->name);

  scale_x = priv->width;
  scale_x *= VIDEO_FAKE_SCALE_ONE;
  scale_x /= VIDEO_FAKE_REFERENCE_WIDTH;
  scale_y = priv->height;
  scale_y *= VIDEO_FAKE_SCALE_ONE;
  scale_y /= VIDEO_FAKE_REFERENCE_HEIGHT;

  priv->font_width = video_fake_scale_value(VIDEO_FAKE_FONT_WIDTH, scale_x);
  priv->font_height =
    video_fake_scale_value(VIDEO_FAKE_FONT_HEIGHT, scale_y);
  priv->font_advance =
    video_fake_scale_value(VIDEO_FAKE_FONT_ADVANCE, scale_x);
  priv->line_height =
    video_fake_scale_value(VIDEO_FAKE_LINE_HEIGHT, scale_y);
  priv->panel_pad_x =
    video_fake_scale_value(VIDEO_FAKE_PANEL_PAD_X, scale_x);
  priv->panel_pad_y =
    video_fake_scale_value(VIDEO_FAKE_PANEL_PAD_Y, scale_y);

  snprintf(line, sizeof(line), "%" PRIu16 "x%" PRIu16 " %s %" PRIu32,
           priv->width, priv->height,
           video_fake_format_name(priv->pixel_format), priv->fps);
  width = MAX(width, strlen(line));
  width = MAX(width, strlen("FRAME 4294967295"));
  width = MAX(width, strlen("TIME 00:00:00:000"));

  panel_width = width * priv->font_advance +
                2 * priv->panel_pad_x - 1;
  panel_height = (VIDEO_FAKE_PANEL_LINES - 1) * priv->line_height +
                 priv->font_height + 2 * priv->panel_pad_y;
  priv->panel_width = MIN(panel_width, priv->width);
  priv->panel_height = MIN(panel_height, priv->height);

  for (i = 0; i < priv->font_height; i++)
    {
      priv->source_rows[i] = i * VIDEO_FAKE_FONT_HEIGHT / priv->font_height;
    }

  for (i = 0; i < priv->font_width; i++)
    {
      priv->source_columns[i] = i * VIDEO_FAKE_FONT_WIDTH / priv->font_width;
    }
}

static int
video_fake_build_capabilities(FAR struct video_fake_s *priv,
                              FAR const video_fake_params_t *params,
                              FAR const struct video_fake_counts_s *counts)
{
  size_t i;

  /* The tables handed to the sensor are the V4L2-visible form of the
   * configuration, so they take the same length as the lists they came
   * from rather than a fixed cap.
   */

  priv->formats_num = counts->formats;
  priv->sizes_num = counts->sizes;
  priv->rates_num = counts->rates;

  priv->fmtdescs = kmm_zalloc(sizeof(*priv->fmtdescs) * counts->formats);
  priv->frmsizes = kmm_zalloc(sizeof(*priv->frmsizes) * counts->sizes);
  priv->frmintervals =
    kmm_zalloc(sizeof(*priv->frmintervals) * counts->rates);

  if (priv->fmtdescs == NULL || priv->frmsizes == NULL ||
      priv->frmintervals == NULL)
    {
      kmm_free(priv->fmtdescs);
      kmm_free(priv->frmsizes);
      kmm_free(priv->frmintervals);
      priv->fmtdescs = NULL;
      priv->frmsizes = NULL;
      priv->frmintervals = NULL;
      return -ENOMEM;
    }

  for (i = 0; i < counts->formats; i++)
    {
      priv->fmtdescs[i].pixelformat = params->formats[i];
      strlcpy(priv->fmtdescs[i].description,
              video_fake_format_description(params->formats[i]),
              sizeof(priv->fmtdescs[i].description));
    }

  for (i = 0; i < counts->sizes; i++)
    {
      priv->frmsizes[i].type = V4L2_FRMSIZE_TYPE_DISCRETE;
      priv->frmsizes[i].discrete.width = params->sizes[i].width;
      priv->frmsizes[i].discrete.height = params->sizes[i].height;
    }

  for (i = 0; i < counts->rates; i++)
    {
      priv->frmintervals[i].type = V4L2_FRMIVAL_TYPE_DISCRETE;
      priv->frmintervals[i].discrete.numerator = 1;
      priv->frmintervals[i].discrete.denominator = params->fps[i];
    }

  return OK;
}

static int
video_fake_check(FAR const struct video_fake_s *priv,
                 uint16_t width, uint16_t height, uint32_t format,
                 uint32_t numerator, uint32_t denominator)
{
  uint32_t fourcc = video_fake_internal_to_format(format);
  size_t i;

  if (fourcc == 0 || numerator == 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < priv->formats_num; i++)
    {
      if (priv->fmtdescs[i].pixelformat == fourcc)
        {
          break;
        }
    }

  if (i == priv->formats_num)
    {
      return -EINVAL;
    }

  for (i = 0; i < priv->sizes_num; i++)
    {
      if (priv->frmsizes[i].discrete.width == width &&
          priv->frmsizes[i].discrete.height == height)
        {
          break;
        }
    }

  if (i == priv->sizes_num)
    {
      return -EINVAL;
    }

  for (i = 0; i < priv->rates_num; i++)
    {
      uint32_t fps = priv->frmintervals[i].discrete.denominator;

      if (numerator != 0 && denominator % numerator == 0 &&
          denominator / numerator == fps)
        {
          return OK;
        }
    }

  return -EINVAL;
}

static int
video_fake_frame_bytes(uint16_t width, uint16_t height, uint32_t format,
                       FAR uint32_t *bytes)
{
  uint64_t size;

  size = width;
  size *= height;
  if (format == IMGDATA_PIX_FMT_YUV420P ||
      format == IMGDATA_PIX_FMT_NV12)
    {
      size = size * VIDEO_FAKE_YUV_SIZE_FACTOR / 2;
    }
  else
    {
      size *= VIDEO_FAKE_MAX_BYTES_PIXEL;
    }

  if (size > UINT32_MAX)
    {
      return -EOVERFLOW;
    }

  *bytes = size;
  return OK;
}

static void video_fake_next_generation(FAR struct video_fake_s *priv)
{
  priv->generation++;
  if (priv->generation == 0)
    {
      priv->generation++;
    }
}

static void
video_fake_bar_color(FAR const struct video_fake_s *priv, uint32_t x,
                     FAR uint8_t *color)
{
  bool rgb = priv->pixel_format == IMGDATA_PIX_FMT_RGB565;
  FAR const uint8_t *bars = rgb ? g_video_fake_bars_rgb :
                                  g_video_fake_bars_yuv;
  uint32_t bar = x * VIDEO_FAKE_BARS / priv->width;

  bar = MIN(bar, VIDEO_FAKE_BARS - 1);

  memcpy(color, bars + bar * 3, 3);
  if (!rgb)
    {
      color[0] = MIN(MAX(color[0], 16), 235);
    }
}

static void
video_fake_mono_color(FAR const struct video_fake_s *priv, bool white,
                      FAR uint8_t *color)
{
  bool rgb = priv->pixel_format == IMGDATA_PIX_FMT_RGB565;
  FAR const uint8_t *bars = rgb ? g_video_fake_bars_rgb :
                                  g_video_fake_bars_yuv;
  size_t offset = white ? 0 : (VIDEO_FAKE_BARS - 1) * 3;

  memcpy(color, bars + offset, 3);
}

static void
video_fake_store_row_pixel(FAR const struct video_fake_s *priv,
                           FAR uint8_t *row, uint32_t x,
                           FAR const uint8_t *color)
{
  uint32_t width = priv->width;
  uint16_t rgb565;

  switch (priv->pixel_format)
    {
      case IMGDATA_PIX_FMT_RGB565:
        rgb565 = (color[0] >> 3) << 11 |
                 (color[1] >> 2) << 5 | color[2] >> 3;
        row[2 * x] = rgb565 & 0xff;
        row[2 * x + 1] = rgb565 >> 8;
        break;

      case IMGDATA_PIX_FMT_YUYV:
        row[2 * x] = color[0];
        if ((x & 1) == 0)
          {
            row[2 * x + 1] = color[1];
            row[2 * x + 3] = color[2];
          }
        break;

      case IMGDATA_PIX_FMT_NV12:
        row[x] = color[0];
        if ((x & 1) == 0)
          {
            row[width + x] = color[1];
            row[width + x + 1] = color[2];
          }
        break;

      default:
        row[x] = color[0];
        if ((x & 1) == 0)
          {
            row[width + x / 2] = color[1];
            row[width + width / 2 + x / 2] = color[2];
          }
        break;
    }
}

static void
video_fake_build_background_row(FAR struct video_fake_s *priv,
                                uint32_t generation)
{
  uint8_t color[3];
  uint32_t x;

  for (x = 0; x < priv->width; x++)
    {
      video_fake_bar_color(priv, x, color);
      video_fake_store_row_pixel(priv, priv->background_row, x, color);
    }

  priv->background_row_generation = generation;
}

static void
video_fake_fill_background(FAR const struct video_fake_s *priv,
                           FAR uint8_t *buffer)
{
  uint32_t width = priv->width;
  uint32_t height = priv->height;
  FAR uint8_t *chroma;
  FAR uint8_t *plane;
  uint32_t y;

  if (priv->pixel_format == IMGDATA_PIX_FMT_YUYV ||
      priv->pixel_format == IMGDATA_PIX_FMT_RGB565)
    {
      for (y = 0; y < height; y++)
        {
          memcpy(buffer + y * width * 2, priv->background_row, width * 2);
        }

      return;
    }

  for (y = 0; y < height; y++)
    {
      memcpy(buffer + y * width, priv->background_row, width);
    }

  chroma = buffer + width * height;
  if (priv->pixel_format == IMGDATA_PIX_FMT_NV12)
    {
      for (y = 0; y < height / 2; y++)
        {
          memcpy(chroma + y * width,
                 priv->background_row + width, width);
        }
    }
  else
    {
      plane = chroma + width * height / 4;
      for (y = 0; y < height / 2; y++)
        {
          memcpy(chroma + y * width / 2,
                 priv->background_row + width, width / 2);
          memcpy(plane + y * width / 2,
                 priv->background_row + width + width / 2, width / 2);
        }
    }
}

static void
video_fake_put_pixel(FAR const struct video_fake_s *priv,
                     FAR uint8_t *buffer, uint32_t x, uint32_t y,
                     FAR const uint8_t *color)
{
  uint32_t width = priv->width;
  uint32_t height = priv->height;
  FAR uint8_t *pixel;
  size_t offset;
  size_t yplane;
  uint16_t rgb565;

  if (x >= width || y >= height)
    {
      return;
    }

  switch (priv->pixel_format)
    {
      case IMGDATA_PIX_FMT_RGB565:
        rgb565 = (color[0] >> 3) << 11 |
                 (color[1] >> 2) << 5 | color[2] >> 3;
        pixel = buffer + (y * width + x) * 2;
        pixel[0] = rgb565 & 0xff;
        pixel[1] = rgb565 >> 8;
        break;

      case IMGDATA_PIX_FMT_YUYV:
        pixel = buffer + (y * width + x) * 2;
        pixel[0] = color[0];
        if ((x & 1) == 0)
          {
            pixel[1] = color[1];
            pixel[3] = color[2];
          }
        break;

      case IMGDATA_PIX_FMT_NV12:
        buffer[y * width + x] = color[0];
        if (((x | y) & 1) == 0)
          {
            pixel = buffer + width * height + y / 2 * width + x;
            pixel[0] = color[1];
            pixel[1] = color[2];
          }
        break;

      default:
        buffer[y * width + x] = color[0];
        if (((x | y) & 1) == 0)
          {
            yplane = width * height;
            offset = (y / 2) * (width / 2) + x / 2;
            buffer[yplane + offset] = color[1];
            buffer[yplane * 5 / 4 + offset] = color[2];
          }
        break;
    }
}

static void
video_fake_fill_rect(FAR const struct video_fake_s *priv,
                     FAR uint8_t *buffer, uint32_t x, uint32_t y,
                     uint32_t width, uint32_t height,
                     FAR const uint8_t *color)
{
  uint32_t xend = MIN(x + width, priv->width);
  uint32_t yend = MIN(y + height, priv->height);
  uint32_t xpos;
  uint32_t ypos;

  for (ypos = y; ypos < yend; ypos++)
    {
      for (xpos = x; xpos < xend; xpos++)
        {
          video_fake_put_pixel(priv, buffer, xpos, ypos, color);
        }
    }
}

static FAR const struct video_fake_glyph_s *
video_fake_find_glyph(char value)
{
  size_t index;

  switch (value)
    {
      case ' ':
        index = 0;
        break;

      case '.':
        index = 1;
        break;

      case '/':
        index = 2;
        break;

      case ':':
        index = 13;
        break;

      case '_':
        index = 40;
        break;

      case 'x':
        index = 41;
        break;

      default:
        if (value >= '0' && value <= '9')
          {
            index = 3 + value - '0';
          }
        else if (value >= 'A' && value <= 'Z')
          {
            index = 14 + value - 'A';
          }
        else
          {
            return NULL;
          }
        break;
    }

  return &g_video_fake_font[index];
}

static void
video_fake_draw_text(FAR const struct video_fake_s *priv,
                     FAR uint8_t *buffer, uint32_t x, uint32_t y,
                     FAR const char *text)
{
  FAR const struct video_fake_glyph_s *glyph;
  uint8_t white[3];
  uint32_t column;
  uint32_t row;
  uint32_t limit;

  if (priv->panel_width <= 2 * priv->panel_pad_x)
    {
      return;
    }

  limit = x + priv->panel_width - 2 * priv->panel_pad_x;

  video_fake_mono_color(priv, true, white);
  while (*text != '\0' && x + priv->font_width <= limit)
    {
      glyph = video_fake_find_glyph(*text++);
      if (glyph != NULL)
        {
          for (row = 0; row < priv->font_height; row++)
            {
              for (column = 0; column < priv->font_width; column++)
                {
                  if ((glyph->rows[priv->source_rows[row]] &
                       (1 << (VIDEO_FAKE_FONT_WIDTH - 1 -
                              priv->source_columns[column]))) != 0)
                    {
                      video_fake_put_pixel(priv, buffer, x + column,
                                           y + row, white);
                    }
                }
            }
        }

      x += priv->font_advance;
    }
}

static uint32_t
video_fake_panel_x(FAR const struct video_fake_s *priv)
{
  /* Keep the panel on an even chroma boundary for 4:2:0 formats, so the
   * first column owns its own U/V sample instead of sharing the bar colour
   * that sits to its left.
   */

  return ((priv->width - priv->panel_width) / 2) & ~1u;
}

static uint32_t
video_fake_panel_y(FAR const struct video_fake_s *priv)
{
  uint32_t slack = priv->height - priv->panel_height;

  /* Keep the panel on an even chroma boundary for 4:2:0 formats. */

  return (slack / 2) & ~1u;
}

static void
video_fake_draw_static_panel(FAR const struct video_fake_s *priv,
                             FAR uint8_t *buffer)
{
  uint32_t x = video_fake_panel_x(priv);
  uint32_t y = video_fake_panel_y(priv);
  uint32_t tx = x + priv->panel_pad_x;
  char line[VIDEO_FAKE_LINE_SIZE];
  uint8_t black[3];

  video_fake_mono_color(priv, false, black);
  video_fake_fill_rect(priv, buffer, x, y, priv->panel_width,
                       priv->panel_height, black);

  y += priv->panel_pad_y;
  video_fake_draw_text(priv, buffer, tx, y, priv->name);
  snprintf(line, sizeof(line), "%" PRIu16 "x%" PRIu16 " %s %" PRIu32,
           priv->width, priv->height,
           video_fake_format_name(priv->pixel_format), priv->fps);
  video_fake_draw_text(priv, buffer, tx,
                       y + priv->line_height, line);
}

static void
video_fake_draw_dynamic_panel(FAR const struct video_fake_s *priv,
                              FAR uint8_t *buffer, uint32_t frame,
                              uint32_t elapsed)
{
  char line[VIDEO_FAKE_LINE_SIZE];
  uint32_t seconds = elapsed / MSEC_PER_SEC;
  uint32_t msec = elapsed % MSEC_PER_SEC;
  uint32_t hour = seconds / SEC_PER_HOUR;
  uint32_t minute = seconds / SEC_PER_MIN % MIN_PER_HOUR;
  uint32_t second = seconds % SEC_PER_MIN;
  uint32_t x = video_fake_panel_x(priv);
  uint32_t y = video_fake_panel_y(priv) + priv->panel_pad_y +
               VIDEO_FAKE_DYNAMIC_LINE * priv->line_height;
  uint32_t tx = x + priv->panel_pad_x;
  uint8_t black[3];

  video_fake_mono_color(priv, false, black);

  /* Repaint just the two dynamic rows.  The last row only needs one glyph
   * height below its baseline, so the rectangle stops there instead of
   * running a full line past the bottom of the panel.
   */

  video_fake_fill_rect(priv, buffer, x, y, priv->panel_width,
                       priv->line_height + priv->font_height, black);

  snprintf(line, sizeof(line), "FRAME %08" PRIu32, frame);
  video_fake_draw_text(priv, buffer, tx, y, line);

  /* The millisecond field is what lets the panel be read back as a rate. */

  snprintf(line, sizeof(line), "TIME %02" PRIu32 ":%02" PRIu32 ":%02" PRIu32
           ":%03" PRIu32, hour % 100, minute, second, msec);
  video_fake_draw_text(priv, buffer, tx,
                       y + priv->line_height, line);
}

static FAR struct video_fake_slot_s *
video_fake_get_slot(FAR struct video_fake_s *priv, FAR uint8_t *buffer)
{
  FAR struct video_fake_slot_s *oldest = &priv->slots[0];
  size_t i;

  for (i = 0; i < VIDEO_FAKE_SLOT_COUNT; i++)
    {
      if (priv->slots[i].addr == buffer)
        {
          return &priv->slots[i];
        }

      if (priv->slots[i].addr == NULL)
        {
          memset(&priv->slots[i], 0, sizeof(priv->slots[i]));
          priv->slots[i].addr = buffer;
          return &priv->slots[i];
        }

      if (priv->slots[i].recency < oldest->recency)
        {
          oldest = &priv->slots[i];
        }
    }

  memset(oldest, 0, sizeof(*oldest));
  oldest->addr = buffer;
  return oldest;
}

static void
video_fake_render(FAR struct video_fake_s *priv, FAR uint8_t *buffer,
                  uint32_t generation, uint32_t frame, uint32_t elapsed)
{
  FAR struct video_fake_slot_s *slot;

  if (priv->background_row_generation != generation)
    {
      video_fake_build_background_row(priv, generation);
    }

  slot = video_fake_get_slot(priv, buffer);
  if (slot->background_generation != generation)
    {
      video_fake_fill_background(priv, buffer);
      video_fake_draw_static_panel(priv, buffer);
    }

  video_fake_draw_dynamic_panel(priv, buffer, frame, elapsed);

  slot->background_generation = generation;
  slot->recency = ++priv->recency;
}

static clock_t video_fake_next_delay(FAR struct video_fake_s *priv)
{
  clock_t now = clock_systime_ticks();
  clock_t period;

  priv->tick_acc += priv->tick_num;
  period = priv->tick_acc / priv->tick_den;
  priv->tick_acc %= priv->tick_den;
  period = MAX(period, 1);
  priv->next_tick += period;

  if (clock_compare(now, priv->next_tick))
    {
      return priv->next_tick - now;
    }

  if (now - priv->next_tick > period)
    {
      priv->next_tick = now + period;
      return period;
    }

  return 0;
}

static void video_fake_worker(FAR void *arg)
{
  FAR struct video_fake_s *priv = arg;
  imgdata_capture_t callback;
  FAR void *callback_arg;
  FAR uint8_t *buffer;
  uint32_t size;
  uint32_t length = 0;
  uint32_t frame;
  uint32_t generation;
  uint32_t elapsed;
  clock_t since_base;
  uint8_t result = 0;
  clock_t now;
  struct timespec capture_time;
  struct timeval timestamp;
  irqstate_t flags;
  int ret = OK;

  flags = spin_lock_irqsave(&priv->lock);
  if (!priv->running)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return;
    }

  callback = priv->callback;
  callback_arg = priv->callback_arg;
  buffer = priv->next_buffer;
  size = priv->buffer_size;
  generation = priv->generation;
  frame = priv->frame++;
  now = clock_systime_ticks();
  spin_unlock_irqrestore(&priv->lock, flags);

  /* TIME is the stream clock shown on the panel, so it comes from the
   * stream start rather than the wall clock: it stays monotonic and is
   * unaffected by time-of-day adjustments.  Anchoring it to the first
   * delivered frame makes FRAME and TIME read as one pair -- FRAME n is
   * stamped n periods after FRAME 0, so n divided by TIME is the rate that
   * is actually being delivered.
   */

  if (frame == 0)
    {
      priv->uptime_base = now;
    }

  since_base = now - priv->uptime_base;
  elapsed = TICK2MSEC(since_base);

  if (buffer != NULL)
    {
      if (size < priv->frame_bytes)
        {
          result = EINVAL;
          length = size;
        }
      else
        {
          if (priv->pixel_format == IMGDATA_PIX_FMT_JPEG ||
              priv->pixel_format == IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG ||
              priv->pixel_format == IMGDATA_PIX_FMT_ENTROPY ||
              priv->pixel_format == IMGDATA_PIX_FMT_UYVY)
            {
              memset(buffer, 0, priv->frame_bytes);
            }
          else
            {
              video_fake_render(priv, buffer, generation, frame, elapsed);
            }

          length = priv->frame_bytes;
        }
    }

  clock_systime_timespec(&capture_time);
  TIMESPEC_TO_TIMEVAL(&timestamp, &capture_time);
  callback(result, length, &timestamp, callback_arg);

  flags = spin_lock_irqsave_nopreempt(&priv->lock);
  if (priv->running)
    {
      ret = work_queue(LPWORK, &priv->work, video_fake_worker, priv,
                       video_fake_next_delay(priv));
      if (ret < 0)
        {
          priv->running = false;
        }
    }

  spin_unlock_irqrestore_nopreempt(&priv->lock, flags);

  if (ret < 0)
    {
      verr("Failed to requeue fake video work: %d\n", ret);
    }
}

static bool video_fake_is_available(FAR struct imgsensor_s *sensor)
{
  return true;
}

static int video_fake_sensor_init(FAR struct imgsensor_s *sensor)
{
  return OK;
}

static int
video_fake_sensor_start(FAR struct imgsensor_s *sensor,
                        imgsensor_stream_type_t type,
                        uint8_t nr_datafmts,
                        FAR imgsensor_format_t *datafmts,
                        FAR imgsensor_interval_t *interval)
{
  if (interval == NULL)
    {
      return -EINVAL;
    }

  return type == IMGSENSOR_STREAM_TYPE_VIDEO ||
         type == IMGSENSOR_STREAM_TYPE_STILL ? OK : -ENOTSUP;
}

static int
video_fake_sensor_stop(FAR struct imgsensor_s *sensor,
                       imgsensor_stream_type_t type)
{
  return OK;
}

static FAR const char *
video_fake_get_driver_name(FAR struct imgsensor_s *sensor)
{
  return "video_fake";
}

static bool video_fake_valid_subimage(uint16_t main_width,
                                      uint16_t main_height,
                                      uint16_t width, uint16_t height,
                                      uint32_t format)
{
  return width > 0 && height > 0 && (width & 1) == 0 &&
         width <= main_width && height <= main_height &&
         (format == IMGDATA_PIX_FMT_UYVY ||
          format == IMGDATA_PIX_FMT_RGB565);
}

static int
video_fake_sensor_validate(FAR struct imgsensor_s *sensor,
                           imgsensor_stream_type_t type,
                           uint8_t nr_datafmts,
                           FAR imgsensor_format_t *datafmts,
                           FAR imgsensor_interval_t *interval)
{
  FAR struct video_fake_s *priv =
    container_of(sensor, struct video_fake_s, sensor);

  if ((type != IMGSENSOR_STREAM_TYPE_VIDEO &&
       type != IMGSENSOR_STREAM_TYPE_STILL) || nr_datafmts == 0 ||
      nr_datafmts > IMGSENSOR_FMT_MAX)
    {
      return -ENOTSUP;
    }

  if (nr_datafmts == 2 &&
      (datafmts[IMGSENSOR_FMT_MAIN].pixelformat !=
       IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG ||
       !video_fake_valid_subimage(datafmts[IMGSENSOR_FMT_MAIN].width,
                                  datafmts[IMGSENSOR_FMT_MAIN].height,
                                  datafmts[IMGSENSOR_FMT_SUB].width,
                                  datafmts[IMGSENSOR_FMT_SUB].height,
                                  datafmts[IMGSENSOR_FMT_SUB].pixelformat)))
    {
      return -EINVAL;
    }

  return video_fake_check(priv, datafmts[IMGSENSOR_FMT_MAIN].width,
                          datafmts[IMGSENSOR_FMT_MAIN].height,
                          datafmts[IMGSENSOR_FMT_MAIN].pixelformat,
                          interval->numerator, interval->denominator);
}

static bool video_fake_control_id(uint32_t id)
{
  switch (id)
    {
      case IMGSENSOR_ID_BRIGHTNESS:
      case IMGSENSOR_ID_CONTRAST:
      case IMGSENSOR_ID_SATURATION:
      case IMGSENSOR_ID_HUE:
      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
      case IMGSENSOR_ID_RED_BALANCE:
      case IMGSENSOR_ID_BLUE_BALANCE:
      case IMGSENSOR_ID_GAMMA:
      case IMGSENSOR_ID_GAMMA_CURVE:
      case IMGSENSOR_ID_EXPOSURE:
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_VFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
      case IMGSENSOR_ID_VFLIP_STILL:
      case IMGSENSOR_ID_SHARPNESS:
      case IMGSENSOR_ID_COLOR_KILLER:
      case IMGSENSOR_ID_COLORFX:
      case IMGSENSOR_ID_AUTOBRIGHTNESS:
      case IMGSENSOR_ID_ROTATE:
      case IMGSENSOR_ID_EXPOSURE_AUTO:
      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
      case IMGSENSOR_ID_FOCUS_ABSOLUTE:
      case IMGSENSOR_ID_FOCUS_RELATIVE:
      case IMGSENSOR_ID_FOCUS_AUTO:
      case IMGSENSOR_ID_ZOOM_ABSOLUTE:
      case IMGSENSOR_ID_ZOOM_RELATIVE:
      case IMGSENSOR_ID_ZOOM_CONTINUOUS:
      case IMGSENSOR_ID_IRIS_ABSOLUTE:
      case IMGSENSOR_ID_IRIS_RELATIVE:
      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
      case IMGSENSOR_ID_IMG_STABILIZATION:
      case IMGSENSOR_ID_ISO_SENSITIVITY:
      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
      case IMGSENSOR_ID_EXPOSURE_METERING:
      case IMGSENSOR_ID_SPOT_POSITION:
      case IMGSENSOR_ID_3A_LOCK:
      case IMGSENSOR_ID_AUTO_FOCUS_START:
      case IMGSENSOR_ID_AUTO_FOCUS_STOP:
      case IMGSENSOR_ID_3A_PARAMETER:
      case IMGSENSOR_ID_3A_STATUS:
      case IMGSENSOR_ID_FLASH_LED_MODE:
      case IMGSENSOR_ID_JPEG_QUALITY:
      case IMGSENSOR_ID_CLIP_VIDEO:
      case IMGSENSOR_ID_CLIP_STILL:
        return true;

      default:
        return false;
    }
}

static int video_fake_control_index(uint32_t id)
{
  int index;

  for (index = 0; index < VIDEO_FAKE_CONTROL_COUNT; index++)
    {
      if (g_video_fake_control_ids[index] == id)
        {
          return index;
        }
    }

  return -EINVAL;
}

static int
video_fake_get_supported_value(FAR struct imgsensor_s *sensor,
                               uint32_t id,
                               FAR imgsensor_supported_value_t *value)
{
  if (value == NULL || !video_fake_control_id(id))
    {
      return -EINVAL;
    }

  memset(value, 0, sizeof(*value));
  value->type = IMGSENSOR_CTRL_TYPE_INTEGER;
  value->u.range.minimum = -100;
  value->u.range.maximum = 100;
  value->u.range.step = 1;
  switch (id)
    {
      case IMGSENSOR_ID_ROTATE:
        value->u.range.minimum = 0;
        value->u.range.maximum = 270;
        break;

      case IMGSENSOR_ID_IRIS_ABSOLUTE:
      case IMGSENSOR_ID_IRIS_RELATIVE:
        value->u.range.minimum = 0;
        break;

      case IMGSENSOR_ID_COLORFX:
      case IMGSENSOR_ID_EXPOSURE_AUTO:
      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
      case IMGSENSOR_ID_EXPOSURE_METERING:
      case IMGSENSOR_ID_ISO_SENSITIVITY:
      case IMGSENSOR_ID_FLASH_LED_MODE:
        value->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        switch (id)
          {
            case IMGSENSOR_ID_EXPOSURE_AUTO:
              value->u.discrete.values = g_video_fake_exposure_auto_values;
              value->u.discrete.nr_values =
                nitems(g_video_fake_exposure_auto_values);
              break;

            case IMGSENSOR_ID_AUTO_N_PRESET_WB:
              value->u.discrete.values = g_video_fake_wb_values;
              value->u.discrete.nr_values = nitems(g_video_fake_wb_values);
              break;

            case IMGSENSOR_ID_EXPOSURE_METERING:
              value->u.discrete.values = g_video_fake_metering_values;
              value->u.discrete.nr_values =
                nitems(g_video_fake_metering_values);
              break;

            case IMGSENSOR_ID_ISO_SENSITIVITY:
              value->u.discrete.values = g_video_fake_iso_values;
              value->u.discrete.nr_values = nitems(g_video_fake_iso_values);
              break;

            case IMGSENSOR_ID_FLASH_LED_MODE:
              value->u.discrete.values = g_video_fake_led_values;
              value->u.discrete.nr_values = nitems(g_video_fake_led_values);
              break;

            default:
              value->u.discrete.values = g_video_fake_colorfx_values;
              value->u.discrete.nr_values =
                nitems(g_video_fake_colorfx_values);
              break;
          }

        value->u.discrete.default_value = value->u.discrete.values[0];
        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
      case IMGSENSOR_ID_HFLIP_VIDEO:
      case IMGSENSOR_ID_VFLIP_VIDEO:
      case IMGSENSOR_ID_HFLIP_STILL:
      case IMGSENSOR_ID_VFLIP_STILL:
      case IMGSENSOR_ID_COLOR_KILLER:
      case IMGSENSOR_ID_AUTOBRIGHTNESS:
      case IMGSENSOR_ID_FOCUS_AUTO:
      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
      case IMGSENSOR_ID_IMG_STABILIZATION:
      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
      case IMGSENSOR_ID_AUTO_FOCUS_START:
      case IMGSENSOR_ID_AUTO_FOCUS_STOP:
        value->type = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        value->u.range.minimum = 0;
        value->u.range.maximum = 1;
        break;

      case IMGSENSOR_ID_3A_LOCK:
      case IMGSENSOR_ID_3A_STATUS:
        value->type = IMGSENSOR_CTRL_TYPE_BITMASK;
        value->u.range.minimum = 0;
        value->u.range.maximum = 7;
        break;

      case IMGSENSOR_ID_GAMMA_CURVE:
      case IMGSENSOR_ID_3A_PARAMETER:
      case IMGSENSOR_ID_CLIP_VIDEO:
      case IMGSENSOR_ID_CLIP_STILL:
        value->type = id == IMGSENSOR_ID_GAMMA_CURVE ?
                      IMGSENSOR_CTRL_TYPE_U16 :
                      (id == IMGSENSOR_ID_3A_PARAMETER ?
                       IMGSENSOR_CTRL_TYPE_U8 : IMGSENSOR_CTRL_TYPE_U32);
        value->u.elems.nr_elems = id == IMGSENSOR_ID_GAMMA_CURVE ?
                                  VIDEO_FAKE_GAMMA_CURVE_NELEM :
                                  (id == IMGSENSOR_ID_3A_PARAMETER ?
                                   VIDEO_FAKE_3A_PARAMETER_NELEM :
                                   IMGSENSOR_CLIP_NELEM);
        value->u.elems.minimum = 0;
        value->u.elems.maximum = id == IMGSENSOR_ID_3A_PARAMETER ? 127 :
                                 (id == IMGSENSOR_ID_GAMMA_CURVE ?
                                  255 : UINT16_MAX);
        value->u.elems.step = 1;
        break;

      default:
        break;
    }

  return OK;
}

static void video_fake_controls_init(FAR struct video_fake_s *priv)
{
  imgsensor_supported_value_t supported;
  int index;

  /* Seed every control with the value it advertises as its default, so a
   * read issued before the first write still reports an accepted value.
   */

  for (index = 0; index < VIDEO_FAKE_CONTROL_COUNT; index++)
    {
      if (video_fake_get_supported_value(&priv->sensor,
                                         g_video_fake_control_ids[index],
                                         &supported) < 0)
        {
          continue;
        }

      priv->controls[index] =
        supported.type == IMGSENSOR_CTRL_TYPE_INTEGER_MENU ?
        supported.u.discrete.default_value :
        CLAMP(supported.u.range.default_value, INT32_MIN, INT32_MAX);
    }
}

static int video_fake_get_value(FAR struct imgsensor_s *sensor,
                                uint32_t id, uint32_t size,
                                FAR imgsensor_value_t *value)
{
  FAR struct video_fake_s *priv =
    container_of(sensor, struct video_fake_s, sensor);
  int index;

  if (value == NULL)
    {
      return -EINVAL;
    }

  /* The capture GET path hands the driver a zeroed union and copies the
   * pointer back, so the array controls lend their shadow row.  The scene
   * path passes a buffer of its own, which the framework then fills from
   * that row, so no write through the caller's pointer happens here.
   */

  switch (id)
    {
      case IMGSENSOR_ID_GAMMA_CURVE:
        value->p_u16 = priv->gamma_curve;
        break;

      case IMGSENSOR_ID_3A_PARAMETER:
        value->p_u8 = priv->threea_parameter;
        break;

      case IMGSENSOR_ID_CLIP_VIDEO:
        value->p_u32 = priv->clip_video;
        break;

      case IMGSENSOR_ID_CLIP_STILL:
        value->p_u32 = priv->clip_still;
        break;

      default:
        index = video_fake_control_index(id);
        if (index < 0)
          {
            return -EINVAL;
          }

        value->value64 = 0;
        value->value32 = priv->controls[index];
        break;
    }

  UNUSED(size);
  return OK;
}

static int
video_fake_set_value(FAR struct imgsensor_s *sensor, uint32_t id,
                     uint32_t size, imgsensor_value_t value)
{
  FAR struct video_fake_s *priv =
    container_of(sensor, struct video_fake_s, sensor);
  imgsensor_supported_value_t supported;
  size_t element_size;
  size_t row_size;
  int index;
  int i;
  int ret;

  ret = video_fake_get_supported_value(sensor, id, &supported);
  if (ret < 0)
    {
      return ret;
    }

  switch (supported.type)
    {
      case IMGSENSOR_CTRL_TYPE_INTEGER_MENU:
        for (i = 0; i < supported.u.discrete.nr_values; i++)
          {
            if (value.value32 == supported.u.discrete.values[i])
              {
                break;
              }
          }

        if (i == supported.u.discrete.nr_values)
          {
            return -EINVAL;
          }

        break;

      case IMGSENSOR_CTRL_TYPE_U8:
      case IMGSENSOR_CTRL_TYPE_U16:
      case IMGSENSOR_CTRL_TYPE_U32:
        element_size = supported.type == IMGSENSOR_CTRL_TYPE_U8 ? 1 :
                       (supported.type == IMGSENSOR_CTRL_TYPE_U16 ? 2 : 4);
        row_size = supported.u.elems.nr_elems * element_size;
        if (value.p_u8 == NULL || size != row_size)
          {
            return -EINVAL;
          }

        for (i = 0; i < supported.u.elems.nr_elems; i++)
          {
            int64_t element = supported.type == IMGSENSOR_CTRL_TYPE_U8 ?
                              value.p_u8[i] :
                              (supported.type == IMGSENSOR_CTRL_TYPE_U16 ?
                               value.p_u16[i] : value.p_u32[i]);

            if (element < supported.u.elems.minimum ||
                element > supported.u.elems.maximum ||
                (element - supported.u.elems.minimum) %
                 supported.u.elems.step != 0)
              {
                return -EINVAL;
              }
          }

        break;

      default:
        if (value.value32 < supported.u.range.minimum ||
            value.value32 > supported.u.range.maximum ||
            (value.value32 - supported.u.range.minimum) %
             supported.u.range.step != 0)
          {
            return -EINVAL;
          }

        break;
    }

  /* Only a validated value reaches the shadow register, so a read reports
   * exactly what a successful write accepted.
   */

  switch (id)
    {
      case IMGSENSOR_ID_GAMMA_CURVE:
        memcpy(priv->gamma_curve, value.p_u16, sizeof(priv->gamma_curve));
        break;

      case IMGSENSOR_ID_3A_PARAMETER:
        memcpy(priv->threea_parameter, value.p_u8,
               sizeof(priv->threea_parameter));
        break;

      case IMGSENSOR_ID_CLIP_VIDEO:
        memcpy(priv->clip_video, value.p_u32, sizeof(priv->clip_video));
        break;

      case IMGSENSOR_ID_CLIP_STILL:
        memcpy(priv->clip_still, value.p_u32, sizeof(priv->clip_still));
        break;

      default:
        index = video_fake_control_index(id);
        if (index < 0)
          {
            return -EINVAL;
          }

        priv->controls[index] = value.value32;
        break;
    }

  return OK;
}

static int video_fake_data_init(FAR struct imgdata_s *data)
{
  return OK;
}

static int video_fake_data_uninit(FAR struct imgdata_s *data)
{
  return video_fake_stop_capture(data);
}

static int video_fake_set_buffer(FAR struct imgdata_s *data,
                                 uint8_t nr_datafmts,
                                 FAR imgdata_format_t *datafmts,
                                 FAR uint8_t *addr, uint32_t size)
{
  FAR struct video_fake_s *priv = (FAR struct video_fake_s *)data;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  priv->next_buffer = addr;
  priv->buffer_size = addr != NULL ? size : 0;
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

static int video_fake_data_validate(FAR struct imgdata_s *data,
                                    uint8_t nr_datafmts,
                                    FAR imgdata_format_t *datafmts,
                                    FAR imgdata_interval_t *interval)
{
  FAR struct video_fake_s *priv = (FAR struct video_fake_s *)data;

  if (nr_datafmts == 0 || nr_datafmts > IMGDATA_FMT_MAX)
    {
      return -ENOTSUP;
    }

  if (nr_datafmts == 2 &&
      (datafmts[IMGDATA_FMT_MAIN].pixelformat !=
       IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG ||
       !video_fake_valid_subimage(datafmts[IMGDATA_FMT_MAIN].width,
                                  datafmts[IMGDATA_FMT_MAIN].height,
                                  datafmts[IMGDATA_FMT_SUB].width,
                                  datafmts[IMGDATA_FMT_SUB].height,
                                  datafmts[IMGDATA_FMT_SUB].pixelformat)))
    {
      return -EINVAL;
    }

  return video_fake_check(priv, datafmts[IMGDATA_FMT_MAIN].width,
                          datafmts[IMGDATA_FMT_MAIN].height,
                          datafmts[IMGDATA_FMT_MAIN].pixelformat,
                          interval->numerator, interval->denominator);
}

static int video_fake_start_capture(FAR struct imgdata_s *data,
                                    uint8_t nr_datafmts,
                                    FAR imgdata_format_t *datafmts,
                                    FAR imgdata_interval_t *interval,
                                    imgdata_capture_t callback,
                                    FAR void *arg)
{
  FAR struct video_fake_s *priv = (FAR struct video_fake_s *)data;
  FAR imgdata_format_t *format;
  uint32_t frame_bytes;
  irqstate_t flags;
  clock_t delay;
  int ret;

  if (nr_datafmts == 0 || nr_datafmts > IMGDATA_FMT_MAX)
    {
      return -ENOTSUP;
    }

  ret = video_fake_data_validate(data, nr_datafmts, datafmts, interval);
  if (ret < 0)
    {
      return ret;
    }

  format = &datafmts[IMGDATA_FMT_MAIN];
  ret = video_fake_check(priv, format->width, format->height,
                         format->pixelformat, interval->numerator,
                         interval->denominator);
  if (ret < 0)
    {
      return ret;
    }

  if (format->pixelformat == IMGDATA_PIX_FMT_JPEG ||
      format->pixelformat == IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG ||
      format->pixelformat == IMGDATA_PIX_FMT_ENTROPY ||
      format->pixelformat == IMGDATA_PIX_FMT_UYVY)
    {
      frame_bytes = format->width * format->height *
        ((format->pixelformat == IMGDATA_PIX_FMT_JPEG ||
          format->pixelformat == IMGDATA_PIX_FMT_ENTROPY) ? 1u : 2u);
    }
  else
    {
      ret = video_fake_frame_bytes(format->width, format->height,
                                   format->pixelformat, &frame_bytes);
    }

  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->running)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return -EBUSY;
    }

  priv->width = format->width;
  priv->height = format->height;
  priv->pixel_format = format->pixelformat;
  priv->frame_bytes = frame_bytes;

  priv->fps = interval->denominator / interval->numerator;
  priv->tick_num = USEC_PER_SEC;
  priv->tick_num *= interval->numerator;
  priv->tick_den = USEC_PER_TICK;
  priv->tick_den *= interval->denominator;
  priv->tick_acc = 0;
  priv->next_tick = clock_systime_ticks();
  priv->frame = 0;
  video_fake_size_panel(priv);

  video_fake_next_generation(priv);
  delay = video_fake_next_delay(priv);
  priv->callback = callback;
  priv->callback_arg = arg;
  priv->running = true;
  spin_unlock_irqrestore(&priv->lock, flags);

  ret = work_queue(LPWORK, &priv->work, video_fake_worker, priv, delay);
  if (ret < 0)
    {
      flags = spin_lock_irqsave(&priv->lock);
      priv->running = false;
      priv->callback = NULL;
      priv->callback_arg = NULL;
      priv->next_buffer = NULL;
      priv->buffer_size = 0;
      spin_unlock_irqrestore(&priv->lock, flags);
    }

  return ret;
}

static int video_fake_stop_capture(FAR struct imgdata_s *data)
{
  FAR struct video_fake_s *priv = (FAR struct video_fake_s *)data;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  priv->running = false;
  spin_unlock_irqrestore(&priv->lock, flags);

  work_cancel_sync(LPWORK, &priv->work);

  flags = spin_lock_irqsave(&priv->lock);
  priv->callback = NULL;
  priv->callback_arg = NULL;
  priv->next_buffer = NULL;
  priv->buffer_size = 0;
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

static int
video_fake_register_one(FAR const video_fake_params_t *params)
{
  struct video_fake_counts_s counts;
  FAR struct video_fake_s *priv;
  FAR struct imgsensor_s *sensor;
  int ret;

  ret = video_fake_validate_params(params, &counts);
  if (ret < 0)
    {
      return ret;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->background_row = kmm_malloc(counts.max_width * 2);
  priv->source_rows = kmm_malloc(counts.max_height);
  priv->source_columns = kmm_malloc(counts.max_width);
  if (priv->background_row == NULL || priv->source_rows == NULL ||
      priv->source_columns == NULL)
    {
      kmm_free(priv->background_row);
      kmm_free(priv->source_rows);
      kmm_free(priv->source_columns);
      kmm_free(priv);
      return -ENOMEM;
    }

  video_fake_make_name(priv, params->devpath);

  ret = video_fake_build_capabilities(priv, params, &counts);
  if (ret < 0)
    {
      kmm_free(priv->source_rows);
      kmm_free(priv->source_columns);
      kmm_free(priv->background_row);
      kmm_free(priv);
      return ret;
    }

  priv->data.ops = &g_video_fake_data_ops;
  priv->sensor.ops = &g_video_fake_sensor_ops;
  priv->sensor.fmtdescs_num = priv->formats_num;
  priv->sensor.fmtdescs = priv->fmtdescs;
  priv->sensor.frmsizes_num = priv->sizes_num;
  priv->sensor.frmsizes = priv->frmsizes;
  priv->sensor.frmintervals_num = priv->rates_num;
  priv->sensor.frmintervals = priv->frmintervals;
  video_fake_controls_init(priv);
  priv->generation = 1;
  spin_lock_init(&priv->lock);

  sensor = &priv->sensor;
  ret = capture_register(params->devpath, &priv->data, &sensor, 1);
  if (ret < 0)
    {
      kmm_free(priv->fmtdescs);
      kmm_free(priv->frmsizes);
      kmm_free(priv->frmintervals);
      kmm_free(priv->background_row);
      kmm_free(priv->source_rows);
      kmm_free(priv->source_columns);
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: video_fake_initialize
 *
 * Description:
 *   Register the fake video devices configured by
 *   CONFIG_VIDEO_FAKE_DEVICE_PARAMS.
 *
 * Returned Value:
 *   The number of devices registered, or a negated errno value if every
 *   device failed to register.
 *
 ****************************************************************************/

int video_fake_initialize(void)
{
  return video_fake_register(g_devparams, nitems(g_devparams));
}

/****************************************************************************
 * Name: video_fake_register
 *
 * Description:
 *   Register independent software-only V4L2 capture devices.
 *
 * Input Parameters:
 *   params  - Fake video parameter array.
 *   nparams - Number of entries in params.
 *
 * Returned Value:
 *   The number of devices registered, or a negated errno value if every
 *   device failed to register.
 *
 ****************************************************************************/

int video_fake_register(FAR const video_fake_params_t *params,
                        size_t nparams)
{
  int successful = 0;
  int last_error = -EINVAL;
  size_t i;

  if (params == NULL || nparams == 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < nparams; i++)
    {
      int ret = video_fake_register_one(&params[i]);

      if (ret < 0)
        {
          FAR const char *devpath = params[i].devpath != NULL ?
                                    params[i].devpath : "(null)";

          vwarn("WARNING: Failed to register %s: %d\n", devpath, ret);
          last_error = ret;
        }
      else
        {
          successful++;
        }
    }

  return successful > 0 ? successful : last_error;
}
