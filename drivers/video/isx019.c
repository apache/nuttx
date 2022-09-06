/****************************************************************************
 * drivers/video/isx019.c
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
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/signal.h>
#include <arch/board/board.h>
#include <nuttx/video/isx019.h>
#include <nuttx/video/imgsensor.h>
#include <math.h>
#include <nuttx/mutex.h>

#include "isx019_reg.h"
#include "isx019_range.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Wait time on power on sequence. */

#define TRANSITION_TIME_TO_STARTUP   (120 * USEC_PER_MSEC) /* unit : usec */
#define TRANSITION_TIME_TO_STREAMING (30 * USEC_PER_MSEC)  /* unit : usec */

/* For get_supported_value() I/F */

#define SET_RANGE(range, min, max, s, def) \
        do                                 \
          {                                \
            (range).minimum       = (min); \
            (range).maximum       = (max); \
            (range).step          = (s);   \
            (range).default_value = (def); \
          }                                \
        while (0);

#define SET_DISCRETE(disc, nr, val, def)   \
        do                                 \
          {                                \
            (disc).nr_values     = (nr);   \
            (disc).values        = (val);  \
            (disc).default_value = (def);  \
          }                                \
        while (0);

#define SET_ELEMS(elem, nr, min, max, s)   \
        do                                 \
          {                                \
            (elem).nr_elems      = (nr);   \
            (elem).minimum       = (min);  \
            (elem).maximum       = (max);  \
            (elem).step          = (s);    \
          }                                \
        while (0);

#define COMPARE_FRAMESIZE(w, h, sup_w, sup_h)  (((w) == (sup_w)) && \
                                                ((h) == (sup_h)))

#define VALIDATE_FRAMESIZE(w, h) (COMPARE_FRAMESIZE((w), (h), 1280, 960) || \
                                  COMPARE_FRAMESIZE((w), (h), 1280, 720) || \
                                  COMPARE_FRAMESIZE((w), (h),  640, 480) || \
                                  COMPARE_FRAMESIZE((w), (h),  640, 360) || \
                                  COMPARE_FRAMESIZE((w), (h),  480, 360) || \
                                  COMPARE_FRAMESIZE((w), (h),  320, 240) || \
                                  COMPARE_FRAMESIZE((w), (h),  160, 120))

#define VALIDATE_THUMNAIL_SIZE(m, s) (((s) != 0) && \
                                      ((m) % (s) == 0) && \
                                      ((m) / (s) < 5)  && \
                                      ((m) / (s) > 0))

/* For set_value() and get_value() I/F */

#define SET_REGINFO(a, c, o, s) do \
                                  {                      \
                                    (a)->category = (c); \
                                    (a)->offset   = (o); \
                                    (a)->size     = (s); \
                                  }                      \
                                while (0);

#define VALIDATE_RANGE(v, min, max, step) (((v) >= (min)) && \
                                           ((v) <= (max)) && \
                                           (((v) - (min)) % (step) == 0))

/* Offset for IMGSENSOR_ID_3A_PARAMETER control */

#define OFFSET_3APARAMETER_AWB_R      (0)
#define OFFSET_3APARAMETER_AWB_B      (2)
#define OFFSET_3APARAMETER_AE_SHTTIME (4)
#define OFFSET_3APARAMETER_AE_GAIN    (8)

/* Index of array for drive mode setting */

#define INDEX_SENS     (0)
#define INDEX_POST     (1)
#define INDEX_SENSPOST (2)
#define INDEX_IO       (3)

/* Timer value for power on control */

#define ISX019_ACCESSIBLE_WAIT_SEC    (0)
#define ISX019_ACCESSIBLE_WAIT_USEC   (200000)
#define FPGA_ACCESSIBLE_WAIT_SEC      (1)
#define FPGA_ACCESSIBLE_WAIT_USEC     (0)

/* Array size of DQT setting for JPEG quality */

#define JPEG_DQT_ARRAY_SIZE (64)

/* ISX019 standard master clock */

#define ISX019_STANDARD_MASTER_CLOCK (27000000)

/* Vivid colors setting */

#define VIVID_COLORS_SATURATION (0xf0)
#define VIVID_COLORS_SHARPNESS  (0x20)

/* Black white colors setting */

#define BW_COLORS_SATURATION (0x00)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct isx019_default_value_s
{
  int32_t brightness;
  int32_t contrast;
  int32_t saturation;
  int32_t hue;
  int32_t awb;
  int32_t gamma;
  int32_t ev;
  int32_t hflip_video;
  int32_t vflip_video;
  int32_t hflip_still;
  int32_t vflip_still;
  int32_t sharpness;
  int32_t ae;
  int32_t exptime;
  int32_t wbmode;
  int32_t hdr;
  int32_t iso;
  int32_t iso_auto;
  int32_t meter;
  int32_t threealock;
  int32_t threeastatus;
  int32_t jpgquality;
};

typedef struct isx019_default_value_s isx019_default_value_t;

struct isx019_rect_s
{
  int32_t left;
  int32_t top;
  uint32_t width;
  uint32_t height;
};

typedef struct isx019_rect_s isx019_rect_t;

struct isx019_dev_s
{
  mutex_t fpga_lock;
  mutex_t i2c_lock;
  FAR struct i2c_master_s *i2c;
  float clock_ratio;
  isx019_default_value_t  default_value;
  imgsensor_stream_type_t stream;
  imgsensor_white_balance_t wb_mode;
  uint8_t flip_video;
  uint8_t flip_still;
  isx019_rect_t clip_video;
  isx019_rect_t clip_still;
  int32_t iso;
  double  gamma;
  int32_t jpg_quality;
  imgsensor_colorfx_t colorfx;
};

typedef struct isx019_dev_s isx019_dev_t;

typedef CODE int32_t (*convert_t)(int32_t value32);

typedef CODE int (*setvalue_t)(imgsensor_value_t value);
typedef CODE int (*getvalue_t)(FAR imgsensor_value_t *value);

struct isx019_reginfo_s
{
  uint16_t category;
  uint16_t offset;
  uint8_t  size;
};

typedef struct isx019_reginfo_s isx019_reginfo_t;

struct isx019_fpga_jpg_quality_s
{
  /* JPEG quality */

  int quality;

  /* DQT header setting for y component */

  uint8_t y_head[JPEG_DQT_ARRAY_SIZE];

  /* DQT calculation data for y component */

  uint8_t y_calc[JPEG_DQT_ARRAY_SIZE];

  /* DQT header setting for c component */

  uint8_t c_head[JPEG_DQT_ARRAY_SIZE];

  /* DQT calculation data for c component */

  uint8_t c_calc[JPEG_DQT_ARRAY_SIZE];
};

typedef struct isx019_fpga_jpg_quality_s isx019_fpga_jpg_quality_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool isx019_is_available(void);
static int isx019_init(void);
static int isx019_uninit(void);
static FAR const char *isx019_get_driver_name(void);
static int isx019_validate_frame_setting(imgsensor_stream_type_t type,
                                         uint8_t nr_datafmt,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval);
static int isx019_start_capture(imgsensor_stream_type_t type,
                                uint8_t nr_datafmt,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval);
static int isx019_stop_capture(imgsensor_stream_type_t type);
static int isx019_get_supported_value(uint32_t id,
                                     FAR imgsensor_supported_value_t *value);
static int isx019_get_value(uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value);
static int isx019_set_value(uint32_t id, uint32_t size,
                            imgsensor_value_t value);
static int initialize_jpg_quality(void);
static int send_read_cmd(FAR struct i2c_config_s *config,
                         uint8_t cat,
                         uint16_t addr,
                         uint8_t size);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static isx019_dev_t g_isx019_private =
{
  NXMUTEX_INITIALIZER,
  NXMUTEX_INITIALIZER,
};

static struct imgsensor_ops_s g_isx019_ops =
{
  isx019_is_available,
  isx019_init,
  isx019_uninit,
  isx019_get_driver_name,
  isx019_validate_frame_setting,
  isx019_start_capture,
  isx019_stop_capture,
  isx019_get_supported_value,
  isx019_get_value,
  isx019_set_value,
};

static isx019_fpga_jpg_quality_t g_isx019_jpg_quality[] =
{
  {
    10,
      {
         21,  15,  15,  26,  18,  26,  43,  21,
         21,  43,  43,  43,  32,  43,  43,  43,
         43,  43,  43,  43,  43,  64,  43,  43,
         43,  43,  43,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          3,   4, 133, 131, 131, 131,   1,   1,
          4, 135,   3, 131, 131, 131,   1,   1,
        133,   3,   2, 131, 131,   1,   1,   1,
        131, 131, 131, 131,   1,   1,   1,   1,
        131, 131, 131,   1,   1,   1,   1,   1,
        131, 131,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
      },
      {
         21,  26,  26,  32,  26,  32,  43,  26,
         26,  43,  64,  43,  32,  43,  64,  64,
         64,  43,  43,  64,  64,  64,  64,  64,
         43,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          3, 133,   2, 131,   1,   1,   1,   1,
        133, 133, 133, 131,   1,   1,   1,   1,
          2, 133,   2, 131,   1,   1,   1,   1,
        131, 131, 131, 131,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
      }
  },
  {
    20,
      {
         18,  14,  14,  14,  15,  14,  21,  15,
         15,  21,  32,  21,  18,  21,  32,  32,
         26,  21,  21,  26,  32,  32,  26,  26,
         26,  26,  26,  32,  43,  32,  32,  32,
         32,  32,  32,  43,  43,  43,  43,  43,
         43,  43,  43,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
        135, 137, 137,   3,   2,   2,   2, 131,
        137,   4,   4,   3, 133, 133,   2, 131,
        137,   4, 135,   3, 133,   2, 131,   1,
          3,   3,   3, 133,   2, 131,   1,   1,
          2, 133, 133,   2, 131,   1,   1,   1,
          2, 133,   2, 131,   1,   1,   1,   1,
          2,   2, 131,   1,   1,   1,   1,   1,
        131, 131,   1,   1,   1,   1,   1,   1,
      },
      {
         21,  21,  21,  21,  26,  21,  26,  21,
         21,  26,  26,  21,  26,  21,  26,  32,
         26,  26,  26,  26,  32,  43,  32,  32,
         32,  32,  32,  43,  64,  43,  43,  43,
         43,  43,  43,  64,  64,  64,  43,  43,
         43,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          3,   3,   3, 133, 133,   2, 131,   1,
          3, 133,   3,   3, 133,   2, 131,   1,
          3,   3, 133, 133,   2, 131,   1,   1,
        133,   3, 133,   2, 131, 131,   1,   1,
        133, 133,   2, 131, 131,   1,   1,   1,
          2,   2, 131, 131,   1,   1,   1,   1,
        131, 131,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
      }
  },
  {
    30,
      {
         15,  11,  11,  11,  12,  11,  15,  12,
         12,  15,  21,  15,  13,  15,  21,  26,
         21,  15,  15,  21,  26,  32,  21,  21,
         21,  21,  21,  32,  32,  21,  26,  26,
         26,  26,  21,  32,  32,  32,  32,  43,
         32,  32,  32,  43,  43,  43,  43,  43,
         43,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          4,  12,  12,   4,   3, 133,   2,   2,
         12, 139, 139,   4,   3,   3,   3,   2,
         12, 139,   5,   4,   3, 133,   2, 131,
          4,   4,   4,   3, 133,   2, 131,   1,
          3,   3,   3, 133, 131, 131,   1,   1,
        133,   3, 133,   2, 131,   1,   1,   1,
          2,   3,   2, 131,   1,   1,   1,   1,
          2,   2, 131,   1,   1,   1,   1,   1,
      },
      {
         18,  15,  15,  18,  18,  18,  21,  18,
         18,  21,  21,  18,  21,  18,  21,  26,
         21,  21,  21,  21,  26,  43,  26,  26,
         26,  26,  26,  43,  43,  32,  32,  32,
         32,  32,  32,  43,  43,  43,  43,  43,
         43,  43,  43,  43,  43,  43,  43,  43,
         43,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
        135,   4, 135,   3,   3, 133, 131, 131,
          4, 135, 135, 135,   3, 133,   2, 131,
        135, 135,   3,   3, 133,   2, 131, 131,
          3, 135,   3, 133,   2, 131, 131,   1,
          3,   3, 133,   2, 131, 131,   1,   1,
        133, 133,   2, 131, 131,   1,   1,   1,
        131,   2, 131, 131,   1,   1,   1,   1,
        131, 131, 131,   1,   1,   1,   1,   1,
      }
  },
  {
    40,
      {
         12,   8,   8,   8,   9,   8,  12,   9,
          9,  12,  18,  11,  10,  11,  18,  21,
         15,  12,  12,  15,  21,  26,  18,  18,
         21,  18,  18,  26,  21,  18,  21,  21,
         21,  21,  18,  21,  21,  26,  26,  32,
         26,  26,  21,  32,  32,  43,  43,  32,
         32,  43,  43,  43,  43,  43,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
        139,   8,   8, 139, 135,   3, 133,   3,
          8,   7,   7,  12,   4, 135, 135,   3,
          8,   7, 141, 139, 135,   3, 133,   2,
        139,  12, 139,   3,   3, 133,   2, 131,
        135,   4, 135,   3,   2, 131, 131,   1,
          3, 135,   3, 133, 131, 131,   1,   1,
        133, 135, 133,   2, 131,   1,   1,   1,
          3,   3,   2, 131,   1,   1,   1,   1,
      },
      {
         13,  11,  11,  13,  14,  13,  15,  14,
         14,  15,  21,  14,  15,  14,  21,  21,
         15,  18,  18,  15,  21,  26,  21,  21,
         21,  21,  21,  26,  32,  26,  21,  21,
         21,  21,  26,  32,  32,  32,  32,  32,
         32,  32,  32,  43,  43,  32,  32,  43,
         43,  43,  43,  43,  43,  43,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          5,  12,   5,   4,   3,   3, 133,   2,
         12, 137, 137, 137,   4,   3, 133,   2,
          5, 137,   4, 135,   3,   3,   2, 131,
          4, 137, 135,   3,   3,   2, 131, 131,
          3,   4,   3,   3,   2,   2, 131,   1,
          3,   3,   3,   2,   2, 131,   1,   1,
        133, 133,   2, 131, 131,   1,   1,   1,
          2,   2, 131, 131,   1,   1,   1,   1,
      }
  },
  {
    50,
      {
          8,   6,   6,   6,   6,   6,   8,   6,
          6,   8,  12,   8,   7,   8,  12,  14,
         10,   8,   8,  10,  14,  15,  13,  13,
         14,  13,  13,  15,  18,  12,  14,  13,
         13,  14,  12,  18,  15,  18,  18,  21,
         18,  18,  15,  26,  26,  26,  26,  26,
         26,  32,  32,  32,  32,  32,  43,  43,
         43,  43,  43,  43,  43,  43,  43,  43,
      },
      {
          8,  11,  11,   8, 139, 137,   4, 135,
         11,  11,  11,   8, 141,   5, 139,   4,
         11,  11,   9,   8,   5, 137, 135, 133,
          8,   8,   8, 137,   5, 135, 133,   2,
        139, 141,   5,   5,   3, 133,   2, 131,
        137,   5, 137, 135, 133,   2, 131, 131,
          4, 139, 135, 133,   2, 131, 131, 131,
        135,   4, 133,   2, 131, 131, 131, 131,
      },
      {
          9,   8,   8,   9,  10,   9,  11,   9,
          9,  11,  14,  11,  13,  11,  14,  18,
         14,  14,  14,  14,  18,  18,  13,  13,
         14,  13,  13,  18,  26,  18,  15,  15,
         15,  15,  18,  26,  21,  21,  21,  21,
         21,  21,  21,  26,  26,  26,  26,  26,
         26,  32,  32,  32,  32,  32,  43,  43,
         43,  43,  43,  43,  43,  43,  43,  43,
      },
      {
          7,   8,   7,  12, 137, 135, 135, 133,
          8, 141,   7,  12, 137,   5, 135,   3,
          7,   7,   5, 137,   5,   4,   3, 133,
         12,  12, 137, 137,   4,   3, 133,   2,
        137, 137,   5,   4,   3, 133,   2, 131,
        135,   5,   4,   3, 133,   2, 131, 131,
        135, 135,   3, 133,   2, 131, 131, 131,
        133,   3, 133,   2, 131, 131, 131, 131,
      }
  },
  {
    60,
      {
          6,   4,   4,   4,   5,   4,   6,   5,
          5,   6,   9,   6,   5,   6,   9,  11,
          8,   6,   6,   8,  11,  12,  10,  10,
         11,  10,  10,  12,  15,  12,  12,  12,
         12,  12,  12,  15,  12,  14,  15,  15,
         15,  14,  12,  18,  18,  21,  21,  18,
         18,  26,  26,  26,  26,  26,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
      },
      {
         11,  16,  16,  11,   7,  12, 139,   4,
         16,  13,  13,  11,   8, 141, 139, 139,
         16,  13,  13,  11, 141, 139, 137, 135,
         11,  11,  11,  12, 139,   4, 135, 133,
          7,   8, 141, 139,   4,   3, 133,   2,
         12, 141, 139,   4,   3, 133,   2,   2,
        139, 139, 137, 135, 133,   2,   2,   2,
          4, 139, 135, 133,   2,   2,   2,   2,
      },
      {
          7,   7,   7,  13,  12,  13,  26,  15,
         15,  26,  26,  21,  18,  21,  26,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
      },
      {
          9,   9,   5, 133, 133,   2,   2,   2,
          9, 139,   4,   3,   2,   2,   2,   2,
          5,   4, 135,   2,   2,   2,   2,   2,
        133,   3,   2,   2,   2,   2,   2,   2,
        133,   2,   2,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   2,   2,   2,   2,
      }
  },
  {
    70,
      {
          4,   3,   3,   3,   3,   3,   4,   3,
          3,   4,   6,   4,   3,   4,   6,   7,
          5,   4,   4,   5,   7,   8,   6,   6,
          7,   6,   6,   8,  10,   8,   9,   9,
          9,   9,   8,  10,  10,  12,  12,  12,
         12,  12,  10,  12,  12,  13,  13,  12,
         12,  18,  18,  18,  18,  18,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
      },
      {
         16,  21,  21,  16,  11,   9,   8, 141,
         21,  21,  21,  16,  13,  11,   8, 141,
         21,  21,  21,  16,  11,   7, 139, 139,
         16,  16,  16,   9,   7, 139, 139, 135,
         11,  13,  11,   7, 139,   5, 135,   3,
          9,  11,   7, 139,   5, 135,   3,   3,
          8,   8, 139, 139, 135,   3,   3,   3,
        141, 141, 139, 135,   3,   3,   3,   3,
      },
      {
          4,   5,   5,   8,   7,   8,  15,  10,
         10,  15,  21,  14,  14,  14,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
      },
      {
         16,  13,   8,   4,   3,   3,   3,   3,
         13,   9, 141, 137,   3,   3,   3,   3,
          8, 141, 137,   3,   3,   3,   3,   3,
          4, 137,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
      }
  },
  {
    80,
      {
          2,   2,   2,   2,   2,   2,   2,   2,
          2,   2,   3,   2,   2,   2,   3,   4,
          3,   2,   2,   3,   4,   5,   4,   4,
          4,   4,   4,   5,   6,   5,   5,   5,
          5,   5,   5,   6,   6,   7,   7,   8,
          7,   7,   6,   9,   9,  10,  10,   9,
          9,  12,  12,  12,  12,  12,  12,  12,
         12,  12,  12,  12,  12,  12,  12,  12,
      },
      {
         32,  32,  32,  32,  21,  16,  13,  11,
         32,  32,  32,  32,  21,  16,  13,  11,
         32,  32,  32,  32,  16,  13,   9,   7,
         32,  32,  32,  16,  13,   9,   7, 139,
         21,  21,  16,  13,   8, 141, 139, 139,
         16,  16,  13,   9, 141, 139, 139, 139,
         13,  13,   9,   7, 139, 139, 139, 139,
         11,  11,   7, 139, 139, 139, 139, 139,
      },
      {
          3,   3,   3,   5,   4,   5,   9,   6,
          6,   9,  13,  11,   9,  11,  13,  15,
         14,  14,  14,  14,  15,  15,  12,  12,
         12,  12,  12,  15,  15,  12,  12,  12,
         12,  12,  12,  15,  12,  12,  12,  12,
         12,  12,  12,  12,  12,  12,  12,  12,
         12,  12,  12,  12,  12,  12,  12,  12,
         12,  12,  12,  12,  12,  12,  12,  12,
      },
      {
         21,  21,  13,   7,   5,   4,   4,   4,
         21,  16,  11,  12, 137, 139, 139, 139,
         13,  11,   7, 137, 139, 139, 139, 139,
          7,  12, 137, 139, 139, 139, 139, 139,
          5, 137, 139, 139, 139, 139, 139, 139,
          4, 139, 139, 139, 139, 139, 139, 139,
          4, 139, 139, 139, 139, 139, 139, 139,
          4, 139, 139, 139, 139, 139, 139, 139,
      }
  },
  {
    90,
      {
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   2,   1,   1,   1,   2,   2,
          2,   1,   1,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   3,   2,   3,   3,
          3,   3,   2,   3,   3,   4,   4,   4,
          4,   4,   3,   5,   5,   5,   5,   5,
          5,   7,   7,   7,   7,   7,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
      },
      {
         64,  64,  64,  64,  32,  32,  32,  21,
         64,  64,  64,  64,  32,  32,  32,  21,
         64,  64,  64,  64,  32,  21,  16,  13,
         64,  64,  64,  32,  21,  16,  13,   9,
         32,  32,  32,  21,  16,  13,   9,   8,
         32,  32,  21,  16,  13,   9,   8,   8,
         32,  32,  16,  13,   9,   8,   8,   8,
         21,  21,  13,   9,   8,   8,   8,   8,
      },
      {
          1,   1,   1,   2,   2,   2,   5,   3,
          3,   5,   7,   5,   4,   5,   7,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
      },
      {
         64,  64,  32,  13,   9,   8,   8,   8,
         64,  32,  21,  13,   8,   8,   8,   8,
         32,  21,  16,   8,   8,   8,   8,   8,
         13,  13,   8,   8,   8,   8,   8,   8,
          9,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
      }
  },
  {
    100,
      {
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   2,   2,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
      },
      {
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  32,
         64,  64,  64,  64,  64,  64,  32,  32,
         64,  64,  64,  64,  64,  32,  32,  21,
         64,  64,  64,  64,  32,  32,  21,  21,
         64,  64,  64,  32,  32,  21,  21,  21,
         64,  64,  32,  32,  21,  21,  21,  21,
      },
      {
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   2,   2,   1,   2,   2,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
      },
      {
         64,  64,  64,  64,  32,  21,  21,  21,
         64,  64,  64,  32,  21,  21,  21,  21,
         64,  64,  64,  21,  21,  21,  21,  21,
         64,  32,  21,  21,  21,  21,  21,  21,
         32,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
      }
  }
};

#define NR_JPGSETTING_TBL \
        (sizeof(g_isx019_jpg_quality) / sizeof(isx019_fpga_jpg_quality_t))

int32_t g_isx019_colorfx[] =
{
  IMGSENSOR_COLORFX_NONE,
  IMGSENSOR_COLORFX_BW,
  IMGSENSOR_COLORFX_VIVID,
};

#define NR_COLORFX (sizeof(g_isx019_colorfx) / sizeof(int32_t))

int32_t g_isx019_wbmode[] =
{
  IMGSENSOR_WHITE_BALANCE_AUTO,
  IMGSENSOR_WHITE_BALANCE_INCANDESCENT,
  IMGSENSOR_WHITE_BALANCE_FLUORESCENT,
  IMGSENSOR_WHITE_BALANCE_DAYLIGHT,
  IMGSENSOR_WHITE_BALANCE_CLOUDY,
  IMGSENSOR_WHITE_BALANCE_SHADE,
};

#define NR_WBMODE (sizeof(g_isx019_wbmode) / sizeof(int32_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int fpga_i2c_write(uint8_t addr, FAR uint8_t *data, uint8_t size)
{
  struct i2c_config_s config;
  static uint8_t buf[FPGA_I2C_REGSIZE_MAX + FPGA_I2C_REGADDR_LEN];
  int ret;

  DEBUGASSERT(size <= FPGA_I2C_REGSIZE_MAX);

  config.frequency = ISX019_I2C_FREQUENCY;
  config.address   = ISX019_I2C_SLVADDR;
  config.addrlen   = ISX019_I2C_SLVADDR_LEN;

  nxmutex_lock(&g_isx019_private.i2c_lock);

  /* ISX019 requires that send read command to ISX019 before FPGA access. */

  send_read_cmd(&config, CAT_VERSION, ROM_VERSION, 1);

  config.frequency = FPGA_I2C_FREQUENCY;
  config.address   = FPGA_I2C_SLVADDR;
  config.addrlen   = FPGA_I2C_SLVADDR_LEN;

  buf[FPGA_I2C_OFFSET_ADDR] = addr;
  memcpy(&buf[FPGA_I2C_OFFSET_WRITEDATA], data, size);
  ret = i2c_write(g_isx019_private.i2c,
                  &config,
                  buf,
                  size + FPGA_I2C_REGADDR_LEN);
  nxmutex_unlock(&g_isx019_private.i2c_lock);

  return ret;
}

static int fpga_i2c_read(uint8_t addr, FAR uint8_t *data, uint8_t size)
{
  int ret;
  struct i2c_config_s config;

  DEBUGASSERT(size <= FPGA_I2C_REGSIZE_MAX);

  config.frequency = ISX019_I2C_FREQUENCY;
  config.address   = ISX019_I2C_SLVADDR;
  config.addrlen   = ISX019_I2C_SLVADDR_LEN;

  nxmutex_lock(&g_isx019_private.i2c_lock);

  /* ISX019 requires that send read command to ISX019 before FPGA access. */

  send_read_cmd(&config, CAT_VERSION, ROM_VERSION, 1);

  config.frequency = FPGA_I2C_FREQUENCY;
  config.address   = FPGA_I2C_SLVADDR;
  config.addrlen   = FPGA_I2C_SLVADDR_LEN;

  ret = i2c_write(g_isx019_private.i2c,
                  &config,
                  &addr,
                  FPGA_I2C_REGADDR_LEN);
  if (ret >= 0)
    {
      ret = i2c_read(g_isx019_private.i2c, &config, data, size);
    }

  nxmutex_unlock(&g_isx019_private.i2c_lock);
  return ret;
}

static void fpga_activate_setting(void)
{
  uint8_t regval = FPGA_ACTIVATE_REQUEST;
  fpga_i2c_write(FPGA_ACTIVATE, &regval, 1);

  while (1)
    {
      fpga_i2c_read(FPGA_ACTIVATE, &regval, 1);
      if (regval == 0)
        {
          break;
        }
    }
}

static uint8_t calc_isx019_chksum(FAR uint8_t *data, uint8_t len)
{
  int i;
  uint8_t chksum = 0;

  for (i = 0; i < len; i++)
    {
      /* ISX019 checksum is lower 8bit of addition result.
       * So, no problem even if overflow occur.
       */

      chksum += data[i];
    }

  return chksum;
}

static bool validate_isx019_chksum(FAR uint8_t *data, uint8_t len)
{
  uint8_t chksum;

  chksum = calc_isx019_chksum(data, len - 1);

  return (data[len - 1] == chksum);
}

static int recv_write_response(FAR struct i2c_config_s *config)
{
  int ret;
  uint8_t buf[ISX019_I2C_WRRES_TOTALLEN];

  ret = i2c_read(g_isx019_private.i2c, config, buf, sizeof(buf));
  if (ret < 0)
    {
      return ret;
    }

  if ((buf[ISX019_I2C_OFFSET_TOTALLEN] != ISX019_I2C_WRRES_TOTALLEN) ||
      (buf[ISX019_I2C_OFFSET_CMDNUM]   != 1) ||
      (buf[ISX019_I2C_OFFSET_CMDLEN]   != ISX019_I2C_WRRES_LEN) ||
      (buf[ISX019_I2C_OFFSET_STS]      != ISX019_I2C_STS_OK) ||
      !validate_isx019_chksum(buf, ISX019_I2C_WRRES_TOTALLEN))
    {
      return -EPROTO;
    }

  return OK;
}

static int recv_read_response(FAR struct i2c_config_s *config,
                              FAR uint8_t *data,
                              uint8_t size)
{
  int ret;
  uint8_t buf[ISX019_I2C_WRREQ_TOTALLEN(ISX019_I2C_REGSIZE_MAX)] =
    {
      0
    };

  DEBUGASSERT(size <= ISX019_I2C_REGSIZE_MAX);

  ret = i2c_read(g_isx019_private.i2c,
                 config, buf, ISX019_I2C_RDRES_TOTALLEN(size));
  if (ret < 0)
    {
      return ret;
    }

  if ((buf[ISX019_I2C_OFFSET_TOTALLEN] != ISX019_I2C_RDRES_TOTALLEN(size)) ||
      (buf[ISX019_I2C_OFFSET_CMDNUM]   != 1) ||
      (buf[ISX019_I2C_OFFSET_CMDLEN]   != ISX019_I2C_RDRES_LEN(size)) ||
      (buf[ISX019_I2C_OFFSET_STS]      != ISX019_I2C_STS_OK) ||
      !validate_isx019_chksum(buf, ISX019_I2C_RDRES_TOTALLEN(size)))
    {
      return -EPROTO;
    }

  memcpy(data, &buf[ISX019_I2C_OFFSET_READDATA], size);

  return OK;
}

static int send_write_cmd(FAR struct i2c_config_s *config,
                          uint8_t cat,
                          uint16_t addr,
                          FAR uint8_t *data,
                          uint8_t size)
{
  int len;
  uint8_t buf[ISX019_I2C_WRREQ_TOTALLEN(ISX019_I2C_REGSIZE_MAX)] =
    {
      0
    };

  DEBUGASSERT(size <= ISX019_I2C_REGSIZE_MAX);

  buf[ISX019_I2C_OFFSET_TOTALLEN]  = ISX019_I2C_WRREQ_TOTALLEN(size);
  buf[ISX019_I2C_OFFSET_CMDNUM]    = 1;
  buf[ISX019_I2C_OFFSET_CMDLEN]    = ISX019_I2C_WRREQ_LEN(size);
  buf[ISX019_I2C_OFFSET_CMD]       = ISX019_I2C_CMD_WRITE;
  buf[ISX019_I2C_OFFSET_CATEGORY]  = cat;
  buf[ISX019_I2C_OFFSET_ADDRESS_H] = addr >> 8;
  buf[ISX019_I2C_OFFSET_ADDRESS_L] = addr & 0xff;

  memcpy(&buf[ISX019_I2C_OFFSET_WRITEDATA], data, size);

  len = ISX019_I2C_OFFSET_WRITEDATA + size;
  buf[len] = calc_isx019_chksum(buf, len);
  len++;

  return i2c_write(g_isx019_private.i2c, config, buf, len);
}

static int isx019_i2c_write(uint8_t cat,
                            uint16_t addr,
                            FAR uint8_t *data,
                            uint8_t size)
{
  int ret;
  struct i2c_config_s config;

  DEBUGASSERT(size <= ISX019_I2C_REGSIZE_MAX);

  config.frequency = ISX019_I2C_FREQUENCY;
  config.address   = ISX019_I2C_SLVADDR;
  config.addrlen   = ISX019_I2C_SLVADDR_LEN;

  nxmutex_lock(&g_isx019_private.i2c_lock);

  ret = send_write_cmd(&config, cat, addr, data, size);
  if (ret == OK)
    {
      ret = recv_write_response(&config);
    }

  nxmutex_unlock(&g_isx019_private.i2c_lock);
  return ret;
}

static int send_read_cmd(FAR struct i2c_config_s *config,
                         uint8_t cat,
                         uint16_t addr,
                         uint8_t size)
{
  int ret;
  int len;
  uint8_t buf[ISX019_I2C_RDREQ_TOTALLEN] =
    {
      0
    };

  buf[ISX019_I2C_OFFSET_TOTALLEN]  = ISX019_I2C_RDREQ_TOTALLEN;
  buf[ISX019_I2C_OFFSET_CMDNUM]    = 1;
  buf[ISX019_I2C_OFFSET_CMDLEN]    = ISX019_I2C_RDREQ_LEN;
  buf[ISX019_I2C_OFFSET_CMD]       = ISX019_I2C_CMD_READ;
  buf[ISX019_I2C_OFFSET_CATEGORY]  = cat;
  buf[ISX019_I2C_OFFSET_ADDRESS_H] = addr >> 8;
  buf[ISX019_I2C_OFFSET_ADDRESS_L] = addr & 0xff;
  buf[ISX019_I2C_OFFSET_READSIZE]  = size;

  len = ISX019_I2C_OFFSET_READSIZE + 1;
  buf[len] = calc_isx019_chksum(buf, len);
  len++;

  ret = i2c_write(g_isx019_private.i2c, config, buf, len);
  return ret;
}

static int isx019_i2c_read(uint8_t cat,
                           uint16_t addr,
                           FAR uint8_t *data,
                           uint8_t size)
{
  int ret;
  struct i2c_config_s config;

  DEBUGASSERT(size <= ISX019_I2C_REGSIZE_MAX);

  config.frequency = ISX019_I2C_FREQUENCY;
  config.address   = ISX019_I2C_SLVADDR;
  config.addrlen   = ISX019_I2C_SLVADDR_LEN;

  nxmutex_lock(&g_isx019_private.i2c_lock);

  ret = send_read_cmd(&config, cat, addr, size);
  if (ret == OK)
    {
      ret = recv_read_response(&config, data, size);
    }

  nxmutex_unlock(&g_isx019_private.i2c_lock);
  return ret;
}

static void fpga_init(void)
{
  uint8_t regval;

  regval = FPGA_RESET_ENABLE;
  fpga_i2c_write(FPGA_RESET, &regval, 1);
  regval = FPGA_DATA_OUTPUT_STOP;
  fpga_i2c_write(FPGA_DATA_OUTPUT, &regval, 1);
  fpga_activate_setting();
  regval = FPGA_RESET_RELEASE;
  fpga_i2c_write(FPGA_RESET, &regval, 1);
  fpga_activate_setting();
}

static int set_drive_mode(void)
{
  uint8_t drv[] =
    {
#ifdef CONFIG_VIDEO_ISX019_DOL2
      DOL2_30FPS_SENS, DOL2_30FPS_POST, DOL2_30FPS_SENSPOST, DOL2_30FPS_IO
#else
      DOL3_30FPS_SENS, DOL3_30FPS_POST, DOL3_30FPS_SENSPOST, DOL3_30FPS_IO
#endif
    };

  nxsig_usleep(TRANSITION_TIME_TO_STARTUP);

  isx019_i2c_write(CAT_CONFIG, MODE_SENSSEL,      &drv[INDEX_SENS], 1);
  isx019_i2c_write(CAT_CONFIG, MODE_POSTSEL,      &drv[INDEX_POST], 1);
  isx019_i2c_write(CAT_CONFIG, MODE_SENSPOST_SEL, &drv[INDEX_SENSPOST], 1);

  nxsig_usleep(TRANSITION_TIME_TO_STREAMING);

  return OK;
}

static bool try_repeat(int sec, int usec, CODE int (*trial_func)(void))
{
  int ret;
  struct timeval start;
  struct timeval now;
  struct timeval delta;
  struct timeval wait;

  wait.tv_sec = sec;
  wait.tv_usec = usec;

  gettimeofday(&start, NULL);
  while (1)
    {
      ret = trial_func();
      if (ret != -ENODEV)
        {
          break;
        }
      else
        {
          gettimeofday(&now, NULL);
          timersub(&now, &start, &delta);
          if (timercmp(&delta, &wait, >))
            {
              break;
            }
        }
    };

  return (ret == OK);
}

static int try_isx019_i2c(void)
{
  uint8_t buf;
  return isx019_i2c_read(CAT_SYSCOM, DEVSTS, &buf, 1);
}

static int try_fpga_i2c(void)
{
  uint8_t buf;
  return fpga_i2c_read(FPGA_VERSION, &buf, 1);
}

static void power_on(void)
{
  g_isx019_private.i2c = board_isx019_initialize();
  board_isx019_power_on();
  board_isx019_release_reset();
}

static void power_off(void)
{
  board_isx019_set_reset();
  board_isx019_power_off();
  board_isx019_uninitialize(g_isx019_private.i2c);
}

static bool isx019_is_available(void)
{
  bool ret;

  power_on();

  /* Try to access via I2C
   * about both ISX019 image sensor and FPGA.
   */

  ret = false;
  if (try_repeat(ISX019_ACCESSIBLE_WAIT_SEC,
                ISX019_ACCESSIBLE_WAIT_USEC,
                try_isx019_i2c))
    {
      if (try_repeat(FPGA_ACCESSIBLE_WAIT_SEC,
                     FPGA_ACCESSIBLE_WAIT_USEC,
                     try_fpga_i2c))
        {
          ret = true;
        }
    }

  power_off();

  return ret;
}

static int32_t get_value32(uint32_t id)
{
  imgsensor_value_t val;
  isx019_get_value(id, 0, &val);
  return val.value32;
}

static void store_default_value(void)
{
  FAR isx019_default_value_t *def = &g_isx019_private.default_value;

  def->brightness   = get_value32(IMGSENSOR_ID_BRIGHTNESS);
  def->contrast     = get_value32(IMGSENSOR_ID_CONTRAST);
  def->saturation   = get_value32(IMGSENSOR_ID_SATURATION);
  def->hue          = get_value32(IMGSENSOR_ID_HUE);
  def->awb          = get_value32(IMGSENSOR_ID_AUTO_WHITE_BALANCE);
  def->gamma        = get_value32(IMGSENSOR_ID_GAMMA);
  def->ev           = get_value32(IMGSENSOR_ID_EXPOSURE);
  def->hflip_video  = get_value32(IMGSENSOR_ID_HFLIP_VIDEO);
  def->vflip_video  = get_value32(IMGSENSOR_ID_VFLIP_VIDEO);
  def->hflip_still  = get_value32(IMGSENSOR_ID_HFLIP_STILL);
  def->vflip_still  = get_value32(IMGSENSOR_ID_VFLIP_STILL);
  def->sharpness    = get_value32(IMGSENSOR_ID_SHARPNESS);
  def->ae           = get_value32(IMGSENSOR_ID_EXPOSURE_AUTO);
  def->exptime      = get_value32(IMGSENSOR_ID_EXPOSURE_ABSOLUTE);
  def->wbmode       = get_value32(IMGSENSOR_ID_AUTO_N_PRESET_WB);
  def->hdr          = get_value32(IMGSENSOR_ID_WIDE_DYNAMIC_RANGE);
  def->iso          = get_value32(IMGSENSOR_ID_ISO_SENSITIVITY);
  def->iso_auto     = get_value32(IMGSENSOR_ID_ISO_SENSITIVITY_AUTO);
  def->meter        = get_value32(IMGSENSOR_ID_EXPOSURE_METERING);
  def->threealock   = get_value32(IMGSENSOR_ID_3A_LOCK);
  def->threeastatus = get_value32(IMGSENSOR_ID_3A_STATUS);
  def->jpgquality   = get_value32(IMGSENSOR_ID_JPEG_QUALITY);
}

static int isx019_init(void)
{
  uint32_t clk;

  power_on();
  set_drive_mode();
  fpga_init();
  initialize_jpg_quality();
  store_default_value();
  clk = board_isx019_get_master_clock();
  g_isx019_private.clock_ratio
    = (float)clk / ISX019_STANDARD_MASTER_CLOCK;

  return OK;
}

static int isx019_uninit(void)
{
  power_off();
  return OK;
}

static FAR const char *isx019_get_driver_name(void)
{
#ifdef CONFIG_VIDEO_ISX019_NAME_WITH_VERSION
  static char name[16];
  uint8_t f_ver = 0;
  uint16_t is_ver = 0;

  isx019_i2c_read(CAT_VERSION, ROM_VERSION, (FAR uint8_t *)&is_ver, 2);
  fpga_i2c_read(FPGA_VERSION, &f_ver, 1);
  snprintf(name, sizeof(name), "ISX019 v%04x_%02d", is_ver, f_ver);

  return name;
#else
  return "ISX019";
#endif
}

static int validate_format(int nr_fmt, FAR imgsensor_format_t *fmt)
{
  int ret;

  uint16_t main_w;
  uint16_t main_h;
  uint16_t sub_w;
  uint16_t sub_h;

  if ((nr_fmt < 1) || (nr_fmt > 2))
    {
      return -EINVAL;
    }

  if (fmt == NULL)
    {
      return -EINVAL;
    }

  main_w = fmt[IMGSENSOR_FMT_MAIN].width;
  main_h = fmt[IMGSENSOR_FMT_MAIN].height;

  switch (fmt[IMGSENSOR_FMT_MAIN].pixelformat)
    {
      case IMGSENSOR_PIX_FMT_UYVY:             /* YUV 4:2:2 */
      case IMGSENSOR_PIX_FMT_RGB565:           /* RGB565 */
      case IMGSENSOR_PIX_FMT_JPEG:             /* JPEG */
      case IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG: /* JPEG + sub image */

        if (!VALIDATE_FRAMESIZE(main_w, main_h))
          {
            ret = -EINVAL;
            break;
          }

        if (nr_fmt > 1)
          {
            sub_w = fmt[IMGSENSOR_FMT_SUB].width;
            sub_h = fmt[IMGSENSOR_FMT_SUB].height;
            if (!VALIDATE_THUMNAIL_SIZE(main_w, sub_w) ||
                !VALIDATE_THUMNAIL_SIZE(main_h, sub_h))
              {
                ret = -EINVAL;
                break;
              }
          }

        ret = OK;
        break;

      default: /* otherwise */
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int validate_frameinterval(FAR imgsensor_interval_t *interval)
{
  int ret = -EINVAL;

  if (interval == NULL)
    {
      return -EINVAL;
    }

  /* Avoid multiplication overflow */

  if ((interval->denominator * 10) / 10 != interval->denominator)
    {
      return -EINVAL;
    }

  /* Avoid division by zero */

  if (interval->numerator == 0)
    {
      return -EINVAL;
    }

  switch ((interval->denominator * 10) / interval->numerator)
    {
      case 300:  /* 30FPS  */
      case 150:  /* 15FPS  */
      case 100:  /* 10FPS  */
      case 75:   /* 7.5FPS */
        ret = OK;
        break;

      default:  /* otherwise  */
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int isx019_validate_frame_setting(imgsensor_stream_type_t type,
                                         uint8_t nr_fmt,
                                         FAR imgsensor_format_t *fmt,
                                         FAR imgsensor_interval_t *interval)
{
  int ret = OK;

  ret = validate_format(nr_fmt, fmt);
  if (ret != OK)
    {
      return ret;
    }

  return validate_frameinterval(interval);
}

static int activate_flip(imgsensor_stream_type_t type)
{
  uint8_t flip;

  flip = (type == IMGSENSOR_STREAM_TYPE_VIDEO) ?
         g_isx019_private.flip_video : g_isx019_private.flip_still;

  return isx019_i2c_write(CAT_CONFIG, REVERSE, &flip, 1);
}

static int activate_clip(imgsensor_stream_type_t type,
                         uint16_t w,
                         uint16_t h)
{
  FAR isx019_rect_t *clip;
  uint8_t size;
  uint8_t top;
  uint8_t left = 0;

  clip = (type == IMGSENSOR_STREAM_TYPE_VIDEO) ?
         &g_isx019_private.clip_video : &g_isx019_private.clip_still;

  switch (w)
    {
      case 1280:
        if (clip->width == 640) /* In this case, c_h == 360 */
          {
            size = FPGA_CLIP_640_360;
            top  = clip->top / FPGA_CLIP_UNIT;
            left = clip->left / FPGA_CLIP_UNIT;

            if (h == 720)
              {
                /* Shift (960 - 720) / 2 lines */

                top  += 120 / FPGA_CLIP_UNIT;
              }
          }
        else if (clip->width == 1280)
          {
            /* In this case, clip->height == 720 */

            size = FPGA_CLIP_1280_720;
            top  = clip->top / FPGA_CLIP_UNIT;
            left = clip->left / FPGA_CLIP_UNIT;
          }
        else /* no clip */
          {
            if (h == 720)
              {
                size = FPGA_CLIP_1280_720;

                /* Shift (960 - 720) / 2 lines */

                top = 120 / FPGA_CLIP_UNIT;
              }
            else
              {
                size = FPGA_CLIP_NON;
                top  = 0;
                left = 0;
              }
          }

        break;

      default: /* 640 */
        if (clip->width == 640)
          {
            /* In this case, clip->height == 360 */

            size = FPGA_CLIP_640_360;
            top  = clip->top / FPGA_CLIP_UNIT;
            left = clip->left / FPGA_CLIP_UNIT;
          }
        else /* no clip */
          {
            if (h == 360)
              {
                size = FPGA_CLIP_640_360;

                /* Shift (480 - 360) / 2 lines */

                top  = 60 / FPGA_CLIP_UNIT;
              }
            else
              {
                size = FPGA_CLIP_NON;
                top  = 0;
                left = 0;
              }
          }

        break;
    }

  fpga_i2c_write(FPGA_CLIP_SIZE, &size, 1);
  fpga_i2c_write(FPGA_CLIP_TOP,  &top, 1);
  fpga_i2c_write(FPGA_CLIP_LEFT, &left, 1);

  return OK;
}

static int isx019_start_capture(imgsensor_stream_type_t type,
                                uint8_t nr_fmt,
                                FAR imgsensor_format_t *fmt,
                                FAR imgsensor_interval_t *interval)
{
  int ret;
  uint8_t regval = 0;

  ret = isx019_validate_frame_setting(type, nr_fmt, fmt, interval);
  if (ret != OK)
    {
      return ret;
    }

  ret = activate_flip(type);
  if (ret != OK)
    {
      return ret;
    }

  nxmutex_lock(&g_isx019_private.fpga_lock);

  /* Update FORMAT_AND_SCALE register of FPGA */

  switch (fmt[IMGSENSOR_FMT_MAIN].pixelformat)
    {
      case IMGSENSOR_PIX_FMT_RGB565:
        regval |= FPGA_FORMAT_RGB;
        break;

      case IMGSENSOR_PIX_FMT_UYVY:
        regval |= FPGA_FORMAT_YUV;
        break;

      case IMGSENSOR_PIX_FMT_JPEG:
        regval |= FPGA_FORMAT_JPEG;
        break;

      default: /* IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG */
        if (nr_fmt == 1)
          {
            regval |= FPGA_FORMAT_JPEG;
          }
        else
          {
            regval |= FPGA_FORMAT_THUMBNAIL;
          }

        break;
    }

  switch (fmt[IMGSENSOR_FMT_MAIN].width)
   {
     case 1280:
       regval |= FPGA_SCALE_1280_960;
       activate_clip(type,
                     fmt[IMGSENSOR_FMT_MAIN].width,
                     fmt[IMGSENSOR_FMT_MAIN].height);
       break;

     case 640:
       regval |= FPGA_SCALE_640_480;
       activate_clip(type,
                     fmt[IMGSENSOR_FMT_MAIN].width,
                     fmt[IMGSENSOR_FMT_MAIN].height);
       break;

     case 320:
       regval |= FPGA_SCALE_320_240;
       break;

     default: /* 160 */

       regval |= FPGA_SCALE_160_120;
       break;
   }

  fpga_i2c_write(FPGA_FORMAT_AND_SCALE, &regval, 1);

  /* Update FPS_AND_THUMBNAIL register of FPGA */

  regval = 0;

  if (nr_fmt == 2)
    {
      switch (fmt[IMGSENSOR_FMT_MAIN].width / fmt[IMGSENSOR_FMT_SUB].width)
        {
          case 1 : /* 1/1 */
            regval |= FPGA_THUMBNAIL_SCALE_1_1;
            break;

          case 2 : /* 1/2 */
            regval |= FPGA_THUMBNAIL_SCALE_1_2;
            break;

          case 4 : /* 1/4 */
            regval |= FPGA_THUMBNAIL_SCALE_1_4;
            break;

          default : /* 1/8 */
            regval |= FPGA_THUMBNAIL_SCALE_1_8;
            break;
        }
    }

  switch ((interval->denominator * 10) / interval->numerator)
    {
      case 300:  /* 30FPS */
        regval |= FPGA_FPS_1_1;
        break;

      case 150:  /* 15FPS */
        regval |= FPGA_FPS_1_2;
        break;

      case 100:  /* 10FPS */
        regval |= FPGA_FPS_1_3;
        break;

      case 75:   /* 7.5FPS */
        regval |= FPGA_FPS_1_4;
        break;

      default:   /* otherwise */

        /* It may not come here because the value has already been validated
         * in validate_frameinterval().
         */

        break;
    }

  fpga_i2c_write(FPGA_FPS_AND_THUMBNAIL, &regval, 1);

  regval = FPGA_DATA_OUTPUT_START;
  fpga_i2c_write(FPGA_DATA_OUTPUT, &regval, 1);

  fpga_activate_setting();
  nxmutex_unlock(&g_isx019_private.fpga_lock);
  g_isx019_private.stream = type;

  return OK;
}

static int isx019_stop_capture(imgsensor_stream_type_t type)
{
  uint8_t regval;

  regval = FPGA_DATA_OUTPUT_STOP;
  nxmutex_lock(&g_isx019_private.fpga_lock);
  fpga_i2c_write(FPGA_DATA_OUTPUT, &regval, 1);
  fpga_activate_setting();
  nxmutex_unlock(&g_isx019_private.fpga_lock);
  return OK;
}

static int isx019_get_supported_value(uint32_t id,
                                      FAR imgsensor_supported_value_t *val)
{
  int ret = OK;
  FAR struct isx019_default_value_s *def = &g_isx019_private.default_value;

  DEBUGASSERT(val);

  switch (id)
    {
      case IMGSENSOR_ID_BRIGHTNESS:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_BRIGHTNESS, MAX_BRIGHTNESS,
                                STEP_BRIGHTNESS, def->brightness);
        break;

      case IMGSENSOR_ID_CONTRAST:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_CONTRAST, MAX_CONTRAST,
                                STEP_CONTRAST, def->contrast);
        break;

      case IMGSENSOR_ID_SATURATION:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_SATURATION, MAX_SATURATION,
                                STEP_SATURATION, def->saturation);
        break;

      case IMGSENSOR_ID_HUE:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_HUE, MAX_HUE,
                                STEP_HUE, def->hue);
        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_AWB, MAX_AWB,
                                STEP_AWB, def->awb);
        break;

      case IMGSENSOR_ID_GAMMA:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_GAMMA, MAX_GAMMA,
                                STEP_GAMMA, def->gamma);
        break;

      case IMGSENSOR_ID_EXPOSURE:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_EXPOSURE, MAX_EXPOSURE,
                                STEP_EXPOSURE, def->ev);
        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_HFLIP, MAX_HFLIP,
                                STEP_HFLIP, def->hflip_video);
        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_VFLIP, MAX_VFLIP,
                                STEP_VFLIP, def->vflip_video);
        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_HFLIP, MAX_HFLIP,
                                STEP_HFLIP, def->hflip_still);
        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_VFLIP, MAX_VFLIP,
                                STEP_VFLIP, def->hflip_still);
        break;

      case IMGSENSOR_ID_SHARPNESS:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_SHARPNESS, MAX_SHARPNESS,
                                STEP_SHARPNESS, def->sharpness);
        break;

      case IMGSENSOR_ID_COLORFX:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        SET_DISCRETE(val->u.discrete,
                     NR_COLORFX,
                     g_isx019_colorfx,
                     IMGSENSOR_COLORFX_NONE);
        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_AE, MAX_AE,
                                STEP_AE, def->ae);
        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_EXPOSURETIME, MAX_EXPOSURETIME,
                                STEP_EXPOSURETIME, def->exptime);
        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        SET_DISCRETE(val->u.discrete,
                     NR_WBMODE,
                     g_isx019_wbmode,
                     IMGSENSOR_WHITE_BALANCE_AUTO);
        break;

      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_HDR, MAX_HDR,
                                STEP_HDR, def->hdr);
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_ISO, MAX_ISO,
                                STEP_ISO, def->iso);
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_AUTOISO, MAX_AUTOISO,
                                STEP_AUTOISO, def->iso_auto);
        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_METER, MAX_METER,
                                STEP_METER, def->meter);
        break;

      case IMGSENSOR_ID_3A_LOCK:
        val->type = IMGSENSOR_CTRL_TYPE_BITMASK;
        SET_RANGE(val->u.range, MIN_3ALOCK, MAX_3ALOCK,
                                STEP_3ALOCK, def->threealock);
        break;

      case IMGSENSOR_ID_3A_PARAMETER:
        val->type = IMGSENSOR_CTRL_TYPE_U8;
        SET_ELEMS(val->u.elems, NRELEM_3APARAM, MIN_3APARAM, MAX_3APARAM,
                                STEP_3APARAM);
        break;

      case IMGSENSOR_ID_3A_STATUS:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_3ASTATUS, MAX_3ASTATUS,
                                STEP_3ASTATUS, def->threeastatus);
        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_JPGQUALITY, MAX_JPGQUALITY,
                                STEP_JPGQUALITY, def->jpgquality);
        break;

      case IMGSENSOR_ID_CLIP_VIDEO:
      case IMGSENSOR_ID_CLIP_STILL:
        val->type = IMGSENSOR_CTRL_TYPE_U32;
        SET_ELEMS(val->u.elems, NRELEM_CLIP, MIN_CLIP, MAX_CLIP,
                                STEP_CLIP);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int32_t not_convert(int32_t val)
{
  return val;
}

static int32_t convert_brightness_is2reg(int32_t val)
{
  return (val << 2);
}

static int32_t convert_brightness_reg2is(int32_t val)
{
  return (val >> 2);
}

static int32_t convert_hue_is2reg(int32_t val)
{
  return (val * 90) / 128;
}

static int32_t convert_hue_reg2is(int32_t val)
{
  return (val * 128) / 90;
}

static int32_t convert_awb_is2reg(int32_t val)
{
  return (val == 1) ? 0 : 2;
}

static int32_t convert_awb_reg2is(int32_t val)
{
  return (val == 0) ? 1 : 0;
}

static int32_t convert_hdr_is2reg(int32_t val)
{
  int32_t ret = AEWDMODE_AUTO;

  switch (val)
    {
      case 0:
        ret = AEWDMODE_NORMAL;
        break;

      case 1:
        ret = AEWDMODE_AUTO;
        break;

      case 2:
        ret = AEWDMODE_HDR;
        break;

      default:
        /* It may not come here because the value has already been validated
         * in validate_value().
         */

        break;
    }

  return ret;
}

static int32_t convert_hdr_reg2is(int32_t val)
{
  int32_t ret;

  switch (val)
    {
      case AEWDMODE_NORMAL:
        ret = 0;
        break;

      case AEWDMODE_AUTO:
        ret = 1;
        break;

      default: /* AEWDMODE_HDR */
        ret = 2;
        break;
    }

  return ret;
}

static convert_t get_reginfo(uint32_t id, bool is_set,
                             FAR isx019_reginfo_t *reg)
{
  convert_t cvrt = NULL;

  DEBUGASSERT(reg);

  switch (id)
    {
      case IMGSENSOR_ID_BRIGHTNESS:
        SET_REGINFO(reg, CAT_PICTTUNE, UIBRIGHTNESS, 2);
        cvrt = is_set ? convert_brightness_is2reg
                      : convert_brightness_reg2is;
        break;

      case IMGSENSOR_ID_CONTRAST:
        SET_REGINFO(reg, CAT_PICTTUNE, UICONTRAST, 1);
        cvrt = not_convert;
        break;

      case IMGSENSOR_ID_SATURATION:
        SET_REGINFO(reg, CAT_PICTTUNE, UISATURATION, 1);
        cvrt = not_convert;
        break;

      case IMGSENSOR_ID_HUE:
        SET_REGINFO(reg, CAT_PICTTUNE, UIHUE, 1);
        cvrt = is_set ? convert_hue_is2reg : convert_hue_reg2is;
        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        SET_REGINFO(reg, CAT_CATAWB, AWBMODE, 1);
        cvrt = is_set ? convert_awb_is2reg : convert_awb_reg2is;
        break;

      case IMGSENSOR_ID_EXPOSURE:
        SET_REGINFO(reg, CAT_AEDGRM, EVSEL, 1);
        cvrt = not_convert;
        break;

      case IMGSENSOR_ID_SHARPNESS:
        SET_REGINFO(reg, CAT_PICTTUNE, UISHARPNESS, 1);
        cvrt = not_convert;
        break;

      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
        SET_REGINFO(reg, CAT_AEWD, AEWDMODE, 1);
        cvrt = is_set ? convert_hdr_is2reg : convert_hdr_reg2is;
        break;

      default:
        break;
    }

  return cvrt;
}

static void set_flip(FAR uint8_t *flip, uint8_t direction, int32_t val)
{
  DEBUGASSERT(flip);

  *flip = (val == 0) ? (*flip & ~direction) : (*flip | direction);
}

static int set_hflip_video(imgsensor_value_t val)
{
  set_flip(&g_isx019_private.flip_video, H_REVERSE, val.value32);
  if (g_isx019_private.stream == IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      activate_flip(IMGSENSOR_STREAM_TYPE_VIDEO);
    }

  return OK;
}

static int set_vflip_video(imgsensor_value_t val)
{
  set_flip(&g_isx019_private.flip_video, V_REVERSE, val.value32);
  if (g_isx019_private.stream == IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      activate_flip(IMGSENSOR_STREAM_TYPE_VIDEO);
    }

  return OK;
}

static int set_hflip_still(imgsensor_value_t val)
{
  set_flip(&g_isx019_private.flip_still, H_REVERSE, val.value32);
  if (g_isx019_private.stream == IMGSENSOR_STREAM_TYPE_STILL)
    {
      activate_flip(IMGSENSOR_STREAM_TYPE_STILL);
    }

  return OK;
}

static int set_vflip_still(imgsensor_value_t val)
{
  set_flip(&g_isx019_private.flip_still, V_REVERSE, val.value32);
  if (g_isx019_private.stream == IMGSENSOR_STREAM_TYPE_STILL)
    {
      activate_flip(IMGSENSOR_STREAM_TYPE_STILL);
    }

  return OK;
}

static int set_colorfx(imgsensor_value_t val)
{
  int ret = -EINVAL;
  FAR isx019_default_value_t *def = &g_isx019_private.default_value;
  int32_t sat;
  int32_t sharp;

  /* ISX019 realize color effects by adjusting saturation and sharpness. */

  switch (val.value32)
    {
      case IMGSENSOR_COLORFX_NONE:

        sat   = def->saturation;
        sharp = def->sharpness;
        break;

      case IMGSENSOR_COLORFX_BW:

        sat   = BW_COLORS_SATURATION;
        sharp = def->sharpness;
        break;

      case IMGSENSOR_COLORFX_VIVID:

        sat   = VIVID_COLORS_SATURATION;
        sharp = VIVID_COLORS_SHARPNESS;
        break;

      default:

        /* It may not come here because the value has already been validated
         * in validate_value().
         */

        break;
    }

  ret = isx019_i2c_write(CAT_PICTTUNE, UISATURATION, (FAR uint8_t *)&sat, 1);
  if (ret == OK)
    {
      ret = isx019_i2c_write(CAT_PICTTUNE,
                             UISHARPNESS,
                             (FAR uint8_t *)&sharp,
                             1);
      if (ret == OK)
        {
          g_isx019_private.colorfx = val.value32;
        }
    }

  return ret;
}

static int set_ae(imgsensor_value_t val)
{
  uint32_t regval = 0;

  if (val.value32 == IMGSENSOR_EXPOSURE_AUTO)
    {
      regval = 0;
    }
  else
    {
      isx019_i2c_read(CAT_AESOUT, SHT_TIME, (FAR uint8_t *)&regval, 4);
    }

  return isx019_i2c_write(CAT_CATAE, SHT_PRIMODE, (FAR uint8_t *)&regval, 4);
}

static int set_exptime(imgsensor_value_t val)
{
  uint32_t regval;

  /* Take into account the master clock and convert unit.
   *   image sensor I/F : 100usec
   *   register         :   1usec
   */

  regval = val.value32 * 100 * g_isx019_private.clock_ratio;

  return isx019_i2c_write(CAT_CATAE, SHT_PRIMODE, (FAR uint8_t *)&regval, 4);
}

static int set_wbmode(imgsensor_value_t val)
{
  /*  AWBMODE     mode0 : auto, mode4 : user defined white balance
   *  AWBUSER_NO  definition number for AWBMODE = mode4
   *  USER0_R, USER1_B : R,B value for AWBUSER_NO = 0
   *  USER1_R, USER1_B : R,B value for AWBUSER_NO = 1
   */

  uint8_t  mode;
  uint16_t r_addr;
  uint16_t b_addr;
  uint16_t r;
  uint16_t b;
  static bool toggle = false;

  if (toggle)
    {
      r_addr = USER0_R;
      b_addr = USER0_B;
      toggle = false;
    }
  else
    {
      r_addr = USER1_R;
      b_addr = USER1_B;
      toggle = true;
    }

  switch (val.value32)
    {
      case IMGSENSOR_WHITE_BALANCE_AUTO:
        mode = AWBMODE_AUTO;
        break;

      case IMGSENSOR_WHITE_BALANCE_INCANDESCENT:
        r = RED_INCANDESCENT;
        b = BLUE_INCANDESCENT;
        mode = AWBMODE_MANUAL;
        break;

      case IMGSENSOR_WHITE_BALANCE_FLUORESCENT:
        r = RED_FLUORESCENT;
        b = BLUE_FLUORESCENT;
        mode = AWBMODE_MANUAL;
        break;

      case IMGSENSOR_WHITE_BALANCE_DAYLIGHT:
        r = RED_DAYLIGHT;
        b = BLUE_DAYLIGHT;
        mode = AWBMODE_MANUAL;
        break;

      case IMGSENSOR_WHITE_BALANCE_CLOUDY:
        r = RED_CLOUDY;
        b = BLUE_CLOUDY;
        mode = AWBMODE_MANUAL;
        break;

      default: /* IMGSENSOR_WHITE_BALANCE_SHADE */
        r = RED_SHADE;
        b = BLUE_SHADE;
        mode = AWBMODE_MANUAL;
        break;
    }

  isx019_i2c_write(CAT_AWB_USERTYPE, r_addr, (FAR uint8_t *)&r, 2);
  isx019_i2c_write(CAT_AWB_USERTYPE, b_addr, (FAR uint8_t *)&b, 2);
  isx019_i2c_write(CAT_CATAWB, AWBUSER_NO, (FAR uint8_t *)&toggle, 1);
  isx019_i2c_write(CAT_CATAWB, AWBMODE, &mode, 1);

  g_isx019_private.wb_mode = val.value32;

  return OK;
}

static int set_meter(imgsensor_value_t val)
{
  uint8_t normal;
  uint8_t hdr;

  switch (val.value32)
    {
      case IMGSENSOR_EXPOSURE_METERING_AVERAGE:
        normal = AEWEIGHT_AVERAGE;
        hdr    = AEWEIGHTHDR_AVERAGE;
        break;

      case IMGSENSOR_EXPOSURE_METERING_CENTER_WEIGHTED:
        normal = AEWEIGHT_CENTER;
        hdr    = AEWEIGHTHDR_CENTER;
        break;

      case IMGSENSOR_EXPOSURE_METERING_SPOT:
        normal = AEWEIGHT_SPOT;
        hdr    = AEWEIGHTHDR_SPOT;
        break;

      default: /* IMGSENSOR_EXPOSURE_METERING_MATRIX */
        normal = AEWEIGHT_MATRIX;
        hdr    = AEWEIGHTHDR_MATRIX;
        break;
    }

  isx019_i2c_write(CAT_AUTOCTRL, AEWEIGHTMODE, &normal, 1);
  isx019_i2c_write(CAT_AEWD, AEWEIGHTMODE_WD, &hdr, 1);

  return OK;
}

static int set_3alock(imgsensor_value_t val)
{
  uint8_t regval;

  regval = (val.value32 & IMGSENSOR_LOCK_WHITE_BALANCE) ? AWBMODE_HOLD
                                                        : AWBMODE_AUTO;
  isx019_i2c_write(CAT_CATAWB, AWBMODE, &regval, 1);

  regval = (val.value32 & IMGSENSOR_LOCK_EXPOSURE) ? AEMODE_HOLD
                                                   : AEMODE_AUTO;
  isx019_i2c_write(CAT_CATAE,  AEMODE,  &regval,  1);

  return OK;
}

static int set_3aparameter(imgsensor_value_t val)
{
  uint16_t gain;
  uint8_t regval;

  if (val.p_u8 == NULL)
    {
      return -EINVAL;
    }

  /* Convert unit
   *  GAIN_LEVEL register(accessed in get_3aparameter()) : 0.3dB
   *  GAIN_PRIMODE register(accessed in this function)   : 0.1dB
   */

  gain = val.p_u8[OFFSET_3APARAMETER_AE_GAIN] * 3;

  isx019_i2c_write
  (CAT_AWB_USERTYPE, USER4_R, &val.p_u8[OFFSET_3APARAMETER_AWB_R], 2);
  isx019_i2c_write
  (CAT_AWB_USERTYPE, USER4_B, &val.p_u8[OFFSET_3APARAMETER_AWB_B], 2);

  regval = 4;
  isx019_i2c_write(CAT_CATAWB, AWBUSER_NO, (FAR uint8_t *)&regval, 1);

  regval = AWBMODE_MANUAL;
  isx019_i2c_write(CAT_CATAWB, AWBMODE, &regval, 1);

  isx019_i2c_write
  (CAT_CATAE, SHT_PRIMODE, &val.p_u8[OFFSET_3APARAMETER_AE_SHTTIME], 4);
  isx019_i2c_write(CAT_CATAE, GAIN_PRIMODE, (FAR uint8_t *)&gain, 2);

  return OK;
}

static uint16_t calc_gain(double iso)
{
  double gain;

  gain = 1 + 10 * log(iso) / M_LN10;

  /* In the above formula, the unit of gain is dB.
   * Because the register has the 0.1dB unit,
   * return 10 times dB value.
   */

  return (uint16_t)(gain * 10);
}

static int set_iso(imgsensor_value_t val)
{
  uint16_t gain;

  /* ISX019 has not ISO sensitivity register but gain register.
   * So, calculate gain from ISO sensitivity.
   */

  gain = calc_gain(val.value32 / 1000);
  isx019_i2c_write(CAT_CATAE, GAIN_PRIMODE, (FAR uint8_t *)&gain, 2);

  g_isx019_private.iso = val.value32;
  return OK;
}

static int set_iso_auto(imgsensor_value_t val)
{
  uint8_t  buf;
  uint16_t gain;

  if (val.value32 == IMGSENSOR_ISO_SENSITIVITY_AUTO)
    {
      gain = 0;
      g_isx019_private.iso = 0;
    }
  else /* IMGSENSOR_ISO_SENSITIVITY_MANUAL */
    {
      isx019_i2c_read(CAT_CATAE, GAIN_PRIMODE, (FAR uint8_t *)&gain, 2);

      if (gain == 0)
        {
          /* gain = 0 means auto adjustment mode.
           * In such a case, apply current auto adjustment value
           * as manual setting.
           * Note : auto adjustment value register has the unit 0.3dB.
           *        So, convert the unit to 0.1dB.
           */

          isx019_i2c_read(CAT_AECOM, GAIN_LEVEL, &buf, 1);
          gain = buf * 3;
        }

      g_isx019_private.iso = val.value32;
    }

  return isx019_i2c_write(CAT_CATAE, GAIN_PRIMODE, (FAR uint8_t *)&gain, 2);
}

static uint16_t calc_gamma_regval(double in, double gamma)
{
  double out;

  /* 1) Calculate the normalized result.
   *    formula :  output = input^gamma
   * 2) Perform scaling for ISX019 register.
   * 3) Change the format from the floating-point number type
   *    to the fixed-point number type according to the register.
   */

  out = pow(in, gamma);
  out *= GAM_OUTPUT_SCALE;

  return ((uint8_t)out) << 2;
}

static int set_gamma(imgsensor_value_t val)
{
  int i;
  uint16_t regval;
  uint16_t offset;
  double gamma;

  gamma = (double)val.value32 / 1000;

  /* ISX019 gamma adjustment feature is constructed by
   * registers for low-input and registers for high-input.
   */

  offset = GAM_KNOT_C0;

  for (i = 0; i < NR_GAM_KNOT_LOWINPUT; i++)
    {
      regval = calc_gamma_regval((double)i * GAM_LOWINPUT_INTERVAL, gamma);
      isx019_i2c_write(CAT_PICTGAMMA, offset, (FAR uint8_t *)&regval, 2);
      offset += 2;
    }

  offset = GAM_KNOT_C11;

  for (i = 0; i < NR_GAM_KNOT_HIGHINPUT; i++)
    {
      regval = calc_gamma_regval
               ((double)(i + 1) * GAM_HIGHINPUT_INTERVAL, gamma);
      isx019_i2c_write(CAT_PICTGAMMA, offset, (FAR uint8_t *)&regval, 2);
      offset += 2;
    }

  /* Special register setting.
   * GAM_KNOT_C9 and GAM_KNOT_C10 need to be set
   * to be continuous.
   * So, this driver set GAM_KNOT_C10 = GAM_KNOT_C8,
   * GAM_KNOT_C9 = GAM_KNOT_C11.
   */

  isx019_i2c_read(CAT_PICTGAMMA,  GAM_KNOT_C8, (FAR uint8_t *)&regval, 2);
  isx019_i2c_write(CAT_PICTGAMMA, GAM_KNOT_C10, (FAR uint8_t *)&regval, 2);
  isx019_i2c_read(CAT_PICTGAMMA,  GAM_KNOT_C11, (FAR uint8_t *)&regval, 2);
  isx019_i2c_write(CAT_PICTGAMMA, GAM_KNOT_C9, (FAR uint8_t *)&regval, 2);

  g_isx019_private.gamma = val.value32;
  return OK;
}

static void search_dqt_data(int32_t quality,
                            FAR uint8_t **y_head, FAR uint8_t **y_calc,
                            FAR uint8_t **c_head, FAR uint8_t **c_calc)
{
  int i;
  FAR isx019_fpga_jpg_quality_t *jpg = &g_isx019_jpg_quality[0];

  *y_head = NULL;
  *y_calc = NULL;
  *c_head = NULL;
  *c_calc = NULL;

  /* Search approximate DQT data from a table by rounding quality. */

  quality = ((quality + 5) / 10) * 10;
  if (quality == 0)
    {
      /* Set the minimum value of quality to 10. */

      quality = 10;
    }

  for (i = 0; i < NR_JPGSETTING_TBL; i++)
    {
      if (quality == jpg->quality)
        {
          *y_head = jpg->y_head;
          *y_calc = jpg->y_calc;
          *c_head = jpg->c_head;
          *c_calc = jpg->c_calc;
          break;
        }

      jpg++;
    }
}

int set_dqt(uint8_t component, uint8_t target, FAR uint8_t *buf)
{
  int i;
  uint8_t addr;
  uint8_t select;
  uint8_t data;
  uint8_t regval;

  if (target == FPGA_DQT_DATA)
    {
      addr   = FPGA_DQT_ADDRESS;
      select = FPGA_DQT_SELECT;
      data   = FPGA_DQT_DATA;
    }
  else
    {
      addr   = FPGA_DQT_CALC_ADDRESS;
      select = FPGA_DQT_CALC_SELECT;
      data   = FPGA_DQT_CALC_DATA;
    }

  fpga_i2c_write(select, &component, 1);
  for (i = 0; i < JPEG_DQT_ARRAY_SIZE; i++)
    {
      regval = i | FPGA_DQT_WRITE | FPGA_DQT_BUFFER;
      fpga_i2c_write(addr, &regval, 1);
      fpga_i2c_write(data, &buf[i], 1);
    }

  return OK;
}

static int set_jpg_quality(imgsensor_value_t val)
{
  FAR uint8_t *y_head;
  FAR uint8_t *y_calc;
  FAR uint8_t *c_head;
  FAR uint8_t *c_calc;

  /* Set JPEG quality by setting DQT information to FPGA. */

  search_dqt_data(val.value32, &y_head, &y_calc, &c_head, &c_calc);
  if ((y_head == NULL) ||
      (y_calc == NULL) ||
      (c_head == NULL) ||
      (c_calc == NULL))
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_isx019_private.fpga_lock);

  /* Update DQT data and activate them. */

  set_dqt(FPGA_DQT_LUMA,   FPGA_DQT_DATA, y_head);
  set_dqt(FPGA_DQT_CHROMA, FPGA_DQT_DATA, c_head);
  set_dqt(FPGA_DQT_LUMA,   FPGA_DQT_CALC_DATA, y_calc);
  set_dqt(FPGA_DQT_CHROMA, FPGA_DQT_CALC_DATA, c_calc);
  fpga_activate_setting();

  /* Update non-active side in preparation for other activation trigger. */

  set_dqt(FPGA_DQT_LUMA,   FPGA_DQT_DATA, y_head);
  set_dqt(FPGA_DQT_CHROMA, FPGA_DQT_DATA, c_head);
  set_dqt(FPGA_DQT_LUMA,   FPGA_DQT_CALC_DATA, y_calc);
  set_dqt(FPGA_DQT_CHROMA, FPGA_DQT_CALC_DATA, c_calc);

  nxmutex_unlock(&g_isx019_private.fpga_lock);

  g_isx019_private.jpg_quality = val.value32;
  return OK;
}

static int initialize_jpg_quality(void)
{
  imgsensor_value_t val;

  val.value32 = CONFIG_VIDEO_ISX019_INITIAL_JPEG_QUALITY;
  return set_jpg_quality(val);
}

static bool validate_clip_setting(FAR uint32_t *clip)
{
  bool ret = false;
  uint32_t w;
  uint32_t h;

  DEBUGASSERT(clip);

  w = clip[IMGSENSOR_CLIP_INDEX_WIDTH];
  h = clip[IMGSENSOR_CLIP_INDEX_HEIGHT];

  if (((w == 1280) && (h == 720)) ||
      ((w ==  640) && (h == 360)) ||
      ((w ==  0) && (h == 0)))
    {
      ret = true;
    }

  return ret;
}

static int set_clip(FAR uint32_t *val, FAR isx019_rect_t *target)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  if (!validate_clip_setting(val))
    {
      return -EINVAL;
    }

  target->left   = (int32_t)val[IMGSENSOR_CLIP_INDEX_LEFT];
  target->top    = (int32_t)val[IMGSENSOR_CLIP_INDEX_TOP];
  target->width  = val[IMGSENSOR_CLIP_INDEX_WIDTH];
  target->height = val[IMGSENSOR_CLIP_INDEX_HEIGHT];

  return OK;
}

static int set_clip_video(imgsensor_value_t val)
{
  return set_clip(val.p_u32, &g_isx019_private.clip_video);
}

static int set_clip_still(imgsensor_value_t val)
{
  return set_clip(val.p_u32, &g_isx019_private.clip_still);
}

static setvalue_t set_value_func(uint32_t id)
{
  setvalue_t func = NULL;

  switch (id)
    {
      case IMGSENSOR_ID_GAMMA:
        func = set_gamma;
        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        func = set_hflip_video;
        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        func = set_vflip_video;
        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        func = set_hflip_still;
        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        func = set_vflip_still;
        break;

      case IMGSENSOR_ID_COLORFX:
        func = set_colorfx;
        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        func = set_ae;
        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        func = set_exptime;
        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        func = set_wbmode;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        func = set_iso;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        func = set_iso_auto;
        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        func = set_meter;
        break;

      case IMGSENSOR_ID_3A_LOCK:
        func = set_3alock;
        break;

      case IMGSENSOR_ID_3A_PARAMETER:
        func = set_3aparameter;
        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        func = set_jpg_quality;
        break;

      case IMGSENSOR_ID_CLIP_VIDEO:
        func = set_clip_video;
        break;

      case IMGSENSOR_ID_CLIP_STILL:
        func = set_clip_still;
        break;

      default:
        break;
    }

  return func;
}

static int32_t get_flip(uint8_t *flip, uint8_t direction)
{
  DEBUGASSERT(flip);

  return (*flip & direction) ? 1 : 0;
}

static int get_hflip_video(FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = get_flip(&g_isx019_private.flip_video, H_REVERSE);
  return OK;
}

static int get_vflip_video(FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = get_flip(&g_isx019_private.flip_video, V_REVERSE);
  return OK;
}

static int get_hflip_still(FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = get_flip(&g_isx019_private.flip_still, H_REVERSE);
  return OK;
}

static int get_vflip_still(FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = get_flip(&g_isx019_private.flip_still, V_REVERSE);
  return OK;
}

static int get_colorfx(FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = g_isx019_private.colorfx;
  return OK;
}

static int get_ae(FAR imgsensor_value_t *val)
{
  uint32_t regval;

  if (val == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(CAT_CATAE, SHT_PRIMODE, (FAR uint8_t *)&regval, 4);

  val->value32 = (regval == 0) ? IMGSENSOR_EXPOSURE_AUTO
                               : IMGSENSOR_EXPOSURE_MANUAL;

  return OK;
}

static int get_exptime(FAR imgsensor_value_t *val)
{
  uint32_t regval;

  isx019_i2c_read(CAT_AESOUT, SHT_TIME, (FAR uint8_t *)&regval, 4);

  /* Round up to the nearest 100usec for eliminating errors in reverting to
   * application value because this driver converts application value to
   * value that takes into account the clock ratio and unit difference.
   */

  val->value32 = ((regval / g_isx019_private.clock_ratio) + 99) / 100;

  return OK;
}

static int get_wbmode(FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = g_isx019_private.wb_mode;

  return OK;
}

static int get_meter(FAR imgsensor_value_t *val)
{
  uint8_t regval;

  if (val == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(CAT_AUTOCTRL, AEWEIGHTMODE, &regval, 1);

  switch (regval)
    {
      case AEWEIGHT_AVERAGE:
        val->value32 = IMGSENSOR_EXPOSURE_METERING_AVERAGE;
        break;

      case AEWEIGHT_CENTER:
        val->value32 = IMGSENSOR_EXPOSURE_METERING_CENTER_WEIGHTED;
        break;

      case AEWEIGHT_SPOT:
        val->value32 = IMGSENSOR_EXPOSURE_METERING_SPOT;
        break;

      default: /* AEWEIGHT_MATRIX */
        val->value32 = IMGSENSOR_EXPOSURE_METERING_MATRIX;
        break;
    }

  return OK;
}

static int get_3alock(FAR imgsensor_value_t *val)
{
  uint8_t regval;
  uint8_t awb;
  uint8_t ae;

  if (val == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(CAT_CATAWB, AWBMODE, &regval, 1);
  awb = (regval == AWBMODE_AUTO) ? 0 : IMGSENSOR_LOCK_WHITE_BALANCE;

  isx019_i2c_read(CAT_CATAE,  AEMODE,  &regval,  1);
  ae = (regval == AEMODE_AUTO) ? 0 : IMGSENSOR_LOCK_EXPOSURE;

  val->value32 = awb | ae;

  return OK;
}

static int get_3aparameter(FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  if (val->p_u8 == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read
  (CAT_AWBSOUT, CONT_R,     &val->p_u8[OFFSET_3APARAMETER_AWB_R], 2);
  isx019_i2c_read
  (CAT_AWBSOUT, CONT_B,     &val->p_u8[OFFSET_3APARAMETER_AWB_B], 2);
  isx019_i2c_read
  (CAT_AESOUT,  SHT_TIME,   &val->p_u8[OFFSET_3APARAMETER_AE_SHTTIME], 4);
  isx019_i2c_read
  (CAT_AECOM,   GAIN_LEVEL, &val->p_u8[OFFSET_3APARAMETER_AE_GAIN], 1);

  return OK;
}

static int get_3astatus(FAR imgsensor_value_t *val)
{
  uint8_t regval;

  if (val == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(CAT_AWBSOUT, AWBSTS, &regval, 1);

  switch (regval)
    {
      case AWBSTS_STABLE:
        val->value32 = IMGSENSOR_3A_STATUS_STABLE;
        break;

      case AWBSTS_AEWAIT:
        val->value32 = IMGSENSOR_3A_STATUS_AE_OPERATING;
        break;

      default:
        val->value32 = IMGSENSOR_3A_STATUS_AE_OPERATING |
                       IMGSENSOR_3A_STATUS_AWB_OPERATING;
    }

  return OK;
}

static double calc_iso(double gain)
{
  int k;
  double z;
  double r;

  /* ISO sensitivity = 10^((gain - 1) / 10)
   * So, replace z = (gain - 1) / 10 and
   * calculate 10^z.
   */

  /*  Divide z into integer and other parts.
   *  z =  log10(E) (k * ln2 + r)
   *  (k : integer, r < 0.5 * ln2)
   *
   * Then, 10^z = (2^k) * e^r (r < 0.5 * ln2)
   */

  z = (gain - 1) / 10;

  k = z * M_LN10 / M_LN2;
  r = z * M_LN10 - k * M_LN2;

  return (1 << k) * exp(r);
}

static int get_iso(FAR imgsensor_value_t *val)
{
  uint8_t buf = 0;

  if (val == NULL)
    {
      return -EINVAL;
    }

  if (g_isx019_private.iso == 0)
    {
      /* iso = 0 means auto adjustment mode.
       * In such a case, get gain from auto adjustment value register,
       * which has the unit 0.3dB, and convert the gain to ISO.
       */

      isx019_i2c_read(CAT_AECOM, GAIN_LEVEL, &buf, 1);
      val->value32 = calc_iso((double)buf * 0.3) * USEC_PER_MSEC;
    }
  else
    {
      val->value32 = g_isx019_private.iso;
    }

  return OK;
}

static int get_iso_auto(FAR imgsensor_value_t *val)
{
  uint16_t gain;

  if (val == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(CAT_CATAE, GAIN_PRIMODE, (FAR uint8_t *)&gain, 2);

  val->value32 = (gain == 0) ? IMGSENSOR_ISO_SENSITIVITY_AUTO
                             : IMGSENSOR_ISO_SENSITIVITY_MANUAL;
  return OK;
}

static int get_gamma(FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = g_isx019_private.gamma;

  return OK;
}

static int get_jpg_quality(FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = g_isx019_private.jpg_quality;
  return OK;
}

static getvalue_t get_value_func(uint32_t id)
{
  getvalue_t func = NULL;

  switch (id)
    {
      case IMGSENSOR_ID_GAMMA:
        func = get_gamma;
        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        func = get_hflip_video;
        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        func = get_vflip_video;
        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        func = get_hflip_still;
        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        func = get_vflip_still;
        break;

      case IMGSENSOR_ID_COLORFX:
        func = get_colorfx;
        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        func = get_ae;
        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        func = get_exptime;
        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        func = get_wbmode;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        func = get_iso;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        func = get_iso_auto;
        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        func = get_meter;
        break;

      case IMGSENSOR_ID_3A_LOCK:
        func = get_3alock;
        break;

      case IMGSENSOR_ID_3A_PARAMETER:
        func = get_3aparameter;
        break;

      case IMGSENSOR_ID_3A_STATUS:
        func = get_3astatus;
        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        func = get_jpg_quality;
        break;

      default:
        break;
    }

  return func;
}

static int isx019_get_value(uint32_t id,
                            uint32_t size,
                            FAR imgsensor_value_t *val)
{
  int ret = -EINVAL;
  isx019_reginfo_t reg;
  convert_t cvrt;
  getvalue_t get;
  int32_t val32;

  DEBUGASSERT(val);

  cvrt = get_reginfo(id, false, &reg);
  if (cvrt)
    {
      ret = isx019_i2c_read
            (reg.category, reg.offset, (FAR uint8_t *)&val32, reg.size);
      val->value32 = cvrt(val32);
    }
  else
    {
      get = get_value_func(id);
      if (get)
        {
          ret = get(val);
        }
    }

  return ret;
}

static int validate_range(int32_t val,
                          FAR imgsensor_capability_range_t *range)
{
  int ret = OK;

  if (!VALIDATE_RANGE(val, range->minimum, range->maximum, range->step))
    {
      ret = -EINVAL;
    }

  return ret;
}

static int validate_discrete(int32_t val,
                             FAR imgsensor_capability_discrete_t *disc)
{
  int ret = -EINVAL;
  int i;

  for (i = 0; i < disc->nr_values; i++)
    {
      if (val == disc->values[i])
        {
          ret = OK;
          break;
        }
    }

  return ret;
}

static int validate_elems_u8(FAR uint8_t *val, uint32_t sz,
                             FAR imgsensor_capability_elems_t *elems)
{
  int ret = OK;
  int i;

  if (sz != elems->nr_elems)
    {
      return -EINVAL;
    }

  for (i = 0; i < elems->nr_elems; i++)
    {
      if (!VALIDATE_RANGE
           (val[i], elems->minimum, elems->maximum, elems->step))
        {
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}

static int validate_elems_u16 (FAR uint16_t *val, uint32_t sz,
                               FAR imgsensor_capability_elems_t *elems)
{
  int ret = OK;
  int i;

  if (sz != elems->nr_elems * sizeof(uint16_t))
    {
      return -EINVAL;
    }

  for (i = 0; i < elems->nr_elems; i++)
    {
      if (!VALIDATE_RANGE
           (val[i], elems->minimum, elems->maximum, elems->step))
        {
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}

static int validate_elems_u32 (FAR uint32_t *val, uint32_t sz,
                               FAR imgsensor_capability_elems_t *elems)
{
  int ret = OK;
  int i;

  if (sz != elems->nr_elems * sizeof(uint32_t))
    {
      return -EINVAL;
    }

  for (i = 0; i < elems->nr_elems; i++)
    {
      if (!VALIDATE_RANGE
           (val[i], elems->minimum, elems->maximum, elems->step))
        {
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}

static int validate_value(uint32_t id,
                          uint32_t size,
                          imgsensor_value_t val)
{
  int ret;
  imgsensor_supported_value_t sup;

  ret = isx019_get_supported_value(id, &sup);
  if (ret != OK)
    {
      return ret;
    }

  switch (sup.type)
    {
      case IMGSENSOR_CTRL_TYPE_INTEGER_MENU:
        ret = validate_discrete(val.value32, &sup.u.discrete);
        break;

      case IMGSENSOR_CTRL_TYPE_U8:
        ret = validate_elems_u8(val.p_u8, size, &sup.u.elems);
        break;

      case IMGSENSOR_CTRL_TYPE_U16:
        ret = validate_elems_u16(val.p_u16, size, &sup.u.elems);
        break;

      case IMGSENSOR_CTRL_TYPE_U32:
        ret = validate_elems_u32(val.p_u32, size, &sup.u.elems);
        break;

      default:
        ret = validate_range(val.value32, &sup.u.range);
        break;
    }

  return ret;
}

static int isx019_set_value(uint32_t id,
                            uint32_t size,
                            imgsensor_value_t val)
{
  int ret = -EINVAL;
  isx019_reginfo_t reg;
  convert_t cvrt;
  setvalue_t set;
  int32_t val32;

  ret = validate_value(id, size, val);
  if (ret != OK)
    {
      return ret;
    }

  cvrt = get_reginfo(id, true, &reg);
  if (cvrt)
    {
      val32 = cvrt(val.value32);
      ret = isx019_i2c_write
            (reg.category, reg.offset, (FAR uint8_t *)&val32, reg.size);
    }
  else
    {
      set = set_value_func(id);
      if (set)
        {
          ret = set(val);
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int isx019_initialize(void)
{
  imgsensor_register(&g_isx019_ops);
  return OK;
}

int isx019_uninitialize(void)
{
  return OK;
}

#ifdef CONFIG_VIDEO_ISX019_REGDEBUG
int isx019_read_register(uint8_t cat,
                         uint16_t addr,
                         FAR uint8_t *buf,
                         uint8_t size)
{
  int ret;

  if (cat == 0xff)
    {
      ret = fpga_i2c_read((uint8_t)addr, buf, size);
    }
  else
    {
      ret = isx019_i2c_read(cat, addr, buf, size);
    }

  return ret;
}
#endif
