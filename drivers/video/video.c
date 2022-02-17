/****************************************************************************
 * drivers/video/video.c
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

#include <sys/ioctl.h>

#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <poll.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>

#include "video_framebuff.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_VIDEO_FILE_PATH     (32)
#define MAX_VIDEO_FMT           (2)
#define VIDEO_FMT_MAIN          (0)
#define VIDEO_FMT_SUB           (1)

#define VIDEO_REMAINING_CAPNUM_INFINITY (-1)

#define VIDEO_SCENE_MAX (sizeof(g_video_scene_parameter) / \
                         sizeof(video_scene_params_t))

#define VIDEO_ID(x, y) (((x) << 16) | y)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum video_state_e
{
  VIDEO_STATE_STREAMOFF = 0, /* capture trigger event is not received */
  VIDEO_STATE_STREAMON  = 1, /* capture trigger event is received,
                              * but capture is not operated.
                              */
  VIDEO_STATE_CAPTURE   = 2, /* On capture */
};

enum video_state_transition_cause
{
  CAUSE_VIDEO_STOP  = 0,     /* Stop  capture event for video stream */
  CAUSE_VIDEO_START = 1,     /* Start capture event for video stream */
  CAUSE_VIDEO_DQBUF = 2,     /* DQBUF timing    for video stream */
  CAUSE_STILL_STOP  = 3,     /* Stop  capture event for still stream */
  CAUSE_STILL_START = 4,     /* Start capture event for still stream */
};

enum video_waitend_cause_e
{
  VIDEO_WAITEND_CAUSE_CAPTUREDONE   = 0,
  VIDEO_WAITEND_CAUSE_DQCANCEL  = 1,
  VIDEO_WAITEND_CAUSE_STILLSTOP = 2,
};

struct video_wait_capture_s
{
  FAR sem_t            dqbuf_wait_flg;

  /* Save container which capture is done */

  FAR vbuf_container_t *done_container;
  enum video_waitend_cause_e waitend_cause;
};

typedef struct video_wait_capture_s video_wait_capture_t;

struct video_format_s
{
  uint16_t width;
  uint16_t height;
  uint32_t pixelformat;
};

typedef struct video_format_s video_format_t;

struct video_type_inf_s
{
  sem_t                lock_state;
  enum video_state_e   state;
  int32_t              remaining_capnum;
  video_wait_capture_t wait_capture;
  uint8_t              nr_fmt;
  video_format_t       fmt[MAX_VIDEO_FMT];
  struct v4l2_fract    frame_interval;
  video_framebuff_t    bufinf;
};

typedef struct video_type_inf_s video_type_inf_t;

struct video_mng_s
{
  FAR char           *devpath;     /* parameter of video_initialize() */
  sem_t              lock_open_num;
  uint8_t            open_num;
  video_type_inf_t   video_inf;
  video_type_inf_t   still_inf;
};

typedef struct video_mng_s video_mng_t;

struct video_scene_params_s
{
  uint8_t mode;   /* enum v4l2_scene_mode */

  int32_t brightness;
  int32_t contrast;
  int32_t saturation;
  int32_t hue;
  bool    awb;
  int32_t red;
  int32_t blue;
  int32_t gamma;
  uint32_t gamma_curve_sz;
  uint8_t *gamma_curve;
  int32_t ev;
  bool    hflip_video;
  bool    vflip_video;
  bool    hflip_still;
  bool    vflip_still;
  int32_t sharpness;
  enum v4l2_colorfx colorfx;
  bool    auto_brightness;
  int32_t rotate;
  enum  v4l2_exposure_auto_type ae;
  int32_t exposure_time;
  int32_t focus;
  bool    af;
  int32_t zoom;
  int32_t iris;
  enum v4l2_auto_n_preset_white_balance wb;
  int32_t wdr;
  bool    stabilization;
  enum v4l2_iso_sensitivity_auto_type iso_auto;
  int32_t iso;
  enum v4l2_exposure_metering meter;
  int32_t threea_lock;
  enum v4l2_flash_led_mode led;
  int32_t jpeg_quality;
};

typedef struct video_scene_params_s video_scene_params_t;

struct video_parameter_name_s
{
  uint32_t id;
  char     *name;
};

typedef struct video_parameter_name_s video_parameter_name_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods. */

static int video_open(FAR struct file *filep);
static int video_close(FAR struct file *filep);
static int video_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/* Common function */

static int video_lock(FAR sem_t *sem);
static int video_unlock(FAR sem_t *sem);
static FAR video_type_inf_t *get_video_type_inf
           (FAR video_mng_t *vmng, uint8_t type);
static enum video_state_e estimate_next_video_state
            (FAR video_mng_t                   *vmng,
             enum video_state_transition_cause cause);
static void change_video_state(FAR video_mng_t    *vmng,
                               enum video_state_e next_state);
static bool is_taking_still_picture(FAR video_mng_t *vmng);
static bool is_bufsize_sufficient(FAR video_mng_t *vmng, uint32_t bufsize);
static void cleanup_resources(FAR video_mng_t *vmng);
static bool is_sem_waited(FAR sem_t *sem);
static int save_scene_param(enum v4l2_scene_mode mode,
                            uint32_t id,
                            struct v4l2_ext_control *control);
static int video_complete_capture(uint8_t  err_code, uint32_t datasize);

/* internal function for each cmds of ioctl */

static int video_reqbufs(FAR struct video_mng_s *vmng,
                         FAR struct v4l2_requestbuffers *reqbufs);
static int video_qbuf(FAR struct video_mng_s *vmng,
                      FAR struct v4l2_buffer *buf);
static int video_dqbuf(FAR struct video_mng_s *vmng,
                       FAR struct v4l2_buffer *buf);
static int video_cancel_dqbuf(FAR struct video_mng_s *vmng,
                              enum v4l2_buf_type type);
static int video_s_fmt(FAR struct video_mng_s *priv,
                       FAR struct v4l2_format *fmt);
static int video_s_parm(FAR struct video_mng_s *priv,
                        FAR struct v4l2_streamparm *parm);
static int video_streamon(FAR struct video_mng_s *vmng,
                          FAR enum v4l2_buf_type *type);
static int video_streamoff(FAR struct video_mng_s *vmng,
                           FAR enum v4l2_buf_type *type);
static int video_do_halfpush(FAR struct video_mng_s *priv,
                             bool enable);
static int video_takepict_start(FAR struct video_mng_s *vmng,
                                int32_t                capture_num);
static int video_takepict_stop(FAR struct video_mng_s *vmng,
                               bool halfpush);
static int video_queryctrl(FAR struct v4l2_queryctrl *ctrl);
static int video_query_ext_ctrl(FAR struct v4l2_query_ext_ctrl *ctrl);
static int video_querymenu(FAR struct v4l2_querymenu *menu);
static int video_g_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl);
static int video_s_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl);
static int video_g_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls);
static int video_s_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls);
static int video_query_ext_ctrl_scene(FAR struct v4s_query_ext_ctrl_scene
                                      *ctrl);
static int video_querymenu_scene(FAR struct v4s_querymenu_scene *menu);
static int video_g_ext_ctrls_scene(FAR struct v4s_ext_controls_scene *ctrls);
static int video_s_ext_ctrls_scene(FAR struct v4s_ext_controls_scene *ctrls);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_video_fops =
{
  video_open,               /* open */
  video_close,              /* close */
  NULL,                     /* read */
  NULL,                     /* write */
  NULL,                     /* seek */
  video_ioctl,              /* ioctl */
  NULL                      /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                    /* unlink */
#endif
};

static bool is_initialized = false;

enum v4l2_scene_mode g_video_scene_mode = V4L2_SCENE_MODE_NONE;
video_scene_params_t g_video_scene_parameter[] =
{
    {
      .mode = V4L2_SCENE_MODE_NONE,
    },
#ifdef CONFIG_VIDEO_SCENE_BACKLIGHT
    {
      .mode = V4L2_SCENE_MODE_BACKLIGHT,
    },
#endif /* CONFIG_VIDEO_SCENE_BACKLIGHT */
#ifdef CONFIG_VIDEO_SCENE_BEACHSNOW
    {
      .mode = V4L2_SCENE_MODE_BEACH_SNOW,
    },
#endif /* CONFIG_VIDEO_SCENE_BEACHSNOW */
#ifdef CONFIG_VIDEO_SCENE_CANDLELIGHT
    {
      .mode = V4L2_SCENE_MODE_CANDLE_LIGHT,
    },
#endif /* CONFIG_VIDEO_SCENE_CANDLELIGHT */
#ifdef CONFIG_VIDEO_SCENE_DAWNDUSK
    {
      .mode = V4L2_SCENE_MODE_DAWN_DUSK,
    },
#endif /* CONFIG_VIDEO_SCENE_DAWNDUSK */
#ifdef CONFIG_VIDEO_SCENE_FALLCOLORS
    {
      .mode = V4L2_SCENE_MODE_FALL_COLORS,
    },
#endif /* CONFIG_VIDEO_SCENE_FALLCOLORS */
#ifdef CONFIG_VIDEO_SCENE_FIREWORKS
    {
      .mode = V4L2_SCENE_MODE_FIREWORKS,
    },
#endif /* CONFIG_VIDEO_SCENE_FIREWORKS */
#ifdef CONFIG_VIDEO_SCENE_LANDSCAPE
    {
      .mode = V4L2_SCENE_MODE_LANDSCAPE,
    },
#endif /* CONFIG_VIDEO_SCENE_LANDSCAPE */
#ifdef CONFIG_VIDEO_SCENE_NIGHT
    {
      .mode = V4L2_SCENE_MODE_NIGHT,
    },
#endif /* CONFIG_VIDEO_SCENE_NIGHT */
#ifdef CONFIG_VIDEO_SCENE_PARTYINDOOR
    {
      .mode = V4L2_SCENE_MODE_PARTY_INDOOR,
    },
#endif /* CONFIG_VIDEO_SCENE_PARTYINDOOR */
#ifdef CONFIG_VIDEO_SCENE_PORTRAIT
    {
      .mode = V4L2_SCENE_MODE_PORTRAIT,
    },
#endif /* CONFIG_VIDEO_SCENE_PORTRAIT */
#ifdef CONFIG_VIDEO_SCENE_SPORTS
    {
      .mode = V4L2_SCENE_MODE_SPORTS,
    },
#endif /* CONFIG_VIDEO_SCENE_SPORTS */
#ifdef CONFIG_VIDEO_SCENE_SUNSET
    {
      .mode = V4L2_SCENE_MODE_SUNSET,
    },
#endif /* CONFIG_VIDEO_SCENE_SUNSET */
#ifdef CONFIG_VIDEO_SCENE_TEXT
    {
      .mode = V4L2_SCENE_MODE_TEXT,
    },
#endif /* CONFIG_VIDEO_SCENE_TEXT */
};

static video_parameter_name_t g_video_parameter_name[] =
{
  {IMGSENSOR_ID_BRIGHTNESS,           "Brightness"},
  {IMGSENSOR_ID_CONTRAST,             "Contrast"},
  {IMGSENSOR_ID_SATURATION,           "Saturation"},
  {IMGSENSOR_ID_HUE,                  "Hue"},
  {IMGSENSOR_ID_AUTO_WHITE_BALANCE,   "Automatic white balance"},
  {IMGSENSOR_ID_RED_BALANCE,          "Red balance"},
  {IMGSENSOR_ID_BLUE_BALANCE,         "Blue balance"},
  {IMGSENSOR_ID_GAMMA,                "Gamma value"},
  {IMGSENSOR_ID_GAMMA_CURVE,          "Gamma adjustment(curve)"},
  {IMGSENSOR_ID_EXPOSURE,             "Exposure value"},
  {IMGSENSOR_ID_HFLIP_VIDEO,          "Mirror horizontally(VIDEO)"},
  {IMGSENSOR_ID_VFLIP_VIDEO,          "Mirror vertically(VIDEO)"},
  {IMGSENSOR_ID_HFLIP_STILL,          "Mirror horizontally(STILL)"},
  {IMGSENSOR_ID_VFLIP_STILL,          "Mirror vertically(STILL)"},
  {IMGSENSOR_ID_SHARPNESS,            "Sharpness"},
  {IMGSENSOR_ID_COLOR_KILLER,         "Color killer"},
  {IMGSENSOR_ID_COLORFX,              "Color effect"},
  {IMGSENSOR_ID_AUTOBRIGHTNESS,       "Auto brightness"},
  {IMGSENSOR_ID_ROTATE,               "Rotate"},
  {IMGSENSOR_ID_EXPOSURE_AUTO,        "Auto Exposure"},
  {IMGSENSOR_ID_EXPOSURE_ABSOLUTE,    "Exposure time(100 usec)"},
  {IMGSENSOR_ID_FOCUS_ABSOLUTE,       "Focus(absolute value)"},
  {IMGSENSOR_ID_FOCUS_RELATIVE,       "Focus(relative value)"},
  {IMGSENSOR_ID_FOCUS_AUTO,           "Continuous Auto Focus"},
  {IMGSENSOR_ID_ZOOM_ABSOLUTE,        "Zoom(absolute value)"},
  {IMGSENSOR_ID_ZOOM_RELATIVE,        "Zoom(relative value)"},
  {IMGSENSOR_ID_ZOOM_CONTINUOUS,      "Continuous zoom"},
  {IMGSENSOR_ID_IRIS_ABSOLUTE,        "Iris(absolute value)"},
  {IMGSENSOR_ID_IRIS_RELATIVE,        "Iris(relative value)"},
  {IMGSENSOR_ID_AUTO_N_PRESET_WB,     "Preset white balance"},
  {IMGSENSOR_ID_WIDE_DYNAMIC_RANGE,   "Wide dynamic range"},
  {IMGSENSOR_ID_IMG_STABILIZATION,    "Image stabilization"},
  {IMGSENSOR_ID_ISO_SENSITIVITY,      "ISO sensitivity"},
  {IMGSENSOR_ID_ISO_SENSITIVITY_AUTO, "Automatic ISO sensitivity"},
  {IMGSENSOR_ID_EXPOSURE_METERING,    "Photometry"},
  {IMGSENSOR_ID_3A_LOCK,              "Lock AWB/AE"},
  {IMGSENSOR_ID_AUTO_FOCUS_START,     "Start single Auto Focus"},
  {IMGSENSOR_ID_AUTO_FOCUS_STOP,      "Stop single Auto Focus"},
  {IMGSENSOR_ID_3A_PARAMETER,         "3A parameter"},
  {IMGSENSOR_ID_3A_STATUS,            "3A status"},
  {IMGSENSOR_ID_FLASH_LED_MODE,       "LED mode"},
  {IMGSENSOR_ID_JPEG_QUALITY,         "JPEG compression quality"}
};

static FAR void *video_handler;
static FAR const struct imgsensor_ops_s *g_video_sensor_ops;
static FAR const struct imgdata_ops_s *g_video_data_ops;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int video_lock(FAR sem_t *sem)
{
  if (sem == NULL)
    {
      return -EINVAL;
    }

  return nxsem_wait_uninterruptible(sem);
}

static int video_unlock(FAR sem_t *sem)
{
  if (sem == NULL)
    {
      return -EINVAL;
    }

  return nxsem_post(sem);
}

static FAR video_type_inf_t *get_video_type_inf
(FAR video_mng_t *vmng, uint8_t type)
{
  FAR video_type_inf_t *type_inf;

  switch (type)
    {
      case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        type_inf = &vmng->video_inf;
        break;

      case V4L2_BUF_TYPE_STILL_CAPTURE:
        type_inf = &vmng->still_inf;
        break;

      default:  /* Error case */
        type_inf = NULL;
        break;
    }

  return type_inf;
}

static enum video_state_e estimate_next_video_state
            (FAR video_mng_t                   *vmng,
             enum video_state_transition_cause cause)
{
  enum video_state_e current_state = vmng->video_inf.state;

  switch (cause)
    {
      case CAUSE_VIDEO_STOP:
        return VIDEO_STATE_STREAMOFF;

      case CAUSE_VIDEO_START:
        if (is_taking_still_picture(vmng))
          {
            return VIDEO_STATE_STREAMON;
          }
        else
          {
            return VIDEO_STATE_CAPTURE;
          }

      case CAUSE_STILL_STOP:
        if (current_state == VIDEO_STATE_STREAMON)
          {
            return VIDEO_STATE_CAPTURE;
          }
        else
          {
            return current_state;
          }

      case CAUSE_STILL_START:
        if (current_state == VIDEO_STATE_CAPTURE)
          {
            return VIDEO_STATE_STREAMON;
          }
        else
          {
            return current_state;
          }

      case CAUSE_VIDEO_DQBUF:
        if ((current_state == VIDEO_STATE_STREAMON) &&
             !is_taking_still_picture(vmng))
          {
            return VIDEO_STATE_CAPTURE;
          }
        else
          {
            return current_state;
          }

      default:
        return current_state;
    }
}

static void convert_to_imgdatafmt(FAR video_format_t *video,
                                  FAR imgdata_format_t *data)
{
  ASSERT(video && data);

  data->width       = video->width;
  data->height      = video->height;
  switch (video->pixelformat)
    {
      case V4L2_PIX_FMT_UYVY :
        data->pixelformat = IMGDATA_PIX_FMT_UYVY;
        break;

      case V4L2_PIX_FMT_RGB565 :
        data->pixelformat = IMGDATA_PIX_FMT_RGB565;
        break;

      case V4L2_PIX_FMT_JPEG :
        data->pixelformat = IMGDATA_PIX_FMT_JPEG;
        break;

      default : /* V4L2_PIX_FMT_JPEG_WITH_SUBIMG */
        data->pixelformat = IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG;
        break;
    }
}

static void convert_to_imgsensorfmt(FAR video_format_t *video,
                                    FAR imgsensor_format_t *sensor)
{
  ASSERT(video && sensor);

  sensor->width       = video->width;
  sensor->height      = video->height;
  switch (video->pixelformat)
    {
      case V4L2_PIX_FMT_UYVY :
        sensor->pixelformat = IMGSENSOR_PIX_FMT_UYVY;
        break;

      case V4L2_PIX_FMT_RGB565 :
        sensor->pixelformat = IMGSENSOR_PIX_FMT_RGB565;
        break;

      case V4L2_PIX_FMT_JPEG :
        sensor->pixelformat = IMGSENSOR_PIX_FMT_JPEG;
        break;

      default : /* V4L2_PIX_FMT_JPEG_WITH_SUBIMG */
        sensor->pixelformat = IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG;
        break;
    }
}

static void convert_to_imgdatainterval(FAR struct v4l2_fract *video,
                                       FAR imgdata_interval_t *data)
{
  ASSERT(video && data);

  data->numerator   = video->numerator;
  data->denominator = video->denominator;
}

static void convert_to_imgsensorinterval(FAR struct v4l2_fract *video,
                                         FAR imgsensor_interval_t *sensor)
{
  ASSERT(video && sensor);

  sensor->numerator   = video->numerator;
  sensor->denominator = video->denominator;
}

static int start_capture(enum v4l2_buf_type type,
                         uint8_t nr_fmt,
                         FAR video_format_t *fmt,
                         FAR struct v4l2_fract *interval,
                         uint32_t bufaddr, uint32_t bufsize)
{
  imgdata_format_t df[MAX_VIDEO_FMT];
  imgsensor_format_t sf[MAX_VIDEO_FMT];
  imgdata_interval_t di;
  imgsensor_interval_t si;

  ASSERT(fmt && interval && g_video_sensor_ops && g_video_data_ops);

  if ((g_video_sensor_ops->start_capture == NULL) ||
      (g_video_data_ops->start_capture == NULL) ||
      (g_video_data_ops->set_buf == NULL))
    {
      return -ENOTTY;
    }

  convert_to_imgdatafmt(&fmt[VIDEO_FMT_MAIN], &df[IMGDATA_FMT_MAIN]);
  convert_to_imgdatafmt(&fmt[VIDEO_FMT_SUB], &df[IMGDATA_FMT_SUB]);
  convert_to_imgdatainterval(interval, &di);
  convert_to_imgsensorfmt(&fmt[VIDEO_FMT_MAIN], &sf[IMGSENSOR_FMT_MAIN]);
  convert_to_imgsensorfmt(&fmt[VIDEO_FMT_SUB], &sf[IMGSENSOR_FMT_SUB]);
  convert_to_imgsensorinterval(interval, &si);

  g_video_sensor_ops->start_capture
    ((type == V4L2_BUF_TYPE_VIDEO_CAPTURE) ?
     IMGSENSOR_STREAM_TYPE_VIDEO : IMGSENSOR_STREAM_TYPE_STILL,
     nr_fmt,
     sf,
     &si);
  g_video_data_ops->start_capture(nr_fmt, df, &di, video_complete_capture);
  g_video_data_ops->set_buf((uint8_t *)bufaddr, bufsize);

  return OK;
}

static void change_video_state(FAR video_mng_t    *vmng,
                               enum video_state_e next_state)
{
  enum video_state_e current_state = vmng->video_inf.state;
  enum video_state_e updated_next_state = next_state;
  FAR vbuf_container_t *container;

  if ((current_state != VIDEO_STATE_CAPTURE) &&
      (next_state    == VIDEO_STATE_CAPTURE))
    {
      container =
              video_framebuff_get_vacant_container(&vmng->video_inf.bufinf);
      if (container)
        {
          start_capture(V4L2_BUF_TYPE_VIDEO_CAPTURE,
                        vmng->video_inf.nr_fmt,
                        vmng->video_inf.fmt,
                        &vmng->video_inf.frame_interval,
                        container->buf.m.userptr,
                        container->buf.length);
        }
      else
        {
          updated_next_state = VIDEO_STATE_STREAMON;
        }
    }
  else
    {
      if ((current_state == VIDEO_STATE_CAPTURE) &&
          (next_state    != VIDEO_STATE_CAPTURE))
        {
          g_video_data_ops->stop_capture();
        }
    }

  vmng->video_inf.state = updated_next_state;

  return;
}

static bool is_taking_still_picture(FAR video_mng_t *vmng)
{
  return ((vmng->still_inf.state == VIDEO_STATE_STREAMON) ||
          (vmng->still_inf.state == VIDEO_STATE_CAPTURE));
}

static bool is_bufsize_sufficient(FAR video_mng_t *vmng, uint32_t bufsize)
{
  /* Depend on format, frame size, and JPEG compression quality */

  return true;
}

static void initialize_frame_setting(FAR uint8_t *nr_fmt,
                                     FAR video_format_t *fmt,
                                     FAR struct v4l2_fract *interval)
{
  ASSERT(nr_fmt && fmt && interval);

  /* Initial setting : QVGA YUV4:2:2 15FPS */

  *nr_fmt = 1;
  fmt[VIDEO_FMT_MAIN].width       = VIDEO_HSIZE_QVGA;
  fmt[VIDEO_FMT_MAIN].height      = VIDEO_VSIZE_QVGA;
  fmt[VIDEO_FMT_MAIN].pixelformat = V4L2_PIX_FMT_UYVY;
  interval->denominator = 15;
  interval->numerator   = 1;
}

static void initialize_streamresources(FAR video_type_inf_t *type_inf)
{
  memset(type_inf, 0, sizeof(video_type_inf_t));
  type_inf->remaining_capnum = VIDEO_REMAINING_CAPNUM_INFINITY;
  nxsem_init(&type_inf->lock_state, 0, 1);
  nxsem_init(&type_inf->wait_capture.dqbuf_wait_flg, 0, 0);
  initialize_frame_setting(&type_inf->nr_fmt,
                           type_inf->fmt,
                           &type_inf->frame_interval);
  video_framebuff_init(&type_inf->bufinf);

  return;
}

static int32_t get_default_value(uint32_t id)
{
  int ret;
  imgsensor_supported_value_t value;

  if ((g_video_sensor_ops == NULL) ||
      (g_video_sensor_ops->get_supported_value == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_sensor_ops->get_supported_value(id, &value);
  if (ret != OK)
    {
      /* Don't care(unsupported parameter) */

      return 0;
    }

  switch (value.type)
    {
      case IMGSENSOR_CTRL_TYPE_INTEGER_MENU:
        return value.u.discrete.default_value;

      case IMGSENSOR_CTRL_TYPE_U8:
      case IMGSENSOR_CTRL_TYPE_U16:
      case IMGSENSOR_CTRL_TYPE_U32:

        /* Don't care */

        return 0;

      default:
        return value.u.range.default_value;
    }
}

static int32_t initialize_scene_gamma(uint8_t **gamma)
{
  int ret;
  imgsensor_supported_value_t sup_val;
  imgsensor_value_t val;
  int32_t sz;

  *gamma = NULL;

  ASSERT(g_video_sensor_ops);

  if ((g_video_sensor_ops->get_supported_value == NULL) ||
      (g_video_sensor_ops->get_value == NULL))
    {
      return -ENOTTY;
    }

  ret = g_video_sensor_ops->get_supported_value
          (IMGSENSOR_ID_GAMMA_CURVE, &sup_val);
  if (ret != OK)
    {
      /* Unsupported parameter */

      return -EINVAL;
    }

  switch (sup_val.type)
    {
      case IMGSENSOR_CTRL_TYPE_U8:
        sz = sup_val.u.elems.nr_elems * sizeof(uint8_t);
        if (sz / sizeof(uint8_t) != sup_val.u.elems.nr_elems)
          {
            /* Multiplication overflow */

            return -EINVAL;
          }

        break;

      case IMGSENSOR_CTRL_TYPE_U16:
        sz = sup_val.u.elems.nr_elems * sizeof(uint16_t);
        if (sz / sizeof(uint16_t) != sup_val.u.elems.nr_elems)
          {
            /* Multiplication overflow */

            return -EINVAL;
          }

        break;

      default: /* IMGSENSOR_CTRL_TYPE_U32 */
        sz = sup_val.u.elems.nr_elems * sizeof(uint32_t);
        if (sz / sizeof(uint32_t) != sup_val.u.elems.nr_elems)
          {
            /* Multiplication overflow */

            return -EINVAL;
          }

        break;
    }

  *gamma = malloc(sz);
  val.p_u8 = (uint8_t *)*gamma;
  g_video_sensor_ops->get_value(IMGSENSOR_ID_GAMMA_CURVE, sz, &val);
  return sz;
}

static void initialize_scene_parameter(video_scene_params_t *sp)
{
  ASSERT(sp);

  sp->brightness      = get_default_value(IMGSENSOR_ID_BRIGHTNESS);
  sp->contrast        = get_default_value(IMGSENSOR_ID_CONTRAST);
  sp->saturation      = get_default_value(IMGSENSOR_ID_SATURATION);
  sp->hue             = get_default_value(IMGSENSOR_ID_HUE);
  sp->awb             = get_default_value(IMGSENSOR_ID_AUTO_WHITE_BALANCE);
  sp->red             = get_default_value(IMGSENSOR_ID_RED_BALANCE);
  sp->blue            = get_default_value(IMGSENSOR_ID_BLUE_BALANCE);
  sp->gamma           = get_default_value(IMGSENSOR_ID_GAMMA);
  sp->gamma_curve_sz  = initialize_scene_gamma(&sp->gamma_curve);
  sp->ev              = get_default_value(IMGSENSOR_ID_EXPOSURE);
  sp->hflip_video     = get_default_value(IMGSENSOR_ID_HFLIP_VIDEO);
  sp->vflip_video     = get_default_value(IMGSENSOR_ID_VFLIP_VIDEO);
  sp->hflip_still     = get_default_value(IMGSENSOR_ID_HFLIP_STILL);
  sp->vflip_still     = get_default_value(IMGSENSOR_ID_VFLIP_STILL);
  sp->sharpness       = get_default_value(IMGSENSOR_ID_SHARPNESS);
  sp->colorfx         = get_default_value(IMGSENSOR_ID_COLORFX);
  sp->auto_brightness = get_default_value(IMGSENSOR_ID_AUTOBRIGHTNESS);
  sp->rotate          = get_default_value(IMGSENSOR_ID_ROTATE);
  sp->ae              = get_default_value(IMGSENSOR_ID_EXPOSURE_AUTO);
  sp->exposure_time   = get_default_value(IMGSENSOR_ID_EXPOSURE_ABSOLUTE);
  sp->focus           = get_default_value(IMGSENSOR_ID_FOCUS_ABSOLUTE);
  sp->af              = get_default_value(IMGSENSOR_ID_FOCUS_AUTO);
  sp->zoom            = get_default_value(IMGSENSOR_ID_ZOOM_ABSOLUTE);
  sp->iris            = get_default_value(IMGSENSOR_ID_IRIS_ABSOLUTE);
  sp->wb              = get_default_value(IMGSENSOR_ID_AUTO_N_PRESET_WB);
  sp->wdr             = get_default_value(IMGSENSOR_ID_WIDE_DYNAMIC_RANGE);
  sp->stabilization   = get_default_value(IMGSENSOR_ID_IMG_STABILIZATION);
  sp->iso_auto        = get_default_value(IMGSENSOR_ID_ISO_SENSITIVITY_AUTO);
  sp->iso             = get_default_value(IMGSENSOR_ID_ISO_SENSITIVITY);
  sp->meter           = get_default_value(IMGSENSOR_ID_EXPOSURE_METERING);
  sp->threea_lock     = get_default_value(IMGSENSOR_ID_3A_LOCK);
  sp->led             = get_default_value(IMGSENSOR_ID_FLASH_LED_MODE);
  sp->jpeg_quality    = get_default_value(IMGSENSOR_ID_JPEG_QUALITY);
}

static void initialize_scenes_parameter(void)
{
  int i;
  video_scene_params_t *sp = &g_video_scene_parameter[0];

  for (i = 0; i < VIDEO_SCENE_MAX; i++)
    {
      initialize_scene_parameter(sp++);
    }
}

static void initialize_resources(FAR video_mng_t *vmng)
{
  initialize_streamresources(&vmng->video_inf);
  initialize_streamresources(&vmng->still_inf);
  initialize_scenes_parameter();

  return;
}

static void cleanup_streamresources(FAR video_type_inf_t *type_inf)
{
  video_framebuff_uninit(&type_inf->bufinf);
  nxsem_destroy(&type_inf->wait_capture.dqbuf_wait_flg);
  nxsem_destroy(&type_inf->lock_state);
  memset(type_inf, 0, sizeof(video_type_inf_t));
  type_inf->remaining_capnum = VIDEO_REMAINING_CAPNUM_INFINITY;

  return;
}

static void cleanup_scene_parameter(video_scene_params_t *sp)
{
  ASSERT(sp);

  if (sp->gamma_curve)
    {
      free(sp->gamma_curve);
    }
}

static void cleanup_scenes_parameter(void)
{
  int i;
  video_scene_params_t *sp = &g_video_scene_parameter[0];

  for (i = 0; i < VIDEO_SCENE_MAX; i++, sp++)
    {
      cleanup_scene_parameter(sp);
    }
}

static void cleanup_resources(FAR video_mng_t *vmng)
{
  /* clean up resource */

  if ((vmng->video_inf.state == VIDEO_STATE_CAPTURE) ||
      (vmng->still_inf.state == VIDEO_STATE_CAPTURE))
    {
      /* If in capture, stop */

      g_video_data_ops->stop_capture();
    }

  cleanup_streamresources(&vmng->video_inf);
  cleanup_streamresources(&vmng->still_inf);
  cleanup_scenes_parameter();

  return;
}

static bool is_sem_waited(FAR sem_t *sem)
{
  int ret;
  int semcount;

  ret = nxsem_get_value(sem, &semcount);
  if ((ret == OK) && (semcount < 0))
    {
      return true;
    }
  else
    {
      return false;
    }
}

static int video_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR video_mng_t  *priv  = (FAR video_mng_t *)inode->i_private;
  int ret = OK;

  video_lock(&priv->lock_open_num);
  if (priv->open_num == 0)
    {
      /* Only in first execution, open device */

      ret = g_video_sensor_ops->init();
      if (ret == OK)
        {
          ret = g_video_data_ops->init();
          if (ret == OK)
            {
              initialize_resources(priv);
            }
        }
    }

  /* In second or later execution, ret is initial value(=OK) */

  if (ret == OK)
    {
      priv->open_num++;
    }

  video_unlock(&priv->lock_open_num);

  return ret;
}

static int video_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR video_mng_t  *priv  = (FAR video_mng_t *)inode->i_private;
  int ret = ERROR;

  video_lock(&priv->lock_open_num);
  if (priv->open_num == 0)
    {
      return OK;
    }

  priv->open_num--;

  if (priv->open_num == 0)
    {
      cleanup_resources(priv);
      g_video_sensor_ops->uninit();
      g_video_data_ops->uninit();
    }

  video_unlock(&priv->lock_open_num);

  return ret;
}

static int video_reqbufs(FAR struct video_mng_s         *vmng,
                         FAR struct v4l2_requestbuffers *reqbufs)
{
  int ret = OK;
  FAR video_type_inf_t *type_inf;
  irqstate_t           flags;

  if ((vmng == NULL) || (reqbufs == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(vmng, reqbufs->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  if (type_inf->state == VIDEO_STATE_CAPTURE)
    {
      /* In capture, REQBUFS is not permitted */

      ret = -EPERM;
    }
  else
    {
      video_framebuff_change_mode(&type_inf->bufinf, reqbufs->mode);

      ret = video_framebuff_realloc_container(&type_inf->bufinf,
                                              reqbufs->count);
    }

  leave_critical_section(flags);

  return ret;
}

static int video_qbuf(FAR struct video_mng_s *vmng,
                      FAR struct v4l2_buffer *buf)
{
  FAR video_type_inf_t *type_inf;
  FAR vbuf_container_t *container;
  enum video_state_e   next_video_state;
  irqstate_t           flags;

  if ((vmng == NULL) || (buf == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(vmng, buf->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (!is_bufsize_sufficient(vmng, buf->length))
    {
      return -EINVAL;
    }

  container = video_framebuff_get_container(&type_inf->bufinf);
  if (container == NULL)
    {
      return -ENOMEM;
    }

  memcpy(&container->buf, buf, sizeof(struct v4l2_buffer));
  video_framebuff_queue_container(&type_inf->bufinf, container);

  video_lock(&type_inf->lock_state);
  flags = enter_critical_section();
  if (type_inf->state == VIDEO_STATE_STREAMON)
    {
      leave_critical_section(flags);

      if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
        {
          video_lock(&vmng->still_inf.lock_state);
          next_video_state = estimate_next_video_state
                             (vmng, CAUSE_VIDEO_START);
          change_video_state(vmng, next_video_state);
          video_unlock(&vmng->still_inf.lock_state);
        }
      else
        {
          container = video_framebuff_get_vacant_container
                      (&type_inf->bufinf);
          if (container)
            {
              start_capture(buf->type,
                            type_inf->nr_fmt,
                            type_inf->fmt,
                            &type_inf->frame_interval,
                            container->buf.m.userptr,
                            container->buf.length);
              type_inf->state = VIDEO_STATE_CAPTURE;
            }
        }
    }
  else
    {
      leave_critical_section(flags);
    }

  video_unlock(&type_inf->lock_state);

  return OK;
}

static int video_dqbuf(FAR struct video_mng_s *vmng,
                       FAR struct v4l2_buffer *buf)
{
  irqstate_t           flags;
  FAR video_type_inf_t *type_inf;
  FAR vbuf_container_t *container;
  sem_t                *dqbuf_wait_flg;
  enum video_state_e   next_video_state;

  if ((vmng == NULL) || (buf == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(vmng, buf->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  container = video_framebuff_dq_valid_container(&type_inf->bufinf);
  if (container == NULL)
    {
      /* Not yet done capture. Wait done */

      dqbuf_wait_flg = &type_inf->wait_capture.dqbuf_wait_flg;

      /* Loop until semaphore is unlocked by capture done or DQCANCEL */

      do
        {
          if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
            {
              /* If start capture condition is satisfied, start capture */

              flags = enter_critical_section();
              next_video_state = estimate_next_video_state
                                  (vmng, CAUSE_VIDEO_DQBUF);
              change_video_state(vmng, next_video_state);
              leave_critical_section(flags);
            }

          nxsem_wait_uninterruptible(dqbuf_wait_flg);
        }
      while (type_inf->wait_capture.waitend_cause ==
                   VIDEO_WAITEND_CAUSE_STILLSTOP);

      container = type_inf->wait_capture.done_container;

      if (!container)
        {
          /* Waking up without captured data means abort.
           * Therefore, Check cause.
           */

          if (type_inf->wait_capture.waitend_cause
               == VIDEO_WAITEND_CAUSE_DQCANCEL)
            {
              return -ECANCELED;
            }
        }

      type_inf->wait_capture.done_container = NULL;
    }

  memcpy(buf, &container->buf, sizeof(struct v4l2_buffer));

  video_framebuff_free_container(&type_inf->bufinf, container);

  return OK;
}

static int video_cancel_dqbuf(FAR struct video_mng_s *vmng,
                              enum v4l2_buf_type type)
{
  FAR video_type_inf_t *type_inf;

  type_inf = get_video_type_inf(vmng, type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (!is_sem_waited(&type_inf->wait_capture.dqbuf_wait_flg))
    {
      /* In not waiting DQBUF case, return OK */

      return OK;
    }

  type_inf->wait_capture.waitend_cause = VIDEO_WAITEND_CAUSE_DQCANCEL;

  /* If capture is done before nxsem_post, cause is overwritten */

  nxsem_post(&type_inf->wait_capture.dqbuf_wait_flg);

  return OK;
}

static int validate_frame_setting(enum v4l2_buf_type type,
                                  uint8_t nr_fmt,
                                  FAR video_format_t *vfmt,
                                  FAR struct v4l2_fract *interval)
{
  int ret;
  imgdata_format_t df[MAX_VIDEO_FMT];
  imgsensor_format_t sf[MAX_VIDEO_FMT];
  imgdata_interval_t di;
  imgsensor_interval_t si;

  ASSERT(vfmt && interval && g_video_sensor_ops && g_video_data_ops);

  if ((g_video_sensor_ops->validate_frame_setting == NULL) ||
      (g_video_data_ops->validate_frame_setting == NULL))
    {
      return -ENOTTY;
    }

  /* Return OK only in case both image data driver and
   * image sensor driver support.
   */

  convert_to_imgdatafmt(&vfmt[VIDEO_FMT_MAIN], &df[IMGDATA_FMT_MAIN]);
  convert_to_imgdatafmt(&vfmt[VIDEO_FMT_SUB], &df[IMGDATA_FMT_SUB]);
  convert_to_imgdatainterval(interval, &di);
  convert_to_imgsensorfmt(&vfmt[VIDEO_FMT_MAIN], &sf[IMGSENSOR_FMT_MAIN]);
  convert_to_imgsensorfmt(&vfmt[VIDEO_FMT_SUB], &sf[IMGSENSOR_FMT_SUB]);
  convert_to_imgsensorinterval(interval, &si);

  ret = g_video_sensor_ops->validate_frame_setting(type, nr_fmt, sf, &si);
  if (ret != OK)
    {
      return ret;
    }

  return g_video_data_ops->validate_frame_setting(nr_fmt, df, &di);
}

static int video_try_fmt(FAR struct video_mng_s *priv,
                         FAR struct v4l2_format *v4l2)
{
  FAR video_type_inf_t *type_inf;
  uint8_t nr_fmt;
  video_format_t vf[MAX_VIDEO_FMT];

  ASSERT(priv && g_video_sensor_ops && g_video_data_ops);

  if ((g_video_sensor_ops->validate_frame_setting == NULL) ||
      (g_video_data_ops->validate_frame_setting == NULL))
    {
      return -ENOTTY;
    }

  if (v4l2 == NULL)
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(priv, v4l2->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  switch (v4l2->fmt.pix.pixelformat)
    {
      case V4L2_PIX_FMT_SUBIMG_UYVY:
      case V4L2_PIX_FMT_SUBIMG_RGB565:
        if (type_inf->fmt[VIDEO_FMT_MAIN].pixelformat
              != V4L2_PIX_FMT_JPEG_WITH_SUBIMG)
          {
            return -EPERM;
          }

        /* Validate both main image and subimage. */

        nr_fmt = 2;
        memcpy(&vf[VIDEO_FMT_MAIN],
               &type_inf->fmt[VIDEO_FMT_MAIN],
               sizeof(video_format_t));
        vf[VIDEO_FMT_SUB].width       = v4l2->fmt.pix.width;
        vf[VIDEO_FMT_SUB].height      = v4l2->fmt.pix.height;
        vf[VIDEO_FMT_SUB].pixelformat
          = (v4l2->fmt.pix.pixelformat == V4L2_PIX_FMT_SUBIMG_UYVY) ?
              V4L2_PIX_FMT_UYVY : V4L2_PIX_FMT_RGB565;

        break;

      case V4L2_PIX_FMT_UYVY:
      case V4L2_PIX_FMT_RGB565:
      case V4L2_PIX_FMT_JPEG:
      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:
        nr_fmt = 1;
        vf[VIDEO_FMT_MAIN].width       = v4l2->fmt.pix.width;
        vf[VIDEO_FMT_MAIN].height      = v4l2->fmt.pix.height;
        vf[VIDEO_FMT_MAIN].pixelformat = v4l2->fmt.pix.pixelformat;

        break;

      default:
        return -EINVAL;
    }

  return validate_frame_setting(v4l2->type,
                                nr_fmt,
                                vf,
                                &type_inf->frame_interval);
}

static int video_s_fmt(FAR struct video_mng_s *priv,
                       FAR struct v4l2_format *fmt)
{
  int ret;
  FAR video_type_inf_t *type_inf;

  ret = video_try_fmt(priv, fmt);
  if (ret != 0)
    {
      return ret;
    }

  type_inf = get_video_type_inf(priv, fmt->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (type_inf->state != VIDEO_STATE_STREAMOFF)
    {
      return -EBUSY;
    }

  switch (fmt->fmt.pix.pixelformat)
    {
      case V4L2_PIX_FMT_SUBIMG_UYVY:
      case V4L2_PIX_FMT_SUBIMG_RGB565:
        if (type_inf->fmt[VIDEO_FMT_MAIN].pixelformat
              != V4L2_PIX_FMT_JPEG_WITH_SUBIMG)
          {
            return -EPERM;
          }

        type_inf->fmt[VIDEO_FMT_SUB].width  = fmt->fmt.pix.width;
        type_inf->fmt[VIDEO_FMT_SUB].height = fmt->fmt.pix.height;
        type_inf->fmt[VIDEO_FMT_SUB].pixelformat
          = (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_SUBIMG_UYVY) ?
              V4L2_PIX_FMT_UYVY : V4L2_PIX_FMT_RGB565;
        type_inf->nr_fmt = 2;
        break;

      default:
        type_inf->fmt[VIDEO_FMT_MAIN].width  = fmt->fmt.pix.width;
        type_inf->fmt[VIDEO_FMT_MAIN].height = fmt->fmt.pix.height;
        type_inf->fmt[VIDEO_FMT_MAIN].pixelformat
          = fmt->fmt.pix.pixelformat;
        type_inf->nr_fmt = 1;
        break;
    }

  return OK;
}

static int video_s_parm(FAR struct video_mng_s *priv,
                        FAR struct v4l2_streamparm *parm)
{
  int ret;
  FAR video_type_inf_t *type_inf;

  ASSERT(g_video_sensor_ops && g_video_data_ops);

  if ((g_video_sensor_ops->validate_frame_setting == NULL) ||
      (g_video_data_ops->validate_frame_setting == NULL) ||
      (parm == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(priv, parm->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (type_inf->state != VIDEO_STATE_STREAMOFF)
    {
      return -EBUSY;
    }

  ret = validate_frame_setting(parm->type,
                               type_inf->nr_fmt,
                               type_inf->fmt,
                               &parm->parm.capture.timeperframe);
  if (ret != OK)
    {
      return ret;
    }

  memcpy(&type_inf->frame_interval,
         &parm->parm.capture.timeperframe,
         sizeof(struct v4l2_fract));

  return ret;
}

static int video_streamon(FAR struct video_mng_s *vmng,
                          FAR enum v4l2_buf_type *type)
{
  FAR video_type_inf_t *type_inf;
  enum video_state_e   next_video_state;
  int                  ret = OK;

  if ((vmng == NULL) || (type == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(vmng, *type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      /* No procedure for VIDIOC_STREAMON(STILL_CAPTURE) */

      return OK;
    }

  video_lock(&type_inf->lock_state);

  if (type_inf->state != VIDEO_STATE_STREAMOFF)
    {
      ret = -EPERM;
    }
  else
    {
      next_video_state = estimate_next_video_state
                          (vmng, CAUSE_VIDEO_START);
      change_video_state(vmng, next_video_state);
    }

  video_unlock(&type_inf->lock_state);

  return ret;
}

static int video_streamoff(FAR struct video_mng_s *vmng,
                           FAR enum v4l2_buf_type *type)
{
  FAR video_type_inf_t *type_inf;
  enum video_state_e   next_video_state;
  irqstate_t           flags;
  int                  ret = OK;

  if ((vmng == NULL) || (type == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(vmng, *type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      /* No procedure for VIDIOC_STREAMOFF(STILL_CAPTURE) */

      return OK;
    }

  flags = enter_critical_section();

  if (type_inf->state == VIDEO_STATE_STREAMOFF)
    {
      ret = -EPERM;
    }
  else
    {
      next_video_state = estimate_next_video_state
                          (vmng, CAUSE_VIDEO_STOP);
      change_video_state(vmng, next_video_state);
    }

  leave_critical_section(flags);

  return ret;
}

static int video_do_halfpush(FAR struct video_mng_s *priv, bool enable)
{
  int ret;
  struct v4l2_ext_controls ext_controls;
  struct v4l2_ext_control  control[2];

  /* Replace to VIDIOC_S_EXT_CTRLS format */

  control[0].id    = V4L2_CID_3A_LOCK;
  control[0].value = enable ?
                     (V4L2_LOCK_EXPOSURE | V4L2_LOCK_WHITE_BALANCE) : 0;
  control[1].id    = V4L2_CID_AUTO_FOCUS_START;
  control[1].value = enable ? true : false;

  ext_controls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
  ext_controls.count      = 2;
  ext_controls.controls   = control;

  /* Execute VIDIOC_S_EXT_CTRLS */

  ret = video_s_ext_ctrls(priv, &ext_controls);

  return ret;
}

static int video_takepict_start(FAR struct video_mng_s *vmng,
                                int32_t capture_num)
{
  irqstate_t           flags;
  enum video_state_e   next_video_state;
  FAR vbuf_container_t *container;

  int                  ret = OK;

  if (vmng == NULL)
    {
      return -EINVAL;
    }

  video_lock(&vmng->still_inf.lock_state);

  if (vmng->still_inf.state != VIDEO_STATE_STREAMOFF)
    {
      ret = -EPERM;
    }
  else
    {
      if (capture_num > 0)
        {
          vmng->still_inf.remaining_capnum = capture_num;
        }
      else
        {
          vmng->still_inf.remaining_capnum = VIDEO_REMAINING_CAPNUM_INFINITY;
        }

      /* Control video stream prior to still stream */

      flags = enter_critical_section();

      next_video_state = estimate_next_video_state(vmng,
                                                   CAUSE_STILL_START);
      change_video_state(vmng, next_video_state);

      leave_critical_section(flags);

      container = video_framebuff_get_vacant_container
                             (&vmng->still_inf.bufinf);
      if (container)
        {
          /* Start still stream capture */

          start_capture(V4L2_BUF_TYPE_STILL_CAPTURE,
                        vmng->still_inf.nr_fmt,
                        vmng->still_inf.fmt,
                        &vmng->still_inf.frame_interval,
                        container->buf.m.userptr,
                        container->buf.length);

          vmng->still_inf.state = VIDEO_STATE_CAPTURE;
        }
      else
        {
          vmng->still_inf.state = VIDEO_STATE_STREAMON;
        }
    }

  video_unlock(&vmng->still_inf.lock_state);

  return ret;
}

static int video_takepict_stop(FAR struct video_mng_s *vmng, bool halfpush)
{
  int        ret = OK;
  irqstate_t flags;
  enum video_state_e next_video_state;

  if (vmng == NULL)
    {
      return -EINVAL;
    }

  video_lock(&vmng->still_inf.lock_state);

  if ((vmng->still_inf.state == VIDEO_STATE_STREAMOFF) &&
      (vmng->still_inf.remaining_capnum == VIDEO_REMAINING_CAPNUM_INFINITY))
    {
      ret = -EPERM;
    }
  else
    {
      flags = enter_critical_section();
      if (vmng->still_inf.state == VIDEO_STATE_CAPTURE)
        {
          g_video_data_ops->stop_capture();
        }

      leave_critical_section(flags);

      vmng->still_inf.state = VIDEO_STATE_STREAMOFF;
      vmng->still_inf.remaining_capnum = VIDEO_REMAINING_CAPNUM_INFINITY;

      /* Control video stream */

      video_lock(&vmng->video_inf.lock_state);
      next_video_state = estimate_next_video_state(vmng,
                                                   CAUSE_STILL_STOP);
      change_video_state(vmng, next_video_state);
      video_unlock(&vmng->video_inf.lock_state);
    }

  video_unlock(&vmng->still_inf.lock_state);

  return ret;
}

static int video_queryctrl(FAR struct v4l2_queryctrl *ctrl)
{
  int                        ret;
  struct v4l2_query_ext_ctrl ext_ctrl;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  /* Replace to VIDIOC_QUERY_EXT_CTRL format */

  ext_ctrl.ctrl_class = ctrl->ctrl_class;
  ext_ctrl.id         = ctrl->id;

  ret = video_query_ext_ctrl(&ext_ctrl);

  if (ret != OK)
    {
      return ret;
    }

  if ((ext_ctrl.type == V4L2_CTRL_TYPE_INTEGER64) ||
      (ext_ctrl.type == V4L2_CTRL_TYPE_U8) ||
      (ext_ctrl.type == V4L2_CTRL_TYPE_U16) ||
      (ext_ctrl.type == V4L2_CTRL_TYPE_U32))
    {
      /* Unsupported type in VIDIOC_QUERYCTRL */

      return -EINVAL;
    }

  /* Replace gotten value to VIDIOC_QUERYCTRL */

  ctrl->type          = ext_ctrl.type;
  ctrl->minimum       = ext_ctrl.minimum;
  ctrl->maximum       = ext_ctrl.maximum;
  ctrl->step          = ext_ctrl.step;
  ctrl->default_value = ext_ctrl.default_value;
  ctrl->flags         = ext_ctrl.flags;
  strncpy(ctrl->name, ext_ctrl.name, sizeof(ctrl->name));

  return OK;
}

static void set_parameter_name(uint32_t id, char *name)
{
  int cnt;
  int size
    = sizeof(g_video_parameter_name) / sizeof(video_parameter_name_t);

  for (cnt = 0; cnt < size; cnt++)
    {
      if (g_video_parameter_name[cnt].id == id)
        {
          break;
        }
    }

  ASSERT(cnt < size);

  /* copy size = 32 is due to V4L2 specification. */

  strncpy(name, g_video_parameter_name[cnt].name, 32);
}

static int video_query_ext_ctrl(FAR struct v4l2_query_ext_ctrl *attr)
{
  int ret;
  imgsensor_supported_value_t value;
  imgsensor_capability_range_t *range = &value.u.range;
  imgsensor_capability_discrete_t *disc = &value.u.discrete;
  imgsensor_capability_elems_t *elem = &value.u.elems;

  ASSERT(g_video_sensor_ops);

  if (g_video_sensor_ops->get_supported_value == NULL)
    {
      return -ENOTTY;
    }

  if (attr == NULL)
    {
      return -EINVAL;
    }

  if ((attr->ctrl_class == V4L2_CTRL_CLASS_CAMERA) &&
      (attr->id == V4L2_CID_SCENE_MODE))
    {
      /* Scene mode is processed in only video driver. */

      attr->type          = V4L2_CTRL_TYPE_INTEGER_MENU;
      attr->minimum       = 0;
      attr->maximum       = VIDEO_SCENE_MAX - 1;
      attr->step          = 1;
      attr->default_value = 0;
      strncpy(attr->name, "Scene Mode", 32);
    }
  else
    {
      ret = g_video_sensor_ops->get_supported_value
              (VIDEO_ID(attr->ctrl_class, attr->id),
               &value);
      if (ret < 0)
        {
          return ret;
        }

      attr->type = value.type;

      switch (value.type)
        {
          case IMGSENSOR_CTRL_TYPE_INTEGER_MENU:
            attr->minimum       = 0;
            attr->maximum       = disc->nr_values - 1;
            attr->step          = 1;
            attr->default_value = disc->default_value;
            break;

          case IMGSENSOR_CTRL_TYPE_U8:
          case IMGSENSOR_CTRL_TYPE_U16:
          case IMGSENSOR_CTRL_TYPE_U32:
            attr->minimum = elem->minimum;
            attr->maximum = elem->maximum;
            attr->step    = elem->step;
            attr->elems   = elem->nr_elems;
            break;

          default:
            attr->minimum       = range->minimum;
            attr->maximum       = range->maximum;
            attr->step          = range->step;
            attr->default_value = range->default_value;
            break;
        }

      set_parameter_name(VIDEO_ID(attr->ctrl_class, attr->id),
                         attr->name);
    }

  return OK;
}

static int video_querymenu(FAR struct v4l2_querymenu *menu)
{
  int ret;
  imgsensor_supported_value_t value;

  ASSERT(g_video_sensor_ops);

  if (g_video_sensor_ops->get_supported_value == NULL)
    {
      return -ENOTTY;
    }

  if (menu == NULL)
    {
      return -EINVAL;
    }

  if ((menu->ctrl_class == V4L2_CTRL_CLASS_CAMERA) &&
      (menu->id == V4L2_CID_SCENE_MODE))
    {
      /* Scene mode is processed in only video driver. */

      if (menu->index > VIDEO_SCENE_MAX - 1)
        {
          return -EINVAL;
        }

      menu->value = g_video_scene_parameter[menu->index].mode;
    }
  else
    {
      ret = g_video_sensor_ops->get_supported_value
              (VIDEO_ID(menu->ctrl_class, menu->id),
               &value);
      if (ret < 0)
        {
          return ret;
        }

      if (value.type != IMGSENSOR_CTRL_TYPE_INTEGER_MENU)
        {
          /* VIDIOC_QUERYMENU is used only for
           * IMGSENSOR_CTRL_TYPE_INTEGER_MENU.
           */

          return -EINVAL;
        }

      if (menu->index >= value.u.discrete.nr_values)
        {
          return -EINVAL;
        }

      menu->value = value.u.discrete.values[menu->index];
    }

  return OK;
}

static int video_g_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl)
{
  int                      ret;
  struct v4l2_ext_controls ext_controls;
  struct v4l2_ext_control  control;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  /* Replace to VIDIOC_G_EXT_CTRLS format */

  control.id    = ctrl->id;

  ext_controls.ctrl_class = V4L2_CTRL_CLASS_USER;
  ext_controls.count      = 1;
  ext_controls.controls   = &control;

  /* Execute VIDIOC_G_EXT_CTRLS */

  ret = video_g_ext_ctrls(priv, &ext_controls);

  if (ret == OK)
    {
      /* Replace gotten value to VIDIOC_G_CTRL parameter */

      ctrl->value = control.value;
    }

  return ret;
}

static int video_s_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl)
{
  int ret;
  struct v4l2_ext_controls ext_controls;
  struct v4l2_ext_control  control;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  /* Replace to VIDIOC_S_EXT_CTRLS format */

  control.id    = ctrl->id;
  control.value = ctrl->value;

  ext_controls.ctrl_class = V4L2_CTRL_CLASS_USER;
  ext_controls.count      = 1;
  ext_controls.controls   = &control;

  /* Execute VIDIOC_S_EXT_CTRLS */

  ret = video_s_ext_ctrls(priv, &ext_controls);

  return ret;
}

static int video_g_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls)
{
  int ret = OK;
  int cnt;
  FAR struct v4l2_ext_control *control;

  ASSERT(g_video_sensor_ops);

  if (g_video_sensor_ops->get_value == NULL)
    {
      return -ENOTTY;
    }

  if ((priv == NULL) || (ctrls == NULL))
    {
      return -EINVAL;
    }

  for (cnt = 0, control = ctrls->controls;
       cnt < ctrls->count;
       cnt++, control++)
    {
      ret = g_video_sensor_ops->get_value
              (VIDEO_ID(ctrls->ctrl_class, control->id),
               control->size,
               (imgsensor_value_t *)&control->value64);
      if (ret < 0)
        {
          /* Set cnt in that error occurred */

          ctrls->error_idx = cnt;
          return ret;
        }
    }

  return ret;
}

static int set_intvalue(uint32_t id, int32_t value32)
{
  imgsensor_value_t value;

  ASSERT(g_video_sensor_ops);

  if (g_video_sensor_ops->set_value == NULL)
    {
      return -ENOTTY;
    }

  value.value32 = value32;
  return g_video_sensor_ops->set_value(id, sizeof(int32_t), value);
}

static int set_pvalue(uint32_t id, int size, void *pval)
{
  imgsensor_value_t value;

  ASSERT(g_video_sensor_ops);

  if (g_video_sensor_ops->set_value == NULL)
    {
      return -ENOTTY;
    }

  value.p_u8 = (uint8_t *)pval;
  return g_video_sensor_ops->set_value(id, size, value);
}

static video_scene_params_t *search_scene_param(enum v4l2_scene_mode mode)
{
  int i;
  video_scene_params_t *sp = &g_video_scene_parameter[0];

  for (i = 0; i < VIDEO_SCENE_MAX; i++, sp++)
    {
      if (sp->mode == mode)
        {
          return sp;
        }
    }

  return NULL;
}

static int reflect_scene_parameter(enum v4l2_scene_mode mode)
{
  video_scene_params_t *sp;

  sp = search_scene_param(mode);
  if (sp == NULL)
    {
      /* Unsupported scene mode */

      return -EINVAL;
    }

  set_intvalue(IMGSENSOR_ID_BRIGHTNESS, sp->brightness);
  set_intvalue(IMGSENSOR_ID_CONTRAST, sp->contrast);
  set_intvalue(IMGSENSOR_ID_SATURATION, sp->saturation);
  set_intvalue(IMGSENSOR_ID_HUE , sp->hue);
  set_intvalue(IMGSENSOR_ID_AUTO_WHITE_BALANCE, sp->awb);
  set_intvalue(IMGSENSOR_ID_RED_BALANCE , sp->red);
  set_intvalue(IMGSENSOR_ID_BLUE_BALANCE, sp->blue);
  set_intvalue(IMGSENSOR_ID_GAMMA, sp->gamma);
  set_pvalue(IMGSENSOR_ID_GAMMA_CURVE, sp->gamma_curve_sz, sp->gamma_curve);
  set_intvalue(IMGSENSOR_ID_EXPOSURE, sp->ev);
  set_intvalue(IMGSENSOR_ID_HFLIP_VIDEO, sp->hflip_video);
  set_intvalue(IMGSENSOR_ID_VFLIP_VIDEO, sp->vflip_video);
  set_intvalue(IMGSENSOR_ID_HFLIP_STILL, sp->hflip_still);
  set_intvalue(IMGSENSOR_ID_VFLIP_STILL, sp->vflip_still);
  set_intvalue(IMGSENSOR_ID_SHARPNESS, sp->sharpness);
  set_intvalue(IMGSENSOR_ID_COLORFX, sp->colorfx);
  set_intvalue(IMGSENSOR_ID_AUTOBRIGHTNESS, sp->auto_brightness);
  set_intvalue(IMGSENSOR_ID_ROTATE, sp->rotate);
  set_intvalue(IMGSENSOR_ID_EXPOSURE_AUTO, sp->ae);
  if ((sp->ae == V4L2_EXPOSURE_MANUAL) ||
      (sp->ae == V4L2_EXPOSURE_SHUTTER_PRIORITY))
    {
      set_intvalue(IMGSENSOR_ID_EXPOSURE_ABSOLUTE, sp->exposure_time);
    }

  set_intvalue(IMGSENSOR_ID_FOCUS_ABSOLUTE, sp->focus);
  set_intvalue(IMGSENSOR_ID_FOCUS_AUTO, sp->af);
  set_intvalue(IMGSENSOR_ID_ZOOM_ABSOLUTE, sp->zoom);
  if ((sp->ae == V4L2_EXPOSURE_MANUAL) ||
      (sp->ae == V4L2_EXPOSURE_APERTURE_PRIORITY))
    {
      set_intvalue(IMGSENSOR_ID_IRIS_ABSOLUTE, sp->iris);
    }

  set_intvalue(IMGSENSOR_ID_AUTO_N_PRESET_WB, sp->wb);
  set_intvalue(IMGSENSOR_ID_WIDE_DYNAMIC_RANGE, sp->wdr);
  set_intvalue(IMGSENSOR_ID_IMG_STABILIZATION, sp->stabilization);
  set_intvalue(IMGSENSOR_ID_ISO_SENSITIVITY_AUTO, sp->iso_auto);
  if (sp->iso_auto == V4L2_ISO_SENSITIVITY_MANUAL)
    {
      set_intvalue(IMGSENSOR_ID_ISO_SENSITIVITY, sp->iso);
    }

  set_intvalue(IMGSENSOR_ID_EXPOSURE_METERING, sp->meter);
  set_intvalue(IMGSENSOR_ID_3A_LOCK, sp->threea_lock);
  set_intvalue(IMGSENSOR_ID_FLASH_LED_MODE, sp->led);
  set_intvalue(IMGSENSOR_ID_JPEG_QUALITY, sp->jpeg_quality);

  g_video_scene_mode = mode;

  return OK;
}

static int video_s_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls)
{
  int ret = OK;
  int cnt;
  FAR struct v4l2_ext_control *control;

  ASSERT(g_video_sensor_ops);

  if (g_video_sensor_ops->set_value == NULL)
    {
      return -ENOTTY;
    }

  if ((priv == NULL) || (ctrls == NULL))
    {
      return -EINVAL;
    }

  for (cnt = 0, control = ctrls->controls;
       cnt < ctrls->count;
       cnt++, control++)
    {
      if ((ctrls->ctrl_class == V4L2_CTRL_CLASS_CAMERA) &&
          (control->id == V4L2_CID_SCENE_MODE))
        {
          ret = reflect_scene_parameter(control->value);
        }
      else
        {
          ret = g_video_sensor_ops->set_value
                  (VIDEO_ID(ctrls->ctrl_class, control->id),
                   control->size,
                   (imgsensor_value_t)control->value64);
          if (ret == 0)
            {
              if (g_video_scene_mode == V4L2_SCENE_MODE_NONE)
                {
                  save_scene_param
                  (V4L2_SCENE_MODE_NONE,
                   VIDEO_ID(ctrls->ctrl_class, control->id),
                   control);
                }
            }
        }

      if (ret < 0)
        {
          /* Set cnt in that error occurred */

          ctrls->error_idx = cnt;
          return ret;
        }
    }

  return ret;
}

static int video_query_ext_ctrl_scene
             (FAR struct v4s_query_ext_ctrl_scene *attr)
{
  if (attr == NULL)
    {
      return -EINVAL;
    }

  return video_query_ext_ctrl(&attr->control);
}

static int video_querymenu_scene(FAR struct v4s_querymenu_scene *menu)
{
  if (menu == NULL)
    {
      return -EINVAL;
    }

  return video_querymenu(&menu->menu);
}

static int read_scene_param(enum v4l2_scene_mode mode,
                            uint32_t id,
                            struct v4l2_ext_control *control)
{
  int ret = OK;
  video_scene_params_t *sp;
  imgsensor_supported_value_t value;

  ASSERT(g_video_sensor_ops);

  if (g_video_sensor_ops->get_supported_value == NULL)
    {
      return -ENOTTY;
    }

  if (control == NULL)
    {
      return -EINVAL;
    }

  sp = search_scene_param(mode);
  if (sp == NULL)
    {
      /* Unsupported scene mode */

      return -EINVAL;
    }

  ret = g_video_sensor_ops->get_supported_value(id, &value);
  if (ret < 0)
    {
      /* Unsupported camera parameter */

      return ret;
    }

  switch (id)
    {
      case IMGSENSOR_ID_BRIGHTNESS:
        control->value = sp->brightness;
        break;

      case IMGSENSOR_ID_CONTRAST:
        control->value = sp->contrast;
        break;

      case IMGSENSOR_ID_SATURATION:
        control->value = sp->saturation;
        break;

      case IMGSENSOR_ID_HUE:
        control->value = sp->hue;
        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        control->value = sp->awb;
        break;

      case IMGSENSOR_ID_RED_BALANCE:
        control->value = sp->red;
        break;

      case IMGSENSOR_ID_BLUE_BALANCE:
        control->value = sp->blue;
        break;

      case IMGSENSOR_ID_GAMMA:
        control->value = sp->gamma;
        break;

      case IMGSENSOR_ID_GAMMA_CURVE:
        memcpy(control->p_u8,
               sp->gamma_curve,
               sp->gamma_curve_sz);
        break;

      case IMGSENSOR_ID_EXPOSURE:
        control->value = sp->ev;
        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        control->value = sp->hflip_video;
        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        control->value = sp->vflip_video;
        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        control->value = sp->hflip_still;
        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        control->value = sp->vflip_still;
        break;

      case IMGSENSOR_ID_SHARPNESS:
        control->value = sp->sharpness;
        break;

      case IMGSENSOR_ID_COLOR_KILLER:
        control->value = (sp->colorfx == V4L2_COLORFX_BW) ? true : false;
        break;

      case IMGSENSOR_ID_COLORFX:
        control->value = sp->colorfx;
        break;

      case IMGSENSOR_ID_AUTOBRIGHTNESS:
        control->value = sp->auto_brightness;
        break;

      case IMGSENSOR_ID_ROTATE:
        control->value = sp->rotate;
        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        control->value = sp->ae;
        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        control->value = sp->exposure_time;
        break;

      case IMGSENSOR_ID_FOCUS_ABSOLUTE:
        control->value = sp->focus;
        break;

      case IMGSENSOR_ID_FOCUS_AUTO:
        control->value = sp->af;
        break;

      case IMGSENSOR_ID_ZOOM_ABSOLUTE:
        control->value = sp->zoom;
        break;

      case IMGSENSOR_ID_IRIS_ABSOLUTE:
        control->value = sp->iris;
        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        control->value = sp->wb;
        break;

      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
        control->value = sp->wdr;
        break;

      case IMGSENSOR_ID_IMG_STABILIZATION:
        control->value = sp->stabilization;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        control->value = sp->iso;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        control->value = sp->iso_auto;
        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        control->value = sp->meter;
        break;

      case IMGSENSOR_ID_3A_LOCK:
        control->value = sp->threea_lock;
        break;

      case IMGSENSOR_ID_FLASH_LED_MODE:
        control->value = sp->led;
        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        control->value = sp->jpeg_quality;
        break;

      default:
        ret = -EINVAL;
    }

  return ret;
}

static int video_g_ext_ctrls_scene(FAR struct v4s_ext_controls_scene *ctrls)
{
  int ret = OK;
  int cnt;
  FAR struct v4l2_ext_control *control;

  if (ctrls == NULL)
    {
      return -EINVAL;
    }

  for (cnt = 0, control = ctrls->control.controls;
       cnt < ctrls->control.count;
       cnt++, control++)
    {
      ret = read_scene_param
              (ctrls->mode,
               VIDEO_ID(ctrls->control.ctrl_class, control->id),
               control);
      if (ret != OK)
        {
          ctrls->control.error_idx = cnt;
          return ret;
        }
    }

  return ret;
}

static int check_range(int64_t value,
                       int64_t min,
                       int64_t max,
                       uint64_t step)
{
  if ((value < min) ||
      (value > max) ||
      ((value - min) % step != 0))
    {
      return -EINVAL;
    }

  return OK;
}

static int save_scene_param(enum v4l2_scene_mode mode,
                            uint32_t id,
                            struct v4l2_ext_control *control)
{
  int ret;
  int i;
  video_scene_params_t *sp;
  imgsensor_supported_value_t value;
  imgsensor_capability_range_t *range = &value.u.range;
  imgsensor_capability_discrete_t *disc = &value.u.discrete;
  imgsensor_capability_elems_t *elem = &value.u.elems;

  ASSERT(g_video_sensor_ops);

  if (g_video_sensor_ops->get_supported_value == NULL)
    {
      return -ENOTTY;
    }

  sp = search_scene_param(mode);
  if (sp == NULL)
    {
      /* Unsupported scene mode */

      return -EINVAL;
    }

  ret = g_video_sensor_ops->get_supported_value(id, &value);
  if (ret < 0)
    {
      /* Unsupported camera parameter */

      return ret;
    }

  switch (value.type)
    {
      case IMGSENSOR_CTRL_TYPE_INTEGER_MENU:

        for (i = 0; i < disc->nr_values; i++)
          {
            if (control->value == disc->values[i])
              {
                break;
              }
          }

        if (i >= disc->nr_values)
          {
            return -EINVAL;
          }

        break;

      case IMGSENSOR_CTRL_TYPE_U8:

        for (i = 0; i < elem->nr_elems; i++)
          {
            ret = check_range(control->p_u8[i],
                              elem->minimum,
                              elem->maximum,
                              elem->step);
            if (ret != OK)
              {
                return ret;
              }
          }

        break;

      case IMGSENSOR_CTRL_TYPE_U16:

        for (i = 0; i < elem->nr_elems; i++)
          {
            ret = check_range(control->p_u16[i],
                              elem->minimum,
                              elem->maximum,
                              elem->step);
            if (ret != OK)
              {
                return ret;
              }
          }

        break;

      case IMGSENSOR_CTRL_TYPE_U32:

        for (i = 0; i < elem->nr_elems; i++)
          {
            ret = check_range(control->p_u32[i],
                              elem->minimum,
                              elem->maximum,
                              elem->step);
            if (ret != OK)
              {
                return ret;
              }
          }

        break;

      default:
        ret = check_range(control->value,
                          range->minimum,
                          range->maximum,
                          range->step);
        if (ret != OK)
          {
            return ret;
          }

        break;
    }

  switch (id)
    {
      case IMGSENSOR_ID_BRIGHTNESS:
        sp->brightness = control->value;
        break;

      case IMGSENSOR_ID_CONTRAST:
        sp->contrast = control->value;
        break;

      case IMGSENSOR_ID_SATURATION:
        sp->saturation = control->value;
        break;

      case IMGSENSOR_ID_HUE:
        sp->hue = control->value;
        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        sp->awb = control->value;
        break;

      case IMGSENSOR_ID_RED_BALANCE:
        sp->red = control->value;
        break;

      case IMGSENSOR_ID_BLUE_BALANCE:
        sp->blue = control->value;
        break;

      case IMGSENSOR_ID_GAMMA:
        sp->gamma = control->value;
        break;

      case IMGSENSOR_ID_GAMMA_CURVE:
        memcpy(sp->gamma_curve,
               control->p_u8,
               sp->gamma_curve_sz);
        break;

      case IMGSENSOR_ID_EXPOSURE:
        sp->ev = control->value;
        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        sp->hflip_video = control->value;
        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        sp->vflip_video = control->value;
        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        sp->hflip_still = control->value;
        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        sp->vflip_still = control->value;
        break;

      case IMGSENSOR_ID_SHARPNESS:
        sp->sharpness = control->value;
        break;

      case IMGSENSOR_ID_COLOR_KILLER:
        sp->colorfx = (control->value == true) ?
                      V4L2_COLORFX_BW : V4L2_COLORFX_NONE;
        break;

      case IMGSENSOR_ID_COLORFX:
        sp->colorfx = control->value;
        break;

      case IMGSENSOR_ID_AUTOBRIGHTNESS:
        sp->auto_brightness = control->value;
        break;

      case IMGSENSOR_ID_ROTATE:
        sp->rotate = control->value;
        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        sp->ae = control->value;
        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        sp->exposure_time = control->value;
        break;

      case IMGSENSOR_ID_FOCUS_ABSOLUTE:
        sp->focus = control->value;
        break;

      case IMGSENSOR_ID_FOCUS_AUTO:
        sp->af = control->value;
        break;

      case IMGSENSOR_ID_ZOOM_ABSOLUTE:
        sp->zoom = control->value;
        break;

      case IMGSENSOR_ID_IRIS_ABSOLUTE:
        sp->iris = control->value;
        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        sp->wb = control->value;
        break;

      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
        sp->wdr = control->value;
        break;

      case IMGSENSOR_ID_IMG_STABILIZATION:
        sp->stabilization = control->value;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        sp->iso = control->value;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        sp->iso_auto = control->value;
        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        sp->meter = control->value;
        break;

      case IMGSENSOR_ID_3A_LOCK:
        sp->threea_lock = control->value;
        break;

      case IMGSENSOR_ID_FLASH_LED_MODE:
        sp->led = control->value;
        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        sp->jpeg_quality = control->value;
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

static int video_s_ext_ctrls_scene(FAR struct v4s_ext_controls_scene *ctrls)
{
  int ret = OK;
  int cnt;
  FAR struct v4l2_ext_control *control;

  if (ctrls == NULL)
    {
      return -EINVAL;
    }

  for (cnt = 0, control = ctrls->control.controls;
       cnt < ctrls->control.count;
       cnt++, control++)
    {
      ret = save_scene_param
              (ctrls->mode,
               VIDEO_ID(ctrls->control.ctrl_class, control->id),
               control);
      if (ret != OK)
        {
          ctrls->control.error_idx = cnt;
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: video_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int video_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR video_mng_t  *priv  = (FAR video_mng_t *)inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case VIDIOC_REQBUFS:
        ret = video_reqbufs(priv, (FAR struct v4l2_requestbuffers *)arg);

        break;

      case VIDIOC_QBUF:
        ret = video_qbuf(priv, (FAR struct v4l2_buffer *)arg);

        break;

      case VIDIOC_DQBUF:
        ret = video_dqbuf(priv, (FAR struct v4l2_buffer *)arg);

        break;

      case VIDIOC_CANCEL_DQBUF:
        ret = video_cancel_dqbuf(priv, (FAR enum v4l2_buf_type)arg);

        break;

      case VIDIOC_STREAMON:
        ret = video_streamon(priv, (FAR enum v4l2_buf_type *)arg);

        break;

      case VIDIOC_STREAMOFF:
        ret = video_streamoff(priv, (FAR enum v4l2_buf_type *)arg);

        break;

      case VIDIOC_DO_HALFPUSH:
        ret = video_do_halfpush(priv, arg);

        break;

      case VIDIOC_TAKEPICT_START:
        ret = video_takepict_start(priv, (int32_t)arg);

        break;

      case VIDIOC_TAKEPICT_STOP:
        ret = video_takepict_stop(priv, arg);

        break;

      case VIDIOC_TRY_FMT:
        ret = video_try_fmt(priv, (FAR struct v4l2_format *)arg);

        break;

      case VIDIOC_S_FMT:
        ret = video_s_fmt(priv, (FAR struct v4l2_format *)arg);

        break;

      case VIDIOC_S_PARM:
        ret = video_s_parm(priv, (FAR struct v4l2_streamparm *)arg);

        break;

      case VIDIOC_QUERYCTRL:
        ret = video_queryctrl((FAR struct v4l2_queryctrl *)arg);

        break;

      case VIDIOC_QUERY_EXT_CTRL:
        ret = video_query_ext_ctrl((FAR struct v4l2_query_ext_ctrl *)arg);

        break;

      case VIDIOC_QUERYMENU:
        ret = video_querymenu((FAR struct v4l2_querymenu *)arg);

        break;

      case VIDIOC_G_CTRL:
        ret = video_g_ctrl(priv, (FAR struct v4l2_control *)arg);

        break;

      case VIDIOC_S_CTRL:
        ret = video_s_ctrl(priv, (FAR struct v4l2_control *)arg);

        break;

      case VIDIOC_G_EXT_CTRLS:
        ret = video_g_ext_ctrls(priv, (FAR struct v4l2_ext_controls *)arg);

        break;

      case VIDIOC_S_EXT_CTRLS:
        ret = video_s_ext_ctrls(priv, (FAR struct v4l2_ext_controls *)arg);

        break;

      case V4SIOC_QUERY_EXT_CTRL_SCENE:
        ret = video_query_ext_ctrl_scene
                ((FAR struct v4s_query_ext_ctrl_scene *)arg);

        break;

      case V4SIOC_QUERYMENU_SCENE:
        ret = video_querymenu_scene((FAR struct v4s_querymenu_scene *)arg);

        break;

      case V4SIOC_G_EXT_CTRLS_SCENE:
        ret = video_g_ext_ctrls_scene
                ((FAR struct v4s_ext_controls_scene *)arg);

        break;

      case V4SIOC_S_EXT_CTRLS_SCENE:
        ret = video_s_ext_ctrls_scene
                ((FAR struct v4s_ext_controls_scene *)arg);

        break;

      default:
        verr("Unrecognized cmd: %d\n", cmd);
        ret = - ENOTTY;
        break;
    }

  return ret;
}

static FAR void *video_register(FAR const char *devpath)
{
  FAR    video_mng_t *priv;
  int    ret;
  size_t allocsize;

  /* Input devpath Error Check */

  if (!devpath)
    {
      return NULL;
    }

  allocsize = strnlen(devpath, MAX_VIDEO_FILE_PATH - 1/* Space for '\0' */);
  if ((allocsize < 2)     ||
      (devpath[0] != '/') ||
      ((allocsize == (MAX_VIDEO_FILE_PATH - 1)) &&
       (devpath[MAX_VIDEO_FILE_PATH] != '\0')))
    {
      return NULL;
    }

  /* Initialize video device structure */

  priv = (FAR video_mng_t *)kmm_malloc(sizeof(video_mng_t));
  if (!priv)
    {
      verr("Failed to allocate instance\n");
      return NULL;
    }

  memset(priv, 0, sizeof(video_mng_t));

  /* Save device path */

  priv->devpath = (FAR char *)kmm_malloc(allocsize + 1);
  if (!priv->devpath)
    {
      kmm_free(priv);
      return NULL;
    }

  memcpy(priv->devpath, devpath, allocsize);
  priv->devpath[allocsize] = '\0';

  /* Initialize semaphore */

  nxsem_init(&priv->lock_open_num, 0, 1);

  /* Register the character driver */

  ret = register_driver(priv->devpath, &g_video_fops, 0666, priv);
  if (ret < 0)
    {
      verr("Failed to register driver: %d\n", ret);
      kmm_free(priv->devpath);
      kmm_free(priv);
      return NULL;
    }

  return (FAR void *)priv;
}

static int video_unregister(FAR video_mng_t *v_mgr)
{
  int ret = OK;

  if (!v_mgr)
    {
      ret = -ENODEV;
    }
  else
    {
      nxsem_destroy(&v_mgr->lock_open_num);

      unregister_driver((const char *)v_mgr->devpath);

      kmm_free(v_mgr->devpath);
      kmm_free(v_mgr);
    }

  return ret;
}

/* Callback function which device driver call when capture has done. */

static int video_complete_capture(uint8_t  err_code, uint32_t datasize)
{
  FAR video_mng_t      *vmng = (FAR video_mng_t *)video_handler;
  FAR video_type_inf_t *type_inf;
  FAR vbuf_container_t *container = NULL;
  enum v4l2_buf_type buf_type;
  irqstate_t           flags;

  flags = enter_critical_section();

  buf_type = (vmng->still_inf.state == VIDEO_STATE_CAPTURE) ?
                V4L2_BUF_TYPE_STILL_CAPTURE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

  type_inf = get_video_type_inf(vmng, buf_type);
  if (type_inf == NULL)
    {
      leave_critical_section(flags);
      return -EINVAL;
    }

  if (err_code == 0)
    {
      type_inf->bufinf.vbuf_curr->buf.flags = 0;
      if (type_inf->remaining_capnum > 0)
        {
          type_inf->remaining_capnum--;
        }
    }
  else
    {
      type_inf->bufinf.vbuf_curr->buf.flags = V4L2_BUF_FLAG_ERROR;
    }

  type_inf->bufinf.vbuf_curr->buf.bytesused = datasize;
  video_framebuff_capture_done(&type_inf->bufinf);

  if (is_sem_waited(&type_inf->wait_capture.dqbuf_wait_flg))
    {
      /* If waiting capture in DQBUF,
       * get/save container and unlock wait
       */

      type_inf->wait_capture.done_container
        = video_framebuff_pop_curr_container(&type_inf->bufinf);
      type_inf->wait_capture.waitend_cause
        = VIDEO_WAITEND_CAUSE_CAPTUREDONE;
      nxsem_post(&type_inf->wait_capture.dqbuf_wait_flg);
    }

  if (type_inf->remaining_capnum == 0)
    {
      g_video_data_ops->stop_capture();
      type_inf->state = VIDEO_STATE_STREAMOFF;

      /* If stop still stream, notify it to video stream */

      if ((buf_type == V4L2_BUF_TYPE_STILL_CAPTURE) &&
           is_sem_waited(&vmng->video_inf.wait_capture.dqbuf_wait_flg))
        {
          vmng->video_inf.wait_capture.waitend_cause
            = VIDEO_WAITEND_CAUSE_STILLSTOP;
          nxsem_post(&vmng->video_inf.wait_capture.dqbuf_wait_flg);
        }
    }
  else
    {
      container = video_framebuff_get_vacant_container(&type_inf->bufinf);
      if (!container)
        {
          g_video_data_ops->stop_capture();
          type_inf->state = VIDEO_STATE_STREAMON;
        }
      else
        {
          g_video_data_ops->set_buf((uint8_t *)container->buf.m.userptr,
                                  container->buf.length);
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int video_initialize(FAR const char *devpath)
{
  if (is_initialized)
    {
      return OK;
    }

  video_handler = video_register(devpath);

  is_initialized = true;

  return OK;
}

int video_uninitialize(void)
{
  if (is_initialized)
    {
      return OK;
    }

  video_unregister(video_handler);

  is_initialized = false;

  return OK;
}

void imgsensor_register(const FAR struct imgsensor_ops_s *ops)
{
  g_video_sensor_ops = ops;
}

void imgdata_register(const FAR struct imgdata_ops_s *ops)
{
  g_video_data_ops = ops;
}

