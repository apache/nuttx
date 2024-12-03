/****************************************************************************
 * drivers/video/v4l2_cap.c
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

#include <debug.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <poll.h>

#include <nuttx/mutex.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/video.h>

#include "video_framebuff.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_CAPTURE_FMT           (2)
#define CAPTURE_FMT_MAIN          (0)
#define CAPTURE_FMT_SUB           (1)

#define REMAINING_CAPNUM_INFINITY (-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum capture_state_e
{
  CAPTURE_STATE_STREAMOFF = 0, /* Capture trigger event is not received */
  CAPTURE_STATE_STREAMON  = 1, /* Capture trigger event is received,
                                * but capture is not operated.
                                */
  CAPTURE_STATE_CAPTURE   = 2, /* On capture */
};

enum capture_state_cause_e
{
  CAUSE_CAPTURE_STOP  = 0,     /* Stop  capture event for capture stream */
  CAUSE_CAPTURE_START = 1,     /* Start capture event for capture stream */
  CAUSE_CAPTURE_DQBUF = 2,     /* DQBUF timing        for video stream */
  CAUSE_STILL_STOP    = 3,     /* Stop  capture event for still stream */
  CAUSE_STILL_START   = 4,     /* Start capture event for still stream */
};

enum capture_waitend_cause_e
{
  WAITEND_CAUSE_CAPTUREDONE = 0,
  WAITEND_CAUSE_DQCANCEL    = 1,
  WAITEND_CAUSE_STILLSTOP   = 2,
};

struct video_format_s
{
  uint16_t width;
  uint16_t height;
  uint32_t pixelformat;
};

typedef struct video_format_s video_format_t;

struct capture_wait_capture_s
{
  sem_t                dqbuf_wait_flg;

  /* Save container which capture is done */

  FAR vbuf_container_t *done_container;
  enum capture_waitend_cause_e waitend_cause;
};

typedef struct capture_wait_capture_s capture_wait_capture_t;

struct capture_type_inf_s
{
  mutex_t                lock_state;
  enum capture_state_e   state;
  int32_t                remaining_capnum;
  capture_wait_capture_t wait_capture;
  uint8_t                nr_fmt;
  video_format_t         fmt[MAX_CAPTURE_FMT];
  struct v4l2_rect       clip;
  struct v4l2_fract      frame_interval;
  video_framebuff_t      bufinf;
  FAR uint8_t            *bufheap;   /* for V4L2_MEMORY_MMAP buffers */
  FAR struct pollfd      *fds;
  uint32_t               seqnum;
};

typedef struct capture_type_inf_s capture_type_inf_t;

struct capture_scene_params_s
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
  FAR uint8_t *gamma_curve;
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
  int32_t spot_pos;
  int32_t threea_lock;
  enum v4l2_flash_led_mode led;
  int32_t jpeg_quality;
};

typedef struct capture_scene_params_s capture_scene_params_t;

struct capture_parameter_name_s
{
  uint32_t id;
  FAR const char *name;
};

typedef struct capture_parameter_name_s capture_parameter_name_t;

struct capture_mng_s
{
  struct v4l2_s          v4l2;

  /* Parameter of capture_initialize() */

  mutex_t                lock_open_num;
  uint8_t                open_num;
  capture_type_inf_t     capture_inf;
  capture_type_inf_t     still_inf;
  FAR struct imgdata_s   *imgdata;
  FAR struct imgsensor_s *imgsensor;
  enum v4l2_scene_mode   capture_scene_mode;
  uint8_t                capture_scence_num;
  FAR capture_scene_params_t *capture_scene_param[V4L2_SCENE_MODE_MAX];
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool                   unlinked;
#endif
};

typedef struct capture_mng_s capture_mng_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR capture_type_inf_t *
get_capture_type_inf(FAR capture_mng_t *cmng, uint8_t type);
static enum capture_state_e
estimate_next_capture_state(FAR capture_mng_t *cmng,
                            enum capture_state_cause_e cause);
static void change_capture_state(FAR capture_mng_t *cmng,
                                 enum capture_state_e next_state);
static bool is_taking_still_picture(FAR capture_mng_t *cmng);
static bool is_bufsize_sufficient(FAR capture_mng_t *cmng, uint32_t bufsize);
static void cleanup_resources(FAR capture_mng_t *cmng);
static bool is_sem_waited(FAR sem_t *sem);
static int save_scene_param(FAR capture_mng_t *cmng,
                            enum v4l2_scene_mode mode,
                            uint32_t id,
                            FAR struct v4l2_ext_control *control);
static int complete_capture(uint8_t err_code, uint32_t datasize,
                            FAR const struct timeval *ts,
                            FAR void *arg);
static int validate_frame_setting(FAR capture_mng_t *cmng,
                                  enum v4l2_buf_type type,
                                  uint8_t nr_fmt,
                                  FAR video_format_t *vfmt,
                                  FAR struct v4l2_rect *clip,
                                  FAR struct v4l2_fract *interval);
static size_t get_bufsize(FAR video_format_t *vf);

/* ioctl function for each cmds of ioctl */

static int capture_querycap(FAR struct file *filep,
                            FAR struct v4l2_capability *cap);
static int capture_g_input(FAR int *num);
static int capture_enum_input(FAR struct file *filep,
                              FAR struct v4l2_input *input);
static int capture_reqbufs(FAR struct file *filep,
                           FAR struct v4l2_requestbuffers *reqbufs);
static int capture_querybuf(FAR struct file *filep,
                            FAR struct v4l2_buffer *buf);
static int capture_qbuf(FAR struct file *filep,
                        FAR struct v4l2_buffer *buf);
static int capture_dqbuf(FAR struct file *filep,
                         FAR struct v4l2_buffer *buf);
static int capture_cancel_dqbuf(FAR struct file *filep,
                                enum v4l2_buf_type type);
static int capture_g_fmt(FAR struct file *filep,
                         FAR struct v4l2_format *fmt);
static int capture_s_fmt(FAR struct file *filep,
                         FAR struct v4l2_format *fmt);
static int capture_try_fmt(FAR struct file *filep,
                           FAR struct v4l2_format *fmt);
static int capture_g_parm(FAR struct file *filep,
                          FAR struct v4l2_streamparm *parm);
static int capture_s_parm(FAR struct file *filep,
                          FAR struct v4l2_streamparm *parm);
static int capture_streamon(FAR struct file *filep,
                            FAR enum v4l2_buf_type *type);
static int capture_streamoff(FAR struct file *filep,
                             FAR enum v4l2_buf_type *type);
static int capture_do_halfpush(FAR struct file *filep,
                               bool enable);
static int capture_takepict_start(FAR struct file *filep,
                                  int32_t capture_num);
static int capture_takepict_stop(FAR struct file *filep,
                                 bool halfpush);
static int capture_s_selection(FAR struct file *filep,
                               FAR struct v4l2_selection *clip);
static int capture_g_selection(FAR struct file *filep,
                               FAR struct v4l2_selection *clip);
static int capture_queryctrl(FAR struct file *filep,
                             FAR struct v4l2_queryctrl *ctrl);
static int capture_query_ext_ctrl(FAR struct file *filep,
                                  FAR struct v4l2_query_ext_ctrl *ctrl);
static int capture_querymenu(FAR struct file *filep,
                             FAR struct v4l2_querymenu *menu);
static int capture_g_ctrl(FAR struct file *filep,
                          FAR struct v4l2_control *ctrl);
static int capture_s_ctrl(FAR struct file *filep,
                          FAR struct v4l2_control *ctrl);
static int capture_g_ext_ctrls(FAR struct file *filep,
                               FAR struct v4l2_ext_controls *ctrls);
static int capture_s_ext_ctrls(FAR struct file *filep,
                               FAR struct v4l2_ext_controls *ctrls);
static int capture_query_ext_ctrl_scene(FAR struct file *filep,
             FAR struct v4s_query_ext_ctrl_scene *ctrl);
static int capture_querymenu_scene(FAR struct file *filep,
             FAR struct v4s_querymenu_scene *menu);
static int capture_g_ext_ctrls_scene(FAR struct file *filep,
             FAR struct v4s_ext_controls_scene *ctrls);
static int capture_s_ext_ctrls_scene(FAR struct file *filep,
             FAR struct v4s_ext_controls_scene *ctrls);
static int capture_enum_fmt(FAR struct file *filep,
                            FAR struct v4l2_fmtdesc *f);
static int capture_enum_frminterval(FAR struct file *filep,
                                    FAR struct v4l2_frmivalenum *f);
static int capture_enum_frmsize(FAR struct file *filep,
                                FAR struct v4l2_frmsizeenum *f);

/* File operations function */

static int capture_open(FAR struct file *filep);
static int capture_close(FAR struct file *filep);
static int capture_mmap(FAR struct file *filep,
                        FAR struct mm_map_entry_s *map);
static int capture_poll(FAR struct file *filep,
                        FAR struct pollfd *fds, bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int capture_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct v4l2_ops_s g_capture_vops =
{
  capture_querycap,                   /* querycap */
  capture_g_input,                    /* g_input */
  capture_enum_input,                 /* enum_input */
  capture_reqbufs,                    /* reqbufs */
  capture_querybuf,                   /* querybuf */
  capture_qbuf,                       /* qbuf */
  capture_dqbuf,                      /* dqbuf */
  capture_cancel_dqbuf,               /* cancel_dqbuf */
  capture_g_fmt,                      /* g_fmt */
  capture_s_fmt,                      /* s_fmt */
  capture_try_fmt,                    /* try_fmt */
  capture_g_parm,                     /* g_parm */
  capture_s_parm,                     /* s_parm */
  capture_streamon,                   /* streamon */
  capture_streamoff,                  /* streamoff */
  capture_do_halfpush,                /* do_halfpush */
  capture_takepict_start,             /* takepict_start */
  capture_takepict_stop,              /* takepict_stop */
  capture_s_selection,                /* s_selection */
  capture_g_selection,                /* g_selection */
  capture_queryctrl,                  /* queryctrl */
  capture_query_ext_ctrl,             /* query_ext_ctrl */
  capture_querymenu,                  /* querymenu */
  capture_g_ctrl,                     /* g_ctrl */
  capture_s_ctrl,                     /* s_ctrl */
  capture_g_ext_ctrls,                /* g_ext_ctrls */
  capture_s_ext_ctrls,                /* s_ext_ctrls */
  capture_query_ext_ctrl_scene,       /* query_ext_ctrl_scene */
  capture_querymenu_scene,            /* querymenu_scene */
  capture_g_ext_ctrls_scene,          /* g_ext_ctrls_scene */
  capture_s_ext_ctrls_scene,          /* s_ext_ctrls_scene */
  capture_enum_fmt,                   /* enum_fmt */
  capture_enum_frminterval,           /* enum_frminterval */
  capture_enum_frmsize                /* enum_frmsize */
};

static const struct file_operations g_capture_fops =
{
  capture_open,               /* open */
  capture_close,              /* close */
  NULL,                       /* read */
  NULL,                       /* write */
  NULL,                       /* seek */
  NULL,                       /* ioctl */
  capture_mmap,               /* mmap */
  NULL,                       /* truncate */
  capture_poll,               /* poll */
  NULL,                       /* readv */
  NULL,                       /* writev */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  capture_unlink,             /* unlink */
#endif
};

static const capture_parameter_name_t g_capture_parameter_name[] =
{
    {
      IMGSENSOR_ID_BRIGHTNESS,           "Brightness"
    },

    {
      IMGSENSOR_ID_CONTRAST,             "Contrast"
    },

    {
      IMGSENSOR_ID_SATURATION,           "Saturation"
    },

    {
      IMGSENSOR_ID_HUE,                  "Hue"
    },

    {
      IMGSENSOR_ID_AUTO_WHITE_BALANCE,   "Automatic white balance"
    },

    {
      IMGSENSOR_ID_RED_BALANCE,          "Red balance"
    },

    {
      IMGSENSOR_ID_BLUE_BALANCE,         "Blue balance"
    },

    {
      IMGSENSOR_ID_GAMMA,                "Gamma value"
    },

    {
      IMGSENSOR_ID_GAMMA_CURVE,          "Gamma adjustment(curve)"
    },

    {
      IMGSENSOR_ID_EXPOSURE,             "Exposure value"
    },

    {
      IMGSENSOR_ID_HFLIP_VIDEO,          "Mirror horizontally(VIDEO)"
    },

    {
      IMGSENSOR_ID_VFLIP_VIDEO,          "Mirror vertically(VIDEO)"
    },

    {
      IMGSENSOR_ID_HFLIP_STILL,          "Mirror horizontally(STILL)"
    },

    {
      IMGSENSOR_ID_VFLIP_STILL,          "Mirror vertically(STILL)"
    },

    {
      IMGSENSOR_ID_SHARPNESS,            "Sharpness"
    },

    {
      IMGSENSOR_ID_COLOR_KILLER,         "Color killer"
    },

    {
      IMGSENSOR_ID_COLORFX,              "Color effect"
    },

    {
      IMGSENSOR_ID_AUTOBRIGHTNESS,       "Auto brightness"
    },

    {
      IMGSENSOR_ID_ROTATE,               "Rotate"
    },

    {
      IMGSENSOR_ID_EXPOSURE_AUTO,        "Auto Exposure"
    },

    {
      IMGSENSOR_ID_EXPOSURE_ABSOLUTE,    "Exposure time(100 usec)"
    },

    {
      IMGSENSOR_ID_FOCUS_ABSOLUTE,       "Focus(absolute value)"
    },

    {
      IMGSENSOR_ID_FOCUS_RELATIVE,       "Focus(relative value)"
    },

    {
      IMGSENSOR_ID_FOCUS_AUTO,           "Continuous Auto Focus"
    },

    {
      IMGSENSOR_ID_ZOOM_ABSOLUTE,        "Zoom(absolute value)"
    },

    {
      IMGSENSOR_ID_ZOOM_RELATIVE,        "Zoom(relative value)"
    },

    {
      IMGSENSOR_ID_ZOOM_CONTINUOUS,      "Continuous zoom"
    },

    {
      IMGSENSOR_ID_IRIS_ABSOLUTE,        "Iris(absolute value)"
    },

    {
      IMGSENSOR_ID_IRIS_RELATIVE,        "Iris(relative value)"
    },

    {
      IMGSENSOR_ID_AUTO_N_PRESET_WB,     "Preset white balance"
    },

    {
      IMGSENSOR_ID_WIDE_DYNAMIC_RANGE,   "Wide dynamic range"
    },

    {
      IMGSENSOR_ID_IMG_STABILIZATION,    "Image stabilization"
    },

    {
      IMGSENSOR_ID_ISO_SENSITIVITY,      "ISO sensitivity"
    },

    {
      IMGSENSOR_ID_ISO_SENSITIVITY_AUTO, "Automatic ISO sensitivity"
    },

    {
      IMGSENSOR_ID_EXPOSURE_METERING,    "Photometry"
    },

    {
      IMGSENSOR_ID_SPOT_POSITION,        "Spot position"
    },

    {
      IMGSENSOR_ID_3A_LOCK,              "Lock AWB/AE"
    },

    {
      IMGSENSOR_ID_AUTO_FOCUS_START,     "Start single Auto Focus"
    },

    {
      IMGSENSOR_ID_AUTO_FOCUS_STOP,      "Stop single Auto Focus"
    },

    {
      IMGSENSOR_ID_3A_PARAMETER,         "3A parameter"
    },

    {
      IMGSENSOR_ID_3A_STATUS,            "3A status"
    },

    {
      IMGSENSOR_ID_FLASH_LED_MODE,       "LED mode"
    },

    {
      IMGSENSOR_ID_JPEG_QUALITY,         "JPEG compression quality"
    }
};

static FAR struct imgsensor_s **g_capture_registered_sensor = NULL;
static size_t g_capture_registered_sensor_num;
static FAR struct imgdata_s *g_capture_data = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR capture_type_inf_t *
get_capture_type_inf(FAR capture_mng_t *cmng, uint8_t type)
{
  FAR capture_type_inf_t *type_inf;

  switch (type)
    {
      case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        type_inf = &cmng->capture_inf;
        break;

      case V4L2_BUF_TYPE_STILL_CAPTURE:
        type_inf = &cmng->still_inf;
        break;

      default:  /* Error case */
        type_inf = NULL;
        break;
    }

  return type_inf;
}

static enum capture_state_e
estimate_next_capture_state(FAR capture_mng_t *cmng,
                            enum capture_state_cause_e cause)
{
  enum capture_state_e current_state = cmng->capture_inf.state;

  switch (cause)
    {
      case CAUSE_CAPTURE_STOP:
        return CAPTURE_STATE_STREAMOFF;

      case CAUSE_CAPTURE_START:
        if (is_taking_still_picture(cmng))
          {
            return CAPTURE_STATE_STREAMON;
          }
        else
          {
            return CAPTURE_STATE_CAPTURE;
          }

      case CAUSE_STILL_STOP:
        if (current_state == CAPTURE_STATE_STREAMON)
          {
            return CAPTURE_STATE_CAPTURE;
          }
        else
          {
            return current_state;
          }

      case CAUSE_STILL_START:
        if (current_state == CAPTURE_STATE_CAPTURE)
          {
            return CAPTURE_STATE_STREAMON;
          }
        else
          {
            return current_state;
          }

      case CAUSE_CAPTURE_DQBUF:
        if (current_state == CAPTURE_STATE_STREAMON &&
            !is_taking_still_picture(cmng))
          {
            return CAPTURE_STATE_CAPTURE;
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

  data->width  = video->width;
  data->height = video->height;
  switch (video->pixelformat)
    {
      case V4L2_PIX_FMT_NV12:
        data->pixelformat = IMGDATA_PIX_FMT_NV12;
        break;

      case V4L2_PIX_FMT_YUV420:
        data->pixelformat = IMGDATA_PIX_FMT_YUV420P;
        break;

      case V4L2_PIX_FMT_YUYV:
        data->pixelformat = IMGDATA_PIX_FMT_YUYV;
        break;

      case V4L2_PIX_FMT_UYVY:
        data->pixelformat = IMGDATA_PIX_FMT_UYVY;
        break;

      case V4L2_PIX_FMT_RGB565:
        data->pixelformat = IMGDATA_PIX_FMT_RGB565;
        break;

      case V4L2_PIX_FMT_JPEG:
        data->pixelformat = IMGDATA_PIX_FMT_JPEG;
        break;

      default: /* V4L2_PIX_FMT_JPEG_WITH_SUBIMG */
        data->pixelformat = IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG;
        break;
    }
}

static void convert_to_imgsensorfmt(FAR video_format_t *video,
                                    FAR imgsensor_format_t *sensor)
{
  ASSERT(video && sensor);

  sensor->width  = video->width;
  sensor->height = video->height;
  switch (video->pixelformat)
    {
      case V4L2_PIX_FMT_NV12:
        sensor->pixelformat = IMGSENSOR_PIX_FMT_NV12;
        break;

      case V4L2_PIX_FMT_YUV420:
        sensor->pixelformat = IMGSENSOR_PIX_FMT_YUV420P;
        break;

      case V4L2_PIX_FMT_YUYV:
        sensor->pixelformat = IMGSENSOR_PIX_FMT_YUYV;
        break;

      case V4L2_PIX_FMT_UYVY:
        sensor->pixelformat = IMGSENSOR_PIX_FMT_UYVY;
        break;

      case V4L2_PIX_FMT_RGB565:
        sensor->pixelformat = IMGSENSOR_PIX_FMT_RGB565;
        break;

      case V4L2_PIX_FMT_JPEG:
        sensor->pixelformat = IMGSENSOR_PIX_FMT_JPEG;
        break;

      default: /* V4L2_PIX_FMT_JPEG_WITH_SUBIMG */
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

static bool is_clipped(FAR struct v4l2_rect *clip)
{
  bool ret = false;

  if (clip)
    {
      if (clip->left  != 0 || clip->top    != 0 ||
          clip->width != 0 || clip->height != 0)
        {
          ret = true;
        }
    }

  return ret;
}

static void get_clipped_format(uint8_t nr_fmt,
                               FAR video_format_t *fmt,
                               FAR struct v4l2_rect *clip,
                               FAR video_format_t *c_fmt)
{
  DEBUGASSERT(fmt && c_fmt);

  if (is_clipped(clip))
    {
      c_fmt[CAPTURE_FMT_MAIN].width  = clip->width;
      c_fmt[CAPTURE_FMT_MAIN].height = clip->height;
      c_fmt[CAPTURE_FMT_MAIN].pixelformat =
        fmt[CAPTURE_FMT_MAIN].pixelformat;

      if (nr_fmt > 1)
        {
          /* Clipped size of  thumbnail is
           * small as ratio of main size and thumbnail size.
           */

          memcpy(&c_fmt[CAPTURE_FMT_SUB],
                 &fmt[CAPTURE_FMT_SUB],
                 sizeof(video_format_t));

          c_fmt[CAPTURE_FMT_SUB].width =
            (uint32_t)c_fmt[CAPTURE_FMT_SUB].width *
            clip->width / fmt[CAPTURE_FMT_MAIN].width;

          c_fmt[CAPTURE_FMT_SUB].height =
            (uint32_t)c_fmt[CAPTURE_FMT_SUB].height *
            clip->height / fmt[CAPTURE_FMT_MAIN].height;
        }
    }
  else
    {
      memcpy(c_fmt, fmt, nr_fmt * sizeof(video_format_t));
    }
}

static int start_capture(FAR struct capture_mng_s *cmng,
                         enum v4l2_buf_type type,
                         uint8_t nr_fmt,
                         FAR video_format_t *fmt,
                         FAR struct v4l2_rect *clip,
                         FAR struct v4l2_fract *interval,
                         uintptr_t bufaddr, uint32_t bufsize)
{
  video_format_t c_fmt[MAX_CAPTURE_FMT];
  imgdata_format_t df[MAX_CAPTURE_FMT];
  imgsensor_format_t sf[MAX_CAPTURE_FMT];
  imgdata_interval_t di;
  imgsensor_interval_t si;

  ASSERT(fmt && interval && cmng->imgsensor && cmng->imgdata);

  get_clipped_format(nr_fmt, fmt, clip, c_fmt);

  convert_to_imgdatafmt(&c_fmt[CAPTURE_FMT_MAIN], &df[IMGDATA_FMT_MAIN]);
  convert_to_imgdatafmt(&c_fmt[CAPTURE_FMT_SUB], &df[IMGDATA_FMT_SUB]);
  convert_to_imgdatainterval(interval, &di);
  convert_to_imgsensorfmt(&fmt[CAPTURE_FMT_MAIN], &sf[IMGSENSOR_FMT_MAIN]);
  convert_to_imgsensorfmt(&fmt[CAPTURE_FMT_SUB], &sf[IMGSENSOR_FMT_SUB]);
  convert_to_imgsensorinterval(interval, &si);

  IMGDATA_SET_BUF(cmng->imgdata,
     nr_fmt, df, (FAR uint8_t *)bufaddr, bufsize);
  IMGDATA_START_CAPTURE(cmng->imgdata,
     nr_fmt, df, &di, complete_capture, cmng);
  IMGSENSOR_START_CAPTURE(cmng->imgsensor,
     type == V4L2_BUF_TYPE_VIDEO_CAPTURE ?
     IMGSENSOR_STREAM_TYPE_VIDEO : IMGSENSOR_STREAM_TYPE_STILL,
     nr_fmt, sf, &si);
  return OK;
}

static void stop_capture(FAR struct capture_mng_s *cmng,
                         enum v4l2_buf_type type)
{
  ASSERT(cmng->imgsensor && cmng->imgdata);

  IMGDATA_STOP_CAPTURE(cmng->imgdata);
  IMGSENSOR_STOP_CAPTURE(cmng->imgsensor,
    type == V4L2_BUF_TYPE_VIDEO_CAPTURE ?
    IMGSENSOR_STREAM_TYPE_VIDEO : IMGSENSOR_STREAM_TYPE_STILL);
}

static void change_capture_state(FAR capture_mng_t *cmng,
                                 enum capture_state_e next_state)
{
  enum capture_state_e current_state = cmng->capture_inf.state;
  enum capture_state_e updated_next_state = next_state;

  if (current_state != CAPTURE_STATE_CAPTURE &&
      next_state    == CAPTURE_STATE_CAPTURE)
    {
      FAR vbuf_container_t *container =
        video_framebuff_get_vacant_container(&cmng->capture_inf.bufinf);
      if (container != NULL)
        {
          cmng->capture_inf.seqnum = 0;
          start_capture(cmng,
                        V4L2_BUF_TYPE_VIDEO_CAPTURE,
                        cmng->capture_inf.nr_fmt,
                        cmng->capture_inf.fmt,
                        &cmng->capture_inf.clip,
                        &cmng->capture_inf.frame_interval,
                        container->buf.m.userptr,
                        container->buf.length);
        }
      else
        {
          updated_next_state = CAPTURE_STATE_STREAMON;
        }
    }
  else if (current_state == CAPTURE_STATE_CAPTURE &&
           next_state    != CAPTURE_STATE_CAPTURE)
    {
          stop_capture(cmng, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    }

  cmng->capture_inf.state = updated_next_state;
}

static bool is_taking_still_picture(FAR capture_mng_t *cmng)
{
  return cmng->still_inf.state == CAPTURE_STATE_STREAMON ||
         cmng->still_inf.state == CAPTURE_STATE_CAPTURE;
}

static bool is_bufsize_sufficient(FAR capture_mng_t *cmng, uint32_t bufsize)
{
  /* Depend on format, frame size, and JPEG compression quality */

  return true;
}

static void initialize_frame_setting(FAR struct imgsensor_s *imgsensor,
                                     FAR uint8_t *nr_fmt,
                                     FAR video_format_t *fmt,
                                     FAR struct v4l2_fract *interval)
{
  ASSERT(nr_fmt && fmt && interval);

  /* Initial setting : QVGA YUV4:2:2 15FPS */

  *nr_fmt = 1;
  if (imgsensor && imgsensor->frmsizes)
    {
      if (imgsensor->frmsizes[0].type == V4L2_FRMSIZE_TYPE_DISCRETE)
        {
          fmt[CAPTURE_FMT_MAIN].width =
            imgsensor->frmsizes[0].discrete.width;
          fmt[CAPTURE_FMT_MAIN].height =
            imgsensor->frmsizes[0].discrete.height;
        }
      else
        {
          fmt[CAPTURE_FMT_MAIN].width =
            imgsensor->frmsizes[0].stepwise.min_width;
          fmt[CAPTURE_FMT_MAIN].height =
            imgsensor->frmsizes[0].stepwise.min_height;
        }
    }
  else
    {
      fmt[CAPTURE_FMT_MAIN].width  = VIDEO_HSIZE_QVGA;
      fmt[CAPTURE_FMT_MAIN].height = VIDEO_VSIZE_QVGA;
    }

  if (imgsensor && imgsensor->fmtdescs)
    {
      fmt[CAPTURE_FMT_MAIN].pixelformat = imgsensor->fmtdescs[0].pixelformat;
    }
  else
    {
      fmt[CAPTURE_FMT_MAIN].pixelformat = V4L2_PIX_FMT_UYVY;
    }

  if (imgsensor && imgsensor->frmintervals)
    {
      if (imgsensor->frmintervals[0].type == V4L2_FRMIVAL_TYPE_DISCRETE)
        {
          interval->denominator =
            imgsensor->frmintervals[0].discrete.denominator;
          interval->numerator =
            imgsensor->frmintervals[0].discrete.numerator;
        }
      else
        {
          interval->denominator =
            imgsensor->frmintervals[0].stepwise.min.denominator;
          interval->numerator =
            imgsensor->frmintervals[0].stepwise.min.numerator;
        }
    }
  else
    {
      interval->denominator = 15;
      interval->numerator   = 1;
    }
}

static void initialize_streamresources(FAR capture_type_inf_t *type_inf,
                                       FAR capture_mng_t *cmng)
{
  memset(type_inf, 0, sizeof(capture_type_inf_t));
  type_inf->remaining_capnum = REMAINING_CAPNUM_INFINITY;
  nxmutex_init(&type_inf->lock_state);
  nxsem_init(&type_inf->wait_capture.dqbuf_wait_flg, 0, 0);
  initialize_frame_setting(cmng->imgsensor, &type_inf->nr_fmt,
                           type_inf->fmt,
                           &type_inf->frame_interval);
  video_framebuff_init(&type_inf->bufinf);
}

static int32_t get_default_value(FAR capture_mng_t *cmng, uint32_t id)
{
  imgsensor_supported_value_t value;
  int ret;

  if (cmng->imgsensor == NULL)
    {
      /* Don't care(unsupported parameter) */

      return 0;
    }

  ret = IMGSENSOR_GET_SUPPORTED_VALUE(cmng->imgsensor, id, &value);
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

static int32_t initialize_scene_gamma(FAR capture_mng_t *cmng,
                                      FAR uint8_t **gamma)
{
  imgsensor_supported_value_t sup_val;
  imgsensor_value_t val;
  int32_t sz;
  int ret;

  *gamma = NULL;

  ASSERT(cmng->imgsensor);

  ret = IMGSENSOR_GET_SUPPORTED_VALUE(cmng->imgsensor,
          IMGSENSOR_ID_GAMMA_CURVE, &sup_val);
  if (ret != OK)
    {
      /* Unsupported parameter */

      return 0;
    }

  switch (sup_val.type)
    {
      case IMGSENSOR_CTRL_TYPE_U8:
        sz = sup_val.u.elems.nr_elems * sizeof(uint8_t);
        if (sz / sizeof(uint8_t) != sup_val.u.elems.nr_elems)
          {
            /* Multiplication overflow */

            return 0;
          }

        break;

      case IMGSENSOR_CTRL_TYPE_U16:
        sz = sup_val.u.elems.nr_elems * sizeof(uint16_t);
        if (sz / sizeof(uint16_t) != sup_val.u.elems.nr_elems)
          {
            /* Multiplication overflow */

            return 0;
          }

        break;

      default: /* IMGSENSOR_CTRL_TYPE_U32 */
        sz = sup_val.u.elems.nr_elems * sizeof(uint32_t);
        if (sz / sizeof(uint32_t) != sup_val.u.elems.nr_elems)
          {
            /* Multiplication overflow */

            return 0;
          }

        break;
    }

  *gamma = kmm_malloc(sz);
  val.p_u8 = (FAR uint8_t *)*gamma;
  IMGSENSOR_GET_VALUE(cmng->imgsensor, IMGSENSOR_ID_GAMMA_CURVE, sz, &val);
  return sz;
}

static int initialize_scene_parameter(FAR capture_mng_t *cmng,
                                      enum v4l2_scene_mode mode,
                                      FAR capture_scene_params_t **vsp)
{
  FAR capture_scene_params_t *sp =
    kmm_malloc(sizeof(capture_scene_params_t));

  if (!sp)
    {
      return -ENOMEM;
    }

  sp->mode            = mode;
  sp->brightness      = get_default_value(cmng, IMGSENSOR_ID_BRIGHTNESS);
  sp->contrast        = get_default_value(cmng, IMGSENSOR_ID_CONTRAST);
  sp->saturation      = get_default_value(cmng, IMGSENSOR_ID_SATURATION);
  sp->hue             = get_default_value(cmng, IMGSENSOR_ID_HUE);
  sp->awb             = get_default_value(cmng,
                                          IMGSENSOR_ID_AUTO_WHITE_BALANCE);
  sp->red             = get_default_value(cmng, IMGSENSOR_ID_RED_BALANCE);
  sp->blue            = get_default_value(cmng, IMGSENSOR_ID_BLUE_BALANCE);
  sp->gamma           = get_default_value(cmng, IMGSENSOR_ID_GAMMA);
  sp->gamma_curve_sz  = initialize_scene_gamma(cmng, &sp->gamma_curve);
  sp->ev              = get_default_value(cmng, IMGSENSOR_ID_EXPOSURE);
  sp->hflip_video     = get_default_value(cmng, IMGSENSOR_ID_HFLIP_VIDEO);
  sp->vflip_video     = get_default_value(cmng, IMGSENSOR_ID_VFLIP_VIDEO);
  sp->hflip_still     = get_default_value(cmng, IMGSENSOR_ID_HFLIP_STILL);
  sp->vflip_still     = get_default_value(cmng, IMGSENSOR_ID_VFLIP_STILL);
  sp->sharpness       = get_default_value(cmng, IMGSENSOR_ID_SHARPNESS);
  sp->colorfx         = get_default_value(cmng, IMGSENSOR_ID_COLORFX);
  sp->auto_brightness = get_default_value(cmng, IMGSENSOR_ID_AUTOBRIGHTNESS);
  sp->rotate          = get_default_value(cmng, IMGSENSOR_ID_ROTATE);
  sp->ae              = get_default_value(cmng, IMGSENSOR_ID_EXPOSURE_AUTO);
  sp->exposure_time   = get_default_value(cmng,
                                         IMGSENSOR_ID_EXPOSURE_ABSOLUTE);
  sp->focus           = get_default_value(cmng, IMGSENSOR_ID_FOCUS_ABSOLUTE);
  sp->af              = get_default_value(cmng, IMGSENSOR_ID_FOCUS_AUTO);
  sp->zoom            = get_default_value(cmng, IMGSENSOR_ID_ZOOM_ABSOLUTE);
  sp->iris            = get_default_value(cmng, IMGSENSOR_ID_IRIS_ABSOLUTE);
  sp->wb              = get_default_value(cmng,
                                          IMGSENSOR_ID_AUTO_N_PRESET_WB);
  sp->wdr             = get_default_value(cmng,
                                          IMGSENSOR_ID_WIDE_DYNAMIC_RANGE);
  sp->stabilization   = get_default_value(cmng,
                                          IMGSENSOR_ID_IMG_STABILIZATION);
  sp->iso_auto        = get_default_value(cmng,
                                          IMGSENSOR_ID_ISO_SENSITIVITY_AUTO);
  sp->iso             = get_default_value(cmng,
                                          IMGSENSOR_ID_ISO_SENSITIVITY);
  sp->meter           = get_default_value(cmng,
                                          IMGSENSOR_ID_EXPOSURE_METERING);
  sp->threea_lock     = get_default_value(cmng, IMGSENSOR_ID_3A_LOCK);
  sp->led             = get_default_value(cmng, IMGSENSOR_ID_FLASH_LED_MODE);
  sp->jpeg_quality    = get_default_value(cmng, IMGSENSOR_ID_JPEG_QUALITY);

  *vsp = sp;

  return OK;
}

static void initialize_scenes_parameter(FAR capture_mng_t *cmng)
{
  memset(cmng->capture_scene_param,
         0, sizeof(cmng->capture_scene_param));

  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_NONE,
      &cmng->capture_scene_param[cmng->capture_scence_num++]);
#ifdef CONFIG_VIDEO_SCENE_BACKLIGHT
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_BACKLIGHT,
           &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_BACKLIGHT */
#ifdef CONFIG_VIDEO_SCENE_BEACHSNOW
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_BEACH_SNOW,
              &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_BEACHSNOW */
#ifdef CONFIG_VIDEO_SCENE_CANDLELIGHT
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_CANDLE_LIGHT,
                &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_CANDLELIGHT */
#ifdef CONFIG_VIDEO_SCENE_DAWNDUSK
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_DAWN_DUSK,
             &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_DAWNDUSK */
#ifdef CONFIG_VIDEO_SCENE_FALLCOLORS
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_FALL_COLORS,
               &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_FALLCOLORS */
#ifdef CONFIG_VIDEO_SCENE_FIREWORKS
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_FIREWORKS,
              &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_FIREWORKS */
#ifdef CONFIG_VIDEO_SCENE_LANDSCAPE
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_LANDSCAPE,
              &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_LANDSCAPE */
#ifdef CONFIG_VIDEO_SCENE_NIGHT
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_NIGHT,
          &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_NIGHT */
#ifdef CONFIG_VIDEO_SCENE_PARTYINDOOR
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_PARTY_INDOOR,
                &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_PARTYINDOOR */
#ifdef CONFIG_VIDEO_SCENE_PORTRAIT
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_PORTRAIT,
             &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_PORTRAIT */
#ifdef CONFIG_VIDEO_SCENE_SPORTS
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_SPORTS,
           &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_SPORTS */
#ifdef CONFIG_VIDEO_SCENE_SUNSET
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_SUNSET,
           &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_SUNSET */
#ifdef CONFIG_VIDEO_SCENE_TEXT
  initialize_scene_parameter(cmng, V4L2_SCENE_MODE_TEXT,
         &cmng->capture_scene_param[cmng->capture_scence_num++]);
#endif /* CONFIG_VIDEO_SCENE_TEXT */
}

static void initialize_resources(FAR capture_mng_t *cmng)
{
  initialize_streamresources(&cmng->capture_inf, cmng);
  initialize_streamresources(&cmng->still_inf, cmng);
  initialize_scenes_parameter(cmng);
}

static void cleanup_streamresources(FAR capture_type_inf_t *type_inf,
                                    FAR capture_mng_t *cmng)
{
  video_framebuff_uninit(&type_inf->bufinf);
  nxsem_destroy(&type_inf->wait_capture.dqbuf_wait_flg);
  nxmutex_destroy(&type_inf->lock_state);
  if (type_inf->bufheap != NULL)
    {
      if (cmng->imgdata->ops->free)
        {
          cmng->imgdata->ops->free(cmng->imgdata, type_inf->bufheap);
        }
      else
        {
          kumm_free(type_inf->bufheap);
        }

      type_inf->bufheap = NULL;
    }
}

static void cleanup_scene_parameter(FAR capture_scene_params_t **vsp)
{
  FAR capture_scene_params_t *sp = *vsp;
  ASSERT(sp);

  if (sp->gamma_curve != NULL)
    {
      kmm_free(sp->gamma_curve);
      sp->gamma_curve = NULL;
      sp->gamma_curve_sz = 0;
    }

  kmm_free(sp);
  *vsp = NULL;
}

static void cleanup_scenes_parameter(FAR capture_mng_t *cmng)
{
  int i;

  for (i = 0; i < cmng->capture_scence_num; i++)
    {
      cleanup_scene_parameter(&cmng->capture_scene_param[i]);
    }

  cmng->capture_scence_num = 0;
}

static void cleanup_resources(FAR capture_mng_t *cmng)
{
  /* If in capture, stop */

  if (cmng->capture_inf.state == CAPTURE_STATE_CAPTURE)
    {
      stop_capture(cmng, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    }

  if (cmng->still_inf.state == CAPTURE_STATE_CAPTURE)
    {
      stop_capture(cmng, V4L2_BUF_TYPE_STILL_CAPTURE);
    }

  /* Clean up resource */

  cleanup_streamresources(&cmng->capture_inf, cmng);
  cleanup_streamresources(&cmng->still_inf, cmng);
  cleanup_scenes_parameter(cmng);
}

static bool is_sem_waited(FAR sem_t *sem)
{
  int semcount;

  return nxsem_get_value(sem, &semcount) == OK && semcount < 0;
}

static int validate_frame_setting(FAR capture_mng_t *cmng,
                                  enum v4l2_buf_type type,
                                  uint8_t nr_fmt,
                                  FAR video_format_t *vfmt,
                                  FAR struct v4l2_rect *clip,
                                  FAR struct v4l2_fract *interval)
{
  video_format_t c_fmt[MAX_CAPTURE_FMT];
  imgdata_format_t df[MAX_CAPTURE_FMT];
  imgsensor_format_t sf[MAX_CAPTURE_FMT];
  imgdata_interval_t di;
  imgsensor_interval_t si;
  int ret;

  ASSERT(vfmt && interval && cmng->imgsensor && cmng->imgdata);

  /* Return OK only in case both image data driver and
   * image sensor driver support.
   */

  get_clipped_format(nr_fmt, vfmt, clip, c_fmt);

  convert_to_imgdatafmt(&c_fmt[CAPTURE_FMT_MAIN], &df[IMGDATA_FMT_MAIN]);
  convert_to_imgdatafmt(&c_fmt[CAPTURE_FMT_SUB], &df[IMGDATA_FMT_SUB]);
  convert_to_imgdatainterval(interval, &di);
  convert_to_imgsensorfmt(&vfmt[CAPTURE_FMT_MAIN], &sf[IMGSENSOR_FMT_MAIN]);
  convert_to_imgsensorfmt(&vfmt[CAPTURE_FMT_SUB], &sf[IMGSENSOR_FMT_SUB]);
  convert_to_imgsensorinterval(interval, &si);

  ret = IMGSENSOR_VALIDATE_FRAME_SETTING(cmng->imgsensor,
            type == V4L2_BUF_TYPE_VIDEO_CAPTURE ?
              IMGSENSOR_STREAM_TYPE_VIDEO : IMGSENSOR_STREAM_TYPE_STILL,
            nr_fmt, sf, &si);
  if (ret != OK)
    {
      return ret;
    }

  return IMGDATA_VALIDATE_FRAME_SETTING(cmng->imgdata, nr_fmt, df, &di);
}

static size_t get_bufsize(FAR video_format_t *vf)
{
  uint32_t width  = vf->width;
  uint32_t height = vf->height;
  size_t ret = width * height;

  switch (vf->pixelformat)
    {
      case V4L2_PIX_FMT_NV12:
      case V4L2_PIX_FMT_YUV420:
        return ret * 3 / 2;
      case V4L2_PIX_FMT_YUYV:
      case V4L2_PIX_FMT_UYVY:
      case V4L2_PIX_FMT_RGB565:
      case V4L2_PIX_FMT_JPEG:
      default:
        return ret * 2;
    }
}

static size_t get_heapsize(FAR capture_type_inf_t *type_inf)
{
  return type_inf->bufinf.container_size *
         get_bufsize(&type_inf->fmt[CAPTURE_FMT_MAIN]);
}

static bool validate_clip_range(int32_t pos, uint32_t c_sz, uint16_t frm_sz)
{
  return pos >= 0 && c_sz <= frm_sz && pos + c_sz <= frm_sz;
}

static bool validate_clip_setting(FAR struct v4l2_rect *clip,
                                  FAR video_format_t *fmt)
{
  DEBUGASSERT(clip && fmt);

  /* Not permit the setting which do not fit inside frame size. */

  return validate_clip_range(clip->left, clip->width,  fmt->width) &&
         validate_clip_range(clip->top,  clip->height, fmt->height);
}

static void set_parameter_name(uint32_t id, FAR char *name)
{
  int size =
    sizeof(g_capture_parameter_name) / sizeof(capture_parameter_name_t);
  int cnt;

  for (cnt = 0; cnt < size; cnt++)
    {
      if (g_capture_parameter_name[cnt].id == id)
        {
          break;
        }
    }

  ASSERT(cnt < size);

  /* copy size = 32 is due to V4L2 specification. */

  strlcpy(name, g_capture_parameter_name[cnt].name, 32);
}

static int set_intvalue(FAR struct capture_mng_s *cmng,
                        uint32_t id, int32_t value32)
{
  imgsensor_value_t value;

  ASSERT(cmng->imgsensor);

  value.value32 = value32;
  return IMGSENSOR_SET_VALUE(cmng->imgsensor, id, sizeof(int32_t), value);
}

static int set_pvalue(FAR struct capture_mng_s *cmng,
                      uint32_t id, int size, void *pval)
{
  imgsensor_value_t value;

  ASSERT(cmng->imgsensor);

  value.p_u8 = (FAR uint8_t *)pval;
  return IMGSENSOR_SET_VALUE(cmng->imgsensor, id, size, value);
}

static capture_scene_params_t *search_scene_param(FAR capture_mng_t *cmng,
                                                  enum v4l2_scene_mode mode)
{
  int i;

  for (i = 0; i < cmng->capture_scence_num; i++)
    {
      if (cmng->capture_scene_param[i]->mode == mode)
        {
          return cmng->capture_scene_param[i];
        }
    }

  return NULL;
}

static int reflect_scene_parameter(FAR capture_mng_t *cmng,
                                   enum v4l2_scene_mode mode)
{
  capture_scene_params_t *sp;

  sp = search_scene_param(cmng, mode);
  if (sp == NULL)
    {
      /* Unsupported scene mode */

      return -EINVAL;
    }

  set_intvalue(cmng, IMGSENSOR_ID_BRIGHTNESS, sp->brightness);
  set_intvalue(cmng, IMGSENSOR_ID_CONTRAST, sp->contrast);
  set_intvalue(cmng, IMGSENSOR_ID_SATURATION, sp->saturation);
  set_intvalue(cmng, IMGSENSOR_ID_HUE , sp->hue);
  set_intvalue(cmng, IMGSENSOR_ID_AUTO_WHITE_BALANCE, sp->awb);
  set_intvalue(cmng, IMGSENSOR_ID_RED_BALANCE , sp->red);
  set_intvalue(cmng, IMGSENSOR_ID_BLUE_BALANCE, sp->blue);
  set_intvalue(cmng, IMGSENSOR_ID_GAMMA, sp->gamma);
  set_pvalue(cmng, IMGSENSOR_ID_GAMMA_CURVE,
             sp->gamma_curve_sz, sp->gamma_curve);
  set_intvalue(cmng, IMGSENSOR_ID_EXPOSURE, sp->ev);
  set_intvalue(cmng, IMGSENSOR_ID_HFLIP_VIDEO, sp->hflip_video);
  set_intvalue(cmng, IMGSENSOR_ID_VFLIP_VIDEO, sp->vflip_video);
  set_intvalue(cmng, IMGSENSOR_ID_HFLIP_STILL, sp->hflip_still);
  set_intvalue(cmng, IMGSENSOR_ID_VFLIP_STILL, sp->vflip_still);
  set_intvalue(cmng, IMGSENSOR_ID_SHARPNESS, sp->sharpness);
  set_intvalue(cmng, IMGSENSOR_ID_COLORFX, sp->colorfx);
  set_intvalue(cmng, IMGSENSOR_ID_AUTOBRIGHTNESS, sp->auto_brightness);
  set_intvalue(cmng, IMGSENSOR_ID_ROTATE, sp->rotate);
  set_intvalue(cmng, IMGSENSOR_ID_EXPOSURE_AUTO, sp->ae);
  if (sp->ae == V4L2_EXPOSURE_MANUAL ||
      sp->ae == V4L2_EXPOSURE_SHUTTER_PRIORITY)
    {
      set_intvalue(cmng, IMGSENSOR_ID_EXPOSURE_ABSOLUTE, sp->exposure_time);
    }

  set_intvalue(cmng, IMGSENSOR_ID_FOCUS_ABSOLUTE, sp->focus);
  set_intvalue(cmng, IMGSENSOR_ID_FOCUS_AUTO, sp->af);
  set_intvalue(cmng, IMGSENSOR_ID_ZOOM_ABSOLUTE, sp->zoom);
  if (sp->ae == V4L2_EXPOSURE_MANUAL ||
      sp->ae == V4L2_EXPOSURE_APERTURE_PRIORITY)
    {
      set_intvalue(cmng, IMGSENSOR_ID_IRIS_ABSOLUTE, sp->iris);
    }

  set_intvalue(cmng, IMGSENSOR_ID_AUTO_N_PRESET_WB, sp->wb);
  set_intvalue(cmng, IMGSENSOR_ID_WIDE_DYNAMIC_RANGE, sp->wdr);
  set_intvalue(cmng, IMGSENSOR_ID_IMG_STABILIZATION, sp->stabilization);
  set_intvalue(cmng, IMGSENSOR_ID_ISO_SENSITIVITY_AUTO, sp->iso_auto);
  if (sp->iso_auto == V4L2_ISO_SENSITIVITY_MANUAL)
    {
      set_intvalue(cmng, IMGSENSOR_ID_ISO_SENSITIVITY, sp->iso);
    }

  set_intvalue(cmng, IMGSENSOR_ID_EXPOSURE_METERING, sp->meter);
  set_intvalue(cmng, IMGSENSOR_ID_3A_LOCK, sp->threea_lock);
  set_intvalue(cmng, IMGSENSOR_ID_FLASH_LED_MODE, sp->led);
  set_intvalue(cmng, IMGSENSOR_ID_JPEG_QUALITY, sp->jpeg_quality);

  cmng->capture_scene_mode = mode;
  return OK;
}

static int read_scene_param(FAR struct capture_mng_s *cmng,
                            enum v4l2_scene_mode mode,
                            uint32_t id,
                            FAR struct v4l2_ext_control *control)
{
  imgsensor_supported_value_t value;
  capture_scene_params_t *sp;
  int ret = OK;

  ASSERT(cmng->imgsensor);

  if (control == NULL)
    {
      return -EINVAL;
    }

  sp = search_scene_param(cmng, mode);
  if (sp == NULL)
    {
      /* Unsupported scene mode */

      return -EINVAL;
    }

  ret = IMGSENSOR_GET_SUPPORTED_VALUE(cmng->imgsensor, id, &value);
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
        control->value = sp->colorfx == V4L2_COLORFX_BW;
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

      case IMGSENSOR_ID_SPOT_POSITION:
        control->value = sp->spot_pos;
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
        break;
    }

  return ret;
}

static int check_range(int64_t value,
                       int64_t min,
                       int64_t max,
                       uint64_t step)
{
  if (value < min || value > max ||
      (value - min) % step != 0)
    {
      return -EINVAL;
    }

  return OK;
}

static int save_scene_param(FAR capture_mng_t *cmng,
                            enum v4l2_scene_mode mode,
                            uint32_t id,
                            FAR struct v4l2_ext_control *control)
{
  imgsensor_supported_value_t value;
  FAR imgsensor_capability_range_t *range = &value.u.range;
  FAR imgsensor_capability_discrete_t *disc = &value.u.discrete;
  FAR imgsensor_capability_elems_t *elem = &value.u.elems;
  FAR capture_scene_params_t *sp;
  int ret;
  int i;

  ASSERT(cmng->imgsensor);

  sp = search_scene_param(cmng, mode);
  if (sp == NULL)
    {
      /* Unsupported scene mode */

      return -EINVAL;
    }

  ret = IMGSENSOR_GET_SUPPORTED_VALUE(cmng->imgsensor, id, &value);
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
        if (control->size < elem->nr_elems * sizeof(uint8_t))
          {
            return -EINVAL;
          }

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
        if (control->size < elem->nr_elems * sizeof(uint16_t))
          {
            return -EINVAL;
          }

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
        if (control->size < elem->nr_elems * sizeof(uint32_t))
          {
            return -EINVAL;
          }

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
        sp->colorfx = control->value ? V4L2_COLORFX_BW : V4L2_COLORFX_NONE;
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

      case IMGSENSOR_ID_SPOT_POSITION:
        sp->spot_pos = control->value;
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

/* Callback function which device driver call when capture has done. */

static int complete_capture(uint8_t err_code,
                            uint32_t datasize,
                            FAR const struct timeval *ts,
                            FAR void *arg)
{
  FAR capture_mng_t *cmng = (FAR capture_mng_t *)arg;
  FAR capture_type_inf_t *type_inf;
  FAR vbuf_container_t *container = NULL;
  enum v4l2_buf_type buf_type;
  irqstate_t           flags;
  imgdata_format_t df[MAX_CAPTURE_FMT];
  video_format_t c_fmt[MAX_CAPTURE_FMT];

  flags = enter_critical_section();

  buf_type = cmng->still_inf.state == CAPTURE_STATE_CAPTURE ?
               V4L2_BUF_TYPE_STILL_CAPTURE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

  type_inf = get_capture_type_inf(cmng, buf_type);
  if (type_inf == NULL)
    {
      leave_critical_section(flags);
      return -EINVAL;
    }

  poll_notify(&type_inf->fds, 1, POLLIN);

  if (err_code == 0)
    {
      type_inf->bufinf.vbuf_next->buf.flags = 0;
      if (type_inf->remaining_capnum > 0)
        {
          type_inf->remaining_capnum--;
        }
    }
  else
    {
      type_inf->bufinf.vbuf_next->buf.flags = V4L2_BUF_FLAG_ERROR;
    }

  type_inf->bufinf.vbuf_next->buf.bytesused = datasize;
  if (ts != NULL)
    {
      type_inf->bufinf.vbuf_next->buf.timestamp = *ts;
    }

  video_framebuff_capture_done(&type_inf->bufinf);

  if (is_sem_waited(&type_inf->wait_capture.dqbuf_wait_flg))
    {
      /* If waiting capture in DQBUF,
       * get/save container and unlock wait
       */

      type_inf->wait_capture.done_container =
        video_framebuff_pop_curr_container(&type_inf->bufinf);
      type_inf->wait_capture.waitend_cause = WAITEND_CAUSE_CAPTUREDONE;
      nxsem_post(&type_inf->wait_capture.dqbuf_wait_flg);
    }

  if (type_inf->remaining_capnum == 0)
    {
      stop_capture(cmng, buf_type);
      type_inf->state = CAPTURE_STATE_STREAMOFF;

      /* If stop still stream, notify it to video stream */

      if (buf_type == V4L2_BUF_TYPE_STILL_CAPTURE &&
          is_sem_waited(&cmng->capture_inf.wait_capture.dqbuf_wait_flg))
        {
          cmng->capture_inf.wait_capture.waitend_cause =
            WAITEND_CAUSE_STILLSTOP;
          nxsem_post(&cmng->capture_inf.wait_capture.dqbuf_wait_flg);
        }
    }
  else
    {
      container = video_framebuff_get_vacant_container(&type_inf->bufinf);
      if (container == NULL)
        {
          stop_capture(cmng, buf_type);
          type_inf->state = CAPTURE_STATE_STREAMON;
        }
      else
        {
          get_clipped_format(type_inf->nr_fmt,
                             type_inf->fmt,
                             &type_inf->clip,
                             c_fmt);

          convert_to_imgdatafmt(&c_fmt[CAPTURE_FMT_MAIN],
                                &df[IMGDATA_FMT_MAIN]);
          convert_to_imgdatafmt(&c_fmt[CAPTURE_FMT_SUB],
                                &df[IMGDATA_FMT_SUB]);

          IMGDATA_SET_BUF(cmng->imgdata,
            type_inf->nr_fmt,
            df,
            (FAR uint8_t *)container->buf.m.userptr,
            container->buf.length);
          container->buf.sequence = type_inf->seqnum++;
        }
    }

  leave_critical_section(flags);
  return OK;
}

static FAR struct imgsensor_s *
get_connected_imgsensor(FAR struct imgsensor_s **sensors,
                        size_t sensor_num)
{
  FAR struct imgsensor_s *sensor = NULL;
  int i;

  for (i = 0; i < sensor_num; i++)
    {
      if (sensors[i] &&
          IMGSENSOR_IS_AVAILABLE(sensors[i]))
        {
          sensor = sensors[i];
          break;
        }
    }

  return sensor;
}

/****************************************************************************
 * Ioctl Functions
 ****************************************************************************/

static int capture_querycap(FAR struct file *filep,
                            FAR struct v4l2_capability *cap)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR const char *name;

  if (cmng == NULL || cap == NULL)
    {
      return -EINVAL;
    }

  ASSERT(cmng->imgsensor);

  name = IMGSENSOR_GET_DRIVER_NAME(cmng->imgsensor);
  if (name == NULL)
    {
      return -ENOTTY;
    }

  memset(cap, 0, sizeof(struct v4l2_capability));

  /* cap->driver needs to be NULL-terminated. */

  strlcpy((FAR char *)cap->driver, name, sizeof(cap->driver));
  cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

  return OK;
}

static int capture_g_input(FAR int *num)
{
  *num = 0;
  return OK;
}

static int capture_enum_input(FAR struct file *filep,
                              FAR struct v4l2_input *input)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR const char *name;

  if (cmng == NULL || input->index > 0)
    {
      return -EINVAL;
    }

  ASSERT(cmng->imgsensor);

  name = IMGSENSOR_GET_DRIVER_NAME(cmng->imgsensor);
  if (name == NULL)
    {
      return -ENOTTY;
    }

  memset(input, 0, sizeof(struct v4l2_input));
  strlcpy((FAR char *)input->name, name, sizeof(input->name));
  input->type = V4L2_INPUT_TYPE_CAMERA;

  return OK;
}

static int capture_reqbufs(FAR struct file *filep,
                           FAR struct v4l2_requestbuffers *reqbufs)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  struct imgdata_s *imgdata;
  irqstate_t flags;
  int ret = OK;

  if (cmng == NULL || reqbufs == NULL)
    {
      return -EINVAL;
    }

  imgdata  = cmng->imgdata;
  type_inf = get_capture_type_inf(cmng, reqbufs->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  if (type_inf->state == CAPTURE_STATE_CAPTURE)
    {
      /* In capture, REQBUFS is not permitted */

      ret = -EPERM;
    }
  else
    {
      if (reqbufs->count > V4L2_REQBUFS_COUNT_MAX)
        {
          reqbufs->count = V4L2_REQBUFS_COUNT_MAX;
        }

      video_framebuff_change_mode(&type_inf->bufinf, reqbufs->mode);
      ret = video_framebuff_realloc_container(&type_inf->bufinf,
                                              reqbufs->count);
      if (ret == OK && reqbufs->memory == V4L2_MEMORY_MMAP)
        {
          if (type_inf->bufheap != NULL)
            {
              if (imgdata->ops->free)
                {
                    imgdata->ops->free(imgdata, type_inf->bufheap);
                }
              else
                {
                  kumm_free(type_inf->bufheap);
                }
            }

          if (imgdata->ops->alloc)
            {
              type_inf->bufheap = imgdata->ops->alloc(imgdata, 32,
                reqbufs->count *
                get_bufsize(&type_inf->fmt[CAPTURE_FMT_MAIN]));
            }
          else
            {
              type_inf->bufheap = kumm_memalign(32, reqbufs->count *
                get_bufsize(&type_inf->fmt[CAPTURE_FMT_MAIN]));
            }

          if (type_inf->bufheap == NULL)
            {
              ret = -ENOMEM;
            }
        }
    }

  leave_critical_section(flags);
  return ret;
}

static int capture_querybuf(FAR struct file *filep,
                            FAR struct v4l2_buffer *buf)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;

  if (cmng == NULL || buf == NULL || buf->memory != V4L2_MEMORY_MMAP)
    {
      return -EINVAL;
    }

  type_inf = get_capture_type_inf(cmng, buf->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (buf->index >= type_inf->bufinf.container_size)
    {
      return -EINVAL;
    }

  buf->length = get_bufsize(&type_inf->fmt[CAPTURE_FMT_MAIN]);
  buf->m.offset = buf->length * buf->index;

  return OK;
}

static int capture_qbuf(FAR struct file *filep,
                        FAR struct v4l2_buffer *buf)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  FAR vbuf_container_t *container;
  enum capture_state_e next_capture_state;
  irqstate_t flags;

  if (cmng == NULL || buf == NULL)
    {
      return -EINVAL;
    }

  type_inf = get_capture_type_inf(cmng, buf->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (!is_bufsize_sufficient(cmng, buf->length))
    {
      return -EINVAL;
    }

  container = video_framebuff_get_container(&type_inf->bufinf);
  if (container == NULL)
    {
      return -ENOMEM;
    }

  memcpy(&container->buf, buf, sizeof(struct v4l2_buffer));
  if (buf->memory == V4L2_MEMORY_MMAP)
    {
      /* only use userptr inside the container */

      container->buf.length = get_bufsize(&type_inf->fmt[CAPTURE_FMT_MAIN]);
      container->buf.m.userptr = (unsigned long)(type_inf->bufheap +
                                 container->buf.length * buf->index);
    }

  video_framebuff_queue_container(&type_inf->bufinf, container);

  nxmutex_lock(&type_inf->lock_state);
  flags = enter_critical_section();
  if (type_inf->state == CAPTURE_STATE_STREAMON)
    {
      leave_critical_section(flags);

      if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
        {
          nxmutex_lock(&cmng->still_inf.lock_state);
          next_capture_state =
            estimate_next_capture_state(cmng, CAUSE_CAPTURE_START);
          change_capture_state(cmng, next_capture_state);
          nxmutex_unlock(&cmng->still_inf.lock_state);
        }
      else
        {
          container =
            video_framebuff_get_vacant_container(&type_inf->bufinf);
          if (container != NULL)
            {
              type_inf->seqnum = 0;
              start_capture(cmng,
                            buf->type,
                            type_inf->nr_fmt,
                            type_inf->fmt,
                            &type_inf->clip,
                            &type_inf->frame_interval,
                            container->buf.m.userptr,
                            container->buf.length);
              type_inf->state = CAPTURE_STATE_CAPTURE;
            }
        }
    }
  else
    {
      leave_critical_section(flags);
    }

  nxmutex_unlock(&type_inf->lock_state);
  return OK;
}

static int capture_dqbuf(FAR struct file *filep,
                         FAR struct v4l2_buffer *buf)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  FAR vbuf_container_t *container;
  FAR sem_t *dqbuf_wait_flg;
  enum capture_state_e next_capture_state;
  irqstate_t flags;

  if (cmng == NULL || buf == NULL)
    {
      return -EINVAL;
    }

  type_inf = get_capture_type_inf(cmng, buf->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  container = video_framebuff_dq_valid_container(&type_inf->bufinf);
  if (container == NULL)
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          return -EAGAIN;
        }

      /* Not yet done capture. Wait done */

      dqbuf_wait_flg = &type_inf->wait_capture.dqbuf_wait_flg;

      /* Loop until semaphore is unlocked by capture done or DQCANCEL */

      do
        {
          if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
            {
              /* If start capture condition is satisfied, start capture */

              flags = enter_critical_section();
              next_capture_state =
                estimate_next_capture_state(cmng, CAUSE_CAPTURE_DQBUF);
              change_capture_state(cmng, next_capture_state);
              leave_critical_section(flags);
            }

          nxsem_wait_uninterruptible(dqbuf_wait_flg);
        }
      while (type_inf->wait_capture.waitend_cause ==
             WAITEND_CAUSE_STILLSTOP);

      container = type_inf->wait_capture.done_container;
      if (container == NULL)
        {
          /* Waking up without captured data means abort.
           * Therefore, Check cause.
           */

          DEBUGASSERT(type_inf->wait_capture.waitend_cause ==
                      WAITEND_CAUSE_DQCANCEL);
          return -ECANCELED;
        }

      type_inf->wait_capture.done_container = NULL;
    }

  memcpy(buf, &container->buf, sizeof(struct v4l2_buffer));
  video_framebuff_free_container(&type_inf->bufinf, container);

  return OK;
}

static int capture_cancel_dqbuf(FAR struct file *filep,
                                enum v4l2_buf_type type)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;

  if (cmng == NULL)
    {
      return -EINVAL;
    }

  type_inf = get_capture_type_inf(cmng, type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (!is_sem_waited(&type_inf->wait_capture.dqbuf_wait_flg))
    {
      /* In not waiting DQBUF case, return OK */

      return OK;
    }

  type_inf->wait_capture.waitend_cause = WAITEND_CAUSE_DQCANCEL;

  /* If capture is done before nxsem_post, cause is overwritten */

  return nxsem_post(&type_inf->wait_capture.dqbuf_wait_flg);
}

static int capture_g_fmt(FAR struct file *filep,
                         FAR struct v4l2_format *fmt)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;

  if (cmng == NULL)
    {
      return -EINVAL;
    }

  type_inf = get_capture_type_inf(cmng, fmt->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  memset(&fmt->fmt, 0, sizeof(fmt->fmt));
  fmt->fmt.pix.width = type_inf->fmt[CAPTURE_FMT_MAIN].width;
  fmt->fmt.pix.height = type_inf->fmt[CAPTURE_FMT_MAIN].height;
  fmt->fmt.pix.pixelformat = type_inf->fmt[CAPTURE_FMT_MAIN].pixelformat;

  return OK;
}

static int capture_s_fmt(FAR struct file *filep,
                         FAR struct v4l2_format *fmt)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  int ret;

  if (cmng == NULL)
    {
      return -EINVAL;
    }

  ret = capture_try_fmt(filep, fmt);
  if (ret != 0)
    {
      return ret;
    }

  type_inf = get_capture_type_inf(cmng, fmt->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (type_inf->state != CAPTURE_STATE_STREAMOFF)
    {
      return -EBUSY;
    }

  switch (fmt->fmt.pix.pixelformat)
    {
      case V4L2_PIX_FMT_SUBIMG_UYVY:
      case V4L2_PIX_FMT_SUBIMG_RGB565:
        if (type_inf->fmt[CAPTURE_FMT_MAIN].pixelformat !=
            V4L2_PIX_FMT_JPEG_WITH_SUBIMG)
          {
            return -EPERM;
          }

        type_inf->fmt[CAPTURE_FMT_SUB].width  = fmt->fmt.pix.width;
        type_inf->fmt[CAPTURE_FMT_SUB].height = fmt->fmt.pix.height;
        type_inf->fmt[CAPTURE_FMT_SUB].pixelformat =
            fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_SUBIMG_UYVY ?
              V4L2_PIX_FMT_UYVY : V4L2_PIX_FMT_RGB565;
        type_inf->nr_fmt = 2;
        break;

      default:
        type_inf->fmt[CAPTURE_FMT_MAIN].width  = fmt->fmt.pix.width;
        type_inf->fmt[CAPTURE_FMT_MAIN].height = fmt->fmt.pix.height;
        type_inf->fmt[CAPTURE_FMT_MAIN].pixelformat =
                                          fmt->fmt.pix.pixelformat;
        type_inf->nr_fmt = 1;
        break;
    }

  return OK;
}

static int capture_try_fmt(FAR struct file *filep,
                           FAR struct v4l2_format *fmt)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  video_format_t vf[MAX_CAPTURE_FMT];
  uint8_t nr_fmt;

  if (cmng == NULL || fmt == NULL)
    {
      return -EINVAL;
    }

  ASSERT(cmng->imgsensor && cmng->imgdata);

  type_inf = get_capture_type_inf(cmng, fmt->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  switch (fmt->fmt.pix.pixelformat)
    {
      case V4L2_PIX_FMT_SUBIMG_UYVY:
      case V4L2_PIX_FMT_SUBIMG_RGB565:
        if (type_inf->fmt[CAPTURE_FMT_MAIN].pixelformat !=
            V4L2_PIX_FMT_JPEG_WITH_SUBIMG)
          {
            return -EPERM;
          }

        /* Validate both main image and subimage. */

        nr_fmt = 2;
        memcpy(&vf[CAPTURE_FMT_MAIN],
               &type_inf->fmt[CAPTURE_FMT_MAIN],
               sizeof(video_format_t));
        vf[CAPTURE_FMT_SUB].width       = fmt->fmt.pix.width;
        vf[CAPTURE_FMT_SUB].height      = fmt->fmt.pix.height;
        vf[CAPTURE_FMT_SUB].pixelformat =
            fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_SUBIMG_UYVY ?
              V4L2_PIX_FMT_UYVY : V4L2_PIX_FMT_RGB565;
        break;
      case V4L2_PIX_FMT_NV12:
      case V4L2_PIX_FMT_YUV420:
      case V4L2_PIX_FMT_YUYV:
      case V4L2_PIX_FMT_UYVY:
      case V4L2_PIX_FMT_RGB565:
      case V4L2_PIX_FMT_JPEG:
      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:
        nr_fmt = 1;
        vf[CAPTURE_FMT_MAIN].width       = fmt->fmt.pix.width;
        vf[CAPTURE_FMT_MAIN].height      = fmt->fmt.pix.height;
        vf[CAPTURE_FMT_MAIN].pixelformat = fmt->fmt.pix.pixelformat;
        break;

      default:
        return -EINVAL;
    }

  return validate_frame_setting(cmng,
                                fmt->type,
                                nr_fmt,
                                vf,
                                &type_inf->clip,
                                &type_inf->frame_interval);
}

static int capture_g_parm(FAR struct file *filep,
                          FAR struct v4l2_streamparm *parm)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  int ret = -EINVAL;

  if (cmng == NULL || parm == NULL)
    {
      return -EINVAL;
    }

  DEBUGASSERT(cmng->imgsensor);

  type_inf = get_capture_type_inf(cmng, parm->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  memset(&parm->parm, 0, sizeof(parm->parm));

  if (type_inf->state == CAPTURE_STATE_CAPTURE)
    {
      /* If capture is started and lower driver has the get_frame_interval(),
       * query lower driver.
       */

      ret = IMGSENSOR_GET_FRAME_INTERVAL(cmng->imgsensor, parm->type,
              (imgsensor_interval_t *)&parm->parm.capture.timeperframe);
    }

  if (ret != OK)
    {
      /* In no capture state or error case, return stored value. */

      memcpy(&parm->parm.capture.timeperframe,
             &type_inf->frame_interval,
             sizeof(struct v4l2_fract));
    }

  parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
  return OK;
}

static int capture_s_parm(FAR struct file *filep,
                          FAR struct v4l2_streamparm *parm)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  int ret;

  if (cmng == NULL || parm == NULL)
    {
      return -EINVAL;
    }

  ASSERT(cmng->imgsensor && cmng->imgdata);

  type_inf = get_capture_type_inf(cmng, parm->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (type_inf->state != CAPTURE_STATE_STREAMOFF)
    {
      return -EBUSY;
    }

  ret = validate_frame_setting(cmng,
                               parm->type,
                               type_inf->nr_fmt,
                               type_inf->fmt,
                               &type_inf->clip,
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

static int capture_streamon(FAR struct file *filep,
                            FAR enum v4l2_buf_type *type)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  enum capture_state_e next_capture_state;
  int ret = OK;

  if (cmng == NULL || type == NULL)
    {
      return -EINVAL;
    }

  type_inf = get_capture_type_inf(cmng, *type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      /* No procedure for VIDIOC_STREAMON(STILL_CAPTURE) */

      return OK;
    }

  nxmutex_lock(&type_inf->lock_state);

  if (type_inf->state != CAPTURE_STATE_STREAMOFF)
    {
      ret = -EPERM;
    }
  else
    {
      next_capture_state =
        estimate_next_capture_state(cmng, CAUSE_CAPTURE_START);
      change_capture_state(cmng, next_capture_state);
    }

  nxmutex_unlock(&type_inf->lock_state);
  return ret;
}

static int capture_streamoff(FAR struct file *filep,
                             FAR enum v4l2_buf_type *type)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  enum capture_state_e next_capture_state;
  irqstate_t flags;
  int ret = OK;

  if (cmng == NULL || type == NULL)
    {
      return -EINVAL;
    }

  type_inf = get_capture_type_inf(cmng, *type);
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

  if (type_inf->state == CAPTURE_STATE_STREAMOFF)
    {
      ret = -EPERM;
    }
  else
    {
      next_capture_state =
        estimate_next_capture_state(cmng, CAUSE_CAPTURE_STOP);
      change_capture_state(cmng, next_capture_state);
    }

  leave_critical_section(flags);

  return ret;
}

static int capture_do_halfpush(FAR struct file *filep, bool enable)
{
  struct v4l2_ext_controls ext_controls;
  struct v4l2_ext_control  control[2];

  /* Replace to VIDIOC_S_EXT_CTRLS format */

  control[0].id    = V4L2_CID_3A_LOCK;
  control[0].value = enable ?
                     V4L2_LOCK_EXPOSURE | V4L2_LOCK_WHITE_BALANCE : 0;
  control[1].id    = V4L2_CID_AUTO_FOCUS_START;
  control[1].value = enable ? true : false;

  ext_controls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
  ext_controls.count      = 2;
  ext_controls.controls   = control;

  /* Execute VIDIOC_S_EXT_CTRLS */

  return capture_s_ext_ctrls(filep, &ext_controls);
}

static int capture_takepict_start(FAR struct file *filep,
                                  int32_t capture_num)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  enum capture_state_e next_capture_state;
  FAR vbuf_container_t *container;
  irqstate_t flags;
  int ret = OK;

  if (cmng == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&cmng->still_inf.lock_state);

  if (cmng->still_inf.state != CAPTURE_STATE_STREAMOFF)
    {
      ret = -EPERM;
    }
  else
    {
      if (capture_num > 0)
        {
          cmng->still_inf.remaining_capnum = capture_num;
        }
      else
        {
          cmng->still_inf.remaining_capnum = REMAINING_CAPNUM_INFINITY;
        }

      /* Control video stream prior to still stream */

      flags = enter_critical_section();

      next_capture_state = estimate_next_capture_state(cmng,
                                                   CAUSE_STILL_START);
      change_capture_state(cmng, next_capture_state);

      leave_critical_section(flags);

      container =
        video_framebuff_get_vacant_container(&cmng->still_inf.bufinf);
      if (container != NULL)
        {
          /* Start still stream capture */

          start_capture(cmng,
                        V4L2_BUF_TYPE_STILL_CAPTURE,
                        cmng->still_inf.nr_fmt,
                        cmng->still_inf.fmt,
                        &cmng->still_inf.clip,
                        &cmng->still_inf.frame_interval,
                        container->buf.m.userptr,
                        container->buf.length);

          cmng->still_inf.state = CAPTURE_STATE_CAPTURE;
        }
      else
        {
          cmng->still_inf.state = CAPTURE_STATE_STREAMON;
        }
    }

  nxmutex_unlock(&cmng->still_inf.lock_state);
  return ret;
}

static int capture_takepict_stop(FAR struct file *filep,
                                 bool halfpush)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  enum capture_state_e next_capture_state;
  irqstate_t flags;
  int ret = OK;

  if (cmng == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&cmng->still_inf.lock_state);

  if (cmng->still_inf.state == CAPTURE_STATE_STREAMOFF &&
      cmng->still_inf.remaining_capnum == REMAINING_CAPNUM_INFINITY)
    {
      ret = -EPERM;
    }
  else
    {
      flags = enter_critical_section();
      if (cmng->still_inf.state == CAPTURE_STATE_CAPTURE)
        {
          stop_capture(cmng, V4L2_BUF_TYPE_STILL_CAPTURE);
        }

      leave_critical_section(flags);

      cmng->still_inf.state = CAPTURE_STATE_STREAMOFF;
      cmng->still_inf.remaining_capnum = REMAINING_CAPNUM_INFINITY;

      /* Control video stream */

      nxmutex_lock(&cmng->capture_inf.lock_state);
      next_capture_state = estimate_next_capture_state(cmng,
                                                   CAUSE_STILL_STOP);
      change_capture_state(cmng, next_capture_state);
      nxmutex_unlock(&cmng->capture_inf.lock_state);
    }

  nxmutex_unlock(&cmng->still_inf.lock_state);
  return ret;
}

static int capture_s_selection(FAR struct file *filep,
                               FAR struct v4l2_selection *clip)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  uint32_t p_u32[IMGSENSOR_CLIP_NELEM];
  imgsensor_value_t val;
  uint32_t id;
  int ret;

  if (cmng == NULL || clip == NULL)
    {
      return -EINVAL;
    }

  ASSERT(cmng->imgsensor);

  type_inf = get_capture_type_inf(cmng, clip->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (type_inf->state != CAPTURE_STATE_STREAMOFF)
    {
      return -EBUSY;
    }

  if (!validate_clip_setting(&clip->r, type_inf->fmt))
    {
      return -EINVAL;
    }

  ret = validate_frame_setting(cmng,
                               clip->type,
                               type_inf->nr_fmt,
                               type_inf->fmt,
                               &clip->r,
                               &type_inf->frame_interval);
  if (ret != OK)
    {
      return ret;
    }

  id = clip->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ?
       IMGSENSOR_ID_CLIP_VIDEO : IMGSENSOR_ID_CLIP_STILL;

  p_u32[IMGSENSOR_CLIP_INDEX_LEFT]   = clip->r.left;
  p_u32[IMGSENSOR_CLIP_INDEX_TOP]    = clip->r.top;
  p_u32[IMGSENSOR_CLIP_INDEX_WIDTH]  = clip->r.width;
  p_u32[IMGSENSOR_CLIP_INDEX_HEIGHT] = clip->r.height;

  val.p_u32 = p_u32;
  ret = IMGSENSOR_SET_VALUE(cmng->imgsensor, id, sizeof(p_u32), val);
  if (ret != OK)
    {
      return ret;
    }

  memcpy(&type_inf->clip, &clip->r, sizeof(struct v4l2_rect));
  return ret;
}

static int capture_g_selection(FAR struct file *filep,
                               FAR struct v4l2_selection *clip)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;

  if (cmng == NULL || clip == NULL)
    {
      return -EINVAL;
    }

  type_inf = get_capture_type_inf(cmng, clip->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  memcpy(&clip->r, &type_inf->clip, sizeof(struct v4l2_rect));
  return OK;
}

static int capture_queryctrl(FAR struct file *filep,
                             FAR struct v4l2_queryctrl *ctrl)
{
  struct v4l2_query_ext_ctrl ext_ctrl;
  int ret;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  /* Replace to VIDIOC_QUERY_EXT_CTRL format */

  ext_ctrl.ctrl_class = ctrl->ctrl_class;
  ext_ctrl.id         = ctrl->id;

  ret = capture_query_ext_ctrl(filep, &ext_ctrl);
  if (ret != OK)
    {
      return ret;
    }

  if (ext_ctrl.type == V4L2_CTRL_TYPE_INTEGER64 ||
      ext_ctrl.type == V4L2_CTRL_TYPE_U8 ||
      ext_ctrl.type == V4L2_CTRL_TYPE_U16 ||
      ext_ctrl.type == V4L2_CTRL_TYPE_U32)
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
  strlcpy(ctrl->name, ext_ctrl.name, sizeof(ctrl->name));

  return OK;
}

static int capture_query_ext_ctrl(FAR struct file *filep,
                                  FAR struct v4l2_query_ext_ctrl *attr)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  imgsensor_supported_value_t value;
  imgsensor_capability_range_t *range = &value.u.range;
  imgsensor_capability_discrete_t *disc = &value.u.discrete;
  imgsensor_capability_elems_t *elem = &value.u.elems;
  int ret;

  if (cmng == NULL || attr == NULL)
    {
      return -EINVAL;
    }

  ASSERT(cmng->imgsensor);

  attr->flags      = 0;
  attr->elem_size  = 0;
  attr->elems      = 1;
  attr->nr_of_dims = 0;
  memset(attr->dims, 0, sizeof(attr->dims));

  if (attr->id == V4L2_CID_SCENE_MODE)
    {
      /* Scene mode is processed in only video driver. */

      attr->type          = V4L2_CTRL_TYPE_INTEGER_MENU;
      attr->minimum       = 0;
      attr->maximum       = cmng->capture_scence_num - 1;
      attr->step          = 1;
      attr->default_value = 0;
      attr->flags         = 0;
      strlcpy(attr->name, "Scene Mode", 32);
    }
  else
    {
      ret = IMGSENSOR_GET_SUPPORTED_VALUE(cmng->imgsensor, attr->id, &value);
      if (ret < 0)
        {
          return ret;
        }

      attr->type = value.type;
      attr->flags = 0;

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

      set_parameter_name(attr->id, attr->name);
    }

  return OK;
}

static int capture_querymenu(FAR struct file *filep,
                             FAR struct v4l2_querymenu *menu)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  imgsensor_supported_value_t value;
  int ret;

  if (cmng == NULL || menu == NULL)
    {
      return -EINVAL;
    }

  ASSERT(cmng->imgsensor);

  if (menu->id == V4L2_CID_SCENE_MODE)
    {
      /* Scene mode is processed in only video driver. */

      if (menu->index > cmng->capture_scence_num - 1)
        {
          return -EINVAL;
        }

      menu->value = cmng->capture_scene_param[menu->index]->mode;
    }
  else
    {
      ret = IMGSENSOR_GET_SUPPORTED_VALUE(cmng->imgsensor,
                                          menu->id,
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

static int capture_g_ctrl(FAR struct file *filep,
                          FAR struct v4l2_control *ctrl)
{
  struct v4l2_ext_controls ext_controls;
  struct v4l2_ext_control control;
  int ret;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  memset(&ext_controls, 0, sizeof(struct v4l2_ext_controls));
  memset(&control, 0, sizeof(struct v4l2_ext_control));

  /* Replace to VIDIOC_G_EXT_CTRLS format */

  control.id = ctrl->id;

  ext_controls.ctrl_class = V4L2_CTRL_CLASS_USER;
  ext_controls.count      = 1;
  ext_controls.controls   = &control;

  /* Execute VIDIOC_G_EXT_CTRLS */

  ret = capture_g_ext_ctrls(filep, &ext_controls);
  if (ret == OK)
    {
      /* Replace gotten value to VIDIOC_G_CTRL parameter */

      ctrl->value = control.value;
    }

  return ret;
}

static int capture_s_ctrl(FAR struct file *filep,
                          FAR struct v4l2_control *ctrl)
{
  struct v4l2_ext_controls ext_controls;
  struct v4l2_ext_control control;

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

  return capture_s_ext_ctrls(filep, &ext_controls);
}

static int capture_g_ext_ctrls(FAR struct file *filep,
                               FAR struct v4l2_ext_controls *ctrls)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR struct v4l2_ext_control *control;
  int ret = OK;
  int cnt;

  if (cmng == NULL || ctrls == NULL)
    {
      return -EINVAL;
    }

  ASSERT(cmng->imgsensor);

  for (cnt = 0, control = ctrls->controls;
       cnt < ctrls->count;
       cnt++, control++)
    {
      if (control->id == V4L2_CID_SCENE_MODE)
        {
          control->value = cmng->capture_scene_mode;
        }
      else
        {
          ret = IMGSENSOR_GET_VALUE(cmng->imgsensor,
                  control->id,
                  control->size,
                  (imgsensor_value_t *)&control->value64);
          if (ret < 0)
            {
              /* Set cnt in that error occurred */

              ctrls->error_idx = cnt;
              return ret;
            }
        }
    }

  return ret;
}

static int capture_s_ext_ctrls(FAR struct file *filep,
                               FAR struct v4l2_ext_controls *ctrls)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR struct v4l2_ext_control *control;
  int ret = OK;
  int cnt;

  if (cmng == NULL || ctrls == NULL)
    {
      return -EINVAL;
    }

  ASSERT(cmng->imgsensor);

  for (cnt = 0, control = ctrls->controls;
       cnt < ctrls->count;
       cnt++, control++)
    {
      if (control->id == V4L2_CID_SCENE_MODE)
        {
          ret = reflect_scene_parameter(cmng, control->value);
        }
      else
        {
          ret = IMGSENSOR_SET_VALUE(cmng->imgsensor,
                  control->id,
                  control->size,
                  (imgsensor_value_t)control->value64);
          if (ret == 0)
            {
              if (cmng->capture_scene_mode == V4L2_SCENE_MODE_NONE)
                {
                  save_scene_param(cmng, V4L2_SCENE_MODE_NONE,
                    control->id,
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

static int capture_query_ext_ctrl_scene(FAR struct file *filep,
             FAR struct v4s_query_ext_ctrl_scene *attr)
{
  if (attr == NULL)
    {
      return -EINVAL;
    }

  return capture_query_ext_ctrl(filep, &attr->control);
}

static int capture_querymenu_scene(FAR struct file *filep,
             FAR struct v4s_querymenu_scene *menu)
{
  if (menu == NULL)
    {
      return -EINVAL;
    }

  return capture_querymenu(filep, &menu->menu);
}

static int capture_s_ext_ctrls_scene(FAR struct file *filep,
             FAR struct v4s_ext_controls_scene *ctrls)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR struct v4l2_ext_control *control;
  int ret = OK;
  int cnt;

  if (cmng == NULL || ctrls == NULL)
    {
      return -EINVAL;
    }

  for (cnt = 0, control = ctrls->control.controls;
       cnt < ctrls->control.count;
       cnt++, control++)
    {
      ret = save_scene_param(cmng, ctrls->mode, control->id, control);
      if (ret != OK)
        {
          ctrls->control.error_idx = cnt;
          return ret;
        }
    }

  return ret;
}

static int capture_g_ext_ctrls_scene(FAR struct file *filep,
             FAR struct v4s_ext_controls_scene *ctrls)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR struct v4l2_ext_control *control;
  int ret = OK;
  int cnt;

  if (cmng == NULL || ctrls == NULL)
    {
      return -EINVAL;
    }

  for (cnt = 0, control = ctrls->control.controls;
       cnt < ctrls->control.count;
       cnt++, control++)
    {
      ret = read_scene_param(cmng, ctrls->mode,
               control->id,
               control);
      if (ret != OK)
        {
          ctrls->control.error_idx = cnt;
          return ret;
        }
    }

  return ret;
}

static int capture_enum_fmt(FAR struct file *filep,
                            FAR struct v4l2_fmtdesc *f)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;

  if (cmng == NULL || f == NULL)
    {
      return -EINVAL;
    }

  if (cmng->imgsensor && cmng->imgsensor->fmtdescs)
    {
      if (f->index > cmng->imgsensor->fmtdescs_num)
        {
          return -EINVAL;
        }
      else
        {
          f->pixelformat = cmng->imgsensor->fmtdescs[f->index].pixelformat;
          strlcpy(f->description,
                  cmng->imgsensor->fmtdescs[f->index].description,
                  sizeof(f->description));
        }
    }
  else
    {
      if (f->index > 0)
        {
          return -EINVAL;
        }

      f->pixelformat = V4L2_PIX_FMT_UYVY;
    }

  return 0;
}

static int capture_enum_frmsize(FAR struct file *filep,
                                FAR struct v4l2_frmsizeenum *f)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;

  if (cmng == NULL || f == NULL)
    {
      return -EINVAL;
    }

  if (cmng->imgsensor && cmng->imgsensor->frmsizes)
    {
      if (f->index > cmng->imgsensor->frmsizes_num)
        {
          return -EINVAL;
        }
      else
        {
          f->type = cmng->imgsensor->frmsizes[f->index].type;
          if (f->type == V4L2_FRMSIZE_TYPE_DISCRETE)
            {
              f->discrete = cmng->imgsensor->frmsizes[f->index].discrete;
            }
          else
            {
              f->stepwise = cmng->imgsensor->frmsizes[f->index].stepwise;
            }
        }
    }
  else
    {
      if (f->index > 0)
        {
          return -EINVAL;
        }

      f->type = V4L2_FRMIVAL_TYPE_DISCRETE;
      f->discrete.width = VIDEO_HSIZE_QVGA;
      f->discrete.height = VIDEO_VSIZE_QVGA;
    }

  return 0;
}

static int capture_enum_frminterval(FAR struct file *filep,
                                    FAR struct v4l2_frmivalenum *f)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;

  if (cmng == NULL || f == NULL)
    {
      return -EINVAL;
    }

  if (cmng->imgsensor && cmng->imgsensor->frmintervals)
    {
      if (f->index > cmng->imgsensor->frmintervals_num)
        {
          return -EINVAL;
        }
      else
        {
          f->type = cmng->imgsensor->frmintervals[f->index].type;
          if (f->type == V4L2_FRMIVAL_TYPE_DISCRETE)
            {
              f->discrete = cmng->imgsensor->frmintervals[f->index].discrete;
            }
          else
            {
              f->stepwise = cmng->imgsensor->frmintervals[f->index].stepwise;
            }
        }
    }
  else
    {
      if (f->index > 0)
        {
          return -EINVAL;
        }

      f->type = V4L2_FRMIVAL_TYPE_DISCRETE;
      f->discrete.denominator = 15;
      f->discrete.numerator = 1;
    }

  return 0;
}

/****************************************************************************
 * File Opterations Functions
 ****************************************************************************/

static int capture_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  int ret = OK;

  if (cmng == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&cmng->lock_open_num);
  if (cmng->open_num == 0)
    {
      /* Only in first execution, open device */

      ret = IMGSENSOR_INIT(cmng->imgsensor);
      if (ret == OK)
        {
          ret = IMGDATA_INIT(cmng->imgdata);
          if (ret == OK)
            {
              initialize_resources(cmng);
            }
        }
      else
        {
          ret = -ENODEV;
        }
    }

  /* In second or later execution, ret is initial value(=OK) */

  if (ret == OK)
    {
      cmng->open_num++;
    }

  nxmutex_unlock(&cmng->lock_open_num);
  return ret;
}

static int capture_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;

  if (cmng == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&cmng->lock_open_num);

  if (--cmng->open_num == 0)
    {
      cleanup_resources(cmng);
      IMGSENSOR_UNINIT(cmng->imgsensor);
      IMGDATA_UNINIT(cmng->imgdata);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
      if (cmng->unlinked)
        {
          nxmutex_unlock(&cmng->lock_open_num);
          nxmutex_destroy(&cmng->lock_open_num);
          kmm_free(cmng);
          inode->i_private = NULL;
          return OK;
        }

#endif
    }

  nxmutex_unlock(&cmng->lock_open_num);
  return OK;
}

static int capture_mmap(FAR struct file *filep,
                        FAR struct mm_map_entry_s *map)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  size_t heapsize;
  int ret = -EINVAL;

  if (cmng == NULL)
    {
      return -EINVAL;
    }

  type_inf = &cmng->capture_inf;
  heapsize = get_heapsize(type_inf);

  if (map->offset >= 0 && map->offset < heapsize &&
      map->length && map->offset + map->length <= heapsize)
    {
      map->vaddr = type_inf->bufheap + map->offset;
      ret = OK;
    }

  return ret;
}

static int capture_poll(FAR struct file *filep,
                        FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR capture_mng_t *cmng = inode->i_private;
  FAR capture_type_inf_t *type_inf;
  enum v4l2_buf_type buf_type;
  irqstate_t flags;

  if (cmng == NULL)
    {
      return -EINVAL;
    }

  buf_type = cmng->still_inf.state == CAPTURE_STATE_CAPTURE ?
               V4L2_BUF_TYPE_STILL_CAPTURE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

  type_inf = get_capture_type_inf(cmng, buf_type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  if (setup)
    {
      if (type_inf->fds == NULL)
        {
          type_inf->fds = fds;
          fds->priv     = &type_inf->fds;
          if (!video_framebuff_is_empty(&type_inf->bufinf))
            {
              poll_notify(&fds, 1, POLLIN);
            }
        }
      else
        {
          leave_critical_section(flags);
          return -EBUSY;
        }
    }
  else if (fds->priv)
    {
      type_inf->fds = NULL;
      fds->priv     = NULL;
    }

  leave_critical_section(flags);

  return OK;
}

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int capture_unlink(FAR struct inode *inode)
{
  FAR capture_mng_t *cmng = inode->i_private;

  if (cmng == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&cmng->lock_open_num);
  if (cmng->open_num == 0)
    {
      nxmutex_unlock(&cmng->lock_open_num);
      nxmutex_destroy(&cmng->lock_open_num);
      kmm_free(cmng);
      inode->i_private = NULL;
    }
  else
    {
      cmng->unlinked = true;
      nxmutex_unlock(&cmng->lock_open_num);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int capture_initialize(FAR const char *devpath)
{
  return capture_register(devpath,
                          g_capture_data,
                          g_capture_registered_sensor,
                          g_capture_registered_sensor_num);
}

int capture_uninitialize(FAR const char *devpath)
{
  return capture_unregister(devpath);
}

int capture_register(FAR const char *devpath,
                     FAR struct imgdata_s *data,
                     FAR struct imgsensor_s **sensors,
                     size_t sensor_num)
{
  FAR capture_mng_t *cmng;
  int ret;

  /* Input devpath Error Check */

  if (devpath == NULL || data == NULL)
    {
      return -EINVAL;
    }

  /* Initialize capture device structure */

  cmng = kmm_zalloc(sizeof(capture_mng_t));
  if (cmng == NULL)
    {
      verr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  cmng->v4l2.vops = &g_capture_vops;
  cmng->v4l2.fops = &g_capture_fops;

  cmng->imgdata   = data;
  cmng->imgsensor = get_connected_imgsensor(sensors, sensor_num);
  if (cmng->imgsensor == NULL)
    {
      kmm_free(cmng);
      return -EINVAL;
    }

  /* Initialize mutex */

  nxmutex_init(&cmng->lock_open_num);

  /* Register the character driver */

  ret = video_register(devpath, (FAR struct v4l2_s *)cmng);
  if (ret < 0)
    {
      verr("Failed to register driver: %d\n", ret);
      nxmutex_destroy(&cmng->lock_open_num);
      kmm_free(cmng);
      return ret;
    }

  return OK;
}

int capture_unregister(FAR const char *devpath)
{
  return unregister_driver(devpath);
}

int imgsensor_register(FAR struct imgsensor_s *sensor)
{
  FAR struct imgsensor_s **new_addr;
  int ret = -ENOMEM;

  new_addr = kmm_realloc(g_capture_registered_sensor, sizeof(sensor) *
                         (g_capture_registered_sensor_num + 1));
  if (new_addr != NULL)
    {
      new_addr[g_capture_registered_sensor_num++] = sensor;
      g_capture_registered_sensor = new_addr;
      ret = OK;
    }

  return ret;
}

void imgdata_register(FAR struct imgdata_s *data)
{
  g_capture_data = data;
}
