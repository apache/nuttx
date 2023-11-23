/****************************************************************************
 * include/nuttx/video/imgsensor.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_IMGSENSOR_H
#define __INCLUDE_NUTTX_VIDEO_IMGSENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Camera parameter IDs */

#define IMGSENSOR_ID_BRIGHTNESS           V4L2_CID_BRIGHTNESS
#define IMGSENSOR_ID_CONTRAST             V4L2_CID_CONTRAST
#define IMGSENSOR_ID_SATURATION           V4L2_CID_SATURATION
#define IMGSENSOR_ID_HUE                  V4L2_CID_HUE
#define IMGSENSOR_ID_AUTO_WHITE_BALANCE   V4L2_CID_AUTO_WHITE_BALANCE
#define IMGSENSOR_ID_RED_BALANCE          V4L2_CID_RED_BALANCE
#define IMGSENSOR_ID_BLUE_BALANCE         V4L2_CID_BLUE_BALANCE
#define IMGSENSOR_ID_GAMMA                V4L2_CID_GAMMA
#define IMGSENSOR_ID_GAMMA_CURVE          V4L2_CID_GAMMA_CURVE
#define IMGSENSOR_ID_EXPOSURE             V4L2_CID_EXPOSURE
#define IMGSENSOR_ID_HFLIP_VIDEO          V4L2_CID_HFLIP
#define IMGSENSOR_ID_VFLIP_VIDEO          V4L2_CID_VFLIP
#define IMGSENSOR_ID_HFLIP_STILL          V4L2_CID_HFLIP_STILL
#define IMGSENSOR_ID_VFLIP_STILL          V4L2_CID_VFLIP_STILL
#define IMGSENSOR_ID_SHARPNESS            V4L2_CID_SHARPNESS
#define IMGSENSOR_ID_COLOR_KILLER         V4L2_CID_COLOR_KILLER
#define IMGSENSOR_ID_COLORFX              V4L2_CID_COLORFX
#define IMGSENSOR_ID_AUTOBRIGHTNESS       V4L2_CID_AUTOBRIGHTNESS
#define IMGSENSOR_ID_ROTATE               V4L2_CID_ROTATE
#define IMGSENSOR_ID_EXPOSURE_AUTO        V4L2_CID_EXPOSURE_AUTO
#define IMGSENSOR_ID_EXPOSURE_ABSOLUTE    V4L2_CID_EXPOSURE_ABSOLUTE
#define IMGSENSOR_ID_FOCUS_ABSOLUTE       V4L2_CID_FOCUS_ABSOLUTE
#define IMGSENSOR_ID_FOCUS_RELATIVE       V4L2_CID_FOCUS_RELATIVE
#define IMGSENSOR_ID_FOCUS_AUTO           V4L2_CID_FOCUS_AUTO
#define IMGSENSOR_ID_ZOOM_ABSOLUTE        V4L2_CID_ZOOM_ABSOLUTE
#define IMGSENSOR_ID_ZOOM_RELATIVE        V4L2_CID_ZOOM_RELATIVE
#define IMGSENSOR_ID_ZOOM_CONTINUOUS      V4L2_CID_ZOOM_CONTINUOUS
#define IMGSENSOR_ID_IRIS_ABSOLUTE        V4L2_CID_IRIS_ABSOLUTE
#define IMGSENSOR_ID_IRIS_RELATIVE        V4L2_CID_IRIS_RELATIVE
#define IMGSENSOR_ID_AUTO_N_PRESET_WB     V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE
#define IMGSENSOR_ID_WIDE_DYNAMIC_RANGE   V4L2_CID_WIDE_DYNAMIC_RANGE
#define IMGSENSOR_ID_IMG_STABILIZATION    V4L2_CID_IMAGE_STABILIZATION
#define IMGSENSOR_ID_ISO_SENSITIVITY      V4L2_CID_ISO_SENSITIVITY
#define IMGSENSOR_ID_ISO_SENSITIVITY_AUTO V4L2_CID_ISO_SENSITIVITY_AUTO
#define IMGSENSOR_ID_EXPOSURE_METERING    V4L2_CID_EXPOSURE_METERING
#define IMGSENSOR_ID_SPOT_POSITION        V4L2_CID_EXPOSURE_METERING_SPOT_POSITION
#define IMGSENSOR_ID_3A_LOCK              V4L2_CID_3A_LOCK
#define IMGSENSOR_ID_AUTO_FOCUS_START     V4L2_CID_AUTO_FOCUS_START
#define IMGSENSOR_ID_AUTO_FOCUS_STOP      V4L2_CID_AUTO_FOCUS_STOP
#define IMGSENSOR_ID_3A_PARAMETER         V4L2_CID_3A_PARAMETER
#define IMGSENSOR_ID_3A_STATUS            V4L2_CID_3A_STATUS
#define IMGSENSOR_ID_FLASH_LED_MODE       V4L2_CID_FLASH_LED_MODE
#define IMGSENSOR_ID_JPEG_QUALITY         V4L2_CID_JPEG_COMPRESSION_QUALITY
#define IMGSENSOR_ID_CLIP_VIDEO           (0xFFFF0000)
#define IMGSENSOR_ID_CLIP_STILL           (0xFFFF0001)

/* Number of elements in clip data array
 * in IMGSENSOR_ID_CLIP_VIDEO and IMGSENSOR_ID_CLIP_STILL case
 */

#define IMGSENSOR_CLIP_NELEM              (4)

/* Index of clip information
 * in IMGSENSOR_ID_CLIP_VIDEO and IMGSENSOR_ID_CLIP_STILL case
 */

#define IMGSENSOR_CLIP_INDEX_LEFT         (0)
#define IMGSENSOR_CLIP_INDEX_TOP          (1)
#define IMGSENSOR_CLIP_INDEX_WIDTH        (2)
#define IMGSENSOR_CLIP_INDEX_HEIGHT       (3)

/* Bit definition for IMGSENSOR_ID_3A_LOCK */

#define IMGSENSOR_LOCK_EXPOSURE      (1 << 0)
#define IMGSENSOR_LOCK_WHITE_BALANCE (1 << 1)
#define IMGSENSOR_LOCK_FOCUS         (1 << 2)

/* Status bit definition for IMGSENSOR_ID_3A_STATUS */

#define IMGSENSOR_3A_STATUS_STABLE        (0)
#define IMGSENSOR_3A_STATUS_AE_OPERATING  (1 << 0)
#define IMGSENSOR_3A_STATUS_AWB_OPERATING (1 << 1)
#define IMGSENSOR_3A_STATUS_AF_OPERATING  (1 << 2)

/* Format definition for start_capture() and validate_frame_setting */

#define IMGSENSOR_FMT_MAX                  (2)
#define IMGSENSOR_FMT_MAIN                 (0)
#define IMGSENSOR_FMT_SUB                  (1)
#define IMGSENSOR_PIX_FMT_UYVY             (0)
#define IMGSENSOR_PIX_FMT_RGB565           (1)
#define IMGSENSOR_PIX_FMT_JPEG             (2)
#define IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG (3)
#define IMGSENSOR_PIX_FMT_SUBIMG_UYVY      (4)
#define IMGSENSOR_PIX_FMT_SUBIMG_RGB565    (5)
#define IMGSENSOR_PIX_FMT_YUYV             (6)
#define IMGSENSOR_PIX_FMT_YUV420P          (7)
#define IMGSENSOR_PIX_FMT_NV12             (8)

/* Method access helper macros */

#define IMGSENSOR_IS_AVAILABLE(s) \
  ((s)->ops->is_available ? (s)->ops->is_available(s) : false)
#define IMGSENSOR_INIT(s) \
  ((s)->ops->init ? (s)->ops->init(s) : -ENOTTY)
#define IMGSENSOR_UNINIT(s) \
  ((s)->ops->uninit ? (s)->ops->uninit(s) : -ENOTTY)
#define IMGSENSOR_GET_DRIVER_NAME(s) \
  ((s)->ops->get_driver_name ? (s)->ops->get_driver_name(s) : NULL)
#define IMGSENSOR_VALIDATE_FRAME_SETTING(s, t, n, f, i) \
  ((s)->ops->validate_frame_setting ? \
   (s)->ops->validate_frame_setting(s, t, n, f, i) : -ENOTTY)
#define IMGSENSOR_START_CAPTURE(s, t, n, f, i) \
  ((s)->ops->start_capture ? \
   (s)->ops->start_capture(s, t, n, f, i) : -ENOTTY)
#define IMGSENSOR_STOP_CAPTURE(s, t) \
  ((s)->ops->stop_capture ? (s)->ops->stop_capture(s, t) : -ENOTTY)
#define IMGSENSOR_GET_FRAME_INTERVAL(s, t, i) \
  ((s)->ops->get_frame_interval ? \
   (s)->ops->get_frame_interval(s, t, i) : -ENOTTY)
#define IMGSENSOR_GET_SUPPORTED_VALUE(s, i, v) \
  ((s)->ops->get_supported_value ? \
   (s)->ops->get_supported_value(s, i, v) : -ENOTTY)
#define IMGSENSOR_GET_VALUE(s, i, l, v) \
  ((s)->ops->get_value ? (s)->ops->get_value(s, i, l, v) : -ENOTTY)
#define IMGSENSOR_SET_VALUE(s, i, l, v) \
  ((s)->ops->set_value ? (s)->ops->set_value(s, i, l, v) : -ENOTTY)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Enumeration for VIDEO_ID_COLORFX */

typedef enum imgsensor_colorfx_e
{
  IMGSENSOR_COLORFX_NONE         = 0,
  IMGSENSOR_COLORFX_BW           = 1,
  IMGSENSOR_COLORFX_SEPIA        = 2,
  IMGSENSOR_COLORFX_NEGATIVE     = 3,
  IMGSENSOR_COLORFX_EMBOSS       = 4,
  IMGSENSOR_COLORFX_SKETCH       = 5,
  IMGSENSOR_COLORFX_SKY_BLUE     = 6,
  IMGSENSOR_COLORFX_GRASS_GREEN  = 7,
  IMGSENSOR_COLORFX_SKIN_WHITEN  = 8,
  IMGSENSOR_COLORFX_VIVID        = 9,
  IMGSENSOR_COLORFX_AQUA         = 10,
  IMGSENSOR_COLORFX_ART_FREEZE   = 11,
  IMGSENSOR_COLORFX_SILHOUETTE   = 12,
  IMGSENSOR_COLORFX_SOLARIZATION = 13,
  IMGSENSOR_COLORFX_ANTIQUE      = 14,
  IMGSENSOR_COLORFX_SET_CBCR     = 15,
  IMGSENSOR_COLORFX_PASTEL       = 16,
} imgsensor_colorfx_t;

/* Enumeration for IMGSENSOR_ID_EXPOSURE_AUTO */

typedef enum imgsensor_exposure_auto_type_e
{
  /* Exposure time:auto,   iris aperture:auto */

  IMGSENSOR_EXPOSURE_AUTO = 0,

  /* Exposure time:manual, iris aperture:manual */

  IMGSENSOR_EXPOSURE_MANUAL = 1,

  /* Exposure time:manual, iris aperture:auto */

  IMGSENSOR_EXPOSURE_SHUTTER_PRIORITY = 2,

  /* Exposure time:auto,   iris aperture:manual */

  IMGSENSOR_EXPOSURE_APERTURE_PRIORITY = 3
} imgsensor_exposure_auto_type_t;

/* Enumeration for IMGSENSOR_ID_AUTO_N_PRESET_WHITE_BALANCE */

typedef enum imgsensor_white_balance_e
{
  IMGSENSOR_WHITE_BALANCE_MANUAL        = 0,
  IMGSENSOR_WHITE_BALANCE_AUTO          = 1,
  IMGSENSOR_WHITE_BALANCE_INCANDESCENT  = 2,
  IMGSENSOR_WHITE_BALANCE_FLUORESCENT   = 3,
  IMGSENSOR_WHITE_BALANCE_FLUORESCENT_H = 4,
  IMGSENSOR_WHITE_BALANCE_HORIZON       = 5,
  IMGSENSOR_WHITE_BALANCE_DAYLIGHT      = 6,
  IMGSENSOR_WHITE_BALANCE_FLASH         = 7,
  IMGSENSOR_WHITE_BALANCE_CLOUDY        = 8,
  IMGSENSOR_WHITE_BALANCE_SHADE         = 9,
} imgsensor_white_balance_t;

/* Enumeration for IMGSENSOR_ID_ISO_SENSITIVITY_AUTO */

typedef enum imgsensor_iso_sensitivity_auto_type_e
{
  IMGSENSOR_ISO_SENSITIVITY_MANUAL = 0,
  IMGSENSOR_ISO_SENSITIVITY_AUTO   = 1,
} imgsensor_iso_sensitivity_auto_type_t;

/* Enumeration for IMGSENSOR_ID_EXPOSURE_METERING */

typedef enum imgsensor_exposure_metering_e
{
  IMGSENSOR_EXPOSURE_METERING_AVERAGE         = 0,
  IMGSENSOR_EXPOSURE_METERING_CENTER_WEIGHTED = 1,
  IMGSENSOR_EXPOSURE_METERING_SPOT            = 2,
  IMGSENSOR_EXPOSURE_METERING_MATRIX          = 3,
} imgsensor_exposure_metering_t;

/* Enumeration for IMGSENSOR_ID_FLASH_LED_MODE */

typedef enum imgsensor_flash_led_mode_e
{
  IMGSENSOR_FLASH_LED_MODE_NONE  = 0,
  IMGSENSOR_FLASH_LED_MODE_FLASH = 1,
  IMGSENSOR_FLASH_LED_MODE_TORCH = 2,
} imgsensor_flash_led_mode_t;

/* Enumeration for get_supported_value() */

typedef enum imgsensor_ctrl_type_e
{
  IMGSENSOR_CTRL_TYPE_INTEGER          = 1,
  IMGSENSOR_CTRL_TYPE_BOOLEAN          = 2,
  IMGSENSOR_CTRL_TYPE_INTEGER64        = 5,
  IMGSENSOR_CTRL_TYPE_BITMASK          = 8,
  IMGSENSOR_CTRL_TYPE_INTEGER_MENU     = 9,
  IMGSENSOR_CTRL_TYPE_U8FIXEDPOINT_Q7  = 10,
  IMGSENSOR_CTRL_TYPE_U16FIXEDPOINT_Q8 = 11,
  IMGSENSOR_CTRL_TYPE_INTEGER_TIMES_3  = 12,
  IMGSENSOR_CTRL_TYPE_U8               = 0x0100,
  IMGSENSOR_CTRL_TYPE_U16              = 0x0101,
  IMGSENSOR_CTRL_TYPE_U32              = 0x0102,
} imgsensor_ctrl_type_t;

/* Enumeration for stream */

typedef enum imgsensor_stream_type_e
{
  IMGSENSOR_STREAM_TYPE_VIDEO = 0,
  IMGSENSOR_STREAM_TYPE_STILL = 1,
} imgsensor_stream_type_t;

/* Structure for validate_frame_setting() and start_capture() */

typedef struct imgsensor_format_s
{
  uint16_t width;
  uint16_t height;
  uint32_t pixelformat;
} imgsensor_format_t;

typedef struct imgsensor_interval_s
{
  uint32_t numerator;
  uint32_t denominator;
} imgsensor_interval_t;

/* Structure for get_supported_value() */

typedef struct imgsensor_capability_range_s
{
  int64_t  minimum;
  int64_t  maximum;
  uint64_t step;
  int64_t  default_value;
} imgsensor_capability_range_t;

typedef struct imgsensor_capability_discrete_s
{
  int8_t  nr_values;
  FAR const int32_t *values;
  int32_t default_value;
} imgsensor_capability_discrete_t;

typedef struct imgsensor_capability_elems_s
{
  uint32_t nr_elems;
  int64_t  minimum;
  int64_t  maximum;
  uint64_t step;
} imgsensor_capability_elems_t;

typedef struct imgsensor_supported_value_s
{
  imgsensor_ctrl_type_t type;   /* Control type */
  union
    {
      /* Use 'range' member in the following types cases.
       *   IMGSENSOR_CTRL_TYPE_INTEGER
       *   IMGSENSOR_CTRL_TYPE_BOOLEAN
       *   IMGSENSOR_CTRL_TYPE_INTEGER64
       *   IMGSENSOR_CTRL_TYPE_BITMASK
       *   IMGSENSOR_CTRL_TYPE_U8FIXEDPOINT_Q7
       *   IMGSENSOR_CTRL_TYPE_U16FIXEDPOINT_Q8
       *   IMGSENSOR_CTRL_TYPE_INTEGER_TIMES_3
       */

      imgsensor_capability_range_t    range;

      /* Use 'discrete' member in the following type case.
       *   IMGSENSOR_CTRL_TYPE_INTEGER_MENU
       */

      imgsensor_capability_discrete_t discrete;

      /* Use 'elems' member in the following types cases.
       *   IMGSENSOR_CTRL_TYPE_U8
       *   IMGSENSOR_CTRL_TYPE_U16
       *   IMGSENSOR_CTRL_TYPE_U32
       */

      imgsensor_capability_elems_t    elems;
    } u;
} imgsensor_supported_value_t;

typedef union imgsensor_value_u
{
  int32_t  value32;
  int64_t  value64;
  uint8_t  *p_u8;
  uint16_t *p_u16;
  uint32_t *p_u32;
} imgsensor_value_t;

/* Structure for Image Sensor I/F */

struct imgsensor_s;
struct imgsensor_ops_s
{
  CODE bool (*is_available)(FAR struct imgsensor_s *sensor);
  CODE int  (*init)(FAR struct imgsensor_s *sensor);
  CODE int  (*uninit)(FAR struct imgsensor_s *sensor);
  CODE const char * (*get_driver_name)(FAR struct imgsensor_s *sensor);
  CODE int  (*validate_frame_setting)(FAR struct imgsensor_s *sensor,
                                      imgsensor_stream_type_t type,
                                      uint8_t nr_datafmts,
                                      FAR imgsensor_format_t *datafmts,
                                      FAR imgsensor_interval_t *interval);
  CODE int  (*start_capture)(FAR struct imgsensor_s *sensor,
                             imgsensor_stream_type_t type,
                             uint8_t nr_datafmts,
                             FAR imgsensor_format_t *datafmts,
                             FAR imgsensor_interval_t *interval);
  CODE int  (*stop_capture)(FAR struct imgsensor_s *sensor,
                            imgsensor_stream_type_t type);
  CODE int  (*get_frame_interval)(FAR struct imgsensor_s *sensor,
                                  imgsensor_stream_type_t type,
                                  FAR imgsensor_interval_t *interval);
  CODE int  (*get_supported_value)(FAR struct imgsensor_s *sensor,
                                   uint32_t id,
                                   FAR imgsensor_supported_value_t *value);
  CODE int  (*get_value)(FAR struct imgsensor_s *sensor,
                         uint32_t id, uint32_t size,
                         FAR imgsensor_value_t *value);
  CODE int  (*set_value)(FAR struct imgsensor_s *sensor,
                         uint32_t id, uint32_t size,
                         imgsensor_value_t value);
};

/* Image sensor private data.  This structure only defines the initial fields
 * of the structure visible to the client.  The specific implementation may
 * add additional, device specific fields after the vtable.
 */

struct imgsensor_s
{
  FAR const struct imgsensor_ops_s *ops;
  size_t fmtdescs_num;
  FAR const struct v4l2_fmtdesc *fmtdescs;
  size_t frmsizes_num;
  FAR const struct v4l2_frmsizeenum *frmsizes;
  size_t frmintervals_num;
  FAR const struct v4l2_frmivalenum *frmintervals;
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Register image sensor operations. */

int imgsensor_register(FAR struct imgsensor_s *sensor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_HALIF_H */
