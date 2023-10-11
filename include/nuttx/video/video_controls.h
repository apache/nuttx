/****************************************************************************
 * include/nuttx/video/video_controls.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_VIDEO_CONTROLS_H
#define __INCLUDE_NUTTX_VIDEO_VIDEO_CONTROLS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Control classes */

#define V4L2_CTRL_CLASS_USER      (0x0100) /* Old-style 'user' controls */
#define V4L2_CTRL_CLASS_CAMERA    (0x0200) /* Camera class controls */
#define V4L2_CTRL_CLASS_FLASH     (0x0300) /* Camera flash controls */
#define V4L2_CTRL_CLASS_JPEG      (0x0400) /* JPEG-compression controls */

#define USER_CID(v)   (V4L2_CTRL_CLASS_USER + (v))
#define CAMERA_CID(v) (V4L2_CTRL_CLASS_CAMERA + (v))
#define FLASH_CID(v)  (V4L2_CTRL_CLASS_FLASH + (v))
#define JPEG_CID(v)   (V4L2_CTRL_CLASS_JPEG + (v))

/* User-class control IDs */

#define V4L2_CID_BRIGHTNESS           USER_CID(0)    /* Brightness */
#define V4L2_CID_CONTRAST             USER_CID(1)    /* Contrast   */
#define V4L2_CID_SATURATION           USER_CID(2)    /* Saturation */
#define V4L2_CID_HUE                  USER_CID(3)    /* Hue        */
#define V4L2_CID_AUTO_WHITE_BALANCE   USER_CID(4)    /* AWB        */
#define V4L2_CID_RED_BALANCE          USER_CID(5)    /* Red balance */
#define V4L2_CID_BLUE_BALANCE         USER_CID(6)    /* Blue balance */
#define V4L2_CID_GAMMA                USER_CID(7)    /* Gamma value adjustment */
#define V4L2_CID_GAMMA_CURVE          USER_CID(8)    /* Gamma curve adjustment */
#define V4L2_CID_EXPOSURE             USER_CID(9)    /* Exposure value */

/* Mirror horizontally(VIDEO) */

#define V4L2_CID_HFLIP                USER_CID(10)

/* Mirror vertically(VIDEO) */

#define V4L2_CID_VFLIP                USER_CID(11)

/* Mirror horizontally(STILL) */

#define V4L2_CID_HFLIP_STILL          USER_CID(12)

/* Mirror vertically(STILL) */

#define V4L2_CID_VFLIP_STILL          USER_CID(13)
#define V4L2_CID_SHARPNESS            USER_CID(14)   /* Sharpness */
#define V4L2_CID_COLOR_KILLER         USER_CID(15)   /* Color killer */
#define V4L2_CID_COLORFX              USER_CID(16)   /* Color effect */

/* Enumeration for V4L2_CID_COLORFX */

enum v4l2_colorfx
{
  V4L2_COLORFX_NONE                = 0,    /* No effect */
  V4L2_COLORFX_BW                  = 1,    /* Black/white */
  V4L2_COLORFX_SEPIA               = 2,    /* Sepia */
  V4L2_COLORFX_NEGATIVE            = 3,    /* Positive/negative inversion */
  V4L2_COLORFX_EMBOSS              = 4,    /* Emboss */
  V4L2_COLORFX_SKETCH              = 5,    /* Sketch */
  V4L2_COLORFX_SKY_BLUE            = 6,    /* Sky blue */
  V4L2_COLORFX_GRASS_GREEN         = 7,    /* Grass green */
  V4L2_COLORFX_SKIN_WHITEN         = 8,    /* Skin whiten */
  V4L2_COLORFX_VIVID               = 9,    /* Vivid */
  V4L2_COLORFX_AQUA                = 10,   /* Aqua */
  V4L2_COLORFX_ART_FREEZE          = 11,   /* Art freeze */
  V4L2_COLORFX_SILHOUETTE          = 12,   /* Silhouette */
  V4L2_COLORFX_SOLARIZATION        = 13,   /* Solarization */
  V4L2_COLORFX_ANTIQUE             = 14,   /* Antique */
  V4L2_COLORFX_SET_CBCR            = 15,   /* Set CbCr */
  V4L2_COLORFX_PASTEL              = 16    /* Pastel */
};

#define V4L2_CID_AUTOBRIGHTNESS       USER_CID(17)   /* Auto brightness */
#define V4L2_CID_ROTATE               USER_CID(18)   /* Rotation */

/* Camera class control IDs */

#define V4L2_CID_EXPOSURE_AUTO        CAMERA_CID(0)  /* Auto exposure */

/* Enumeration for V4L2_CID_EXPOSURE_AUTO */

enum v4l2_exposure_auto_type
{
  /* Exposure time:auto,   iris aperture:auto */

  V4L2_EXPOSURE_AUTO               = 0,

  /* Exposure time:manual, iris aperture:manual */

  V4L2_EXPOSURE_MANUAL             = 1,

  /* Exposure time:manual, iris aperture:auto */

  V4L2_EXPOSURE_SHUTTER_PRIORITY   = 2,

  /* Exposure time:auto,   iris aperture:manual */

  V4L2_EXPOSURE_APERTURE_PRIORITY  = 3
};

#define V4L2_CID_EXPOSURE_ABSOLUTE    CAMERA_CID(1) /* Exposure time */

#define V4L2_CID_FOCUS_ABSOLUTE       CAMERA_CID(2) /* Focus */
#define V4L2_CID_FOCUS_RELATIVE       CAMERA_CID(3) /* Focus */
#define V4L2_CID_FOCUS_AUTO           CAMERA_CID(4) /* Auto focus */

#define V4L2_CID_ZOOM_ABSOLUTE        CAMERA_CID(5) /* Zoom(absolute)  */
#define V4L2_CID_ZOOM_RELATIVE        CAMERA_CID(6) /* Zoom(relative)  */
#define V4L2_CID_ZOOM_CONTINUOUS      CAMERA_CID(7) /* Continuous zoom */

#define V4L2_CID_IRIS_ABSOLUTE        CAMERA_CID(8) /* Iris(absolute) */
#define V4L2_CID_IRIS_RELATIVE        CAMERA_CID(9) /* Iris(relative) */

/* Preset white balance */

#define V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE CAMERA_CID(10)

/* Enumeration for V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE */

enum v4l2_auto_n_preset_white_balance
{
  V4L2_WHITE_BALANCE_MANUAL        = 0, /* Manual */
  V4L2_WHITE_BALANCE_AUTO          = 1, /* Automatic */
  V4L2_WHITE_BALANCE_INCANDESCENT  = 2, /* Incandescent */
  V4L2_WHITE_BALANCE_FLUORESCENT   = 3, /* Fluorescent */
  V4L2_WHITE_BALANCE_FLUORESCENT_H = 4, /* Fluorescent H */
  V4L2_WHITE_BALANCE_HORIZON       = 5, /* Horizon */
  V4L2_WHITE_BALANCE_DAYLIGHT      = 6, /* Daylight */
  V4L2_WHITE_BALANCE_FLASH         = 7, /* Flash */
  V4L2_WHITE_BALANCE_CLOUDY        = 8, /* Cloudy */
  V4L2_WHITE_BALANCE_SHADE         = 9, /* Shade */
};

#define V4L2_CID_WIDE_DYNAMIC_RANGE   CAMERA_CID(11) /* Wide dynamic range */

/* Image stabilization */

#define V4L2_CID_IMAGE_STABILIZATION  CAMERA_CID(12)
#define V4L2_CID_ISO_SENSITIVITY      CAMERA_CID(13) /* ISO sensitivity */

/* Auto ISO sensitivity */

#define V4L2_CID_ISO_SENSITIVITY_AUTO CAMERA_CID(14)

/* Enumeration for V4L2_CID_ISO_SENSITIVITY_AUTO */

enum v4l2_iso_sensitivity_auto_type
{
  V4L2_ISO_SENSITIVITY_MANUAL  = 0,  /* Manual */
  V4L2_ISO_SENSITIVITY_AUTO    = 1,  /* Automatic */
};

#define V4L2_CID_EXPOSURE_METERING    CAMERA_CID(15) /* Exposure metering */

/* Enumeration for V4L2_CID_EXPOSURE_METERING */

enum v4l2_exposure_metering
{
  V4L2_EXPOSURE_METERING_AVERAGE         = 0, /* Average */
  V4L2_EXPOSURE_METERING_CENTER_WEIGHTED = 1, /* Center weighted */
  V4L2_EXPOSURE_METERING_SPOT            = 2, /* Spot */
  V4L2_EXPOSURE_METERING_MATRIX          = 3, /* Matrix */
};

#define V4L2_CID_SCENE_MODE           CAMERA_CID(16) /* Scene selection */

/* Enumeration for V4L2_CID_SCENE_MODE */

enum v4l2_scene_mode
{
  V4L2_SCENE_MODE_NONE         = 0,    /* No scene */
  V4L2_SCENE_MODE_BACKLIGHT    = 1,    /* Backlight */
  V4L2_SCENE_MODE_BEACH_SNOW   = 2,    /* Beach snow */
  V4L2_SCENE_MODE_CANDLE_LIGHT = 3,    /* Candle light */
  V4L2_SCENE_MODE_DAWN_DUSK    = 4,    /* Dawn dask */
  V4L2_SCENE_MODE_FALL_COLORS  = 5,    /* Fall colors */
  V4L2_SCENE_MODE_FIREWORKS    = 6,    /* Fire works */
  V4L2_SCENE_MODE_LANDSCAPE    = 7,    /* Landscape */
  V4L2_SCENE_MODE_NIGHT        = 8,    /* Night */
  V4L2_SCENE_MODE_PARTY_INDOOR = 9,    /* Indoor party */
  V4L2_SCENE_MODE_PORTRAIT     = 10,   /* Portrait */
  V4L2_SCENE_MODE_SPORTS       = 11,   /* Sports */
  V4L2_SCENE_MODE_SUNSET       = 12,   /* Sunset */
  V4L2_SCENE_MODE_TEXT         = 13,   /* Text */
  V4L2_SCENE_MODE_MAX          = 14    /* Max number */
};

#define V4L2_CID_3A_LOCK              CAMERA_CID(17) /* Lock 3A */
#define V4L2_LOCK_EXPOSURE            (1 << 0)       /* Exposure bit for
                                                      *   V4L2_CID_3A_LOCK */
#define V4L2_LOCK_WHITE_BALANCE       (1 << 1)       /* White balance bit for
                                                      *   V4L2_CID_3A_LOCK */
#define V4L2_LOCK_FOCUS               (1 << 2)       /* Focus bit for
                                                      *   V4L2_CID_3A_LOCK */

#define V4L2_CID_AUTO_FOCUS_START     CAMERA_CID(18) /* Start single AF */
#define V4L2_CID_AUTO_FOCUS_STOP      CAMERA_CID(19) /* Stop single AF */

#define V4L2_CID_3A_PARAMETER         CAMERA_CID(20) /* 3A parameter     */
#define V4L2_CID_3A_STATUS            CAMERA_CID(21) /* 3A status        */
#define V4L2_3A_STATUS_STABLE         (0)            /* 3A  is stable    */
#define V4L2_3A_STATUS_AE_OPERATING   (1 << 0)       /* AE  is operating */
#define V4L2_3A_STATUS_AWB_OPERATING  (1 << 1)       /* AWB is operating */
#define V4L2_3A_STATUS_AF_OPERATING   (1 << 2)       /* AF  is operating */

/* Spot position in spot exposure metering */

#define V4L2_CID_EXPOSURE_METERING_SPOT_POSITION CAMERA_CID(22)

/* Flash and privacy (indicator) light controls */

#define V4L2_CID_FLASH_LED_MODE       FLASH_CID(0)

/* Enumeration for V4L2_CID_FLASH_LED_MODE */

enum v4l2_flash_led_mode
{
  V4L2_FLASH_LED_MODE_NONE,  /* Not use LED */
  V4L2_FLASH_LED_MODE_FLASH, /* Flash mode */
  V4L2_FLASH_LED_MODE_TORCH, /* Torch mode */
};

/* JPEG-class control IDs */

#define V4L2_CID_JPEG_COMPRESSION_QUALITY JPEG_CID(0) /* JPEG quality */

#endif /* __INCLUDE_NUTTX_VIDEO_VIDEO_CONTROLS_H */
