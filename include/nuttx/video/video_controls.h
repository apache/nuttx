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

#define V4L2_CTRL_CLASS_USER      (0x0000) /**< Old-style 'user' controls */
#define V4L2_CTRL_CLASS_CAMERA    (0x0001) /**< Camera class controls */
#define V4L2_CTRL_CLASS_FLASH     (0x0002) /**< Camera flash controls */
#define V4L2_CTRL_CLASS_JPEG      (0x0003) /**< JPEG-compression controls */

/* User-class control IDs */

#define V4L2_CID_BRIGHTNESS         (0)    /**< Brightness */
#define V4L2_CID_CONTRAST           (1)    /**< Contrast   */
#define V4L2_CID_SATURATION         (2)    /**< Saturation */
#define V4L2_CID_HUE                (3)    /**< Hue        */
#define V4L2_CID_AUTO_WHITE_BALANCE (4)    /**< AWB        */
#define V4L2_CID_RED_BALANCE        (5)    /**< Red balance */
#define V4L2_CID_BLUE_BALANCE       (6)    /**< Blue balance */
#define V4L2_CID_GAMMA              (7)    /**< Gamma value adjustment */
#define V4L2_CID_GAMMA_CURVE        (8)    /**< Gamma curve adjustment */
#define V4L2_CID_EXPOSURE           (9)    /**< Exposure value */
#define V4L2_CID_HFLIP              (10)   /**< Mirror horizontally(VIDEO) */
#define V4L2_CID_VFLIP              (11)   /**< Mirror vertically(VIDEO) */
#define V4L2_CID_HFLIP_STILL        (12)   /**< Mirror horizontally(STILL) */
#define V4L2_CID_VFLIP_STILL        (13)   /**< Mirror vertically(STILL) */
#define V4L2_CID_SHARPNESS          (14)   /**< Sharpness */
#define V4L2_CID_COLOR_KILLER       (15)   /**< Color killer */
#define V4L2_CID_COLORFX            (16)   /**< Color effect */

/** enumeration for V4L2_CID_COLORFX */

enum v4l2_colorfx
{
  V4L2_COLORFX_NONE                = 0,    /**< no effect */
  V4L2_COLORFX_BW                  = 1,    /**< Black/white */
  V4L2_COLORFX_SEPIA               = 2,    /**< Sepia */
  V4L2_COLORFX_NEGATIVE            = 3,    /**< positive/negative inversion */
  V4L2_COLORFX_EMBOSS              = 4,    /**< Emboss */
  V4L2_COLORFX_SKETCH              = 5,    /**< Sketch */
  V4L2_COLORFX_SKY_BLUE            = 6,    /**< Sky blue */
  V4L2_COLORFX_GRASS_GREEN         = 7,    /**< Grass green */
  V4L2_COLORFX_SKIN_WHITEN         = 8,    /**< Skin whiten */
  V4L2_COLORFX_VIVID               = 9,    /**< Vivid */
  V4L2_COLORFX_AQUA                = 10,   /**< Aqua */
  V4L2_COLORFX_ART_FREEZE          = 11,   /**< Art freeze */
  V4L2_COLORFX_SILHOUETTE          = 12,   /**< Silhouette */
  V4L2_COLORFX_SOLARIZATION        = 13,   /**< Solarization */
  V4L2_COLORFX_ANTIQUE             = 14,   /**< Antique */
  V4L2_COLORFX_SET_CBCR            = 15,   /**< Set CbCr */
  V4L2_COLORFX_PASTEL              = 16    /**< Pastel */
};
#define V4L2_CID_AUTOBRIGHTNESS     (17)   /**< Auto brightness */
#define V4L2_CID_ROTATE             (18)   /**< Rotation */

/**  Camera class control IDs */

#define V4L2_CID_EXPOSURE_AUTO      (0)    /**< Auto exposure */

/** enumeration for V4L2_CID_EXPOSURE_AUTO */

enum  v4l2_exposure_auto_type
{
  /** exposure time:auto,   iris aperture:auto */

  V4L2_EXPOSURE_AUTO               = 0,

  /** exposure time:manual, iris aperture:manual */

  V4L2_EXPOSURE_MANUAL             = 1,

  /** exposure time:manual, iris aperture:auto */

  V4L2_EXPOSURE_SHUTTER_PRIORITY   = 2,

  /** exposure time:auto,   iris aperture:manual */

  V4L2_EXPOSURE_APERTURE_PRIORITY  = 3
};
#define V4L2_CID_EXPOSURE_ABSOLUTE  (1)    /**< Exposure time */

#define V4L2_CID_FOCUS_ABSOLUTE     (2)    /** Focus */
#define V4L2_CID_FOCUS_RELATIVE     (3)    /** Focus */
#define V4L2_CID_FOCUS_AUTO         (4)    /** Auto focus */

#define V4L2_CID_ZOOM_ABSOLUTE      (5)    /** Zoom(absolute)  */
#define V4L2_CID_ZOOM_RELATIVE      (6)    /** Zoom(relative)  */
#define V4L2_CID_ZOOM_CONTINUOUS    (7)    /** Continuous zoom */

#define V4L2_CID_IRIS_ABSOLUTE      (8)    /** Iris(absolute) */
#define V4L2_CID_IRIS_RELATIVE      (9)    /** Iris(relative) */

#define V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE (10) /**< Preset white balance */

/** enumeration for V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE */

enum v4l2_auto_n_preset_white_balance
{
  V4L2_WHITE_BALANCE_MANUAL        = 0, /**< Manual */
  V4L2_WHITE_BALANCE_AUTO          = 1, /**< Automatic */
  V4L2_WHITE_BALANCE_INCANDESCENT  = 2, /**< Incandescent */
  V4L2_WHITE_BALANCE_FLUORESCENT   = 3, /**< Fluorescent */
  V4L2_WHITE_BALANCE_FLUORESCENT_H = 4, /**< Fluorescent H */
  V4L2_WHITE_BALANCE_HORIZON       = 5, /**< Horizon */
  V4L2_WHITE_BALANCE_DAYLIGHT      = 6, /**< Daylight */
  V4L2_WHITE_BALANCE_FLASH         = 7, /**< Flash */
  V4L2_WHITE_BALANCE_CLOUDY        = 8, /**< Cloudy */
  V4L2_WHITE_BALANCE_SHADE         = 9, /**< Shade */
};

#define V4L2_CID_WIDE_DYNAMIC_RANGE   (11) /**< Wide dynamic range */
#define V4L2_CID_IMAGE_STABILIZATION  (12) /**< Image stabilization */

#define V4L2_CID_ISO_SENSITIVITY      (13) /**< ISO sensitivity */
#define V4L2_CID_ISO_SENSITIVITY_AUTO (14) /**< Auto ISO sensitivity */

/** enumeration for V4L2_CID_ISO_SENSITIVITY_AUTO */

enum v4l2_iso_sensitivity_auto_type
{
  V4L2_ISO_SENSITIVITY_MANUAL  = 0,  /**< Manual */
  V4L2_ISO_SENSITIVITY_AUTO    = 1,  /**< Automatic */
};

#define V4L2_CID_EXPOSURE_METERING    (15)    /**< Exposure metering */

/** enumeration for V4L2_CID_EXPOSURE_METERING */

enum v4l2_exposure_metering
{
  V4L2_EXPOSURE_METERING_AVERAGE         = 0, /**< Average */
  V4L2_EXPOSURE_METERING_CENTER_WEIGHTED = 1, /**< Center weighted */
  V4L2_EXPOSURE_METERING_SPOT            = 2, /**< Spot */
  V4L2_EXPOSURE_METERING_MATRIX          = 3, /**< Matrix */
};

#define V4L2_CID_SCENE_MODE     (16)   /**< Scene selection */

/** enumeration for V4L2_CID_SCENE_MODE */

enum v4l2_scene_mode
{
  V4L2_SCENE_MODE_NONE         = 0,    /**< No scene */
  V4L2_SCENE_MODE_BACKLIGHT    = 1,    /**< Backlight */
  V4L2_SCENE_MODE_BEACH_SNOW   = 2,    /**< Beach snow */
  V4L2_SCENE_MODE_CANDLE_LIGHT = 3,    /**< Candle light */
  V4L2_SCENE_MODE_DAWN_DUSK    = 4,    /**< Dawn dask */
  V4L2_SCENE_MODE_FALL_COLORS  = 5,    /**< Fall colors */
  V4L2_SCENE_MODE_FIREWORKS    = 6,    /**< Fire works */
  V4L2_SCENE_MODE_LANDSCAPE    = 7,    /**< Landscape */
  V4L2_SCENE_MODE_NIGHT        = 8,    /**< Night */
  V4L2_SCENE_MODE_PARTY_INDOOR = 9,    /**< Indoor party */
  V4L2_SCENE_MODE_PORTRAIT     = 10,   /**< Portrait */
  V4L2_SCENE_MODE_SPORTS       = 11,   /**< Sports */
  V4L2_SCENE_MODE_SUNSET       = 12,   /**< Sunset */
  V4L2_SCENE_MODE_TEXT         = 13    /**< Text */
};

#define V4L2_CID_3A_LOCK         (17)     /**< Lock 3A */
#define V4L2_LOCK_EXPOSURE       (1 << 0) /**< Exposure bit for
                                           *   V4L2_CID_3A_LOCK */
#define V4L2_LOCK_WHITE_BALANCE  (1 << 1) /**< White balance bit for
                                           *   V4L2_CID_3A_LOCK */
#define V4L2_LOCK_FOCUS          (1 << 2) /**< Focus bit for
                                           *   V4L2_CID_3A_LOCK */

#define V4L2_CID_AUTO_FOCUS_START (18)    /**< Start single AF */
#define V4L2_CID_AUTO_FOCUS_STOP  (19)    /**< Stop single AF */

#define V4L2_CID_3A_PARAMETER        (20)     /**< 3A parameter     */
#define V4L2_CID_3A_STATUS           (21)     /**< 3A status        */
#define V4L2_3A_STATUS_STABLE        (0)      /**< 3A  is stable    */
#define V4L2_3A_STATUS_AE_OPERATING  (1 << 0) /**< AE  is operating */
#define V4L2_3A_STATUS_AWB_OPERATING (1 << 1) /**< AWB is operating */
#define V4L2_3A_STATUS_AF_OPERATING  (1 << 2) /**< AF  is operating */

/** Flash and privacy (indicator) light controls */

#define V4L2_CID_FLASH_LED_MODE   (0)

/** enumeration for V4L2_CID_FLASH_LED_MODE */

enum v4l2_flash_led_mode
{
  V4L2_FLASH_LED_MODE_NONE,  /**< Not use LED */
  V4L2_FLASH_LED_MODE_FLASH, /**< Flash mode */
  V4L2_FLASH_LED_MODE_TORCH, /**< Torch mode */
};

/* JPEG-class control IDs */

#define	V4L2_CID_JPEG_COMPRESSION_QUALITY (0) /**< JPEG quality */

#endif /* __INCLUDE_NUTTX_VIDEO_VIDEO_CONTROLS_H */
