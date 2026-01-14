/****************************************************************************
 * include/sys/video_controls.h
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

#ifndef __INCLUDE_SYS_VIDEO_CONTROLS_H
#define __INCLUDE_SYS_VIDEO_CONTROLS_H

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
#define V4L2_CTRL_CLASS_CODEC     (0x0500) /* Stateful codec controls */

#define V4L2_CTRL_CLASS_MPEG      V4L2_CTRL_CLASS_CODEC

#define USER_CID(v)   (V4L2_CTRL_CLASS_USER   + (v))
#define CAMERA_CID(v) (V4L2_CTRL_CLASS_CAMERA + (v))
#define FLASH_CID(v)  (V4L2_CTRL_CLASS_FLASH  + (v))
#define JPEG_CID(v)   (V4L2_CTRL_CLASS_JPEG   + (v))

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

/* MPEG-class control IDs */

/* The MPEG controls are applicable to all codec controls
 * and the 'MPEG' part of the define is historical
 */

#define V4L2_CID_CODEC_BASE     (V4L2_CTRL_CLASS_CODEC | 0x900)
#define V4L2_CID_CODEC_CLASS    (V4L2_CTRL_CLASS_CODEC | 1)

/*  MPEG video controls specific to multiplexed streams */

#define V4L2_CID_MPEG_VIDEO_ENCODING (V4L2_CID_CODEC_BASE + 200)

enum v4l2_mpeg_video_encoding
{
  V4L2_MPEG_VIDEO_ENCODING_MPEG_1     = 0,
  V4L2_MPEG_VIDEO_ENCODING_MPEG_2     = 1,
  V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC = 2,
};

#define V4L2_CID_MPEG_VIDEO_ASPECT (V4L2_CID_CODEC_BASE + 201)

enum v4l2_mpeg_video_aspect
{
  V4L2_MPEG_VIDEO_ASPECT_1x1     = 0,
  V4L2_MPEG_VIDEO_ASPECT_4x3     = 1,
  V4L2_MPEG_VIDEO_ASPECT_16x9    = 2,
  V4L2_MPEG_VIDEO_ASPECT_221x100 = 3,
};

#define V4L2_CID_MPEG_VIDEO_B_FRAMES     (V4L2_CID_CODEC_BASE + 202)
#define V4L2_CID_MPEG_VIDEO_GOP_SIZE     (V4L2_CID_CODEC_BASE + 203)
#define V4L2_CID_MPEG_VIDEO_GOP_CLOSURE  (V4L2_CID_CODEC_BASE + 204)
#define V4L2_CID_MPEG_VIDEO_PULLDOWN     (V4L2_CID_CODEC_BASE + 205)
#define V4L2_CID_MPEG_VIDEO_BITRATE_MODE (V4L2_CID_CODEC_BASE + 206)

enum v4l2_mpeg_video_bitrate_mode
{
  V4L2_MPEG_VIDEO_BITRATE_MODE_VBR = 0,
  V4L2_MPEG_VIDEO_BITRATE_MODE_CBR = 1,
  V4L2_MPEG_VIDEO_BITRATE_MODE_CQ  = 2,
};

#define V4L2_CID_MPEG_VIDEO_BITRATE                      (V4L2_CID_CODEC_BASE + 207)
#define V4L2_CID_MPEG_VIDEO_BITRATE_PEAK                 (V4L2_CID_CODEC_BASE + 208)
#define V4L2_CID_MPEG_VIDEO_TEMPORAL_DECIMATION          (V4L2_CID_CODEC_BASE + 209)
#define V4L2_CID_MPEG_VIDEO_MUTE                         (V4L2_CID_CODEC_BASE + 210)
#define V4L2_CID_MPEG_VIDEO_MUTE_YUV                     (V4L2_CID_CODEC_BASE + 211)
#define V4L2_CID_MPEG_VIDEO_DECODER_SLICE_INTERFACE      (V4L2_CID_CODEC_BASE + 212)
#define V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER (V4L2_CID_CODEC_BASE + 213)
#define V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB      (V4L2_CID_CODEC_BASE + 214)
#define V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE              (V4L2_CID_CODEC_BASE + 215)
#define V4L2_CID_MPEG_VIDEO_HEADER_MODE                  (V4L2_CID_CODEC_BASE + 216)

enum v4l2_mpeg_video_header_mode
{
  V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE              = 0,
  V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME = 1,
};

#define V4L2_CID_MPEG_VIDEO_MAX_REF_PIC           (V4L2_CID_CODEC_BASE + 217)
#define V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE          (V4L2_CID_CODEC_BASE + 218)
#define V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES (V4L2_CID_CODEC_BASE + 219)
#define V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB    (V4L2_CID_CODEC_BASE + 220)
#define V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE      (V4L2_CID_CODEC_BASE + 221)

enum v4l2_mpeg_video_multi_slice_mode
{
  V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE    = 0,
  V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB    = 1,
  V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_BYTES = 2,

  /* Kept for backwards compatibility reasons. Stupid typo... */

  V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB     = 1,
  V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES  = 2,
};

#define V4L2_CID_MPEG_VIDEO_VBV_SIZE                  (V4L2_CID_CODEC_BASE + 222)
#define V4L2_CID_MPEG_VIDEO_DEC_PTS                   (V4L2_CID_CODEC_BASE + 223)
#define V4L2_CID_MPEG_VIDEO_DEC_FRAME                 (V4L2_CID_CODEC_BASE + 224)
#define V4L2_CID_MPEG_VIDEO_VBV_DELAY                 (V4L2_CID_CODEC_BASE + 225)
#define V4L2_CID_MPEG_VIDEO_REPEAT_SEQ_HEADER         (V4L2_CID_CODEC_BASE + 226)
#define V4L2_CID_MPEG_VIDEO_MV_H_SEARCH_RANGE         (V4L2_CID_CODEC_BASE + 227)
#define V4L2_CID_MPEG_VIDEO_MV_V_SEARCH_RANGE         (V4L2_CID_CODEC_BASE + 228)
#define V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME           (V4L2_CID_CODEC_BASE + 229)
#define V4L2_CID_MPEG_VIDEO_BASELAYER_PRIORITY_ID     (V4L2_CID_CODEC_BASE + 230)
#define V4L2_CID_MPEG_VIDEO_AU_DELIMITER              (V4L2_CID_CODEC_BASE + 231)
#define V4L2_CID_MPEG_VIDEO_LTR_COUNT                 (V4L2_CID_CODEC_BASE + 232)
#define V4L2_CID_MPEG_VIDEO_FRAME_LTR_INDEX           (V4L2_CID_CODEC_BASE + 233)
#define V4L2_CID_MPEG_VIDEO_USE_LTR_FRAMES            (V4L2_CID_CODEC_BASE + 234)
#define V4L2_CID_MPEG_VIDEO_DEC_CONCEAL_COLOR         (V4L2_CID_CODEC_BASE + 235)
#define V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD      (V4L2_CID_CODEC_BASE + 236)
#define V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD_TYPE (V4L2_CID_CODEC_BASE + 237)

enum v4l2_mpeg_video_intra_refresh_period_type
{
  V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD_TYPE_RANDOM = 0,
  V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD_TYPE_CYCLIC = 1,
};

/* CIDs for the FWHT codec as used by the vicodec driver. */

#define V4L2_CID_FWHT_I_FRAME_QP               (V4L2_CID_CODEC_BASE + 290)
#define V4L2_CID_FWHT_P_FRAME_QP               (V4L2_CID_CODEC_BASE + 291)

#define V4L2_CID_MPEG_VIDEO_H263_I_FRAME_QP    (V4L2_CID_CODEC_BASE + 300)
#define V4L2_CID_MPEG_VIDEO_H263_P_FRAME_QP    (V4L2_CID_CODEC_BASE + 301)
#define V4L2_CID_MPEG_VIDEO_H263_B_FRAME_QP    (V4L2_CID_CODEC_BASE + 302)
#define V4L2_CID_MPEG_VIDEO_H263_MIN_QP        (V4L2_CID_CODEC_BASE + 303)
#define V4L2_CID_MPEG_VIDEO_H263_MAX_QP        (V4L2_CID_CODEC_BASE + 304)
#define V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP    (V4L2_CID_CODEC_BASE + 350)
#define V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP    (V4L2_CID_CODEC_BASE + 351)
#define V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP    (V4L2_CID_CODEC_BASE + 352)
#define V4L2_CID_MPEG_VIDEO_H264_MIN_QP        (V4L2_CID_CODEC_BASE + 353)
#define V4L2_CID_MPEG_VIDEO_H264_MAX_QP        (V4L2_CID_CODEC_BASE + 354)
#define V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM (V4L2_CID_CODEC_BASE + 355)
#define V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE      (V4L2_CID_CODEC_BASE + 356)
#define V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE  (V4L2_CID_CODEC_BASE + 357)

enum v4l2_mpeg_video_h264_entropy_mode
{
  V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC = 0,
  V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC = 1,
};

#define V4L2_CID_MPEG_VIDEO_H264_I_PERIOD (V4L2_CID_CODEC_BASE + 358)
#define V4L2_CID_MPEG_VIDEO_H264_LEVEL    (V4L2_CID_CODEC_BASE + 359)

enum v4l2_mpeg_video_h264_level
{
  V4L2_MPEG_VIDEO_H264_LEVEL_1_0 = 0,
  V4L2_MPEG_VIDEO_H264_LEVEL_1B  = 1,
  V4L2_MPEG_VIDEO_H264_LEVEL_1_1 = 2,
  V4L2_MPEG_VIDEO_H264_LEVEL_1_2 = 3,
  V4L2_MPEG_VIDEO_H264_LEVEL_1_3 = 4,
  V4L2_MPEG_VIDEO_H264_LEVEL_2_0 = 5,
  V4L2_MPEG_VIDEO_H264_LEVEL_2_1 = 6,
  V4L2_MPEG_VIDEO_H264_LEVEL_2_2 = 7,
  V4L2_MPEG_VIDEO_H264_LEVEL_3_0 = 8,
  V4L2_MPEG_VIDEO_H264_LEVEL_3_1 = 9,
  V4L2_MPEG_VIDEO_H264_LEVEL_3_2 = 10,
  V4L2_MPEG_VIDEO_H264_LEVEL_4_0 = 11,
  V4L2_MPEG_VIDEO_H264_LEVEL_4_1 = 12,
  V4L2_MPEG_VIDEO_H264_LEVEL_4_2 = 13,
  V4L2_MPEG_VIDEO_H264_LEVEL_5_0 = 14,
  V4L2_MPEG_VIDEO_H264_LEVEL_5_1 = 15,
  V4L2_MPEG_VIDEO_H264_LEVEL_5_2 = 16,
  V4L2_MPEG_VIDEO_H264_LEVEL_6_0 = 17,
  V4L2_MPEG_VIDEO_H264_LEVEL_6_1 = 18,
  V4L2_MPEG_VIDEO_H264_LEVEL_6_2 = 19,
};

#define V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA (V4L2_CID_CODEC_BASE + 360)
#define V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA  (V4L2_CID_CODEC_BASE + 361)
#define V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE  (V4L2_CID_CODEC_BASE + 362)

enum v4l2_mpeg_video_h264_loop_filter_mode
{
  V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED                    = 0,
  V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED                   = 1,
  V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY = 2,
};

#define V4L2_CID_MPEG_VIDEO_H264_PROFILE (V4L2_CID_CODEC_BASE + 363)

enum v4l2_mpeg_video_h264_profile
{
  V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE             = 0,
  V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE = 1,
  V4L2_MPEG_VIDEO_H264_PROFILE_MAIN                 = 2,
  V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED             = 3,
  V4L2_MPEG_VIDEO_H264_PROFILE_HIGH                 = 4,
  V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10              = 5,
  V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422             = 6,
  V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE  = 7,
  V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10_INTRA        = 8,
  V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA       = 9,
  V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_INTRA       = 10,
  V4L2_MPEG_VIDEO_H264_PROFILE_CAVLC_444_INTRA      = 11,
  V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_BASELINE    = 12,
  V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH        = 13,
  V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH_INTRA  = 14,
  V4L2_MPEG_VIDEO_H264_PROFILE_STEREO_HIGH          = 15,
  V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH       = 16,
  V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH     = 17,
};

#define V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT (V4L2_CID_CODEC_BASE + 364)
#define V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH  (V4L2_CID_CODEC_BASE + 365)
#define V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE     (V4L2_CID_CODEC_BASE + 366)
#define V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC        (V4L2_CID_CODEC_BASE + 367)

enum v4l2_mpeg_video_h264_vui_sar_idc
{
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED = 0,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_1x1         = 1,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_12x11       = 2,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_10x11       = 3,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_16x11       = 4,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_40x33       = 5,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_24x11       = 6,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_20x11       = 7,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_32x11       = 8,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_80x33       = 9,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_18x11       = 10,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_15x11       = 11,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_64x33       = 12,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_160x99      = 13,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_4x3         = 14,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_3x2         = 15,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_2x1         = 16,
  V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED    = 17,
};

#define V4L2_CID_MPEG_VIDEO_H264_SEI_FRAME_PACKING       (V4L2_CID_CODEC_BASE + 368)
#define V4L2_CID_MPEG_VIDEO_H264_SEI_FP_CURRENT_FRAME_0  (V4L2_CID_CODEC_BASE + 369)
#define V4L2_CID_MPEG_VIDEO_H264_SEI_FP_ARRANGEMENT_TYPE (V4L2_CID_CODEC_BASE + 370)

enum v4l2_mpeg_video_h264_sei_fp_arrangement_type
{
  V4L2_MPEG_VIDEO_H264_SEI_FP_ARRANGEMENT_TYPE_CHECKERBOARD = 0,
  V4L2_MPEG_VIDEO_H264_SEI_FP_ARRANGEMENT_TYPE_COLUMN       = 1,
  V4L2_MPEG_VIDEO_H264_SEI_FP_ARRANGEMENT_TYPE_ROW          = 2,
  V4L2_MPEG_VIDEO_H264_SEI_FP_ARRANGEMENT_TYPE_SIDE_BY_SIDE = 3,
  V4L2_MPEG_VIDEO_H264_SEI_FP_ARRANGEMENT_TYPE_TOP_BOTTOM   = 4,
  V4L2_MPEG_VIDEO_H264_SEI_FP_ARRANGEMENT_TYPE_TEMPORAL     = 5,
};

#define V4L2_CID_MPEG_VIDEO_H264_FMO          (V4L2_CID_CODEC_BASE + 371)
#define V4L2_CID_MPEG_VIDEO_H264_FMO_MAP_TYPE (V4L2_CID_CODEC_BASE + 372)

enum v4l2_mpeg_video_h264_fmo_map_type
{
  V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_INTERLEAVED_SLICES        = 0,
  V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_SCATTERED_SLICES          = 1,
  V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_FOREGROUND_WITH_LEFT_OVER = 2,
  V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_BOX_OUT                   = 3,
  V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_RASTER_SCAN               = 4,
  V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_WIPE_SCAN                 = 5,
  V4L2_MPEG_VIDEO_H264_FMO_MAP_TYPE_EXPLICIT                  = 6,
};

#define V4L2_CID_MPEG_VIDEO_H264_FMO_SLICE_GROUP      (V4L2_CID_CODEC_BASE + 373)
#define V4L2_CID_MPEG_VIDEO_H264_FMO_CHANGE_DIRECTION (V4L2_CID_CODEC_BASE + 374)

enum v4l2_mpeg_video_h264_fmo_change_dir
{
  V4L2_MPEG_VIDEO_H264_FMO_CHANGE_DIR_RIGHT = 0,
  V4L2_MPEG_VIDEO_H264_FMO_CHANGE_DIR_LEFT  = 1,
};

#define V4L2_CID_MPEG_VIDEO_H264_FMO_CHANGE_RATE          (V4L2_CID_CODEC_BASE + 375)
#define V4L2_CID_MPEG_VIDEO_H264_FMO_RUN_LENGTH           (V4L2_CID_CODEC_BASE + 376)
#define V4L2_CID_MPEG_VIDEO_H264_ASO                      (V4L2_CID_CODEC_BASE + 377)
#define V4L2_CID_MPEG_VIDEO_H264_ASO_SLICE_ORDER          (V4L2_CID_CODEC_BASE + 378)
#define V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING      (V4L2_CID_CODEC_BASE + 379)
#define V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_TYPE (V4L2_CID_CODEC_BASE + 380)

enum v4l2_mpeg_video_h264_hierarchical_coding_type
{
  V4L2_MPEG_VIDEO_H264_HIERARCHICAL_CODING_B = 0,
  V4L2_MPEG_VIDEO_H264_HIERARCHICAL_CODING_P = 1,
};

#define V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER    (V4L2_CID_CODEC_BASE + 381)
#define V4L2_CID_MPEG_VIDEO_H264_HIERARCHICAL_CODING_LAYER_QP (V4L2_CID_CODEC_BASE + 382)
#define V4L2_CID_MPEG_VIDEO_H264_CONSTRAINED_INTRA_PREDICTION (V4L2_CID_CODEC_BASE + 383)
#define V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET       (V4L2_CID_CODEC_BASE + 384)
#define V4L2_CID_MPEG_VIDEO_H264_I_FRAME_MIN_QP               (V4L2_CID_CODEC_BASE + 385)
#define V4L2_CID_MPEG_VIDEO_H264_I_FRAME_MAX_QP               (V4L2_CID_CODEC_BASE + 386)
#define V4L2_CID_MPEG_VIDEO_H264_P_FRAME_MIN_QP               (V4L2_CID_CODEC_BASE + 387)
#define V4L2_CID_MPEG_VIDEO_H264_P_FRAME_MAX_QP               (V4L2_CID_CODEC_BASE + 388)
#define V4L2_CID_MPEG_VIDEO_H264_B_FRAME_MIN_QP               (V4L2_CID_CODEC_BASE + 389)
#define V4L2_CID_MPEG_VIDEO_H264_B_FRAME_MAX_QP               (V4L2_CID_CODEC_BASE + 390)
#define V4L2_CID_MPEG_VIDEO_H264_HIER_CODING_L0_BR            (V4L2_CID_CODEC_BASE + 391)
#define V4L2_CID_MPEG_VIDEO_H264_HIER_CODING_L1_BR            (V4L2_CID_CODEC_BASE + 392)
#define V4L2_CID_MPEG_VIDEO_H264_HIER_CODING_L2_BR            (V4L2_CID_CODEC_BASE + 393)
#define V4L2_CID_MPEG_VIDEO_H264_HIER_CODING_L3_BR            (V4L2_CID_CODEC_BASE + 394)
#define V4L2_CID_MPEG_VIDEO_H264_HIER_CODING_L4_BR            (V4L2_CID_CODEC_BASE + 395)
#define V4L2_CID_MPEG_VIDEO_H264_HIER_CODING_L5_BR            (V4L2_CID_CODEC_BASE + 396)
#define V4L2_CID_MPEG_VIDEO_H264_HIER_CODING_L6_BR            (V4L2_CID_CODEC_BASE + 397)
#define V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP                  (V4L2_CID_CODEC_BASE + 400)
#define V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP                  (V4L2_CID_CODEC_BASE + 401)
#define V4L2_CID_MPEG_VIDEO_MPEG4_B_FRAME_QP                  (V4L2_CID_CODEC_BASE + 402)
#define V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP                      (V4L2_CID_CODEC_BASE + 403)
#define V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP                      (V4L2_CID_CODEC_BASE + 404)
#define V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL                       (V4L2_CID_CODEC_BASE + 405)

enum v4l2_mpeg_video_mpeg4_level
{
  V4L2_MPEG_VIDEO_MPEG4_LEVEL_0  = 0,
  V4L2_MPEG_VIDEO_MPEG4_LEVEL_0B = 1,
  V4L2_MPEG_VIDEO_MPEG4_LEVEL_1  = 2,
  V4L2_MPEG_VIDEO_MPEG4_LEVEL_2  = 3,
  V4L2_MPEG_VIDEO_MPEG4_LEVEL_3  = 4,
  V4L2_MPEG_VIDEO_MPEG4_LEVEL_3B = 5,
  V4L2_MPEG_VIDEO_MPEG4_LEVEL_4  = 6,
  V4L2_MPEG_VIDEO_MPEG4_LEVEL_5  = 7,
};

#define V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE (V4L2_CID_CODEC_BASE + 406)

enum v4l2_mpeg_video_mpeg4_profile
{
  V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE                     = 0,
  V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_SIMPLE            = 1,
  V4L2_MPEG_VIDEO_MPEG4_PROFILE_CORE                       = 2,
  V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE_SCALABLE            = 3,
  V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_CODING_EFFICIENCY = 4,
};

#define V4L2_CID_MPEG_VIDEO_MPEG4_QPEL  (V4L2_CID_CODEC_BASE + 407)

/*  Control IDs for VP8 streams
 *  Although VP8 is not part of MPEG we add these controls to the MPEG class
 *  as that class is already handling other video compression standards
 */
#define V4L2_CID_MPEG_VIDEO_VPX_NUM_PARTITIONS  (V4L2_CID_CODEC_BASE + 500)

enum v4l2_vp8_num_partitions
{
  V4L2_CID_MPEG_VIDEO_VPX_1_PARTITION  = 0,
  V4L2_CID_MPEG_VIDEO_VPX_2_PARTITIONS = 1,
  V4L2_CID_MPEG_VIDEO_VPX_4_PARTITIONS = 2,
  V4L2_CID_MPEG_VIDEO_VPX_8_PARTITIONS = 3,
};

#define V4L2_CID_MPEG_VIDEO_VPX_IMD_DISABLE_4X4 (V4L2_CID_CODEC_BASE + 501)
#define V4L2_CID_MPEG_VIDEO_VPX_NUM_REF_FRAMES  (V4L2_CID_CODEC_BASE + 502)

enum v4l2_vp8_num_ref_frames
{
  V4L2_CID_MPEG_VIDEO_VPX_1_REF_FRAME = 0,
  V4L2_CID_MPEG_VIDEO_VPX_2_REF_FRAME = 1,
  V4L2_CID_MPEG_VIDEO_VPX_3_REF_FRAME = 2,
};

#define V4L2_CID_MPEG_VIDEO_VPX_FILTER_LEVEL            (V4L2_CID_CODEC_BASE + 503)
#define V4L2_CID_MPEG_VIDEO_VPX_FILTER_SHARPNESS        (V4L2_CID_CODEC_BASE + 504)
#define V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_REF_PERIOD (V4L2_CID_CODEC_BASE + 505)
#define V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_SEL        (V4L2_CID_CODEC_BASE + 506)

enum v4l2_vp8_golden_frame_sel
{
  V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_USE_PREV       = 0,
  V4L2_CID_MPEG_VIDEO_VPX_GOLDEN_FRAME_USE_REF_PERIOD = 1,
};

#define V4L2_CID_MPEG_VIDEO_VPX_MIN_QP     (V4L2_CID_CODEC_BASE + 507)
#define V4L2_CID_MPEG_VIDEO_VPX_MAX_QP     (V4L2_CID_CODEC_BASE + 508)
#define V4L2_CID_MPEG_VIDEO_VPX_I_FRAME_QP (V4L2_CID_CODEC_BASE + 509)
#define V4L2_CID_MPEG_VIDEO_VPX_P_FRAME_QP (V4L2_CID_CODEC_BASE + 510)

#define V4L2_CID_MPEG_VIDEO_VP8_PROFILE    (V4L2_CID_CODEC_BASE + 511)

enum v4l2_mpeg_video_vp8_profile
{
  V4L2_MPEG_VIDEO_VP8_PROFILE_0 = 0,
  V4L2_MPEG_VIDEO_VP8_PROFILE_1 = 1,
  V4L2_MPEG_VIDEO_VP8_PROFILE_2 = 2,
  V4L2_MPEG_VIDEO_VP8_PROFILE_3 = 3,
};

#endif /* __INCLUDE_SYS_VIDEO_CONTROLS_H */
