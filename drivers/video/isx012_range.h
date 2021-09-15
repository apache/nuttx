/****************************************************************************
 * drivers/video/isx012_range.h
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
#include "isx012_reg.h"

#ifndef __DRIVERS_VIDEO_ISX012_RANGE_H
#define __DRIVERS_VIDEO_ISX012_RANGE_H

/* Definition for control brightness */

#define ISX012_TYPE_BRIGHTNESS      V4L2_CTRL_TYPE_INTEGER
#define ISX012_DEF_BRIGHTNESS       (0)
#define ISX012_MIN_BRIGHTNESS       (-128)
#define ISX012_MAX_BRIGHTNESS       (127)
#define ISX012_STEP_BRIGHTNESS      (1)
#define ISX012_REG_BRIGHTNESS       UIBRIGHTNESS
#define ISX012_SIZE_BRIGHTNESS      (1)

/* Definition for control contrast */

#define ISX012_TYPE_CONTRAST        V4L2_CTRL_TYPE_U8FIXEDPOINT_Q7
#define ISX012_DEF_CONTRAST         (0x80)
#define ISX012_MIN_CONTRAST         (0x00)
#define ISX012_MAX_CONTRAST         (0xFF)
#define ISX012_STEP_CONTRAST        (1)
#define ISX012_REG_CONTRAST         UICONTRAST
#define ISX012_SIZE_CONTRAST        (1)

/* Definition for control saturation */

#define ISX012_TYPE_SATURATION      V4L2_CTRL_TYPE_INTEGER
#define ISX012_DEF_SATURATION       (0)
#define ISX012_MIN_SATURATION       (0)
#define ISX012_MAX_SATURATION       (255)
#define ISX012_STEP_SATURATION      (1)
#define ISX012_REG_SATURATION       UISATURATION_TYPE1
#define ISX012_SIZE_SATURATION      (1)

/* Definition for control hue */

#define ISX012_TYPE_HUE             V4L2_CTRL_TYPE_INTEGER
#define ISX012_DEF_HUE              (0)
#define ISX012_MIN_HUE              (0)
#define ISX012_MAX_HUE              (255)
#define ISX012_STEP_HUE             (1)
#define ISX012_REG_HUE              UIHUE_TYPE1
#define ISX012_SIZE_HUE             (1)

/* Definition for control auto white balance */

#define ISX012_TYPE_AUTOWB          V4L2_CTRL_TYPE_BOOLEAN
#define ISX012_DEF_AUTOWB           true
#define ISX012_MIN_AUTOWB           false
#define ISX012_MAX_AUTOWB           true
#define ISX012_STEP_AUTOWB          (1)
#define ISX012_REG_AUTOWB           CPUEXT
#define ISX012_SIZE_AUTOWB          (1)

/* Definition for control red balance */

#define ISX012_TYPE_REDBALANCE      V4L2_CTRL_TYPE_INTEGER
#define ISX012_DEF_REDBALANCE       (0)
#define ISX012_MIN_REDBALANCE       (0)
#define ISX012_MAX_REDBALANCE       (65535)
#define ISX012_STEP_REDBALANCE      (1)
#define ISX012_REG_REDBALANCE       RATIO_R_AUTO
#define ISX012_SIZE_REDBALANCE      (2)

/* Definition for control blue balance */

#define ISX012_TYPE_BLUEBALANCE     V4L2_CTRL_TYPE_INTEGER
#define ISX012_DEF_BLUEBALANCE      (0)
#define ISX012_MIN_BLUEBALANCE      (0)
#define ISX012_MAX_BLUEBALANCE      (65535)
#define ISX012_STEP_BLUEBALANCE     (1)
#define ISX012_REG_BLUEBALANCE      RATIO_B_AUTO
#define ISX012_SIZE_BLUEBALANCE     (2)

/* Definition for control gamma curve */

#define ISX012_TYPE_GAMMACURVE      V4L2_CTRL_TYPE_U16
#define ISX012_DEF_GAMMACURVE       (0)
#define ISX012_MIN_GAMMACURVE       (0)
#define ISX012_MAX_GAMMACURVE       (511)
#define ISX012_STEP_GAMMACURVE      (1)
#define ISX012_ELEMSIZE_GAMMACURVE  (1)
#define ISX012_ELEMS_GAMMACURVE     (19)
#define ISX012_REG_GAMMACURVE       GAMMA_BASE
#define ISX012_SIZE_GAMMACURVE      (2)

/* Definition for control exposure value */

#define ISX012_TYPE_EXPOSURE        V4L2_CTRL_TYPE_INTEGER_TIMES_3
#define ISX012_DEF_EXPOSURE         (0)
#define ISX012_MIN_EXPOSURE         (-6)
#define ISX012_MAX_EXPOSURE         (6)
#define ISX012_STEP_EXPOSURE        (1)
#define ISX012_REG_EXPOSURE         EVSEL
#define ISX012_SIZE_EXPOSURE        (1)

/* Definition for control horizontal mirroring(V4L2_BUF_TYPE_VIDEO_CAPTURE) */

#define ISX012_TYPE_HFLIP           V4L2_CTRL_TYPE_BOOLEAN
#define ISX012_DEF_HFLIP            false
#define ISX012_MIN_HFLIP            false
#define ISX012_MAX_HFLIP            true
#define ISX012_STEP_HFLIP           (1)
#define ISX012_REG_HFLIP            READVECT_MONI
#define ISX012_SIZE_HFLIP           (1)

/* Definition for control vertical mirroring(V4L2_BUF_TYPE_VIDEO_CAPTURE) */

#define ISX012_TYPE_VFLIP           V4L2_CTRL_TYPE_BOOLEAN
#define ISX012_DEF_VFLIP            false
#define ISX012_MIN_VFLIP            false
#define ISX012_MAX_VFLIP            true
#define ISX012_STEP_VFLIP           (1)
#define ISX012_REG_VFLIP            READVECT_MONI
#define ISX012_SIZE_VFLIP           (1)

/* Definition for control horizontal mirroring(V4L2_BUF_TYPE_STILL_CAPTURE) */

#define ISX012_TYPE_HFLIP_STILL     V4L2_CTRL_TYPE_BOOLEAN
#define ISX012_DEF_HFLIP_STILL      false
#define ISX012_MIN_HFLIP_STILL      false
#define ISX012_MAX_HFLIP_STILL      true
#define ISX012_STEP_HFLIP_STILL     (1)
#define ISX012_REG_HFLIP_STILL      READVECT_CAP
#define ISX012_SIZE_HFLIP_STILL     (1)

/* Definition for control vertical mirroring(V4L2_BUF_TYPE_STILL_CAPTURE) */

#define ISX012_TYPE_VFLIP_STILL     V4L2_CTRL_TYPE_BOOLEAN
#define ISX012_DEF_VFLIP_STILL      false
#define ISX012_MIN_VFLIP_STILL      false
#define ISX012_MAX_VFLIP_STILL      true
#define ISX012_STEP_VFLIP_STILL     (1)
#define ISX012_REG_VFLIP_STILL      READVECT_CAP
#define ISX012_SIZE_VFLIP_STILL     (1)

/* Definition for control sharpness */

#define ISX012_TYPE_SHARPNESS       V4L2_CTRL_TYPE_INTEGER
#define ISX012_DEF_SHARPNESS        (0)
#define ISX012_MIN_SHARPNESS        (0)
#define ISX012_MAX_SHARPNESS        (255)
#define ISX012_STEP_SHARPNESS       (1)
#define ISX012_REG_SHARPNESS        UISHARPNESS_POS_TYPE1
#define ISX012_SIZE_SHARPNESS       (1)

/* Definition for control color killer */

#define ISX012_TYPE_COLORKILLER     V4L2_CTRL_TYPE_BOOLEAN
#define ISX012_DEF_COLORKILLER      false
#define ISX012_MIN_COLORKILLER      false
#define ISX012_MAX_COLORKILLER      true
#define ISX012_STEP_COLORKILLER     (1)
#define ISX012_REG_COLORKILLER      FMODE
#define ISX012_SIZE_COLORKILLER     (1)

/* Definition for control color effect */

#define ISX012_TYPE_COLOREFFECT     V4L2_CTRL_TYPE_INTEGER_MENU
#define ISX012_DEF_COLOREFFECT      V4L2_COLORFX_NONE
#define ISX012_MIN_COLOREFFECT      (0)
#define ISX012_MAX_COLOREFFECT      (6)
#define ISX012_STEP_COLOREFFECT     (1)
#define ISX012_REG_COLOREFFECT      FMODE
#define ISX012_SIZE_COLOREFFECT     (1)

/* Definition for control auto exposure */

#define ISX012_TYPE_EXPOSUREAUTO         V4L2_CTRL_TYPE_INTEGER
#define ISX012_DEF_EXPOSUREAUTO          (0)
#define ISX012_MIN_EXPOSUREAUTO          (0)
#define ISX012_MAX_EXPOSUREAUTO          (1)
#define ISX012_STEP_EXPOSUREAUTO         (1)

#define ISX012_REG_EXPOSUREAUTOVALUE_LSB SHT_TIME_AUTO_L
#define ISX012_REG_EXPOSUREAUTOVALUE_MSB SHT_TIME_AUTO_H
#define ISX012_SIZE_EXPOSUREAUTOVALUE    (2)

/* Definition for control exposure time */

#define ISX012_TYPE_EXPOSURETIME    V4L2_CTRL_TYPE_INTEGER
#define ISX012_DEF_EXPOSURETIME     (0)
#define ISX012_MIN_EXPOSURETIME     (1)
#define ISX012_MAX_EXPOSURETIME     (21000)
#define ISX012_STEP_EXPOSURETIME    (1)
#define ISX012_REG_EXPOSURETIME     SHT_PREMODE_TYPE1
#define ISX012_SIZE_EXPOSURETIME    (2)

#define ISX012_UNIT_EXPOSURETIME_US (100)

/* Definition for control photometry */

#define ISX012_TYPE_PHOTOMETRY    V4L2_CTRL_TYPE_INTEGER_MENU
#define ISX012_DEF_PHOTOMETRY     V4L2_EXPOSURE_METERING_AVERAGE
#define ISX012_MIN_PHOTOMETRY     (0)
#define ISX012_MAX_PHOTOMETRY     (3)
#define ISX012_STEP_PHOTOMETRY    (1)
#define ISX012_REG_PHOTOMETRY     AE_SUB_SN1
#define ISX012_SIZE_PHOTOMETRY    (1)

/* Definition for control zoom */

#define ISX012_TYPE_ZOOM            V4L2_CTRL_TYPE_U16FIXEDPOINT_Q8
#define ISX012_DEF_ZOOM             (0x0100)
#define ISX012_MIN_ZOOM             (0x0100)
#define ISX012_MAX_ZOOM             (0x1000)
#define ISX012_STEP_ZOOM            (1)
#define ISX012_REG_ZOOM             EZOOM_MAG
#define ISX012_SIZE_ZOOM            (2)

/* Definition for control preset white balance */

#define ISX012_TYPE_PRESETWB        V4L2_CTRL_TYPE_INTEGER_MENU
#define ISX012_DEF_PRESETWB         V4L2_WHITE_BALANCE_AUTO
#define ISX012_MIN_PRESETWB         (0)
#define ISX012_MAX_PRESETWB         (5)
#define ISX012_STEP_PRESETWB        (1)
#define ISX012_REG_PRESETWB         AWB_SN1
#define ISX012_SIZE_PRESETWB        (2)

/* Definition for control YGAMMA adujust */

#define ISX012_TYPE_YGAMMA          V4L2_CTRL_TYPE_BOOLEAN
#define ISX012_DEF_YGAMMA           (false)
#define ISX012_MIN_YGAMMA           (false)
#define ISX012_MAX_YGAMMA           (true)
#define ISX012_STEP_YGAMMA          (1)
#define ISX012_REG_YGAMMA           YGAMMA_MODE
#define ISX012_SIZE_YGAMMA          (1)

/* Definition for control ISO sensitivity */

#define ISX012_TYPE_ISO              V4L2_CTRL_TYPE_INTEGER_MENU
#define ISX012_DEF_ISO               (0)
#define ISX012_MIN_ISO               (0)
#define ISX012_MAX_ISO               (18)
#define ISX012_STEP_ISO              (1)
#define ISX012_REG_ISO               ISO_TYPE1
#define ISX012_SIZE_ISO              (1)

/* Definition for control ISO automatic */

#define ISX012_TYPE_ISOAUTO          V4L2_CTRL_TYPE_INTEGER_MENU
#define ISX012_DEF_ISOAUTO           (false)
#define ISX012_MIN_ISOAUTO           (0)
#define ISX012_MAX_ISOAUTO           (1)
#define ISX012_STEP_ISOAUTO          (1)
#define ISX012_REG_ISOAUTO           ISO_TYPE1
#define ISX012_SIZE_ISOAUTO          (1)
#define ISX012_REG_ISOAUTOVALUE      ISOSENS_OUT
#define ISX012_SIZE_ISOAUTOVALUE     (1)

/* Definition for control 3A lock */

#define ISX012_TYPE_3ALOCK           V4L2_CTRL_TYPE_BITMASK
#define ISX012_DEF_3ALOCK            (0)
#define ISX012_MIN_3ALOCK            (0)
#define ISX012_MAX_3ALOCK            (3)
#define ISX012_STEP_3ALOCK           (1)
#define ISX012_REG_3ALOCK            CPUEXT
#define ISX012_SIZE_3ALOCK           (1)

/* Definition for control JPEG compression quality */

#define ISX012_TYPE_JPGQUALITY      V4L2_CTRL_TYPE_INTEGER
#define ISX012_DEF_JPGQUALITY       (80)
#define ISX012_MIN_JPGQUALITY       (1)
#define ISX012_MAX_JPGQUALITY       (100)
#define ISX012_STEP_JPGQUALITY      (1)
#define ISX012_REG_JPGQUALITY       INT_QLTY2
#define ISX012_SIZE_JPGQUALITY      (1)

#endif /* __DRIVERS_VIDEO_ISX012_RANGE_H */
