/****************************************************************************
 * drivers/video/isx019_range.h
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

#ifndef __DRIVERS_VIDEO_ISX019_RANGE_H
#define __DRIVERS_VIDEO_ISX019_RANGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <limits.h>
#include <nuttx/video/imgsensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIN_BRIGHTNESS        (-128)
#define MAX_BRIGHTNESS        (127)
#define STEP_BRIGHTNESS       (1)

#define MIN_CONTRAST          (0)
#define MAX_CONTRAST          (255)
#define STEP_CONTRAST         (1)

#define MIN_SATURATION        (0)
#define MAX_SATURATION        (255)
#define STEP_SATURATION       (1)

#define MIN_HUE               (-128)
#define MAX_HUE               (127)
#define STEP_HUE              (1)

#define MIN_GAMMA             (0)
#define MAX_GAMMA             (INT_MAX)
#define STEP_GAMMA            (1)

#define MIN_AWB               (0)
#define MAX_AWB               (1)
#define STEP_AWB              (1)

#define MIN_EXPOSURE          (-6)
#define MAX_EXPOSURE          (6)
#define STEP_EXPOSURE         (1)

#define MIN_HFLIP             (0)
#define MAX_HFLIP             (1)
#define STEP_HFLIP            (1)

#define MIN_VFLIP        (0)
#define MAX_VFLIP        (1)
#define STEP_VFLIP       (1)

#define MIN_SHARPNESS        (0)
#define MAX_SHARPNESS        (255)
#define STEP_SHARPNESS       (1)

#define MIN_AE        (0)
#define MAX_AE        (1)
#define STEP_AE       (1)

#define MIN_EXPOSURETIME        (1)
#define MAX_EXPOSURETIME        (102000)
#define STEP_EXPOSURETIME       (1)

#define MIN_WBMODE        (0)
#define MAX_WBMODE        (7)
#define STEP_WBMODE       (1)

#define MIN_HDR           (0)
#define MAX_HDR           (2)
#define STEP_HDR          (1)

#define MIN_METER        (0)
#define MAX_METER        (3)
#define STEP_METER       (1)

#define MIN_PAN        (-32)
#define MAX_PAN        (32)
#define STEP_PAN       (1)

#define MIN_TILT        (-32)
#define MAX_TILT        (32)
#define STEP_TILT       (1)

#define MIN_ISO        (1)
#define MAX_ISO        (INT_MAX)
#define STEP_ISO       (1)

#define MIN_AUTOISO        (0)
#define MAX_AUTOISO        (1)
#define STEP_AUTOISO       (1)

#define MIN_3ALOCK        (0)
#define MAX_3ALOCK        (3)
#define STEP_3ALOCK       (1)

#define NRELEM_3APARAM     (9)
#define MIN_3APARAM        (0)
#define MAX_3APARAM        (255)
#define STEP_3APARAM       (1)

#define MIN_3ASTATUS        (0)
#define MAX_3ASTATUS        (3)
#define STEP_3ASTATUS       (1)

#define MIN_JPGQUALITY        (1)
#define MAX_JPGQUALITY        (100)
#define STEP_JPGQUALITY       (1)

#define NRELEM_CLIP           IMGSENSOR_CLIP_NELEM
#define MIN_CLIP              (0)
#define MAX_CLIP              (1280)
#define STEP_CLIP             (8)

#endif /* __DRIVERS_VIDEO_ISX019_RANGE_H */
