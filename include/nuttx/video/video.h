/****************************************************************************
 * include/nuttx/video/video.h
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
 * The definitions and prototypes in this file are created with reference to
 * the FreeBSD V4L2 driver header at: https://github.com/freebsd/freebsd-src
 * /sys/contrib/v4l/videodev2.h
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_VIDEO_VIDEO_H
#define __INCLUDE_NUTTX_VIDEO_VIDEO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/videoio.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIDEO_HSIZE_QVGA        (320)   /* QVGA    horizontal size */
#define VIDEO_VSIZE_QVGA        (240)   /* QVGA    vertical   size */
#define VIDEO_HSIZE_VGA         (640)   /* VGA     horizontal size */
#define VIDEO_VSIZE_VGA         (480)   /* VGA     vertical   size */
#define VIDEO_HSIZE_QUADVGA     (1280)  /* QUADVGA horizontal size */
#define VIDEO_VSIZE_QUADVGA     (960)   /* QUADVGA vertical   size */
#define VIDEO_HSIZE_HD          (1280)  /* HD      horizontal size */
#define VIDEO_VSIZE_HD          (720)   /* HD      vertical   size */
#define VIDEO_HSIZE_FULLHD      (1920)  /* FULLHD  horizontal size */
#define VIDEO_VSIZE_FULLHD      (1080)  /* FULLHD  vertical   size */
#define VIDEO_HSIZE_5M          (2560)  /* 5M      horizontal size */
#define VIDEO_VSIZE_5M          (1920)  /* 5M      vertical   size */
#define VIDEO_HSIZE_3M          (2048)  /* 3M      horizontal size */
#define VIDEO_VSIZE_3M          (1536)  /* 3M      vertical   size */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct imgdata_s;
struct imgsensor_s;

/* Initialize video driver.
 *
 *  param [in] devpath: path to video device
 *
 *  Return on success, 0 is returned. On failure,
 *  negative value is returned.
 */

int video_initialize(FAR const char *devpath);

/* Uninitialize video driver.
 *
 *  Return on success, 0 is returned. On failure,
 *  negative value is returned.
 */

int video_uninitialize(FAR const char *devpath);

/* New API to register video driver.
 *
 *  param [in] devpath: path to video device
 *  param [in] data: provide imgdata ops
 *  param [in] sensor: provide imgsensor ops array
 *  param [in] sensor_num: the number of imgsensor ops array
 *
 *  Return on success, 0 is returned. On failure,
 *  negative value is returned.
 */

int video_register(FAR const char *devpath,
                   FAR struct imgdata_s *data,
                   FAR struct imgsensor_s **sensors,
                   size_t sensor_num);

/* New API to Unregister video driver.
 *
 *  param [in] devpath: path to video device
 *
 *  Return on success, 0 is returned. On failure,
 *  negative value is returned.
 */

int video_unregister(FAR const char *devpath);

#ifdef __cplusplus
}
#endif
#endif /* __INCLUDE_NUTTX_VIDEO_VIDEO_H */
