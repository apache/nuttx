/****************************************************************************
 * include/nuttx/video/v4l2_cap.h
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
 ****************************************************************************/

#ifndef __NUTTX_VIDEO_V4L2_CAP_H
#define __NUTTX_VIDEO_V4L2_CAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize capture driver.
 *
 *  param [in] devpath: path to capture device
 *
 *  Return on success, 0 is returned. On failure,
 *  negative value is returned.
 */

int capture_initialize(FAR const char *devpath);

/* Uninitialize capture driver.
 *
 *  Return on success, 0 is returned. On failure,
 *  negative value is returned.
 */

int capture_uninitialize(FAR const char *devpath);

/* New API to register capture driver.
 *
 *  param [in] devpath: path to capture device
 *  param [in] data: provide imgdata ops
 *  param [in] sensor: provide imgsensor ops array
 *  param [in] sensor_num: the number of imgsensor ops array
 *
 *  Return on success, 0 is returned. On failure,
 *  negative value is returned.
 */

int capture_register(FAR const char *devpath,
                     FAR struct imgdata_s *data,
                     FAR struct imgsensor_s **sensors,
                     size_t sensor_num);

/* New API to Unregister capture driver.
 *
 *  param [in] devpath: path to capture device
 *
 *  Return on success, 0 is returned. On failure,
 *  negative value is returned.
 */

int capture_unregister(FAR const char *devpath);

#ifdef __cplusplus
}
#endif

#endif /* __NUTTX_VIDEO_V4L2_CAP_H */
