/****************************************************************************
 * include/nuttx/video/video_fake.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_VIDEO_FAKE_H
#define __INCLUDE_NUTTX_VIDEO_VIDEO_FAKE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

#include <nuttx/compiler.h>

#ifdef CONFIG_VIDEO_FAKE

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

struct video_fake_size_s
{
  uint16_t width;
  uint16_t height;
};

typedef struct video_fake_size_s video_fake_size_t;

/* Each list is zero terminated, and zero is not a usable entry for any of
 * them: a format must be a real V4L2 fourcc, a size must have both sides
 * non-zero, and a frame rate must be non-zero.  That lets a configuration
 * be written as a plain initializer with no separate count.
 *
 * The terminator needs room of its own, so each array holds one more entry
 * than the configuration may use: a list that fills the array without a
 * terminator is a configuration error the driver reports instead of one it
 * silently truncates.
 */

#define VIDEO_FAKE_FORMATS_MAX  8
#define VIDEO_FAKE_SIZES_MAX    8
#define VIDEO_FAKE_FPS_MAX      8

struct video_fake_params_s
{
  FAR const char   *devpath;
  uint32_t          formats[VIDEO_FAKE_FORMATS_MAX + 1];
  video_fake_size_t sizes[VIDEO_FAKE_SIZES_MAX + 1];
  uint32_t          fps[VIDEO_FAKE_FPS_MAX + 1];
};

typedef struct video_fake_params_s video_fake_params_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: video_fake_initialize
 *
 * Description:
 *   Register the fake video devices configured by
 *   CONFIG_VIDEO_FAKE_DEVICE_PARAMS.
 *
 * Returned Value:
 *   The number of devices registered, or a negated errno value if every
 *   device failed to register.
 *
 ****************************************************************************/

int video_fake_initialize(void);

/****************************************************************************
 * Name: video_fake_register
 *
 * Description:
 *   Register independent software-only V4L2 capture devices.
 *
 * Input Parameters:
 *   params  - Fake video parameter array.
 *   nparams - Number of entries in params.
 *
 * Returned Value:
 *   The number of devices registered, or a negated errno value if every
 *   device failed to register.
 *
 ****************************************************************************/

int video_fake_register(FAR const video_fake_params_t *params,
                        size_t nparams);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_VIDEO_FAKE */

#endif /* __INCLUDE_NUTTX_VIDEO_VIDEO_FAKE_H */
