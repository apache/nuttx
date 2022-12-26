/****************************************************************************
 * include/nuttx/video/imgdata.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_IMGDATA_H
#define __INCLUDE_NUTTX_VIDEO_IMGDATA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Format definition for start_capture() and validate_frame_setting */

#define IMGDATA_FMT_MAX                  (2)
#define IMGDATA_FMT_MAIN                 (0)
#define IMGDATA_FMT_SUB                  (1)
#define IMGDATA_PIX_FMT_UYVY             (0)
#define IMGDATA_PIX_FMT_RGB565           (1)
#define IMGDATA_PIX_FMT_JPEG             (2)
#define IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG (3)
#define IMGDATA_PIX_FMT_SUBIMG_UYVY      (4)
#define IMGDATA_PIX_FMT_SUBIMG_RGB565    (5)
#define IMGDATA_PIX_FMT_YUYV             (6)
#define IMGDATA_PIX_FMT_YUV420P          (7)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Structure for validate_frame_setting() and start_capture() */

typedef struct imgdata_format_s
{
  uint16_t width;
  uint16_t height;
  uint32_t pixelformat;
} imgdata_format_t;

typedef struct imgdata_interval_s
{
  uint32_t numerator;
  uint32_t denominator;
} imgdata_interval_t;

typedef int (*imgdata_capture_t)(uint8_t result, uint32_t size);

/* Structure for Data Control I/F */

struct imgdata_ops_s
{
  CODE int (*init)(void);
  CODE int (*uninit)(void);

  CODE int (*set_buf)(uint8_t *addr, uint32_t size);

  CODE int (*validate_frame_setting)(uint8_t nr_datafmts,
                                     FAR imgdata_format_t *datafmts,
                                     FAR imgdata_interval_t *interval);
  CODE int (*start_capture)(uint8_t nr_datafmts,
                            FAR imgdata_format_t *datafmts,
                            FAR imgdata_interval_t *interval,
                            FAR imgdata_capture_t callback);
  CODE int (*stop_capture)(void);
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

/* Register image data operations. */

void imgdata_register(FAR const struct imgdata_ops_s *ops);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_IMGDATA_H */
