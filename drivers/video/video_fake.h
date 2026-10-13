/****************************************************************************
 * drivers/video/video_fake.h
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

#ifndef __DRIVERS_VIDEO_VIDEO_FAKE_H
#define __DRIVERS_VIDEO_VIDEO_FAKE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/v4l2_cap.h>
#include <nuttx/video/video.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_VIDEO_FAKE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIDEO_FAKE_BARS               8
#define VIDEO_FAKE_PANEL_LINES        4

/* Match the camera HAL mock's CP437 8x8 bitmap without pulling its C++ code
 * or a runtime font library into NuttX.
 *
 * Keep gaps between characters and lines so the four parameter rows remain
 * readable at the smallest frame size.
 *
 * The panel is then sized to the text rather than to a fixed box: width is
 * the widest line this instance can print plus padding on each side, and
 * height spans (LINES - 1) line heights plus one glyph height, likewise
 * padded.  Text is left-aligned, so the horizontal padding is what keeps it
 * off the left edge.
 */

#define VIDEO_FAKE_FONT_WIDTH         8
#define VIDEO_FAKE_FONT_HEIGHT        8
#define VIDEO_FAKE_FONT_ADVANCE       9
#define VIDEO_FAKE_LINE_HEIGHT        14
#define VIDEO_FAKE_PANEL_PAD_X        5
#define VIDEO_FAKE_PANEL_PAD_Y        5
#define VIDEO_FAKE_DYNAMIC_LINE       2
#define VIDEO_FAKE_REFERENCE_WIDTH    640
#define VIDEO_FAKE_REFERENCE_HEIGHT   480
#define VIDEO_FAKE_SCALE_SHIFT        16
#define VIDEO_FAKE_SCALE_ONE          (1 << VIDEO_FAKE_SCALE_SHIFT)

#define VIDEO_FAKE_MAX_BYTES_PIXEL    2
#define VIDEO_FAKE_YUV_SIZE_FACTOR    3

/* Camera control shadow registers.  Single element controls are indexed by
 * video_fake_control_index(), the array controls have a row each.
 */

#define VIDEO_FAKE_CONTROL_COUNT      41
#define VIDEO_FAKE_GAMMA_CURVE_NELEM  16
#define VIDEO_FAKE_3A_PARAMETER_NELEM 8

#define VIDEO_FAKE_NAME_SIZE          32
#define VIDEO_FAKE_LINE_SIZE          32
#define VIDEO_FAKE_SLOT_COUNT         (CONFIG_VIDEO_REQBUFS_COUNT_MAX + 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct video_fake_slot_s
{
  FAR uint8_t *addr;
  uint32_t background_generation;
  uint64_t recency;
};

struct video_fake_counts_s
{
  size_t formats;
  size_t sizes;
  size_t rates;
  uint16_t max_width;
  uint16_t max_height;
};

struct video_fake_s
{
  struct imgdata_s data; /* Must remain first for data-op casts. */
  struct imgsensor_s sensor;

  /* Capability tables sized from the instance's own configuration. */

  FAR struct v4l2_fmtdesc *fmtdescs;
  FAR struct v4l2_frmsizeenum *frmsizes;
  FAR struct v4l2_frmivalenum *frmintervals;
  size_t formats_num;
  size_t sizes_num;
  size_t rates_num;
  char name[VIDEO_FAKE_NAME_SIZE];

  /* Panel geometry follows the active stream size, so it is recomputed when
   * a stream starts and reused for every buffer in that stream.
   */

  uint32_t panel_width;
  uint32_t panel_height;
  uint32_t font_width;
  uint32_t font_height;
  uint32_t font_advance;
  uint32_t line_height;
  uint32_t panel_pad_x;
  uint32_t panel_pad_y;
  FAR uint8_t *source_rows;
  FAR uint8_t *source_columns;

  FAR uint8_t *background_row;
  uint32_t background_row_generation;
  struct video_fake_slot_s slots[VIDEO_FAKE_SLOT_COUNT];
  uint64_t recency;

  spinlock_t lock;
  bool running;
  imgdata_capture_t callback;
  FAR void *callback_arg;
  FAR uint8_t *next_buffer;
  uint32_t buffer_size;
  uint32_t generation;

  struct work_s work;
  uint16_t width;
  uint16_t height;
  uint32_t pixel_format;
  uint32_t frame_bytes;
  uint32_t fps;
  uint64_t tick_num;
  uint64_t tick_den;
  uint64_t tick_acc;
  clock_t next_tick;
  clock_t uptime_base;
  uint32_t frame;

  /* A control reports what set_value() last accepted, so each one keeps its
   * own storage rather than a constant.  The array controls need a row each
   * because the capture GET path reads them through a pointer instead of
   * copying them back itself.
   */

  int32_t  controls[VIDEO_FAKE_CONTROL_COUNT];
  uint16_t gamma_curve[VIDEO_FAKE_GAMMA_CURVE_NELEM];
  uint8_t  threea_parameter[VIDEO_FAKE_3A_PARAMETER_NELEM];
  uint32_t clip_video[IMGSENSOR_CLIP_NELEM];
  uint32_t clip_still[IMGSENSOR_CLIP_NELEM];
};

#endif /* CONFIG_VIDEO_FAKE */

#endif /* __DRIVERS_VIDEO_VIDEO_FAKE_H */
