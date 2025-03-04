/****************************************************************************
 * include/nuttx/video/vesagtf.h
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_VIDEO_VESAGTF_H
#define __INCLUDE_NUTTX_VIDEO_VESAGTF_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default values to use for params. */

#define VESAGTF_MARGIN_PPT      18    /* 1.8% */
#define VESAGTF_MIN_PORCH       1     /* minimum front porch */
#define VESAGTF_VSYNC_RQD       3     /* vsync width in lines */
#define VESAGTF_HSYNC_PCT       8     /* width of hsync % of total line */
#define VESAGTF_MIN_VSBP        550   /* min vsync + back porch (usec) */
#define VESAGTF_M               600   /* blanking formula gradient */
#define VESAGTF_C               40    /* blanking formula offset */
#define VESAGTF_K               128   /* blanking formula scaling factor */
#define VESAGTF_J               20    /* blanking formula scaling factor */

#define VESAGTF_FLAG_ILACE      0x0001/* use interlace */
#define VESAGTF_FLAG_MARGINS    0x0002/* use margins */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use VESA GTF formula to generate a monitor mode, given resolution and
 * refresh rates.
 */

struct vesagtf_params
{
  unsigned int margin_ppt;    /* Bertical margin size, percent * 10 think
                               * parts-per-thousand */
  unsigned int min_porch;     /* Minimum front porch */
  unsigned int vsync_rqd;     /* Width of vsync in lines */
  unsigned int hsync_pct;     /* Hsync as % of total width */
  unsigned int min_vsbp;      /* Minimum vsync + back porch (usec) */
  unsigned int m;             /* Blanking formula gradient */
  unsigned int c;             /* Blanking formula offset */
  unsigned int k;             /* Blanking formula scaling factor */
  unsigned int j;             /* Blanking formula scaling factor */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct videomode_s;  /* Forward reference */

/****************************************************************************
 * Name: vesagtf_mode
 *
 * Description:
 *   Use VESA GTF formula to generate monitor timings.  Assumes default
 *   GTF parameters, non-interlaced, and no margins.
 *
 ****************************************************************************/

void vesagtf_mode(unsigned int x, unsigned int y, unsigned int refresh,
                  FAR struct videomode_s *videomode);

/****************************************************************************
 * Name: vesagtf_mode_params
 *
 * Description:
 *   vesagtf_mode_params() - as defined by the GTF Timing Standard, compute
 *   the Stage 1 Parameters using the vertical refresh frequency.  In other
 *   words: input a desired resolution and desired refresh rate, and
 *   output the GTF mode timings.
 *
 ****************************************************************************/

void vesagtf_mode_params(unsigned int x,
                         unsigned int y,
                         unsigned int refresh,
                         FAR struct vesagtf_params *params,
                         unsigned int flags,
                         FAR struct videomode_s *videomode);

#endif /* __INCLUDE_NUTTX_VIDEO_VESAGTF_H */
