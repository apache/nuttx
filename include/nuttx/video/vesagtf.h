/**************************************************************************7
 * include/nuttx/video/vesagtf.h
 * EDID (Extended Display Identification Data) Format
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic in FreeBSD which has an equivalent 3-clause BSD
 * license:
 *
 *   Copyright (c) 2006 Itronix Inc. All rights reserved.
 *   Written by Garrett D'Amore for Itronix Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_VIDEO_VESAGTF
#define __INCLUDE_NUTTX_VIDEO_VESAGTF

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

void vesagtf_mode_params(unsigned int x, unsigned int y, unsigned int refresh,
                         FAR struct vesagtf_params *params,
                         unsigned int flags,
                         FAR struct videomode_s *videomode);

#endif /* __INCLUDE_NUTTX_VIDEO_VESAGTF */
