/****************************************************************************
 * include/nuttx/video/videomode.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_VIDEOMODE_H
#define __INCLUDE_NUTTX_VIDEO_VIDEOMODE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Video mode flags used in struct hdmi_videomode_s */

#define VID_PHSYNC                        (1 << 0)
#define VID_NHSYNC                        (1 << 1)
#define VID_PVSYNC                        (1 << 2)
#define VID_NVSYNC                        (1 << 3)
#define VID_INTERLACE                     (1 << 4)
#define VID_DBLSCAN                       (1 << 5)
#define VID_CSYNC                         (1 << 6)
#define VID_PCSYNC                        (1 << 7)
#define VID_NCSYNC                        (1 << 8)
#define VID_HSKEW                         (1 << 9)
#define VID_BCAST                         (1 << 10)
#define VID_PIXMUX                        (1 << 12)
#define VID_DBLCLK                        (1 << 13)
#define VID_CLKDIV2                       (1 << 14)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This structure represents one video mode extracted from the EDID.
 * CAREFUL:  Fields  may not change without also modification to initializer
 * in videomode_lookup.c.
 */

struct videomode_s
{
  uint32_t dotclock;     /* Dot clock frequency in kHz. */
  uint16_t hdisplay;
  uint16_t hsync_start;
  uint16_t hsync_end;
  uint16_t htotal;
  uint16_t vdisplay;
  uint16_t vsync_start;
  uint16_t vsync_end;
  uint16_t vtotal;
  uint16_t hskew;
  uint16_t flags;        /* Video mode flags; see above. */
  FAR const char *name;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  sort_videomodes
 *
 * Description:
 *   Sort video modes by refresh rate, aspect ratio, then resolution.
 *   Preferred mode or largest mode is first in the list and other modes
 *   are sorted on closest match to that mode.
 *
 *   Note that the aspect ratio calculation treats "close" aspect ratios
 *   (within 12.5%) as the same for this purpose.
 *
 * Input Parameters:
 *   modes     - A reference to the first entry in a list of video modes
 *   preferred - A pointer to the pointer to the preferred mode in the list
 *   nmodes    - The number of modes in the list
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sort_videomodes(FAR struct videomode_s *modes,
                     FAR struct videomode_s **preferred,
                     unsigned int nmodes);

/****************************************************************************
 * Name:  videomode_lookup
 *
 * Description:
 *   Find the video mode in a look-up table
 *
 ****************************************************************************/

FAR const struct videomode_s *videomode_lookup_by_name(FAR const char *name);

/****************************************************************************
 * Name:  videomode_lookup_by_dotclock
 *
 * Description:
 *   Find the video mode in a look-up table with the matching width and
 *   height and the closest dot clock that does not exceed the requested
 *   dot clock.
 *
 ****************************************************************************/

#if 0 /* Not used */
FAR const struct videomode_s *
  videomode_lookup_by_dotclock(uint16_t width, uint16_t height,
                               uint32_t dotclock);
#endif

/****************************************************************************
 * Name:  videomode_lookup_by_refresh
 *
 * Description:
 *   Find the video mode in a look-up table with the matching width and
 *   height and the closest refresh rate that does not exceed the requested
 *   rate.
 *
 ****************************************************************************/

#if 0 /* Not used */
FAR const struct videomode_s *
  videomode_lookup_by_refresh(uint16_t width, uint16_t height,
                              uint16_t refresh);
#endif

/****************************************************************************
 * Name:  videomode_dump
 *
 * Description:
 *   Dump the content of a video mode one one line to the SYSLOG.
 *
 * Input Parameters:
 *   prefix    - A string to print at the beginning of the line.  May be
 *               NULL
 *   videomode - The videomode to be dumped
 *   terse     - True:  Print only a minimal amount of data, sufficient to
 *               identify the video mode.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void videomode_dump(FAR const char *prefix,
                    FAR const struct videomode_s *videomode, bool terse);

#endif /* __INCLUDE_NUTTX_VIDEO_VIDEOMODE_H */
