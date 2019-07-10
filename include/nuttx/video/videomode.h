/****************************************************************************
 * include/nuttx/video/videomode.h
 * EDID (Extended Display Identification Data) Format
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Some of structures in this file derive from FreeBSD which has a
 * compatible 2-clause BSD license:
 *
 *  Copyright (c) 2006 Itronix Inc. All rights reserved.
 *  Written by Garrett D'Amore for Itronix Inc.
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
