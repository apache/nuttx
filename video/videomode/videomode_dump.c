/****************************************************************************
 * video/videomode/videomode_dump.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from logic in FreeBSD which has an compatible 2-clause BSD
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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE NETBSD FOUNDATION BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/video/videomode.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DIVIDE(x,y)     (((x) + ((y) / 2)) / (y))

/****************************************************************************
 * Name:  videomode_refresh
 *
 * Description:
 *   Calculate the refresh rate.
 *
 * Input Parameters:
 *   videomode - The videomode to be dumped
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static uint32_t videomode_refresh(FAR const struct videomode_s *videomode)
{
  return DIVIDE(DIVIDE(videomode->dotclock * 1000, videomode->htotal),
                videomode->vtotal);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
                    FAR const struct videomode_s *videomode, bool terse)
{
  if (videomode != NULL)
    {
      if (prefix != NULL)
        {
          syslog(LOG_INFO, "%s", prefix);
        }

      syslog(LOG_INFO, "%ux%u @ %luHz",
             videomode->hdisplay, videomode->vdisplay,
            (unsigned long)videomode_refresh(videomode));

      if (!terse)
        {
          syslog(LOG_INFO, " (%lu %u %u %u %u %u %u",
                 (unsigned long)videomode->dotclock,
                 videomode->hsync_start,
                 videomode->hsync_end,
                 videomode->htotal,
                 videomode->vsync_start,
                 videomode->vsync_end,
                 videomode->vtotal);
          syslog(LOG_INFO, " %s%sH %s%sV)\n",
                 videomode->flags & VID_PHSYNC ? "+" : "",
                 videomode->flags & VID_NHSYNC ? "-" : "",
                 videomode->flags & VID_PVSYNC ? "+" : "",
                 videomode->flags & VID_NVSYNC ? "-" : "");
        }
    }
}
