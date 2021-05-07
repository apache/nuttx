/****************************************************************************
 * video/videomode/videomode_dump.c
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
