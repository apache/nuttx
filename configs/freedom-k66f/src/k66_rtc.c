/****************************************************************************
 * config/freedom-k66f/src/k66_bringup.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <syslog.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/timers/rtc.h>

#include "freedom-k66f.h"
#include "kinetis_alarm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: k66_rtc_initialize
 *
 * Description:
 *   Called to initialize the RTC FREEDOM-K66F board.
 *
 ************************************************************************************/

int k66_rtc_initialize(void)
{
  int ret = -EIO;
  FAR struct rtc_lowerhalf_s *lower;

  /* Instantiate the KINETIS lower-half RTC driver */

  lower = kinetis_rtc_lowerhalf();
  if (!lower)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to instantiate the RTC lower-half driver\n");
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);

      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind/register the RTC driver: %d\n",
                 ret);
        }
    }
  return ret;
}
