/****************************************************************************
 * libc/time/lib_asctimer.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdio.h>
#include <time.h>

#ifdef CONFIG_TIME_EXTENDED

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Note: These strings duplicate other definitions in other files.  These
 * definitions could be combined to save a little FLASH space.
 */

static const char * const g_wday_name[7] =
{
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

static const char * const g_mon_name[12] =
{
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  asctime_r
 *
 * Description:
 *   asctime and asctime_r convert the time provided in a struct tm to a
 *   string representation.  asctime is not re-entrant; asctime_r is re-
 *   entrant.
 *
 * Parameters:
 *   tp  - Pointer to the time to be converted.
 *   buf - A user provided buffer to receive the 26 character time string.
 *
 * Return Value:
 *   One success, the pointer to the 'buf' is returned; on failure, NULL is
 *   returned.
 *
 ****************************************************************************/

FAR char *asctime_r(FAR const struct tm *tp, FAR char *buf)
{
  snprintf(buf, 26, "%.3s %.3s%3d %.2d:%.2d:%.2d %d\n",
           g_wday_name[tp->tm_wday], g_mon_name[tp->tm_mon],
           tp->tm_mday, tp->tm_hour, tp->tm_min, tp->tm_sec,
           1900 + tp->tm_year);

  return buf;
}

#endif /* CONFIG_TIME_EXTENDED */
