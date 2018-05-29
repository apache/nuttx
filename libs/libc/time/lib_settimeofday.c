/****************************************************************************
 * libs/libc/time/lib_settimeofday.c
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

#include <sys/time.h>
#include <errno.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: settimeofday
 *
 * Description:
 *   Set the current time
 *
 *   Conforming to SVr4, 4.3BSD. POSIX.1-2001 describes gettimeofday() but
 *   not settimeofday().
 *
 *   NuttX implements settimeofday() as a thin layer around clock_settime();
 *
 * Input Parameters:
 *   tv - The net to time to be set
 *   tz - Ignored
 *
 * Returned Value:
 *   Zero (OK) on success;  -1 is returned on failure with the errno variable
 *   set appropriately.
 *
 ****************************************************************************/

int settimeofday(FAR const struct timeval *tv, FAR struct timezone *tz)
{
  struct timespec ts;

#ifdef CONFIG_DEBUG_FEATURES
  if (!tv || tv->tv_usec >= USEC_PER_SEC)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Convert the timeval to a timespec */

  ts.tv_sec  = tv->tv_sec;
  ts.tv_nsec = tv->tv_usec * NSEC_PER_USEC;

  /* Let clock_settime do the work */

  return clock_settime(CLOCK_REALTIME, &ts);
}
