/****************************************************************************
 * libs/libc/stdlib/lib_ptsnamer.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef CONFIG_PSEUDOTERM_SUSV1

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ptsname_r
 *
 * Description:
 *   The ptsname_r() function is the reentrant equivalent of ptsname().
 *   It returns the name of the slave pseudoterminal device as a null-
 *   terminated string in the buffer pointed to by buf.  The buflen
 *   argument specifies the number of bytes available in buf.
 *
 * Returned Value:
 *   On success, ptsname_r() returns 0.  On failure, a nonzero value is
 *   returned and errno is set to indicate the error.
 *
 *     EINVAL (ptsname_r() only) buf is NULL.
 *     ENOTTY fd does not refer to a pseudoterminal master device.
 *     ERANGE (ptsname_r() only) buf is too small.
 *
 ****************************************************************************/

int ptsname_r(int fd, FAR char *buf, size_t buflen)
{
  int ptyno;
  int ret;

  DEBUGASSERT(buf != NULL);

  /* Get the slave PTY number */

  ret = ioctl(fd, TIOCGPTN, (unsigned long)((uintptr_t)&ptyno));
  if (ret < 0)
    {
      return ret;
    }

  /* Create the device name.  This current does not handler EINVAL or ERANGE
   * error detection.
   */

  snprintf(buf, buflen, "/dev/pts/%d", ptyno);
  return OK;
}

#endif /* CONFIG_PSEUDOTERM_SUSV1 */
