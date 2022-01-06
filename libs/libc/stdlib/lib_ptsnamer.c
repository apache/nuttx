/****************************************************************************
 * libs/libc/stdlib/lib_ptsnamer.c
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

#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef CONFIG_PSEUDOTERM

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

#ifdef CONFIG_PSEUDOTERM_SUSV1
  snprintf(buf, buflen, "/dev/pts/%d", ptyno);
#else
  snprintf(buf, buflen, "/dev/ttyp%d", ptyno);
#endif

  return OK;
}

#endif /* CONFIG_PSEUDOTERM */
