/****************************************************************************
 * libc/misc/lib_ioctl.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#include <stdarg.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "lib_internal.h"

#if defined(CONFIG_LIBC_IOCTL_VARIADIC) && CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ioctl/fs_ioctl
 *
 * Description:
 *   Perform device specific operations.
 *
 * Parameters:
 *   fd       File/socket descriptor of device
 *   req      The ioctl command
 *   ...      One argument of type unsigned long is expected
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   -1 on failure withi errno set properly:
 *
 *   EBADF
 *     'fd' is not a valid descriptor.
 *   EFAULT
 *     'arg' references an inaccessible memory area.
 *   EINVAL
 *     'cmd' or 'arg' is not valid.
 *   ENOTTY
 *     'fd' is not associated with a character special device.
 *   ENOTTY
 *      The specified request does not apply to the kind of object that the
 *      descriptor 'fd' references.
 *
 ****************************************************************************/

int ioctl(int fd, int req, ...)
{
  va_list ap;
  unsigned long arg;

  /* Get the unsigned long argument.
   *
   * REVISIT:  This could cause of the crash down the road if the actual size
   * of the argument is anything other than sizeof(unsigned long);
   */

  va_start(ap, req);
  arg = va_arg(ap, unsigned long);
  va_end(ap);

  /* Then let fs_ioctl() to the real work */

  return fs_ioctl(fd, req, arg);
}

#endif /* CONFIG_LIBC_IOCTL_VARIADIC && CONFIG_NFILE_DESCRIPTORS > 0 */
