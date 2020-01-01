/****************************************************************************
 * fs/inode/fs_fileopen.c
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <fcntl.h>
#include <stdarg.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_open
 *
 * Description:
 *   file_open() is similar to the standard 'open' interface except that it
 *   returns an instance of 'struct file' rather than a file descriptor.  It
 *   also is not a cancellation point and does not modify the errno variable.
 *
 * Input Parameters:
 *   filep  - The caller provided location in which to return the 'struct
 *            file' instance.
 *   path   - The full path to the file to be open.
 *   oflags - open flags
 *   ...    - Variable number of arguments, may include 'mode_t mode'
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  On failure, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

int file_open(FAR struct file *filep, FAR const char *path, int oflags, ...)
{
  va_list ap;
  int ret;
  int fd;

  DEBUGASSERT(filep != NULL && path != NULL);

  /* At present, this is just a placeholder.  It is just a wrapper around
   * nx_open() followed by a called to file_detach().  Ideally, this should
   * a native open function that opens the VFS node directly without using
   * any file descriptors.
   */

  va_start(ap, oflags);
  fd = nx_vopen(path, oflags, ap);
  va_end(ap);

  if (fd < 0)
    {
      return fd;
    }

  /* Detach the file structure from the file descriptor so that it can be
   * used on any thread.
   */

  ret = file_detach(fd, filep);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  return OK;
}
