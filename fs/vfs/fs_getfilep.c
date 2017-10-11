/****************************************************************************
 * fs/vfs/fs_getfilep.c
 *
 *   Copyright (C) 2014, 2016-2017 Gregory Nutt. All rights reserved.
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
#include <sys/socket.h>

#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs_getfilep
 *
 * Description:
 *   Given a file descriptor, return the corresponding instance of struct
 *   file.  NOTE that this function will currently fail if it is provided
 *   with a socket descriptor.
 *
 * Input Parameters:
 *   fd    - The file descriptor
 *   filep - The location to return the struct file instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int fs_getfilep(int fd, FAR struct file **filep)
{
  FAR struct filelist *list;

  DEBUGASSERT(filep != NULL);
  *filep = (FAR struct file *)NULL;

  if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
    {
      return -EBADF;
    }

  /* The descriptor is in a valid range to file descriptor... Get the
   * thread-specific file list.
   */

  list = sched_getfiles();

  /* The file list can be NULL under two cases:  (1) One is an obscure
   * cornercase:  When memory management debug output is enabled.  Then
   * there may be attempts to write to stdout from malloc before the group
   * data has been allocated.  The other other is (2) if this is a kernel
   * thread.  Kernel threads have no allocated file descriptors.
   */

  if (list == NULL)
    {
      return -EAGAIN;
    }

  /* And return the file pointer from the list */

  *filep = &list->fl_files[fd];
  return OK;
}
