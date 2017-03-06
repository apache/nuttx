/****************************************************************************
 * fs/vfs/fs_read.c
 *
 *   Copyright (C) 2007-2009, 2012-2014, 2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/cancelpt.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_read
 *
 * Description:
 *   This is the internal implementation of read().
 *
 * Parameters:
 *   file     File structure instance
 *   buf      User-provided to save the data
 *   nbytes   The maximum size of the user-provided buffer
 *
 * Return:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or -1 on failure with errno set appropriately.
 *
 ****************************************************************************/

ssize_t file_read(FAR struct file *filep, FAR void *buf, size_t nbytes)
{
  FAR struct inode *inode;
  int ret = -EBADF;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  /* Was this file opened for read access? */

  if ((filep->f_oflags & O_RDOK) == 0)
    {
      /* No.. File is not read-able */

      ret = -EACCES;
    }

  /* Is a driver or mountpoint registered? If so, does it support the read
   * method?
   */

  else if (inode != NULL && inode->u.i_ops && inode->u.i_ops->read)
    {
      /* Yes.. then let it perform the read.  NOTE that for the case of the
       * mountpoint, we depend on the read methods being identical in
       * signature and position in the operations vtable.
       */

      ret = (int)inode->u.i_ops->read(filep, (FAR char *)buf, (size_t)nbytes);
    }

  /* If an error occurred, set errno and return -1 (ERROR) */

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  /* Otherwise, return the number of bytes read */

  return ret;
}

/****************************************************************************
 * Name: read
 *
 * Description:
 *   The standard, POSIX read interface.
 *
 * Parameters:
 *   file     File structure instance
 *   buf      User-provided to save the data
 *   nbytes   The maximum size of the user-provided buffer
 *
 * Return:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or -1 on failure with errno set appropriately.
 *
 ****************************************************************************/

ssize_t read(int fd, FAR void *buf, size_t nbytes)
{
  ssize_t ret;

  /* read() is a cancellation point */

  (void)enter_cancellation_point();

  /* Did we get a valid file descriptor? */

#if CONFIG_NFILE_DESCRIPTORS > 0
  if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
#endif
    {
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
      /* No.. If networking is enabled, read() is the same as recv() with
       * the flags parameter set to zero.  Note that recv() sets
       * the errno variable.
       */

      ret = recv(fd, buf, nbytes, 0);
#else
      /* No networking... it is a bad descriptor in any event */

      set_errno(EBADF);
      ret = ERROR;
#endif
    }

#if CONFIG_NFILE_DESCRIPTORS > 0
  else
    {
      FAR struct file *filep;

      /* The descriptor is in a valid range to file descriptor... do the
       * read.  First, get the file structure.  Note that on failure,
       * fs_getfilep() will set the errno variable.
       */

      filep = fs_getfilep(fd);
      if (filep == NULL)
        {
          /* The errno value has already been set */

          ret = ERROR;
        }
      else
        {
          /* Then let file_read do all of the work.  Note that file_read()
           * sets the errno variable.
           */

          ret = file_read(filep, buf, nbytes);
        }
    }
#endif

  leave_cancellation_point();
  return ret;
}
