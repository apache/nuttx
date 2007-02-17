/************************************************************
 * fs_open.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>

#if CONFIG_NFILE_DESCRIPTORS >0

#include <sys/types.h>

#include <stdio.h>
#include <sched.h>
#include <errno.h>

#ifdef CONFIG_FILE_MODE
#include <stdarg.h>
#endif

#include <nuttx/fs.h>
#include <errno.h>

#include "fs_internal.h"

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

int inode_checkflags(struct inode *inode, int oflags)
{
  if (((oflags & O_RDOK) != 0 && !inode->i_ops->read) ||
      ((oflags & O_WROK) != 0 && !inode->i_ops->write))
    {
      *get_errno_ptr() = EACCES;
      return ERROR;
    }
  else
    {
      return OK;
    }
}

int open(const char *path, int oflags, ...)
{
  struct filelist *list;
  struct inode *inode;
  int status;
  int fd;

  /* Get the thread-specific file list */

  list = sched_getfiles();
  if (!list)
    {
      *get_errno_ptr() = EMFILE;
      return ERROR;
    }

#ifdef CONFIG_FILE_MODE
# warning "File creation not implemented"
  mode_t mode = 0;

  /* If the file is opened for creation, then get the mode bits */

  if (oflags & (O_WRONLY|O_CREAT) != 0)
    {
      va_list ap;
      va_start(ap, oflags);
      mode = va_arg(ap, mode_t);
      va_end(ap);
    }
#endif

  /* Get an inode for this file */

  inode = inode_find(path);
  if (!inode)
    {
      /* "O_CREAT is not set and the named file does not exist.  Or,
       * a directory component in pathname does not exist or is a
       * dangling symbolic link.
       */

      *get_errno_ptr() = ENOENT;
      return ERROR;
    }

  /* Make sure that the inode supports the requested access */

  if (inode_checkflags(inode, oflags) != OK)
    {
      inode_release(inode);
      return ERROR;
    }

  /* Associate the inode with a file structure */

  fd = files_allocate(inode, oflags, 0);
  if (fd < 0)
    {
      inode_release(inode);
      *get_errno_ptr() = EMFILE;
      return ERROR;
    }

  /* Perform the driver open operation */

  status = OK;
  if (inode->i_ops && inode->i_ops->open)
    {
      status = inode->i_ops->open(&list->fl_files[fd]);
    }

  if (status != OK || !inode->i_ops)
    {
      files_release(fd);
      inode_release(inode);
      *get_errno_ptr() = ENODEV;
      return ERROR;
    }

  return fd;
}

#endif /* CONFIG_NFILE_DESCRIPTORS */
