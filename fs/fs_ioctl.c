/************************************************************
 * fs_ioctl.c
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
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include "fs_internal.h"

/************************************************************
 * Global Functions
 ************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0

int ioctl(int fd, int req, unsigned long arg)
{
  FAR struct filelist *list;
  int ret = EBADF;

  /* Get the thread-specific file list */

  list = sched_getfiles();
  if (!list)
    {
      *get_errno_ptr() = EMFILE;
      return ERROR;
    }

  /* Were we give a valid file descriptor? */

  if ((unsigned int)fd < CONFIG_NFILE_DESCRIPTORS)
    {
      FAR struct file *this_file = &list->fl_files[fd];
      struct inode *inode        = this_file->f_inode;

      /* Is a driver registered? Does it support the ioctl method? */

      if (inode && inode->i_ops && inode->i_ops->ioctl)
	{
	  /* Yes, then let it perform the ioctl */

	  ret = (int)inode->i_ops->ioctl(this_file, req, arg);
	}
    }
  return ret;
}

#endif /* CONFIG_NFILE_DESCRIPTORS */
