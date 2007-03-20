/************************************************************
 * fs_closedir.c
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
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdlib.h>
#include <dirent.h>
#include <errno.h>
#include <nuttx/fs.h>
#include "fs_internal.h"

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Name: seekdir
 *
 * Description:
 *    The closedir() function closes the directory stream 
 *    associated with 'dirp'.  The directory stream
 *    descriptor 'dirp' is not available after this call.
 *
 * Inputs:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *
 * Return:
 *   The closedir() function returns 0 on success.  On error,
 *   -1 is returned, and errno is set appropriately.
 *
 ************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0

int closedir(FAR DIR *dirp)
{
  struct internal_dir_s *idir = (struct internal_dir_s *)dirp;

  if (!idir)
    {
      *get_errno_ptr() = EBADF;
      return ERROR;
    }

  /* Release our references on the contained inodes */

  if (idir->root)
    {
      inode_release(idir->root);
    }

  if (idir->next)
    {
      inode_release(idir->next);
    }

  /* Then release the container */

  free(idir);
  return OK;
}

#endif /* CONFIG_NFILE_DESCRIPTORS */
