/************************************************************
 * fs_readdir.c
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
#include <string.h>
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
 * Name: readdir
 *
 * Description:
 *   The readdir() function returns a pointer to a dirent
 *   structure representing the next directory entry in the
 *   directory stream pointed to by dir.  It returns NULL on
 *   reaching the end-of-file or if an error occurred.
 *
 * Inputs:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *
 * Return:
 *   The readdir() function returns a pointer to a dirent
 *   structure, or NULL if an error occurs or end-of-file
 *   is reached.  On error, errno is set appropriately.
 *
 *   EBADF   - Invalid directory stream descriptor dir
 *
 ************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0

FAR struct dirent *readdir(DIR *dirp)
{
  FAR struct internal_dir_s *idir = (struct internal_dir_s *)dirp;
  FAR struct inode *prev;

  if (!idir)
    {
      *get_errno_ptr() = EBADF;
      return NULL;
    }

  /* Check if we are at the end of the list */

  if (!idir->next)
    {
      return NULL;
    }

  /* Copy the inode name into the dirent structure */

  strncpy(idir->dir.d_name, idir->next->i_name, NAME_MAX+1);

  /* If the node has file operations, we will say that it is
   * a file.
   */

  idir->dir.d_type = 0;
  if (idir->next->i_ops)
    {
      idir->dir.d_type |= DTYPE_FILE;
    }

  /* If the node has child node(s), then we will say that it
   * is a directory.  NOTE: that the node can be both!
   */

  if (idir->next->i_child || !idir->next->i_ops)
    {
      idir->dir.d_type |= DTYPE_DIRECTORY;
    }

  /* Now get the inode to vist next time that readdir() is called */

  inode_semtake();

  prev       = idir->next;
  idir->next = prev->i_peer; /* The next node to visit */

  if (idir->next)
    {
      /* Increment the reference count on this next node */

      idir->next->i_crefs++;
    }

  inode_semgive();

  if (prev)
    {
      inode_release(prev);
    }

  return &idir->dir;
}

#endif /* CONFIG_NFILE_DESCRIPTORS */
