/************************************************************
 * fs_seekdir.c
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
#include <dirent.h>
#include <errno.h>
#include <nuttx/fs.h>
#include "fs_internal.h"

/************************************************************
 * Private Functions
 ************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0

static inline void seekpsuedodir(struct internal_dir_s *idir, off_t offset)
{
  struct inode *curr;
  struct inode *prev;
  off_t pos;

  /* Determine a starting point for the seek.  If the seek
   * is "forward" from the current position, then we will
   * start at the current poisition.  Otherwise, we will
   * "rewind" to the root dir.
   */

  if ( offset < idir->position )
    {
       pos  = 0;
       curr = idir->root;
    }
  else
    {
       pos  = idir->position;
       curr = idir->u.psuedo.next;
    }

  /* Traverse the peer list starting at the 'root' of the
   * the list until we find the node at 'offset".  If devices
   * are being registered and unregistered, then this can
   * be a very unpredictable operation.
   */

  inode_semtake();
  for (; curr && pos != offset; pos++, curr = curr->i_peer);

  /* Now get the inode to vist next time that readdir() is called */

  prev                = idir->u.psuedo.next;
  idir->u.psuedo.next = curr; /* The next node to visit (might be null) */
  idir->position      = pos;  /* Might be beyond the last dirent */

  if (curr)
    {
      /* Increment the reference count on this next node */

      curr->i_crefs++;
    }

  inode_semgive();

  if (prev)
    {
      inode_release(prev);
    }
}

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Name: seekdir
 *
 * Description:
 *   The seekdir() function sets the location in the
 *   directory stream from which the next readdir() call will
 *   start.  seekdir() should be used with an offset returned
 *   by telldir().
 *
 * Inputs:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *   offset -- offset to seek to
 *
 * Return:
 *   None
 *
 ************************************************************/

void seekdir(FAR DIR *dirp, off_t offset)
{
  struct internal_dir_s *idir = (struct internal_dir_s *)dirp;

  /* Sanity checks */

  if (!idir || !idir->root)
    {
      return;
    }

  /* The way we handle the readdir depends on the type of inode
   * that we are dealing with.
   */

  if (INODE_IS_MOUNTPT(idir->root))
    {
      /* The node is a file system mointpoint */

#warning "Mountpoint support not implemented"
    }
  else
    {
      /* The node is part of the root psuedo file system */

      seekpsuedodir(idir, offset);
    }
}

#endif /* CONFIG_NFILE_DESCRIPTORS */
