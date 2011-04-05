/****************************************************************************
 * fs/fs_fdopen.c
 *
 *   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs_fdopen
 ****************************************************************************/

FAR struct file_struct *fs_fdopen(int fd, int oflags, FAR _TCB *tcb)
{
  FAR struct filelist   *flist;
  FAR struct streamlist *slist;
  FAR struct inode      *inode;
  FAR FILE              *stream;
  int                    err = OK;
  int                    ret;
  int                    i;

  /* Check input parameters */

  if (fd < 0)
    {
      err = EBADF;
      goto errout;
    }

  /* A NULL TCB pointer means to use this threads TCB.  This is a little
   * hack the let's this function be called from user-space (via a syscall)
   * without having access to the TCB.
   */

  if (!tcb)
    {
      tcb = sched_self();
    }

  /* Get the file and stream list from the TCB */

  flist = tcb->filelist;
  slist = tcb->streams;

  /* Get the inode associated with the file descriptor.  This should
   * normally be the case if fd >= 0.  But not in the case where the
   * called attempts to explictly stdin with fdopen(0) but stdin has
   * been closed.
   */
  
  inode = flist->fl_files[fd].f_inode;
  if (!inode)
    {
      err = ENOENT;
      goto errout;
    }

  /* Make sure that the inode supports the requested access.  In
   * the case of fdopen, we are not actually creating the file -- in
   * particular w and w+ do not truncate the file and any files have
   * already been created.
   */

  if (inode_checkflags(inode, oflags) != OK)
    {
      err = EACCES;
      goto errout;
    }

  /* Find an unallocated FILE structure */

  ret = sem_wait(&slist->sl_sem);
  if (ret != OK)
    {
      goto errout_with_errno;
    }

  for (i = 0 ; i < CONFIG_NFILE_STREAMS; i++)
    {
      stream = &slist->sl_streams[i];
      if (stream->fs_filedes < 0)
        {
          /* Zero the structure */
#if CONFIG_STDIO_BUFFER_SIZE > 0
          memset(stream, 0, sizeof(FILE));
#elif CONFIG_NUNGET_CHARS > 0
          stream->fs_nungotten = 0;
#endif

#if CONFIG_STDIO_BUFFER_SIZE > 0
          /* Initialize the semaphore the manages access to the buffer */

          (void)sem_init(&stream->fs_sem, 0, 1);

          /* Allocate the IO buffer */

          stream->fs_bufstart = kmalloc(CONFIG_STDIO_BUFFER_SIZE);
          if (!stream)
            {
              err = ENOMEM;
              goto errout_with_sem;
            }

          /* Set up pointers */

          stream->fs_bufend  = &stream->fs_bufstart[CONFIG_STDIO_BUFFER_SIZE];
          stream->fs_bufpos  = stream->fs_bufstart;
          stream->fs_bufpos  = stream->fs_bufstart;
          stream->fs_bufread = stream->fs_bufstart;
#endif
          /* Save the file description and open flags.  Setting the
           * file descriptor locks this stream.
           */

          stream->fs_filedes = fd;
          stream->fs_oflags  = oflags;

          sem_post(&slist->sl_sem);
          return stream;
        }
    }

#if CONFIG_STDIO_BUFFER_SIZE > 0
errout_with_sem:
#endif
  sem_post(&slist->sl_sem);

errout:
  set_errno(err);
errout_with_errno:
  return NULL;
}

