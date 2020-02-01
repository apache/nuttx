/****************************************************************************
 * fs/vfs/fs_fdopen.c
 *
 *   Copyright (C) 2007-2014, 2017 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs_checkfd
 *
 * Description:
 *   Check if the file descriptor is valid for the provided TCB and if it
 *   supports the requested access.
 *
 ****************************************************************************/

static inline int fs_checkfd(FAR struct tcb_s *tcb, int fd, int oflags)
{
  FAR struct file *filep;
  FAR struct inode *inode;
  int ret;

  DEBUGASSERT(tcb && tcb->group);

  /* Get the file structure corresponding to the file descriptor. */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      return ret;
    }

  /* Get the inode associated with the file descriptor.  This should
   * normally be the case if fd >= 0.  But not in the case where the
   * called attempts to explicitly stdin with fdopen(0) but stdin has
   * been closed.
   */

  inode = filep->f_inode;
  if (!inode)
    {
      /* No inode -- descriptor does not correspond to an open file */

      return -ENOENT;
    }

  /* Make sure that the inode supports the requested access.  In
   * the case of fdopen, we are not actually creating the file -- in
   * particular w and w+ do not truncate the file and any files have
   * already been created.
   */

  if (inode_checkflags(inode, oflags) != OK)
    {
      /* Cannot support the requested access */

      return -EACCES;
    }

  /* Looks good to me */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs_fdopen
 *
 * Description:
 *   This function does the core operations for fopen and fdopen.
 *
 ****************************************************************************/

FAR struct file_struct *fs_fdopen(int fd, int oflags, FAR struct tcb_s *tcb)
{
  FAR struct streamlist *slist;
  FAR FILE              *stream;
  int                    errcode = OK;
  int                    ret;
  int                    i;

  /* Check input parameters */

  if (fd < 0)
    {
      errcode = EBADF;
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

  DEBUGASSERT(tcb && tcb->group);

  /* Verify that this is a valid file/socket descriptor and that the
   * requested access can be support.
   *
   * Is this fd in the range of valid file descriptors?  Socket descriptors
   * lie in a different range.
   */

  if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
    {
      /* No.. If networking is enabled then this might be a socket
       * descriptor.
       */

#ifdef CONFIG_NET
      ret = net_checksd(fd, oflags);
#else
      /* No networking... it is just a bad descriptor */

      errcode = EBADF;
      goto errout;
#endif
    }

  /* The descriptor is in a valid range to file descriptor... perform some
   * more checks.
   */

  else
    {
      ret = fs_checkfd(tcb, fd, oflags);
    }

  /* Do we have a good descriptor of some sort? */

  if (ret < 0)
    {
      /* No... return the reported error */

      errcode = -ret;
      goto errout;
    }

  /* Get the stream list from the TCB */

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
  slist = tcb->group->tg_streamlist;
#else
  slist = &tcb->group->tg_streamlist;
#endif

  /* Find an unallocated FILE structure in the stream list */

  ret = nxsem_wait(&slist->sl_sem);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout;
    }

  for (i = 0 ; i < CONFIG_NFILE_STREAMS; i++)
    {
      stream = &slist->sl_streams[i];
      if (stream->fs_fd < 0)
        {
          /* Zero the structure */

          memset(stream, 0, sizeof(FILE));

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
          /* Initialize the semaphore the manages access to the buffer */

          nxsem_init(&stream->fs_sem, 0, 1);

#if CONFIG_STDIO_BUFFER_SIZE > 0
          /* Allocate the IO buffer at the appropriate privilege level for
           * the group.
           */

          stream->fs_bufstart =
            group_malloc(tcb->group, CONFIG_STDIO_BUFFER_SIZE);

          if (!stream->fs_bufstart)
            {
              errcode = ENOMEM;
              goto errout_with_sem;
            }

          /* Set up pointers */

          stream->fs_bufend  = &stream->fs_bufstart[CONFIG_STDIO_BUFFER_SIZE];
          stream->fs_bufpos  = stream->fs_bufstart;
          stream->fs_bufread = stream->fs_bufstart;

#ifdef CONFIG_STDIO_LINEBUFFER
          /* Setup buffer flags */

          stream->fs_flags  |= __FS_FLAG_LBF; /* Line buffering */

#endif /* CONFIG_STDIO_LINEBUFFER */
#endif /* CONFIG_STDIO_BUFFER_SIZE > 0 */
#endif /* !CONFIG_STDIO_DISABLE_BUFFERING */

          /* Save the file description and open flags.  Setting the
           * file descriptor locks this stream.
           */

          stream->fs_fd      = fd;
          stream->fs_oflags  = (uint16_t)oflags;

          nxsem_post(&slist->sl_sem);
          return stream;
        }
    }

  /* No free stream available.. report ENFILE */

  errcode = ENFILE;

#if !defined(CONFIG_STDIO_DISABLE_BUFFERING) && CONFIG_STDIO_BUFFER_SIZE > 0
errout_with_sem:
#endif
  nxsem_post(&slist->sl_sem);

errout:
  set_errno(errcode);
  return NULL;
}
