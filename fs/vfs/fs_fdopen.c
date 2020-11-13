/****************************************************************************
 * fs/vfs/fs_fdopen.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
#include <nuttx/lib/lib.h>
#include <nuttx/net/net.h>

#include "inode/inode.h"

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

int fs_fdopen(int fd, int oflags, FAR struct tcb_s *tcb,
              FAR struct file_struct **filep)
{
  FAR struct streamlist *slist;
  FAR FILE              *stream;
  int                    ret;

  /* Check input parameters */

  if (fd < 0)
    {
      ret = -EBADF;
      goto errout;
    }

  /* A NULL TCB pointer means to use this threads TCB.  This is a little
   * hack the let's this function be called from user-space (via a syscall)
   * without having access to the TCB.
   */

  if (!tcb)
    {
      tcb = nxsched_self();
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

      ret = -EBADF;
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

      goto errout;
    }

  /* Get the stream list from the TCB */

#ifdef CONFIG_MM_KERNEL_HEAP
  slist = tcb->group->tg_streamlist;
#else
  slist = &tcb->group->tg_streamlist;
#endif

  /* Allocate FILE structure */

  if (fd >= 3)
    {
      stream = group_zalloc(tcb->group, sizeof(FILE));
      if (stream == NULL)
        {
          ret = -ENOMEM;
          goto errout;
        }

      /* Add FILE structure to the stream list */

      ret = nxsem_wait(&slist->sl_sem);
      if (ret < 0)
        {
          group_free(tcb->group, stream);
          goto errout;
        }

      if (slist->sl_tail)
        {
          slist->sl_tail->fs_next = stream;
          slist->sl_tail = stream;
        }
      else
        {
          slist->sl_head = stream;
          slist->sl_tail = stream;
        }

      nxsem_post(&slist->sl_sem);

      /* Initialize the semaphore the manages access to the buffer */

      lib_sem_initialize(stream);
    }
  else
    {
      stream = &slist->sl_std[fd];
    }

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
#if CONFIG_STDIO_BUFFER_SIZE > 0
  /* Set up pointers */

  stream->fs_bufstart = stream->fs_buffer;
  stream->fs_bufend   = &stream->fs_bufstart[CONFIG_STDIO_BUFFER_SIZE];
  stream->fs_bufpos   = stream->fs_bufstart;
  stream->fs_bufread  = stream->fs_bufstart;
  stream->fs_flags    = __FS_FLAG_UBF; /* Fake setvbuf and fclose */

#ifdef CONFIG_STDIO_LINEBUFFER
  /* Setup buffer flags */

  stream->fs_flags   |= __FS_FLAG_LBF; /* Line buffering */

#endif /* CONFIG_STDIO_LINEBUFFER */
#endif /* CONFIG_STDIO_BUFFER_SIZE > 0 */
#endif /* CONFIG_STDIO_DISABLE_BUFFERING */

  /* Save the file description and open flags.  Setting the
   * file descriptor locks this stream.
   */

  stream->fs_fd       = fd;
  stream->fs_oflags   = oflags;

  if (filep != NULL)
    {
      *filep = stream;
    }

  return OK;

errout:
  if (filep != NULL)
    {
      *filep = NULL;
    }

  return ret;
}
