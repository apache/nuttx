/****************************************************************************
 * libs/libc/misc/lib_stream.c
 *
 *   Copyright (C) 2007, 2011, 2013-2014, 2017 Gregory Nutt. All rights reserved.
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
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lib/lib.h>

#include "libc.h"

#if (!defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_BUILD_KERNEL)) || \
      defined(__KERNEL__)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_stream_initialize
 *
 * Description:
 *   This function is called when a new task is allocated.  It initializes
 *   the streamlist instance that is stored in the task group.
 *
 ****************************************************************************/

#if CONFIG_NFILE_STREAMS > 0
void lib_stream_initialize(FAR struct task_group_s *group)
{
  FAR struct streamlist *list;
  int i;

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
  DEBUGASSERT(group && group->tg_streamlist);
  list = group->tg_streamlist;
#else
  DEBUGASSERT(group);
  list = &group->tg_streamlist;
#endif

  /* Initialize the list access mutex */

  (void)nxsem_init(&list->sl_sem, 0, 1);

  /* Initialize each FILE structure */

  for (i = 0; i < CONFIG_NFILE_STREAMS; i++)
   {
      FAR struct file_struct *stream = &list->sl_streams[i];

      /* Clear the IOB */

      memset(stream, 0, sizeof(FILE));

      /* Indicate not opened */

      stream->fs_fd = -1;

      /* Initialize the stream semaphore to one to support one-at-
       * a-time access to private data sets.
       */

      lib_sem_initialize(&list->sl_streams[i]);
    }
}
#endif /* CONFIG_NFILE_STREAMS > 0 */

/****************************************************************************
 * Name: lib_stream_release
 *
 * Description:
 *   This function is called when a TCB is destroyed.  Note that it does not
 *   close the files by releasing the inode.  That happens separately when
 *   the file descriptor list is freed.
 *
 ****************************************************************************/

#if CONFIG_NFILE_STREAMS > 0
void lib_stream_release(FAR struct task_group_s *group)
{
  FAR struct streamlist *list;
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  int i;
#endif

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
  DEBUGASSERT(group && group->tg_streamlist);
  list = group->tg_streamlist;
#else
  DEBUGASSERT(group);
  list = &group->tg_streamlist;
#endif

  /* Destroy the semaphore and release the filelist */

  (void)_SEM_DESTROY(&list->sl_sem);

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  /* Release each stream in the list */

  for (i = 0; i < CONFIG_NFILE_STREAMS; i++)
    {
      FAR struct file_struct *stream = &list->sl_streams[i];

      /* Destroy the semaphore that protects the IO buffer */

      (void)_SEM_DESTROY(&stream->fs_sem);

      /* Release the IO buffer */

      if (stream->fs_bufstart != NULL &&
          (stream->fs_flags & __FS_FLAG_UBF) == 0)
        {
#ifndef CONFIG_BUILD_KERNEL
          /* Release memory from the user heap */

          sched_ufree(stream->fs_bufstart);
#else
          /* If the exiting group is unprivileged, then it has an address
           * environment.  Don't bother to release the memory in this case...
           * There is no point since the memory lies in the user heap which
           * will be destroyed anyway.  But if this is a privileged group,
           * when we still have to release the memory using the kernel
           * allocator.
           */

          if ((group->tg_flags & GROUP_FLAG_PRIVILEGED) != 0)
            {
              sched_kfree(stream->fs_bufstart);
            }
#endif
        }
    }
#endif
}

#endif /* CONFIG_NFILE_STREAMS > 0 */
#endif /* (!CONFIG_BUILD_PROTECTED &&7 !CONFIG_BUILD_KERNEL) || __KERNEL__ */
