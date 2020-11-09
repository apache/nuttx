/****************************************************************************
 * libs/libc/misc/lib_stream.c
 *
 *   Copyright (C) 2007, 2011, 2013-2014, 2017 Gregory Nutt.
 *   All rights reserved.
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

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)

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

#ifdef CONFIG_FILE_STREAM
void lib_stream_initialize(FAR struct task_group_s *group)
{
  FAR struct streamlist *list;

#ifdef CONFIG_MM_KERNEL_HEAP
  DEBUGASSERT(group && group->tg_streamlist);
  list = group->tg_streamlist;
#else
  DEBUGASSERT(group);
  list = &group->tg_streamlist;
#endif

  /* Initialize the list access mutex */

  _SEM_INIT(&list->sl_sem, 0, 1);
  list->sl_head = NULL;
  list->sl_tail = NULL;

  /* Initialize stdin, stdout and stderr stream */

  list->sl_std[0].fs_fd = -1;
  lib_sem_initialize(&list->sl_std[0]);
  list->sl_std[1].fs_fd = -1;
  lib_sem_initialize(&list->sl_std[1]);
  list->sl_std[2].fs_fd = -1;
  lib_sem_initialize(&list->sl_std[2]);
}
#endif /* CONFIG_FILE_STREAM */

/****************************************************************************
 * Name: lib_stream_release
 *
 * Description:
 *   This function is called when a TCB is destroyed.  Note that it does not
 *   close the files by releasing the inode.  That happens separately when
 *   the file descriptor list is freed.
 *
 ****************************************************************************/

#ifdef CONFIG_FILE_STREAM
void lib_stream_release(FAR struct task_group_s *group)
{
  FAR struct streamlist *list;

#ifdef CONFIG_MM_KERNEL_HEAP
  DEBUGASSERT(group && group->tg_streamlist);
  list = group->tg_streamlist;
#else
  DEBUGASSERT(group);
  list = &group->tg_streamlist;
#endif

  /* Destroy the semaphore and release the filelist */

  _SEM_DESTROY(&list->sl_sem);

  /* Release each stream in the list */

  list->sl_tail = NULL;
  while (list->sl_head != NULL)
    {
      FAR struct file_struct *stream = list->sl_head;

      list->sl_head = stream->fs_next;

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
      /* Destroy the semaphore that protects the IO buffer */

      _SEM_DESTROY(&stream->fs_sem);
#endif

      /* Release the stream */

#ifdef CONFIG_BUILD_KERNEL
      /* If the exiting group is unprivileged, then it has an address
       * environment.  Don't bother to release the memory in this case...
       * There is no point since the memory lies in the user heap which
       * will be destroyed anyway.  But if this is a privileged group,
       * when we still have to release the memory using the kernel
       * allocator.
       */

      if ((group->tg_flags & GROUP_FLAG_PRIVILEGED) != 0)
#endif
        {
          group_free(group, stream);
        }
    }

  /* Destroy stdin, stdout and stderr stream */

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  _SEM_DESTROY(&list->sl_std[0].fs_sem);
  _SEM_DESTROY(&list->sl_std[1].fs_sem);
  _SEM_DESTROY(&list->sl_std[2].fs_sem);
#endif
}

#endif /* CONFIG_FILE_STREAM */
#endif /* CONFIG_BUILD_FLAT || __KERNEL__ */
