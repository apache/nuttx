/****************************************************************************
 * libs/libc/stdio/lib_libstream.c
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
#include <assert.h>
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

/****************************************************************************
 * Name: lib_stream_release
 *
 * Description:
 *   This function is called when a TCB is destroyed.  Note that it does not
 *   close the files by releasing the inode.  That happens separately when
 *   the file descriptor list is freed.
 *
 ****************************************************************************/

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

      _RMUTEX_DESTROY(&stream->fs_lock);
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
  _RMUTEX_DESTROY(&list->sl_std[0].fs_lock);
  _RMUTEX_DESTROY(&list->sl_std[1].fs_lock);
  _RMUTEX_DESTROY(&list->sl_std[2].fs_lock);
#endif
}

#endif /* CONFIG_BUILD_FLAT || __KERNEL__ */

void lib_stream_semtake(FAR struct streamlist *list)
{
  int ret;

  /* Take the semaphore (perhaps waiting) */

  while ((ret = _SEM_WAIT(&list->sl_sem)) < 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      DEBUGASSERT(_SEM_ERRNO(ret) == EINTR || _SEM_ERRNO(ret) == ECANCELED);
      UNUSED(ret);
    }
}

void lib_stream_semgive(FAR struct streamlist *list)
{
  _SEM_POST(&list->sl_sem);
}
