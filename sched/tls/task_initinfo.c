/****************************************************************************
 * sched/tls/task_initinfo.c
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

#include <errno.h>
#include <fcntl.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "tls.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_init_stream
 *
 * Description:
 *   This function is called when a new task is allocated.  It initializes
 *   the streamlist instance that is stored in the task group.
 *
 ****************************************************************************/

#ifdef CONFIG_FILE_STREAM
static void task_init_stream(FAR struct streamlist *list)
{
  FAR struct file_struct *stream = list->sl_std;
  int i;

  /* Initialize the list access mutex */

  nxmutex_init(&list->sl_lock);
  sq_init(&list->sl_queue);

  /* Initialize stdin, stdout and stderr stream */

  for (i = 0; i < 3; i++)
    {
      nxrmutex_init(&stream[i].fs_lock);

#if !defined(CONFIG_STDIO_DISABLE_BUFFERING) && CONFIG_STDIO_BUFFER_SIZE > 0
      /* Set up pointers */

      stream[i].fs_bufstart = stream[i].fs_buffer;
      stream[i].fs_bufend   = stream[i].fs_bufstart +
                              CONFIG_STDIO_BUFFER_SIZE;
      stream[i].fs_bufpos   = stream[i].fs_bufstart;
      stream[i].fs_bufread  = stream[i].fs_bufstart;
      stream[i].fs_flags    = __FS_FLAG_UBF; /* Fake setvbuf and fclose */
#  ifdef CONFIG_STDIO_LINEBUFFER
      /* Setup buffer flags */

      stream[i].fs_flags   |= __FS_FLAG_LBF; /* Line buffering */

#  endif /* CONFIG_STDIO_LINEBUFFER */

      /* Save the file description and open flags.  Setting the
       * file descriptor locks this stream.
       */

      stream[i].fs_cookie   = (FAR void *)(intptr_t)i;
      stream[i].fs_oflags   = i ? O_WROK : O_RDONLY;

      /* Assign custom callbacks to NULL. */

      stream[i].fs_iofunc.read  = NULL;
      stream[i].fs_iofunc.write = NULL;
      stream[i].fs_iofunc.seek  = NULL;
      stream[i].fs_iofunc.close = NULL;
#endif /* !CONFIG_STDIO_DISABLE_BUFFERING && CONFIG_STDIO_BUFFER_SIZE > 0 */
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_init_info
 *
 * Description:
 *   Allocate and initialize task_info_s structure.
 *
 * Input Parameters:
 *   - group: The group of new task
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int task_init_info(FAR struct task_group_s *group)
{
  FAR struct task_info_s *info;

  /* Allocate task info for group */

#ifdef CONFIG_MM_KERNEL_HEAP
  info = group_zalloc(group, sizeof(struct task_info_s));
  if (info == NULL)
    {
      return -ENOMEM;
    }
#else
  info = &group->tg_info_;
#endif

  /* Initialize user space mutex */

  nxmutex_init(&info->ta_lock);
  group->tg_info = info;

#ifdef CONFIG_PTHREAD_ATFORK
  list_initialize(&info->ta_atfork);
#endif

#ifdef CONFIG_FILE_STREAM
  /* Initialize file streams for the task group */

  task_init_stream(&info->ta_streamlist);
#endif

  return OK;
}
