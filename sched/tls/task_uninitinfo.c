/****************************************************************************
 * sched/tls/task_uninitinfo.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/list.h>
#include <nuttx/lib/lib.h>
#include <nuttx/environ.h>

#include "tls.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_uninit_stream
 *
 * Description:
 *   This function is called when a TCB is destroyed.  Note that it does not
 *   close the files by releasing the inode.  That happens separately when
 *   the file descriptor list is freed.
 *
 ****************************************************************************/

#ifdef CONFIG_FILE_STREAM
static void task_uninit_stream(FAR struct task_group_s *group)
{
  FAR struct streamlist *list;
  FAR struct file_struct *stream;
  FAR sq_entry_t *curr;
  FAR sq_entry_t *next;

  DEBUGASSERT(group && group->tg_info);
  list = &group->tg_info->ta_streamlist;
  stream = list->sl_std;

  /* Destroy the mutex and release the filelist */

  nxmutex_destroy(&list->sl_lock);

  /* Destroy stdin, stdout and stderr stream */

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  nxrmutex_destroy(&stream[0].fs_lock);
  nxrmutex_destroy(&stream[1].fs_lock);
  nxrmutex_destroy(&stream[2].fs_lock);
#endif

  /* Release each stream in the list */

  sq_for_every_safe(&list->sl_queue, curr, next)
    {
      stream = container_of(curr, struct file_struct, fs_entry);

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
      /* Destroy the mutex that protects the IO buffer */

      nxrmutex_destroy(&stream->fs_lock);
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
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_uninit_info
 *
 * Description:
 *   Uninitilize and free task_info_s structure.
 *
 * Input Parameters:
 *   - group: The group of new task
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void task_uninit_info(FAR struct task_group_s *group)
{
  FAR struct task_info_s *info = group->tg_info;

#ifdef CONFIG_BUILD_KERNEL
  /* The user heap is reclaimed with the address environment.  Its lists
   * may have been corrupted by the exiting task; neither traversing them
   * nor freeing their user allocations is necessary in the kernel.  A
   * vfork() child uses the heap of its parent, which stays.
   */

  if ((group->tg_flags & (GROUP_FLAG_PRIVILEGED | GROUP_FLAG_VFORK)) == 0)
    {
      return;
    }
#endif

#ifdef CONFIG_PTHREAD_ATFORK
  /* Remove the functions that registered with pthread_atfork() */

  FAR struct list_node *list = &info->ta_atfork;
  FAR struct pthread_atfork_s *entry;

  while (!list_is_empty(list))
    {
      entry = list_first_entry(list,
                               struct pthread_atfork_s, node);
      list_delete_init(&entry->node);

      /* pthread_atfork() allocated the entry in the heap of the process:
       * the user heap, unless the process is privileged.
       */

#ifdef CONFIG_BUILD_KERNEL
      /* The user heap of an unprivileged group goes away with its address
       * environment.
       */

      if ((group->tg_flags & GROUP_FLAG_PRIVILEGED) != 0)
#endif
        {
          group_free(group, entry);
        }
    }
#endif

#ifdef CONFIG_FILE_STREAM
  /* Free resource held by the stream list */

  task_uninit_stream(group);

  /* libc allocated the getusershell() state in the heap of the process.
   * Its stream was closed with the stream list.
   */

  if (info->ta_usershell != NULL)
    {
      group_free(group, info->ta_usershell->us_line);
      group_free(group, info->ta_usershell);
      info->ta_usershell = NULL;
    }
#endif /* CONFIG_FILE_STREAM */

#ifdef CONFIG_NETDB_DNSCLIENT
  /* libc allocated the resolver state in the heap of the process */

  if (info->ta_res != NULL)
    {
      group_free(group, info->ta_res);
      info->ta_res = NULL;
    }
#endif

  /* Release all environment variables */

  env_release(info);

#if defined(CONFIG_LIBC_ELF) || defined(CONFIG_BINFMT_LOADABLE)
  nxrmutex_destroy(&info->ta_modlock);
#endif

  nxrmutex_destroy(&info->ta_lock);
#ifdef CONFIG_MM_KERNEL_HEAP
  group_free(group, info);
#endif
}
