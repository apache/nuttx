/****************************************************************************
 * libs/libc/environ/env_alloc.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <string.h>

#include <nuttx/tls.h>

#ifdef __KERNEL__
#  include <nuttx/kmalloc.h>
#endif

#include "environ/environ.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Each block that env_alloc() gives starts with this header.  The list at
 * ta_envmem and flink hold the address after the header, the address that
 * env_alloc() returned.
 */

struct env_block_s
{
  FAR void *flink;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_heap_alloc
 *
 * Description:
 *   The environment is in the heap that holds the task information: the
 *   kernel heap for a kernel thread, the user heap for a user task.  Kernel
 *   code also changes the environment of kernel threads (the idle task at
 *   boot) and of a new user task.
 *
 ****************************************************************************/

static FAR struct env_block_s *env_heap_alloc(FAR struct task_info_s *info,
                                              size_t size)
{
#ifdef __KERNEL__
  return kmm_heapmember(info) ? kmm_malloc(size) : kumm_malloc(size);
#else
  UNUSED(info);
  return lib_umalloc(size);
#endif
}

/****************************************************************************
 * Name: env_heap_free
 ****************************************************************************/

static void env_heap_free(FAR struct task_info_s *info,
                          FAR struct env_block_s *block)
{
#ifdef __KERNEL__
  if (kmm_heapmember(info))
    {
      kmm_free(block);
    }
  else
    {
      kumm_free(block);
    }
#else
  UNUSED(info);
  lib_ufree(block);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_alloc
 ****************************************************************************/

FAR void *env_alloc(FAR struct task_info_s *info, size_t size)
{
  FAR struct env_block_s *block;

  block = env_heap_alloc(info, sizeof(*block) + size);
  if (block == NULL)
    {
      return NULL;
    }

  block->flink    = info->ta_envmem;
  info->ta_envmem = block + 1;
  return block + 1;
}

/****************************************************************************
 * Name: env_free
 ****************************************************************************/

void env_free(FAR struct task_info_s *info, FAR void *mem)
{
  FAR struct env_block_s *block = (FAR struct env_block_s *)mem - 1;
  FAR void **prev;

  for (prev = &info->ta_envmem;
       *prev != NULL;
       prev = &((FAR struct env_block_s *)*prev - 1)->flink)
    {
      if (*prev == mem)
        {
          *prev = block->flink;
          env_heap_free(info, block);
          return;
        }
    }

  DEBUGPANIC();
}

/****************************************************************************
 * Name: env_reserve
 ****************************************************************************/

int env_reserve(FAR struct task_info_s *info, size_t more)
{
  FAR char **envp = info->ta_envp;
  FAR char **arr;
  size_t envc = 0;
  size_t envpc;

  /* The program can change the entries of environ in place, so count
   * them again.
   */

  while (envp != NULL && envp[envc] != NULL)
    {
      envc++;
    }

  info->ta_envc = envc;

  if (envp != NULL && envp == info->ta_envarr &&
      envc + more < info->ta_envpc)
    {
      return OK;
    }

  envpc = envc + more + SCHED_ENVIRON_RESERVED + 1;
  arr   = env_alloc(info, sizeof(*arr) * envpc);
  if (arr == NULL)
    {
      return -ENOMEM;
    }

  if (envc > 0)
    {
      memcpy(arr, envp, sizeof(*arr) * envc);
    }

  arr[envc] = NULL;

  /* A full array of libc is not used again.  An array of libc that the
   * program replaced stays until exit: the program can assign it to
   * environ again.
   */

  if (envp != NULL && envp == info->ta_envarr)
    {
      env_free(info, envp);
    }

  info->ta_envp   = arr;
  info->ta_envarr = arr;
  info->ta_envpc  = envpc;
  return OK;
}
