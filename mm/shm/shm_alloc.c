/****************************************************************************
 * mm/shm/shm_alloc.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/addrenv.h>
#include <nuttx/sched.h>
#include <nuttx/mm/gran.h>
#include <nuttx/pgalloc.h>
#include <nuttx/mm/shm.h>

#include "shm/shm.h"

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shm_alloc
 *
 * Description:
 *   Allocate virtual memory region from the shared memory pool.
 *
 * Input Parameters:
 *   group - A reference to the group structure to be un-initialized.
 *   vaddr - Virtual start address where the allocation starts, if NULL, will
 *           seek and return an address that satisfies the 'size' parameter
 *   size - Size of the area to allocate
 *
 * Returned Value:
 *   Pointer to reserved vaddr, or NULL if out-of-memory
 *
 ****************************************************************************/

FAR void *shm_alloc(FAR struct task_group_s *group, FAR void *vaddr,
                    size_t size)
{
  FAR void *ret = NULL;

  DEBUGASSERT(group != NULL);

  if (group->tg_shm.gs_handle != NULL)
    {
      if (vaddr == NULL)
        {
          ret = gran_alloc(group->tg_shm.gs_handle, size);
        }
      else
        {
          ret = gran_reserve(group->tg_shm.gs_handle, (uintptr_t)vaddr,
                             size);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: shm_free
 *
 * Description:
 *   Free a previously allocated virtual memory region back to the shared
 *   memory pool.
 *
 * Input Parameters:
 *   group - A reference to the group structure to be un-initialized.
 *   vaddr - Virtual start address where the allocation starts.
 *   size - Size of the allocated area.
 *
 ****************************************************************************/

void shm_free(FAR struct task_group_s *group, FAR void *vaddr, size_t size)
{
  DEBUGASSERT(group != NULL);

  if (group->tg_shm.gs_handle != NULL)
    {
      gran_free(group->tg_shm.gs_handle, vaddr, size);
    }
}

#endif /* CONFIG_MM_SHM */
