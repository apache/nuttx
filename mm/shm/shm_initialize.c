/****************************************************************************
 * mm/shm/shm_initialize.c
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

#include <nuttx/addrenv.h>
#include <nuttx/sched.h>
#include <nuttx/mm/gran.h>
#include <nuttx/pgalloc.h>
#include <nuttx/mm/shm.h>

#include "shm/shm.h"

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* State of the all shared memory */

struct shm_info_s g_shminfo =
{
  SEM_INITIALIZER(1)
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shm_group_initialize
 *
 * Description:
 *   Initialize the group shared memory data structures when a new task
 *   group is initialized.
 *
 * Input Parameters:
 *   group - A reference to the new group structure to be initialized.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int shm_group_initialize(FAR struct task_group_s *group)
{
  int ret = -ENOMEM;

  GRAN_HANDLE shm_allocator =
    gran_initialize((FAR void *)CONFIG_ARCH_SHM_VBASE,
                    ARCH_SHM_MAXPAGES << MM_PGSHIFT,
                    MM_PGSHIFT, MM_PGSHIFT);

  if (!shm_allocator)
    {
      shmerr("ERROR: gran_initialize() failed\n");
      return ret;
    }

  return vm_allocator_add(VM_ALLOCATOR_SHM, shm_allocator);
}

/****************************************************************************
 * Name: shm_group_release
 *
 * Description:
 *   Release resources used by the group shared memory logic.  This function
 *   is called at the time at the group is destroyed.
 *
 * Input Parameters:
 *   group - A reference to the group structure to be un-initialized.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void shm_group_release(FAR struct task_group_s *group)
{
  GRAN_HANDLE handle;
  DEBUGASSERT(group);

  handle = vm_allocator_get(VM_ALLOCATOR_SHM);
  if (handle)
    {
      gran_release(handle);
    }

  vm_allocator_rm(VM_ALLOCATOR_SHM);
}

#endif /* CONFIG_MM_SHM */
