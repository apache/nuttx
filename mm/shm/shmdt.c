/****************************************************************************
 * mm/shm/shmdt.c
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

#include <sys/shm.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/pgalloc.h>
#include <nuttx/mm/map.h>

#include "shm/shm.h"

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmdt
 *
 * Description:
 *   The shmdt() function detaches the shared memory segment located at
 *   the address specified by shmaddr from the address space of the calling
 *   process.
 *
 * Input Parameters:
 *   shmid - Shared memory identifier
 *
 * Returned Value:
 *   Upon successful completion, shmdt() will decrement the value of
 *   shm_nattch in the data structure associated with the shared memory ID
 *   of the attached shared memory segment and return 0.
 *
 *   Otherwise, the shared memory segment will not be detached, shmdt()
 *   will return -1, and errno will be set to indicate the error.
 *
 ****************************************************************************/

int shmdt(FAR const void *shmaddr)
{
  FAR struct tcb_s *tcb;
  FAR struct mm_map_entry_s *entry;
  FAR struct task_group_s *group;
  int ret;

  /* Get the TCB and group containing our virtual memory allocator */

  tcb = nxsched_self();
  DEBUGASSERT(tcb && tcb->group);
  group = tcb->group;

  /* Get exclusive access to process' mm_map */

  ret = mm_map_lock();
  if (ret == OK)
    {
      /* Perform the reverse lookup to get the shmid corresponding to this
       * shmaddr. The mapping is matched with just shmaddr == map->vaddr.
       */

      entry = mm_map_find(get_current_mm(), shmaddr, 1);
      if (entry && entry->vaddr == shmaddr)
        {
          DEBUGASSERT(entry->munmap);
          ret = entry->munmap(group, entry, entry->vaddr, entry->length);
        }
      else
        {
          shmerr("ERROR: No region matching this virtual address: %p\n",
                 shmaddr);

          ret = -EINVAL;
        }

      mm_map_unlock();
    }

  if (ret < 0)
    {
      set_errno(-ret);
      ret = -1;
    }

  return ret;
}

#endif /* CONFIG_MM_SHM */
