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
#include <nuttx/mm/shm.h>
#include <nuttx/pgalloc.h>
#include <nuttx/mm/map.h>

#include "shm/shm.h"

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmdt_priv
 *
 * Description:
 *   The shmdt_priv() function detaches the shared memory segment located at
 *   the address specified by shmaddr from the address space of the calling
 *   process.
 *
 * Input Parameters:
 *   shmid - Shared memory identifier
 *   group - Current task_group, NULL during process exit
 *
 * Returned Value:
 *   Upon successful completion, shmdt_priv() will decrement the value of
 *   shm_nattch in the data structure associated with the shared memory ID
 *   of the attached shared memory segment and return 0.
 *
 *   Otherwise, the shared memory segment will not be detached, shmdt_priv()
 *   will return -1, and errno will be set to indicate the error.
 *
 *   - EINVAL
 *     The value of shmaddr is not the data segment start address of a
 *     shared memory segment.
 *
 ****************************************************************************/

int shmdt_priv(FAR struct task_group_s *group, FAR const void *shmaddr,
               int shmid)
{
  FAR struct shm_region_s *region;
  pid_t pid;
  unsigned int npages;
  int ret;

  /* Get the region associated with the shmid */

  region =  &g_shminfo.si_region[shmid];
  DEBUGASSERT((region->sr_flags & SRFLAG_INUSE) != 0);

  /* Get exclusive access to the region data structure */

  ret = nxmutex_lock(&region->sr_lock);
  if (ret < 0)
    {
      shmerr("ERROR: nxsem_wait failed: %d\n", ret);
      goto errout_with_errno;
    }

  if (!group)
    {
      pid = 0;
      goto on_process_exit;
    }
  else
    {
      pid = group->tg_pid;
    }

  /* Free the virtual address space */

  shm_free(group, (FAR void *)shmaddr, region->sr_ds.shm_segsz);

  /* Convert the region size to pages */

  npages = MM_NPAGES(region->sr_ds.shm_segsz);

  /* Detach, i.e, unmap, on shared memory region from a user virtual
   * address.
   */

  ret = up_shmdt((uintptr_t)shmaddr, npages);
  if (ret < 0)
    {
      shmerr("ERROR: up_shmdt() failed\n");
    }

on_process_exit:

  /* Decrement the count of processes attached to this region.
   * If the count decrements to zero and there is a pending unlink,
   * then destroy the shared memory region now and stop any further
   * operations on it.
   */

  DEBUGASSERT(region->sr_ds.shm_nattch > 0);
  if (region->sr_ds.shm_nattch <= 1)
    {
      region->sr_ds.shm_nattch = 0;
      if ((region->sr_flags & SRFLAG_UNLINKED) != 0)
        {
          shm_destroy(shmid);
          return OK;
        }
    }
  else
    {
      /* Just decrement the number of processes attached to the shared
       * memory region.
       */

      region->sr_ds.shm_nattch--;
    }

  /* Save the process ID of the last operation */

  region->sr_ds.shm_lpid = pid;

  /* Save the time of the last shmdt() */

  region->sr_ds.shm_dtime = time(NULL);

  /* Release our lock on the entry */

  nxmutex_unlock(&region->sr_lock);
  return OK;

errout_with_errno:
  set_errno(-ret);
  return ERROR;
}

int shmdt(FAR const void *shmaddr)
{
  FAR struct tcb_s *tcb;
  FAR struct mm_map_entry_s *entry;
  FAR struct task_group_s *group;
  int shmid;
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

      entry = mm_map_find(shmaddr, 1);
      if (!entry || entry->vaddr != shmaddr)
        {
          ret = -EINVAL;
          shmerr("ERROR: No region matching this virtual address: %p\n",
                 shmaddr);

          mm_map_unlock();
          return -EINVAL;
        }

      shmid = entry->priv.i;

      /* Indicate that there is no longer any mapping for this region. */

      if (mm_map_remove(get_group_mm(group), entry) < 0)
        {
          shmerr("ERROR: mm_map_remove() failed\n");
        }

      mm_map_unlock();

      ret = shmdt_priv(tcb->group, shmaddr, shmid);
    }

  return ret;
}

#endif /* CONFIG_MM_SHM */
