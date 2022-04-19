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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/mm/shm.h>
#include <nuttx/mm/vm_map.h>

#include "shm/shm.h"

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmdt
 *
 * Description:
 *   The shmdt() function detaches the shared memory segment located at the
 *   address specified by shmaddr from the address space of the calling
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
 *   - EINVAL
 *     The value of shmaddr is not the data segment start address of a
 *     shared memory segment.
 *
 ****************************************************************************/

int shmdt(FAR const void *shmaddr)
{
  FAR struct shm_region_s *region;
  FAR const struct vm_map_entry_s *map;
  unsigned int npages;
  int shmid;
  int ret;
  GRAN_HANDLE gran = vm_allocator_get(VM_ALLOCATOR_SHM);

  DEBUGASSERT(gran);

  /* Perform the reverse lookup to get the shmid corresponding to this
   * shmaddr.
   */

  map = vm_map_find(shmaddr);

  if (!map || map->type != VM_MAP_SHM)
    {
      ret = -EINVAL;
      shmerr("ERROR: No region matching this virtual address: %p\n",
             shmaddr);
      goto errout_with_errno;
    }

  shmid = map->id.shmid;

  /* Get the region associated with the shmid */

  region =  &g_shminfo.si_region[shmid];
  DEBUGASSERT((region->sr_flags & SRFLAG_INUSE) != 0);

  /* Get exclusive access to the region data structure */

  ret = nxsem_wait(&region->sr_sem);
  if (ret < 0)
    {
      shmerr("ERROR: nxsem_wait failed: %d\n", ret);
      goto errout_with_errno;
    }

  /* Free the virtual address space */

  gran_free(gran, (FAR void *)shmaddr,
            region->sr_ds.shm_segsz);

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

  /* Indicate that there is no longer any mapping for this region. */

  if (vm_map_rm(shmaddr) < 0)
    {
      shmerr("ERROR: vm_map_rm() failed\n");
    }

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

  region->sr_ds.shm_lpid = getpid();

  /* Save the time of the last shmdt() */

  region->sr_ds.shm_dtime = time(NULL);

  /* Release our lock on the entry */

  nxsem_post(&region->sr_sem);
  return OK;

errout_with_errno:
  set_errno(-ret);
  return ERROR;
}

#endif /* CONFIG_MM_SHM */
