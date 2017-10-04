/****************************************************************************
 * mm/shm/shmat.c
 *
 *   Copyright (C) 2014, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/shm.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/pgalloc.h>

#include "shm/shm.h"

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmat
 *
 * Description:
 *   The shmat() function attaches the shared memory segment associated with
 *   the shared memory identifier specified by shmid to the address space of
 *   the calling process. The segment is attached at the address specified
 *   by one of the following criteria:
 *
 *     - If shmaddr is a null pointer, the segment is attached at the first
 *       available address as selected by the system.
 *     - If shmaddr is not a null pointer and (shmflg & SHM_RND) is non-
 *       zero, the segment is attached at the address given by
 *       (shmaddr - ((uintptr_t)shmaddr % SHMLBA)).
 *     - If shmaddr is not a null pointer and (shmflg & SHM_RND) is 0, the
 *       segment is attached at the address given by shmaddr.
 *     - The segment is attached for reading if (shmflg & SHM_RDONLY) is
 *       non-zero and the calling process has read permission; otherwise, if
 *       it is 0 and the calling process has read and write permission, the
 *       segment is attached for reading and writing.
 *
 * Input Parameters:
 *   shmid  - Shared memory identifier
 *   smaddr - Determines mapping of the shared memory region
 *   shmflg - See SHM_* definitions in include/sys/shm.h.  Only SHM_RDONLY
 *            and SHM_RND are supported.
 *
 * Returned Value:
 *   Upon successful completion, shmat() will increment the value of
 *   shm_nattch in the data structure associated with the shared memory ID
 *   of the attached shared memory segment and return the segment's start
 *   address.
 *
 *   Otherwise, the shared memory segment will not be attached, shmat() will
 *   return -1, and errno will be set to indicate the error.
 *
 *     - EACCES
 *       Operation permission is denied to the calling process
 *     - EINVAL
 *       The value of shmid is not a valid shared memory identifier, the
 *       shmaddr is not a null pointer, and the value of
 *       (shmaddr -((uintptr_t)shmaddr % SHMLBA)) is an illegal address for
 *       attaching shared memory; or the shmaddr is not a null pointer,
 *       (shmflg & SHM_RND) is 0, and the value of shmaddr is an illegal
 *       address for attaching shared memory.
 *     - EMFILE
 *       The number of shared memory segments attached to the calling
 *       process would exceed the system-imposed limit.
 *     - ENOMEM
 *       The available data space is not large enough to accommodate the
 *       shared memory segment.
 *
 ****************************************************************************/

FAR void *shmat(int shmid, FAR const void *shmaddr, int shmflg)
{
  FAR struct shm_region_s *region;
  FAR struct task_group_s *group;
  FAR struct tcb_s *tcb;
  uintptr_t vaddr;
  unsigned int npages;
  int ret;

  /* Get the region associated with the shmid */

  DEBUGASSERT(shmid >= 0 && shmid < CONFIG_ARCH_SHM_MAXREGIONS);
  region =  &g_shminfo.si_region[shmid];
  DEBUGASSERT((region->sr_flags & SRFLAG_INUSE) != 0);

  /* Get the TCB and group containing our virtual memory allocator */

  tcb = sched_self();
  DEBUGASSERT(tcb && tcb->group);
  group = tcb->group;
  DEBUGASSERT(group->tg_shm.gs_handle != NULL &&
              group->tg_shm.gs_vaddr[shmid] == 0);

  /* Get exclusive access to the region data structure */

  ret = nxsem_wait(&region->sr_sem);
  if (ret < 0)
    {
      shmerr("ERROR: nxsem_wait failed: %d\n", ret);
      goto errout_with_ret;
    }

  /* Set aside a virtual address space to span this physical region */

  vaddr = (uintptr_t)gran_alloc(group->tg_shm.gs_handle,
                                region->sr_ds.shm_segsz);
  if (vaddr == 0)
    {
      shmerr("ERROR: gran_alloc() failed\n");
      ret = -ENOMEM;
      goto errout_with_semaphore;
    }

  /* Convert the region size to pages */

  npages = MM_PGALIGNUP(region->sr_ds.shm_segsz);

  /* Attach, i.e, map, on shared memory region to the user virtual address. */

  ret = up_shmat(region->sr_pages, npages, vaddr);
  if (ret < 0)
    {
      shmerr("ERROR: up_shmat() failed\n");
      goto errout_with_vaddr;
    }

  /* Save the virtual address of the region.  We will need that in shmat()
   * to do the reverse lookup:  Give the virtual address of the region to
   * detach, we need to get the region table index.
   */

  group->tg_shm.gs_vaddr[shmid] = vaddr;

  /* Increment the count of processes attached to this region */

  region->sr_ds.shm_nattch++;

  /* Save the process ID of the last operation */

  region->sr_ds.shm_lpid = tcb->pid;

  /* Save the time of the last shmat() */

  region->sr_ds.shm_atime = time(NULL);

  /* Release our lock on the entry */

  nxsem_post(&region->sr_sem);
  return (FAR void *)vaddr;

errout_with_vaddr:
  gran_free(group->tg_shm.gs_handle, (FAR void *)vaddr,
            region->sr_ds.shm_segsz);

errout_with_semaphore:
  nxsem_post(&region->sr_sem);

errout_with_ret:
  set_errno(-ret);
  return (FAR void *)ERROR;
}

#endif /* CONFIG_MM_SHM */
