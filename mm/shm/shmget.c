/****************************************************************************
 * mm/shm/shmget.c
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
#include <sys/ipc.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <nuttx/pgalloc.h>
#include <nuttx/mm/shm.h>

#include "shm/shm.h"

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shm_find
 *
 * Description:
 *   Find the shared memory region with matching key
 *
 * Input Parameters:
 *   key - The value that uniquely identifies a shared memory region.
 *
 * Returned Value:
 *   On success, an index in the range of 0 to CONFIG_ARCH_SHM_MAXREGIONS-1
 *   is returned to identify the matching region; -ENOENT is returned on
 *   failure.
 *
 ****************************************************************************/

static int shm_find(key_t key)
{
  int i;

  for (i = 0; i < CONFIG_ARCH_SHM_MAXREGIONS; i++)
    {
      if (g_shminfo.si_region[i].sr_key == key)
        {
          return i;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: shm_reserve
 *
 * Description:
 *   Allocate an unused shared memory region.  That is one with a key of -1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, an index in the range of 0 to CONFIG_ARCH_SHM_MAXREGIONS-1
 *   is returned to identify the matching region; -ENOSPC is returned on
 *   failure.
 *
 ****************************************************************************/

static int shm_reserve(key_t key, int shmflg)
{
  FAR struct shm_region_s *region;
  int i;

  for (i = 0; i < CONFIG_ARCH_SHM_MAXREGIONS; i++)
    {
      /* Is this region in use? */

      region = &g_shminfo.si_region[i];
      if (region->sr_flags == SRFLAG_AVAILABLE)
        {
          /* No... reserve it for the caller now */

          memset(region, 0, sizeof(struct shm_region_s));
          region->sr_key   = key;
          region->sr_flags = SRFLAG_INUSE;

          nxsem_init(&region->sr_sem, 0, 1);

          /* Set the low-order nine bits of shm_perm.mode to the low-order
           * nine bits of shmflg.
           */

          region->sr_ds.shm_perm.mode = shmflg & IPC_MODE;

          /* The value of shm_segsz is left equal to zero for now because no
           * memory has yet been allocated.
           *
           * The values of shm_lpid, shm_nattch, shm_atime, and shm_dtime are
           * set equal to 0.
           */

          /* The value of shm_ctime is set equal to the current time. */

          region->sr_ds.shm_ctime = time(NULL);
          return i;
        }
    }

  return -ENOSPC;
}

/****************************************************************************
 * Name: shm_extend
 *
 * Description:
 *   Extend the size of a memory regions by allocating physical pages as
 *   necessary
 *
 * Input Parameters:
 *   shmid - The index of the region of interest in the shared memory region
 *     table.
 *   size - The new size of the region.
 *
 * Returned Value:
 *   Zero is returned on success; -ENOMEM is returned on failure.
 *   (Should a different error be returned if the region is just too big?)
 *
 ****************************************************************************/

static int shm_extend(int shmid, size_t size)
{
  FAR struct shm_region_s *region = &g_shminfo.si_region[shmid];
  unsigned int pgalloc;
  unsigned int pgneeded;

  /* This is the number of pages that are needed to satisfy the allocation */

  pgneeded = MM_NPAGES(size);

  /* This is the number of pages that have already been allocated */

  pgalloc = MM_NPAGES(region->sr_ds.shm_segsz);

  /* Loop until all pages have been allocated (or something bad happens) */

  while (pgalloc < pgneeded && pgalloc < CONFIG_ARCH_SHM_NPAGES)
    {
      /* Allocate one more physical page */

      region->sr_pages[pgalloc] = mm_pgalloc(1);
      if (region->sr_pages[pgalloc] == 0)
        {
          shmerr("ERROR: mm_pgalloc(1) failed\n");
          break;
        }

      /* Increment the number of pages successfully allocated */

      pgalloc++;
    }

  /* We get here (1) because all of the pages were successfully, (2) because
   * mm_pgalloc() failed, or (3) because any additional pages allocated
   * would exceed CONFIG_ARCH_SHM_NPAGES.
   */

  if (pgalloc < pgneeded)
    {
      /* Set the amount memory available which will be less than the
       * requested size.
       */

      region->sr_ds.shm_segsz = pgalloc << MM_PGSHIFT;
      return -ENOMEM;
    }

  /* Set the new region size and return success */

  region->sr_ds.shm_segsz = size;
  return OK;
}

/****************************************************************************
 * Name: shm_create
 *
 * Description:
 *   Create the shared memory region.
 *
 * Input Parameters:
 *   key     - The key that is used to access the unique shared memory
 *             identifier.
 *   size    - The shared memory region that is created will be at least
 *             this size in bytes.
 *   shmflgs - See IPC_* definitions in sys/ipc.h.  Only the values
 *             IPC_PRIVATE or IPC_CREAT are supported.
 *
 * Returned Value:
 *   Zero is returned on success;  A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int shm_create(key_t key, size_t size, int shmflg)
{
  FAR struct shm_region_s *region;
  int shmid;
  int ret;

  /* Reserve the shared memory region */

  ret = shm_reserve(key, shmflg);
  if (ret < 0)
    {
      shmerr("ERROR: shm_reserve failed: %d\n", ret);
      return ret;
    }

  /* Save the shared memory ID */

  shmid = ret;

  /* Then allocate the physical memory (by extending it from the initial
   * size of zero).
   */

  ret = shm_extend(shmid, size);
  if (ret < 0)
    {
      /* Free any partial allocations and unreserve the region */

      shm_destroy(shmid);
      return ret;
    }

  /* Save the process ID of the creator */

  region = &g_shminfo.si_region[shmid];
  region->sr_ds.shm_cpid = getpid();

  /* Return the shared memory ID */

  return shmid;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shmget
 *
 * Description:
 *   The shmget() function will return the shared memory identifier
 *   associated with key.
 *
 *   A shared memory identifier, associated data structure, and shared
 *   memory segment of at least size bytes is created for key if one of the
 *   following is true:
 *
 *     - The argument key is equal to IPC_PRIVATE.
 *     - The argument key does not already have a shared memory identifier
 *       associated with it and (shmflg & IPC_CREAT) is non-zero.
 *
 *   Upon creation, the data structure associated with the new shared memory
 *   identifier will be initialized as follows:
 *
 *     - The low-order nine bits of shm_perm.mode are set equal to the low-
 *       order nine bits of shmflg.
 *     - The value of shm_segsz is set equal to the value of size.
 *     - The values of shm_lpid, shm_nattch, shm_atime, and shm_dtime are
 *       set equal to 0.
 *     - The value of shm_ctime is set equal to the current time.
 *
 *   When the shared memory segment is created, it will be initialized with
 *   all zero values.
 *
 * Input Parameters:
 *   key     - The key that is used to access the unique shared memory
 *             identifier.
 *   size    - The shared memory region that is created will be at least
 *             this size in bytes.
 *   shmflgs - See IPC_* definitions in sys/ipc.h.  Only the values
 *             IPC_PRIVATE or IPC_CREAT are supported.
 *
 * Returned Value:
 *   Upon successful completion, shmget() will return a non-negative
 *   integer, namely a shared memory identifier; otherwise, it will return
 *   -1 and set errno to indicate the error.
 *
 *     - EACCES
 *       A shared memory identifier exists for key but operation permission
 *       as specified by the low-order nine bits of shmflg would not be
 *       granted.
 *     - EEXIST
 *       A shared memory identifier exists for the argument key but
 *      (shmflg & IPC_CREAT) && (shmflg & IPC_EXCL) are non-zero.
 *     - EINVAL
 *       A shared memory segment is to be created and the value of size is
 *       less than the system-imposed minimum or greater than the system-
 *       imposed maximum.
 *     - EINVAL
 *       No shared memory segment is to be created and a shared memory
 *       segment exists for key but the size of the segment associated with
 *       it is less than size and size is not 0.
 *     - ENOENT
 *       A shared memory identifier does not exist for the argument key and
 *       (shmflg & IPC_CREAT) is 0.
 *     - ENOMEM
 *       A shared memory identifier and associated shared memory segment
 *       will be created, but the amount of available physical memory is
 *       not sufficient to fill the request.
 *     - ENOSPC
 *       A shared memory identifier is to be created, but the system-imposed
 *       limit on the maximum number of allowed shared memory identifiers
 *       system-wide would be exceeded.
 *
 * POSIX Deviations:
 *     - The values of shm_perm.cuid, shm_perm.uid, shm_perm.cgid, and
 *       shm_perm.gid should be set equal to the effective user ID and
 *       effective group ID, respectively, of the calling process.
 *       The NuttX ipc_perm structure, however, does not support these
 *       fields because user and group IDs are not yet supported by NuttX.
 *
 ****************************************************************************/

int shmget(key_t key, size_t size, int shmflg)
{
  FAR struct shm_region_s *region;
  int shmid = -1;
  int ret;

  /* Check for the special case where the caller doesn't really want shared
   * memory (they why do they bother to call us?)
   */

  if (key == IPC_PRIVATE)
    {
      /* Not yet implemented */

      ret = -ENOSYS;
      goto errout;
    }

  /* Get exclusive access to the global list of shared memory regions */

  ret = nxsem_wait(&g_shminfo.si_sem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Find the requested memory region */

  ret = shm_find(key);
  if (ret < 0)
    {
      /* The memory region does not exist.. create it if IPC_CREAT is
       * included in the shmflags.
       */

      if ((shmflg & IPC_CREAT) != 0)
        {
          /* Create the memory region */

          ret = shm_create(key, size, shmflg);
          if (ret < 0)
            {
              shmerr("ERROR: shm_create failed: %d\n", ret);
              goto errout_with_semaphore;
            }

          /* Return the shared memory ID */

          shmid = ret;
        }
      else
        {
          /* Fail with ENOENT */

          goto errout_with_semaphore;
        }
    }

  /* The region exists */

  else
    {
      /* Remember the shared memory ID */

      shmid = ret;

      /* Is the region big enough for the request? */

      region = &g_shminfo.si_region[shmid];
      if (region->sr_ds.shm_segsz < size)
        {
          /* We we asked to create the region?  If so we can just
           * extend it.
           *
           * REVISIT: We should check the mode bits of the regions
           * first
           */

          if ((shmflg & IPC_CREAT) != 0)
            {
              /* Extend the region */

              ret = shm_extend(shmid, size);
              if (ret < 0)
                {
                  shmerr("ERROR: shm_create failed: %d\n", ret);
                  goto errout_with_semaphore;
                }
            }
          else
            {
              /* Fail with EINVAL */

              ret = -EINVAL;
              goto errout_with_semaphore;
            }
        }

      /* The region is already big enough or else we successfully
       * extended the size of the region.  If the region was previously
       * deleted, but waiting for processes to detach from the region,
       * then it is no longer deleted.
       */

      region->sr_flags = SRFLAG_INUSE;
    }

  /* Release our lock on the shared memory region list */

  nxsem_post(&g_shminfo.si_sem);
  return shmid;

errout_with_semaphore:
  nxsem_post(&g_shminfo.si_sem);

errout:
  set_errno(-ret);
  return ERROR;
}

#endif /* CONFIG_MM_SHM */
