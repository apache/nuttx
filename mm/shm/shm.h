/****************************************************************************
 * mm/shm/shm.h
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

#ifndef __MM_SHM_SHM_H
#define __MM_SHM_SHM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdint.h>

#include <nuttx/addrenv.h>
#include <nuttx/mutex.h>

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit definitions for the struct shm_region_s sr_flags field */

#define SRFLAG_AVAILABLE 0        /* Available if no flag bits set */
#define SRFLAG_INUSE     (1 << 0) /* Bit 0: Region is in use */
#define SRFLAG_UNLINKED  (1 << 1) /* Bit 1: Region perists while references */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents the state of one shared memory region
 * allocation.  Cast compatible with struct shmid_ds.
 */

struct shm_region_s
{
  struct  shmid_ds sr_ds;  /* Region info */
  bool    sr_flags;        /* See SRFLAGS_* definitions */
  key_t   sr_key;          /* Lookup key */
  mutex_t sr_lock;         /* Manages exclusive access to this region */

  /* List of physical pages allocated for this memory region */

  uintptr_t sr_pages[CONFIG_ARCH_SHM_NPAGES];
};

/* This structure represents the set of all shared memory regions.
 * Access to the region
 */

struct shm_info_s
{
  mutex_t si_lock;         /* Manages exclusive access to the region list */
  struct shm_region_s si_region[CONFIG_ARCH_SHM_MAXREGIONS];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* State of the all shared memory */

extern struct shm_info_s g_shminfo;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: shm_destroy
 *
 * Description:
 *   Destroy a memory region.  This function is called:
 *
 *   - On certain conditions when shmget() is not successful in instantiating
 *     the full memory region and we need to clean up and free a table entry.
 *   - When shmctl() is called with cmd == IPC_RMID and there are no
 *     processes attached to the memory region.
 *   - When shmdt() is called after the last process detaches from memory
 *     region after it was previously marked for deletion by shmctl().
 *
 * Input Parameters:
 *   shmid - Shared memory identifier
 *
 * Returned Value:
 *   None
 *
 * Assumption:
 *   The caller holds either the region table mutex or else the
 *   mutex on the particular entry being deleted.
 *
 ****************************************************************************/

void shm_destroy(int shmid);

#endif /* CONFIG_MM_SHM */
#endif /* __MM_SHM_SHM_H */
