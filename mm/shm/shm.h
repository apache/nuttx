/****************************************************************************
 * mm/shm/shm.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __MM_SHM_SHM_H
#define __MM_SHM_SHM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdint.h>
#include <semaphore.h>

#include <nuttx/addrenv.h>

#ifdef CONFIG_MM_SHM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IPC_PRIVATE is the only value for the the SHM key that is guaranteed to
 * be invalid.
 */

#define SHM_INVALID_KEY IPC_PRIVATE

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents the state of one shared memory region
 * allocation.  Cast compatible with struct shmid_ds.
 */

struct shm_region_s
{
  struct shmid_ds sr_ds; /* Region info */
  key_t sr_key;          /* Lookup key.  IPC_PRIVATE means unused */
  sem_t sr_sem;          /* Manages exclusive access to this region */

  /* List of physical pages allocated for this memory region */

  uintptr_t sr_pages[CONFIG_ARCH_SHM_NPAGES];
};

/* This structure represents the set of all shared memory regions.
 * Access to the region 
 */

struct shm_info_s
{
  sem_t si_sem;         /* Manages exclusive access to the region list */
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

#endif /* CONFIG_MM_SHM */
#endif /* __MM_SHM_SHM_H */
