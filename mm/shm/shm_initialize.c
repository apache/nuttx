/****************************************************************************
 * mm/shm/shm_initialize.c
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

struct shm_info_s g_shminfo;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shm_initialize
 *
 * Description:
 *   Perform one time, start-up initialization of the shared memory logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void shm_initialize(void)
{
  /* Initialize the shared memory region list */

  nxsem_init(&g_shminfo.si_sem, 0, 1);
}

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
  DEBUGASSERT(group && !group->tg_shm.gs_handle);

  group->tg_shm.gs_handle =
    gran_initialize((FAR void *)CONFIG_ARCH_SHM_VBASE,
                    ARCH_SHM_MAXPAGES << MM_PGSHIFT,
                    MM_PGSHIFT, MM_PGSHIFT);

  if (!group->tg_shm.gs_handle)
    {
      shmerr("ERROR: gran_initialize() failed\n");
      return -ENOMEM;
    }

  return OK;
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

  handle = group->tg_shm.gs_handle;
  if (handle)
    {
      gran_release(handle);
    }
}

#endif /* CONFIG_MM_SHM */
