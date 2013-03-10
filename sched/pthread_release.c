/************************************************************************
 * sched/pthread_release.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <debug.h>

#include "os_internal.h"
#include "pthread_internal.h"

/************************************************************************
 * Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Global Variables
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: pthread_release
 *
 * Description:
 *   Release pthread resources from the task group with the group
 *   terminated.
 *
 * Parameters:
 *   group = The task group containing the pthread resources to be
 *           released.
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *
 ************************************************************************/

void pthread_release(FAR struct task_group_s *group)
{
  FAR struct join_s *join;

  sdbg("group=0x%p\n", group);

  /* Visit and delete each join structure still in the list.  Since we
   * are last exiting thread of the group, no special protection should
   * be required.
   */

  while (group->tg_joinhead)
    {
      /* Remove the join from the head of the list. */

      join = group->tg_joinhead;
      group->tg_joinhead = join->next;

      /* Destroy the join semaphores */

      (void)sem_destroy(&join->data_sem);
      (void)sem_destroy(&join->exit_sem);

      /* And deallocate the join structure */

      sched_kfree(join);
    }

  /* Destroy the join list semaphore */

  (void)sem_destroy(&group->tg_joinsem);
}
