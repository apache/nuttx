/****************************************************************************
 * libs/libc/string/lib_psa_init.c
 *
 *   Copyright (C) 2013-2015, 2018 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <spawn.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawnattr_init
 *
 * Description:
 *   The posix_spawnattr_init() function initializes the object referenced
 *   by attr, to an empty set of spawn attributes for subsequent use in a
 *   call to posix_spawn() or posix_spawnp().
 *
 * Input Parameters:
 *   attr - The address of the spawn attributes to be initialized.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int posix_spawnattr_init(posix_spawnattr_t *attr)
{
  struct sched_param param;
  int ret;

  DEBUGASSERT(attr);

  /* Flags: None */

  attr->flags = 0;

  /* Set the default priority to the same priority as this task */

  ret = _SCHED_GETPARAM(0, &param);
  if (ret < 0)
    {
      return _SCHED_ERRNO(ret);
    }

  attr->priority            = param.sched_priority;

  /* Set the default scheduler policy to the policy of this task */

  ret = _SCHED_GETSCHEDULER(0);
  if (ret < 0)
    {
      return _SCHED_ERRNO(ret);
    }

  attr->policy              = ret;

#ifndef CONFIG_DISABLE_SIGNALS
  /* Empty signal mask */

  attr->sigmask             = 0;
#endif

#ifdef CONFIG_SCHED_SPORADIC
  /* Sporadic scheduling parameters */

  attr->low_priority        = (uint8_t)param.sched_ss_low_priority;
  attr->max_repl            = (uint8_t)param.sched_ss_max_repl;
  attr->repl_period.tv_sec  = param.sched_ss_repl_period.tv_sec;
  attr->repl_period.tv_nsec = param.sched_ss_repl_period.tv_nsec;
  attr->budget.tv_sec       = param.sched_ss_init_budget.tv_sec;
  attr->budget.tv_nsec      = param.sched_ss_init_budget.tv_nsec;
#endif


#ifndef CONFIG_ARCH_ADDRENV
  /* Default stack size */

  attr->stacksize           = CONFIG_TASK_SPAWN_DEFAULT_STACKSIZE;
#endif

  return OK;
}
