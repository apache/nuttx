/****************************************************************************
 * libs/libc/spawn/lib_psa_init.c
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

  /* Empty signal mask */

  attr->sigmask             = 0;

#ifdef CONFIG_SCHED_SPORADIC
  /* Sporadic scheduling parameters */

  attr->low_priority        = (uint8_t)param.sched_ss_low_priority;
  attr->max_repl            = (uint8_t)param.sched_ss_max_repl;
  attr->repl_period.tv_sec  = param.sched_ss_repl_period.tv_sec;
  attr->repl_period.tv_nsec = param.sched_ss_repl_period.tv_nsec;
  attr->budget.tv_sec       = param.sched_ss_init_budget.tv_sec;
  attr->budget.tv_nsec      = param.sched_ss_init_budget.tv_nsec;
#endif

  /* Default stack size */

  attr->stacksize           = CONFIG_TASK_SPAWN_DEFAULT_STACKSIZE;

  return OK;
}
