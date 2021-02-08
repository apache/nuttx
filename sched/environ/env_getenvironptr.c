/****************************************************************************
 * sched/environ/env_getenvironptr.c
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

#ifndef CONFIG_DISABLE_ENVIRON

#include <sched.h>
#include <stdlib.h>
#include "sched/sched.h"

#undef get_environ_ptr

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_environ_ptr
 *
 * Description:
 *   Return a pointer to the thread specific environ variable.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A pointer to the per-thread environ variable.
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR char **get_environ_ptr(void)
{
#if 1

  /* Type of internal representation of environment is incompatible with
   * char ** return value.
   */

  return NULL;

#else

  /* Return a reference to the thread-private environ in the TCB. */

  FAR struct tcb_s *ptcb = this_task();
  if (ptcb->envp)
    {
      return &ptcb->envp->ev_env;
    }
  else
    {
      return NULL;
    }

#endif
}

#endif /* CONFIG_DISABLE_ENVIRON */
