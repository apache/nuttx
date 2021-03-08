/****************************************************************************
 * libs/libc/spawn/lib_psa_getschedparam.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawnattr_getschedparam
 *
 * Description:
 *   The posix_spawnattr_getschedparam() function will obtain the value of
 *   the spawn-schedparam attribute from the attributes object referenced
 *   by attr.
 *
 * Input Parameters:
 *   attr - The address spawn attributes to be queried.
 *   param - The location to return the spawn-schedparam value.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int posix_spawnattr_getschedparam(FAR const posix_spawnattr_t *attr,
                                  FAR struct sched_param *param)
{
  DEBUGASSERT(attr && param);
  param->sched_priority = attr->priority;
#ifdef CONFIG_SCHED_SPORADIC
  param->sched_ss_low_priority        = (int)attr->low_priority;
  param->sched_ss_max_repl            = (int)attr->max_repl;
  param->sched_ss_repl_period.tv_sec  = attr->repl_period.tv_sec;
  param->sched_ss_repl_period.tv_nsec = attr->repl_period.tv_nsec;
  param->sched_ss_init_budget.tv_sec  = attr->budget.tv_sec;
  param->sched_ss_init_budget.tv_nsec = attr->budget.tv_nsec;
#endif
  return OK;
}
