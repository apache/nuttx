/****************************************************************************
 * libs/libc/pthread/pthread_attr_getschedparam.c
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

#include <pthread.h>
#include <string.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_attr_getschedparam
 *
 * Description:
 *
 * Input Parameters:
 *   attr
 *   param
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an error code.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_attr_getschedparam(FAR const pthread_attr_t *attr,
                               FAR struct sched_param *param)
{
  int ret;

  linfo("attr=%p param=%p\n", attr, param);

  if (!attr || !param)
    {
      ret = EINVAL;
    }
  else
    {
      param->sched_priority               = (int)attr->priority;
#ifdef CONFIG_SCHED_SPORADIC
      param->sched_ss_low_priority        = (int)attr->low_priority;
      param->sched_ss_max_repl            = (int)attr->max_repl;
      param->sched_ss_repl_period.tv_sec  = attr->repl_period.tv_sec;
      param->sched_ss_repl_period.tv_nsec = attr->repl_period.tv_nsec;
      param->sched_ss_init_budget.tv_sec  = attr->budget.tv_sec;
      param->sched_ss_init_budget.tv_nsec = attr->budget.tv_nsec;
#endif
      ret = OK;
    }

  linfo("Returning %d\n", ret);
  return ret;
}
