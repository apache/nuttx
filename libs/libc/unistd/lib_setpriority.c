/****************************************************************************
 * libs/libc/unistd/lib_setpriority.c
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

#include <errno.h>
#include <unistd.h>

#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setpriority
 *
 * Description:
 *   The setpriority() function shall set the nice value of a process,
 *   process group, or user to value+ {NZERO}.
 *
 * Input Parameters:
 *   which  - PRIO_PROCESS, PRIO_PGRP, or PRIO_USER, ignored in current
 *            implementation.
 *   who    - Process id is interpreted relative to "which"
 *   value  - Value in the range SCHED_PRIORITY_MIN to SCHED_PRIORITY_MAX
 *
 * Returned Value:
 *   Upon successful completion, setpriority() shall return 0; otherwise,
 *   -1 shall be returned and errno set to indicate the error.
 *
 * Assumptions:
 *
 ****************************************************************************/

int setpriority(int which, id_t who, int value)
{
  struct sched_param param;
  int ret;

  UNUSED(which);

  if (who == 0)
    {
      who = _SCHED_GETTID();
    }

  ret = sched_getparam(who, &param);
  if (ret < 0)
    {
      return ret;
    }

  param.sched_priority = NZERO - value;

  return sched_setparam(who, &param);
}
