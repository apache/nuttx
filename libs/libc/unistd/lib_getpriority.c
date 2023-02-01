/****************************************************************************
 * libs/libc/unistd/lib_getpriority.c
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
#include <sys/resource.h>

#include <sched.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getpriority
 *
 * Description:
 *   The getpriority() function shall obtain the nice value of a process,
 *   process group, or user.
 *
 * Input Parameters:
 *   which  - PRIO_PROCESS, PRIO_PGRP, or PRIO_USER, ignored in current
 *            implementation.
 *   who    - Process id is interpreted relative to "which"
 *
 * Returned Value:
 *   Upon successful completion, getpriority() shall return an integer in
 *   the range -{NZERO} to {NZERO}-1. Otherwise, -1 shall be returned and
 *   errno set to indicate the error. The following errors may be
 *   reported:
 *
 *   - ESRCH: No process was located using the which and who values
 *            specified.
 *   - EINVAL: which was not one of PRIO_PROCESS, PRIO_PGRP, or
 *             PRIO_USER.
 *
 * Assumptions:
 *
 ****************************************************************************/

int getpriority(int which, id_t who)
{
  struct sched_param param;
  int ret;

  if (which > PRIO_USER || which < PRIO_PROCESS)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (who == 0)
    {
      who = _SCHED_GETTID();
    }

  ret = sched_getparam(who, &param);
  if (ret < 0)
    {
      return ret;
    }

  /* Since -1 is a legal return value, clear errno to avoid the chaos */

  set_errno(0);

  return NZERO - param.sched_priority;
}
