/****************************************************************************
 * sched/sched/sched_verifytcb.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdbool.h>
#include <sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_verify_pid
 *
 * Description:
 *   Given a task ID, this function will check whether the
 *   TCB corresponding to the PID exists.
 *
 ****************************************************************************/

bool nxsched_verify_pid(pid_t pid)
{
  FAR struct tcb_s *ret;

  ret = nxsched_get_tcb(pid);
  if (ret)
    {
      nxsched_put_tcb(ret);
      return true;
    }

  return false;
}
