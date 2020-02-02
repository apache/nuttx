/****************************************************************************
 * sched/sched/sched_getcpu.c
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
#include <nuttx/arch.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_getcpu
 *
 * Description:
 *    sched_getcpu() returns the number of the CPU on which the calling
 *    thread is currently executing.
 *
 *    The return CPU number is guaranteed to be valid only at the time of
 *    the call.  Unless the CPU affinity has been fixed using
 *    sched_setaffinity(), the OS might change the CPU at any time.  The
 *    caller must allow for the possibility that the information returned is
 *    no longer current by the time the call returns.
 *
 *    Non-Standard.  Functionally equivalent to the GLIBC __GNU_SOURCE
 *    interface of the same name.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A non-negative CPU number is returned on success.  -1 (ERROR) is
 *   returned on failure with the errno value set to indicate the cause of
 *   the failure.
 *
 ****************************************************************************/

int sched_getcpu(void)
{
  return up_cpu_index();  /* Does not fail */
}
