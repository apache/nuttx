/****************************************************************************
 * arch/arm/src/common/arm_onexit.c
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
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "svcall.h"
#include "arm_internal.h"

#if defined(CONFIG_SCHED_ATEXIT) || defined(CONFIG_SCHED_ONEXIT) && \
    !defined(CONFIG_BUILD_FLAT)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_nxtask_onexit
 *
 * Description:
 *   In this kernel mode build, this function will be called to execute a
 *   onexit callback in user-space.
 *
 * Input Parameters:
 *   func       - The user-space exit callback
 *   exitcode   - The exit status of task
 *   arg        - The data pointer from user space
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void up_nxtask_onexit(onexitfunc_t func, int exitcode, FAR void *arg)
{
  sys_call3(SYS_nxtask_onexit, (uintptr_t)func, (uintptr_t)exitcode,
            (uintptr_t)arg);
}

#endif
