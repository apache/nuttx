/****************************************************************************
 * arch/arm/src/common/arm_pthread_exit.c
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

#include "svcall.h"
#include "arm_internal.h"

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__) && \
    !defined(CONFIG_DISABLE_PTHREAD)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_pthread_exit
 *
 * Description:
 *   In this kernel mode build, this function will be called to execute a
 *   pthread in user-space. This kernel-mode stub will then be called
 *   transfer control to the user-mode pthread_exit.
 *
 * Input Parameters:
 *   exit       - The user-space pthread_exit function
 *   exit_value - The pointer of the pthread exit parameter
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void up_pthread_exit(pthread_exitroutine_t exit, FAR void *exit_value)
{
  /* Let sys_call2() do all of the work */

  sys_call2(SYS_pthread_exit, (uintptr_t)exit, (uintptr_t)exit_value);

  /* Suppress "'noreturn' function does return" warning */

  while (1);
}

#endif /* !CONFIG_BUILD_FLAT && __KERNEL__ && !CONFIG_DISABLE_PTHREAD */
