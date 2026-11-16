/****************************************************************************
 * libs/libc/unistd/lib_getpgrp.c
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

#include <unistd.h>

#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getpgrp
 *
 * Description:
 *   Get the Process Group ID of the currently executing task.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   The process group ID of the caller.  Without process groups
 *   (CONFIG_SCHED_PROCESS_GROUPS), it is the process ID of the caller.
 *
 ****************************************************************************/

pid_t getpgrp(void)
{
#ifdef CONFIG_SCHED_PROCESS_GROUPS
  return getpgid(0);
#else
  return _SCHED_GETPID();
#endif
}
