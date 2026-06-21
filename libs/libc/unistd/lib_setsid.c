/****************************************************************************
 * libs/libc/unistd/lib_setsid.c
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

#include <unistd.h>

#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setsid
 *
 * Description:
 *   Create a new session, if the calling process is not already a process
 *   group leader.  NuttX does not implement sessions or process groups, so
 *   each task is treated as its own session and process group.  This stub
 *   therefore returns the caller's PID, which acts as the session ID.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The session ID (the caller's PID) is returned.  A failure return of
 *   (pid_t)-1 with errno set to EPERM is not possible here because there is
 *   no process-group leadership to conflict with.
 *
 ****************************************************************************/

pid_t setsid(void)
{
  return _SCHED_GETPID();
}
