/****************************************************************************
 * libs/libc/unistd/lib_wait4.c
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

#include <sys/wait.h>
#include <sys/resource.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wait4
 *
 * Description:
 *   wait4() is waitpid() plus the reaped child's resource usage in one
 *   call. Built entirely on top of the existing waitpid()/getrusage()
 *   primitives (each already correctly proxied across build separation --
 *   waitpid() is a real syscall, getrusage() is plain libc), so this needs
 *   no syscall plumbing of its own and works unmodified in flat,
 *   protected, and kernel builds alike.
 *
 *   NuttX's getrusage() only supports RUSAGE_SELF/RUSAGE_CHILDREN (not a
 *   specific pid), so the returned usage is exact only when the caller has
 *   a single outstanding child at a time.
 *
 ****************************************************************************/

pid_t wait4(pid_t pid, FAR int *stat_loc, int options,
            FAR struct rusage *rusage)
{
  pid_t ret = waitpid(pid, stat_loc, options);

  if (ret > 0 && rusage != NULL)
    {
      getrusage(RUSAGE_CHILDREN, rusage);
    }

  return ret;
}
