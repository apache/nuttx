/****************************************************************************
 * libs/libc/misc/lib_getstacksize.c
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

#include <sys/resource.h>
#include <pthread.h>

#include <nuttx/lib/lib.h>
#include <nuttx/tls.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_get_stacksize
 *
 * Description:
 *   Return the default stack size of a new thread or program: the
 *   RLIMIT_STACK soft limit, kept between PTHREAD_STACK_MIN and the largest
 *   stack that a task can have, or the given size if there is no limit.
 *
 * Input Parameters:
 *   size - The stack size to use if there is no limit.
 *
 * Returned Value:
 *   The stack size in bytes.
 *
 ****************************************************************************/

size_t lib_get_stacksize(size_t size)
{
  struct rlimit rlim;

  if (getrlimit(RLIMIT_STACK, &rlim) < 0 || rlim.rlim_cur == RLIM_INFINITY)
    {
      return size;
    }

  if (rlim.rlim_cur < PTHREAD_STACK_MIN)
    {
      rlim.rlim_cur = PTHREAD_STACK_MIN;
    }

#ifdef CONFIG_TLS_ALIGNED
  if (rlim.rlim_cur > TLS_MAXSTACK)
    {
      rlim.rlim_cur = TLS_MAXSTACK;
    }
#endif

  return rlim.rlim_cur;
}
