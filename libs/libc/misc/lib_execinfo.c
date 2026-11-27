/****************************************************************************
 * libs/libc/misc/lib_execinfo.c
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

#include <execinfo.h>
#include <sched.h>
#include <unistd.h>

/* The OS code uses the macros of <execinfo.h> (__KERNEL__): this file has
 * the functions only for programs.
 */

#ifndef __KERNEL__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: backtrace
 *
 * Description:
 *   Store the return addresses of the calling thread in buffer, as the
 *   backtrace() macro of the OS code does.  Without CONFIG_SCHED_BACKTRACE
 *   there are none.
 *
 * Input Parameters:
 *   buffer - Where the addresses go
 *   size   - The number of entries of buffer
 *
 * Returned Value:
 *   The number of addresses stored in buffer.
 *
 ****************************************************************************/

int backtrace(FAR void **buffer, int size)
{
  return sched_backtrace(gettid(), buffer, size, 1);
}

/****************************************************************************
 * Name: dump_stack
 *
 * Description:
 *   Write the backtrace of the calling thread to the system log.
 *
 ****************************************************************************/

void dump_stack(void)
{
  sched_dumpstack(gettid());
}

#endif /* !__KERNEL__ */
