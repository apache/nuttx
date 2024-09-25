/****************************************************************************
 * libs/libc/unistd/lib_getrlimit.c
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

#include <string.h>
#include <errno.h>

#include <sys/resource.h>
#include <pthread.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getrlimit
 *
 * Description:
 *   The getrlimit() and setrlimit() system calls get and
 *   set resource limits respectively.
 *
 ****************************************************************************/

int getrlimit(int resource, FAR struct rlimit *rlp)
{
  UNUSED(resource);

  if (rlp == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* This is a dummy realization to make the compiler happy */

  memset(rlp, 0, sizeof(*rlp));

  switch (resource)
    {
      case RLIMIT_NOFILE:
        {
          rlp->rlim_cur = OPEN_MAX;
          rlp->rlim_max = OPEN_MAX;
        }
        break;
      case RLIMIT_STACK:
        {
          rlp->rlim_cur = PTHREAD_STACK_DEFAULT;
          rlp->rlim_max = RLIM_INFINITY;
        }
        break;
      case RLIMIT_RTPRIO:
        {
          rlp->rlim_cur = SCHED_PRIORITY_DEFAULT;
          rlp->rlim_max = SCHED_PRIORITY_MAX;
        }
        break;
      case RLIMIT_RTTIME:
        {
          rlp->rlim_cur = UINT32_MAX;
          rlp->rlim_max = UINT32_MAX;
        }
        break;
      default:
        break;
    }

  return OK;
}
