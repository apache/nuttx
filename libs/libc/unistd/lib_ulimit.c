/****************************************************************************
 * libs/libc/unistd/lib_ulimit.c
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

#include <errno.h>
#include <ulimit.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ulimit
 *
 * Description:
 *   The ulimit will make calls to getrlimit() and setrlimit() to perform
 *   get and set resource limits respectively.
 *
 * Returned value:
 *   On success, return a non-negative value.
 *   On failure, return -1 and set the errno value.
 *
 ****************************************************************************/

long ulimit(int cmd, long newlimit)
{
  long ret = ERROR;

  switch (cmd)
    {
      case UL_GETFSIZE:
        {
          struct rlimit rlp;
          getrlimit(RLIMIT_FSIZE, &rlp);
          ret = rlp.rlim_cur / 512UL;
        }
        break;
      case UL_SETFSIZE:
        {
          struct rlimit rlp;
          rlp.rlim_max = RLIM_INFINITY;
          rlp.rlim_cur = newlimit * 512UL;
          ret = setrlimit(RLIMIT_FSIZE, &rlp);
        }
        break;
      default:
        set_errno(EINVAL);
        break;
    }

  return ret;
}
