/****************************************************************************
 * libs/libc/unistd/lib_getrusage.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getrusage
 *
 * Description:
 *   The getrusage() function shall provide measures of the resources used
 *   by the current process or its terminated and waited-for child processes.
 *   If the value of the who argument is RUSAGE_SELF, information shall be
 *   returned about resources used by the current process. If the value of
 *   the who argument is RUSAGE_CHILDREN, information shall be returned
 *   about resources used by the terminated and waited-for children of the
 *   current process. If the child is never waited for (for example, if the
 *   parent has SA_NOCLDWAIT set or sets SIGCHLD to SIG_IGN), the resource
 *   information for the child process is discarded and not included in the
 *   resource information provided by getrusage().
 *
 *   Note: This is currently a dummy implementation and as such does not
 *   honor the 'who' parameter.
 *
 ****************************************************************************/

int getrusage(int who, FAR struct rusage *r_usage)
{
  UNUSED(who);

  if (r_usage == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  memset(r_usage, 0, sizeof(*r_usage));
  return OK;
}
