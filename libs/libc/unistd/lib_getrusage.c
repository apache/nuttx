/****************************************************************************
 * libs/libc/unistd/lib_getrusage.c
 *
 *   Copyright (C) 2019 Xiaomi Inc. All rights reserved.
 *   Author:  Xiang Xiao <xiaoxiang@xiaomi.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 ****************************************************************************/

int getrusage(int who, FAR struct rusage *r_usage)
{
  if (r_usage == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  memset(r_usage, 0, sizeof(*r_usage));
  return OK;
}
