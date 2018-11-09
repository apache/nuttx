/****************************************************************************
 * include/sys/resource.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_SYS_RESOURCE_H
#define __INCLUDE_SYS_RESOURCE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sys/time.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Possible values of the 'which' argument of getpriority() and
 * setpriority()
 */

#define PRIO_PROCESS    1           /* 'who' argument is a process ID */
#define PRIO_PGRP       2           /* 'who' argument is a process group ID */
#define PRIO_USER       3           /* 'who' argument is a user ID */

/* Possible values of the 'who' parameter of getrusage(): */

#define RUSAGE_SELF     0           /* Returns information about the current
                                     * process */
#define RUSAGE_CHILDREN 1           /* Returns information about children of
                                     * the current process */

/* Possible values for the resource argument of getrlimit() and setrlimit(): */

#define RLIMIT_CORE     1           /* Limit on size of core dump file */
#define RLIMIT_CPU      2           /* Limit on CPU time per process. */
#define RLIMIT_DATA     3           /* Limit on data segment size */
#define RLIMIT_FSIZE    4           /* Limit on file size */
#define RLIMIT_NOFILE   5           /* Limit on number of open files */
#define RLIMIT_STACK    6           /* Limit on stack size */
#define RLIMIT_AS       7           /* Limit on address space size */

/* The following symbolic constants are defined.  Each is a value of type
 * rlim_t.
 *
 * On implementations where all resource limits are representable in an
 * object of type rlim_t, RLIM_SAVED_MAX and RLIM_SAVED_CUR need not be
 * distinct from RLIM_INFINITY.
 */

#define RLIM_INFINITY   UINT32_MAX  /* No limit */
#define RLIM_SAVED_MAX  UINT32_MAX  /* Unrepresentable saved hard limit */
#define RLIM_SAVED_CUR  UINT32_MAX  /* Unrepresentable saved soft limit */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* All resource limits are represented with this type.  It must be an unsigned
 * integral type.
 */

typedef uint32_t rlim_t;

/* Minimal, compliant rlimit structure */

struct rlimit
{
  rlim_t rlim_cur;          /* The current (soft) limit */
  rlim_t rlim_max;          /* The hard limit */
};

/* Minimal, compliant rusage structure */

struct rusage
{
  struct timeval ru_utime;  /* User time used */
  struct timeval ru_stime;  /* System time used */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int  getpriority(int which, id_t who);
int  getrlimit(int resource, FAR struct rlimit *rlp);
int  getrusage(int who, FAR struct rusage *r_usage);
int  setpriority(int which, id_t who, int value);
int  setrlimit(int resource, FAR const struct rlimit *rlp);

#endif /* __INCLUDE_SYS_RESOURCE_H */
