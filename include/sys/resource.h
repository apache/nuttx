/****************************************************************************
 * include/sys/resource.h
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

/* Possible values for the resource argument of getrlimit() and setrlimit() */

#define RLIMIT_CORE     1           /* Limit on size of core dump file */
#define RLIMIT_CPU      2           /* Limit on CPU time per process. */
#define RLIMIT_DATA     3           /* Limit on data segment size */
#define RLIMIT_FSIZE    4           /* Limit on file size */
#define RLIMIT_NOFILE   5           /* Limit on number of open files */
#define RLIMIT_STACK    6           /* Limit on stack size */
#define RLIMIT_AS       7           /* Limit on address space size */

#if defined(CONFIG_FS_LARGEFILE) && defined(CONFIG_HAVE_LONG_LONG)
#  define RLIM_INFINITY    UINT64_MAX /* No limit */
#  define RLIM_SAVED_MAX   UINT64_MAX /* Unrepresentable saved hard limit */
#  define RLIM_SAVED_CUR   UINT64_MAX /* Unrepresentable saved soft limit */

#  define RLIM64_INFINITY  RLIM_INFINITY
#  define RLIM64_SAVED_MAX RLIM_SAVED_MAX
#  define RLIM64_SAVED_CUR RLIM_SAVED_CUR

#  define getrlimit64      getrlimit
#  define setrlimit64      setrlimit
#  define prlimit64        prlimit

#  define rlimit64         rlimit
#  define rlim64_t         rlim_t
#else
/* The following symbolic constants are defined.  Each is a value of type
 * rlim_t.
 *
 * On implementations where all resource limits are representable in an
 * object of type rlim_t, RLIM_SAVED_MAX and RLIM_SAVED_CUR need not be
 * distinct from RLIM_INFINITY.
 */

#  define RLIM_INFINITY  UINT32_MAX /* No limit */
#  define RLIM_SAVED_MAX UINT32_MAX /* Unrepresentable saved hard limit */
#  define RLIM_SAVED_CUR UINT32_MAX /* Unrepresentable saved soft limit */
#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* All resource limits are represented with this type.
 * It must be an unsigned integral type.
 */

#if defined(CONFIG_FS_LARGEFILE) && defined(CONFIG_HAVE_LONG_LONG)
typedef uint64_t rlim_t;
#else
typedef uint32_t rlim_t;
#endif

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

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int  getpriority(int which, id_t who);
int  getrlimit(int resource, FAR struct rlimit *rlp);
int  getrusage(int who, FAR struct rusage *r_usage);
int  setpriority(int which, id_t who, int value);
int  setrlimit(int resource, FAR const struct rlimit *rlp);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_RESOURCE_H */
