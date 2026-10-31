/****************************************************************************
 * include/limits.h
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

#ifndef __INCLUDE_LIMITS_H
#define __INCLUDE_LIMITS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Architecture specific limits */

#include <arch/limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default values for user configurable limits ******************************/

/* Maximum number of bytes in a filename (not including terminating null). */

#ifndef CONFIG_NAME_MAX
#  define CONFIG_NAME_MAX 32
#endif

/* Maximum number of bytes in a pathname, including the terminating null
 * character.
 */

#ifndef CONFIG_PATH_MAX
#  define CONFIG_PATH_MAX 256
#endif

/* Maximum length of any multibyte character in any locale.
 * We define this value here since the gcc header does not define
 * the correct value.
 */

#define MB_LEN_MAX            4

/* Invariant minimum values required by POSIX *******************************/

#define _POSIX_AIO_LISTIO_MAX                   2
#define _POSIX_AIO_MAX                          1
#define _POSIX_ARG_MAX                          4096
#define _POSIX_CHILD_MAX                        25
#define _POSIX_CLOCKRES_MIN                     20000000
#define _POSIX_DELAYTIMER_MAX                   32
#define _POSIX_HOST_NAME_MAX                    255
#define _POSIX_LINK_MAX                         8
#define _POSIX_LOGIN_NAME_MAX                   9
#define _POSIX_MAX_CANON                        255
#define _POSIX_MAX_INPUT                        255
#define _POSIX_MQ_OPEN_MAX                      8
#define _POSIX_MQ_PRIO_MAX                      32
#define _POSIX_NAME_MAX                         14
#define _POSIX_NGROUPS_MAX                      8
#define _POSIX_OPEN_MAX                         20
#define _POSIX_PATH_MAX                         256
#define _POSIX_PIPE_BUF                         512
#define _POSIX_RE_DUP_MAX                       255
#define _POSIX_RTSIG_MAX                        8
#define _POSIX_SEM_NSEMS_MAX                    256
#define _POSIX_SEM_VALUE_MAX                    32767
#define _POSIX_SIGQUEUE_MAX                     32
#define _POSIX_STREAM_MAX                       8
#define _POSIX_SYMLINK_MAX                      255
#define _POSIX_SYMLOOP_MAX                      8
#define _POSIX_THREAD_DESTRUCTOR_ITERATIONS     4
#define _POSIX_THREAD_KEYS_MAX                  128
#define _POSIX_THREAD_THREADS_MAX               64
#define _POSIX_TIMER_MAX                        32
#define _POSIX_TTY_NAME_MAX                     9
#define _POSIX_TZNAME_MAX                       6

#define _POSIX2_BC_BASE_MAX                     99
#define _POSIX2_BC_DIM_MAX                      2048
#define _POSIX2_BC_SCALE_MAX                    99
#define _POSIX2_BC_STRING_MAX                   1000
#define _POSIX2_CHARCLASS_NAME_MAX              14
#define _POSIX2_COLL_WEIGHTS_MAX                2
#define _POSIX2_EXPR_NEST_MAX                   32
#define _POSIX2_LINE_MAX                        2048
#define _POSIX2_RE_DUP_MAX                      255

#ifdef CONFIG_SMALL_MEMORY

#define _POSIX_SIZE_MAX                         65535        /* See sys/types.h */
#define _POSIX_SIZE_MIN                         0

#define _POSIX_SSIZE_MAX                        32767        /* See sys/types.h */
#define _POSIX_SSIZE_MIN                        -32768

#else /* CONFIG_SMALL_MEMORY */

#define _POSIX_SIZE_MAX                         ULONG_MAX
#define _POSIX_SIZE_MIN                         0

#define _POSIX_SSIZE_MAX                        LONG_MAX
#define _POSIX_SSIZE_MIN                        LONG_MIN

#endif /* CONFIG_SMALL_MEMORY */

/* Numerical limits.  These values may be increased from the POSIX minimum
 * values above or made indeterminate
 */

#ifdef CONFIG_TASK_ARG_MAX
#  define ARG_MAX                               CONFIG_TASK_ARG_MAX
#else
#  define ARG_MAX                               _POSIX_ARG_MAX
#endif

#define CHILD_MAX                               _POSIX_CHILD_MAX

#ifdef CONFIG_LINE_MAX
#  define LINE_MAX                              CONFIG_LINE_MAX
#else
#  define LINE_MAX                              _POSIX2_LINE_MAX
#endif

#define LINK_MAX                                128
#define MAX_CANON                               _POSIX_MAX_CANON
#define MAX_INPUT                               _POSIX_MAX_INPUT
#define NAME_MAX                                CONFIG_NAME_MAX
#define TTY_NAME_MAX                            NAME_MAX
#define LOGIN_NAME_MAX                          (NAME_MAX + 1)

#if defined(CONFIG_SCHED_NGROUPS) && CONFIG_SCHED_NGROUPS > 0
#  define NGROUPS_MAX                           CONFIG_SCHED_NGROUPS
#else
#  define NGROUPS_MAX                           0
#endif

#if defined(CONFIG_LIBC_OPEN_MAX) && CONFIG_LIBC_OPEN_MAX > 0
#  define OPEN_MAX                              CONFIG_LIBC_OPEN_MAX
#else
#  define OPEN_MAX                              _POSIX_OPEN_MAX
#endif

#define PATH_MAX                                CONFIG_PATH_MAX
#define PIPE_BUF                                _POSIX_PIPE_BUF
#define SIZE_MAX                                _POSIX_SIZE_MAX
#define SIZE_MIN                                _POSIX_SIZE_MIN
#define RSIZE_MAX                               _POSIX_SIZE_MAX
#define SSIZE_MAX                               _POSIX_SSIZE_MAX
#define SSIZE_MIN                               _POSIX_SSIZE_MIN
#define STREAM_MAX                              OPEN_MAX
#define TZNAME_MAX                              _POSIX_TZNAME_MAX
#define TZ_MAX_TIMES                            CONFIG_LIBC_TZ_MAX_TIMES
#define TZ_MAX_TYPES                            CONFIG_LIBC_TZ_MAX_TYPES

#define RTSIG_MAX                               32
#define SIGQUEUE_MAX                            _POSIX_SIGQUEUE_MAX

#define SYMLINK_MAX                             _POSIX_SYMLINK_MAX
#define SYMLOOP_MAX                             _POSIX_SYMLOOP_MAX

#define DELAYTIMER_MAX                          _POSIX_DELAYTIMER_MAX
#define TIMER_MAX                               _POSIX_TIMER_MAX

#ifdef CONFIG_USEC_PER_TICK
#  define CLOCKRES_MIN                          ((CONFIG_USEC_PER_TICK)*1000)
#else
#  define CLOCKRES_MIN                          (10*1000000)
#endif

#ifdef INT64_MAX
#  define CLOCK_MAX                             INT64_MAX
#else
#  define CLOCK_MAX                             LLONG_MAX
#endif

/* Other invariant values */

/* CHARCLASS_NAME_MAX
 *   Maximum number of bytes in a character class name. Minimum Acceptable
 *   Value: 14
 */

#define CHARCLASS_NAME_MAX 14

/* Maximum value of digit in calls to the printf() and scanf() functions.
 * Minimum Acceptable Value: 9
 */

#ifdef CONFIG_LIBC_NUMBERED_ARGS
#  ifdef CONFIG_LIBC_NL_ARGMAX
#    define NL_ARGMAX CONFIG_LIBC_NL_ARGMAX
#  else
#    define NL_ARGMAX 9
#  endif
#endif

/* NL_LANGMAX
 *    Maximum number of bytes in a LANG name. Minimum Acceptable Value: 14
 */

#define NL_LANGMAX 14

/* NL_MSGMAX
 *    Maximum message number. Minimum Acceptable Value: 32 67
 */

#define NL_MSGMAX 32767

/* NL_NMAX
 *   Maximum number of bytes in an N-to-1 collation mapping. Minimum
 *   Acceptable Value: *
 */

/* NL_SETMAX
 *   Maximum set number. Minimum Acceptable Value: 255
 */

#define NL_SETMAX 255

/* NL_TEXTMAX
 *   Maximum number of bytes in a message string. Minimum Acceptable Value:
 *   _POSIX2_LINE_MAX
 */

#define NL_TEXTMAX                              _POSIX2_LINE_MAX

/* NZERO
 *   Default process priority. Minimum Acceptable Value: 100
 */

#define NZERO                                   SCHED_PRIORITY_DEFAULT

/* Required for asynchronous I/O */

#ifdef CONFIG_FS_AIO_LISTIO_MAX
#  define AIO_LISTIO_MAX                        CONFIG_FS_AIO_LISTIO_MAX
#else
#  define AIO_LISTIO_MAX                        _POSIX_AIO_LISTIO_MAX
#endif
#define AIO_MAX                                 _POSIX_AIO_MAX

/* Required for POSIX message passing */

#define MQ_OPEN_MAX                             8
#define MQ_PRIO_MAX                             UCHAR_MAX

/* Required for POSIX semaphores */

#define SEM_NSEMS_MAX                           INT_MAX
#define SEM_VALUE_MAX                           INT_MAX

/* Required for POSIX pthread management.  POSIX requires at least
 * _POSIX_THREAD_KEYS_MAX keys: set CONFIG_TLS_NELEM to 128 or more.
 */

#ifndef PTHREAD_KEYS_MAX
#  define PTHREAD_KEYS_MAX                      CONFIG_TLS_NELEM
#endif

#define PTHREAD_DESTRUCTOR_ITERATIONS _POSIX_THREAD_DESTRUCTOR_ITERATIONS

/* Required for readv() and writev() */

/* There really is no upper limit on the number of vectors */

#define IOV_MAX        INT_MAX

#define HOST_NAME_MAX  CONFIG_HOST_NAME_MAX

/* ptrdiff_t limits */

#define PTRDIFF_MAX PTR_MAX
#define PTRDIFF_MIN PTR_MIN

#endif /* __INCLUDE_LIMITS_H */
