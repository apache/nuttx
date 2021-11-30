/********************************************************************************
 * include/limits.h
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
 ********************************************************************************/

#ifndef __INCLUDE_LIMITS_H
#define __INCLUDE_LIMITS_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

/* Architecture specific limits */

#include <arch/limits.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/* Default values for user configurable limits **********************************/

/* Maximum number of bytes in a filename (not including terminating null). */

#ifndef CONFIG_NAME_MAX
#  define CONFIG_NAME_MAX 32
#endif

/* Maximum number of bytes in a pathname, including the terminating null
 * character.
 */

#ifndef CONFIG_PATH_MAX
#  if CONFIG_NAME_MAX < 64
#    define CONFIG_PATH_MAX (4*CONFIG_NAME_MAX + 1)
#  else
#    define CONFIG_PATH_MAX 256
#  endif
#endif

/* Maximum length of any multibyte character in any locale.
 * We define this value here since the gcc header does not define
 * the correct value.
 */

#define MB_LEN_MAX            1

/* Configurable limits required by POSIX ****************************************
 *
 * Required for all implementations:
 *
 *   _POSIX_ARG_MAX        Total length of string arguments
 *   _POSIX_CHILD_MAX      Number of child tasks active
 *   _POSIX_LINK_MAX       The number of links a file can have
 *   _POSIX_MAX_CANON      Number bytes in TTY canonical input queue
 *   _POSIX_MAX_INPUT      Number bytes in TTY canonical input queue
 *   _POSIX_NAME_MAX       Number of bytes in a file or pathname component
 *   _POSIX_NGROUPS_MAX    Number supplementary group IDs
 *   _POSIX_OPEN_MAX       Number of files a task can have open at once
 *   _POSIX_PATH_MAX       Number of bytes in a full pathname (including NULL)
 *   _POSIX_PIPE_BUF       Number of bytes for atomic write into pipe
 *   _POSIX_SSIZE_MAX      Largest filesystem write; also max value of ssize_t
 *   _POSIX_STREAM_MAX     Number of std I/O streams open at once
 *   _POSIX_TZNAME_MAX     Max number of bytes of a timezone name
 *
 * Required for sigqueue
 *
 *   _POSIX_RTSIG_MAX      Difference between SIGRTMIN and SIGRTMAX
 *   _POSIX_SIGQUEUE_MAX   Max number signals a task can queue
 *
 * Required for POSIX timers
 *
 *   _POSIX_DELAYTIMER_MAX Max number timer overruns
 *   _POSIX_TIMER_MAX      Max number of timers per task
 *   _POSIX_CLOCKRES_MIN   Clock resolution in nanoseconds
 *
 * Required for asynchronous I/O
 *
 *   _POSIX_AIO_LISTIO_MAX Max number of AIOs in single listio call
 *   _POSIX_AIO_MAX        Max number of simultaneous AIO operations
 *
 * Required for POSIX message passing
 *
 *   _POSIX_MQ_OPEN_MAX    Max number message queues task may open (mq_open)
 *   _POSIX_MQ_PRIO_MAX    Max message priority (mq_send)
 *
 * Required for POSIX semaphores
 *
 *   _POSIX_SEM_NSEMS_MAX  Max number of open semaphores per task
 *   _POSIX_SEM_VALUE_MAX  Max value a semaphore may have
 *
 * Required for symbolic links
 *   _POSIX_SYMLOOP_MAX   Maximum number of symbolic links that can be
 *                        reliably traversed in the resolution of a pathname
 *                        in the absence of a loop.
 *
 */

#define _POSIX_ARG_MAX        4096
#define _POSIX_CHILD_MAX      6
#define _POSIX_LINK_MAX       8
#define _POSIX_MAX_CANON      255
#define _POSIX_MAX_INPUT      255
#define _POSIX_NAME_MAX       CONFIG_NAME_MAX
#define _POSIX_NGROUPS_MAX    0
#define _POSIX_OPEN_MAX       16
#define _POSIX_PATH_MAX       CONFIG_PATH_MAX
#define _POSIX_PIPE_BUF       512
#define _POSIX_STREAM_MAX     16
#define _POSIX_TZNAME_MAX     3

#ifdef CONFIG_SMALL_MEMORY

#define _POSIX_SIZE_MAX       65535        /* See sys/types.h */
#define _POSIX_SIZE_MIN       0

#define _POSIX_SSIZE_MAX      32767        /* See sys/types.h */
#define _POSIX_SSIZE_MIN      -32768

#else /* CONFIG_SMALL_MEMORY */

#define _POSIX_SIZE_MAX       4294967295UL /* See sys/types.h */
#define _POSIX_SIZE_MIN       0

#define _POSIX_SSIZE_MAX      2147483647L  /* See sys/types.h */
#define _POSIX_SSIZE_MIN      -2147483648L

#endif /* CONFIG_SMALL_MEMORY */

/* Required for sigqueue */

#define _POSIX_RTSIG_MAX      31
#define _POSIX_SIGQUEUE_MAX   32

/* Required for symbolic links */

#define _POSIX_SYMLOOP_MAX    8

/* Required for POSIX timers.
 *
 * _POSIX_DELAYTIMER_MAX is the number of timer expiration overruns.
 *
 * _POSIX_TIMER_MAX is the per-process number of timers.
 *
 * _POSIX_CLOCKRES_MIN is the resolution of the CLOCK_REALTIME clock in
 *    nanoseconds.  CLOCK_REALTIME is controlled by the NuttX system time.
 *    The default value is the system timer which has a resolution of 1000
 *    microseconds.  This default setting can be overridden by defining the
 *    clock interval in microseconds as CONFIG_USEC_PER_TICK in the NuttX
 *    configuration file.
 */

#define _POSIX_DELAYTIMER_MAX 32
#define _POSIX_TIMER_MAX      32

#ifdef CONFIG_USEC_PER_TICK
# define _POSIX_CLOCKRES_MIN  ((CONFIG_USEC_PER_TICK)*1000)
#else
# define _POSIX_CLOCKRES_MIN  (10*1000000)
#endif

/* Required for asynchronous I/O */

#define _POSIX_AIO_LISTIO_MAX 2
#define _POSIX_AIO_MAX        1

/* Required for POSIX message passing */

#define _POSIX_MQ_OPEN_MAX    8
#define _POSIX_MQ_PRIO_MAX    UCHAR_MAX

/* Required for POSIX semaphores */

#define _POSIX_SEM_NSEMS_MAX  INT_MAX
#define _POSIX_SEM_VALUE_MAX  0x7fff

/* Numerical limits.  These values may be increased from the POSIX minimum
 * values above or made indeterminate
 */

#define ARG_MAX        _POSIX_ARG_MAX
#define CHILD_MAX      _POSIX_CHILD_MAX
#define LINK_MAX       _POSIX_LINK_MAX
#define MAX_CANON      _POSIX_MAX_CANON
#define MAX_INPUT      _POSIX_MAX_INPUT
#define NAME_MAX       _POSIX_NAME_MAX
#define TTY_NAME_MAX   _POSIX_NAME_MAX
#define NGROUPS_MAX    _POSIX_NGROUPS_MAX
#define OPEN_MAX       _POSIX_OPEN_MAX
#define PATH_MAX       _POSIX_PATH_MAX
#define PIPE_BUF       _POSIX_PIPE_BUF
#define SIZE_MAX       _POSIX_SIZE_MAX
#define SIZE_MIN       _POSIX_SIZE_MIN
#define RSIZE_MAX      _POSIX_SIZE_MAX
#define SSIZE_MAX      _POSIX_SSIZE_MAX
#define SSIZE_MIN      _POSIX_SSIZE_MIN
#define STREAM_MAX     _POSIX_STREAM_MAX
#define TZNAME_MAX     _POSIX_TZNAME_MAX
#define TZ_MAX_TIMES   CONFIG_LIBC_TZ_MAX_TIMES
#define TZ_MAX_TYPES   CONFIG_LIBC_TZ_MAX_TYPES

#define RTSIG_MAX      _POSIX_RTSIG_MAX
#define SIGQUEUE_MAX   _POSIX_SIGQUEUE_MAX

#define SYMLOOP_MAX    _POSIX_SYMLOOP_MAX

#define DELAYTIMER_MAX _POSIX_DELAYTIMER_MAX
#define TIMER_MAX      _POSIX_TIMER_MAX
#define CLOCKRES_MIN   _POSIX_CLOCKRES_MIN

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

#define NL_TEXTMAX _POSIX2_LINE_MAX

/* NZERO
 *   Default process priority. Minimum Acceptable Value: 128
 */

#define NZERO 128

/* Required for asynchronous I/O */

#define AIO_LISTIO_MAX _POSIX_AIO_LISTIO_MAX
#define AIO_MAX        _POSIX_AIO_MAX

/* Required for POSIX message passing */

#define MQ_OPEN_MAX    _POSIX_MQ_OPEN_MAX
#define MQ_PRIO_MAX    _POSIX_MQ_PRIO_MAX

/* Required for POSIX semaphores */

#define SEM_NSEMS_MAX  _POSIX_SEM_NSEMS_MAX
#define SEM_VALUE_MAX  _POSIX_SEM_VALUE_MAX

/* Required for readv() and writev() */

/* There really is no upper limit on the number of vectors */

#define IOV_MAX        INT_MAX

#define HOST_NAME_MAX  32

#endif /* __INCLUDE_LIMITS_H */
