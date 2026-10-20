/****************************************************************************
 * libs/libc/unistd/lib_sysconf.c
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

#include <nuttx/atexit.h>

#include <sys/resource.h>
#include <sys/sysinfo.h>
#include <unistd.h>
#include <limits.h>
#include <sched.h>
#include <errno.h>

#ifdef CONFIG_MM_PGSIZE
#  define DEFAULT_MM_PGSIZE CONFIG_MM_PGSIZE
#else
#  define DEFAULT_MM_PGSIZE CONFIG_PTHREAD_STACK_MIN
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sysconf
 *
 * Description:
 *   The sysconf() function provides a method for the application to
 *   determine the current value of a configurable system limit or option
 *   (variable).  The implementation will support all of the variables
 *   listed in the following table and may support others.
 *
 *   The 'name' argument represents the system variable to be queried. The
 *   following table lists the minimal set of system variables from
 *   <limits.h> or <unistd.h> that can be returned by sysconf(), and the
 *   symbolic constants defined in <unistd.h> that are the corresponding
 *   values used for name.
 *
 *     Variable                            Value of Name
 *
 *     {AIO_LISTIO_MAX}                    _SC_AIO_LISTIO_MAX
 *     {AIO_MAX}                           _SC_AIO_MAX
 *     {AIO_PRIO_DELTA_MAX}                _SC_AIO_PRIO_DELTA_MAX
 *     {ARG_MAX}                           _SC_ARG_MAX
 *     {ATEXIT_MAX}                        _SC_ATEXIT_MAX
 *     {BC_BASE_MAX}                       _SC_BC_BASE_MAX
 *     {BC_DIM_MAX}                        _SC_BC_DIM_MAX
 *     {BC_SCALE_MAX}                      _SC_BC_SCALE_MAX
 *     {BC_STRING_MAX}                     _SC_BC_STRING_MAX
 *     {CHILD_MAX}                         _SC_CHILD_MAX
 *     Clock ticks/second                  _SC_CLK_TCK
 *     {COLL_WEIGHTS_MAX}                  _SC_COLL_WEIGHTS_MAX
 *     {DELAYTIMER_MAX}                    _SC_DELAYTIMER_MAX
 *     {EXPR_NEST_MAX}                     _SC_EXPR_NEST_MAX
 *     {HOST_NAME_MAX}                     _SC_HOST_NAME_MAX
 *     {IOV_MAX}                           _SC_IOV_MAX
 *     {LINE_MAX}                          _SC_LINE_MAX
 *     {LOGIN_NAME_MAX}                    _SC_LOGIN_NAME_MAX
 *     {NGROUPS_MAX}                       _SC_NGROUPS_MAX
 *     Maximum size of getgrgid_r() and    _SC_GETGR_R_SIZE_MAX
 *     getgrnam_r() data buffers
 *     Maximum size of getpwuid_r() and    _SC_GETPW_R_SIZE_MAX
 *     getpwnam_r() data buffers
 *     {MQ_OPEN_MAX}                       _SC_MQ_OPEN_MAX
 *     {MQ_PRIO_MAX}                       _SC_MQ_PRIO_MAX
 *     {OPEN_MAX}                          _SC_OPEN_MAX
 *     _POSIX_ADVISORY_INFO                _SC_ADVISORY_INFO
 *     _POSIX_BARRIERS                     _SC_BARRIERS
 *     _POSIX_ASYNCHRONOUS_IO              _SC_ASYNCHRONOUS_IO
 *     _POSIX_CLOCK_SELECTION              _SC_CLOCK_SELECTION
 *     _POSIX_CPUTIME                      _SC_CPUTIME
 *     _POSIX_FSYNC                        _SC_FSYNC
 *     _POSIX_IPV6                         _SC_IPV6
 *     _POSIX_JOB_CONTROL                  _SC_JOB_CONTROL
 *     _POSIX_MAPPED_FILES                 _SC_MAPPED_FILES
 *     _POSIX_MEMLOCK                      _SC_MEMLOCK
 *     _POSIX_MEMLOCK_RANGE                _SC_MEMLOCK_RANGE
 *     _POSIX_MEMORY_PROTECTION            _SC_MEMORY_PROTECTION
 *     _POSIX_MESSAGE_PASSING              _SC_MESSAGE_PASSING
 *     _POSIX_MONOTONIC_CLOCK              _SC_MONOTONIC_CLOCK
 *     _POSIX_PRIORITIZED_IO               _SC_PRIORITIZED_IO
 *     _POSIX_PRIORITY_SCHEDULING          _SC_PRIORITY_SCHEDULING
 *     _POSIX_RAW_SOCKETS                  _SC_RAW_SOCKETS
 *     _POSIX_READER_WRITER_LOCKS          _SC_READER_WRITER_LOCKS
 *     _POSIX_REALTIME_SIGNALS             _SC_REALTIME_SIGNALS
 *     _POSIX_REGEXP                       _SC_REGEXP
 *     _POSIX_SAVED_IDS                    _SC_SAVED_IDS
 *     _POSIX_SEMAPHORES                   _SC_SEMAPHORES
 *     _POSIX_SHARED_MEMORY_OBJECTS        _SC_SHARED_MEMORY_OBJECTS
 *     _POSIX_SHELL                        _SC_SHELL
 *     _POSIX_SPAWN                        _SC_SPAWN
 *     _POSIX_SPIN_LOCKS                   _SC_SPIN_LOCKS
 *     _POSIX_SPORADIC_SERVER              _SC_SPORADIC_SERVER
 *     _POSIX_SS_REPL_MAX                  _SC_SS_REPL_MAX
 *     _POSIX_SYNCHRONIZED_IO              _SC_SYNCHRONIZED_IO
 *     _POSIX_THREAD_ATTR_STACKADDR        _SC_THREAD_ATTR_STACKADDR
 *     _POSIX_THREAD_ATTR_STACKSIZE        _SC_THREAD_ATTR_STACKSIZE
 *     _POSIX_THREAD_CPUTIME               _SC_THREAD_CPUTIME
 *     _POSIX_THREAD_PRIO_INHERIT          _SC_THREAD_PRIO_INHERIT
 *     _POSIX_THREAD_PRIO_PROTECT          _SC_THREAD_PRIO_PROTECT
 *     _POSIX_THREAD_PRIORITY_SCHEDULING   _SC_THREAD_PRIORITY_SCHEDULING
 *     _POSIX_THREAD_PROCESS_SHARED        _SC_THREAD_PROCESS_SHARED
 *     _POSIX_THREAD_SAFE_FUNCTIONS        _SC_THREAD_SAFE_FUNCTIONS
 *     _POSIX_THREAD_SPORADIC_SERVER       _SC_THREAD_SPORADIC_SERVER
 *     _POSIX_THREADS                      _SC_THREADS
 *     _POSIX_TIMEOUTS                     _SC_TIMEOUTS
 *     _POSIX_TIMERS                       _SC_TIMERS
 *     _POSIX_TRACE                        _SC_TRACE
 *     _POSIX_TRACE_EVENT_FILTER           _SC_TRACE_EVENT_FILTER
 *     _POSIX_TRACE_EVENT_NAME_MAX         _SC_TRACE_EVENT_NAME_MAX
 *     _POSIX_TRACE_INHERIT                _SC_TRACE_INHERIT
 *     _POSIX_TRACE_LOG                    _SC_TRACE_LOG
 *     _POSIX_TRACE_NAME_MAX               _SC_TRACE_NAME_MAX
 *     _POSIX_TRACE_SYS_MAX                _SC_TRACE_SYS_MAX
 *     _POSIX_TRACE_USER_EVENT_MAX         _SC_TRACE_USER_EVENT_MAX
 *     _POSIX_TYPED_MEMORY_OBJECTS         _SC_TYPED_MEMORY_OBJECTS
 *     _POSIX_VERSION                      _SC_VERSION
 *     _POSIX_V6_ILP32_OFF32               _SC_V6_ILP32_OFF32
 *     _POSIX_V6_ILP32_OFFBIG              _SC_V6_ILP32_OFFBIG
 *     _POSIX_V6_LP64_OFF64                _SC_V6_LP64_OFF64
 *     _POSIX_V6_LPBIG_OFFBIG              _SC_V6_LPBIG_OFFBIG
 *     _POSIX2_C_BIND                      _SC_2_C_BIND
 *     _POSIX2_C_DEV                       _SC_2_C_DEV
 *     _POSIX2_CHAR_TERM                   _SC_2_CHAR_TERM
 *     _POSIX2_FORT_DEV                    _SC_2_FORT_DEV
 *     _POSIX2_FORT_RUN                    _SC_2_FORT_RUN
 *     _POSIX2_LOCALEDEF                   _SC_2_LOCALEDEF
 *     _POSIX2_PBS                         _SC_2_PBS
 *     _POSIX2_PBS_ACCOUNTING              _SC_2_PBS_ACCOUNTING
 *     _POSIX2_PBS_CHECKPOINT              _SC_2_PBS_CHECKPOINT
 *     _POSIX2_PBS_LOCATE                  _SC_2_PBS_LOCATE
 *     _POSIX2_PBS_MESSAGE                 _SC_2_PBS_MESSAGE
 *     _POSIX2_PBS_TRACK                   _SC_2_PBS_TRACK
 *     _POSIX2_SW_DEV                      _SC_2_SW_DEV
 *     _POSIX2_UPE                         _SC_2_UPE
 *     _POSIX2_VERSION                     _SC_2_VERSION
 *     {PAGE_SIZE}                         _SC_PAGE_SIZE
 *     {PAGESIZE}                          _SC_PAGESIZE
 *     _POSIX_THREAD_DESTRUCTOR_ITERATIONS _SC_THREAD_DESTRUCTOR_ITERATIONS
 *     _POSIX_THREAD_KEYS_MAX              _SC_THREAD_KEYS_MAX
 *     {PTHREAD_STACK_MIN}                 _SC_THREAD_STACK_MIN
 *     _POSIX_THREAD_THREADS_MAX           _SC_THREAD_THREADS_MAX
 *     {RE_DUP_MAX}                        _SC_RE_DUP_MAX
 *     {RTSIG_MAX}                         _SC_RTSIG_MAX
 *     {SEM_NSEMS_MAX}                     _SC_SEM_NSEMS_MAX
 *     {SEM_VALUE_MAX}                     _SC_SEM_VALUE_MAX
 *     {SIGQUEUE_MAX}                      _SC_SIGQUEUE_MAX
 *     {STREAM_MAX}                        _SC_STREAM_MAX
 *     {SYMLOOP_MAX}                       _SC_SYMLOOP_MAX
 *     {TIMER_MAX}                         _SC_TIMER_MAX
 *     {TTY_NAME_MAX}                      _SC_TTY_NAME_MAX
 *     {TZNAME_MAX}                        _SC_TZNAME_MAX
 *     _XBS5_ILP32_OFF32 (LEGACY)          _SC_XBS5_ILP32_OFF32 (LEGACY)
 *     _XBS5_ILP32_OFFBIG (LEGACY)         _SC_XBS5_ILP32_OFFBIG (LEGACY)
 *     _XBS5_LP64_OFF64 (LEGACY)           _SC_XBS5_LP64_OFF64 (LEGACY)
 *     _XBS5_LPBIG_OFFBIG (LEGACY)         _SC_XBS5_LPBIG_OFFBIG (LEGACY)
 *     _XOPEN_CRYPT                        _SC_XOPEN_CRYPT
 *     _XOPEN_ENH_I18N                     _SC_XOPEN_ENH_I18N
 *     _XOPEN_LEGACY                       _SC_XOPEN_LEGACY
 *     _XOPEN_REALTIME                     _SC_XOPEN_REALTIME
 *     _XOPEN_REALTIME_THREADS             _SC_XOPEN_REALTIME_THREADS
 *     _XOPEN_SHM                          _SC_XOPEN_SHM
 *     _XOPEN_STREAMS                      _SC_XOPEN_STREAMS
 *     _XOPEN_UNIX                         _SC_XOPEN_UNIX
 *     _XOPEN_VERSION                      _SC_XOPEN_VERSION
 *
 * Returned Value:
 *   If name is an invalid value, sysconf() will return -1 and set errno to
 *   EINVAL to indicate the error. If the variable corresponding to name has
 *   no limit, sysconf() will return -1 without changing the value of errno.
 *    Note that indefinite limits do not imply infinite limits; see
 *   <limits.h>.
 *
 *   Otherwise, sysconf() will return the current variable value on the
 *   system.  The value returned will not be more restrictive than the
 *   corresponding value described to the application when it was compiled
 *   with the implementation's <limits.h> or <unistd.h>. The value will not
 *   change during the lifetime of the calling process, except that
 *   sysconf(_SC_OPEN_MAX) may return different values before and after a
 *   call to setrlimit() which changes the RLIMIT_NOFILE soft limit.
 *
 *   If the variable corresponding to name is dependent on an unsupported
 *   option, the results are unspecified.
 *
 ****************************************************************************/

long sysconf(int name)
{
  switch (name)
    {
      /* Limits of <limits.h> */

      case _SC_AIO_LISTIO_MAX:
        return AIO_LISTIO_MAX;

      case _SC_AIO_MAX:
        return AIO_MAX;

      case _SC_AIO_PRIO_DELTA_MAX:
        return 0;

      case _SC_ARG_MAX:
        return ARG_MAX;

      case _SC_ATEXIT_MAX:
        return ATEXIT_MAX;

      case _SC_BC_BASE_MAX:
        return _POSIX2_BC_BASE_MAX;

      case _SC_BC_DIM_MAX:
        return _POSIX2_BC_DIM_MAX;

      case _SC_BC_SCALE_MAX:
        return _POSIX2_BC_SCALE_MAX;

      case _SC_BC_STRING_MAX:
        return _POSIX2_BC_STRING_MAX;

      case _SC_CHILD_MAX:
        return CHILD_MAX;

      case _SC_CLK_TCK:
        return CLK_TCK;

      case _SC_COLL_WEIGHTS_MAX:
        return _POSIX2_COLL_WEIGHTS_MAX;

      case _SC_DELAYTIMER_MAX:
        return DELAYTIMER_MAX;

      case _SC_EXPR_NEST_MAX:
        return _POSIX2_EXPR_NEST_MAX;

      case _SC_GETGR_R_SIZE_MAX:
#ifdef CONFIG_LIBC_GROUP_FILE
        return CONFIG_LIBC_GROUP_LINESIZE +
               (CONFIG_LIBC_GROUP_NUSERS + 1) * sizeof(FAR char *);
#else
        return -1;
#endif

      case _SC_GETPW_R_SIZE_MAX:
#ifdef CONFIG_LIBC_PASSWD_LINESIZE
        return CONFIG_LIBC_PASSWD_LINESIZE;
#else
        return _POSIX_GETPW_R_SIZE_MAX;
#endif

      case _SC_HOST_NAME_MAX:
        return HOST_NAME_MAX;

      case _SC_IOV_MAX:
        return IOV_MAX;

      case _SC_LINE_MAX:
        return LINE_MAX;

      case _SC_LOGIN_NAME_MAX:
        return LOGIN_NAME_MAX;

      case _SC_MQ_OPEN_MAX:
        return MQ_OPEN_MAX;

      case _SC_MQ_PRIO_MAX:
        return MQ_PRIO_MAX;

      case _SC_NGROUPS_MAX:
        return NGROUPS_MAX;

      case _SC_OPEN_MAX:
        {
          struct rlimit rlim;

          /* The soft limit of the process, which setrlimit() can change */

          if (getrlimit(RLIMIT_NOFILE, &rlim) == 0)
            {
              return rlim.rlim_cur;
            }

          return OPEN_MAX;
        }

      case _SC_PAGESIZE:
        return DEFAULT_MM_PGSIZE;

      case _SC_RE_DUP_MAX:
        return _POSIX2_RE_DUP_MAX;

      case _SC_RTSIG_MAX:
        return RTSIG_MAX;

      case _SC_SEM_NSEMS_MAX:
        return SEM_NSEMS_MAX;

      case _SC_SEM_VALUE_MAX:
        return SEM_VALUE_MAX;

      case _SC_SIGQUEUE_MAX:
        return SIGQUEUE_MAX;

      case _SC_STREAM_MAX:
        return STREAM_MAX;

      case _SC_SYMLOOP_MAX:
        return SYMLOOP_MAX;

      case _SC_THREAD_DESTRUCTOR_ITERATIONS:
        return PTHREAD_DESTRUCTOR_ITERATIONS;

      /* A configuration without pthreads has no CONFIG_TLS_NELEM */

      case _SC_THREAD_KEYS_MAX:
#ifdef CONFIG_TLS_NELEM
        return PTHREAD_KEYS_MAX;
#else
        return 0;
#endif

      case _SC_THREAD_STACK_MIN:
        return CONFIG_PTHREAD_STACK_MIN;

      case _SC_TIMER_MAX:
        return TIMER_MAX;

      case _SC_TTY_NAME_MAX:
        return TTY_NAME_MAX;

      case _SC_TZNAME_MAX:
        return TZNAME_MAX;

      /* Memory and processors */

      case _SC_PHYS_PAGES:
      case _SC_AVPHYS_PAGES:
        {
          struct sysinfo info;

          if (sysinfo(&info) < 0)
            {
              return -1;
            }

          return (name == _SC_PHYS_PAGES ? info.totalram : info.freeram) *
                 info.mem_unit / DEFAULT_MM_PGSIZE;
        }

      case _SC_NPROCESSORS_CONF:
      case _SC_NPROCESSORS_ONLN:
        return CONFIG_SMP_NCPUS;

      /* Options of <unistd.h>: the value of the macro, -1 if not supported */

      case _SC_ASYNCHRONOUS_IO:
#ifdef _POSIX_ASYNCHRONOUS_IO
        return _POSIX_ASYNCHRONOUS_IO;
#else
        return -1;
#endif

      case _SC_BARRIERS:
        return _POSIX_BARRIERS;

      case _SC_CLOCK_SELECTION:
        return _POSIX_CLOCK_SELECTION;

      case _SC_CPUTIME:
        return _POSIX_CPUTIME;

      case _SC_MAPPED_FILES:
        return _POSIX_MAPPED_FILES;

      case _SC_MESSAGE_PASSING:
        return _POSIX_MESSAGE_PASSING;

      case _SC_MONOTONIC_CLOCK:
        return _POSIX_MONOTONIC_CLOCK;

      case _SC_PRIORITIZED_IO:
        return _POSIX_PRIORITIZED_IO;

      case _SC_PRIORITY_SCHEDULING:
        return _POSIX_PRIORITY_SCHEDULING;

      case _SC_READER_WRITER_LOCKS:
        return _POSIX_READER_WRITER_LOCKS;

      case _SC_REALTIME_SIGNALS:
        return _POSIX_REALTIME_SIGNALS;

      case _SC_SEMAPHORES:
        return _POSIX_SEMAPHORES;

      case _SC_SHARED_MEMORY_OBJECTS:
        return _POSIX_SHARED_MEMORY_OBJECTS;

      case _SC_SPIN_LOCKS:
        return _POSIX_SPIN_LOCKS;

      case _SC_SPORADIC_SERVER:
        return _POSIX_SPORADIC_SERVER;

      case _SC_SYNCHRONIZED_IO:
        return _POSIX_SYNCHRONIZED_IO;

      case _SC_THREAD_ATTR_STACKADDR:
        return _POSIX_THREAD_ATTR_STACKADDR;

      case _SC_THREAD_ATTR_STACKSIZE:
        return _POSIX_THREAD_ATTR_STACKSIZE;

      case _SC_THREAD_CPUTIME:
        return _POSIX_THREAD_CPUTIME;

      case _SC_THREAD_PRIORITY_SCHEDULING:
        return _POSIX_THREAD_PRIORITY_SCHEDULING;

      case _SC_THREAD_PROCESS_SHARED:
        return _POSIX_THREAD_PROCESS_SHARED;

      case _SC_THREAD_SPORADIC_SERVER:
        return _POSIX_THREAD_SPORADIC_SERVER;

      case _SC_THREADS:
        return _POSIX_THREADS;

      case _SC_TIMEOUTS:
#ifdef _POSIX_TIMEOUTS
        return _POSIX_TIMEOUTS;
#else
        return -1;
#endif

      case _SC_TIMERS:
#ifdef _POSIX_TIMERS
        return _POSIX_TIMERS;
#else
        return -1;
#endif

      case _SC_VERSION:
        return _POSIX_VERSION;

      case _SC_XOPEN_UNIX:
        return _XOPEN_UNIX;

      case _SC_XOPEN_VERSION:
        return _XOPEN_VERSION;

      /* Options that <unistd.h> leaves undefined: sysconf() tells if this
       * configuration has them.
       */

      case _SC_FSYNC:
        return _POSIX_VERSION;

      case _SC_IPV6:
#ifdef CONFIG_NET_IPv6
        return _POSIX_VERSION;
#else
        return -1;
#endif

      case _SC_REGEXP:
#ifdef CONFIG_LIBC_REGEX
        return _POSIX_VERSION;
#else
        return -1;
#endif

      case _SC_THREAD_PRIO_INHERIT:
#if defined(CONFIG_PRIORITY_INHERITANCE) && !defined(CONFIG_DISABLE_PTHREAD)
        return _POSIX_VERSION;
#else
        return -1;
#endif

      case _SC_THREAD_PRIO_PROTECT:
#if defined(CONFIG_PRIORITY_PROTECT) && !defined(CONFIG_DISABLE_PTHREAD)
        return _POSIX_VERSION;
#else
        return -1;
#endif

      /* The C-language compilation environment of this build */

      case _SC_V6_ILP32_OFF32:
      case _SC_XBS5_ILP32_OFF32:
        return sizeof(int) == 4 && sizeof(long) == 4 &&
               sizeof(FAR void *) == 4 && sizeof(off_t) == 4 ? 1 : -1;

      case _SC_V6_ILP32_OFFBIG:
      case _SC_XBS5_ILP32_OFFBIG:
        return sizeof(int) == 4 && sizeof(long) == 4 &&
               sizeof(FAR void *) == 4 && sizeof(off_t) == 8 ? 1 : -1;

      case _SC_V6_LP64_OFF64:
      case _SC_XBS5_LP64_OFF64:
        return sizeof(int) == 4 && sizeof(long) == 8 &&
               sizeof(FAR void *) == 8 && sizeof(off_t) == 8 ? 1 : -1;

      /* Names that POSIX defines, for options and limits that NuttX does not
       * have: -1 without a change of errno.
       */

      case _SC_2_C_BIND:
      case _SC_2_C_DEV:
      case _SC_2_CHAR_TERM:
      case _SC_2_FORT_DEV:
      case _SC_2_FORT_RUN:
      case _SC_2_LOCALEDEF:
      case _SC_2_PBS:
      case _SC_2_PBS_ACCOUNTING:
      case _SC_2_PBS_CHECKPOINT:
      case _SC_2_PBS_LOCATE:
      case _SC_2_PBS_MESSAGE:
      case _SC_2_PBS_TRACK:
      case _SC_2_SW_DEV:
      case _SC_2_UPE:
      case _SC_2_VERSION:
      case _SC_ADVISORY_INFO:
      case _SC_JOB_CONTROL:
      case _SC_MEMLOCK:
      case _SC_MEMLOCK_RANGE:
      case _SC_MEMORY_PROTECTION:
      case _SC_RAW_SOCKETS:
      case _SC_SAVED_IDS:
      case _SC_SHELL:
      case _SC_SPAWN:
      case _SC_SS_REPL_MAX:
      case _SC_THREAD_SAFE_FUNCTIONS:
      case _SC_THREAD_THREADS_MAX:
      case _SC_TRACE:
      case _SC_TRACE_EVENT_FILTER:
      case _SC_TRACE_EVENT_NAME_MAX:
      case _SC_TRACE_INHERIT:
      case _SC_TRACE_LOG:
      case _SC_TRACE_NAME_MAX:
      case _SC_TRACE_SYS_MAX:
      case _SC_TRACE_USER_EVENT_MAX:
      case _SC_TYPED_MEMORY_OBJECTS:
      case _SC_V6_LPBIG_OFFBIG:
      case _SC_XBS5_LPBIG_OFFBIG:
      case _SC_XOPEN_CRYPT:
      case _SC_XOPEN_ENH_I18N:
      case _SC_XOPEN_LEGACY:
      case _SC_XOPEN_REALTIME:
      case _SC_XOPEN_REALTIME_THREADS:
      case _SC_XOPEN_SHM:
      case _SC_XOPEN_STREAMS:
        return -1;

      default:
        set_errno(EINVAL);
        return -1;
    }
}
