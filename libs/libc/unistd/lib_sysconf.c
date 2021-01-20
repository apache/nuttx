/****************************************************************************
 * libs/libc/unistd/lib_sysconf.c
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

#include <unistd.h>
#include <sched.h>
#include <errno.h>

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
 *     Variable                           Value of Name
 *
 *     {AIO_LISTIO_MAX}                   _SC_AIO_LISTIO_MAX
 *     {AIO_MAX}                          _SC_AIO_MAX
 *     {AIO_PRIO_DELTA_MAX}               _SC_AIO_PRIO_DELTA_MAX
 *     {ARG_MAX}                          _SC_ARG_MAX
 *     {ATEXIT_MAX}                       _SC_ATEXIT_MAX
 *     {BC_BASE_MAX}                      _SC_BC_BASE_MAX
 *     {BC_DIM_MAX}                       _SC_BC_DIM_MAX
 *     {BC_SCALE_MAX}                     _SC_BC_SCALE_MAX
 *     {BC_STRING_MAX}                    _SC_BC_STRING_MAX
 *     {CHILD_MAX}                        _SC_CHILD_MAX
 *     Clock ticks/second                 _SC_CLK_TCK
 *     {COLL_WEIGHTS_MAX}                 _SC_COLL_WEIGHTS_MAX
 *     {DELAYTIMER_MAX}                   _SC_DELAYTIMER_MAX
 *     {EXPR_NEST_MAX}                    _SC_EXPR_NEST_MAX
 *     {HOST_NAME_MAX}                    _SC_HOST_NAME_MAX
 *     {IOV_MAX}                          _SC_IOV_MAX
 *     {LINE_MAX}                         _SC_LINE_MAX
 *     {LOGIN_NAME_MAX}                   _SC_LOGIN_NAME_MAX
 *     {NGROUPS_MAX}                      _SC_NGROUPS_MAX
 *     Maximum size of getgrgid_r() and   _SC_GETGR_R_SIZE_MAX
 *     getgrnam_r() data buffers
 *     Maximum size of getpwuid_r() and   _SC_GETPW_R_SIZE_MAX
 *     getpwnam_r() data buffers
 *     {MQ_OPEN_MAX}                      _SC_MQ_OPEN_MAX
 *     {MQ_PRIO_MAX}                      _SC_MQ_PRIO_MAX
 *     {OPEN_MAX}                         _SC_OPEN_MAX
 *     _POSIX_ADVISORY_INFO               _SC_ADVISORY_INFO
 *     _POSIX_BARRIERS                    _SC_BARRIERS
 *     _POSIX_ASYNCHRONOUS_IO             _SC_ASYNCHRONOUS_IO
 *     _POSIX_CLOCK_SELECTION             _SC_CLOCK_SELECTION
 *     _POSIX_CPUTIME                     _SC_CPUTIME
 *     _POSIX_FSYNC                       _SC_FSYNC
 *     _POSIX_IPV6                        _SC_IPV6
 *     _POSIX_JOB_CONTROL                 _SC_JOB_CONTROL
 *     _POSIX_MAPPED_FILES                _SC_MAPPED_FILES
 *     _POSIX_MEMLOCK                     _SC_MEMLOCK
 *     _POSIX_MEMLOCK_RANGE               _SC_MEMLOCK_RANGE
 *     _POSIX_MEMORY_PROTECTION           _SC_MEMORY_PROTECTION
 *     _POSIX_MESSAGE_PASSING             _SC_MESSAGE_PASSING
 *     _POSIX_MONOTONIC_CLOCK             _SC_MONOTONIC_CLOCK
 *     _POSIX_PRIORITIZED_IO              _SC_PRIORITIZED_IO
 *     _POSIX_PRIORITY_SCHEDULING         _SC_PRIORITY_SCHEDULING
 *     _POSIX_RAW_SOCKETS                 _SC_RAW_SOCKETS
 *     _POSIX_READER_WRITER_LOCKS         _SC_READER_WRITER_LOCKS
 *     _POSIX_REALTIME_SIGNALS            _SC_REALTIME_SIGNALS
 *     _POSIX_REGEXP                      _SC_REGEXP
 *     _POSIX_SAVED_IDS                   _SC_SAVED_IDS
 *     _POSIX_SEMAPHORES                  _SC_SEMAPHORES
 *     _POSIX_SHARED_MEMORY_OBJECTS       _SC_SHARED_MEMORY_OBJECTS
 *     _POSIX_SHELL                       _SC_SHELL
 *     _POSIX_SPAWN                       _SC_SPAWN
 *     _POSIX_SPIN_LOCKS                  _SC_SPIN_LOCKS
 *     _POSIX_SPORADIC_SERVER             _SC_SPORADIC_SERVER
 *     _POSIX_SS_REPL_MAX                 _SC_SS_REPL_MAX
 *     _POSIX_SYNCHRONIZED_IO             _SC_SYNCHRONIZED_IO
 *     _POSIX_THREAD_ATTR_STACKADDR       _SC_THREAD_ATTR_STACKADDR
 *     _POSIX_THREAD_ATTR_STACKSIZE       _SC_THREAD_ATTR_STACKSIZE
 *     _POSIX_THREAD_CPUTIME              _SC_THREAD_CPUTIME
 *     _POSIX_THREAD_PRIO_INHERIT         _SC_THREAD_PRIO_INHERIT
 *     _POSIX_THREAD_PRIO_PROTECT         _SC_THREAD_PRIO_PROTECT
 *     _POSIX_THREAD_PRIORITY_SCHEDULING  _SC_THREAD_PRIORITY_SCHEDULING
 *     _POSIX_THREAD_PROCESS_SHARED       _SC_THREAD_PROCESS_SHARED
 *     _POSIX_THREAD_SAFE_FUNCTIONS       _SC_THREAD_SAFE_FUNCTIONS
 *     _POSIX_THREAD_SPORADIC_SERVER      _SC_THREAD_SPORADIC_SERVER
 *     _POSIX_THREADS                     _SC_THREADS
 *     _POSIX_TIMEOUTS                    _SC_TIMEOUTS
 *     _POSIX_TIMERS                      _SC_TIMERS
 *     _POSIX_TRACE                       _SC_TRACE
 *     _POSIX_TRACE_EVENT_FILTER          _SC_TRACE_EVENT_FILTER
 *     _POSIX_TRACE_EVENT_NAME_MAX        _SC_TRACE_EVENT_NAME_MAX
 *     _POSIX_TRACE_INHERIT               _SC_TRACE_INHERIT
 *     _POSIX_TRACE_LOG                   _SC_TRACE_LOG
 *     _POSIX_TRACE_NAME_MAX              _SC_TRACE_NAME_MAX
 *     _POSIX_TRACE_SYS_MAX               _SC_TRACE_SYS_MAX
 *     _POSIX_TRACE_USER_EVENT_MAX        _SC_TRACE_USER_EVENT_MAX
 *     _POSIX_TYPED_MEMORY_OBJECTS        _SC_TYPED_MEMORY_OBJECTS
 *     _POSIX_VERSION                     _SC_VERSION
 *     _POSIX_V6_ILP32_OFF32              _SC_V6_ILP32_OFF32
 *     _POSIX_V6_ILP32_OFFBIG             _SC_V6_ILP32_OFFBIG
 *     _POSIX_V6_LP64_OFF64               _SC_V6_LP64_OFF64
 *     _POSIX_V6_LPBIG_OFFBIG             _SC_V6_LPBIG_OFFBIG
 *     _POSIX2_C_BIND                     _SC_2_C_BIND
 *     _POSIX2_C_DEV                      _SC_2_C_DEV
 *     _POSIX2_CHAR_TERM                  _SC_2_CHAR_TERM
 *     _POSIX2_FORT_DEV                   _SC_2_FORT_DEV
 *     _POSIX2_FORT_RUN                   _SC_2_FORT_RUN
 *     _POSIX2_LOCALEDEF                  _SC_2_LOCALEDEF
 *     _POSIX2_PBS                        _SC_2_PBS
 *     _POSIX2_PBS_ACCOUNTING             _SC_2_PBS_ACCOUNTING
 *     _POSIX2_PBS_CHECKPOINT             _SC_2_PBS_CHECKPOINT
 *     _POSIX2_PBS_LOCATE                 _SC_2_PBS_LOCATE
 *     _POSIX2_PBS_MESSAGE                _SC_2_PBS_MESSAGE
 *     _POSIX2_PBS_TRACK                  _SC_2_PBS_TRACK
 *     _POSIX2_SW_DEV                     _SC_2_SW_DEV
 *     _POSIX2_UPE                        _SC_2_UPE
 *     _POSIX2_VERSION                    _SC_2_VERSION
 *     {PAGE_SIZE}                        _SC_PAGE_SIZE
 *     {PAGESIZE}                         _SC_PAGESIZE
 *     {PTHREAD_DESTRUCTOR_ITERATIONS}    _SC_THREAD_DESTRUCTOR_ITERATIONS
 *     {PTHREAD_KEYS_MAX}                 _SC_THREAD_KEYS_MAX
 *     {PTHREAD_STACK_MIN}                _SC_THREAD_STACK_MIN
 *     {PTHREAD_THREADS_MAX}              _SC_THREAD_THREADS_MAX
 *     {RE_DUP_MAX}                       _SC_RE_DUP_MAX
 *     {RTSIG_MAX}                        _SC_RTSIG_MAX
 *     {SEM_NSEMS_MAX}                    _SC_SEM_NSEMS_MAX
 *     {SEM_VALUE_MAX}                    _SC_SEM_VALUE_MAX
 *     {SIGQUEUE_MAX}                     _SC_SIGQUEUE_MAX
 *     {STREAM_MAX}                       _SC_STREAM_MAX
 *     {SYMLOOP_MAX}                      _SC_SYMLOOP_MAX
 *     {TIMER_MAX}                        _SC_TIMER_MAX
 *     {TTY_NAME_MAX}                     _SC_TTY_NAME_MAX
 *     {TZNAME_MAX}                       _SC_TZNAME_MAX
 *     _XBS5_ILP32_OFF32 (LEGACY)         _SC_XBS5_ILP32_OFF32 (LEGACY)
 *     _XBS5_ILP32_OFFBIG (LEGACY)        _SC_XBS5_ILP32_OFFBIG (LEGACY)
 *     _XBS5_LP64_OFF64 (LEGACY)          _SC_XBS5_LP64_OFF64 (LEGACY)
 *     _XBS5_LPBIG_OFFBIG (LEGACY)        _SC_XBS5_LPBIG_OFFBIG (LEGACY)
 *     _XOPEN_CRYPT                       _SC_XOPEN_CRYPT
 *     _XOPEN_ENH_I18N                    _SC_XOPEN_ENH_I18N
 *     _XOPEN_LEGACY                      _SC_XOPEN_LEGACY
 *     _XOPEN_REALTIME                    _SC_XOPEN_REALTIME
 *     _XOPEN_REALTIME_THREADS            _SC_XOPEN_REALTIME_THREADS
 *     _XOPEN_SHM                         _SC_XOPEN_SHM
 *     _XOPEN_STREAMS                     _SC_XOPEN_STREAMS
 *     _XOPEN_UNIX                        _SC_XOPEN_UNIX
 *     _XOPEN_VERSION                     _SC_XOPEN_VERSION
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
  int errcode;

  /* NOTE:  The initialize implementation of this interface is very sparse.
   * It was originally created to support only the functionality of
   * getdtablesize() but can be extended to support as much of the standard
   * POSIX sysconf() as is necessary.
   */

  switch (name)
    {
      case _SC_OPEN_MAX:
        return _POSIX_OPEN_MAX;

      case _SC_ATEXIT_MAX:
#ifdef CONFIG_SCHED_EXIT_MAX
        return CONFIG_SCHED_EXIT_MAX;
#else
        return 0;
#endif

      case _SC_NPROCESSORS_CONF:
      case _SC_NPROCESSORS_ONLN:
#ifdef CONFIG_SMP_NCPUS
        return CONFIG_SMP_NCPUS;
#else
        return 1;
#endif

      case _SC_MONOTONIC_CLOCK:
#ifdef CONFIG_CLOCK_MONOTONIC
        return 1;
#else
        return 0;
#endif

      case _SC_PAGESIZE:
#ifdef CONFIG_MM_PGSIZE
        return CONFIG_MM_PGSIZE;
#else
        return 1;
#endif

      default:
#if 0 /* Assume valid but not implemented for the time being */
        errcode = EINVAL;
#else
        errcode = ENOSYS;
#endif
        break;
    }

  set_errno(errcode);
  return ERROR;
}
