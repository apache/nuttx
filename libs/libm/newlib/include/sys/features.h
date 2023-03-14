/****************************************************************************
 * libs/libm/newlib/include/sys/features.h
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

#ifndef __LIBS_LIBM_NEWLIB_INCLUDE_SYS_FEATURES_H
#define __LIBS_LIBM_NEWLIB_INCLUDE_SYS_FEATURES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <_newlib_version.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Macro to test version of GCC.  Returns 0 for non-GCC or too old GCC. */
#ifndef __GNUC_PREREQ
# if defined __GNUC__ && defined __GNUC_MINOR__
#  define __GNUC_PREREQ(maj, min) \
  ((__GNUC__ << 16) + __GNUC_MINOR__ >= ((maj) << 16) + (min))
# else
#  define __GNUC_PREREQ(maj, min)   0
# endif
#endif /* __GNUC_PREREQ */
/* Version with trailing underscores for BSD compatibility. */
#define __GNUC_PREREQ__(ma, mi)     __GNUC_PREREQ(ma, mi)

#ifdef _GNU_SOURCE
#undef _ATFILE_SOURCE
#define _ATFILE_SOURCE                      1
#undef  _DEFAULT_SOURCE
#define _DEFAULT_SOURCE                     1
#undef _ISOC99_SOURCE
#define _ISOC99_SOURCE                      1
#undef _ISOC11_SOURCE
#define _ISOC11_SOURCE                      1
#undef _POSIX_SOURCE
#define _POSIX_SOURCE                       1
#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE                     200809L
#undef _XOPEN_SOURCE
#define _XOPEN_SOURCE                       700
#undef _XOPEN_SOURCE_EXTENDED
#define _XOPEN_SOURCE_EXTENDED              1
#endif /* _GNU_SOURCE */

#if defined(_BSD_SOURCE) || defined(_SVID_SOURCE) ||      \
  (!defined(__STRICT_ANSI__) && !defined(_ANSI_SOURCE) && \
  !defined(_ISOC99_SOURCE) && !defined(_POSIX_SOURCE) &&  \
  !defined(_POSIX_C_SOURCE) && !defined(_XOPEN_SOURCE))
#undef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE                     1
#endif

#if defined(_DEFAULT_SOURCE)
#undef _POSIX_SOURCE
#define _POSIX_SOURCE                       1
#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE                     200809L
#endif

#if !defined(_POSIX_SOURCE) && !defined(_POSIX_C_SOURCE) && \
  ((!defined(__STRICT_ANSI__) && !defined(_ANSI_SOURCE)) || \
  (_XOPEN_SOURCE - 0) >= 500)
#define _POSIX_SOURCE                       1
#if !defined(_XOPEN_SOURCE) || (_XOPEN_SOURCE - 0) >= 700
#define _POSIX_C_SOURCE                     200809L
#elif (_XOPEN_SOURCE - 0) >= 600
#define _POSIX_C_SOURCE                     200112L
#elif (_XOPEN_SOURCE - 0) >= 500
#define _POSIX_C_SOURCE                     199506L
#elif (_XOPEN_SOURCE - 0) < 500
#define _POSIX_C_SOURCE                     2
#endif
#endif

#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200809
#undef _ATFILE_SOURCE
#define _ATFILE_SOURCE                      1
#endif

#ifdef _ATFILE_SOURCE
#define __ATFILE_VISIBLE                    1
#else
#define __ATFILE_VISIBLE                    0
#endif

#ifdef _DEFAULT_SOURCE
#define __BSD_VISIBLE                       1
#else
#define __BSD_VISIBLE                       0
#endif

#ifdef _GNU_SOURCE
#define __GNU_VISIBLE                       1
#else
#define __GNU_VISIBLE                       0
#endif

#if defined(_ISOC11_SOURCE) || \
  (__STDC_VERSION__ - 0) >= 201112L || (__cplusplus - 0) >= 201103L
#define __ISO_C_VISIBLE                     2011
#elif defined(_ISOC99_SOURCE) || (_POSIX_C_SOURCE - 0) >= 200112L || \
  (__STDC_VERSION__ - 0) >= 199901L || defined(__cplusplus)
#define __ISO_C_VISIBLE                     1999
#else
#define __ISO_C_VISIBLE                     1990
#endif

#if defined(_LARGEFILE_SOURCE) || (_XOPEN_SOURCE - 0) >= 500
#define __LARGEFILE_VISIBLE                 1
#else
#define __LARGEFILE_VISIBLE                 0
#endif

#ifdef _DEFAULT_SOURCE
#define __MISC_VISIBLE                      1
#else
#define __MISC_VISIBLE                      0
#endif

#if (_POSIX_C_SOURCE - 0) >= 200809L
#define __POSIX_VISIBLE                     200809
#elif (_POSIX_C_SOURCE - 0) >= 200112L
#define __POSIX_VISIBLE                     200112
#elif (_POSIX_C_SOURCE - 0) >= 199506L
#define __POSIX_VISIBLE                     199506
#elif (_POSIX_C_SOURCE - 0) >= 199309L
#define __POSIX_VISIBLE                     199309
#elif (_POSIX_C_SOURCE - 0) >= 2 || defined(_XOPEN_SOURCE)
#define __POSIX_VISIBLE                     199209
#elif defined(_POSIX_SOURCE) || defined(_POSIX_C_SOURCE)
#define __POSIX_VISIBLE                     199009
#else
#define __POSIX_VISIBLE                     0
#endif

#ifdef _DEFAULT_SOURCE
#define __SVID_VISIBLE                      1
#else
#define __SVID_VISIBLE                      0
#endif

#if (_XOPEN_SOURCE - 0) >= 700
#define __XSI_VISIBLE                       700
#elif (_XOPEN_SOURCE - 0) >= 600
#define __XSI_VISIBLE                       600
#elif (_XOPEN_SOURCE - 0) >= 500
#define __XSI_VISIBLE                       500
#elif defined(_XOPEN_SOURCE) && defined(_XOPEN_SOURCE_EXTENDED)
#define __XSI_VISIBLE                       4
#elif defined(_XOPEN_SOURCE)
#define __XSI_VISIBLE                       1
#else
#define __XSI_VISIBLE                       0
#endif

#if _FORTIFY_SOURCE > 0 && !defined(__cplusplus) && !defined(__lint__) && \
  (__OPTIMIZE__ > 0 || defined(__clang__)) && __GNUC_PREREQ__(4, 1) &&    \
  !defined(_LIBC)
#  if _FORTIFY_SOURCE > 1
#    define __SSP_FORTIFY_LEVEL             2
#  else
#    define __SSP_FORTIFY_LEVEL             1
#  endif
#else
#  define __SSP_FORTIFY_LEVEL               0
#endif

/* RTEMS adheres to POSIX -- 1003.1b with some features from annexes.  */

#ifdef __rtems__
#define _POSIX_JOB_CONTROL                  1
#define _POSIX_SAVED_IDS                    1
#define _POSIX_VERSION                      199309L
#define _POSIX_ASYNCHRONOUS_IO              1
#define _POSIX_FSYNC                        1
#define _POSIX_MAPPED_FILES                 1
#define _POSIX_MEMLOCK                      1
#define _POSIX_MEMLOCK_RANGE                1
#define _POSIX_MEMORY_PROTECTION            1
#define _POSIX_MESSAGE_PASSING              1
#define _POSIX_MONOTONIC_CLOCK              200112L
#define _POSIX_CLOCK_SELECTION              200112L
#define _POSIX_PRIORITIZED_IO               1
#define _POSIX_PRIORITY_SCHEDULING          1
#define _POSIX_REALTIME_SIGNALS             1
#define _POSIX_SEMAPHORES                   1
#define _POSIX_SHARED_MEMORY_OBJECTS        1
#define _POSIX_SYNCHRONIZED_IO              1
#define _POSIX_TIMERS                       1
#define _POSIX_BARRIERS                     200112L
#define _POSIX_READER_WRITER_LOCKS          200112L
#define _POSIX_SPIN_LOCKS                   200112L

/* In P1003.1b but defined by drafts at least as early as P1003.1c/D10  */
#define _POSIX_THREADS                      1
#define _POSIX_THREAD_ATTR_STACKADDR        1
#define _POSIX_THREAD_ATTR_STACKSIZE        1
#define _POSIX_THREAD_PRIORITY_SCHEDULING   1
#define _POSIX_THREAD_PRIO_INHERIT          1
#define _POSIX_THREAD_PRIO_PROTECT          1
#define _POSIX_THREAD_PROCESS_SHARED        1
#define _POSIX_THREAD_SAFE_FUNCTIONS        1

/* P1003.4b/D8 defines the constants below this comment. */
#define _POSIX_SPAWN                        1
#define _POSIX_TIMEOUTS                     1
#define _POSIX_CPUTIME                      1
#define _POSIX_THREAD_CPUTIME               1
#define _POSIX_SPORADIC_SERVER              1
#define _POSIX_THREAD_SPORADIC_SERVER       1
#define _POSIX_DEVICE_CONTROL               1
#define _POSIX_DEVCTL_DIRECTION             1
#define _POSIX_INTERRUPT_CONTROL            1
#define _POSIX_ADVISORY_INFO                1

/* UNIX98 added some new pthread mutex attributes */
#define _UNIX98_THREAD_MUTEX_ATTRIBUTES     1

/* POSIX 1003.26-2003 defined device control method */
#define _POSIX_26_VERSION                   200312L

#endif

/* XMK loosely adheres to POSIX -- 1003.1 */
#ifdef __XMK__
#define _POSIX_THREADS                      1
#define _POSIX_THREAD_PRIORITY_SCHEDULING   1
#endif

#ifdef __svr4__
# define _POSIX_JOB_CONTROL                 1
# define _POSIX_SAVED_IDS                   1
# define _POSIX_VERSION                     199009L
#endif

#ifdef __CYGWIN__

#if __POSIX_VISIBLE >= 200809
#define _POSIX_VERSION                      200809L
#define _POSIX2_VERSION                     200809L
#elif __POSIX_VISIBLE >= 200112
#define _POSIX_VERSION                      200112L
#define _POSIX2_VERSION                     200112L
#elif __POSIX_VISIBLE >= 199506
#define _POSIX_VERSION                      199506L
#define _POSIX2_VERSION                     199506L
#elif __POSIX_VISIBLE >= 199309
#define _POSIX_VERSION                      199309L
#define _POSIX2_VERSION                     199209L
#elif __POSIX_VISIBLE >= 199209
#define _POSIX_VERSION                      199009L
#define _POSIX2_VERSION                     199209L
#elif __POSIX_VISIBLE
#define _POSIX_VERSION                      199009L
#endif
#if __XSI_VISIBLE >= 4
#define _XOPEN_VERSION                      __XSI_VISIBLE
#endif

#define _POSIX_ADVISORY_INFO                200809L
#define _POSIX_ASYNCHRONOUS_IO              200809L
#define _POSIX_BARRIERS                     200809L
#define _POSIX_CHOWN_RESTRICTED             1
#define _POSIX_CLOCK_SELECTION              200809L
#define _POSIX_CPUTIME                      200809L
#define _POSIX_FSYNC                        200809L
#define _POSIX_IPV6                         200809L
#define _POSIX_JOB_CONTROL                  1
#define _POSIX_MAPPED_FILES                 200809L

#define _POSIX_MEMLOCK_RANGE                200809L
#define _POSIX_MEMORY_PROTECTION            200809L
#define _POSIX_MESSAGE_PASSING              200809L
#define _POSIX_MONOTONIC_CLOCK              200809L
#define _POSIX_NO_TRUNC                     1

#define _POSIX_PRIORITY_SCHEDULING          200809L
#define _POSIX_RAW_SOCKETS                  200809L
#define _POSIX_READER_WRITER_LOCKS          200809L
#define _POSIX_REALTIME_SIGNALS             200809L
#define _POSIX_REGEXP                       1
#define _POSIX_SAVED_IDS                    1
#define _POSIX_SEMAPHORES                   200809L
#define _POSIX_SHARED_MEMORY_OBJECTS        200809L
#define _POSIX_SHELL                        1
#define _POSIX_SPAWN                        200809L
#define _POSIX_SPIN_LOCKS                   200809L

#define _POSIX_SYNCHRONIZED_IO              200809L
#define _POSIX_THREAD_ATTR_STACKADDR        200809L
#define _POSIX_THREAD_ATTR_STACKSIZE        200809L
#define _POSIX_THREAD_CPUTIME               200809L

#define _POSIX_THREAD_PRIORITY_SCHEDULING   200809L
#define _POSIX_THREAD_PROCESS_SHARED        200809L
#define _POSIX_THREAD_SAFE_FUNCTIONS        200809L

#define _POSIX_THREADS                      200809L
#define _POSIX_TIMEOUTS                     200809L
#define _POSIX_TIMERS                       200809L

#define _POSIX_VDISABLE                     '\0'

#if __POSIX_VISIBLE >= 2
#define _POSIX2_C_VERSION                   _POSIX2_VERSION
#define _POSIX2_C_BIND                      _POSIX2_VERSION
#define _POSIX2_C_DEV                       _POSIX2_VERSION
#define _POSIX2_CHAR_TERM                   _POSIX2_VERSION

#define _POSIX2_SW_DEV          _POSIX2_VERSION
#define _POSIX2_UPE             _POSIX2_VERSION
#endif /* __POSIX_VISIBLE >= 2 */

#define _POSIX_V6_ILP32_OFF32   -1
#ifdef __LP64__
#define _POSIX_V6_ILP32_OFFBIG  -1
#define _POSIX_V6_LP64_OFF64    1
#define _POSIX_V6_LPBIG_OFFBIG  1
#else
#define _POSIX_V6_ILP32_OFFBIG  1
#define _POSIX_V6_LP64_OFF64    -1
#define _POSIX_V6_LPBIG_OFFBIG  -1
#endif
#define _POSIX_V7_ILP32_OFF32   _POSIX_V6_ILP32_OFF32
#define _POSIX_V7_ILP32_OFFBIG  _POSIX_V6_ILP32_OFFBIG
#define _POSIX_V7_LP64_OFF64    _POSIX_V6_LP64_OFF64
#define _POSIX_V7_LPBIG_OFFBIG  _POSIX_V6_LPBIG_OFFBIG
#define _XBS5_ILP32_OFF32       _POSIX_V6_ILP32_OFF32
#define _XBS5_ILP32_OFFBIG      _POSIX_V6_ILP32_OFFBIG
#define _XBS5_LP64_OFF64        _POSIX_V6_LP64_OFF64
#define _XBS5_LPBIG_OFFBIG      _POSIX_V6_LPBIG_OFFBIG

#if __XSI_VISIBLE
#define _XOPEN_CRYPT            1
#define _XOPEN_ENH_I18N         1

#define _XOPEN_SHM              1

#endif /* __XSI_VISIBLE */

#define __STDC_ISO_10646__      201806L

#endif /* __CYGWIN__ */

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __LIBS_LIBM_NEWLIB_INCLUDE_SYS_FEATURES_H */
