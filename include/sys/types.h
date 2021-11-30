/****************************************************************************
 * include/sys/types.h
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

#ifndef __INCLUDE_SYS_TYPES_H
#define __INCLUDE_SYS_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alternative values for type bool (for historic reasons) */

#ifndef TRUE
#  define TRUE  1
#endif

#ifndef FALSE
#  define FALSE 0
#endif

/* NULL is usually defined in stddef.h (which includes this file) */

#ifndef NULL
  /* SDCC is sensitive to NULL pointer type conversions, and C++ defines
   * NULL as zero
   */

#  if defined(SDCC) || defined(__SDCC) || defined(__cplusplus)
#    define NULL (0)
#  else
#    define NULL ((void*)0)
#  endif
#endif

/* Scheduling Priorities.
 *
 * NOTES:
 * - Only the idle task can take the true minimum priority.
 * - These definitions are non-standard internal definitions and, for
 *   portability reasons, should not be used by application software.
 */

#define SCHED_PRIORITY_MAX     255
#define SCHED_PRIORITY_DEFAULT 100
#define SCHED_PRIORITY_MIN       1
#define SCHED_PRIORITY_IDLE      0

#if defined(CONFIG_FS_LARGEFILE) && defined(CONFIG_HAVE_LONG_LONG)
#  define fsblkcnt64_t           fsblkcnt_t
#  define fsfilcnt64_t           fsfilcnt_t
#  define blkcnt64_t             blkcnt_t
#  define off64_t                off_t
#  define fpos64_t               fpos_t
#endif

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Misc. scalar types */

/* mode_t is an integer type used for file attributes.  mode_t needs
 * to be at least 16-bits but, in fact, must be sizeof(int) because it is
 * passed via varargs.
 */

typedef unsigned int mode_t;

/* size_t is used for sizes of memory objects.
 * ssize_t is used for a count of bytes or an error indication.
 *
 * See also definitions of SIZE_MAX et al in limits.h.
 *
 * REVISIT: size_t belongs in stddef.h
 */

#ifdef CONFIG_SMALL_MEMORY

typedef uint16_t     size_t;
typedef int16_t      ssize_t;
typedef uint16_t     rsize_t;

#else /* CONFIG_SMALL_MEMORY */

typedef _size_t      size_t;
typedef _ssize_t     ssize_t;
typedef _size_t      rsize_t;

#endif /* CONFIG_SMALL_MEMORY */

/* uid_t is used for user IDs
 * gid_t is used for group IDs.
 */

typedef int16_t      uid_t;
typedef int16_t      gid_t;

/* dev_t is used for device IDs */

typedef uint16_t     dev_t;

/* ino_t is used for file serial numbers */

typedef uint16_t     ino_t;

/* nlink_t is used for link counts */

typedef uint16_t     nlink_t;

/* pid_t is used for process IDs and process group IDs. It must be signed
 * because negative PID values are used to represent invalid PIDs.
 */

typedef int16_t      pid_t;

/* id_t is a general identifier that can be used to contain at least a pid_t,
 * uid_t, or gid_t.
 */

typedef int16_t      id_t;

/* Unix requires a key of type key_t defined in file sys/types.h for
 * requesting resources such as shared memory segments, message queues and
 * semaphores. A key is simply an integer of type key_t
 */

typedef int16_t      key_t;

/* Signed integral type of the result of subtracting two pointers */

typedef intptr_t     ptrdiff_t;

#if !defined(__cplusplus)
/* Wide, 16-bit character types.  wchar_t is a built-in type in C++ and
 * its declaration here may cause compilation errors on some compilers.
 *
 * REVISIT: wchar_t belongs in stddef.h
 */

typedef uint16_t     wchar_t;
#endif

/* wint_t
 *   An integral type capable of storing any valid value of wchar_t, or WEOF.
 */

typedef int wint_t;

/* wctype_t
 *   A scalar type of a data object that can hold values which represent
 *   locale-specific character classification.
 */

typedef int wctype_t;

#if defined(CONFIG_FS_LARGEFILE) && defined(CONFIG_HAVE_LONG_LONG)
/* Large file versions */

typedef uint64_t     fsblkcnt_t;
typedef uint64_t     fsfilcnt_t;

typedef uint64_t     blkcnt_t;
typedef int64_t      off_t;
typedef int64_t      fpos_t;
#else
/* fsblkcnt_t and fsfilcnt_t shall be defined as unsigned integer types. */

typedef uint32_t     fsblkcnt_t;
typedef uint32_t     fsfilcnt_t;

/* blkcnt_t and off_t are signed integer types.
 *
 *   blkcnt_t is used for file block counts.
 *   off_t is used for file offsets and sizes.
 *   fpos_t is used for file positions.
 *
 * Hence, all should be independent of processor architecture.
 */

typedef uint32_t     blkcnt_t;
typedef int32_t      off_t;
typedef int32_t      fpos_t;
#endif

/* blksize_t is a signed integer value used for file block sizes */

typedef int16_t      blksize_t;

/* Network related */

typedef unsigned int socklen_t;
#define __socklen_t_defined
typedef uint16_t     sa_family_t;

/* Used for system times in clock ticks. This type is the natural width of
 * the system timer.
 *
 * NOTE: The signed-ness of clock_t is not specified at OpenGroup.org.  An
 * unsigned type is used to support the full range of the internal clock.
 */

#ifdef CONFIG_SYSTEM_TIME64
typedef uint64_t     clock_t;
#else
typedef uint32_t     clock_t;
#endif

/* The type useconds_t shall be an unsigned integer type capable of storing
 * values at least in the range [0, 1000000]. The type suseconds_t shall be
 * a signed integer type capable of storing values at least in the range
 * [-1, 1000000].
 */

typedef uint32_t     useconds_t;
typedef int32_t      suseconds_t;

#ifdef CONFIG_SMP
/* This is the smallest integer type that will hold a bitset of all CPUs */

#if (CONFIG_SMP_NCPUS <= 8)
typedef volatile uint8_t cpu_set_t;
#elif (CONFIG_SMP_NCPUS <= 16)
typedef volatile uint16_t cpu_set_t;
#elif (CONFIG_SMP_NCPUS <= 32)
typedef volatile uint32_t cpu_set_t;
#else
#  error SMP: Extensions needed to support this number of CPUs
#endif
#else
typedef volatile uint8_t cpu_set_t;
#endif /* CONFIG_SMP */

/* BSD types provided only to support porting to NuttX. */

typedef unsigned char  u_char;
typedef unsigned short u_short;
typedef unsigned int   u_int;
typedef unsigned long  u_long;

/* SYSV types provided only to support porting to NuttX.  */

typedef unsigned char  unchar;
typedef unsigned short ushort;
typedef unsigned int   uint;
typedef unsigned long  ulong;
typedef signed char    s_char;
typedef FAR char      *caddr_t;

/* These were defined by ISO C without the first `_'.  */

typedef uint8_t  u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
#ifdef __INT24_DEFINED
typedef uint24_t u_int24_t;
#endif
#ifdef __INT64_DEFINED
typedef uint64_t u_int64_t;
#endif

/* Task entry point */

typedef CODE int (*main_t)(int argc, FAR char *argv[]);

/* POSIX-like OS return values: */

enum
{
  ERROR = -1,
  OK = 0,
};

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_SYS_TYPES_H */
