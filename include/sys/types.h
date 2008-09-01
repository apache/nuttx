/****************************************************************************
 * sys/types.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __SYS_TYPES_H
#define __SYS_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/types.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Values for type boolean */

#define TRUE 1
#define FALSE 0

/* NULL is usually defined in stddef.h (which includes this file) */

#ifndef NULL
    /* SDCC is sensitive to NULL pointer type conversions */
#  ifdef SDCC
#    define NULL (0)
#  else
#    define NULL ((void*)0)
#  endif
#endif

/* POSIX-like OS return values: */

#if !defined(__cplusplus)
#  undef  ERROR
#  define ERROR -1
#endif

#undef  OK
#define OK 0

/* HPUX-like MIN/MAX value */

#define PRIOR_RR_MIN      0
#define PRIOR_RR_MAX    255
#define PRIOR_FIFO_MIN    0
#define PRIOR_FIFO_MAX  255
#define PRIOR_OTHER_MIN   0
#define PRIOR_OTHER_MAX 255

/* Scheduling Priorities.  NOTE:  Only the idle task can take
 * the TRUE minimum priority. */

#define SCHED_PRIORITY_MAX     255
#define SCHED_PRIORITY_DEFAULT 100
#define SCHED_PRIORITY_MIN       1
#define SCHED_PRIORITY_IDLE      0

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef float  float32;
#ifndef CONFIG_HAVE_DOUBLE
typedef float  double_t;
typedef float  float64;
#else
typedef double double_t;
typedef double float64;
#endif

/* Misc. scalar types */

typedef unsigned int mode_t; /* Needs at least 16-bits but must be */
                               /* sizeof(int) because it is passed */
                               /* via varargs. */
#ifdef CONFIG_SMALL_MEMORY
typedef uint16       size_t;
typedef sint16       ssize_t;
typedef sint16       off_t;
typedef uint16       blksize_t;
typedef uint16       blkcnt_t;
#else
typedef uint32       size_t;
typedef sint32       ssize_t;
typedef sint32       off_t;
typedef uint16       blksize_t;
typedef uint32       blkcnt_t;
#endif
typedef off_t        fpos_t;
typedef sint16       uid_t;
typedef sint16       gid_t;
typedef uint16       dev_t;
typedef uint16       ino_t;
typedef unsigned int sig_atomic_t;
typedef int          pid_t;
typedef int          STATUS;

typedef unsigned int socklen_t;
typedef uint16       sa_family_t;

/* Process entry point */

typedef int (*main_t)(int argc, char *argv[]);

#endif

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#endif /* __SYS_TYPES_H */
