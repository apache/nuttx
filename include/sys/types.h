/************************************************************
 * types.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

#ifndef __SYS_TYPES_H
#define __SYS_TYPES_H

/************************************************************
 * Included Files
 ************************************************************/

#include <arch/types.h>

/************************************************************
 * Definitions
 ************************************************************/

/* Values for type boolean */

#define TRUE 1
#define FALSE 0

/* NULL is usually defined in stddef.h */

#ifndef NULL
#define NULL (void*)0L
#endif

/* POSIX-like OS return values: */

#if !defined(__cplusplus)
#undef  ERROR
#define ERROR -1
#endif

#undef  OK
#define OK 0

/* POSIX-like scheduling policies (only SCHED_FIFO is supported) */

#define SCHED_FIFO  1  /* FIFO per priority scheduling policy */
#define SCHED_RR    2  /* Round robin scheduling policy */
#define SCHED_OTHER 4  /* Not used */

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

/* oflag bit settings for sem_open and mq_open */

#define O_RDONLY    0x01   /* Open for read access */
#define O_WRONLY    0x02   /* Open for write access */
#define O_RDWR      0x03   /* Open for both read & write access */
#define O_CREAT     0x04   /* Create semaphore/message queue */
#define O_EXCL      0x08   /* Name must not exist when opened  */
#define O_APPEND    0x10
#define O_TRUNC     0x20
#define O_NONBLOCK  0x40   /* Don't wait for data */
#define O_NDELAY    O_NONBLOCK
#define O_LOCK      0x80

#define O_RDOK      O_RDONLY /* Not POSIX */
#define O_WROK      O_WRONLY /* Not POSIX */

/************************************************************
 * Type Declarations
 ************************************************************/

/* Misc. scalar types */

typedef uint32        mode_t;
typedef uint32        size_t;
typedef sint32        ssize_t;
//typedef sint32      time_t;
typedef sint32        off_t;
typedef sint32        uid_t;
typedef sint32        gid_t;
typedef uint32        dev_t;
typedef uint32        ino_t;
typedef unsigned int  sig_atomic_t;
typedef int           pid_t;
typedef int           STATUS;

/* Process entry point */

typedef int (*main_t)(int argc, char *argv[]);

/* This is the POSIX-like scheduling parameter structure */

struct sched_param
{
  int sched_priority;
};

/************************************************************
 * Global Function Prototypes
 ************************************************************/

#endif /* __SYS_TYPES_H */
