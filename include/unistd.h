/************************************************************
 * unistd.h
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

#ifndef __UNISTD_H
#define __UNISTD_H

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <nuttx/compiler.h>

/************************************************************
 * Definitions
 ************************************************************/

/* The number of functions that may be registerd to be called
 * at program exit.
 */

#define ATEXIT_MAX 1

/* Values for seeking */

#define SEEK_SET    0  /* From the start of the file */
#define SEEK_CUR    1  /* From the current file offset */
#define SEEK_END    2  /* From the end of the file */

/* Bit values for the second argument to access */

#define F_OK        0  /* Test existence */
#define R_OK        1  /* Test read permission */
#define W_OK        2  /* Test write permission */
#define X_OK        4  /* Test execute permission */

/* POSIX feature set macros */

#define  POSIX_VERSION
#undef  _POSIX_SAVED_IDS
#undef  _POSIX_JOB_CONTROL
#define _POSIX_REALTIME_SIGNALS 1
#define _POSIX_MESSAGE_PASSING 1
#undef  _POSIX_MAPPED_FILES
#undef  _POSIX_SHARED_MEMORY_OBJECTS
#define _POSIX_PRIORITY_SCHEDULING 1
#define _POSIX_TIMERS
#undef  _POSIX_MEMLOCK
#undef  _POSIX_MEMLOCK_RANGE
#undef  _POSIX_FSYNC
#define _POSIX_SYNCHRONIZED_IO
#undef  _POSIX_ASYNCHRONOUS_IO
#undef  _POSIX_PRIORITIZED_IO

/* Execution time constants (not supported) */

#undef  _POSIX_CHOWN_RESTRICTED
#undef  _POSIX_NO_TRUNC
#undef  _POSIX_VDISABLE

#define _POSIX_SYNC_IO
#undef  _POSIX_ASYNC_IO
#undef  _POSIX_PRIO_IO

#define fsync(f)

/************************************************************
 * Global Function Prototypes
 ************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Task Control Interfaces (based on ANSII APIs) */

EXTERN pid_t getpid(void);
EXTERN void _exit(int status) noreturn_function;
EXTERN unsigned int sleep(unsigned int seconds);
EXTERN void  usleep(unsigned long usec);

/* File descriptor operations */

EXTERN int   close(int fd);
EXTERN int   dup(int fildes);
EXTERN int   dup2(int fildes1, int fildes2);
EXTERN off_t lseek(int fd, off_t offset, int whence);
EXTERN int   read(int fd, void *buf, unsigned int nbytes);
EXTERN int   unlink(const char *path);
EXTERN int   write(int fd, const void *buf, unsigned int nbytes);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __UNISTD_H */
