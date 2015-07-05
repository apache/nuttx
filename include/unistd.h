/****************************************************************************
 * include/unistd.h
 *
 *   Copyright (C) 2007-2009, 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_UNISTD_H
#define __INCLUDE_UNISTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The number of functions that may be registered to be called
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
#define _POSIX_TIMERS 1
#undef  _POSIX_MEMLOCK
#undef  _POSIX_MEMLOCK_RANGE
#undef  _POSIX_FSYNC
#define _POSIX_SYNCHRONIZED_IO 1
#undef  _POSIX_ASYNCHRONOUS_IO
#undef  _POSIX_PRIORITIZED_IO

/* Execution time constants (not supported) */

#undef  _POSIX_CHOWN_RESTRICTED
#undef  _POSIX_NO_TRUNC
#undef  _POSIX_VDISABLE

#define _POSIX_SYNC_IO 1
#undef  _POSIX_ASYNC_IO
#undef  _POSIX_PRIO_IO

#define fdatasync(f) fsync(f)

#define HOST_NAME_MAX 32

/****************************************************************************
 * Global Variables
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Used by getopt (obviously NOT thread safe!).  These variables cannot be
 * accessed directly by an external NXFLAT module.  In that case, accessor
 * functions must be used.
 */

#ifndef __NXFLAT__
EXTERN FAR char *optarg; /* Optional argument following option */
EXTERN int       optind; /* Index into argv */
EXTERN int       optopt; /* unrecognized option character */
#else
#  define optarg  (*(getoptargp()))
#  define optind  (*(getoptindp()))
#  define optopt  (*(getoptoptp()))
#endif

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

/* Task Control Interfaces */

pid_t   vfork(void);
pid_t   getpid(void);
void    _exit(int status) noreturn_function;
unsigned int sleep(unsigned int seconds);
int     usleep(useconds_t usec);
int     pause(void);

/* File descriptor operations */

int     close(int fd);
int     dup(int fd);
int     dup2(int fd1, int fd2);
int     fsync(int fd);
off_t   lseek(int fd, off_t offset, int whence);
ssize_t read(int fd, FAR void *buf, size_t nbytes);
ssize_t write(int fd, FAR const void *buf, size_t nbytes);
ssize_t pread(int fd, FAR void *buf, size_t nbytes, off_t offset);
ssize_t pwrite(int fd, FAR const void *buf, size_t nbytes, off_t offset);

/* Memory management */

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_MM_PGALLOC) && \
    defined(CONFIG_ARCH_USE_MMU)
FAR void *sbrk(intptr_t incr);
#endif

/* Special devices */

int     pipe(int fd[2]);

/* Working directory operations */

int     chdir(FAR const char *path);
FAR char *getcwd(FAR char *buf, size_t size);

/* File path operations */

int     access(FAR const char *path, int amode);
int     rmdir(FAR const char *pathname);
int     unlink(FAR const char *pathname);

/* Execution of programs from files */

#ifdef CONFIG_LIBC_EXECFUNCS
int     execl(FAR const char *path, ...);
int     execv(FAR const char *path, FAR char *const argv[]);
#endif

/* Other */

int     getopt(int argc, FAR char *const argv[], FAR const char *optstring);

/* Accessor functions intended for use only by external NXFLAT
 * modules.  The global variables optarg, optind, and optopt cannot
 * be referenced directly from external modules.
 */

FAR char **getoptargp(void); /* Optional argument following option */
int       *getoptindp(void); /* Index into argv */
int       *getoptoptp(void); /* unrecognized option character */

#ifdef CONFIG_NET
int     gethostname(FAR char *name, size_t size);
int     sethostname(FAR const char *name, size_t size);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_UNISTD_H */
