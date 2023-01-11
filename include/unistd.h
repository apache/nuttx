/****************************************************************************
 * include/unistd.h
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

#ifndef __INCLUDE_UNISTD_H
#define __INCLUDE_UNISTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <nuttx/compiler.h>
#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Values for seeking */

#define SEEK_SET    0  /* From the start of the file */
#define SEEK_CUR    1  /* From the current file offset */
#define SEEK_END    2  /* From the end of the file */

/* Bit values for the second argument to access */

#define F_OK        0  /* Test existence */
#define X_OK        1  /* Test execute permission */
#define W_OK        2  /* Test write permission */
#define R_OK        4  /* Test read permission */

/* POSIX feature set macros */

#define POSIX_VERSION
#undef  _POSIX_SAVED_IDS
#undef  _POSIX_JOB_CONTROL
#define _POSIX_REALTIME_SIGNALS 1
#define _POSIX_MESSAGE_PASSING 1
#undef  _POSIX_MAPPED_FILES
#undef  _POSIX_SHARED_MEMORY_OBJECTS
#define _POSIX_PRIORITY_SCHEDULING 1
#ifndef CONFIG_DISABLE_POSIX_TIMERS
#  define _POSIX_TIMERS 1
#endif
#if !defined(CONFIG_DISABLE_MQUEUE) && !defined(CONFIG_DISABLE_PTHREAD)
#  define _POSIX_TIMEOUTS 1
#endif
#undef  _POSIX_MEMLOCK
#undef  _POSIX_MEMLOCK_RANGE
#undef  _POSIX_FSYNC
#define _POSIX_SYNCHRONIZED_IO 1

#ifdef CONFIG_FS_AIO
#  define _POSIX_ASYNCHRONOUS_IO 1
#else
#  undef  _POSIX_ASYNCHRONOUS_IO
#endif

#undef  _POSIX_PRIORITIZED_IO

#ifdef CONFIG_SCHED_SPORADIC
#  define _POSIX_SPORADIC_SERVER 1
#  define _POSIX_THREAD_SPORADIC_SERVER 1
#else
#  define _POSIX_SPORADIC_SERVER -1
#  define _POSIX_THREAD_SPORADIC_SERVER -1
#endif

/* Execution time constants (not supported) */

#undef  _POSIX_CHOWN_RESTRICTED
#undef  _POSIX_NO_TRUNC
#undef  _POSIX_VDISABLE

#define _POSIX_SYNC_IO 1
#undef  _POSIX_ASYNC_IO
#undef  _POSIX_PRIO_IO

/* Constants used with POSIX pathconf().  pathconf() will return -1 and set
 * errno to ENOSYS for most of these.
 */

#define _PC_2_SYMLINKS                   0x0001
#define _PC_ALLOC_SIZE_MIN               0x0002
#define _PC_ASYNC_IO                     0x0003
#define _PC_CHOWN_RESTRICTED             0x0004
#define _PC_FILESIZEBITS                 0x0005
#define _PC_LINK_MAX                     0x0006
#define _PC_MAX_CANON                    0x0007
#define _PC_MAX_INPUT                    0x0008
#define _PC_NAME_MAX                     0x0009
#define _PC_NO_TRUNC                     0x000a
#define _PC_PATH_MAX                     0x000b
#define _PC_PIPE_BUF                     0x000c
#define _PC_PRIO_IO                      0x000d
#define _PC_REC_INCR_XFER_SIZE           0x000e
#define _PC_REC_MIN_XFER_SIZE            0x000f
#define _PC_REC_XFER_ALIGN               0x0010
#define _PC_SYMLINK_MAX                  0x0011
#define _PC_SYNC_IO                      0x0012
#define _PC_VDISABLE                     0x0013

/* Constants used with POSIX sysconf().  sysconf() will return -1 and set
 * errno to ENOSYS for most of these.
 */

#define _SC_2_C_BIND                     0x0001
#define _SC_2_C_DEV                      0x0002
#define _SC_2_CHAR_TERM                  0x0003
#define _SC_2_FORT_DEV                   0x0004
#define _SC_2_FORT_RUN                   0x0005
#define _SC_2_LOCALEDEF                  0x0006
#define _SC_2_PBS                        0x0007
#define _SC_2_PBS_ACCOUNTING             0x0008
#define _SC_2_PBS_CHECKPOINT             0x0009
#define _SC_2_PBS_LOCATE                 0x000a
#define _SC_2_PBS_MESSAGE                0x000b
#define _SC_2_PBS_TRACK                  0x000c
#define _SC_2_SW_DEV                     0x000d
#define _SC_2_UPE                        0x000e
#define _SC_2_VERSION                    0x000f
#define _SC_ADVISORY_INFO                0x0010
#define _SC_AIO_LISTIO_MAX               0x0011
#define _SC_AIO_MAX                      0x0012
#define _SC_AIO_PRIO_DELTA_MAX           0x0013
#define _SC_ARG_MAX                      0x0014
#define _SC_ASYNCHRONOUS_IO              0x0015
#define _SC_ATEXIT_MAX                   0x0016
#define _SC_BARRIERS                     0x0017
#define _SC_BC_BASE_MAX                  0x0018
#define _SC_BC_DIM_MAX                   0x0019
#define _SC_BC_SCALE_MAX                 0x001a
#define _SC_BC_STRING_MAX                0x001b
#define _SC_CHILD_MAX                    0x001c
#define _SC_CLK_TCK                      0x001d
#define _SC_CLOCK_SELECTION              0x001e
#define _SC_COLL_WEIGHTS_MAX             0x001f
#define _SC_CPUTIME                      0x0020
#define _SC_DELAYTIMER_MAX               0x0021
#define _SC_EXPR_NEST_MAX                0x0022
#define _SC_FSYNC                        0x0023
#define _SC_GETGR_R_SIZE_MAX             0x0024
#define _SC_GETPW_R_SIZE_MAX             0x0025
#define _SC_HOST_NAME_MAX                0x0026
#define _SC_IOV_MAX                      0x0027
#define _SC_IPV6                         0x0028
#define _SC_JOB_CONTROL                  0x0029
#define _SC_LINE_MAX                     0x002a
#define _SC_LOGIN_NAME_MAX               0x002b
#define _SC_MAPPED_FILES                 0x002c
#define _SC_MEMLOCK                      0x002d
#define _SC_MEMLOCK_RANGE                0x002e
#define _SC_MEMORY_PROTECTION            0x002f
#define _SC_MESSAGE_PASSING              0x0030
#define _SC_MONOTONIC_CLOCK              0x0031
#define _SC_MQ_OPEN_MAX                  0x0032
#define _SC_MQ_PRIO_MAX                  0x0033
#define _SC_NGROUPS_MAX                  0x0034
#define _SC_OPEN_MAX                     0x0035
#define _SC_PAGE_SIZE                    0x0036
#define _SC_PAGESIZE                     _SC_PAGE_SIZE
#define _SC_PRIORITIZED_IO               0x0037
#define _SC_PRIORITY_SCHEDULING          0x0038
#define _SC_RAW_SOCKETS                  0x0039
#define _SC_RE_DUP_MAX                   0x003a
#define _SC_READER_WRITER_LOCKS          0x003b
#define _SC_REALTIME_SIGNALS             0x003c
#define _SC_REGEXP                       0x003d
#define _SC_RTSIG_MAX                    0x003e
#define _SC_SAVED_IDS                    0x003f
#define _SC_SEM_NSEMS_MAX                0x0040
#define _SC_SEM_VALUE_MAX                0x0041
#define _SC_SEMAPHORES                   0x0042
#define _SC_SHARED_MEMORY_OBJECTS        0x0043
#define _SC_SHELL                        0x0044
#define _SC_SIGQUEUE_MAX                 0x0045
#define _SC_SPAWN                        0x0046
#define _SC_SPIN_LOCKS                   0x0047
#define _SC_SPORADIC_SERVER              0x0048
#define _SC_SS_REPL_MAX                  0x0049
#define _SC_STREAM_MAX                   0x004a
#define _SC_SYMLOOP_MAX                  0x004b
#define _SC_SYNCHRONIZED_IO              0x004c
#define _SC_THREAD_ATTR_STACKADDR        0x004d
#define _SC_THREAD_ATTR_STACKSIZE        0x004e
#define _SC_THREAD_CPUTIME               0x004f
#define _SC_THREAD_DESTRUCTOR_ITERATIONS 0x0050
#define _SC_THREAD_KEYS_MAX              0x0051
#define _SC_THREAD_PRIO_INHERIT          0x0052
#define _SC_THREAD_PRIO_PROTECT          0x0053
#define _SC_THREAD_PRIORITY_SCHEDULING   0x0054
#define _SC_THREAD_PROCESS_SHARED        0x0055
#define _SC_THREAD_SAFE_FUNCTIONS        0x0056
#define _SC_THREAD_SPORADIC_SERVER       0x0057
#define _SC_THREAD_STACK_MIN             0x0058
#define _SC_THREAD_THREADS_MAX           0x0059
#define _SC_THREADS                      0x005a
#define _SC_TIMEOUTS                     0x005b
#define _SC_TIMER_MAX                    0x005c
#define _SC_TIMERS                       0x005d
#define _SC_TRACE                        0x005e
#define _SC_TRACE_EVENT_FILTER           0x005f
#define _SC_TRACE_EVENT_NAME_MAX         0x0060
#define _SC_TRACE_INHERIT                0x0061
#define _SC_TRACE_LOG                    0x0062
#define _SC_TRACE_NAME_MAX               0x0063
#define _SC_TRACE_SYS_MAX                0x0064
#define _SC_TRACE_USER_EVENT_MAX         0x0065
#define _SC_TTY_NAME_MAX                 0x0066
#define _SC_TYPED_MEMORY_OBJECTS         0x0067
#define _SC_TZNAME_MAX                   0x0068
#define _SC_V6_ILP32_OFF32               0x0069
#define _SC_V6_ILP32_OFFBIG              0x006a
#define _SC_V6_LP64_OFF64                0x006b
#define _SC_V6_LPBIG_OFFBIG              0x006c
#define _SC_VERSION                      0x006d
#define _SC_XBS5_ILP32_OFF32             0x006e  /* (LEGACY) */
#define _SC_XBS5_ILP32_OFFBIG            0x006f  /* (LEGACY) */
#define _SC_XBS5_LP64_OFF64              0x0070  /* (LEGACY) */
#define _SC_XBS5_LPBIG_OFFBIG            0x0071  /* (LEGACY) */
#define _SC_XOPEN_CRYPT                  0x0072
#define _SC_XOPEN_ENH_I18N               0x0073
#define _SC_XOPEN_LEGACY                 0x0074
#define _SC_XOPEN_REALTIME               0x0075
#define _SC_XOPEN_REALTIME_THREADS       0x0076
#define _SC_XOPEN_SHM                    0x0077
#define _SC_XOPEN_STREAMS                0x0078
#define _SC_XOPEN_UNIX                   0x0079
#define _SC_XOPEN_VERSION                0x007a

#define _SC_PHYS_PAGES                   0x007b
#define _SC_AVPHYS_PAGES                 0x007c

#define _SC_NPROCESSORS_CONF             0x007d
#define _SC_NPROCESSORS_ONLN             0x007e

/* The following symbolic constants must be defined for file streams: */

#define STDERR_FILENO                    2       /* File number of stderr */
#define STDIN_FILENO                     0       /* File number of stdin */
#define STDOUT_FILENO                    1       /* File number of stdout */

/* Helpers and legacy compatibility definitions */

#define link(p1, p2)                     symlink((p1), (p2))
#define syncfs(f)                        fsync(f)
#define fdatasync(f)                     fsync(f)
#define getdtablesize(f)                 ((int)sysconf(_SC_OPEN_MAX))
#define getpagesize(f)                   ((int)sysconf(_SC_PAGESIZE))

/* Accessor functions associated with getopt(). */

#define optarg                           (*(getoptargp()))
#define opterr                           (*(getopterrp()))
#define optind                           (*(getoptindp()))
#define optopt                           (*(getoptoptp()))

#if defined(CONFIG_FS_LARGEFILE) && defined(CONFIG_HAVE_LONG_LONG)
#  define lseek64                        lseek
#  define pread64                        pread
#  define pwrite64                       pwrite
#  define truncate64                     truncate
#  define ftruncate64                    ftruncate
#  define lockf64                        lockf
#endif

/* NOTE: NuttX provides only one implementation:  If
 * CONFIG_LIBC_ENVPATH is defined, then only execvp/execlp/execvpe behavior
 * is supported; otherwise, only execv/execl/execve behavior is supported.
 */

#ifdef CONFIG_LIBC_EXECFUNCS
#  define execvp                         execv
#  define execlp                         execl
#  define execvpe                        execve
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Task Control Interfaces */

pid_t   vfork(void);
pid_t   getpid(void);
pid_t   gettid(void);
pid_t   getppid(void);
void    _exit(int status) noreturn_function;
unsigned int sleep(unsigned int seconds);
int     usleep(useconds_t usec);
int     pause(void);
int     nice(int inc);

int     daemon(int nochdir, int noclose);

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
int     ftruncate(int fd, off_t length);
int     fchown(int fd, uid_t owner, gid_t group);

/* Check if a file descriptor corresponds to a terminal I/O file */

int     isatty(int fd);

FAR char *ttyname(int fd);
int       ttyname_r(int fd, FAR char *buf, size_t buflen);

/* Memory management */

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_MM_PGALLOC) && \
    defined(CONFIG_ARCH_USE_MMU)
FAR void *sbrk(intptr_t incr);
#endif

/* Special devices */

#define pipe(fd) pipe2(fd, 0)
int     pipe2(int pipefd[2], int flags);

/* Schedule an alarm */

unsigned int alarm(unsigned int seconds);

/* Working directory operations */

int     chdir(FAR const char *path);
int     fchdir(int fd);
FAR char *getcwd(FAR char *buf, size_t size);

/* File path operations */

int     access(FAR const char *path, int amode);
int     rmdir(FAR const char *pathname);
int     unlink(FAR const char *pathname);
int     truncate(FAR const char *path, off_t length);
int     symlink(FAR const char *path1, FAR const char *path2);
ssize_t readlink(FAR const char *path, FAR char *buf, size_t bufsize);
int     chown(FAR const char *path, uid_t owner, gid_t group);
int     lchown(FAR const char *path, uid_t owner, gid_t group);

/* Execution of programs from files */

#ifdef CONFIG_LIBC_EXECFUNCS
int     execl(FAR const char *path, FAR const char *arg0, ...);
int     execle(FAR const char *path, FAR const char *arg0, ...);
int     execv(FAR const char *path, FAR char * const argv[]);
int     execve(FAR const char *path, FAR char *const argv[],
               FAR char *const envp[]);
#endif

/* Byte operations */

void    swab(FAR const void *src, FAR void *dest, ssize_t nbytes);

/* getopt and friends */

int     getopt(int argc, FAR char * const argv[], FAR const char *optstring);

/* Accessor functions associated with getopt(). */

FAR char **getoptargp(void);  /* Optional argument following option */
FAR int   *getopterrp(void);  /* Print error message */
FAR int   *getoptindp(void);  /* Index into argv */
FAR int   *getoptoptp(void);  /* Unrecognized option character */

int     gethostname(FAR char *name, size_t namelen);
int     sethostname(FAR const char *name, size_t namelen);

/* Get configurable system variables */

long    sysconf(int name);
long    fpathconf(int fildes, int name);
long    pathconf(FAR const char *path, int name);

/* User and group identity management */

int     setuid(uid_t uid);
uid_t   getuid(void);
int     setgid(gid_t gid);
gid_t   getgid(void);

int     seteuid(uid_t uid);
uid_t   geteuid(void);
int     setegid(gid_t gid);
gid_t   getegid(void);

int     setreuid(uid_t ruid, uid_t euid);
int     setregid(gid_t rgid, gid_t egid);

int     getentropy(FAR void *buffer, size_t length);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_UNISTD_H */
