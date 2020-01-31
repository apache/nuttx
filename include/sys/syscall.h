/****************************************************************************
 * include/sys/syscall.h
 * This file contains the system call numbers.
 *
 *   Copyright (C) 2011-2019 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_SYSCALL_H
#define __INCLUDE_SYS_SYSCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#ifdef CONFIG_LIB_SYSCALL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Reserve the first system calls for platform-specific usage if so
 * configured.
 */

#ifndef CONFIG_SYS_RESERVED
#  define CONFIG_SYS_RESERVED          (0)
#endif

/* System call numbers
 *
 * These first system calls are supported regardless of the NuttX
 * configuration
 */

#define SYS__exit                      (CONFIG_SYS_RESERVED + 0)
#define SYS_exit                       (CONFIG_SYS_RESERVED + 1)
#define SYS_get_errno                  (CONFIG_SYS_RESERVED + 2)
#define SYS_getpid                     (CONFIG_SYS_RESERVED + 3)

#define SYS_sched_getparam             (CONFIG_SYS_RESERVED + 4)
#define SYS_sched_getscheduler         (CONFIG_SYS_RESERVED + 5)
#define SYS_sched_lock                 (CONFIG_SYS_RESERVED + 6)
#define SYS_sched_lockcount            (CONFIG_SYS_RESERVED + 7)
#define SYS_sched_rr_get_interval      (CONFIG_SYS_RESERVED + 8)
#define SYS_sched_setparam             (CONFIG_SYS_RESERVED + 9)
#define SYS_sched_setscheduler         (CONFIG_SYS_RESERVED + 10)
#define SYS_sched_unlock               (CONFIG_SYS_RESERVED + 11)
#define SYS_sched_yield                (CONFIG_SYS_RESERVED + 12)
#ifdef CONFIG_SMP
#  define SYS_sched_getcpu             (CONFIG_SYS_RESERVED + 13)
#  define __SYS_set_errno              (CONFIG_SYS_RESERVED + 14)
#else
#  define __SYS_set_errno              (CONFIG_SYS_RESERVED + 13)
#endif

#define SYS_set_errno                  (__SYS_set_errno + 0)
#define SYS_uname                      (__SYS_set_errno + 1)
#define __SYS_uid                      (__SYS_set_errno + 2)

/* User identity */

#ifdef CONFIG_SCHED_USER_IDENTITY
#  define SYS_setuid                   (__SYS_uid + 0)
#  define SYS_getuid                   (__SYS_uid + 1)
#  define SYS_setgid                   (__SYS_uid + 2)
#  define SYS_getgid                   (__SYS_uid + 3)
#  define __SYS_sem                    (__SYS_uid + 4)
#else
#  define __SYS_sem                     __SYS_uid
#endif

/* Semaphores */

#define SYS_sem_destroy                (__SYS_sem + 0)
#define SYS_sem_post                   (__SYS_sem + 1)
#define SYS_sem_timedwait              (__SYS_sem + 2)
#define SYS_sem_trywait                (__SYS_sem + 3)
#define SYS_sem_wait                   (__SYS_sem + 4)

#ifdef CONFIG_PRIORITY_INHERITANCE
#  define SYS_sem_setprotocol          (__SYS_sem + 5)
#  define __SYS_named_sem              (__SYS_sem + 6)
#else
#  define __SYS_named_sem              (__SYS_sem + 5)
#endif

/* Named semaphores */

#ifdef CONFIG_FS_NAMED_SEMAPHORES
#  define SYS_sem_open                 __SYS_named_sem
#  define SYS_sem_close                (__SYS_named_sem + 1)
#  define SYS_sem_unlink               (__SYS_named_sem + 2)
#  define __SYS_task_create            (__SYS_named_sem + 3)
#else
#  define __SYS_task_create            __SYS_named_sem
#endif

/* Task creation APIs based on global entry points cannot be use with
 * address environments.
 */

#ifndef CONFIG_BUILD_KERNEL
#  define SYS_task_create              __SYS_task_create
#ifdef CONFIG_BUILD_PROTECTED
#  define SYS_nx_task_spawn            (__SYS_task_create + 1)
#  define __SYS_task_delete            (__SYS_task_create + 2)
#else
#  define __SYS_task_delete            (__SYS_task_create + 1)
#endif

/* pgalloc() is only available with address environments with the page
 * allocator selected.  MMU support from the CPU is also required.
 */

#else
#  define SYS_pgalloc                  __SYS_task_create
#  define __SYS_task_delete            (__SYS_task_create + 1)
#endif

#  define SYS_task_delete              __SYS_task_delete
#  define SYS_task_restart             (__SYS_task_delete + 1)
#  define SYS_task_setcancelstate      (__SYS_task_delete + 2)
#  define SYS_up_assert                (__SYS_task_delete + 3)

#  ifdef CONFIG_CANCELLATION_POINTS
#    define SYS_task_setcanceltype     (__SYS_task_delete + 4)
#    define SYS_task_testcancel        (__SYS_task_delete + 5)
#    define __SYS_vfork                (__SYS_task_delete + 6)
#  else
#    define __SYS_vfork                (__SYS_task_delete + 4)
#  endif

/* The following can be individually enabled */

#if defined(CONFIG_SCHED_WAITPID) && defined(CONFIG_ARCH_HAVE_VFORK)
#  define SYS_vfork                    __SYS_vfork
#  define __SYS_atexit                 (__SYS_vfork + 1)
#else
#  define __SYS_atexit                 __SYS_vfork
#endif

#ifdef CONFIG_SCHED_ATEXIT
#  define SYS_atexit                   __SYS_atexit
#  define __SYS_on_exit                (__SYS_atexit + 1)
#else
#  define __SYS_on_exit                __SYS_atexit
#endif

#ifdef CONFIG_SCHED_ONEXIT
#  define SYS_on_exit                  __SYS_on_exit
#  define __SYS_waitpid                (__SYS_on_exit + 1)
#else
#  define __SYS_waitpid                __SYS_on_exit
#endif

#ifdef CONFIG_SCHED_WAITPID
#  define SYS_waitpid                  __SYS_waitpid
#  ifdef CONFIG_SCHED_HAVE_PARENT
#    define SYS_wait                   (__SYS_waitpid + 1)
#    define SYS_waitid                 (__SYS_waitpid + 2)
#    define __SYS_insmod               (__SYS_waitpid + 3)
#  else
#    define __SYS_insmod               (__SYS_waitpid + 1)
#endif
#else
#  define __SYS_insmod                 __SYS_waitpid
#endif

/* The following can only be defined if we are configured to load
 * OS modules from a file system.
 */

#ifdef CONFIG_MODULE
#  define SYS_insmod                   __SYS_insmod
#  define SYS_rmmod                   (__SYS_insmod + 1)
#  define SYS_modhandle               (__SYS_insmod + 2)
#  define __SYS_exec                  (__SYS_insmod + 3)
#else
#  define __SYS_exec                   __SYS_insmod
#endif

/* The following can only be defined if we are configured to execute
 * programs from a file system.
 */

#ifndef CONFIG_BINFMT_DISABLE
#  ifndef CONFIG_BUILD_KERNEL
#    define SYS_exec                   __SYS_exec
#    define __SYS_posix_spawn          (__SYS_exec + 1)
#  else
#    define __SYS_posix_spawn          __SYS_exec
#  endif
#  ifdef CONFIG_LIBC_EXECFUNCS
#    ifdef CONFIG_LIB_ENVPATH
#      define SYS_posix_spawnp         __SYS_posix_spawn
#    else
#      define SYS_posix_spawn          __SYS_posix_spawn
#    endif
#    define SYS_execv                  (__SYS_posix_spawn + 1)
#    define __SYS_signals              (__SYS_posix_spawn + 2)
#  else
#    define __SYS_signals              __SYS_posix_spawn
#  endif
#else
#  define __SYS_signals                __SYS_exec
#endif

/* The following are only defined is signals are supported in the NuttX
 * configuration.
 */

#define SYS_kill                       (__SYS_signals + 0)
#define SYS_sigaction                  (__SYS_signals + 1)
#define SYS_sigpending                 (__SYS_signals + 2)
#define SYS_sigprocmask                (__SYS_signals + 3)
#define SYS_sigqueue                   (__SYS_signals + 4)
#define SYS_sigsuspend                 (__SYS_signals + 5)
#define SYS_sigtimedwait               (__SYS_signals + 6)
#define SYS_sigwaitinfo                (__SYS_signals + 7)
#define SYS_clock_nanosleep            (__SYS_signals + 8)
#define __SYS_clock                    (__SYS_signals + 9)

/* The following are only defined if the system clock is enabled in the
 * NuttX configuration.
 */

#define SYS_clock                      (__SYS_clock + 0)
#define SYS_clock_getres               (__SYS_clock + 1)
#define SYS_clock_gettime              (__SYS_clock + 2)
#define SYS_clock_settime              (__SYS_clock + 3)
#ifdef CONFIG_CLOCK_TIMEKEEPING
#  define SYS_adjtime                  (__SYS_clock + 4)
#  define __SYS_timers                 (__SYS_clock + 5)
#else
#  define __SYS_timers                 (__SYS_clock + 4)
#endif

/* The following are defined only if POSIX timers are supported */

#ifndef CONFIG_DISABLE_POSIX_TIMERS
#  define SYS_timer_create             (__SYS_timers + 0)
#  define SYS_timer_delete             (__SYS_timers + 1)
#  define SYS_timer_getoverrun         (__SYS_timers + 2)
#  define SYS_timer_gettime            (__SYS_timers + 3)
#  define SYS_timer_settime            (__SYS_timers + 4)
#  define SYS_getitimer                (__SYS_timers + 5)
#  define SYS_setitimer                (__SYS_timers + 6)
#  define __SYS_syslog                 (__SYS_timers + 7)
#else
#  define __SYS_syslog                 __SYS_timers
#endif

/* Unconditional system logging */

#define SYS_nx_vsyslog                 (__SYS_syslog + 0)
#define __SYS_descriptors              (__SYS_syslog + 1)

/* The following are defined if either file or socket descriptor are
 * enabled.
 */

#define SYS_close                    (__SYS_descriptors + 0)

#ifdef CONFIG_LIBC_IOCTL_VARIADIC
#  define SYS_fs_ioctl               (__SYS_descriptors + 1)
#else
#  define SYS_ioctl                  (__SYS_descriptors + 1)
#endif

#define SYS_read                     (__SYS_descriptors + 2)
#define SYS_write                    (__SYS_descriptors + 3)
#define SYS_pread                    (__SYS_descriptors + 4)
#define SYS_pwrite                   (__SYS_descriptors + 5)

#ifdef CONFIG_FS_AIO
#  define SYS_aio_read               (__SYS_descriptors + 6)
#  define SYS_aio_write              (__SYS_descriptors + 7)
#  define SYS_aio_fsync              (__SYS_descriptors + 8)
#  define SYS_aio_cancel             (__SYS_descriptors + 9)
#  define __SYS_poll                 (__SYS_descriptors + 10)
#else
#  define __SYS_poll                 (__SYS_descriptors + 6)
#endif

#define SYS_poll                     __SYS_poll
#define SYS_select                   (__SYS_poll + 1)
#define SYS_ppoll                    (__SYS_poll + 2)
#define SYS_pselect                  (__SYS_poll + 3)
#define __SYS_ifindex                (__SYS_poll + 4)

#ifdef CONFIG_NETDEV_IFINDEX
#  define SYS_if_indextoname         __SYS_ifindex
#  define SYS_if_nametoindex         (__SYS_ifindex + 1)
#  define __SYS_termios              (__SYS_ifindex + 2)
#else
#  define __SYS_termios               __SYS_ifindex
#endif

#ifdef CONFIG_SERIAL_TERMIOS
#  define SYS_tcdrain                __SYS_termios
#  define __SYS_boardctl             (__SYS_termios + 1)
#else
#  define __SYS_boardctl             __SYS_termios
#endif

/* Board support */

#ifdef CONFIG_LIB_BOARDCTL
#    define SYS_boardctl                __SYS_boardctl
#  define __SYS_filedesc                (__SYS_boardctl + 1)
#else
#  define __SYS_filedesc               __SYS_boardctl
#endif

/* The following are defined if file descriptors are enabled */

#define SYS_closedir                   (__SYS_filedesc + 0)
#define SYS_dup                        (__SYS_filedesc + 1)
#define SYS_dup2                       (__SYS_filedesc + 2)
#define SYS_fcntl                      (__SYS_filedesc + 3)
#define SYS_lseek                      (__SYS_filedesc + 4)
#define SYS_mmap                       (__SYS_filedesc + 5)
#define SYS_open                       (__SYS_filedesc + 6)
#define SYS_opendir                    (__SYS_filedesc + 7)
#define SYS_readdir                    (__SYS_filedesc + 8)
#define SYS_rewinddir                  (__SYS_filedesc + 9)
#define SYS_seekdir                    (__SYS_filedesc + 10)
#define SYS_stat                       (__SYS_filedesc + 11)
#define SYS_fstat                      (__SYS_filedesc + 12)
#define SYS_statfs                     (__SYS_filedesc + 13)
#define SYS_fstatfs                    (__SYS_filedesc + 14)
#define SYS_telldir                    (__SYS_filedesc + 15)

#ifdef CONFIG_FS_RAMMAP
#  define SYS_munmap                   (__SYS_filedesc + 16)
#  define __SYS_link                   (__SYS_filedesc + 17)
#else
#  define __SYS_link                   (__SYS_filedesc + 16)
#endif

#if defined(CONFIG_PSEUDOFS_SOFTLINKS)
#  define SYS_link                     (__SYS_link + 0)
#  define SYS_readlink                 (__SYS_link + 1)
#  define __SYS_pipes                  (__SYS_link + 2)
#else
#  define __SYS_pipes                  __SYS_link
#endif

#if defined(CONFIG_PIPES) && CONFIG_DEV_PIPE_SIZE > 0
#  define SYS_pipe2                    (__SYS_pipes + 0)
#  define __SYS_mkfifo2                (__SYS_pipes + 1)
#else
#  define __SYS_mkfifo2                (__SYS_pipes + 0)
#endif

#if defined(CONFIG_PIPES) && CONFIG_DEV_FIFO_SIZE > 0
#  define SYS_mkfifo2                  (__SYS_mkfifo2 + 0)
#  define __SYS_fs_fdopen              (__SYS_mkfifo2 + 1)
#else
#  define __SYS_fs_fdopen              (__SYS_mkfifo2 + 0)
#endif

#if CONFIG_NFILE_STREAMS > 0
#  define SYS_fs_fdopen                (__SYS_fs_fdopen + 0)
#  define SYS_sched_getstreams         (__SYS_fs_fdopen + 1)
#  define __SYS_sendfile               (__SYS_fs_fdopen + 2)
#else
#  define __SYS_sendfile               (__SYS_fs_fdopen + 0)
#endif

#if defined(CONFIG_NET_SENDFILE)
#  define SYS_sendfile,                __SYS_sendfile
#  define __SYS_mountpoint             (__SYS_sendfile + 1)
#else
#  define __SYS_mountpoint             __SYS_sendfile
#endif

#if !defined(CONFIG_DISABLE_MOUNTPOINT)
#if defined(CONFIG_FS_READABLE)
#    define SYS_mount                  (__SYS_mountpoint + 0)
#    define __SYS_fs                   (__SYS_mountpoint + 1)
#else
#    define __SYS_fs                   (__SYS_mountpoint + 0)
#endif
#  define SYS_fsync                    (__SYS_fs + 0)
#  define SYS_ftruncate                (__SYS_fs + 1)
#  define SYS_mkdir                    (__SYS_fs + 2)
#  define SYS_rename                   (__SYS_fs + 3)
#  define SYS_rmdir                    (__SYS_fs + 4)
#  define SYS_umount2                  (__SYS_fs + 5)
#  define SYS_unlink                   (__SYS_fs + 6)
#  define __SYS_shm                    (__SYS_fs + 7)
#else
#  define __SYS_shm                    __SYS_mountpoint
#endif

/* Shared memory interfaces */

#ifdef CONFIG_MM_SHM
#  define SYS_shmget                   (__SYS_shm + 0)
#  define SYS_shmat                    (__SYS_shm + 1)
#  define SYS_shmctl                   (__SYS_shm + 2)
#  define SYS_shmdt                    (__SYS_shm + 3)
#  define __SYS_pthread                (__SYS_shm + 4)
#else
#  define __SYS_pthread                __SYS_shm
#endif

/* The following are defined if pthreads are enabled */

#ifndef CONFIG_DISABLE_PTHREAD
#  define SYS_pthread_cancel           (__SYS_pthread + 0)
#  define SYS_pthread_cond_broadcast   (__SYS_pthread + 1)
#  define SYS_pthread_cond_signal      (__SYS_pthread + 2)
#  define SYS_pthread_cond_wait        (__SYS_pthread + 3)
#  define SYS_pthread_create           (__SYS_pthread + 4)
#  define SYS_pthread_detach           (__SYS_pthread + 5)
#  define SYS_pthread_exit             (__SYS_pthread + 6)
#  define SYS_pthread_get_stackaddr_np (__SYS_pthread + 7)
#  define SYS_pthread_get_stacksize_np (__SYS_pthread + 8)
#  define SYS_pthread_getschedparam    (__SYS_pthread + 9)
#  define SYS_pthread_getspecific      (__SYS_pthread + 10)
#  define SYS_pthread_join             (__SYS_pthread + 11)
#  define SYS_pthread_key_create       (__SYS_pthread + 12)
#  define SYS_pthread_key_delete       (__SYS_pthread + 13)
#  define SYS_pthread_mutex_destroy    (__SYS_pthread + 14)
#  define SYS_pthread_mutex_init       (__SYS_pthread + 15)
#  define SYS_pthread_mutex_timedlock  (__SYS_pthread + 16)
#  define SYS_pthread_mutex_trylock    (__SYS_pthread + 17)
#  define SYS_pthread_mutex_unlock     (__SYS_pthread + 18)

#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
#  define SYS_pthread_mutex_consistent (__SYS_pthread + 19)
#  define __SYS_pthread_setschedparam  (__SYS_pthread + 20)
#else
#  define __SYS_pthread_setschedparam  (__SYS_pthread + 19)
#endif

#  define SYS_pthread_setschedparam    (__SYS_pthread_setschedparam + 0)
#  define SYS_pthread_setschedprio     (__SYS_pthread_setschedparam + 1)
#  define SYS_pthread_setspecific      (__SYS_pthread_setschedparam + 2)
#  define __SYS_pthread_smp            (__SYS_pthread_setschedparam + 3)

#  ifdef CONFIG_SMP
#    define SYS_pthread_setaffinity_np (__SYS_pthread_smp + 0)
#    define SYS_pthread_getaffinity_np (__SYS_pthread_smp + 1)
#    define __SYS_pthread_signals      (__SYS_pthread_smp + 2)
#  else
#    define __SYS_pthread_signals      __SYS_pthread_smp
#  endif

#  define SYS_pthread_cond_timedwait   (__SYS_pthread_signals + 0)
#  define SYS_pthread_kill             (__SYS_pthread_signals + 1)
#  define SYS_pthread_sigmask          (__SYS_pthread_signals + 2)
#  define __SYS_pthread_cleanup        (__SYS_pthread_signals + 3)

#  ifdef CONFIG_PTHREAD_CLEANUP
#    define SYS_pthread_cleanup_push   (__SYS_pthread_cleanup + 0)
#    define SYS_pthread_cleanup_pop    (__SYS_pthread_cleanup + 1)
#    define __SYS_mqueue               (__SYS_pthread_cleanup + 2)
#  else
#    define __SYS_mqueue               __SYS_pthread_cleanup
#  endif

#else
#  define __SYS_mqueue                 __SYS_pthread
#endif

/* The following are defined only if message queues are enabled */

#ifndef CONFIG_DISABLE_MQUEUE
#  define SYS_mq_close                 (__SYS_mqueue + 0)
#  define SYS_mq_getattr               (__SYS_mqueue + 1)
#  define SYS_mq_notify                (__SYS_mqueue + 2)
#  define SYS_mq_open                  (__SYS_mqueue + 3)
#  define SYS_mq_receive               (__SYS_mqueue + 4)
#  define SYS_mq_send                  (__SYS_mqueue + 5)
#  define SYS_mq_setattr               (__SYS_mqueue + 6)
#  define SYS_mq_timedreceive          (__SYS_mqueue + 7)
#  define SYS_mq_timedsend             (__SYS_mqueue + 8)
#  define SYS_mq_unlink                (__SYS_mqueue + 9)
#  define __SYS_environ                (__SYS_mqueue + 10)
#else
#  define __SYS_environ                __SYS_mqueue
#endif

/* The following are defined only if environment variables are supported */

#ifndef CONFIG_DISABLE_ENVIRON
#  define SYS_clearenv                 (__SYS_environ + 0)
#  define SYS_getenv                   (__SYS_environ + 1)
#  define SYS_putenv                   (__SYS_environ + 2)
#  define SYS_setenv                   (__SYS_environ + 3)
#  define SYS_unsetenv                 (__SYS_environ + 4)
#  define __SYS_netdb                  (__SYS_environ + 5)
#else
#  define __SYS_netdb                __SYS_environ
#endif

/* The following are defined if netdb is supported */

#ifdef CONFIG_LIBC_NETDB
#  define SYS_sethostname              (__SYS_netdb + 0)
#  define __SYS_network                (__SYS_netdb + 1)
#else
#  define __SYS_network                __SYS_netdb
#endif

/* The following are defined only if networking AND sockets are supported */

#ifdef CONFIG_NET
#  define SYS_accept                   (__SYS_network + 0)
#  define SYS_bind                     (__SYS_network + 1)
#  define SYS_connect                  (__SYS_network + 2)
#  define SYS_getpeername              (__SYS_network + 3)
#  define SYS_getsockname              (__SYS_network + 4)
#  define SYS_getsockopt               (__SYS_network + 5)
#  define SYS_listen                   (__SYS_network + 6)
#  define SYS_recv                     (__SYS_network + 7)
#  define SYS_recvfrom                 (__SYS_network + 8)
#  define SYS_send                     (__SYS_network + 9)
#  define SYS_sendto                   (__SYS_network + 10)
#  define SYS_setsockopt               (__SYS_network + 11)
#  define SYS_socket                   (__SYS_network + 12)
#else
#  define SYS_socket                    __SYS_network
#endif

/* The following is defined only if CONFIG_TASK_NAME_SIZE > 0 */

#if CONFIG_TASK_NAME_SIZE > 0
#  define SYS_prctl                    (SYS_socket + 1)
#else
#  define SYS_prctl                    SYS_socket
#endif

/* The following is defined only if entropy pool random number generator
 * is enabled.
 */

#ifdef CONFIG_CRYPTO_RANDOM_POOL
#  define SYS_getrandom                (SYS_prctl + 1)
#  define SYS_maxsyscall               (SYS_prctl + 2)
#else
#  define SYS_maxsyscall               (SYS_prctl + 1)
#endif

/* Note that the reported number of system calls does *NOT* include the
 * architecture-specific system calls.  If the "real" total is required,
 * use SYS_maxsyscall.
 */

#define SYS_nsyscalls                  (SYS_maxsyscall-CONFIG_SYS_RESERVED)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef __KERNEL__

/* Function lookup tables.  This table is indexed by the system call numbers
 * defined above.  Given the system call number, this table provides the
 * address of the corresponding system function.
 *
 * This table is only available during the kernel phase of a kernel build.
 */

EXTERN const uintptr_t g_funclookup[SYS_nsyscalls];

/* Given the system call number, the corresponding entry in this table
 * provides the address of the stub function.
 *
 * This table is only available during the kernel phase of a kernel build.
 */

EXTERN const uintptr_t g_stublookup[SYS_nsyscalls];

#endif

/* Given the system call number, the corresponding entry in this table
 * provides the number of parameters needed by the function.
 */

EXTERN const uint8_t g_funcnparms[SYS_nsyscalls];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_LIB_SYSCALL */
#endif /* __INCLUDE_SYS_SYSCALL_H */
