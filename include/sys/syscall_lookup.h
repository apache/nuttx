/****************************************************************************
 * include/sys/syscall_lookup.h
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

/* SYSCALL_LOOKUP must be defined before including this file.
 *
 * These first system calls are supported regardless of the NuttX
 * configuration
 */

SYSCALL_LOOKUP1(_exit,                     1)
SYSCALL_LOOKUP(_assert,                    2)
SYSCALL_LOOKUP(getpid,                     0)
SYSCALL_LOOKUP(gettid,                     0)
SYSCALL_LOOKUP(prctl,                      2)

#ifdef CONFIG_SCHED_HAVE_PARENT
  SYSCALL_LOOKUP(getppid,                  0)
#endif

SYSCALL_LOOKUP(sched_getparam,             2)
SYSCALL_LOOKUP(sched_getscheduler,         1)
SYSCALL_LOOKUP(sched_lock,                 0)
SYSCALL_LOOKUP(sched_lockcount,            0)
SYSCALL_LOOKUP(sched_rr_get_interval,      2)
SYSCALL_LOOKUP(sched_setparam,             2)
SYSCALL_LOOKUP(sched_setscheduler,         3)
SYSCALL_LOOKUP(sched_unlock,               0)
SYSCALL_LOOKUP(sched_yield,                0)
SYSCALL_LOOKUP(nxsched_get_stackinfo,      2)

#ifdef CONFIG_SCHED_BACKTRACE
  SYSCALL_LOOKUP(sched_backtrace,          4)
#endif

#ifdef CONFIG_SMP
  SYSCALL_LOOKUP(sched_getaffinity,        3)
  SYSCALL_LOOKUP(sched_getcpu,             0)
  SYSCALL_LOOKUP(sched_setaffinity,        3)
#endif

SYSCALL_LOOKUP(sysinfo,                    1)

SYSCALL_LOOKUP(gethostname,                2)
SYSCALL_LOOKUP(sethostname,                2)

/* User identity */

#ifdef CONFIG_SCHED_USER_IDENTITY
  SYSCALL_LOOKUP(setuid,                   1)
  SYSCALL_LOOKUP(getuid,                   0)
  SYSCALL_LOOKUP(setgid,                   1)
  SYSCALL_LOOKUP(getgid,                   0)
#endif

/* Semaphores */

SYSCALL_LOOKUP(sem_destroy,                1)
SYSCALL_LOOKUP(sem_post,                   1)
SYSCALL_LOOKUP(sem_clockwait,              3)
SYSCALL_LOOKUP(sem_timedwait,              2)
SYSCALL_LOOKUP(sem_trywait,                1)
SYSCALL_LOOKUP(sem_wait,                   1)

#ifdef CONFIG_PRIORITY_INHERITANCE
  SYSCALL_LOOKUP(sem_setprotocol,          2)
#endif

/* Named semaphores */

#ifdef CONFIG_FS_NAMED_SEMAPHORES
  SYSCALL_LOOKUP(sem_open,                 4)
  SYSCALL_LOOKUP(sem_close,                1)
  SYSCALL_LOOKUP(sem_unlink,               1)
#endif

#ifndef CONFIG_BUILD_KERNEL
  SYSCALL_LOOKUP(task_create,              5)
  SYSCALL_LOOKUP(task_spawn,               6)
  SYSCALL_LOOKUP(task_delete,              1)
  SYSCALL_LOOKUP(task_restart,             1)
#else
  SYSCALL_LOOKUP(pgalloc,                  2)
#endif

SYSCALL_LOOKUP(task_setcancelstate,        2)

#ifdef CONFIG_CANCELLATION_POINTS
  SYSCALL_LOOKUP(task_setcanceltype,       2)
  SYSCALL_LOOKUP(task_testcancel,          0)
#endif

#if CONFIG_TLS_TASK_NELEM > 0
  SYSCALL_LOOKUP(task_tls_alloc,           1)
#endif

/* The following can be individually enabled */

#if defined(CONFIG_SCHED_WAITPID) && defined(CONFIG_ARCH_HAVE_VFORK)
  SYSCALL_LOOKUP(vfork,                    0)
#endif

#ifdef CONFIG_SCHED_WAITPID
  SYSCALL_LOOKUP(waitpid,                  3)
#ifdef CONFIG_SCHED_HAVE_PARENT
  SYSCALL_LOOKUP(wait,                     1)
  SYSCALL_LOOKUP(waitid,                   4)
#endif
#endif

/* The following can only be defined if we are configured to load
 * OS modules from a file system.
 */

#ifdef CONFIG_MODULE
  SYSCALL_LOOKUP(insmod,                   2)
  SYSCALL_LOOKUP(rmmod,                    1)
  SYSCALL_LOOKUP(modhandle,                1)
#endif

/* The following can only be defined if we are configured to execute
 * programs from a file system.
 */

#ifndef CONFIG_BINFMT_DISABLE
#ifndef CONFIG_BUILD_KERNEL
  SYSCALL_LOOKUP(exec,                     4)
#endif
#ifdef CONFIG_LIBC_EXECFUNCS
  SYSCALL_LOOKUP(posix_spawn,              6)
  SYSCALL_LOOKUP(execve,                   3)
#endif
#endif

/* The following are only defined is signals are supported in the NuttX
 * configuration.
 */

SYSCALL_LOOKUP(kill,                       2)
SYSCALL_LOOKUP(sigaction,                  3)
SYSCALL_LOOKUP(sigpending,                 1)
SYSCALL_LOOKUP(sigprocmask,                3)
SYSCALL_LOOKUP(sigqueue,                   3)
SYSCALL_LOOKUP(sigsuspend,                 1)
SYSCALL_LOOKUP(sigtimedwait,               3)
SYSCALL_LOOKUP(sigwaitinfo,                2)
SYSCALL_LOOKUP(clock_nanosleep,            4)

/* The following are only defined if the system clock is enabled in the
 * NuttX configuration.
 */

SYSCALL_LOOKUP(clock,                      0)
SYSCALL_LOOKUP(clock_getres,               2)
SYSCALL_LOOKUP(clock_gettime,              2)
SYSCALL_LOOKUP(clock_settime,              2)
#ifdef CONFIG_CLOCK_TIMEKEEPING
  SYSCALL_LOOKUP(adjtime,                  2)
#endif

/* The following are defined only if POSIX timers are supported */

#ifndef CONFIG_DISABLE_POSIX_TIMERS
  SYSCALL_LOOKUP(timer_create,             3)
  SYSCALL_LOOKUP(timer_delete,             1)
  SYSCALL_LOOKUP(timer_getoverrun,         1)
  SYSCALL_LOOKUP(timer_gettime,            2)
  SYSCALL_LOOKUP(timer_settime,            4)
  SYSCALL_LOOKUP(getitimer,                2)
  SYSCALL_LOOKUP(setitimer,                3)
#endif

/* System logging */

SYSCALL_LOOKUP(nx_vsyslog,                 3)

/* The following are defined if either file or socket descriptor are
 * enabled.
 */

SYSCALL_LOOKUP(close,                      1)
SYSCALL_LOOKUP(ioctl,                      3)
SYSCALL_LOOKUP(read,                       3)
SYSCALL_LOOKUP(write,                      3)
SYSCALL_LOOKUP(pread,                      4)
SYSCALL_LOOKUP(pwrite,                     4)
#ifdef CONFIG_FS_AIO
  SYSCALL_LOOKUP(aio_read,                 1)
  SYSCALL_LOOKUP(aio_write,                1)
  SYSCALL_LOOKUP(aio_fsync,                2)
  SYSCALL_LOOKUP(aio_cancel,               2)
#endif
  SYSCALL_LOOKUP(poll,                     3)
  SYSCALL_LOOKUP(select,                   5)
  SYSCALL_LOOKUP(ppoll,                    4)
  SYSCALL_LOOKUP(pselect,                  6)
#ifdef CONFIG_EVENT_FD
  SYSCALL_LOOKUP(eventfd,                  2)
#endif
#ifdef CONFIG_TIMER_FD
  SYSCALL_LOOKUP(timerfd_create,           2)
  SYSCALL_LOOKUP(timerfd_settime,          4)
  SYSCALL_LOOKUP(timerfd_gettime,          2)
#endif

/* Board support */

#ifdef CONFIG_BOARDCTL
  SYSCALL_LOOKUP(boardctl,                 2)
#endif

/* The following are defined if file descriptors are enabled */

SYSCALL_LOOKUP(dup,                        1)
SYSCALL_LOOKUP(dup2,                       2)
SYSCALL_LOOKUP(fcntl,                      3)
SYSCALL_LOOKUP(lseek,                      3)
SYSCALL_LOOKUP(mmap,                       6)
SYSCALL_LOOKUP(open,                       3)
SYSCALL_LOOKUP(stat,                       2)
SYSCALL_LOOKUP(lstat,                      2)
SYSCALL_LOOKUP(fstat,                      2)
SYSCALL_LOOKUP(statfs,                     2)
SYSCALL_LOOKUP(fstatfs,                    2)
SYSCALL_LOOKUP(sendfile,                   4)
SYSCALL_LOOKUP(chmod,                      2)
SYSCALL_LOOKUP(lchmod,                     2)
SYSCALL_LOOKUP(fchmod,                     2)
SYSCALL_LOOKUP(chown,                      3)
SYSCALL_LOOKUP(lchown,                     3)
SYSCALL_LOOKUP(fchown,                     3)
SYSCALL_LOOKUP(utimens,                    2)
SYSCALL_LOOKUP(lutimens,                   2)
SYSCALL_LOOKUP(futimens,                   2)
SYSCALL_LOOKUP(munmap,                     2)

#if defined(CONFIG_PSEUDOFS_SOFTLINKS)
  SYSCALL_LOOKUP(symlink,                  2)
  SYSCALL_LOOKUP(readlink,                 3)
#endif

#if defined(CONFIG_PIPES) && CONFIG_DEV_PIPE_SIZE > 0
  SYSCALL_LOOKUP(pipe2,                    2)
#endif

#if defined(CONFIG_PIPES) && CONFIG_DEV_FIFO_SIZE > 0
  SYSCALL_LOOKUP(nx_mkfifo,                3)
#endif

#ifdef CONFIG_FILE_STREAM
  SYSCALL_LOOKUP(fs_fdopen,                4)
#endif

#ifndef CONFIG_DISABLE_MOUNTPOINT
  SYSCALL_LOOKUP(mount,                    5)
  SYSCALL_LOOKUP(fsync,                    1)
  SYSCALL_LOOKUP(ftruncate,                2)
  SYSCALL_LOOKUP(mkdir,                    2)
  SYSCALL_LOOKUP(rename,                   2)
  SYSCALL_LOOKUP(rmdir,                    1)
  SYSCALL_LOOKUP(umount2,                  2)
  SYSCALL_LOOKUP(unlink,                   1)
#endif

/* Shared memory interfaces */

#ifdef CONFIG_MM_SHM
  SYSCALL_LOOKUP(shmget,                   3)
  SYSCALL_LOOKUP(shmat,                    3)
  SYSCALL_LOOKUP(shmctl,                   3)
  SYSCALL_LOOKUP(shmdt,                    1)
#endif

/* The following are defined if pthreads are enabled */

#ifndef CONFIG_DISABLE_PTHREAD
  SYSCALL_LOOKUP(pthread_cancel,           1)
  SYSCALL_LOOKUP(pthread_cond_broadcast,   1)
  SYSCALL_LOOKUP(pthread_cond_signal,      1)
  SYSCALL_LOOKUP(pthread_cond_wait,        2)
  SYSCALL_LOOKUP(nx_pthread_create,        5)
  SYSCALL_LOOKUP(pthread_detach,           1)
  SYSCALL_LOOKUP(nx_pthread_exit,          1)
  SYSCALL_LOOKUP(pthread_getschedparam,    3)
  SYSCALL_LOOKUP(pthread_join,             2)
  SYSCALL_LOOKUP(pthread_mutex_destroy,    1)
  SYSCALL_LOOKUP(pthread_mutex_init,       2)
  SYSCALL_LOOKUP(pthread_mutex_timedlock,  2)
  SYSCALL_LOOKUP(pthread_mutex_trylock,    1)
  SYSCALL_LOOKUP(pthread_mutex_unlock,     1)
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
  SYSCALL_LOOKUP(pthread_mutex_consistent, 1)
#endif
  SYSCALL_LOOKUP(pthread_setschedparam,    3)
  SYSCALL_LOOKUP(pthread_setschedprio,     2)
#ifdef CONFIG_SMP
  SYSCALL_LOOKUP(pthread_setaffinity_np,   3)
  SYSCALL_LOOKUP(pthread_getaffinity_np,   3)
#endif
  SYSCALL_LOOKUP(pthread_cond_clockwait,   4)
  SYSCALL_LOOKUP(pthread_kill,             2)
  SYSCALL_LOOKUP(pthread_sigmask,          3)
#endif

/* The following are defined only if message queues are enabled */

#ifndef CONFIG_DISABLE_MQUEUE
  SYSCALL_LOOKUP(mq_close,                 1)
  SYSCALL_LOOKUP(mq_getattr,               2)
  SYSCALL_LOOKUP(mq_notify,                2)
  SYSCALL_LOOKUP(mq_open,                  4)
  SYSCALL_LOOKUP(mq_receive,               4)
  SYSCALL_LOOKUP(mq_send,                  4)
  SYSCALL_LOOKUP(mq_setattr,               3)
  SYSCALL_LOOKUP(mq_timedreceive,          5)
  SYSCALL_LOOKUP(mq_timedsend,             5)
  SYSCALL_LOOKUP(mq_unlink,                1)
#endif

/* The following are defined only if environment variables are supported */

#ifndef CONFIG_DISABLE_ENVIRON
  SYSCALL_LOOKUP(get_environ_ptr,          0)
  SYSCALL_LOOKUP(clearenv,                 0)
  SYSCALL_LOOKUP(getenv,                   1)
  SYSCALL_LOOKUP(putenv,                   1)
  SYSCALL_LOOKUP(setenv,                   3)
  SYSCALL_LOOKUP(unsetenv,                 1)
#endif

/* The following are defined only if networking AND sockets are supported */

#ifdef CONFIG_NET
  SYSCALL_LOOKUP(accept4,                  4)
  SYSCALL_LOOKUP(bind,                     3)
  SYSCALL_LOOKUP(connect,                  3)
  SYSCALL_LOOKUP(getpeername,              3)
  SYSCALL_LOOKUP(getsockname,              3)
  SYSCALL_LOOKUP(getsockopt,               5)
  SYSCALL_LOOKUP(listen,                   2)
  SYSCALL_LOOKUP(recv,                     4)
  SYSCALL_LOOKUP(recvfrom,                 6)
  SYSCALL_LOOKUP(recvmsg,                  3)
  SYSCALL_LOOKUP(send,                     4)
  SYSCALL_LOOKUP(sendto,                   6)
  SYSCALL_LOOKUP(sendmsg,                  3)
  SYSCALL_LOOKUP(setsockopt,               5)
  SYSCALL_LOOKUP(socket,                   3)
  SYSCALL_LOOKUP(socketpair,               4)
#endif

/* The following is defined only if entropy pool random number generator
 * is enabled.
 */

#ifdef CONFIG_CRYPTO_RANDOM_POOL
  SYSCALL_LOOKUP(arc4random_buf,           2)
#endif
