/****************************************************************************
 * syscall/syscall_lookup.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SYSCALL_LOOKUP must be defined before including this file.
 *
 * These first system calls are supported regardless of the NuttX
 * configuration
 */

SYSCALL_LOOKUP1(_exit,                    1, STUB__exit)
SYSCALL_LOOKUP(exit,                      1, STUB_exit)
SYSCALL_LOOKUP(get_errno,                 0, STUB_get_errno)
SYSCALL_LOOKUP(getpid,                    0, STUB_getpid)
SYSCALL_LOOKUP(sched_getparam,            2, STUB_sched_getparam)
SYSCALL_LOOKUP(sched_getscheduler,        1, STUB_sched_getscheduler)
SYSCALL_LOOKUP(sched_lock,                0, STUB_sched_lock)
SYSCALL_LOOKUP(sched_lockcount,           0, STUB_sched_lockcount)
SYSCALL_LOOKUP(sched_rr_get_interval,     2, STUB_sched_rr_get_interval)
SYSCALL_LOOKUP(sched_setparam,            2, STUB_sched_setparam)
SYSCALL_LOOKUP(sched_setscheduler,        3, STUB_sched_setscheduler)
SYSCALL_LOOKUP(sched_unlock,              0, STUB_sched_unlock)
SYSCALL_LOOKUP(sched_yield,               0, STUB_sched_yield)
SYSCALL_LOOKUP(sem_close,                 1, STUB_sem_close)
SYSCALL_LOOKUP(sem_destroy,               2, STUB_sem_destroy)
SYSCALL_LOOKUP(sem_open,                  6, STUB_sem_open)
SYSCALL_LOOKUP(sem_post,                  1, STUB_sem_post)
SYSCALL_LOOKUP(sem_trywait,               1, STUB_sem_trywait)
SYSCALL_LOOKUP(sem_unlink,                1, STUB_sem_unlink)
SYSCALL_LOOKUP(sem_wait,                  1, STUB_sem_wait)
SYSCALL_LOOKUP(set_errno,                 1, STUB_set_errno)
SYSCALL_LOOKUP(task_create,               5, STUB_task_create)
SYSCALL_LOOKUP(task_delete,               1, STUB_task_delete)
SYSCALL_LOOKUP(task_restart,              1, STUB_task_restart)
SYSCALL_LOOKUP(up_assert,                 2, STUB_up_assert)

/* The following can be individually enabled */

#ifdef CONFIG_ARCH_HAVE_VFORK
  SYSCALL_LOOKUP(vfork,                   0, SYS_vfork)
#endif

#ifdef CONFIG_SCHED_ATEXIT
  SYSCALL_LOOKUP(atexit,                  1, STUB_atexit)
#endif

#ifdef CONFIG_SCHED_ONEXIT
  SYSCALL_LOOKUP(on_exit,                 2, STUB_on_exit)
#endif

#ifdef CONFIG_SCHED_WAITPID
  SYSCALL_LOOKUP(waitpid,                 3, STUB_waitpid)
#  ifdef CONFIG_SCHED_HAVE_PARENT
  SYSCALL_LOOKUP(wait,                    1, STUB_wait)
  SYSCALL_LOOKUP(waitid,                  4, STUB_waitid)
#  endif
#endif

/* The following can only be defined if we are configured to execute
 * programs from a file system.
 */

#if defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_LIBC_EXECFUNCS)
#  ifdef CONFIG_BINFMT_EXEPATH
  SYSCALL_LOOKUP(posix_spawnp,            6, SYS_posixspawnp)
#  else
  SYSCALL_LOOKUP(posix_spawn,             6, SYS_posixspawn)
#  endif
  SYSCALL_LOOKUP(execv,                   2, SYS_execv)
  SYSCALL_LOOKUP(execl,                   6, SYS_execl)
#endif

/* The following are only defined is signals are supported in the NuttX
 * configuration.
 */

#ifndef CONFIG_DISABLE_SIGNALS
  SYSCALL_LOOKUP(kill,                    2, STUB_kill)
  SYSCALL_LOOKUP(sigaction,               3, STUB_sigaction)
  SYSCALL_LOOKUP(sigpending,              1, STUB_sigpending)
  SYSCALL_LOOKUP(sigprocmask,             3, STUB_sigprocmask)
  SYSCALL_LOOKUP(sigqueue,                3, STUB_sigqueue)
  SYSCALL_LOOKUP(sigsuspend,              1, STUB_sigsuspend)
  SYSCALL_LOOKUP(sigtimedwait,            3, STUB_sigtimedwait)
  SYSCALL_LOOKUP(sigwaitinfo,             2, STUB_sigwaitinfo)
  SYSCALL_LOOKUP(sleep,                   1, STUB_sleep)
  SYSCALL_LOOKUP(usleep,                  1, STUB_usleep)
#endif

/* The following are only defined if the system clock is enabled in the
 * NuttX configuration.
 */

#ifndef CONFIG_DISABLE_CLOCK
  SYSCALL_LOOKUP(syscall_clock_systimer,  0, STUB_clock_systimer)
  SYSCALL_LOOKUP(clock_getres,            2, STUB_clock_getres)
  SYSCALL_LOOKUP(clock_gettime,           2, STUB_clock_gettime)
  SYSCALL_LOOKUP(clock_settime,           2, STUB_clock_settime)
  SYSCALL_LOOKUP(gettimeofday,            2, STUB_gettimeofday)
#endif

/* The following are defined only if POSIX timers are supported */

#ifndef CONFIG_DISABLE_POSIX_TIMERS
  SYSCALL_LOOKUP(timer_create,            3, STUB_timer_create)
  SYSCALL_LOOKUP(timer_delete,            1, STUB_timer_delete)
  SYSCALL_LOOKUP(timer_getoverrun,        1, STUB_timer_getoverrun)
  SYSCALL_LOOKUP(timer_gettime,           2, STUB_timer_gettime)
  SYSCALL_LOOKUP(timer_settime,           4, STUB_timer_settime)
#endif

/* The following are defined if either file or socket descriptor are
 * enabled.
 */

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0 
  SYSCALL_LOOKUP(close,                   1, STUB_close)
  SYSCALL_LOOKUP(ioctl,                   3, STUB_ioctl)
  SYSCALL_LOOKUP(read,                    3, STUB_read)
  SYSCALL_LOOKUP(write,                   3, STUB_write)
#  ifndef CONFIG_DISABLE_POLL
  SYSCALL_LOOKUP(poll,                    3, STUB_poll)
  SYSCALL_LOOKUP(select,                  5, STUB_select)
#  endif
#endif

/* The following are defined if file descriptors are enabled */

#if CONFIG_NFILE_DESCRIPTORS > 0
  SYSCALL_LOOKUP(closedir,                1, STUB_closedir)
  SYSCALL_LOOKUP(dup,                     1, STUB_dup)
  SYSCALL_LOOKUP(dup2,                    2, STUB_dup2)
  SYSCALL_LOOKUP(fcntl,                   6, STUB_fcntl)
  SYSCALL_LOOKUP(lseek,                   3, STUB_lseek)
  SYSCALL_LOOKUP(mkfifo,                  2, STUB_mkfifo)
  SYSCALL_LOOKUP(mmap,                    6, STUB_mmap)
  SYSCALL_LOOKUP(open,                    6, STUB_open)
  SYSCALL_LOOKUP(opendir,                 1, STUB_opendir)
  SYSCALL_LOOKUP(pipe,                    1, STUB_pipe)
  SYSCALL_LOOKUP(readdir,                 1, STUB_readdir)
  SYSCALL_LOOKUP(rewinddir,               1, STUB_rewinddir)
  SYSCALL_LOOKUP(seekdir,                 2, STUB_seekdir)
  SYSCALL_LOOKUP(stat,                    2, STUB_stat)
  SYSCALL_LOOKUP(statfs,                  2, STUB_statfs)
  SYSCALL_LOOKUP(telldir,                 1, STUB_telldir)

#  if CONFIG_NFILE_STREAMS > 0
  SYSCALL_LOOKUP(fdopen,                  3, STUB_fs_fdopen)
  SYSCALL_LOOKUP(sched_getstreams,        0, STUB_sched_getstreams)
#endif

#  if !defined(CONFIG_DISABLE_MOUNTPOINT)
  SYSCALL_LOOKUP(fsync,                   1, STUB_fsync)
  SYSCALL_LOOKUP(mkdir,                   2, STUB_mkdir)
  SYSCALL_LOOKUP(mount,                   5, STUB_mount)
  SYSCALL_LOOKUP(rename,                  2, STUB_rename)
  SYSCALL_LOOKUP(rmdir,                   1, STUB_rmdir)
  SYSCALL_LOOKUP(umount,                  1, STUB_umount)
  SYSCALL_LOOKUP(unlink,                  1, STUB_unlink)
#  endif
#endif

/* The following are defined if pthreads are enabled */

#ifndef CONFIG_DISABLE_PTHREAD
  SYSCALL_LOOKUP(pthread_barrier_destroy, 1, STUB_pthread_barrier_destroy)
  SYSCALL_LOOKUP(pthread_barrier_init,    3, STUB_pthread_barrier_init)
  SYSCALL_LOOKUP(pthread_barrier_wait,    1, STUB_pthread_barrier_wait)
  SYSCALL_LOOKUP(pthread_cancel,          1, STUB_pthread_cancel)
  SYSCALL_LOOKUP(pthread_cond_broadcast,  1, STUB_pthread_cond_broadcast)
  SYSCALL_LOOKUP(pthread_cond_destroy,    1, STUB_pthread_cond_destroy)
  SYSCALL_LOOKUP(pthread_cond_init,       2, STUB_pthread_cond_init)
  SYSCALL_LOOKUP(pthread_cond_signal,     1, STUB_pthread_cond_signal)
  SYSCALL_LOOKUP(pthread_cond_wait,       2, STUB_pthread_cond_wait)
  SYSCALL_LOOKUP(pthread_create,          4, STUB_pthread_create)
  SYSCALL_LOOKUP(pthread_detach,          1, STUB_pthread_detach)
  SYSCALL_LOOKUP(pthread_exit,            1, STUB_pthread_exit)
  SYSCALL_LOOKUP(pthread_getschedparam,   3, STUB_pthread_getschedparam)
  SYSCALL_LOOKUP(pthread_getspecific,     1, STUB_pthread_getspecific)
  SYSCALL_LOOKUP(pthread_join,            2, STUB_pthread_join)
  SYSCALL_LOOKUP(pthread_key_create,      2, STUB_pthread_key_create)
  SYSCALL_LOOKUP(pthread_key_delete,      1, STUB_pthread_key_delete)
  SYSCALL_LOOKUP(pthread_mutex_destroy,   1, STUB_pthread_mutex_destroy)
  SYSCALL_LOOKUP(pthread_mutex_init,      2, STUB_pthread_mutex_init)
  SYSCALL_LOOKUP(pthread_mutex_lock,      1, STUB_pthread_mutex_lock)
  SYSCALL_LOOKUP(pthread_mutex_trylock,   1, STUB_pthread_mutex_trylock)
  SYSCALL_LOOKUP(pthread_mutex_unlock,    1, STUB_pthread_mutex_unlock)
  SYSCALL_LOOKUP(pthread_once,            2, STUB_pthread_once)
  SYSCALL_LOOKUP(pthread_setcancelstate,  2, STUB_pthread_setcancelstate)
  SYSCALL_LOOKUP(pthread_setschedparam,   3, STUB_pthread_setschedparam)
  SYSCALL_LOOKUP(pthread_setschedprio,    2, STUB_pthread_setschedprio)
  SYSCALL_LOOKUP(pthread_setspecific,     2, STUB_pthread_setspecific)
  SYSCALL_LOOKUP(pthread_yield,           0, STUB_pthread_yield)
#  ifndef CONFIG_DISABLE_SIGNAL
  SYSCALL_LOOKUP(pthread_cond_timedwait,  3, STUB_pthread_cond_timedwait)
  SYSCALL_LOOKUP(pthread_kill,            2, STUB_pthread_kill)
  SYSCALL_LOOKUP(pthread_sigmask,         3, STUB_pthread_sigmask)
#  endif
#endif

/* The following are defined only if message queues are enabled */

#ifndef CONFIG_DISABLE_MQUEUE
  SYSCALL_LOOKUP(mq_close,                1, STUB_mq_close)
  SYSCALL_LOOKUP(mq_notify,               2, STUB_mq_notify)
  SYSCALL_LOOKUP(mq_open,                 6, STUB_mq_open)
  SYSCALL_LOOKUP(mq_receive,              4, STUB_mq_receive)
  SYSCALL_LOOKUP(mq_send,                 4, STUB_mq_send)
  SYSCALL_LOOKUP(mq_timedreceive,         5, STUB_mq_timedreceive)
  SYSCALL_LOOKUP(mq_timedsend,            5, STUB_mq_timedsend)
  SYSCALL_LOOKUP(mq_unlink,               1, STUB_mq_unlink)
#endif

/* The following are defined only if environment variables are supported */

#ifndef CONFIG_DISABLE_ENVIRON
  SYSCALL_LOOKUP(clearenv,                0, STUB_clearenv)
  SYSCALL_LOOKUP(getenv,                  1, STUB_getenv)
  SYSCALL_LOOKUP(putenv,                  1, STUB_putenv)
  SYSCALL_LOOKUP(setenv,                  3, STUB_setenv)
  SYSCALL_LOOKUP(unsetenv,                1, STUB_unsetenv)
#endif

/* The following are defined only if networking AND sockets are supported */

#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  SYSCALL_LOOKUP(accept,                  3, STUB_accept)
  SYSCALL_LOOKUP(bind,                    3, STUB_bind)
  SYSCALL_LOOKUP(connect,                 3, STUB_connect)
  SYSCALL_LOOKUP(getsockopt,              5, STUB_getsockopt)
  SYSCALL_LOOKUP(listen,                  2, STUB_listen)
  SYSCALL_LOOKUP(recv,                    4, STUB_recv)
  SYSCALL_LOOKUP(recvfrom,                6, STUB_recvfrom)
  SYSCALL_LOOKUP(send,                    4, STUB_send)
  SYSCALL_LOOKUP(sendto,                  6, STUB_sendto)
  SYSCALL_LOOKUP(setsockopt,              5, STUB_setsockopt)
  SYSCALL_LOOKUP(socket,                  3, STUB_socket)
#endif

/* The following is defined only if CONFIG_TASK_NAME_SIZE > 0 */

#if CONFIG_TASK_NAME_SIZE > 0
  SYSCALL_LOOKUP(prctl,                   5, STUB_prctl)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/


