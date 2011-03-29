/****************************************************************************
 * include/sys/syscall.h
 * This file contains the system call numbers.
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_SYSCALL_H
#define __INCLUDE_SYS_SYSCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Reserve the first system calls for platform-specific usage */

#ifndef CONFIG_CONFIG_SYS_RESERVED
#  define CONFIG_SYS_RESERVED         (32)
#endif

/* System call numbers */

#define SYS_accept                    (CONFIG_SYS_RESERVED+0)
#define SYS_bind                      (CONFIG_SYS_RESERVED+1)
#define SYS_chdir                     (CONFIG_SYS_RESERVED+2)
#define SYS_clock_getres              (CONFIG_SYS_RESERVED+3)
#define SYS_clock_gettime             (CONFIG_SYS_RESERVED+4)
#define SYS_clock_settime             (CONFIG_SYS_RESERVED+5)
#define SYS_close                     (CONFIG_SYS_RESERVED+6)
#define SYS_connect                   (CONFIG_SYS_RESERVED+7)
#define SYS_creat                     (CONFIG_SYS_RESERVED+8)
#define SYS_dup                       (CONFIG_SYS_RESERVED+9)
#define SYS_dup2                      (CONFIG_SYS_RESERVED+10)
#define SYS_exit                      (CONFIG_SYS_RESERVED+11)
#define SYS_fcntl                     (CONFIG_SYS_RESERVED+12)
#define SYS_fstat                     (CONFIG_SYS_RESERVED+13)
#define SYS_fstatfs                   (CONFIG_SYS_RESERVED+14)
#define SYS_fsync                     (CONFIG_SYS_RESERVED+15)
#define SYS_getcwd                    (CONFIG_SYS_RESERVED+16)
#define SYS_getpid                    (CONFIG_SYS_RESERVED+17)
#define SYS_getsockopt                (CONFIG_SYS_RESERVED+18)
#define SYS_gettimeofday              (CONFIG_SYS_RESERVED+19)
#define SYS_ioctl                     (CONFIG_SYS_RESERVED+20)
#define SYS_kill                      (CONFIG_SYS_RESERVED+21)
#define SYS_listen                    (CONFIG_SYS_RESERVED+22)
#define SYS_lseek                     (CONFIG_SYS_RESERVED+23)
#define SYS_mkdir                     (CONFIG_SYS_RESERVED+24)
#define SYS_mmap                      (CONFIG_SYS_RESERVED+25)
#define SYS_mount                     (CONFIG_SYS_RESERVED+26)
#define SYS_mq_notify                 (CONFIG_SYS_RESERVED+27)
#define SYS_mq_open                   (CONFIG_SYS_RESERVED+28)
#define SYS_mq_timedreceive           (CONFIG_SYS_RESERVED+29)
#define SYS_mq_timedsend              (CONFIG_SYS_RESERVED+30)
#define SYS_mq_unlink                 (CONFIG_SYS_RESERVED+31)
#define SYS_munmap                    (CONFIG_SYS_RESERVED+32)
#define SYS_open                      (CONFIG_SYS_RESERVED+33)
#define SYS_pipe                      (CONFIG_SYS_RESERVED+34)
#define SYS_poll                      (CONFIG_SYS_RESERVED+35)
#define SYS_read                      (CONFIG_SYS_RESERVED+36)
#define SYS_readdir                   (CONFIG_SYS_RESERVED+37)
#define SYS_reboot                    (CONFIG_SYS_RESERVED+38)
#define SYS_recvfrom                  (CONFIG_SYS_RESERVED+39)
#define SYS_rename                    (CONFIG_SYS_RESERVED+40)
#define SYS_rmdir                     (CONFIG_SYS_RESERVED+41)
#define SYS_sched_getparam            (CONFIG_SYS_RESERVED+42)
#define SYS_sched_get_priority_max    (CONFIG_SYS_RESERVED+43)
#define SYS_sched_get_priority_min    (CONFIG_SYS_RESERVED+44)
#define SYS_sched_getscheduler        (CONFIG_SYS_RESERVED+45)
#define SYS_sched_rr_get_interval     (CONFIG_SYS_RESERVED+46)
#define SYS_sched_setparam            (CONFIG_SYS_RESERVED+47)
#define SYS_sched_setscheduler        (CONFIG_SYS_RESERVED+48)
#define SYS_sched_yield               (CONFIG_SYS_RESERVED+49)
#define SYS_select                    (CONFIG_SYS_RESERVED+50)
#define SYS_sendto                    (CONFIG_SYS_RESERVED+51)
#define SYS_setsockopt                (CONFIG_SYS_RESERVED+52)
#define SYS_sigaction                 (CONFIG_SYS_RESERVED+53)
#define SYS_signal                    (CONFIG_SYS_RESERVED+54)
#define SYS_sigpending                (CONFIG_SYS_RESERVED+55)
#define SYS_sigprocmask               (CONFIG_SYS_RESERVED+56)
#define SYS_sigsuspend                (CONFIG_SYS_RESERVED+57)
#define SYS_socket                    (CONFIG_SYS_RESERVED+58)
#define SYS_stat                      (CONFIG_SYS_RESERVED+59)
#define SYS_statfs                    (CONFIG_SYS_RESERVED+60)
#define SYS_task_create               (CONFIG_SYS_RESERVED+61)
#define SYS_task_delete               (CONFIG_SYS_RESERVED+62)
#define SYS_task_init                 (CONFIG_SYS_RESERVED+63)
#define SYS_task_restart              (CONFIG_SYS_RESERVED+64)
#define SYS_timer_create              (CONFIG_SYS_RESERVED+65)
#define SYS_timer_delete              (CONFIG_SYS_RESERVED+66)
#define SYS_timer_getoverrun          (CONFIG_SYS_RESERVED+67)
#define SYS_timer_gettime             (CONFIG_SYS_RESERVED+68)
#define SYS_timer_settime             (CONFIG_SYS_RESERVED+69)
#define SYS_umount                    (CONFIG_SYS_RESERVED+70)
#define SYS_unlink                    (CONFIG_SYS_RESERVED+71)
#define SYS_waitid                    (CONFIG_SYS_RESERVED+72)
#define SYS_waitpid                   (CONFIG_SYS_RESERVED+73)
#define SYS_write                     (CONFIG_SYS_RESERVED+74)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SYS_SYSCALL_H */

