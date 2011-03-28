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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYS_accept                    (0)
#define SYS_bind                      (1)
#define SYS_chdir                     (2)
#define SYS_clock_getres              (3)
#define SYS_clock_gettime             (4)
#define SYS_clock_settime             (5)
#define SYS_close                     (6)
#define SYS_connect                   (7)
#define SYS_creat                     (8)
#define SYS_dup                       (9)
#define SYS_dup2                      (10)
#define SYS_exit                      (11)
#define SYS_fcntl                     (12)
#define SYS_fstat                     (13)
#define SYS_fstatfs                   (14)
#define SYS_fsync                     (15)
#define SYS_getcwd                    (16)
#define SYS_getpid                    (17)
#define SYS_getsockopt                (18)
#define SYS_gettimeofday              (19)
#define SYS_ioctl                     (20)
#define SYS_kill                      (21)
#define SYS_listen                    (22)
#define SYS_lseek                     (23)
#define SYS_mkdir                     (24)
#define SYS_mmap                      (25)
#define SYS_mount                     (26)
#define SYS_mq_notify                 (27)
#define SYS_mq_open                   (28)
#define SYS_mq_timedreceive           (29)
#define SYS_mq_timedsend              (30)
#define SYS_mq_unlink                 (31)
#define SYS_munmap                    (32)
#define SYS_open                      (33)
#define SYS_pipe                      (34)
#define SYS_poll                      (35)
#define SYS_read                      (36)
#define SYS_readdir                   (37)
#define SYS_reboot                    (38)
#define SYS_recvfrom                  (39)
#define SYS_rename                    (40)
#define SYS_rmdir                     (41)
#define SYS_sched_getparam            (42)
#define SYS_sched_get_priority_max    (43)
#define SYS_sched_get_priority_min    (44)
#define SYS_sched_getscheduler        (45)
#define SYS_sched_rr_get_interval     (46)
#define SYS_sched_setparam            (47)
#define SYS_sched_setscheduler        (48)
#define SYS_sched_yield               (49)
#define SYS_select                    (50)
#define SYS_sendto                    (51)
#define SYS_setsockopt                (52)
#define SYS_sigaction                 (53)
#define SYS_signal                    (54)
#define SYS_sigpending                (55)
#define SYS_sigprocmask               (56)
#define SYS_sigsuspend                (57)
#define SYS_socket                    (58)
#define SYS_stat                      (59)
#define SYS_statfs                    (60)
#define SYS_task_create               (61)
#define SYS_task_delete               (62)
#define SYS_task_init                 (63)
#define SYS_task_restart              (64)
#define SYS_timer_create              (65)
#define SYS_timer_delete              (66)
#define SYS_timer_getoverrun          (67)
#define SYS_timer_gettime             (68)
#define SYS_timer_settime             (69)
#define SYS_umount                    (70)
#define SYS_unlink                    (71)
#define SYS_waitid                    (72)
#define SYS_waitpid                   (73)
#define SYS_write                     (74)

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

