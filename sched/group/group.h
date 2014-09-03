/****************************************************************************
 * sched/group/group.h
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
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

#ifndef __SCHED_GROUP_GROUP_H
#define __SCHED_GROUP_GROUP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <queue.h>
#include <sched.h>

#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Any negative GID is invalid. */

#define INVALID_GROUP_ID    (pid_t)-1
#define IS_INVALID_GID(gid) ((int)(gid) < 0)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef int (*foreachchild_t)(pid_t pid, FAR void *arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
/* This is the head of a list of all group members */

extern FAR struct task_group_s *g_grouphead;
#endif

#ifdef CONFIG_ARCH_ADDRENV
/* This variable holds the group ID of the current task group.  This ID is
 * zero if the current task is a kernel thread that has no address
 * environment (other than the kernel context).
 *
 * This must only be accessed with interrupts disabled.
 */

extern gid_t g_gid_current;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SCHED_CHILD_STATUS
void weak_function task_initialize(void);
#endif

/* Task group data structure management */

#ifdef HAVE_TASK_GROUP
int  group_allocate(FAR struct task_tcb_s *tcb, uint8_t ttype);
int  group_initialize(FAR struct task_tcb_s *tcb);
#ifndef CONFIG_DISABLE_PTHREAD
int  group_bind(FAR struct pthread_tcb_s *tcb);
int  group_join(FAR struct pthread_tcb_s *tcb);
#endif
void group_leave(FAR struct tcb_s *tcb);

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
FAR struct task_group_s *group_findbygid(gid_t gid);
#endif

#ifdef HAVE_GROUP_MEMBERS
FAR struct task_group_s *group_findbypid(pid_t pid);
int group_foreachchild(FAR struct task_group_s *group,
                       foreachchild_t handler, FAR void *arg);
int group_killchildren(FAR struct task_tcb_s *tcb);
#endif

#ifdef CONFIG_ARCH_ADDRENV
/* Group address environment management */

int group_addrenv(FAR struct tcb_s *tcb);
#endif

/* Convenience functions */

FAR struct task_group_s *task_getgroup(pid_t pid);

/* Signaling group members */

#ifndef CONFIG_DISABLE_SIGNALS
int  group_signal(FAR struct task_group_s *group, FAR siginfo_t *siginfo);
#endif
#endif /* HAVE_TASK_GROUP */

/* Parent/child data management */

#ifdef CONFIG_SCHED_HAVE_PARENT
int task_reparent(pid_t ppid, pid_t chpid);

#ifdef CONFIG_SCHED_CHILD_STATUS
FAR struct child_status_s *group_allocchild(void);
void group_freechild(FAR struct child_status_s *status);
void group_addchild(FAR struct task_group_s *group,
                    FAR struct child_status_s *child);
FAR struct child_status_s *group_exitchild(FAR struct task_group_s *group);
FAR struct child_status_s *group_findchild(FAR struct task_group_s *group,
                                           pid_t pid);
FAR struct child_status_s *group_removechild(FAR struct task_group_s *group,
                                             pid_t pid);
void group_removechildren(FAR struct task_group_s *group);

#endif /* CONFIG_SCHED_CHILD_STATUS */
#endif /* CONFIG_SCHED_HAVE_PARENT */

/* Group data resource configuration */

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0
int  group_setupidlefiles(FAR struct task_tcb_s *tcb);
int  group_setuptaskfiles(FAR struct task_tcb_s *tcb);
#if CONFIG_NFILE_STREAMS > 0
int  group_setupstreams(FAR struct task_tcb_s *tcb);
#endif
#endif

#endif /* __SCHED_GROUP_GROUP_H */
