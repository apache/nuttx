/****************************************************************************
 * sched/group/group.h
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
#include <nuttx/sched.h>

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
/* This variable holds the current task group.  This pointer is NULL
 * if the current task is a kernel thread that has no address environment
 * (other than the kernel context).
 *
 * This must only be accessed with interrupts disabled.
 */

extern FAR struct task_group_s *g_group_current[CONFIG_SMP_NCPUS];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SCHED_CHILD_STATUS
void weak_function task_initialize(void);
#endif

/* Task group data structure management */

int  group_allocate(FAR struct task_tcb_s *tcb, uint8_t ttype);
void group_initialize(FAR struct task_tcb_s *tcb);
#ifndef CONFIG_DISABLE_PTHREAD
int  group_bind(FAR struct pthread_tcb_s *tcb);
int  group_join(FAR struct pthread_tcb_s *tcb);
#endif
void group_leave(FAR struct tcb_s *tcb);
#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
void group_add_waiter(FAR struct task_group_s *group);
void group_del_waiter(FAR struct task_group_s *group);
#endif

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
FAR struct task_group_s *group_findbypid(pid_t pid);
#endif

#ifdef HAVE_GROUP_MEMBERS
int group_foreachchild(FAR struct task_group_s *group,
                       foreachchild_t handler, FAR void *arg);
int group_kill_children(FAR struct tcb_s *tcb);
#ifdef CONFIG_SIG_SIGSTOP_ACTION
int group_suspend_children(FAR struct tcb_s *tcb);
int group_continue(FAR struct tcb_s *tcb);
#endif
#endif

#ifdef CONFIG_ARCH_ADDRENV
/* Group address environment management */

int group_addrenv(FAR struct tcb_s *tcb);
#endif

/* Convenience functions */

FAR struct task_group_s *task_getgroup(pid_t pid);

/* Signaling group members */

int group_signal(FAR struct task_group_s *group, FAR siginfo_t *siginfo);

/* Parent/child data management */

#ifdef CONFIG_SCHED_HAVE_PARENT
int task_reparent(pid_t ppid, pid_t chpid);

#ifdef CONFIG_SCHED_CHILD_STATUS
FAR struct child_status_s *group_alloc_child(void);
void group_free_child(FAR struct child_status_s *status);
void group_add_child(FAR struct task_group_s *group,
                     FAR struct child_status_s *child);
FAR struct child_status_s *group_exit_child(FAR struct task_group_s *group);
FAR struct child_status_s *group_find_child(FAR struct task_group_s *group,
                                            pid_t pid);
FAR struct child_status_s *group_remove_child(FAR struct task_group_s *group,
                                              pid_t pid);
void group_remove_children(FAR struct task_group_s *group);

#endif /* CONFIG_SCHED_CHILD_STATUS */
#endif /* CONFIG_SCHED_HAVE_PARENT */

/* Group data resource configuration */

int  group_setupidlefiles(FAR struct task_tcb_s *tcb);
int  group_setuptaskfiles(FAR struct task_tcb_s *tcb);
#ifdef CONFIG_FILE_STREAM
int  group_setupstreams(FAR struct task_tcb_s *tcb);
#endif

#endif /* __SCHED_GROUP_GROUP_H */
