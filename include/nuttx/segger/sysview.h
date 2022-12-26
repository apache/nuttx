/****************************************************************************
 * include/nuttx/segger/sysview.h
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

#ifndef __INCLUDE_NUTTX_SEGGER_SYSVIEW_H
#define __INCLUDE_NUTTX_SEGGER_SYSVIEW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SEGGER_SYSVIEW_PREFIX
#  define PREFIX(fun) sysview ## _ ## fun
#else
#  define PREFIX(fun) fun
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sysview_initialize
 *
 * Description:
 *   Initializes the SYSVIEW module.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SEGGER_SYSVIEW
int sysview_initialize(void);

#  ifdef CONFIG_SEGGER_SYSVIEW_PREFIX

void PREFIX(sched_note_start)(struct tcb_s *tcb);

void PREFIX(sched_note_stop)(struct tcb_s *tcb);

void PREFIX(sched_note_suspend)(struct tcb_s *tcb);

void PREFIX(sched_note_resume)(struct tcb_s *tcb);

#    ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void PREFIX(sched_note_irqhandler)(int irq, void *handler, bool enter);
#    endif

#    ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void PREFIX(sched_note_syscall_enter)(int nr);
void PREFIX(sched_note_syscall_leave)(int nr, uintptr_t result);
#    endif

#    ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
void PREFIX(sched_note_filter_mode)(struct note_filter_mode_s *oldm,
                            struct note_filter_mode_s *newm);
#    endif

#    ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void PREFIX(sched_note_filter_irq)(struct note_filter_irq_s *oldf,
                           struct note_filter_irq_s *newf);
#    endif

#    ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void PREFIX(sched_note_filter_syscall)(struct note_filter_syscall_s *oldf,
                               struct note_filter_syscall_s *newf);
#    endif

#  endif /* CONFIG_SEGGER_SYSVIEW_PREFIX */

#endif /* CONFIG_SEGGER_SYSVIEW */

#endif /* __INCLUDE_NUTTX_SEGGER_SYSVIEW_H */
