/****************************************************************************
 * include/nuttx/note/note_driver.h
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

#ifndef __INCLUDE_NUTTX_NOTE_NOTE_DRIVER_H
#define __INCLUDE_NUTTX_NOTE_NOTE_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>

#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct note_driver_s;

struct note_driver_ops_s
{
  CODE void (*add)(FAR struct note_driver_s *drv,
                   FAR const void *note, size_t notelen);
  CODE void (*start)(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb);
  CODE void (*stop)(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb);
#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
  CODE void (*suspend)(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb);
  CODE void (*resume)(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb);
#  ifdef CONFIG_SMP
  CODE void (*cpu_start)(FAR struct note_driver_s *drv,
                         FAR struct tcb_s *tcb, int cpu);
  CODE void (*cpu_started)(FAR struct note_driver_s *drv,
                           FAR struct tcb_s *tcb);
  CODE void (*cpu_pause)(FAR struct note_driver_s *drv,
                         FAR struct tcb_s *tcb, int cpu);
  CODE void (*cpu_paused)(FAR struct note_driver_s *drv,
                          FAR struct tcb_s *tcb);
  CODE void (*cpu_resume)(FAR struct note_driver_s *drv,
                          FAR struct tcb_s *tcb, int cpu);
  CODE void (*cpu_resumed)(FAR struct note_driver_s *drv,
                           FAR struct tcb_s *tcb);
#  endif
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
  CODE void (*premption)(FAR struct note_driver_s *drv,
                         FAR struct tcb_s *tcb, bool locked);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
  CODE void (*csection)(FAR struct note_driver_s *drv,
                        FAR struct tcb_s *tcb, bool enter);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  CODE void (*spinlock)(FAR struct note_driver_s *drv, FAR struct tcb_s *tcb,
                        FAR volatile void *spinlock, int type);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
  CODE void (*syscall_enter)(FAR struct note_driver_s *drv,
                             int nr, int argc, va_list *ap);
  CODE void (*syscall_leave)(FAR struct note_driver_s *drv,
                             int nr, uintptr_t result);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  CODE void (*irqhandler)(FAR struct note_driver_s *drv, int irq,
                          FAR void *handler, bool enter);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_DUMP
  CODE void (*string)(FAR struct note_driver_s *drv, uintptr_t ip,
                      FAR const char *buf);
  CODE void (*dump)(FAR struct note_driver_s *drv, uintptr_t ip,
                    uint8_t event, FAR const void *buf, size_t len);
  CODE void (*vprintf)(FAR struct note_driver_s *drv, uintptr_t ip,
                       FAR const char *fmt, va_list va) printf_like(3, 0);
  CODE void (*vbprintf)(FAR struct note_driver_s *drv, uintptr_t ip,
                        uint8_t event, FAR const char *fmt,
                        va_list va) printf_like(4, 0);
#endif
};

struct note_driver_s
{
  FAR const struct note_driver_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__KERNEL__) || defined(CONFIG_BUILD_FLAT)

/****************************************************************************
 * Name: note_initialize
 *
 * Description:
 *   Register sched note related drivers at /dev folder that can be used by
 *   an application to read or filter the note data.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DRIVER_NOTE
int note_initialize(void);
#endif

#endif /* defined(__KERNEL__) || defined(CONFIG_BUILD_FLAT) */

#if CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE > 0

/****************************************************************************
 * Name: note_get_taskname
 *
 * Description:
 *   Get the task name string of the specified PID
 *
 * Input Parameters:
 *   PID - Task ID
 *   name - Task name buffer
 *          this buffer must be greater than CONFIG_TASK_NAME_SIZE + 1
 *
 * Returned Value:
 *   Retrun OK if task name can be retrieved, otherwise -ESRCH
 *
 ****************************************************************************/

int note_get_taskname(pid_t pid, FAR char *name);

#endif /* CONFIG_DRIVER_NOTE_TASKNAME_BUFSIZE > 0 */

#endif /* __INCLUDE_NUTTX_NOTE_NOTE_DRIVER_H */
