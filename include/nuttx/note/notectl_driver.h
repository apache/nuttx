/****************************************************************************
 * include/nuttx/note/notectl_driver.h
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

#ifndef __INCLUDE_NUTTX_NOTE_NOTECTL_DRIVER_H
#define __INCLUDE_NUTTX_NOTE_NOTECTL_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/sched_note.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_SCHED_INSTRUMENTATION

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* NOTECTL_GETMODE
 *              - Get note filter mode
 *                Argument: A writable pointer to struct note_filter_mode_s
 * NOTECTL_SETMODE
 *              - Set note filter mode
 *                Argument: A read-only pointer to struct note_filter_mode_s
 * NOTECTL_GETSYSCALLFILTER
 *              - Get syscall filter setting
 *                Argument: A writable pointer to struct
 *                          note_filter_syscall_s
 * NOTECTL_SETSYSCALLFILTER
 *              - Set syscall filter setting
 *                Argument: A read-only pointer to struct
 *                          note_filter_syscall_s
 * NOTECTL_GETIRQFILTER
 *              - Get IRQ filter setting
 *                Argument: A writable pointer to struct
 *                          note_filter_irq_s
 * NOTECTL_SETIRQFILTER
 *              - Set IRQ filter setting
 *                Argument: A read-only pointer to struct
 *                          note_filter_irq_s
 */

#ifdef CONFIG_DRIVERS_NOTECTL

#define NOTECTL_GETMODE             _NOTECTLIOC(0x01)
#define NOTECTL_SETMODE             _NOTECTLIOC(0x02)
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
#define NOTECTL_GETSYSCALLFILTER    _NOTECTLIOC(0x03)
#define NOTECTL_SETSYSCALLFILTER    _NOTECTLIOC(0x04)
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
#define NOTECTL_GETIRQFILTER        _NOTECTLIOC(0x05)
#define NOTECTL_SETIRQFILTER        _NOTECTLIOC(0x06)
#endif

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__KERNEL__) || defined(CONFIG_BUILD_FLAT)

/****************************************************************************
 * Name: notectl_register
 *
 * Description:
 *   Register a driver at /dev/notectl that can be used by an application to
 *   control the note filter.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_NOTECTL
int notectl_register(void);
#endif

#endif /* defined(__KERNEL__) || defined(CONFIG_BUILD_FLAT) */

#endif /* CONFIG_SCHED_INSTRUMENTATION */

#endif /* __INCLUDE_NUTTX_NOTE_NOTECTL_DRIVER_H */
