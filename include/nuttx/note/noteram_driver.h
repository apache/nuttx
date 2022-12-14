/****************************************************************************
 * include/nuttx/note/noteram_driver.h
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

#ifndef __INCLUDE_NUTTX_NOTE_NOTERAM_DRIVER_H
#define __INCLUDE_NUTTX_NOTE_NOTERAM_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* NOTERAM_CLEAR
 *              - Clear all contents of the circular buffer
 *                Argument: Ignored
 * NOTERAM_GETMODE
 *              - Get overwrite mode
 *                Argument: A writable pointer to unsigned int
 * NOTERAM_SETMODE
 *              - Set overwrite mode
 *                Argument: A read-only pointer to unsigned int
 * NOTERAM_GETTASKNAME
 *              - Get task name string
 *                Argument: A writable pointer to struct
 *                          noteram_get_taskname_s
 *                Result:   If -ESRCH, the corresponding task name doesn't
 *                          exist.
 */

#ifdef CONFIG_DRIVER_NOTERAM
#define NOTERAM_CLEAR           _NOTERAMIOC(0x01)
#define NOTERAM_GETMODE         _NOTERAMIOC(0x02)
#define NOTERAM_SETMODE         _NOTERAMIOC(0x03)
#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
#define NOTERAM_GETTASKNAME     _NOTERAMIOC(0x04)
#endif
#endif

/* Overwrite mode definitions */

#ifdef CONFIG_DRIVER_NOTERAM
#define NOTERAM_MODE_OVERWRITE_DISABLE      0
#define NOTERAM_MODE_OVERWRITE_ENABLE       1
#define NOTERAM_MODE_OVERWRITE_OVERFLOW     2
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the type of the argument passed to the NOTERAM_GETTASKNAME ioctl */

#if CONFIG_DRIVER_NOTERAM_TASKNAME_BUFSIZE > 0
struct noteram_get_taskname_s
{
  pid_t pid;
  char taskname[CONFIG_TASK_NAME_SIZE + 1];
};
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__KERNEL__) || defined(CONFIG_BUILD_FLAT)

/****************************************************************************
 * Name: noteram_register
 *
 * Description:
 *   Register RAM note driver at /dev/note/ram that can be used by an
 *   application to read note data from the circular note buffer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on succress. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DRIVER_NOTERAM
int noteram_register(void);
#endif

#endif /* defined(__KERNEL__) || defined(CONFIG_BUILD_FLAT) */

#endif /* __INCLUDE_NUTTX_NOTE_NOTERAM_DRIVER_H */
