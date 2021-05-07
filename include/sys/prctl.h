/****************************************************************************
 * include/sys/prctl.h
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

#ifndef __INCLUDE_SYS_PRCTL_H
#define __INCLUDE_SYS_PRCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Supported prctl() commands.
 *
 *  PR_SET_NAME
 *    Set the name of the calling thread, using the value in the location
 *    pointed to by (char *) arg2. The name can be up to
 *    CONFIG_TASK_NAME_SIZE long, including the terminating null byte.
 *    (If the length of the string, including the terminating null byte,
 *    exceeds CONFIG_TASK_NAME_SIZE bytes, the string is silently truncated.)
 *    As an example:
 *
 *      prctl(PR_SET_NAME, "MyName");
 *
 *  PR_GET_NAME
 *    Return the name of the calling thread, in the buffer pointed to by
 *    (char *) arg2.  The buffer should allow space for up to
 *    CONFIG_TASK_NAME_SIZE bytes; the returned string will be
 *    null-terminated. As an example:
 *
 *      char myname[CONFIG_TASK_NAME_SIZE];
 *      prctl(PR_GET_NAME, myname);
 *
 *  PR_SET_NAME_EXT
 *    Set the task (or thread) name for the thread whose ID is in required
 *    arg2 (int), using the value in the location pointed to by required arg1
 *    (char*).  The name can be up to CONFIG_TASK_NAME_SIZE long (including
 *    any null termination).  The thread ID of 0 will set the name of the
 *    calling thread. As an example:
 *
 *      prctl(PR_SET_NAME_EXT, "MyName", pid);
 *
 *  PR_GET_NAME_EXT
 *    Return the task (or thread) name for the for the thread whose ID is
 *    optional arg2 (int), in the buffer pointed to by optional arg1
 *    (char *). The buffer must be CONFIG_TASK_NAME_SIZE long (including
 *    any null termination). As an example:
 *
 *      char myname[CONFIG_TASK_NAME_SIZE];
 *      prctl(PR_GET_NAME_EXT, myname, pid);
 */

#define PR_SET_NAME     1
#define PR_GET_NAME     2
#define PR_SET_NAME_EXT 3
#define PR_GET_NAME_EXT 4

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: prctl
 *
 * Description:
 *   prctl() is called with a first argument describing what to do (with
 *   values PR_* defined above) and with additional arguments depending on
 *   the specific command.
 *
 * Returned Value:
 *   The returned value may depend on the specific command.  For PR_SET_NAME
 *   and PR_GET_NAME, the returned value of 0 indicates successful operation.
 *   On any failure, -1 is retruend and the errno value is set appropriately.
 *
 *     EINVAL The value of 'option' is not recognized.
 *     EFAULT optional arg1 is not a valid address.
 *     ESRCH  No task/thread can be found corresponding to that specified
 *       by optional arg1.
 *
 ****************************************************************************/

int prctl(int option, ...);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_PRCTL_H */
