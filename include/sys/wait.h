/****************************************************************************
 * include/sys/wait.h
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

#ifndef __INCLUDE_SYS_WAIT_H
#define __INCLUDE_SYS_WAIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <signal.h>

#ifdef CONFIG_SCHED_WAITPID

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following are provided for analysis of returned status values.
 * Encoded is as follows as 2 bytes of _info(MS) then two bytes of code (LS).
 * Code:
 *   0 - Child has exited, info is the exit code.
 *   Other values - Not implemented
 */

#define WEXITSTATUS(s)  (((s) >> 8) & 0xff) /* Return exit status */
#define WIFEXITED(s)    (((s) & 0xff) == 0) /* True: Child exited normally */

#define WIFCONTINUED(s) (false)  /* True: Child has been continued */
#define WIFSIGNALED(s)  (false)  /* True: Child exited due to uncaught signal */
#define WIFSTOPPED(s)   (false)  /* True: Child is currently stopped */
#define WSTOPSIG(s)     (false)  /* Return signal number that caused process to stop */
#define WTERMSIG(s)     (false)  /* Return signal number that caused process to terminate */

/* The following symbolic constants are possible values for the options
 * argument to waitpid() (1) and/or waitid() (2),
 */

#define WCONTINUED      (1 << 0) /* Status for child that has been continued (1)(2) */
#define WNOHANG         (1 << 1) /* Do not wait if status not available (1) (2) */
#define WUNTRACED       (1 << 2) /* Report status of stopped child process (1) */
#define WEXITED         (1 << 3) /* Wait for processes that have exited (2) */
#define WSTOPPED        (1 << 4) /* Status for child stopped on signal (2) */
#define WNOWAIT         (1 << 5) /* Keep the process in a waitable state (2) */
#define WCLAIMED        (1 << 7) /* Non-standard (For internal OS use only) */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

enum idtype_e
{
  P_PID = 1,
  P_PGID = 2,
  P_ALL = 3
};
typedef enum idtype_e idtype_t;

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

pid_t wait(FAR int *stat_loc);
int   waitid(idtype_t idtype, id_t id, FAR siginfo_t *info, int options);
pid_t waitpid(pid_t pid, FAR int *stat_loc, int options);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SCHED_WAITPID */
#endif /* __INCLUDE_SYS_WAIT_H */
