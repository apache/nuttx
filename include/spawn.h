/****************************************************************************
 * include/spawn.h
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

#ifndef __INCLUDE_SPAWN_H
#define __INCLUDE_SPAWN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <sched.h>
#include <signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_TASK_SPAWN_DEFAULT_STACKSIZE
#  define CONFIG_TASK_SPAWN_DEFAULT_STACKSIZE 2048
#endif

/* "The spawn.h header shall define the flags that may be set in a
 * posix_spawnattr_t object using the posix_spawnattr_setflags() function:"
 */

#define POSIX_SPAWN_RESETIDS      (1 << 0)  /* 1: Reset effective user ID */
#define POSIX_SPAWN_SETPGROUP     (1 << 1)  /* 1: Set process group */
#define POSIX_SPAWN_SETSCHEDPARAM (1 << 2)  /* 1: Set task's priority */
#define POSIX_SPAWN_SETSCHEDULER  (1 << 3)  /* 1: Set task's scheduler policy */
#define POSIX_SPAWN_SETSIGDEF     (1 << 4)  /* 1: Set default signal actions */
#define POSIX_SPAWN_SETSIGMASK    (1 << 5)  /* 1: Set sigmask */
#define POSIX_SPAWN_SETSID        (1 << 7)  /* 1: Create the new session(glibc specific) */

/* NOTE: NuttX provides only one implementation:  If
 * CONFIG_LIBC_ENVPATH is defined, then only posix_spawnp() behavior
 * is supported; otherwise, only posix_spawn behavior is supported.
 */

#define posix_spawnp              posix_spawn

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* "The spawn.h header shall define the posix_spawnattr_t and
 * posix_spawn_file_actions_t types used in performing spawn operations.
 *
 * The internal structure underlying the posix_spawnattr_t is exposed here
 * because the user will be required to allocate this memory.
 */

struct timespec;
struct posix_spawnattr_s
{
  /* Used by posix_spawn, posix_spawnp, and task_spawn */

  uint8_t  flags;                /* See POSIX_SPAWN_ definitions */
  uint8_t  priority;             /* Task scheduling priority */
  uint8_t  policy;               /* Task scheduling policy */

#ifdef CONFIG_SCHED_SPORADIC
  uint8_t  low_priority;         /* Low scheduling priority */
  uint8_t  max_repl;             /* Maximum pending replenishments */
#endif

  sigset_t sigmask;              /* Signals to be masked */

#ifndef CONFIG_BUILD_KERNEL
  /* Used only by task_spawn (non-standard) */

  FAR void *stackaddr;           /* Task stack address */
  size_t    stacksize;           /* Task stack size */
#endif

#ifdef CONFIG_SCHED_SPORADIC
  struct timespec repl_period;   /* Replenishment period */
  struct timespec budget;        /* Initial budget */
#endif
};

typedef struct posix_spawnattr_s posix_spawnattr_t;

/* posix_spawn_file_actions_addclose(), posix_spawn_file_actions_adddup2(),
 * and posix_spawn_file_actions_addopen() will allocate memory and append
 * a new file action to an instance of posix_spawn_file_actions_t.  The
 * internal representation of these structures is not exposed to the user.
 * The user need only know that the size sizeof(posix_spawn_file_actions_t)
 * will hold a pointer to data.
 */

typedef FAR void *posix_spawn_file_actions_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* "The following shall be declared as functions and may also be defined as
 * macros. Function prototypes shall be provided."
 */

#ifdef __cplusplus
extern "C"
{
#endif

/* Spawn interfaces *********************************************************/

/* Standard posix_spawn[p] interfaces.  These functions execute files in the
 * file system at 'path'
 */

int posix_spawn(FAR pid_t *pid, FAR const char *path,
      FAR const posix_spawn_file_actions_t *file_actions,
      FAR const posix_spawnattr_t *attr,
      FAR char * const argv[], FAR char * const envp[]);

#ifndef CONFIG_BUILD_KERNEL
/* Non-standard task_spawn interface.  This function uses the same
 * semantics to execute a file in memory at 'entry', giving it the name
 * 'name'.
 */

int task_spawn(FAR const char *name, main_t entry,
      FAR const posix_spawn_file_actions_t *file_actions,
      FAR const posix_spawnattr_t *attr,
      FAR char * const argv[], FAR char * const envp[]);
#endif

/* File action interfaces ***************************************************/

/* File action initialization and destruction */

int posix_spawn_file_actions_init(
      FAR posix_spawn_file_actions_t *file_actions);
int posix_spawn_file_actions_destroy(
      FAR posix_spawn_file_actions_t *file_actions);

/* Add file action interfaces */

int posix_spawn_file_actions_addclose(
      FAR posix_spawn_file_actions_t *file_actions,
      int fd);
int posix_spawn_file_actions_adddup2(
      FAR posix_spawn_file_actions_t *file_actions,
      int fd1, int fd2);
int posix_spawn_file_actions_addopen(
      FAR posix_spawn_file_actions_t *file_actions,
      int fd, FAR const char *path, int oflags, mode_t mode);

/* Spawn attributes interfaces **********************************************/

/* Spawn attributes initialization and destruction */

int posix_spawnattr_init(FAR posix_spawnattr_t *attr);

/* int posix_spawnattr_destroy(FAR posix_spawnattr_t *); */
#define posix_spawnattr_destroy(attr) (0)

/* Get spawn attributes interfaces */

int posix_spawnattr_getflags(FAR const posix_spawnattr_t *attr,
                             FAR short *flags);
#define posix_spawnattr_getpgroup(attr,group) (ENOSYS)
int posix_spawnattr_getschedparam(FAR const posix_spawnattr_t *attr,
                                  FAR struct sched_param *param);
int posix_spawnattr_getschedpolicy(FAR const posix_spawnattr_t *attr,
                                   FAR int *policy);
#define posix_spawnattr_getsigdefault(attr,sigdefault) (ENOSYS)
int posix_spawnattr_getsigmask(FAR const posix_spawnattr_t *attr,
                               FAR sigset_t *sigmask);

/* Set spawn attributes interfaces */

int posix_spawnattr_setflags(FAR posix_spawnattr_t *attr, short flags);
#define posix_spawnattr_setpgroup(attr,group) (ENOSYS)
int posix_spawnattr_setschedparam(FAR posix_spawnattr_t *attr,
                                  FAR const struct sched_param *param);
int posix_spawnattr_setschedpolicy(FAR posix_spawnattr_t *attr, int policy);
#define posix_spawnattr_setsigdefault(attr,sigdefault) (ENOSYS)
int posix_spawnattr_setsigmask(FAR posix_spawnattr_t *attr,
                               FAR const sigset_t *sigmask);

/* Non-standard get/set spawn attributes interfaces for use only with
 * task_spawn()
 */

#ifndef CONFIG_BUILD_KERNEL
int task_spawnattr_getstackaddr(FAR const posix_spawnattr_t *attr,
                                FAR void **stackaddr);
int task_spawnattr_setstackaddr(FAR posix_spawnattr_t *attr,
                                FAR void *stackaddr);

int task_spawnattr_getstacksize(FAR const posix_spawnattr_t *attr,
                                FAR size_t *stacksize);
int task_spawnattr_setstacksize(FAR posix_spawnattr_t *attr,
                                size_t stacksize);
#else
#  define task_spawnattr_getstackaddr(fa, addr) (*(addr) = NULL, 0)
#  define task_spawnattr_setstackaddr(fa) (0)
#  define task_spawnattr_getstacksize(fa, size) (*(size) = 0, 0)
#  define task_spawnattr_setstacksize(fa) (0)
#endif

/* Non standard debug functions */

#ifdef CONFIG_DEBUG_FEATURES
void posix_spawn_file_actions_dump(
                          FAR posix_spawn_file_actions_t *file_actions);
void posix_spawnattr_dump(FAR posix_spawnattr_t *attr);
#else
#  define posix_spawn_file_actions_dump(fa)
#  define posix_spawnattr_dump(a)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SPAWN_H */
