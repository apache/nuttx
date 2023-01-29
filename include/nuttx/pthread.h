/****************************************************************************
 * include/nuttx/pthread.h
 * Non-standard, NuttX-specific pthread-related declarations.
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

#ifndef __INCLUDE_NUTTX_PTHREAD_H
#define __INCLUDE_NUTTX_PTHREAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <pthread.h>
#include <sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default pthread attribute initializer */

#define PTHREAD_DEFAULT_POLICY SCHED_NORMAL

/* A lot of hassle to use the old-fashioned struct initializers.  But this
 * gives us backward compatibility with some very old compilers.
 */

#if defined(CONFIG_SCHED_SPORADIC) && defined(CONFIG_SMP)
#  define PTHREAD_ATTR_INITIALIZER \
  { \
    PTHREAD_DEFAULT_PRIORITY, /* priority */ \
    PTHREAD_DEFAULT_POLICY,   /* policy */ \
    PTHREAD_EXPLICIT_SCHED,   /* inheritsched */ \
    PTHREAD_CREATE_JOINABLE,  /* detachstate */ \
    0,                        /* low_priority */ \
    0,                        /* max_repl */ \
    0,                        /* affinity */ \
    NULL,                     /* stackaddr */ \
    PTHREAD_STACK_DEFAULT,    /* stacksize */ \
    {0, 0},                   /* repl_period */ \
    {0, 0}                    /* budget */ \
  }
#elif defined(CONFIG_SCHED_SPORADIC)
#  define PTHREAD_ATTR_INITIALIZER \
  { \
    PTHREAD_DEFAULT_PRIORITY, /* priority */ \
    PTHREAD_DEFAULT_POLICY,   /* policy */ \
    PTHREAD_EXPLICIT_SCHED,   /* inheritsched */ \
    PTHREAD_CREATE_JOINABLE,  /* detachstate */ \
    0,                        /* low_priority */ \
    0,                        /* max_repl */ \
    NULL,                     /* stackaddr */ \
    PTHREAD_STACK_DEFAULT,    /* stacksize */ \
    {0, 0},                   /* repl_period */ \
    {0, 0},                   /* budget */ \
  }
#elif defined(CONFIG_SMP)
#  define PTHREAD_ATTR_INITIALIZER \
  { \
    PTHREAD_DEFAULT_PRIORITY, /* priority */ \
    PTHREAD_DEFAULT_POLICY,   /* policy */ \
    PTHREAD_EXPLICIT_SCHED,   /* inheritsched */ \
    PTHREAD_CREATE_JOINABLE,  /* detachstate */ \
    0,                        /* affinity */ \
    NULL,                     /* stackaddr */ \
    PTHREAD_STACK_DEFAULT,    /* stacksize */ \
  }
#else
#  define PTHREAD_ATTR_INITIALIZER \
  { \
    PTHREAD_DEFAULT_PRIORITY, /* priority */ \
    PTHREAD_DEFAULT_POLICY,   /* policy */ \
    PTHREAD_EXPLICIT_SCHED,   /* inheritsched */ \
    PTHREAD_CREATE_JOINABLE,  /* detachstate */ \
    NULL,                     /* stackaddr */ \
    PTHREAD_STACK_DEFAULT,    /* stacksize */ \
  }
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Default pthread attributes.  This global can only be shared within the
 * kernel- or within the user- address space.
 */

EXTERN const pthread_attr_t g_default_pthread_attr;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  nx_pthread_create
 *
 * Description:
 *   This function creates and activates a new thread with specified
 *   attributes.
 *
 * Input Parameters:
 *    trampoline - The user space startup function
 *    thread     - The pthread handle to be used
 *    attr       - It points to a pthread_attr_t structure whose contents are
 *                 used at thread creation time to determine attributes
 *                 for the new thread
 *    entry      - The new thread starts execution by invoking entry
 *    arg        - It is passed as the sole argument of entry
 *
 * Returned Value:
 *   OK (0) on success; a (non-negated) errno value on failure. The errno
 *   variable is not set.
 *
 ****************************************************************************/

int nx_pthread_create(pthread_trampoline_t trampoline, FAR pthread_t *thread,
                      FAR const pthread_attr_t *attr,
                      pthread_startroutine_t entry, pthread_addr_t arg);

/****************************************************************************
 * Name: nx_pthread_exit
 *
 * Description:
 *   Terminate execution of a thread started with pthread_create.
 *
 * Input Parameters:
 *   exit_value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nx_pthread_exit(FAR void *exit_value) noreturn_function;

/****************************************************************************
 * Name: pthread_cleanup_popall
 *
 * Description:
 *   The pthread_cleanup_popall() is an internal function that will pop and
 *   execute all clean-up functions.  This function is only called from
 *   within the pthread_exit() and pthread_cancellation() logic
 *
 * Input Parameters:
 *   tls - The local storage info of the exiting thread
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PTHREAD_CLEANUP
struct tls_info_s;
void pthread_cleanup_popall(FAR struct tls_info_s *tls);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_PTHREAD_H */
