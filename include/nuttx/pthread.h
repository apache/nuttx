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

#if CONFIG_RR_INTERVAL == 0
#  define PTHREAD_DEFAULT_POLICY SCHED_FIFO
#else
#  define PTHREAD_DEFAULT_POLICY SCHED_RR
#endif

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
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Default pthread attributes.  This global can only be shared within the
 * kernel- or within the user- address space.
 */

EXTERN const pthread_attr_t g_default_pthread_attr;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_PTHREAD_H */
