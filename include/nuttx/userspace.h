/****************************************************************************
 * include/nuttx/userspace.h
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

#ifndef __INCLUDE_NUTTX_USERSPACE_H
#define __INCLUDE_NUTTX_USERSPACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <signal.h>
#include <pthread.h>

#include <nuttx/arch.h>

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If CONFIG_BUILD_PROTECTED, then CONFIG_NUTTX_USERSPACE must be defined to
 * provide the address where the user-space header can be found in memory.
 */

#ifndef CONFIG_NUTTX_USERSPACE
#  error "CONFIG_NUTTX_USERSPACE is not defined"
#endif

/* Let's insist on 4-byte alignment.  This alignment may not be required
 * technically for all platforms.  However, neither is it an unreasonable
 * requirement for any platform.
 */

#if (CONFIG_NUTTX_USERSPACE & 3) != 0
#  warning "CONFIG_NUTTX_USERSPACE is not aligned to a 4-byte boundary"
#endif

/* Helper Macros ************************************************************/

/* This macro is used to access the struct userpace_s header that can be
 * found at the beginning of the user-space blob.
 */

#define USERSPACE ((FAR struct userspace_s *)CONFIG_NUTTX_USERSPACE)

/* In user space, these functions are directly callable.  In kernel space,
 * they can be called through the userspace structure.
 */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct mm_heaps_s; /* Forward reference */

/* Every user-space blob starts with a header that provides information about
 * the blob.  The form of that header is provided by struct userspace_s. An
 * instance of this is expected to reside at CONFIG_NUTTX_USERSPACE.
 */

struct userspace_s
{
  /* General memory map */

  main_t    us_entrypoint;
  uintptr_t us_textstart;
  uintptr_t us_textend;
  uintptr_t us_datasource;
  uintptr_t us_datastart;
  uintptr_t us_dataend;
  uintptr_t us_bssstart;
  uintptr_t us_bssend;
  uintptr_t us_heapend;

  /* Memory manager heap structure */

  FAR struct mm_heap_s **us_heap;

  /* Task startup routine */

  CODE void (*task_startup)(main_t entrypt, int argc, FAR char *argv[]);

  /* Signal handler trampoline */

  CODE void (*signal_handler)(_sa_sigaction_t sighand, int signo,
    FAR siginfo_t *info, FAR void *ucontext);

  /* User-space work queue support */

#ifdef CONFIG_LIB_USRWORK
  CODE int (*work_usrstart)(void);
#endif
};

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_BUILD_PROTECTED */
#endif /* __INCLUDE_NUTTX_USERSPACE_H */
