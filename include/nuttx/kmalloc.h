/****************************************************************************
 * include/nuttx/kmalloc.h
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

#ifndef __INCLUDE_NUTTX_KMALLOC_H
#define __INCLUDE_NUTTX_KMALLOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <malloc.h>
#include <stdbool.h>
#include <stdlib.h>

#include <nuttx/mm/mm.h>
#include <nuttx/userspace.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef KMALLOC_EXTERN
#if defined(__cplusplus)
#  define KMALLOC_EXTERN extern "C"
extern "C"
{
#else
#  define KMALLOC_EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* For a monolithic, kernel-mode NuttX build.  Special allocators must be
 * used.  Otherwise, the standard allocators prototyped in stdlib.h may
 * be used for both the kernel- and user-mode objects.
 */

/* This family of allocators is used to manage user-accessible memory
 * from the kernel.  In the flat build, the following are declared in
 * stdlib.h and are directly callable.  In the kernel-phase of the kernel
 * build, the following are defined in userspace.h as macros that call
 * into user-space via a header at the beginning of the user-space blob.
 */

#define kumm_initialize(h,s)     umm_initialize(h,s)
#define kumm_addregion(h,s)      umm_addregion(h,s)

#define kumm_calloc(n,s)         calloc(n,s);
#define kumm_malloc(s)           malloc(s)
#define kumm_malloc_size(p)      malloc_size(p)
#define kumm_zalloc(s)           zalloc(s)
#define kumm_realloc(p,s)        realloc(p,s)
#define kumm_memalign(a,s)       memalign(a,s)
#define kumm_free(p)             free(p)
#define kumm_mallinfo()          mallinfo()

/* This family of allocators is used to manage kernel protected memory */

#ifndef CONFIG_MM_KERNEL_HEAP
/* If this the kernel phase of a kernel build, and there are only user-space
 * allocators, then the following are defined in userspace.h as macros that
 * call into user-space via a header at the beginning of the user-space blob.
 */

#  define kmm_initialize(h,s)    /* Initialization done by kumm_initialize */
#  define kmm_addregion(h,s)     umm_addregion(h,s)

#  define kmm_calloc(n,s)        calloc(n,s);
#  define kmm_malloc(s)          malloc(s)
#  define kmm_malloc_size(p)     malloc_size(p)
#  define kmm_zalloc(s)          zalloc(s)
#  define kmm_realloc(p,s)       realloc(p,s)
#  define kmm_memalign(a,s)      memalign(a,s)
#  define kmm_free(p)            free(p)
#  define kmm_mallinfo()         mallinfo()

#else
/* Otherwise, the kernel-space allocators are declared in
 * include/nuttx/mm/mm.h and we can call them directly.
 */

#endif

#ifdef CONFIG_MM_KERNEL_HEAP
/****************************************************************************
 * Group memory management
 *
 *   Manage memory allocations appropriately for the group type.  If the
 *   memory is part of a privileged group, then it should be allocated so
 *   that it is only accessible by privileged code;  Otherwise, it is a
 *   user mode group and must be allocated so that it accessible by
 *   unprivileged code.
 *
 ****************************************************************************/

/* Functions defined in group/group_malloc.c ********************************/

FAR void *group_malloc(FAR struct task_group_s *group, size_t nbytes);

/* Functions defined in group/group_zalloc.c ********************************/

FAR void *group_zalloc(FAR struct task_group_s *group, size_t nbytes);

/* Functions defined in group/group_free.c **********************************/

void group_free(FAR struct task_group_s *group, FAR void *mem);

#else
  /* In the flat build, there is only one memory allocator and no distinction
   * in privileges.
   */

#  define group_malloc(g,n) kumm_malloc(n)
#  define group_zalloc(g,n) kumm_zalloc(n)
#  define group_free(g,m)   kumm_free(m)

#endif

#undef KMALLOC_EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_KMALLOC_H */
