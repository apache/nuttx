/****************************************************************************
 * include/nuttx/lib/lib.h
 * Non-standard, internal APIs available in lib/.
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

#ifndef __INCLUDE_NUTTX_LIB_LIB_H
#define __INCLUDE_NUTTX_LIB_LIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The NuttX C library can be built in two modes: (1) as a standard,
 * C-library that can be used by normal, user-space applications, or
 * (2) as a special, kernel-mode C-library only used within the OS.
 * If NuttX is not being built as separated kernel- and user-space modules,
 * then only the first mode is supported.
 */

#if defined(__KERNEL__)

  /* Domain-specific allocations */

#  define lib_malloc(s)       kmm_malloc(s)
#  define lib_malloc_size(p)  kmm_malloc_size(p)
#  define lib_zalloc(s)       kmm_zalloc(s)
#  define lib_realloc(p,s)    kmm_realloc(p,s)
#  define lib_memalign(p,s)   kmm_memalign(p,s)
#  define lib_free(p)         kmm_free(p)

  /* User-accessible allocations */

#  define lib_umalloc(s)      kumm_malloc(s)
#  define lib_umalloc_size(p) kumm_malloc_size(p)
#  define lib_uzalloc(s)      kumm_zalloc(s)
#  define lib_urealloc(p,s)   kumm_realloc(p,s)
#  define lib_umemalign(p,s)  kumm_memalign(p,s)
#  define lib_ufree(p)        kumm_free(p)

#else

  /* Domain-specific allocations */

#  define lib_malloc(s)       malloc(s)
#  define lib_malloc_size(p)  malloc_size(p)
#  define lib_zalloc(s)       zalloc(s)
#  define lib_realloc(p,s)    realloc(p,s)
#  define lib_memalign(p,s)   memalign(p,s)
#  define lib_free(p)         free(p)

  /* User-accessible allocations */

#  define lib_umalloc(s)      malloc(s)
#  define lib_umalloc_size(p) malloc_size(p)
#  define lib_uzalloc(s)      zalloc(s)
#  define lib_urealloc(p,s)   realloc(p,s)
#  define lib_umemalign(p,s)  memalign(p,s)
#  define lib_ufree(p)        free(p)

#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

/* Functions contained in lib_streams.c *************************************/

#ifdef CONFIG_FILE_STREAM
struct task_group_s;
void lib_stream_initialize(FAR struct task_group_s *group);
void lib_stream_release(FAR struct task_group_s *group);
#endif

/* Functions defined in lib_filesem.c ***************************************/

#ifdef CONFIG_STDIO_DISABLE_BUFFERING
#  define lib_sem_initialize(s)
#  define lib_take_semaphore(s)
#  define lib_give_semaphore(s)
#else
void lib_sem_initialize(FAR struct file_struct *stream);
void lib_take_semaphore(FAR struct file_struct *stream);
void lib_give_semaphore(FAR struct file_struct *stream);
#endif

/* Functions defined in lib_srand.c *****************************************/

unsigned long nrand(unsigned long limit);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_LIB_LIB_H */
