/****************************************************************************
 * include/gcov.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_GCOV_H
#define __INCLUDE_GCOV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL
#  define GCOV_OP_RESET   1
#  define GCOV_OP_DUMP    2
#  define GCOV_OP_MIN     GCOV_OP_RESET
#  define GCOV_OP_MAX     GCOV_OP_DUMP
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef CONFIG_LIB_SYSCALL
/****************************************************************************
 * Name: gcov
 *
 * Description:
 *   Request kernel space gcov data operations.
 *
 * Parameters:
 *   op - operation code between GCOV_OP_MIN and GCOV_OP_MAX
 *
 * Return:
 *   status code.
 ****************************************************************************/

extern int gcov(int op);
#endif

/****************************************************************************
 * Name: __gcov_reset
 *
 * Description:
 *   Set all counters to zero.
 *
 ****************************************************************************/

extern void __gcov_reset(void);

/****************************************************************************
 * Name: __gcov_dump
 *
 * Description:
 *   Write profile information to a file.
 *
 ****************************************************************************/

extern void __gcov_dump(void);

#if defined(CONFIG_COVERAGE_MINI) && defined(CONFIG_ARCH_TOOLCHAIN_GCC)
/****************************************************************************
 * Name: __gcov_unload
 *
 * Description:
 *   Forget the coverage data in the memory from addr to addr + size.  The
 *   loader calls it before it frees a module.
 *
 ****************************************************************************/

void __gcov_unload(FAR const void *addr, size_t size);
#else
#  define __gcov_unload(addr, size)
#endif

/****************************************************************************
 * Name: __gcov_info_to_gcda
 *
 * Description:
 *   Convert the gcov information referenced by INFO to a gcda data stream.
 *
 * Parameters:
 *   info - Pointer to the gcov information.
 *   filename - Callback function to get the filename.
 *   dump - Callback function to write the gcda data.
 *   allocate - Callback function to allocate memory.
 *   arg - User-provided argument.
 *
 ****************************************************************************/

extern void __gcov_info_to_gcda(FAR const struct gcov_info *info,
                                FAR void (*filename)(FAR const char *,
                                                     FAR void *),
                                FAR void (*dump)(FAR const void *,
                                                 unsigned int, FAR void *),
                                FAR void *(*allocate)(unsigned int,
                                                      FAR void *),
                                FAR void *arg);

/****************************************************************************
 * Name: __gcov_filename_to_gcfn
 *
 * Description:
 *   Convert the filename to a gcfn data stream.
 *
 * Parameters:
 *   filename - Pointer to the filename.
 *   dump - Callback function to write the gcfn data.
 *   arg - User-provided argument.
 *
 ****************************************************************************/

extern void __gcov_filename_to_gcfn(FAR const char *filename,
                                    FAR void (*dump)(FAR const void *,
                                                     unsigned int,
                                                     FAR void *),
                                     FAR void *arg);

/****************************************************************************
 * Name: __gcov_dump_to_memory
 *
 * Description:
 *   Dump gcov data directly to memory
 *
 * Parameters:
 *   ptr - Memory Address
 *   size - Memory block size
 *
 ****************************************************************************/

size_t __gcov_dump_to_memory(FAR void *ptr, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_GCOV_H */
