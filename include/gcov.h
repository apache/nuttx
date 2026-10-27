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

#include <stdint.h>
#include <sys/types.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct gcov_info;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
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
 * Name: __gcov_init, __gcov_merge_add, __gcov_exit, __gcov_fork
 *
 * Description:
 *   The functions that instrumented code calls.  The symbol table of the
 *   loadable modules takes them from here.
 *
 ****************************************************************************/

void __gcov_init(FAR struct gcov_info *info);
void __gcov_merge_add(FAR uint64_t *counters, unsigned int n_counters);
void __gcov_exit(void);
#ifdef CONFIG_ARCH_HAVE_FORK
pid_t __gcov_fork(void);
#endif

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

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_GCOV_H */
