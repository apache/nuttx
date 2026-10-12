/****************************************************************************
 * mm/umm_heap/umm_heap.h
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

#ifndef __MM_UMM_HEAP_UMM_HEAP_H
#define __MM_UMM_HEAP_UMM_HEAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include <nuttx/addrenv.h>
#include <nuttx/mm/mm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* errno is in the TLS of the calling thread.  The kernel uses the user heap
 * through kumm_*(), also while the address environment of another process
 * is selected, where that TLS is not mapped.  As kmm_*(), the kernel copy
 * of the user heap does not set errno.
 */

#if defined(CONFIG_BUILD_FLAT) || !defined(__KERNEL__)
#  define umm_set_errno(e) set_errno(e)
#else
#  define umm_set_errno(e) ((void)(e))
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
void umm_try_initialize(void);
#endif

#endif /* __MM_UMM_HEAP_UMM_HEAP_H */
