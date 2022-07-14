/****************************************************************************
 * arch/arm64/src/common/addrenv.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ADDRENV_H
#define __ARCH_ARM64_SRC_COMMON_ADDRENV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include "arm64_internal.h"

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Aligned size of the kernel stack */

#ifdef CONFIG_ARCH_KERNEL_STACK
#  define ARCH_KERNEL_STACKSIZE  STACK_ALIGN_UP(CONFIG_ARCH_KERNEL_STACKSIZE)
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

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

/****************************************************************************
 * Name: arm_addrenv_create_region
 *
 * Description:
 *   Create one memory region.
 *
 * Returned Value:
 *   On success, the number of pages allocated is returned.  Otherwise, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

int arm64_addrenv_create_region(uintptr_t **list, size_t listlen,
                                uintptr_t vaddr, size_t regionsize,
                                uint32_t mmuflags);

/****************************************************************************
 * Name: arm_addrenv_destroy_region
 *
 * Description:
 *   Destroy one memory region.
 *
 ****************************************************************************/

void arm64_addrenv_destroy_region(uintptr_t **list, size_t listlen,
                                  uintptr_t vaddr, bool keep);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_ARCH_ADDRENV */
#endif /* __ARCH_ARM64_SRC_COMMON_ADDRENV_H */
