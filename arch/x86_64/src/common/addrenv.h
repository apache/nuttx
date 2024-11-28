/****************************************************************************
 * arch/x86_64/src/common/addrenv.h
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

#ifndef __ARCH_X86_64_SRC_COMMON_ADDRENV_H
#define __ARCH_X86_64_SRC_COMMON_ADDRENV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include "x86_64_internal.h"

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Aligned size of the kernel stack */

#ifdef CONFIG_ARCH_KERNEL_STACK
#  define ARCH_KERNEL_STACKSIZE STACK_ALIGN_UP(CONFIG_ARCH_KERNEL_STACKSIZE)
#endif

/* Base address for address environment */

#if CONFIG_ARCH_TEXT_VBASE != 0
#  define ARCH_ADDRENV_VBASE    (CONFIG_ARCH_TEXT_VBASE)
#else
#  define ARCH_ADDRENV_VBASE    (CONFIG_ARCH_DATA_VBASE)
#endif

/* Maximum user address environment size */

#define ARCH_ADDRENV_MAX_SIZE   (0x40000000)

/* User address environment end */

#define ARCH_ADDRENV_VEND       (ARCH_ADDRENV_VBASE + ARCH_ADDRENV_MAX_SIZE - 1)

/* Flags for kernel page tables */

#define MMU_KPGT_FLAGS          (X86_PAGE_WR)

/* Mark user memory if in kernel build */

#ifdef CONFIG_BUILD_KERNEL
#  define MMU_USER_DEFAULT      (X86_PAGE_USER)
#else
#  define MMU_USER_DEFAULT      (0)
#endif

/* Flags for user page tables */

#define MMU_UPGT_FLAGS          (X86_PAGE_WR | MMU_USER_DEFAULT)

/* Flags for user FLASH (RX) and user RAM (RW) */

#define MMU_UDATA_FLAGS         (X86_PAGE_WR | MMU_USER_DEFAULT)
#define MMU_UTEXT_FLAGS         (X86_PAGE_WR | MMU_USER_DEFAULT)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_ADDRENV */
#endif /* __ARCH_X86_64_SRC_COMMON_ADDRENV_H */
