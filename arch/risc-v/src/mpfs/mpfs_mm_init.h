/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_mm_init.h
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

#ifndef __ARCH_RISC_V_SRC_MPFS_MPFS_MM_INIT_H
#define __ARCH_RISC_V_SRC_MPFS_MPFS_MM_INIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "riscv_mmu.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uintptr_t g_kernel_pgt_pbase;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_kernel_mappings
 *
 * Description:
 *  Setup kernel mappings when using CONFIG_BUILD_KERNEL. Sets up the kernel
 *  MMU mappings.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
void mpfs_kernel_mappings(void);
#endif

/****************************************************************************
 * Name: mpfs_mm_init
 *
 * Description:
 *  Setup kernel mappings when using CONFIG_BUILD_KERNEL. Sets up kernel MMU
 *  mappings. Function also sets the first address environment (satp value).
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
static inline void mpfs_mm_init(void)
{
  /* Setup the kernel mappings */

  mpfs_kernel_mappings();

  /* Enable MMU (note: system is still in M-mode) */

  mmu_enable(g_kernel_pgt_pbase, 0);
}
#endif

#endif /* __ARCH_RISC_V_SRC_MPFS_MPFS_MM_INIT_H */
