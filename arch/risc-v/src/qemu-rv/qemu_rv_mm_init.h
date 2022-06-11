/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_mm_init.h
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

#ifndef __ARCH_RISC_V_SRC_QEMURV_QEMURV_MM_INIT_H
#define __ARCH_RISC_V_SRC_QEMURV_QEMURV_MM_INIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "riscv_mmu.h"

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_rv_kernel_mappings
 *
 * Description:
 *  Setup kernel mappings when using CONFIG_BUILD_KERNEL. Sets up the kernel
 *  MMU mappings.
 *
 ****************************************************************************/

void qemu_rv_kernel_mappings(void);

/****************************************************************************
 * Name: qemu_rv_mm_init
 *
 * Description:
 *  Setup kernel mappings when using CONFIG_BUILD_KERNEL. Sets up kernel MMU
 *  mappings. Function also sets the first address environment (satp value).
 *
 ****************************************************************************/

void qemu_rv_mm_init(void);

#endif /* __ARCH_RISC_V_SRC_QEMURV_QEMURV_MM_INIT_H */
