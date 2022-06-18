/****************************************************************************
 * arch/arm64/src/common/arm64_smp.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_SMP_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_SMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "arm64_internal.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm_cpu_boot
 *
 * Description:
 *   Continues the C-level initialization started by the assembly language
 *   __cpu[n]_start function.  At a minimum, this function needs to
 *   initialize interrupt handling and, perhaps, wait on WFI for
 *   arm_cpu_start() to issue an SGI.
 *
 *   This function must be provided by the each ARMv7-A MCU and implement
 *   MCU-specific initialization logic.
 *
 * Input Parameters:
 *   cpu - The CPU index.  This is the same value that would be obtained by
 *      calling up_cpu_index();
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

void arm64_cpu_boot(int cpu);

/****************************************************************************
 * Name: arm64_enable_smp
 *
 * Description:
 *
 * Returned Value:
 *
 ****************************************************************************/

void arm64_enable_smp(int cpu);

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SMP */
#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_SMP_H */
