/****************************************************************************
 * arch/arm/include/arm_a_r/arch.h
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

#ifndef __ARCH_ARM_INCLUDE_ARMV_A_R_ARCH_H
#define __ARCH_ARM_INCLUDE_ARMV_A_R_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/arm_a_r/cp15.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return the real core number regardless CONFIG_SMP setting
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_MULTICPU
noinstrument_function
static inline_function int up_cpu_index(void)
{
  uint32_t mpidr;

  /* Read the Multiprocessor Affinity Register (MPIDR) */

  mpidr = CP15_GET(MPIDR);

  /* And return the CPU ID field */

  return (mpidr & MPIDR_CPUID_MASK) >> MPIDR_CPUID_SHIFT;
}
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_ARMV_A_R_ARCH_H */
