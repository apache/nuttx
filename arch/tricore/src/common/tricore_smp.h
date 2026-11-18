/****************************************************************************
 * arch/tricore/src/common/tricore_smp.h
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

#ifndef __ARCH_TRICORE_SRC_COMMON_TRICORE_SMP_H
#define __ARCH_TRICORE_SRC_COMMON_TRICORE_SMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <limits.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Every secondary CPU keeps its CSA pool, its interrupt stack and its idle
 * stack at the start of its own data scratchpad RAM, at the global address
 * of that RAM: segment 7 for core 0, 6 for core 1 and so on.
 */

#define TRICORE_SMP_SEG_SHIFT       28
#define TRICORE_SMP_DSPR_SEG_TOP    7
#define TRICORE_SMP_CSA_FRAME       64
#define TRICORE_SMP_CSA_RESERVE     3

#define TRICORE_SMP_CSA_SIZE \
  (CONFIG_TRICORE_SMP_CSA_COUNT * TRICORE_SMP_CSA_FRAME)

#define TRICORE_SMP_ISTACK_TOP \
  (TRICORE_SMP_CSA_SIZE + CONFIG_ARCH_INTERRUPTSTACK)

#define TRICORE_SMP_USTACK_TOP \
  (TRICORE_SMP_ISTACK_TOP + CONFIG_IDLETHREAD_STACKSIZE)

#define TRICORE_SMP_DSPR(coreid) \
  ((TRICORE_SMP_DSPR_SEG_TOP - (coreid)) << TRICORE_SMP_SEG_SHIFT)

/* On TC3x CPU5 has the core ID 6 */

#ifdef CONFIG_ARCH_CHIP_TC3X
#  define TRICORE_SMP_TC3X_CPU5         5
#  define TRICORE_SMP_TC3X_CORE_ID_CPU5 6
#  define TRICORE_SMP_CORE_ID(cpu) \
     ((cpu) == TRICORE_SMP_TC3X_CPU5 ? TRICORE_SMP_TC3X_CORE_ID_CPU5 : (cpu))
#  define TRICORE_SMP_CPU_OF_CORE(coreid) \
     ((coreid) == TRICORE_SMP_TC3X_CORE_ID_CPU5 ? \
      TRICORE_SMP_TC3X_CPU5 : (coreid))
#else
#  define TRICORE_SMP_CORE_ID(cpu)        (cpu)
#  define TRICORE_SMP_CPU_OF_CORE(coreid) (coreid)
#endif

#define TRICORE_SMP_CPU_DSPR(cpu) \
  TRICORE_SMP_DSPR(TRICORE_SMP_CORE_ID(cpu))

/* CSA balance: free frames scanned at each context switch, the low water
 * mark checked every TRICORE_SMP_CSA_LOW_EVERY switches, and a full scan
 */

#define TRICORE_SMP_CSA_SWITCH_SCAN 8
#define TRICORE_SMP_CSA_LOW_WATER   32
#define TRICORE_SMP_CSA_LOW_EVERY   64
#define TRICORE_SMP_CSA_SCAN_ALL    UINT_MAX

/* The inter-CPU interrupt from CPU "from" to CPU "to" is GPSR from/to */

#define TRICORE_SMP_IPI_IRQ(from, to) TRICORE_GPSR_IRQNUM(from, to)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_SMP
void tricore_csa_balance(void);
void tricore_csa_switch_balance(void);
void tricore_csa_reclaim_ipi(void);
void tricore_smp_initialize(void);
void tricore_cpu_boot(void);
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_TRICORE_SRC_COMMON_TRICORE_SMP_H */
