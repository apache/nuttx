/****************************************************************************
 * arch/arm/src/lc823450/chip.h
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

#ifndef _ARCH_ARM_SRC_LC823450_CHIP_H
#define _ARCH_ARM_SRC_LC823450_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <arch/lc823450/chip.h>
#  include <arch/lc823450/irq.h>
#  include "arm_arch.h"
#  include "lc823450_irq.h"
#  include "arm_arch.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Provide the required number of peripheral interrupt vector definitions.
 * The definition LC823450_IRQ_NEXTINT simply comes from the chip-specific
 * IRQ header file included by arch/lc823450/irq.h.
 */

#define ARMV7M_PERIPHERAL_INTERRUPTS  LC823450_IRQ_NEXTINT

/* Access to COREID register */

#define LC823450_CORE_BASE  0xe00fe000
#define CORE_COREID         (LC823450_CORE_BASE + 0)
#define CORE_COREID_ID      (1 << 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#ifdef __ASSEMBLY__

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the  "top" the correct interrupt stack
 *   for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
  .macro  setintstack, tmp1, tmp2
  ldr \tmp1, =CORE_COREID
  ldr \tmp1, [\tmp1, 0]          /* tmp1 = getreg32(CORE_COREID)  */
  ldr \tmp2, =g_cpu_intstack_top
  ldr sp, [\tmp2, \tmp1, lsl #2] /* sp = g_cpu_intstack_top[tmp1] */
  .endm
#endif /* CONFIG_SMP && CONFIG_ARCH_INTERRUPTSTACK > 7 */

#endif /* __ASSEMBLY__  */
#endif /* _ARCH_ARM_SRC_LC823450_CHIP_H */
