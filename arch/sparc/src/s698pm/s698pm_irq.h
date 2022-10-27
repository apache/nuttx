/****************************************************************************
 * arch/sparc/src/s698pm/s698pm_irq.h
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

#ifndef __ARCH_SPARC_SRC_S698PM_S698PM_IRQ_H
#define __ARCH_SPARC_SRC_S698PM_S698PM_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CPU interrupt types. */

#define S698PM_CPUINT_LEVEL   0
#define S698PM_CPUINT_EDGE    1

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  s698pm_cpuint_initialize
 *
 * Description:
 *   Initialize CPU interrupts
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int s698pm_cpuint_initialize(void);

/****************************************************************************
 * Name:  s698pm_setup_irq
 *
 * Description:
 *   This function sets up the IRQ. It allocates a CPU interrupt of the given
 *   priority andattaches it to the given irq.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0~3
 *   irq      - The irq number from irq.h to be assigned to a EXT interrupt.
 *   priority - Interrupt's priority (0~1).
 *
 * Returned Value:
 *   The allocated CPU interrupt on success, a negated errno value on
 *   failure.
 *
 ****************************************************************************/

int s698pm_setup_irq(int cpu, int irq, int priority);

/****************************************************************************
 * Name:  s698pm_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by s698pm_setup_irq.
 *   It detaches a ext interrupt from a CPU irq.
 *
 * Input Parameters:
 *   irq      - The irq number from irq.h to be assigned to a EXT interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void s698pm_teardown_irq(int irq);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_SPARC_SRC_S698PM_S698PM_IRQ_H */
