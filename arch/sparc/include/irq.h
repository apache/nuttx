/****************************************************************************
 * arch/sparc/include/irq.h
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_SPARC_INCLUDE_IRQ_H
#define __ARCH_SPARC_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/* Include NuttX-specific IRQ definitions */

#include <nuttx/irq.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/* Include AVR architecture-specific IRQ definitions (including register
 * save structure and up_irq_save()/up_irq_restore() macros
 */

#ifdef CONFIG_ARCH_SPARC_V8
# include <arch/sparc_v8/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
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
 * Inline functions
 ****************************************************************************/

/* Return the current value of the stack pointer */

static inline uint32_t up_getsp(void)
{
  uint32_t retval;

  do
    {
      retval = 0;
      __asm__ volatile("mov %%sp, %0" :  "=r" (retval) : "0" (retval));
    }
  while (0);

  return retval;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This holds a references to the current interrupt level register storage
 * structure.  If is non-NULL only during interrupt processing.
 */

EXTERN volatile uint32_t *g_current_regs;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return an index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 ****************************************************************************/

#define up_cpu_index() (0)

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description:
 *   Return true is we are currently executing in the interrupt
 *   handler context.
 *
 ****************************************************************************/

#define up_interrupt_context() (g_current_regs != NULL)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_SPARC_INCLUDE_IRQ_H */

