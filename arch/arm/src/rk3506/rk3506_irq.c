/****************************************************************************
 * arch/arm/src/rk3506/rk3506_irq.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "sctlr.h"
#include "gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size of the interrupt stack allocation */

#define INTSTACK_ALLOC (CONFIG_SMP_NCPUS * INTSTACK_SIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
/* In the SMP configuration, we will need custom IRQ and FIQ stacks.
 * These definitions provide the aligned stack allocations.
 */

static uint64_t g_irqstack_alloc[INTSTACK_ALLOC >> 3];
static uint64_t g_fiqstack_alloc[INTSTACK_ALLOC >> 3];

/* These are arrays that point to the top of each interrupt stack */

uintptr_t g_irqstack_top[CONFIG_SMP_NCPUS] =
{
  (uintptr_t)g_irqstack_alloc + INTSTACK_SIZE,
#if CONFIG_SMP_NCPUS > 1
  (uintptr_t)g_irqstack_alloc + (2 * INTSTACK_SIZE),
#endif
#if CONFIG_SMP_NCPUS > 2
  (uintptr_t)g_irqstack_alloc + (3 * INTSTACK_SIZE),
#endif
#if CONFIG_SMP_NCPUS > 3
  (uintptr_t)g_irqstack_alloc + (4 * INTSTACK_SIZE)
#endif
};

uintptr_t g_fiqstack_top[CONFIG_SMP_NCPUS] =
{
  (uintptr_t)g_fiqstack_alloc + INTSTACK_SIZE,
#if CONFIG_SMP_NCPUS > 1
  (uintptr_t)g_fiqstack_alloc + 2 * INTSTACK_SIZE,
#endif
#if CONFIG_SMP_NCPUS > 2
  (uintptr_t)g_fiqstack_alloc + 3 * INTSTACK_SIZE,
#endif
#if CONFIG_SMP_NCPUS > 3
  (uintptr_t)g_fiqstack_alloc + 4 * INTSTACK_SIZE
#endif
};

#endif

/* Symbols defined via the linker script */

extern uint8_t _vector_start[]; /* Beginning of vector block */
extern uint8_t _vector_end[];   /* End+1 of vector block */

/* Raw UART4 debug breadcrumb helpers (defined in rk3506_boot.c). Used to
 * localize where boot hangs, independent of the OS/serial driver. The
 * marker prints repeatedly so it survives the CH340 USB re-enumeration
 * window (board-powered adapter re-enumerates 2-3s on every reset).
 */

void rk3506_rawuart_puts(const char *s);
void rk3506_dbgmark(const char *tag);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   This function is called by up_initialize() during the bring-up of the
 *   system.  It is the responsibility of this function to but the interrupt
 *   subsystem into the working and ready state.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
#ifdef CONFIG_RK3506_AMP_SECONDARY
  int i;
#endif

#ifdef CONFIG_ARCH_LOWVECTORS
  /* Set the VBAR register to the address of the vector table */

  DEBUGASSERT((((uintptr_t)_vector_start) & ~VBAR_MASK) == 0);
  cp15_wrvbar((uint32_t)_vector_start);
#endif

  /* The following operations need to be atomic, but since this function is
   * called early in the initialization sequence, we expect to have exclusive
   * access to the GIC.
   */

#ifdef CONFIG_RK3506_AMP_SECONDARY
  /* AMP secondary core: the GIC distributor (GICD) is owned and already
   * initialized by the Linux side running on the other cores. We must NOT
   * call arm_gic0_initialize() here, otherwise we would reset the global
   * distributor configuration and break Linux. Only initialize our own
   * GIC CPU interface (GICC).
   */

  arm_gic_initialize();   /* CPU interface (GICC) only */

  /* AMP coexistence fix (root cause of the early IRQ-storm crash):
   *
   * arm_gic_initialize() only disables this core's banked SGIs/PPIs
   * (IRQ 0-31). Any SPI (IRQ >= 32) left enabled, pending, and targeting
   * CPU2 in the shared distributor would be taken the instant we run
   * up_irq_enable() below, vectoring into irq_unexpected_isr() (no handler)
   * and producing a crash/garbage loop on UART4 (observed symptom).
   *
   * NuttX on CPU2 is brought up by U-Boot BEFORE Linux configures the GIC,
   * so it is safe to clear the SPI enable bits here: Linux re-enables the
   * SPIs it needs during its own GIC init later. This mirrors RT-Thread's
   * arm_gic_dist_init(), proven to coexist with Linux on this SoC.
   */

  for (i = 32; i < NR_IRQS; i += 32)
    {
      putreg32(0xffffffff, GIC_ICDICER(i));  /* Clear-Enable: disable SPIs */
    }
#else
  /* Initialize the Generic Interrupt Controller (GIC) for CPU0.
   * In AMP mode, we want arm_gic0_initialize to be called only once.
   */

  if (sched_getcpu() == 0)
    {
      arm_gic0_initialize();  /* Initialization unique to CPU0 */
    }

  arm_gic_initialize();   /* Initialization common to all CPUs */
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  arm_color_intstack();
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_get_intstackbase
 *
 * Description:
 *   Return a pointer to the "alloc" the correct interrupt stack allocation
 *   for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
uintptr_t up_get_intstackbase(int cpu)
{
  return g_irqstack_top[cpu] - INTSTACK_SIZE;
}
#endif
