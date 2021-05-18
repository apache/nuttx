/****************************************************************************
 * arch/xtensa/src/esp32/esp32_irq.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "esp32_cpuint.h"
#include "esp32_smp.h"
#include "esp32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt stack definitions for SMP */

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
#  define INTSTACK_ALLOC (CONFIG_SMP_NCPUS * INTSTACK_SIZE)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a reference to the current interrupt level
 * register storage structure.  It is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

#ifdef CONFIG_SMP
/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];

#else

volatile uint32_t *g_current_regs[1];

#endif

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
/* In the SMP configuration, we will need custom interrupt stacks.
 * These definitions provide the aligned stack allocations.
 */

static uint32_t g_intstackalloc[INTSTACK_ALLOC >> 2];

/* These definitions provide the "top" of the push-down stacks. */

uintptr_t g_cpu_intstack_top[CONFIG_SMP_NCPUS] =
{
  (uintptr_t)g_intstackalloc + INTSTACK_SIZE,
#if CONFIG_SMP_NCPUS > 1
  (uintptr_t)g_intstackalloc + (2 * INTSTACK_SIZE),
#endif /* CONFIG_SMP_NCPUS > 1 */
};
#endif /* defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_irq_dump
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void esp32_irq_dump(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();
#warning Missing logic
  leave_critical_section(flags);
}
#else
#  define esp32_irq_dump(msg, irq)
#endif

/****************************************************************************
 * Name: xtensa_attach_fromcpu1_interrupt
 ****************************************************************************/

#ifdef CONFIG_SMP
static inline void xtensa_attach_fromcpu1_interrupt(void)
{
  int cpuint;

  /* Allocate a level-sensitive, priority 1 CPU interrupt for the UART */

  cpuint = esp32_alloc_levelint(1);
  DEBUGASSERT(cpuint >= 0);

  /* Connect all CPU peripheral source to allocated CPU interrupt */

  up_disable_irq(cpuint);
  esp32_attach_peripheral(0, ESP32_PERIPH_CPU_CPU1, cpuint);

  /* Attach the inter-CPU interrupt. */

  irq_attach(ESP32_IRQ_CPU_CPU1, (xcpt_t)esp32_fromcpu1_interrupt, NULL);

  /* Enable the inter 0 CPU interrupt. */

  up_enable_irq(cpuint);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Initialize CPU interrupts */

  esp32_cpuint_initialize();

  /* Attach and enable internal interrupts */

#ifdef CONFIG_SMP
  /* Attach and enable the inter-CPU interrupt */

  xtensa_attach_fromcpu1_interrupt();
#endif

  esp32_irq_dump("initial", NR_IRQS);

#ifdef CONFIG_ESP32_GPIO_IRQ
  /* Initialize GPIO interrupt support */

  esp32_gpioirqinitialize();
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts.  Also clears PS.EXCM */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: xtensa_intstack_top
 *
 * Description:
 *   Return a pointer to the top of the correct interrupt stack for the
 *   given CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
uintptr_t xtensa_intstack_top(void)
{
  return g_cpu_intstack_top[up_cpu_index()];
}

/****************************************************************************
 * Name: xtensa_intstack_alloc
 *
 * Description:
 *   Return a pointer to the "alloc" the correct interrupt stack allocation
 *   for the current CPU.
 *
 ****************************************************************************/

uintptr_t xtensa_intstack_alloc(void)
{
  return g_cpu_intstack_top[up_cpu_index()] - INTSTACK_SIZE;
}
#endif
