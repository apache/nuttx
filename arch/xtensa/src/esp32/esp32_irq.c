/****************************************************************************
 * arch/xtensa/src/esp32/esp32_irq.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "esp32_cpuint.h"
#include "esp32_smp.h"
#include "esp32_gpio.h"

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

#if defined(CONFIG_STACK_COLORATION) && defined(HAVE_INTERRUPTSTACK)
  /* Colorize the interrupt stack for debug purposes */

#warning Missing logic
#endif

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
