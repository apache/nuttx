/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_irq.c
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

#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <arch/board/board.h>

#include "chip.h"
#include "nvic.h"
#include "ram_vectors.h"
#include "arm_internal.h"

#ifdef CONFIG_SMP
#  include "init/init.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (CXD56M4_SYSH_PRIORITY_DEFAULT << 24 | CXD56M4_SYSH_PRIORITY_DEFAULT << 16 | \
   CXD56M4_SYSH_PRIORITY_DEFAULT << 8 | CXD56M4_SYSH_PRIORITY_DEFAULT)

#define INTC_EN(n) (CXD56_INTC_BASE + 0x10 + (((n) >> 5) << 2))

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
#  define INTSTACK_ALLOC (CONFIG_SMP_NCPUS * INTSTACK_SIZE)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

/* For the case of configurations with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];

#ifdef CONFIG_SMP
static volatile int8_t g_cpu_for_irq[CXD56_IRQ_NIRQS];
extern void up_send_irqreq(int idx, int irq, int cpu);
#endif

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
/* In the SMP configuration, we will need custom interrupt stacks.
 * These definitions provide the aligned stack allocations.
 */

static uint64_t g_intstack_alloc[INTSTACK_ALLOC >> 3];

/* These definitions provide the "top" of the push-down stacks. */

const uint32_t g_cpu_intstack_top[CONFIG_SMP_NCPUS] =
{
  (uint32_t)g_intstack_alloc + INTSTACK_SIZE,
#if CONFIG_SMP_NCPUS > 1
  (uint32_t)g_intstack_alloc + (2 * INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 2
  (uint32_t)g_intstack_alloc + (3 * INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 3
  (uint32_t)g_intstack_alloc + (4 * INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 4
  (uint32_t)g_intstack_alloc + (5 * INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 5
  (uint32_t)g_intstack_alloc + (6 * INTSTACK_SIZE),
#endif /* CONFIG_SMP_NCPUS > 5 */
#endif /* CONFIG_SMP_NCPUS > 4 */
#endif /* CONFIG_SMP_NCPUS > 3 */
#endif /* CONFIG_SMP_NCPUS > 2 */
#endif /* CONFIG_SMP_NCPUS > 1 */
};
#endif /* defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7 */

/* This is the address of the  exception vector table (determined by the
 * linker script).
 */

extern uint32_t _vectors[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void cxd56_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();
  irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
  irqinfo("  INTCTRL:    %08x VECTAB: %08x\n", getreg32(NVIC_INTCTRL),
          getreg32(NVIC_VECTAB));
  irqinfo("  IRQ ENABLE: %08x %08x\n", getreg32(NVIC_IRQ0_31_ENABLE),
          getreg32(NVIC_IRQ32_63_ENABLE));
  irqinfo("  SYSH_PRIO:  %08x %08x %08x\n",
          getreg32(NVIC_SYSH4_7_PRIORITY),
          getreg32(NVIC_SYSH8_11_PRIORITY),
          getreg32(NVIC_SYSH12_15_PRIORITY));
  irqinfo("  IRQ PRIO:   %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_3_PRIORITY),
          getreg32(NVIC_IRQ4_7_PRIORITY),
          getreg32(NVIC_IRQ8_11_PRIORITY),
          getreg32(NVIC_IRQ12_15_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ16_19_PRIORITY),
          getreg32(NVIC_IRQ20_23_PRIORITY),
          getreg32(NVIC_IRQ24_27_PRIORITY),
          getreg32(NVIC_IRQ28_31_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ32_35_PRIORITY),
          getreg32(NVIC_IRQ36_39_PRIORITY),
          getreg32(NVIC_IRQ40_43_PRIORITY),
          getreg32(NVIC_IRQ44_47_PRIORITY));
  irqinfo("              %08x %08x %08x\n",
          getreg32(NVIC_IRQ48_51_PRIORITY),
          getreg32(NVIC_IRQ52_55_PRIORITY),
          getreg32(NVIC_IRQ56_59_PRIORITY));
  leave_critical_section(flags);
}
#else
#  define cxd56_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: cxd56_nmi, cxd56_busfault, cxd56_usagefault, cxd56_pendsv,
 *       cxd56_dbgmonitor, cxd56_pendsv, cxd56_reserved
 *
 * Description:
 *   Handlers for various exceptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int cxd56_nmi(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int cxd56_busfault(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Bus fault received\n");
  PANIC();
  return 0;
}

static int cxd56_usagefault(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Usage fault received\n");
  PANIC();
  return 0;
}

static int cxd56_pendsv(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int cxd56_dbgmonitor(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Debug Monitor received\n");
  PANIC();
  return 0;
}

static int cxd56_reserved(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: cxd56_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_USEBASEPRI
static inline void cxd56_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}
#endif

static int excinfo(int irq, uintptr_t *regaddr, uint32_t *bit)
{
  *regaddr = NVIC_SYSHCON;
  switch (irq)
    {
      case CXD56_IRQ_MEMFAULT:
        *bit = NVIC_SYSHCON_MEMFAULTENA;
        break;

      case CXD56_IRQ_BUSFAULT:
        *bit = NVIC_SYSHCON_BUSFAULTENA;
        break;

      case CXD56_IRQ_USAGEFAULT:
        *bit = NVIC_SYSHCON_USGFAULTENA;
        break;

      case CXD56_IRQ_SYSTICK:
        *regaddr = NVIC_SYSTICK_CTRL;
        *bit     = NVIC_SYSTICK_CTRL_ENABLE;
        break;

      default:
        return ERROR; /* Invalid or unsupported exception */
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Complete initialization of the interrupt system and enable normal,
 *   interrupt processing.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
  int num_priority_registers;

#ifdef CONFIG_SMP
  int i;
  for (i = 0; i < CXD56_IRQ_NIRQS; i++)
    {
      g_cpu_for_irq[i] = -1;
    }
#endif

  /* Disable all interrupts */

  putreg32(0, NVIC_IRQ0_31_ENABLE);
  putreg32(0, NVIC_IRQ32_63_ENABLE);
  putreg32(0, NVIC_IRQ64_95_ENABLE);
  putreg32(0, NVIC_IRQ96_127_ENABLE);

  /* Make sure that we are using the correct vector table.  The default
   * vector address is 0x0000:0000 but if we are executing code that is
   * positioned in SRAM or in external FLASH, then we may need to reset
   * the interrupt vector so that it refers to the table in SRAM or in
   * external FLASH.
   */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARCH_RAMVECTORS
  /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

  arm_ramvec_initialize();
#endif

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  /* The NVIC ICTR register (bits 0-4) holds the number of interrupt
   * lines that the NVIC supports:
   *
   *  0 -> 32 interrupt lines,  8 priority registers
   *  1 -> 64 "       " "   ", 16 priority registers
   *  2 -> 96 "       " "   ", 32 priority registers
   *  ...
   */

  num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;

  /* Now set all of the interrupt lines to the default priority */

  regaddr = NVIC_IRQ0_3_PRIORITY;
  while (num_priority_registers--)
    {
      putreg32(DEFPRIORITY32, regaddr);
      regaddr += 4;
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(CXD56_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(CXD56_IRQ_HARDFAULT, arm_hardfault, NULL);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
  /* up_prioritize_irq(CXD56_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN);
   */

#endif

#ifdef CONFIG_ARMV7M_USEBASEPRI
  cxd56_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARM_MPU
  irq_attach(CXD56_IRQ_MEMFAULT, arm_memfault, NULL);
  up_enable_irq(CXD56_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(CXD56_IRQ_NMI, cxd56_nmi, NULL);
#  ifndef CONFIG_ARM_MPU
  irq_attach(CXD56_IRQ_MEMFAULT, arm_memfault, NULL);
#  endif
  irq_attach(CXD56_IRQ_BUSFAULT, cxd56_busfault, NULL);
  irq_attach(CXD56_IRQ_USAGEFAULT, cxd56_usagefault, NULL);
  irq_attach(CXD56_IRQ_PENDSV, cxd56_pendsv, NULL);
  irq_attach(CXD56_IRQ_DBGMONITOR, cxd56_dbgmonitor, NULL);
  irq_attach(CXD56_IRQ_RESERVED, cxd56_reserved, NULL);
#endif

  cxd56_dumpnvic("initial", CXD56_IRQ_NIRQS);

  /* If a debugger is connected, try to prevent it from catching hardfaults.
   * If CONFIG_ARMV7M_USEBASEPRI, no hardfaults are expected in normal
   * operation.
   */

#if defined(CONFIG_DEBUG_FEATURES) && !defined(CONFIG_ARMV7M_USEBASEPRI)
    {
      uint32_t regval;

      regval  = getreg32(NVIC_DEMCR);
      regval &= ~NVIC_DEMCR_VCHARDERR;
      putreg32(regval, NVIC_DEMCR);
    }
#endif

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (irq >= CXD56_IRQ_EXTINT)
    {
#ifdef CONFIG_SMP
      /* Obtain cpu number which enabled this irq */

      int8_t cpu = g_cpu_for_irq[irq];

      if (-1 == cpu)
        {
          /* Already disabled */

          return;
        }

      /* If a different cpu requested, send an irq request */

      if (cpu != (int8_t)up_cpu_index())
        {
          up_send_irqreq(1, irq, cpu);
          return;
        }

      g_cpu_for_irq[irq] = -1;
#endif

      irqstate_t flags = spin_lock_irqsave(NULL);
      irq -= CXD56_IRQ_EXTINT;
      bit  = 1 << (irq & 0x1f);

      regval  = getreg32(INTC_EN(irq));
      regval &= ~bit;
      putreg32(regval, INTC_EN(irq));
      spin_unlock_irqrestore(NULL, flags);
      putreg32(bit, NVIC_IRQ_CLEAR(irq));
    }
  else
    {
      if (excinfo(irq, &regaddr, &bit) == OK)
        {
          regval  = getreg32(regaddr);
          regval &= ~bit;
          putreg32(regval, regaddr);
        }
    }

  cxd56_dumpnvic("disable", irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bit;

  if (irq >= CXD56_IRQ_EXTINT)
    {
#ifdef CONFIG_SMP
      int cpu = up_cpu_index();

      /* Set the caller cpu for this irq */

      g_cpu_for_irq[irq] = (int8_t)cpu;

      /* EXTINT needs to be handled on CPU0 to avoid deadlock */

      if (irq > CXD56_IRQ_EXTINT && irq != CXD56_IRQ_SW_INT && 0 != cpu)
        {
          up_send_irqreq(0, irq, 0);
          return;
        }
#endif

      irqstate_t flags = spin_lock_irqsave(NULL);
      irq -= CXD56_IRQ_EXTINT;
      bit  = 1 << (irq & 0x1f);

      regval  = getreg32(INTC_EN(irq));
      regval |= bit;
      putreg32(regval, INTC_EN(irq));
      spin_unlock_irqrestore(NULL, flags);
      putreg32(bit, NVIC_IRQ_ENABLE(irq));
    }
  else
    {
      if (excinfo(irq, &regaddr, &bit) == OK)
        {
          regval  = getreg32(regaddr);
          regval |= bit;
          putreg32(regval, regaddr);
        }
    }

  cxd56_dumpnvic("enable", irq);
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
#ifdef CONFIG_ARCH_LEDS_CPU_ACTIVITY
  board_autoled_on(LED_CPU);
#endif

  /* Check for external interrupt */

  if (irq >= CXD56_IRQ_EXTINT)
    {
      irq -= CXD56_IRQ_EXTINT;
      putreg32(1 << (irq & 0x1f), NVIC_IRQ_CLRPEND(irq));
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq >= CXD56_IRQ_MEMFAULT && irq < NR_IRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < CXD56_IRQ_EXTINT)
    {
      /* NVIC_SYSH_PRIORITY() maps {0..15} to one of three priority
       * registers (0-3 are invalid)
       */

      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq -= 4;
    }
  else
    {
      /* NVIC_IRQ_PRIORITY() maps {0..} to one of many priority registers */

      irq -= CXD56_IRQ_EXTINT;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval  = getreg32(regaddr);
  shift   = ((irq & 3) << 3);
  regval &= ~(0xff << shift);
  regval |= (priority << shift);
  putreg32(regval, regaddr);

  cxd56_dumpnvic("prioritize", irq);
  return OK;
}
#endif

/****************************************************************************
 * Name: arm_intstack_top
 *
 * Description:
 *   Return a pointer to the top the correct interrupt stack allocation
 *   for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
uintptr_t arm_intstack_top(void)
{
  return g_cpu_intstack_top[up_cpu_index()];
}
#endif

/****************************************************************************
 * Name: arm_intstack_alloc
 *
 * Description:
 *   Return a pointer to the "alloc" the correct interrupt stack allocation
 *   for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
uintptr_t arm_intstack_alloc(void)
{
  return g_cpu_intstack_top[up_cpu_index()] - INTSTACK_SIZE;
}
#endif
