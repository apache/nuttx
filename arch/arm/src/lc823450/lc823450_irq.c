/****************************************************************************
 * arch/arm/src/lc823450/lc823450_irq.c
 *
 *   Copyright 2014,2015,2016,2017,2018 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
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
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "ram_vectors.h"
#include "up_arch.h"
#include "up_internal.h"
#include "chip.h"
#include "lc823450_intc.h"

#ifdef CONFIG_DVFS
#  include "lc823450_dvfs2.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
   NVIC_SYSH_PRIORITY_DEFAULT)

/* Given the address of a NVIC ENABLE register, this is the offset to
 * the corresponding CLEAR ENABLE register.
 */

#define NVIC_ENA_OFFSET    (0)
#define NVIC_CLRENA_OFFSET (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)

/* Size of the interrupt stack allocation */

#define INTSTACK_ALLOC (CONFIG_SMP_NCPUS * INTSTACK_SIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_SMP
/* For the case of configurations with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];
#else
volatile uint32_t *g_current_regs[1];
#endif

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
/* In the SMP configuration, we will need two custom interrupt stacks.
 * These definitions provide the aligned stack allocations.
 */

uint64_t g_instack_alloc[INTSTACK_ALLOC >> 3];

/* These definitions provide the "top" of the push-down stacks. */

const uint32_t g_cpu0_instack_base =
  (uint32_t)g_instack_alloc + INTSTACK_SIZE;

#if CONFIG_SMP_NCPUS > 1
const uint32_t g_cpu1_instack_base =
  (uint32_t)g_instack_alloc + 2 * INTSTACK_SIZE;
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LC823450_VIRQ
static struct lc823450_irq_ops *virq_ops[LC823450_IRQ_NVIRTUALIRQS];
#endif /* CONFIG_LC823450_VIRQ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *current_regs;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void lc823450_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();
  irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
  irqinfo("  INTCTRL:    %08x VECTAB:  %08x\n",
          getreg32(NVIC_INTCTRL), getreg32(NVIC_VECTAB));
#if 0
  irqinfo("  SYSH ENABLE MEMFAULT: %08x BUSFAULT: %08x USGFAULT: %08x SYSTICK: %08x\n",
          getreg32(NVIC_SYSHCON_MEMFAULTENA), getreg32(NVIC_SYSHCON_BUSFAULTENA),
          getreg32(NVIC_SYSHCON_USGFAULTENA), getreg32(NVIC_SYSTICK_CTRL_ENABLE));
#endif
  irqinfo("  IRQ ENABLE: %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ32_63_ENABLE),
          getreg32(NVIC_IRQ64_95_ENABLE));
  irqinfo("  SYSH_PRIO:  %08x %08x %08x\n",
          getreg32(NVIC_SYSH4_7_PRIORITY), getreg32(NVIC_SYSH8_11_PRIORITY),
          getreg32(NVIC_SYSH12_15_PRIORITY));
  irqinfo("  IRQ PRIO:   %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_3_PRIORITY), getreg32(NVIC_IRQ4_7_PRIORITY),
          getreg32(NVIC_IRQ8_11_PRIORITY), getreg32(NVIC_IRQ12_15_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ16_19_PRIORITY), getreg32(NVIC_IRQ20_23_PRIORITY),
          getreg32(NVIC_IRQ24_27_PRIORITY), getreg32(NVIC_IRQ28_31_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ32_35_PRIORITY), getreg32(NVIC_IRQ36_39_PRIORITY),
          getreg32(NVIC_IRQ40_43_PRIORITY), getreg32(NVIC_IRQ44_47_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ48_51_PRIORITY), getreg32(NVIC_IRQ52_55_PRIORITY),
          getreg32(NVIC_IRQ56_59_PRIORITY), getreg32(NVIC_IRQ60_63_PRIORITY));
  irqinfo("              %08x\n",
          getreg32(NVIC_IRQ64_67_PRIORITY));
  leave_critical_section(flags);
}
#else
#  define lc823450_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: lc823450_nmi, lc823450_busfault, lc823450_usagefault, lc823450_pendsv,
 *       lc823450_dbgmonitor, lc823450_pendsv, lc823450_reserved
 *
 * Description:
 *   Handlers for various execptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
static int lc823450_nmi(int irq, FAR void *context, FAR void *arg)
{
  (void)enter_critical_section();
  irqinfo("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int lc823450_busfault(int irq, FAR void *context, FAR void *arg)
{
  (void)enter_critical_section();
  irqinfo("PANIC!!! Bus fault received: %08x\n", getreg32(NVIC_CFAULTS));
  PANIC();
  return 0;
}

static int lc823450_usagefault(int irq, FAR void *context, FAR void *arg)
{
  (void)enter_critical_section();
  irqinfo("PANIC!!! Usage fault received: %08x\n", getreg32(NVIC_CFAULTS));
  PANIC();
  return 0;
}

static int lc823450_pendsv(int irq, FAR void *context, FAR void *arg)
{
  (void)enter_critical_section();
  irqinfo("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int lc823450_dbgmonitor(int irq, FAR void *context, FAR void *arg)
{
  (void)enter_critical_section();
  irqinfo("PANIC!!! Debug Monitor receieved\n");
  PANIC();
  return 0;
}

static int lc823450_reserved(int irq, FAR void *context, FAR void *arg)
{
  (void)enter_critical_section();
  irqinfo("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: lc823450_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_USEBASEPRI
static inline void lc823450_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval  = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}
#endif

/***********************************************************************
 * Name: lc823450_extint_clr
 *
 * Description:
 *   clear irq factor
 *
 ***********************************************************************/

static void lc823450_extint_clr(int irq)
{
  uint32_t regaddr;
  int port;
  int pin;

  DEBUGASSERT(irq >= LC823450_IRQ_GPIO00 && irq <= LC823450_IRQ_GPIO59);

  irq -= LC823450_IRQ_GPIO00;

  port = (irq & 0x70) >> 4;
  pin = irq & 0xf;

  regaddr = INTC_REG(EXTINTnCLR_BASE, port);
  putreg32(1 << pin, regaddr);

  return;
}

/***********************************************************************
 * Name: lc823450_extint_isr
 *
 * Description:
 *   Handle external interrupt.
 *
 ***********************************************************************/

static int lc823450_extint_isr(int irq, FAR void *context, FAR void *arg)
{
  uint32_t regaddr;
  uint32_t pending;
  int port;

  DEBUGASSERT(irq >= LC823450_IRQ_EXTINT0 && irq <= LC823450_IRQ_EXTINT5);

  port = irq - LC823450_IRQ_EXTINT0;

  /* Read irq factor */

  regaddr = INTC_REG(EXTINTn_BASE, port);
  pending = getreg32(regaddr);

  /* Clear irq factor */

  regaddr = INTC_REG(EXTINTnCLR_BASE, port);
  putreg32(pending, regaddr);

  irq = LC823450_IRQ_GPIO00 + (port * 0x10);

  /* Re-deliver the IRQ */

  for ( ; pending != 0; irq++, pending >>= 1)
    {
      if ((pending & 1) != 0)
        {
          irq_dispatch(irq, context);
        }
    }

  return OK;
}

/***********************************************************************
 * Name: lc823425_extint_initialize
 *
 * Description:
 *   Initialize external interrup.
 *
 ***********************************************************************/

static void lc823450_extint_initialize(void)
{
  int ret;

  ret = irq_attach(LC823450_IRQ_EXTINT0, lc823450_extint_isr, NULL);
  DEBUGASSERT(ret == OK);

  ret = irq_attach(LC823450_IRQ_EXTINT1, lc823450_extint_isr, NULL);
  DEBUGASSERT(ret == OK);

  ret = irq_attach(LC823450_IRQ_EXTINT2, lc823450_extint_isr, NULL);
  DEBUGASSERT(ret == OK);

  ret = irq_attach(LC823450_IRQ_EXTINT3, lc823450_extint_isr, NULL);
  DEBUGASSERT(ret == OK);

  ret = irq_attach(LC823450_IRQ_EXTINT4, lc823450_extint_isr, NULL);
  DEBUGASSERT(ret == OK);

  ret = irq_attach(LC823450_IRQ_EXTINT5, lc823450_extint_isr, NULL);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);

  up_enable_irq(LC823450_IRQ_EXTINT0);
  up_enable_irq(LC823450_IRQ_EXTINT1);
  up_enable_irq(LC823450_IRQ_EXTINT2);
  up_enable_irq(LC823450_IRQ_EXTINT3);
  up_enable_irq(LC823450_IRQ_EXTINT4);
  up_enable_irq(LC823450_IRQ_EXTINT5);
}

/****************************************************************************
 * Name: lc823450_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int lc823450_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                         uintptr_t offset)
{
  DEBUGASSERT(irq >= LC823450_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= LC823450_IRQ_NIRQS)
    {
      int port = ((irq - LC823450_IRQ_GPIO00) & 0x70) >> 4;

      *regaddr = INTC_REG(EXTINTnM_BASE, port);
      *bit = 1 << ((irq - LC823450_IRQ_GPIO00) & 0xf);
    }
  else if (irq >= LC823450_IRQ_INTERRUPTS)
    {
      if (irq < LC823450_IRQ_INTERRUPTS + 32)
        {
           *regaddr = (NVIC_IRQ0_31_ENABLE + offset);
           *bit     = 1 << (irq - LC823450_IRQ_INTERRUPTS);
        }
      else if (irq < LC823450_IRQ_INTERRUPTS + 64)
        {
           *regaddr = (NVIC_IRQ32_63_ENABLE + offset);
           *bit     = 1 << (irq - LC823450_IRQ_INTERRUPTS - 32);
        }
      else if (irq < NR_IRQS)
        {
           *regaddr = (NVIC_IRQ64_95_ENABLE + offset);
           *bit     = 1 << (irq - LC823450_IRQ_INTERRUPTS - 64);
        }
      else
        {
          return ERROR; /* Invalid interrupt */
        }
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
       *regaddr = NVIC_SYSHCON;
       if (irq == LC823450_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == LC823450_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == LC823450_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == LC823450_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return ERROR; /* Invalid or unsupported exception */
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
  int num_priority_registers;

  /* Disable all interrupts */

  putreg32(0, NVIC_IRQ0_31_ENABLE);
  putreg32(0, NVIC_IRQ32_63_ENABLE);

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
  {
    size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
    up_stack_color((FAR void *)((uintptr_t)&g_intstackbase - intstack_size),
                   intstack_size);
  }
#endif

  /* The standard location for the vector table is at the beginning of FLASH
   * at address 0x0800:0000.  If we are using the STMicro DFU bootloader, then
   * the vector table will be offset to a different location in FLASH and we
   * will need to set the NVIC vector location to this alternative location.
   *
   * If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

#if defined(CONFIG_ARCH_RAMVECTORS)
  up_ramvec_initialize();
#elif defined(CONFIG_LC823450_DFU)
  putreg32((uint32_t)_vectors, NVIC_VECTAB);
#endif

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  /* The NVIC ICTR register (bits 0-4) holds the number of of interrupt
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

  irq_attach(LC823450_IRQ_SVCALL, up_svcall, NULL);
  irq_attach(LC823450_IRQ_HARDFAULT, up_hardfault, NULL);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
  /* up_prioritize_irq(LC823450_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */
#endif
#ifdef CONFIG_ARMV7M_USEBASEPRI
   lc823450_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARMV7M_MPU
  irq_attach(LC823450_IRQ_MEMFAULT, up_memfault, NULL);
  up_enable_irq(LC823450_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG
  irq_attach(LC823450_IRQ_NMI, lc823450_nmi, NULL);
#ifndef CONFIG_ARMV7M_MPU
  irq_attach(LC823450_IRQ_MEMFAULT, up_memfault, NULL);
#endif
  irq_attach(LC823450_IRQ_BUSFAULT, lc823450_busfault, NULL);
  irq_attach(LC823450_IRQ_USAGEFAULT, lc823450_usagefault, NULL);
  irq_attach(LC823450_IRQ_PENDSV, lc823450_pendsv, NULL);
  irq_attach(LC823450_IRQ_DBGMONITOR, lc823450_dbgmonitor, NULL);
  irq_attach(LC823450_IRQ_RESERVED, lc823450_reserved, NULL);
#endif

  /* Initialize external interrupt. */

  lc823450_extint_initialize();

  lc823450_dumpnvic("initial", NR_IRQS);

#define NVIC_CFGCON_DIV_0_TRP (1 << 4)
  modifyreg32(NVIC_CFGCON, 0, NVIC_CFGCON_DIV_0_TRP);

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

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

#ifdef CONFIG_LC823450_VIRQ
  if (irq >= LC823450_IRQ_VIRTUAL &&
    irq < LC823450_IRQ_VIRTUAL + LC823450_IRQ_NVIRTUALIRQS)
    {
      struct lc823450_irq_ops *ops;

      ops = virq_ops[irq - LC823450_IRQ_VIRTUAL];
      if (ops && ops->disable)
        {
          ops->disable(irq);
        }

      return;
    }
#endif /* CONFIG_LC823450_VIRQ */

  if (lc823450_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= LC823450_IRQ_NIRQS)
        {
          regval  = getreg32(regaddr);
          regval |= bit;
          putreg32(regval, regaddr);
        }
      else if (irq >= LC823450_IRQ_INTERRUPTS)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval &= ~bit;
          putreg32(regval, regaddr);
        }
    }

  /* lc823450_dumpnvic("disable", irq); */
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
  irqstate_t flags;

#ifdef CONFIG_LC823450_VIRQ
  if (irq >= LC823450_IRQ_VIRTUAL &&
      irq < LC823450_IRQ_VIRTUAL + LC823450_IRQ_NVIRTUALIRQS)
    {
      struct lc823450_irq_ops *ops;

      ops = virq_ops[irq - LC823450_IRQ_VIRTUAL];
      if (ops && ops->enable)
        {
          ops->enable(irq);
        }

      return;
    }
#endif /* CONFIG_LC823450_VIRQ */

  if (lc823450_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      flags = spin_lock_irqsave();

      if (irq >= LC823450_IRQ_NIRQS)
        {
          /* Clear already asserted IRQ */

          lc823450_extint_clr(irq);

          regval  = getreg32(regaddr);
          regval &= ~bit;
          putreg32(regval, regaddr);
        }
      else if (irq >= LC823450_IRQ_INTERRUPTS)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval |= bit;
          putreg32(regval, regaddr);
        }

      spin_unlock_irqrestore(flags);
    }

  /* lc823450_dumpnvic("enable", irq); */
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
  if (irq < LC823450_IRQ_SYSTICK)
    {
      return;
    }

#ifdef CONFIG_DVFS
  lc823450_dvfs_exit_idle(irq);
#endif

  board_autoled_on(LED_CPU0 + up_cpu_index());

#ifdef CONFIG_SMP
  if (irq > LC823450_IRQ_LPDSP0 && 1 == up_cpu_index())
    {
      /* IRQ should be handled on CPU0 */

      DEBUGASSERT(false);
    }
#endif

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

  DEBUGASSERT(irq >= LC823450_IRQ_MEMFAULT && irq < NR_IRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < LC823450_IRQ_INTERRUPTS)
    {
      /* NVIC_SYSH_PRIORITY() maps {0..15} to one of three priority
       * registers (0-3 are invalid)
       */

      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq    -= 4;
    }
  else
    {
      /* NVIC_IRQ_PRIORITY() maps {0..} to one of many priority registers */

      irq    -= LC823450_IRQ_INTERRUPTS;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

  /* lc823450_dumpnvic("prioritize", irq); */
  return OK;
}
#endif

/****************************************************************************
 * Name: lc823450_irq_srctype
 *
 * Description:
 *   Set source type of external interrupt.
 *
 ****************************************************************************/

int lc823450_irq_srctype(int irq, enum lc823450_srctype_e srctype)
{
  irqstate_t flags;
  uint32_t regaddr;
  uint32_t regval;
  int port;
  int gpio;

#ifdef CONFIG_LC823450_VIRQ
  if (irq >= LC823450_IRQ_VIRTUAL &&
      irq < LC823450_IRQ_VIRTUAL + LC823450_IRQ_NVIRTUALIRQS)
    {
      struct lc823450_irq_ops *ops;

      ops = virq_ops[irq - LC823450_IRQ_VIRTUAL];
      if (ops && ops->srctype)
        {
          return ops->srctype(irq, srctype);
        }

      return OK;
    }
#endif /* CONFIG_LC823450_VIRQ */

  DEBUGASSERT(srctype < SRCTYPE_MAX);
  DEBUGASSERT(irq >= LC823450_IRQ_GPIO00 && irq <= LC823450_IRQ_GPIO59);

  irq -= LC823450_IRQ_GPIO00;

  port = (irq & 0x70) >> 4;
  gpio = irq & 0xf;

  flags = spin_lock_irqsave();

  regaddr = INTC_REG(EXTINTnCND_BASE, port);
  regval = getreg32(regaddr);

  regval &= ~(3 << gpio * 2);
  regval |= srctype << gpio * 2;

  putreg32(regval, regaddr);

  spin_unlock_irqrestore(flags);

  return OK;
}

/****************************************************************************
 * Name: lc823450_irq_register
 *
 * Description:
 *   Register IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_LC823450_VIRQ
int lc823450_irq_register(int irq, struct lc823450_irq_ops *ops)
{
  if (irq >= LC823450_IRQ_VIRTUAL &&
      irq < LC823450_IRQ_VIRTUAL + LC823450_IRQ_NVIRTUALIRQS)
    {
      irqstate_t flags;
      flags = irqsave();
      virq_ops[irq - LC823450_IRQ_VIRTUAL] = ops;
      irqrestore(flags);
    }
  else
    {
      return -1;
    }
  return OK;
}
#endif /* CONFIG_LC823450_VIRQ */
