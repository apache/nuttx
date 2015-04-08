/****************************************************************************
 * arch/arm/src/lpc17/kinetis_irq.c
 *
 *   Copyright (C) 2011, 2013-14 Gregory Nutt. All rights reserved.
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

#include "nvic.h"
#include "ram_vectors.h"
#include "up_arch.h"
#include "up_internal.h"
#include "kinetis_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 |\
   NVIC_SYSH_PRIORITY_DEFAULT << 16 |\
   NVIC_SYSH_PRIORITY_DEFAULT << 8  |\
   NVIC_SYSH_PRIORITY_DEFAULT)

/* Given the address of a NVIC ENABLE register, this is the offset to
 * the corresponding CLEAR ENABLE register.
 */

#define NVIC_ENA_OFFSET    (0)
#define NVIC_CLRENA_OFFSET (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ)
static void kinetis_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = irqsave();
  lldbg("NVIC (%s, irq=%d):\n", msg, irq);
  lldbg("  INTCTRL:    %08x VECTAB: %08x\n",
        getreg32(NVIC_INTCTRL), getreg32(NVIC_VECTAB));
#if 0
  lldbg("  SYSH ENABLE MEMFAULT: %08x BUSFAULT: %08x USGFAULT: %08x SYSTICK: %08x\n",
        getreg32(NVIC_SYSHCON_MEMFAULTENA), getreg32(NVIC_SYSHCON_BUSFAULTENA),
        getreg32(NVIC_SYSHCON_USGFAULTENA), getreg32(NVIC_SYSTICK_CTRL_ENABLE));
#endif
  lldbg("  IRQ ENABLE: %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ32_63_ENABLE);
        getreg32(NVIC_IRQ64_95_ENABLE), getreg32(NVIC_IRQ96_127_ENABLE));
  lldbg("  SYSH_PRIO:  %08x %08x %08x\n",
        getreg32(NVIC_SYSH4_7_PRIORITY), getreg32(NVIC_SYSH8_11_PRIORITY),
        getreg32(NVIC_SYSH12_15_PRIORITY));
  lldbg("  IRQ PRIO:   %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ0_3_PRIORITY), getreg32(NVIC_IRQ4_7_PRIORITY),
        getreg32(NVIC_IRQ8_11_PRIORITY), getreg32(NVIC_IRQ12_15_PRIORITY));
  lldbg("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ16_19_PRIORITY), getreg32(NVIC_IRQ20_23_PRIORITY),
        getreg32(NVIC_IRQ24_27_PRIORITY), getreg32(NVIC_IRQ28_31_PRIORITY));
  lldbg("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ32_35_PRIORITY), getreg32(NVIC_IRQ36_39_PRIORITY),
        getreg32(NVIC_IRQ40_43_PRIORITY), getreg32(NVIC_IRQ44_47_PRIORITY));
  lldbg("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ48_51_PRIORITY), getreg32(NVIC_IRQ52_55_PRIORITY),
        getreg32(NVIC_IRQ56_59_PRIORITY), getreg32(NVIC_IRQ60_63_PRIORITY));
  lldbg("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ64_67_PRIORITY), getreg32(NVIC_IRQ68_71_PRIORITY),
        getreg32(NVIC_IRQ72_75_PRIORITY), getreg32(NVIC_IRQ76_79_PRIORITY));
  lldbg("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ80_83_PRIORITY), getreg32(NVIC_IRQ84_87_PRIORITY),
        getreg32(NVIC_IRQ88_91_PRIORITY), getreg32(NVIC_IRQ92_95_PRIORITY));
  lldbg("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ96_99_PRIORITY), getreg32(NVIC_IRQ100_103_PRIORITY),
        getreg32(NVIC_IRQ104_107_PRIORITY), getreg32(NVIC_IRQ108_111_PRIORITY));
#if NR_VECTORS > 111
  lldbg("              %08x %08x\n",
        getreg32(NVIC_IRQ112_115_PRIORITY), getreg32(NVIC_IRQ116_119_PRIORITY));
#endif

  irqrestore(flags);
}
#else
#  define kinetis_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: kinetis_nmi, kinetis_busfault, kinetis_usagefault, kinetis_pendsv,
 *       kinetis_dbgmonitor, kinetis_pendsv, kinetis_reserved
 *
 * Description:
 *   Handlers for various execptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
static int kinetis_nmi(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int kinetis_busfault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Bus fault recived\n");
  PANIC();
  return 0;
}

static int kinetis_usagefault(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Usage fault received\n");
  PANIC();
  return 0;
}

static int kinetis_pendsv(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int kinetis_dbgmonitor(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Debug Monitor receieved\n");
  PANIC();
  return 0;
}

static int kinetis_reserved(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: kinetis_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_USEBASEPRI
static inline void kinetis_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval  = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}
#endif

/****************************************************************************
 * Name: kinetis_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int kinetis_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                           uintptr_t offset)
{
  DEBUGASSERT(irq >= KINETIS_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= KINETIS_IRQ_EXTINT)
    {
      if (irq < (KINETIS_IRQ_EXTINT+32))
        {
           *regaddr = (NVIC_IRQ0_31_ENABLE + offset);
           *bit     = 1 << (irq - KINETIS_IRQ_EXTINT);
        }
      else if (irq < (KINETIS_IRQ_EXTINT+64))
        {
           *regaddr = (NVIC_IRQ32_63_ENABLE + offset);
           *bit     = 1 << (irq - KINETIS_IRQ_EXTINT - 32);
        }
      else if (irq < (KINETIS_IRQ_EXTINT+96))
        {
           *regaddr = (NVIC_IRQ64_95_ENABLE + offset);
           *bit     = 1 << (irq - KINETIS_IRQ_EXTINT - 64);
        }
      else if (irq < NR_IRQS)
        {
           *regaddr = (NVIC_IRQ96_127_ENABLE + offset);
           *bit     = 1 << (irq - KINETIS_IRQ_EXTINT - 96);
        }
      else
        {
          return ERROR; /* Invalid irq */
        }
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
       *regaddr = NVIC_SYSHCON;
       if (irq == KINETIS_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == KINETIS_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == KINETIS_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == KINETIS_IRQ_SYSTICK)
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
  uintptr_t regaddr;
  int nintlines;
  int i;

  /* The NVIC ICTR register (bits 0-4) holds the number of of interrupt
   * lines that the NVIC supports, defined in groups of 32. That is,
   * the total number of interrupt lines is up to (32*(INTLINESNUM+1)).
   *
   *  0 -> 32 interrupt lines, 1 enable register,   8 priority registers
   *  1 -> 64 "       " "   ", 2 enable registers, 16 priority registers
   *  2 -> 96 "       " "   ", 3 enable regsiters, 24 priority registers
   *  ...
   */

  nintlines = (getreg32(NVIC_ICTR) & NVIC_ICTR_INTLINESNUM_MASK) + 1;

  /* Disable all interrupts.  There are nintlines interrupt enable
   * registers.
   */

  for (i = nintlines, regaddr = NVIC_IRQ0_31_ENABLE;
       i > 0;
       i--, regaddr += 4)
    {
      putreg32(0, regaddr);
    }

  /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

#ifdef CONFIG_ARCH_RAMVECTORS
  up_ramvec_initialize();
#endif

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  /* Now set all of the interrupt lines to the default priority.  There are
   * nintlines * 8 priority registers.
   */

  for (i = (nintlines << 3), regaddr = NVIC_IRQ0_3_PRIORITY;
       i > 0;
       i--, regaddr += 4)
    {
      putreg32(DEFPRIORITY32, regaddr);
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(KINETIS_IRQ_SVCALL, up_svcall);
  irq_attach(KINETIS_IRQ_HARDFAULT, up_hardfault);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
/* up_prioritize_irq(KINETIS_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */
#endif
#ifdef CONFIG_ARMV7M_USEBASEPRI
   kinetis_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARMV7M_MPU
  irq_attach(KINETIS_IRQ_MEMFAULT, up_memfault);
  up_enable_irq(KINETIS_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG
  irq_attach(KINETIS_IRQ_NMI, kinetis_nmi);
#ifndef CONFIG_ARMV7M_MPU
  irq_attach(KINETIS_IRQ_MEMFAULT, up_memfault);
#endif
  irq_attach(KINETIS_IRQ_BUSFAULT, kinetis_busfault);
  irq_attach(KINETIS_IRQ_USAGEFAULT, kinetis_usagefault);
  irq_attach(KINETIS_IRQ_PENDSV, kinetis_pendsv);
  irq_attach(KINETIS_IRQ_DBGMONITOR, kinetis_dbgmonitor);
  irq_attach(KINETIS_IRQ_RESERVED, kinetis_reserved);
#endif

  kinetis_dumpnvic("initial", NR_IRQS);

  /* Initialize logic to support a second level of interrupt decoding for
   * configured pin interrupts.
   */

#ifdef CONFIG_GPIO_IRQ
  kinetis_pinirqinitialize();
#endif

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  irqenable();
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

  if (kinetis_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= KINETIS_IRQ_EXTINT)
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

  kinetis_dumpnvic("disable", irq);
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

  if (kinetis_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      if (irq >= KINETIS_IRQ_EXTINT)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval  = getreg32(regaddr);
          regval |= bit;
          putreg32(regval, regaddr);
        }
    }

  kinetis_dumpnvic("enable", irq);
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *  Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
#if 0 /* Does not appear to be necessary in most cases */
  kinetis_clrpend(irq);
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

  DEBUGASSERT(irq >= KINETIS_IRQ_MEMFAULT && irq < NR_IRQS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < KINETIS_IRQ_EXTINT)
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

      irq    -= KINETIS_IRQ_EXTINT;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

  kinetis_dumpnvic("prioritize", irq);
  return OK;
}
#endif
