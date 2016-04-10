/****************************************************************************
 * arch/arm/src/efm32/efm32_irq.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "nvic.h"
#include "ram_vectors.h"
#include "up_arch.h"
#include "sched/sched.h"
#include "up_internal.h"

#include "chip.h"
#include "efm32_gpio.h"

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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

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
 * Name: efm32_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ)
static void efm32_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();
  lldbg("NVIC (%s, irq=%d):\n", msg, irq);
  lldbg("  INTCTRL:    %08x VECTAB:  %08x\n",
        getreg32(NVIC_INTCTRL), getreg32(NVIC_VECTAB));
  lldbg("  SYSH ENABLE MEMFAULT: %08x BUSFAULT: %08x USGFAULT: %08x SYSTICK: %08x\n",
        getreg32(NVIC_SYSHCON_MEMFAULTENA), getreg32(NVIC_SYSHCON_BUSFAULTENA),
        getreg32(NVIC_SYSHCON_USGFAULTENA), getreg32(NVIC_SYSTICK_CTRL_ENABLE));
  lldbg("  IRQ ENABLE: %08x %08x %08x\n",
        getreg32(NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ32_63_ENABLE),
        getreg32(NVIC_IRQ64_95_ENABLE));
  lldbg("  SYSH_PRIO:  %08x %08x %08x\n",
        getreg32(NVIC_SYSH4_7_PRIORITY), getreg32(NVIC_SYSH8_11_PRIORITY),
        getreg32(NVIC_SYSH12_15_PRIORITY));
  lldbg("  IRQ PRIO:   %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ0_3_PRIORITY), getreg32(NVIC_IRQ4_7_PRIORITY),
        getreg32(NVIC_IRQ8_11_PRIORITY), getreg32(NVIC_IRQ12_15_PRIORITY));
  lldbg("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ16_19_PRIORITY), getreg32(NVIC_IRQ20_23_PRIORITY),
        getreg32(NVIC_IRQ24_27_PRIORITY), getreg32(NVIC_IRQ28_31_PRIORITY));
#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 32)
  lldbg("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ32_35_PRIORITY), getreg32(NVIC_IRQ36_39_PRIORITY),
        getreg32(NVIC_IRQ40_43_PRIORITY), getreg32(NVIC_IRQ44_47_PRIORITY));
#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 48)
  lldbg("              %08x %08x %08x %08x\n",
        getreg32(NVIC_IRQ48_51_PRIORITY), getreg32(NVIC_IRQ52_55_PRIORITY),
        getreg32(NVIC_IRQ56_59_PRIORITY), getreg32(NVIC_IRQ60_63_PRIORITY));
#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 64)
  lldbg("              %08x\n",
        getreg32(NVIC_IRQ64_67_PRIORITY));
#endif
#endif
#endif
  leave_critical_section(flags);
}
#else
#  define efm32_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: efm32_nmi, efm32_busfault, efm32_usagefault, efm32_pendsv,
 *       efm32_dbgmonitor, efm32_pendsv, efm32_reserved
 *
 * Description:
 *   Handlers for various exceptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
static int efm32_nmi(int irq, FAR void *context)
{
  (void)up_irq_save();
  dbg("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int efm32_busfault(int irq, FAR void *context)
{
  (void)up_irq_save();
  dbg("PANIC!!! Bus fault received: %08x\n", getreg32(NVIC_CFAULTS));
  PANIC();
  return 0;
}

static int efm32_usagefault(int irq, FAR void *context)
{
  (void)up_irq_save();
  dbg("PANIC!!! Usage fault received: %08x\n", getreg32(NVIC_CFAULTS));
  PANIC();
  return 0;
}

static int efm32_pendsv(int irq, FAR void *context)
{
  (void)up_irq_save();
  dbg("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int efm32_dbgmonitor(int irq, FAR void *context)
{
  (void)up_irq_save();
  dbg("PANIC!!! Debug Monitor received\n");
  PANIC();
  return 0;
}

static int efm32_reserved(int irq, FAR void *context)
{
  (void)up_irq_save();
  dbg("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: efm32_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_USEBASEPRI
static inline void efm32_prioritize_syscall(int priority)
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
 * Name: efm32_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int efm32_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                         uintptr_t offset)
{
  DEBUGASSERT(irq >= EFM32_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt or (a second level GPIO interrupt) */

  if (irq >= EFM32_IRQ_INTERRUPTS)
    {
      /* Is this an external interrupt? */

      if (irq < NR_VECTORS)
        {
          /* Yes.. We have support implemented for vectors 0-95 */

          DEBUGASSERT(irq < (EFM32_IRQ_INTERRUPTS + 96));

#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 32)
          /* Check for vectors 0-31 */

          if (irq < EFM32_IRQ_INTERRUPTS + 32)
#endif
            {
              *regaddr = (NVIC_IRQ0_31_ENABLE + offset);
              *bit     = 1 << (irq - EFM32_IRQ_INTERRUPTS);
            }
#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 32)
          /* Yes..  Check for vectors 32-63 */

          else
#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 64)
          if (irq < EFM32_IRQ_INTERRUPTS + 64)
#endif
            {
              *regaddr = (NVIC_IRQ32_63_ENABLE + offset);
              *bit     = 1 << (irq - EFM32_IRQ_INTERRUPTS - 32);
            }
#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 64)
          /* Yes..  Check for vectors 64-95 */

          else
#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 96)
          /* Yes..  Check for vectors 64-95 */

          if (irq < NR_VECTORS)
#endif
            {
              *regaddr = (NVIC_IRQ64_95_ENABLE + offset);
              *bit     = 1 << (irq - EFM32_IRQ_INTERRUPTS - 64);
            }
#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 96)
          else
            {
              return -EINVAL; /* We should never get here */
            }
#endif
#endif
#endif
        }
      else
        {
          return -EINVAL; /* Invalid interrupt */
        }
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == EFM32_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == EFM32_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == EFM32_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == EFM32_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return -EINVAL; /* Invalid or unsupported exception */
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
#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 32)
  putreg32(0, NVIC_IRQ32_63_ENABLE);
#if NR_VECTORS >= (EFM32_IRQ_INTERRUPTS + 64)
  putreg32(0, NVIC_IRQ64_95_ENABLE);
#endif
#endif

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
  /* Colorize the interrupt stack for debug purposes */

  {
    size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
    up_stack_color((FAR void *)((uintptr_t)&g_intstackbase - intstack_size),
                   intstack_size);
  }
#endif

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

  up_ramvec_initialize();
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

  irq_attach(EFM32_IRQ_SVCALL, up_svcall);
  irq_attach(EFM32_IRQ_HARDFAULT, up_hardfault);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARMV7M_USEBASEPRI
   efm32_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARM_MPU
  irq_attach(EFM32_IRQ_MEMFAULT, up_memfault);
  up_enable_irq(EFM32_IRQ_MEMFAULT);
#endif

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG
  irq_attach(EFM32_IRQ_NMI, efm32_nmi);
#ifndef CONFIG_ARM_MPU
  irq_attach(EFM32_IRQ_MEMFAULT, up_memfault);
#endif
  irq_attach(EFM32_IRQ_BUSFAULT, efm32_busfault);
  irq_attach(EFM32_IRQ_USAGEFAULT, efm32_usagefault);
  irq_attach(EFM32_IRQ_PENDSV, efm32_pendsv);
  irq_attach(EFM32_IRQ_DBGMONITOR, efm32_dbgmonitor);
  irq_attach(EFM32_IRQ_RESERVED, efm32_reserved);
#endif

  efm32_dumpnvic("initial", NR_VECTORS);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
#ifdef CONFIG_EFM32_GPIO_IRQ
  /* Initialize logic to support a second level of interrupt decoding for
   * GPIO pins.
   */

  efm32_gpioirqinitialize();
#endif

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

  if (efm32_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= EFM32_IRQ_INTERRUPTS)
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
#ifdef CONFIG_EFM32_GPIO_IRQ
  else
    {
      /* Maybe it is a (derived) GPIO IRQ */

      efm32_gpioirqdisable(irq);
    }
#endif

  efm32_dumpnvic("disable", irq);
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

  if (efm32_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      if (irq >= EFM32_IRQ_INTERRUPTS)
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
#ifdef CONFIG_EFM32_GPIO_IRQ
  else
    {
      /* Maybe it is a (derived) PIO IRQ */

      efm32_gpioirqenable(irq);
    }
#endif

  efm32_dumpnvic("enable", irq);
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

  DEBUGASSERT(irq >= EFM32_IRQ_MEMFAULT && irq < NR_VECTORS &&
              (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < EFM32_IRQ_INTERRUPTS)
    {
      /* NVIC_SYSH_PRIORITY() maps {0..15} to one of three priority
       * registers (0-3 are invalid)
       */

      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq    -= 4;
    }
  else (irq < NR_VECTORS)
    {
      /* NVIC_IRQ_PRIORITY() maps {0..} to one of many priority registers */

      irq    -= EFM32_IRQ_INTERRUPTS;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }
  else
    {
      /* Must be a GPIO interrupt */

      return -EINVAL;
    }

  regval      = getreg32(regaddr);
  shift       = ((irq & 3) << 3);
  regval     &= ~(0xff << shift);
  regval     |= (priority << shift);
  putreg32(regval, regaddr);

  efm32_dumpnvic("prioritize", irq);
  return OK;
}
#endif
