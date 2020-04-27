/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_irq.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <arch/irq.h>
#include <arch/pic32mz/cp0.h>

#include "mips_arch.h"
#include "mips_internal.h"

#include "hardware/pic32mz_int.h"
#include "pic32mz_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_PIC32MZ_MVEC
#  error "Multi-vectors not supported"
#endif

/* Interrupt controller definitions *****************************************/

/* Number of interrupt enable/interrupt status registers */

#define INT_NREGS ((NR_IRQS + 31) >> 5)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs holds a references to the current interrupt level
 * register storage structure.  It is non-NULL only during interrupt
 * processing.  Access to g_current_regs must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_prioritize_irq
 ****************************************************************************/

#ifndef CONFIG_ARCH_IRQPRIO
static int pic32mz_prioritize_irq(int irq, int priority);
#else
#  define pic32mz_prioritize_irq(i,p) up_prioritize_irq(i,p)
#endif

/****************************************************************************
 * Name: pic32mz_ifs
 ****************************************************************************/

static uintptr_t pic32mz_ifs(int irq)
{
  if ((unsigned)irq < NR_IRQS)
    {
      return PIC32MZ_INT_IFS(irq >> 5);
    }

  return 0;
}

/****************************************************************************
 * Name: pic32mz_ifsclr
 ****************************************************************************/

static uintptr_t pic32mz_ifsclr(int irq)
{
  if ((unsigned)irq < NR_IRQS)
    {
      return PIC32MZ_INT_IFSCLR(irq >> 5);
    }

  return 0;
}

/****************************************************************************
 * Name: pic32mz_iec
 ****************************************************************************/

static uintptr_t pic32mz_iec(int irq)
{
  if ((unsigned)irq < NR_IRQS)
    {
      return PIC32MZ_INT_IEC(irq >> 5);
    }

  return 0;
}

/****************************************************************************
 * Name: pic32mz_iecset
 ****************************************************************************/

static uintptr_t pic32mz_iecset(int irq)
{
  if ((unsigned)irq < NR_IRQS)
    {
      return PIC32MZ_INT_IECSET(irq >> 5);
    }

  return 0;
}

/****************************************************************************
 * Name: pic32mz_iecclr
 ****************************************************************************/

static uintptr_t pic32mz_iecclr(int irq)
{
  if ((unsigned)irq < NR_IRQS)
    {
      return PIC32MZ_INT_IECCLR(irq >> 5);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regval;
  int irq;

  /* Disable all interrupts */

  putreg32(0xffff, PIC32MZ_INT_IEC0CLR); /* Interrupts 0-31 */
  putreg32(0xffff, PIC32MZ_INT_IEC1CLR); /* Interrupts 32-63 */
  putreg32(0xffff, PIC32MZ_INT_IEC2CLR); /* Interrupts 64-95 */
  putreg32(0xffff, PIC32MZ_INT_IEC3CLR); /* Interrupts 96-127 */
  putreg32(0xffff, PIC32MZ_INT_IEC4CLR); /* Interrupts 128-159 */
  putreg32(0xffff, PIC32MZ_INT_IEC5CLR); /* Interrupts 160-191 */

  /* Set all interrupts to the default (middle) priority */

  for (irq = 0; irq < NR_IRQS; irq++)
    {
      pic32mz_prioritize_irq(irq, (INT_IPC_MID_PRIORITY << 2));
    }

  /* Set the Software Interrupt0 to a special priority */

  pic32mz_prioritize_irq(PIC32MZ_IRQ_CS0, (CHIP_SW0_PRIORITY << 2));

  /* Set the BEV bit in the STATUS register */

  regval  = cp0_getstatus();
  regval |= CP0_STATUS_BEV;
  cp0_putstatus(regval);

  /* Set the EBASE value to the beginning of boot FLASH.  In single-vector
   * mode, interrupt vectors should go to EBASE + 0x0200 0r 0xbfc00200.
   */

  cp0_putebase(0xbfc00000);

  /* Set the INTCTL vector spacing to non-zero */

  cp0_putintctl(0x00000020);

  /* Set the IV bit in the CAUSE register */

  regval  = cp0_getcause();
  regval |= CP0_CAUSE_IV;
  cp0_putcause(regval);

  /* Clear the EXL and BEV bits in the STATUS register */

  regval  = cp0_getstatus();
  regval &= ~(CP0_STATUS_EXL | CP0_STATUS_BEV);
  cp0_putstatus(regval);

  /* Configure multi- or single- vector interrupts */

#ifdef CONFIG_PIC32MZ_MVEC
  putreg32(INT_INTCON_MVEC, PIC32MZ_INT_INTCONSET);
#else
  putreg32(INT_INTCON_MVEC, PIC32MZ_INT_INTCONCLR);
#endif

  /* Initialize GPIO change notification handling */

#ifdef CONFIG_PIC32MZ_GPIOIRQ
  pic32mz_gpioirqinitialize();
#endif

  /* Attach and enable software interrupts */

  irq_attach(PIC32MZ_IRQ_CS0, up_swint0, NULL);
  up_enable_irq(PIC32MZ_IRQ_CS0);

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* And finally, enable interrupts */

  /* Interrupts are enabled by setting the IE bit in the CP0 status
   * register
   */

  regval = 0;
  asm volatile("ei    %0" : "=r"(regval));

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Then enable all interrupt levels */

  up_irq_restore(CP0_STATUS_INT_ENALL);
#else
  /* Enable only software interrupts */

  up_irq_restore(CP0_STATUS_INT_SW0);
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
  uint32_t regaddr;
  int bitno;

  /* Disable the interrupt by clearing the associated bit in the IEC
   * register
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);
  regaddr = pic32mz_iecclr(irq);
  bitno   = (unsigned)irq & 31;

  DEBUGASSERT(regaddr);
  if (regaddr)
    {
      /* Disable the interrupt */

      putreg32((1 << bitno), regaddr);
    }
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
  uint32_t regaddr;
  int bitno;

  /* Enable the interrupt by setting the associated bit in the IEC register */

  DEBUGASSERT((unsigned)irq < NR_IRQS);
  regaddr = pic32mz_iecset(irq);
  bitno   = (unsigned)irq & 31;

  DEBUGASSERT(regaddr);
  if (regaddr)
    {
      /* Disable the interrupt */

      putreg32((1 << bitno), regaddr);
    }
}

/****************************************************************************
 * Name: up_pending_irq
 *
 * Description:
 *   Return true if the interrupt is pending and unmasked.
 *
 ****************************************************************************/

bool up_pending_irq(int irq)
{
  uintptr_t ifsaddr;
  uintptr_t iecaddr;
  uint32_t regval;
  int bitno;

  /* Test if the interrupt is pending by reading both the IEC and IFS
   * register. Return true if the bit associated with the irq is both pending
   * the IFs and enabled in the IEC.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);
  ifsaddr = pic32mz_ifs(irq);
  iecaddr = pic32mz_iec(irq);
  bitno   = (unsigned)irq & 31;

  DEBUGASSERT(ifsaddr && iecaddr);
  if (ifsaddr && iecaddr)
    {
      /* Get the set of unmasked, pending interrupts.  Return true if the
       * interrupt is pending and unmask.
       */

      regval = getreg32(ifsaddr) & getreg32(iecaddr);
      return (regval & (1 << bitno)) != 0;
    }

  return false;
}

/****************************************************************************
 * Name: up_clrpend_irq
 *
 * Description:
 *   Clear any pending interrupt
 *
 ****************************************************************************/

void up_clrpend_irq(int irq)
{
  uintptr_t regaddr;
  int bitno;

  /* Acknowledge the interrupt by clearing the associated bit in the IFS
   * register.  It is necessary to do this BEFORE lowering the interrupt
   * priority level otherwise recursive interrupts would occur.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);
  regaddr = pic32mz_ifsclr(irq);
  bitno   = (unsigned)irq & 31;

  DEBUGASSERT(regaddr);
  if (regaddr)
    {
      /* Acknowledge the interrupt */

      putreg32((1 << bitno), regaddr);
    }
}

/****************************************************************************
 * Name: up_clrpend_sw0
 *
 * Description:
 *   Clear a pending Software Interrupt.
 *
 ****************************************************************************/

void up_clrpend_sw0(void)
{
  up_clrpend_irq(PIC32MZ_IRQ_CS0);
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ by setting the priority and sub-priority
 *   fields in the PIC32MZ IPC registers.  There are 12 IPC registers, IPC0
 *   through IPC11.  Each has sub-priority fields for 8 interrupts for a
 *   total of 96 interrupts max.
 *
 *   Each interrupt priority is represent by a group of 5 bits: a 3-bit
 *   priority and a 2-bit sub-priority.  These have different meanings to
 *   the hardware.  The priority is the priority level that is enabled
 *   or masked by the IPL field of the CAUSE register.  The sub-priority
 *   only mediates ties when two interrupts with the same priority pend
 *   simultaneously.
 *
 *   In this function, we just treat this as a single 5-bit priority.
 *   (MS 3-bits=priority; LS 2-bits=sub-priority).
 *
 *   The 5-bit priority/sub-priority fields are arranged at byte boundaries
 *   within each IPC register:
 *
 *     xxxP PPSS xxxP PPSS xxxP PPSS xxxP PPSS
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_IRQPRIO
static int pic32mz_prioritize_irq(int irq, int priority)
#else
int up_prioritize_irq(int irq, int priority)
#endif
{
  int regndx;
  int shift;

  /* Don't allow this function to be used for disabling interrupts.  There is
   * no good reason for this restriction other than I want to make sure that
   * the 5-bit priority values passed to this function are *not* confused
   * with the 3-bit hardware priority values.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS && (unsigned)(priority >> 2) > 0);
  if (irq < NR_IRQS)
    {
      /* Get the index to the IPC register and the shift to the 5-bit
       * priority field for this IRQ.
       */

      regndx = irq >> 2;       /* Range: 0-11 */
      shift  = (irq & 3) << 3; /* {0, 8, 16, 24 } */

      /* Set the new interrupt priority (momentarily disabling interrupts) */

      putreg32(0x1f << shift, PIC32MZ_INT_IPCCLR(regndx));
      putreg32(priority << shift, PIC32MZ_INT_IPCSET(regndx));
      return OK;
    }

  return -EINVAL;
}
