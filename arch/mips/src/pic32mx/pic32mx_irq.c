/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_irq.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/irq.h>
#include <arch/pic32mx/cp0.h>

#include "mips_internal.h"
#include "pic32mx_int.h"
#include "pic32mx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_PIC32MX_MVEC
#  error "Multi-vectors not supported"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_ARCH_IRQPRIO
static int up_prioritize_irq(int irq, int priority);
#endif

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

  putreg32(0xffff, PIC32MX_INT_IEC0CLR);
  putreg32(0xffff, PIC32MX_INT_IEC1CLR);
#ifdef PIC32MX_INT_IEC2CLR
  putreg32(0xffff, PIC32MX_INT_IEC2CLR);
#endif

  /* Set all interrupts to the default (middle) priority */

  for (irq = 0; irq < NR_IRQS; irq++)
    {
      up_prioritize_irq(irq, (INT_IPC_MID_PRIORITY << 2));
    }

  /* Set the Software Interrupt0 to a special priority */

  up_prioritize_irq(PIC32MX_IRQ_CS0, (CHIP_SW0_PRIORITY << 2));

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

#ifdef CONFIG_PIC32MX_MVEC
  putreg32(INT_INTCON_MVEC, PIC32MX_INT_INTCONSET);
#else
  putreg32(INT_INTCON_MVEC, PIC32MX_INT_INTCONCLR);
#endif

  /* Initialize GPIO change notification handling */

#ifdef CONFIG_PIC32MX_GPIOIRQ
  pic32mx_gpioirqinitialize();
#endif

  /* Attach and enable software interrupts */

  irq_attach(PIC32MX_IRQ_CS0, mips_swint0, NULL);
  up_enable_irq(PIC32MX_IRQSRC_CS0);

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

  DEBUGASSERT(irq >= PIC32MX_IRQSRC_FIRST && irq <= PIC32MX_IRQSRC_LAST);
  if (irq >= PIC32MX_IRQSRC_FIRST)
    {
      if (irq <= PIC32MX_IRQSRC0_LAST)
        {
          /* Use IEC0 */

          regaddr = PIC32MX_INT_IEC0CLR;
          bitno   = irq - PIC32MX_IRQSRC0_FIRST;
        }
      else if (irq <= PIC32MX_IRQSRC1_LAST)
        {
          /* Use IEC1 */

          regaddr = PIC32MX_INT_IEC1CLR;
          bitno   = irq - PIC32MX_IRQSRC1_FIRST;
        }
#ifdef PIC32MX_IRQSRC2_FIRST
      else if (irq <= PIC32MX_IRQSRC2_LAST)
        {
          /* Use IEC2 */

          regaddr = PIC32MX_INT_IEC2CLR;
          bitno   = irq - PIC32MX_IRQSRC2_FIRST;
        }
#endif
      else
        {
          /* Value out of range.. just ignore */

          return;
        }

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

  DEBUGASSERT(irq >= PIC32MX_IRQSRC_FIRST && irq <= PIC32MX_IRQSRC_LAST);
  if (irq >= PIC32MX_IRQSRC_FIRST)
    {
      if (irq <= PIC32MX_IRQSRC0_LAST)
        {
          /* Use IEC0 */

          regaddr = PIC32MX_INT_IEC0SET;
          bitno   = irq - PIC32MX_IRQSRC0_FIRST;
        }
      else if (irq <= PIC32MX_IRQSRC1_LAST)
        {
          /* Use IEC1 */

          regaddr = PIC32MX_INT_IEC1SET;
          bitno   = irq - PIC32MX_IRQSRC1_FIRST;
        }
#ifdef PIC32MX_IRQSRC2_FIRST
      else if (irq <= PIC32MX_IRQSRC2_LAST)
        {
          /* Use IEC2 */

          regaddr = PIC32MX_INT_IEC2SET;
          bitno   = irq - PIC32MX_IRQSRC2_FIRST;
        }
#endif
      else
        {
          /* Value out of range.. just ignore */

          return;
        }

      /* Disable the interrupt */

      putreg32((1 << bitno), regaddr);
    }
}

/****************************************************************************
 * Name: mips_pending_irq
 *
 * Description:
 *   Return true if the interrupt is pending and unmasked.
 *
 ****************************************************************************/

bool mips_pending_irq(int irq)
{
  uint32_t ifsaddr;
  uint32_t iecaddr;
  uint32_t regval;
  int bitno;

  /* Test if the interrupt is pending by reading both the IEC and IFS
   * register. Return true if the bit associated with the irq is both pending
   * the IFs and enabled in the IEC.
   */

  DEBUGASSERT(irq >= PIC32MX_IRQSRC_FIRST && irq <= PIC32MX_IRQSRC_LAST);
  if (irq >= PIC32MX_IRQSRC_FIRST)
    {
      if (irq <= PIC32MX_IRQSRC0_LAST)
        {
          /* Use IFS0 */

          ifsaddr = PIC32MX_INT_IFS0;
          iecaddr = PIC32MX_INT_IEC0;
          bitno   = irq - PIC32MX_IRQSRC0_FIRST;
        }
      else if (irq <= PIC32MX_IRQSRC1_LAST)
        {
          /* Use IFS1 */

          ifsaddr = PIC32MX_INT_IFS1;
          iecaddr = PIC32MX_INT_IEC1;
          bitno   = irq - PIC32MX_IRQSRC1_FIRST;
        }
#ifdef PIC32MX_IRQSRC2_FIRST
      else if (irq <= PIC32MX_IRQSRC2_LAST)
        {
          /* Use IFS2 */

          ifsaddr = PIC32MX_INT_IFS2;
          iecaddr = PIC32MX_INT_IEC2;
          bitno   = irq - PIC32MX_IRQSRC2_FIRST;
        }
#endif
      else
        {
          /* Value out of range.. just ignore */

          return false;
        }

      /* Get the set of unmasked, pending interrupts.  Return true if the
       * interrupt is pending and unmask.
       */

      regval = getreg32(ifsaddr) & getreg32(iecaddr);
      return (regval & (1 << bitno)) != 0;
    }

  return false;
}

/****************************************************************************
 * Name: mips_clrpend_irq
 *
 * Description:
 *   Clear any pending interrupt
 *
 ****************************************************************************/

void mips_clrpend_irq(int irq)
{
  uint32_t regaddr;
  int bitno;

  /* Acknowledge the interrupt by clearing the associated bit in the IFS
   * register.  It is necessary to do this BEFORE lowering the interrupt
   * priority level otherwise recursive interrupts would occur.
   */

  DEBUGASSERT(irq >= PIC32MX_IRQSRC_FIRST && irq <= PIC32MX_IRQSRC_LAST);
  if (irq >= PIC32MX_IRQSRC_FIRST)
    {
      if (irq <= PIC32MX_IRQSRC0_LAST)
        {
          /* Use IFS0 */

          regaddr = PIC32MX_INT_IFS0CLR;
          bitno   = irq - PIC32MX_IRQSRC0_FIRST;
        }
      else if (irq <= PIC32MX_IRQSRC1_LAST)
        {
          /* Use IFS1 */

          regaddr = PIC32MX_INT_IFS1CLR;
          bitno   = irq - PIC32MX_IRQSRC1_FIRST;
        }
#ifdef PIC32MX_IRQSRC2_FIRST
      else if (irq <= PIC32MX_IRQSRC2_LAST)
        {
          /* Use IFS2 */

          regaddr = PIC32MX_INT_IFS2CLR;
          bitno   = irq - PIC32MX_IRQSRC2_FIRST;
        }
#endif
      else
        {
          /* Value out of range.. just ignore */

          return;
        }

      /* Acknowledge the interrupt */

      putreg32((1 << bitno), regaddr);
    }
}

/****************************************************************************
 * Name: mips_clrpend_sw0
 *
 * Description:
 *   Clear a pending Software Interrupt.
 *
 ****************************************************************************/

void mips_clrpend_sw0(void)
{
  mips_clrpend_irq(PIC32MX_IRQSRC_CS0);
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ by setting the priority and sub-priority
 *   fields in the PIC32MX IPC registers.  There are 12 IPC registers, IPC0
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
static
#endif
int up_prioritize_irq(int irq, int priority)
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

      putreg32(0x1f << shift, PIC32MX_INT_IPCCLR(regndx));
      putreg32(priority << shift, PIC32MX_INT_IPCSET(regndx));
      return OK;
    }

  return -EINVAL;
}
