/****************************************************************************
 * arch/arm/src/armv7-a/arm_gicv2_dump.c
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

#include "arm_arch.h"
#include "gic.h"

#if defined(CONFIG_ARMV7A_HAVE_GICv2) && defined(CONFIG_DEBUG_IRQ_INFO)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_gic_dump_cpu
 *
 * Description:
 *   Dump CPU interface registers.
 *
 * Input Parameters:
 *   all - True: Dump all IRQs; False: Dump only registers for this IRQ
 *   irq - if all == false, then dump only this IRQ.
 *   nlines - Number of interrupts to dump if all == true;
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void arm_gic_dump_cpu(bool all, int irq, int nlines)
{
  irqinfo("  CPU Interface Registers:\n");
  irqinfo("       ICR: %08x    PMR: %08x    BPR: %08x    IAR: %08x\n",
          getreg32(GIC_ICCICR), getreg32(GIC_ICCPMR),
          getreg32(GIC_ICCBPR), getreg32(GIC_ICCIAR));
  irqinfo("       RPR: %08x   HPIR: %08x   ABPR: %08x\n",
          getreg32(GIC_ICCRPR), getreg32(GIC_ICCHPIR),
          getreg32(GIC_ICCABPR));
  irqinfo("      AIAR: %08x  AHPIR: %08x    IDR: %08x\n",
          getreg32(GIC_ICCAIAR), getreg32(GIC_ICCAHPIR),
          getreg32(GIC_ICCIDR));
  irqinfo("      APR1: %08x   APR2: %08x   APR3: %08x   APR4: %08x\n",
          getreg32(GIC_ICCAPR1), getreg32(GIC_ICCAPR2),
          getreg32(GIC_ICCAPR3), getreg32(GIC_ICCAPR4));
  irqinfo("    NSAPR1: %08x NSAPR2: %08x NSAPR3: %08x NSAPR4: %08x\n",
          getreg32(GIC_ICCNSAPR1), getreg32(GIC_ICCNSAPR2),
          getreg32(GIC_ICCNSAPR3), getreg32(GIC_ICCNSAPR4));
}

/****************************************************************************
 * Name: arm_gic_dumpregs
 *
 * Description:
 *   Dump registers with 4x8-bit per interrupt
 *
 * Input Parameters:
 *   regaddr - First register address
 *   nlines  - Number of interrupt lines
 *   incr    - IRQs per 32-bit word
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void arm_gic_dumpregs(uintptr_t regaddr, int nlines, int incr)
{
  unsigned int i;

  incr <<= 2;
  for (i = 0; i < nlines; i += incr, regaddr += 16)
    {
      irqinfo("         %08x %08x %08x %08x\n",
              getreg32(regaddr), getreg32(regaddr + 4),
              getreg32(regaddr + 8), getreg32(regaddr + 12));
    }
}

/****************************************************************************
 * Name: arm_gic_dump4
 *
 * Description:
 *   Dump registers with 4x8-bit per interrupt
 *
 * Input Parameters:
 *   name    - Register name
 *   regaddr - First register address
 *   nlines  - Number of interrupt lines
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void arm_gic_dump4(const char *name, uintptr_t regaddr,
                                 int nlines)
{
  irqinfo("       %s[%08lx]\n", name, (unsigned long)regaddr);
  arm_gic_dumpregs(regaddr, nlines, 4);
}

/****************************************************************************
 * Name: arm_gic_dump8
 *
 * Description:
 *   Dump registers with 8x4-bit per interrupt
 *
 * Input Parameters:
 *   name    - Register name
 *   regaddr - First register address
 *   nlines  - Number of interrupt lines
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void arm_gic_dump8(const char *name, uintptr_t regaddr,
                                 int nlines)
{
  irqinfo("       %s[%08lx]\n", name, (unsigned long)regaddr);
  arm_gic_dumpregs(regaddr, nlines, 8);
}

/****************************************************************************
 * Name: arm_gic_dump16
 *
 * Description:
 *   Dump registers with 16 x 2-bit per interrupt
 *
 * Input Parameters:
 *   name    - Register name
 *   regaddr - First register address
 *   nlines  - Number of interrupt lines
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void arm_gic_dump16(const char *name, uintptr_t regaddr,
                                  int nlines)
{
  irqinfo("       %s[%08lx]\n", name, (unsigned long)regaddr);
  arm_gic_dumpregs(regaddr, nlines, 16);
}

/****************************************************************************
 * Name: arm_gic_dump32
 *
 * Description:
 *   Dump registers with 32 x 1-bit per interrupt
 *
 * Input Parameters:
 *   name    - Register name
 *   regaddr - First register address
 *   nlines  - Number of interrupt lines
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void arm_gic_dump32(const char *name, uintptr_t regaddr,
                                  int nlines)
{
  irqinfo("       %s[%08lx]\n", name, (unsigned long)regaddr);
  arm_gic_dumpregs(regaddr, nlines, 32);
}

/****************************************************************************
 * Name: arm_gic_dump_distributor
 *
 * Description:
 *   Dump distributor interface registers.
 *
 * Input Parameters:
 *   all    - True: Dump all IRQs; False: Dump only registers for this IRQ
 *   irq    - if all == false, then dump only this IRQ.
 *   nlines - Number of interrupts to dump if all == true;
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void arm_gic_dump_distributor(bool all, int irq, int nlines)
{
  irqinfo("  Distributor Registers:\n");
  irqinfo("       DCR: %08x   ICTR: %08x   IIDR: %08x\n",
          getreg32(GIC_ICDDCR), getreg32(GIC_ICDICTR),
          getreg32(GIC_ICDIIDR));

  if (all)
    {
      arm_gic_dump32("ISR",  GIC_ICDISR(0),  nlines);
      arm_gic_dump32("ISER/ICER", GIC_ICDISER(0), nlines);
      arm_gic_dump32("ISPR/ICPR", GIC_ICDISPR(0), nlines);
      arm_gic_dump32("SAR/CAR", GIC_ICDSAR(0), nlines);
      arm_gic_dump4("IPR", GIC_ICDIPR(0), nlines);
      arm_gic_dump4("IPTR", GIC_ICDIPTR(0), nlines);
      arm_gic_dump16("ICFR", GIC_ICDICFR(0), nlines);
      arm_gic_dump32("PPSIR/SPISR", GIC_ICDPPISR, nlines);
      arm_gic_dump32("NSACR", GIC_ICDNSACR(0), nlines);
      arm_gic_dump8("SCPR/SSPR", GIC_ICDSCPR(0), nlines);
    }
  else
    {
      irqinfo("       ISR: %08x   ISER: %08x   ISPR: %08x    SAR: %08x\n",
              getreg32(GIC_ICDISR(irq)), getreg32(GIC_ICDISER(irq)),
              getreg32(GIC_ICDISPR(irq)), getreg32(GIC_ICDSAR(irq)));
      irqinfo("       IPR: %08x   IPTR: %08x   ICFR: %08x  SPISR: %08x\n",
              getreg32(GIC_ICDIPR(irq)), getreg32(GIC_ICDIPTR(irq)),
              getreg32(GIC_ICDICFR(irq)), getreg32(GIC_ICDSPISR(irq)));
      irqinfo("     NSACR: %08x   SCPR: %08x\n",
              getreg32(GIC_ICDNSACR(irq)), getreg32(GIC_ICDSCPR(irq)));
    }

  irqinfo("       PIDR[%08lx]:\n", (unsigned long)GIC_ICDPIDR(0));
  irqinfo("         %08x %08x %08x %08x\n",
          getreg32(GIC_ICDPIDR(0)), getreg32(GIC_ICDPIDR(1)),
          getreg32(GIC_ICDPIDR(2)), getreg32(GIC_ICDPIDR(3)));
  irqinfo("         %08x %08x %08x %08x\n",
          getreg32(GIC_ICDPIDR(4)), getreg32(GIC_ICDPIDR(5)),
          getreg32(GIC_ICDPIDR(6)));
  irqinfo("       CIDR[%08lx]:\n", (unsigned long)GIC_ICDCIDR(0));
  irqinfo("         %08x %08x %08x %08x\n",
          getreg32(GIC_ICDCIDR(0)), getreg32(GIC_ICDCIDR(1)),
          getreg32(GIC_ICDCIDR(2)), getreg32(GIC_ICDCIDR(3)));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_gic_dump
 *
 * Description:
 *   Dump GIC registers to the SYSLOG device
 *
 * Input Parameters:
 *   msg - Message to print with the register data
 *   all - True: Dump all IRQs; False: Dump only registers for this IRQ
 *   irq - if all == false, then dump only this IRQ.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void arm_gic_dump(const char *msg, bool all, int irq)
{
  unsigned int nlines = arm_gic_nlines();

  if (all)
    {
      irqinfo("GIC: %s NLINES=%u\n", msg, nlines);
    }
  else
    {
      irqinfo("GIC: %s IRQ=%d\n", msg, irq);
    }

  arm_gic_dump_cpu(all, irq, nlines);
  arm_gic_dump_distributor(all, irq, nlines);
}

#endif /* CONFIG_ARMV7A_HAVE_GICv2 && CONFIG_DEBUG_IRQ_INFO */
