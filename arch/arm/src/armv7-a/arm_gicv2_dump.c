/****************************************************************************
 * arch/arm/src/armv7-a/arm_gicv2_dump.c
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

#include <stdint.h>
#include <syslog.h>
#include <debug.h>

#include "arm_internal.h"
#include "gic.h"

#if defined(CONFIG_ARMV7A_HAVE_GICv2) && defined(CONFIG_ARMV7A_GICv2_DUMP)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  define gicdump(fmt, ...) syslog(LOG_ALERT, fmt, ##__VA_ARGS__)
#else
#  define gicdump(fmt, ...)
#endif

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
  gicdump("  CPU Interface Registers:\n");
  gicdump("       ICR: %08lx    PMR: %08lx    BPR: %08lx    IAR: %08lx\n",
          getreg32(GIC_ICCICR), getreg32(GIC_ICCPMR),
          getreg32(GIC_ICCBPR), getreg32(GIC_ICCIAR));
  gicdump("       RPR: %08lx   HPIR: %08lx   ABPR: %08lx\n",
          getreg32(GIC_ICCRPR), getreg32(GIC_ICCHPIR),
          getreg32(GIC_ICCABPR));
  gicdump("      AIAR: %08lx  AHPIR: %08lx    IDR: %08lx\n",
          getreg32(GIC_ICCAIAR), getreg32(GIC_ICCAHPIR),
          getreg32(GIC_ICCIDR));
  gicdump("      APR1: %08lx   APR2: %08lx   APR3: %08lx   APR4: %08lx\n",
          getreg32(GIC_ICCAPR1), getreg32(GIC_ICCAPR2),
          getreg32(GIC_ICCAPR3), getreg32(GIC_ICCAPR4));
  gicdump("    NSAPR1: %08lx NSAPR2: %08lx NSAPR3: %08lx NSAPR4: %08lx\n",
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
      gicdump("         %08lx %08lx %08lx %08lx\n",
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
  gicdump("       %s[%08lx]\n", name, (unsigned long)regaddr);
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
  gicdump("       %s[%08lx]\n", name, (unsigned long)regaddr);
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
  gicdump("       %s[%08lx]\n", name, (unsigned long)regaddr);
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
  gicdump("       %s[%08lx]\n", name, (unsigned long)regaddr);
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
  gicdump("  Distributor Registers:\n");
  gicdump("       DCR: %08lx   ICTR: %08lx   IIDR: %08lx\n",
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
      arm_gic_dump16("NSACR", GIC_ICDNSACR(0), nlines);
      arm_gic_dump8("SCPR/SSPR", GIC_ICDSCPR(0), nlines);
    }
  else
    {
      gicdump("       ISR: %08lx   ISER: %08lx   ISPR: %08lx"
              "    SAR: %08lx\n",
              getreg32(GIC_ICDISR(irq)), getreg32(GIC_ICDISER(irq)),
              getreg32(GIC_ICDISPR(irq)), getreg32(GIC_ICDSAR(irq)));
      gicdump("       IPR: %08lx   IPTR: %08lx   ICFR: %08lx"
              "  SPISR: %08lx\n",
              getreg32(GIC_ICDIPR(irq)), getreg32(GIC_ICDIPTR(irq)),
              getreg32(GIC_ICDICFR(irq)), getreg32(GIC_ICDSPISR(irq)));
      gicdump("     NSACR: %08lx   SCPR: %08lx\n",
              getreg32(GIC_ICDNSACR(irq)), getreg32(GIC_ICDSCPR(irq)));
    }

  gicdump("       PIDR[%08lx]:\n", (unsigned long)GIC_ICDPIDR(0));
  gicdump("         %08lx %08lx %08lx %08lx\n",
          getreg32(GIC_ICDPIDR(0)), getreg32(GIC_ICDPIDR(1)),
          getreg32(GIC_ICDPIDR(2)), getreg32(GIC_ICDPIDR(3)));
  gicdump("         %08lx %08lx %08lx\n",
          getreg32(GIC_ICDPIDR(4)), getreg32(GIC_ICDPIDR(5)),
          getreg32(GIC_ICDPIDR(6)));
  gicdump("       CIDR[%08lx]:\n", (unsigned long)GIC_ICDCIDR(0));
  gicdump("         %08lx %08lx %08lx %08lx\n",
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
      gicdump("GIC: %s NLINES=%u\n", msg, nlines);
    }
  else
    {
      gicdump("GIC: %s IRQ=%d\n", msg, irq);
    }

  arm_gic_dump_cpu(all, irq, nlines);
  arm_gic_dump_distributor(all, irq, nlines);
}

#endif /* CONFIG_ARMV7A_HAVE_GICv2 && CONFIG_ARMV7A_GICv2_DUMP */
