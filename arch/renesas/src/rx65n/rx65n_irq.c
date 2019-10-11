/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_irq.c
 *
 *   Copyright (C) 2008-2019 Gregory Nutt. All rights reserved.
 *   Author: Anjana <anjana@tataelxsi.co.in> 
 *          Surya <surya.prakash@tataelxsi.co.in>
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include "rx65n/iodefine.h"

#include "up_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This holds a references to the current interrupt level register storage
 * structure.  If is non-NULL only during interrupt processing.
 */

/* Actually a pointer to the beginning of a uint8_t array */

volatile uint32_t *g_current_regs;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Currents_regs is non-NULL only while processing an interrupt */

  g_current_regs = NULL;

  /* Enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  if (irq == RX65N_CMI0_IRQ)
    {
      ICU.IER[3].BIT.IEN4 = 0;
    }

#ifdef CONFIG_RX65N_SCI0
  if (irq == RX65N_RXI0_IRQ)
    {
      ICU.IER[7].BIT.IEN2 = 0;
    }

  if (irq == RX65N_TXI0_IRQ)
    {
      ICU.IER[7].BIT.IEN3 = 0;
    }

  if (irq == RX65N_ERI0_IRQ)
    {
      ICU.GRPBL0.BIT.IS1 = 0;
      ICU.GENBL0.BIT.EN1 = 0;
    }

  if (irq == RX65N_TEI0_IRQ)
    {
      ICU.GRPBL0.BIT.IS0 = 0;
      ICU.GENBL0.BIT.EN0 = 0;
    }
#endif

#ifdef CONFIG_RX65N_SCI1
  if (irq == RX65N_RXI1_IRQ)
    {
      ICU.IER[7].BIT.IEN4 = 0;
    }

  if (irq == RX65N_TXI1_IRQ)
    {
       ICU.IER[7].BIT.IEN5 = 0;
    }

  if (irq == RX65N_ERI1_IRQ)
    {
       ICU.GRPBL0.BIT.IS3 = 0;
       ICU.GENBL0.BIT.EN3 = 0;
    }

  if (irq == RX65N_TEI1_IRQ)
    {
       ICU.GRPBL0.BIT.IS2 = 0;
       ICU.GENBL0.BIT.EN2 = 0;
    }
#endif

#ifdef CONFIG_RX65N_SCI2
  if (irq == RX65N_RXI2_IRQ)
    {
       ICU.IER[7].BIT.IEN6 = 0;
    }

  if (irq == RX65N_TXI2_IRQ)
    {
       ICU.IER[7].BIT.IEN7 = 0;
    }

  if (irq == RX65N_ERI2_IRQ)
    {
       ICU.GRPBL0.BIT.IS5 = 0;
       ICU.GENBL0.BIT.EN5 = 0;
    }

  if (irq == RX65N_TEI2_IRQ)
    {
       ICU.GRPBL0.BIT.IS4 = 0;
       ICU.GENBL0.BIT.EN4 = 0;
    }
#endif
#ifdef CONFIG_RX65N_SCI3
  if (irq == RX65N_RXI3_IRQ)
    {
       ICU.IER[10].BIT.IEN0 = 0;
    }

  if (irq == RX65N_TXI3_IRQ)
    {
       ICU.IER[10].BIT.IEN1 = 0;
    }

  if (irq == RX65N_ERI3_IRQ)
    {
       ICU.GRPBL0.BIT.IS7 = 0;
       ICU.GENBL0.BIT.EN7 = 0;
    }

  if (irq == RX65N_TEI3_IRQ)
    {
       ICU.GRPBL0.BIT.IS6 = 0;
       ICU.GENBL0.BIT.EN6 = 0;
    }
#endif

#ifdef CONFIG_RX65N_SCI4
  if (irq == RX65N_RXI4_IRQ)
    {
       ICU.IER[10].BIT.IEN2 = 0;
    }

  if (irq == RX65N_TXI4_IRQ)
    {
       ICU.IER[10].BIT.IEN3 = 0;
    }

  if (irq == RX65N_ERI4_IRQ)
    {
       ICU.GRPBL0.BIT.IS9 = 0;
       ICU.GENBL0.BIT.EN9 = 0;
    }

  if (irq == RX65N_TEI4_IRQ)
    {
       ICU.GRPBL0.BIT.IS8 = 0;
       ICU.GENBL0.BIT.EN8 = 0;
    }
#endif

#ifdef CONFIG_RX65N_SCI5
  if (irq == RX65N_RXI5_IRQ)
    {
       ICU.IER[10].BIT.IEN4 = 0;
    }

  if (irq == RX65N_TXI5_IRQ)
    {
       ICU.IER[10].BIT.IEN5 = 0;
    }

  if (irq == RX65N_ERI5_IRQ)
    {
       ICU.GRPBL0.BIT.IS11 = 0;
       ICU.GENBL0.BIT.EN11 = 0;
    }

  if (irq == RX65N_TEI5_IRQ)
    {
       ICU.GRPBL0.BIT.IS10 = 0;
       ICU.GENBL0.BIT.EN10 = 0;
    }
#endif
#ifdef CONFIG_RX65N_SCI6
  if (irq == RX65N_RXI6_IRQ)
    {
      ICU.IER[10].BIT.IEN6 = 0;
    }

  if (irq == RX65N_TXI6_IRQ)
    {
       ICU.IER[10].BIT.IEN7 = 0;
    }

  if (irq == RX65N_ERI6_IRQ)
    {
       ICU.GRPBL0.BIT.IS13 = 0;
       ICU.GENBL0.BIT.EN13 = 0;
    }

  if (irq == RX65N_TEI6_IRQ)
    {
       ICU.GRPBL0.BIT.IS12 = 0;
       ICU.GENBL0.BIT.EN12 = 0;
    }
#endif

#ifdef CONFIG_RX65N_SCI7
  if (irq == RX65N_RXI7_IRQ)
    {
       ICU.IER[12].BIT.IEN2 = 0;
    }

  if (irq == RX65N_TXI7_IRQ)
    {
       ICU.IER[12].BIT.IEN3 = 0;
    }

  if (irq == RX65N_ERI7_IRQ)
    {
       ICU.GRPBL0.BIT.IS15 = 0;
       ICU.GENBL0.BIT.EN15 = 0;
    }

  if (irq == RX65N_TEI7_IRQ)
    {
       ICU.GRPBL0.BIT.IS14 = 0;
       ICU.GENBL0.BIT.EN14 = 0;
    }
#endif

#ifdef CONFIG_RX65N_SCI8
  if (irq == RX65N_RXI8_IRQ)
    {
       ICU.IER[12].BIT.IEN4 = 0;
    }

  if (irq == RX65N_TXI8_IRQ)
    {
       ICU.IER[12].BIT.IEN5 = 0;
    }

  if (irq == RX65N_ERI8_IRQ)
    {
       ICU.GRPBL1.BIT.IS25 = 0;
       ICU.GENBL1.BIT.EN25 = 0;
    }

  if (irq == RX65N_TEI8_IRQ)
    {
       ICU.GRPBL1.BIT.IS24 = 0;
       ICU.GENBL1.BIT.EN24 = 0;
    }
#endif

#ifdef CONFIG_RX65N_SCI9
  if (irq == RX65N_RXI9_IRQ)
    {
       ICU.IER[12].BIT.IEN6 = 0;
    }

  if (irq == RX65N_TXI9_IRQ)
    {
       ICU.IER[12].BIT.IEN7 = 0;
    }

  if (irq == RX65N_ERI9_IRQ)
    {
       ICU.GRPBL1.BIT.IS27 = 0;
       ICU.GENBL1.BIT.EN27 = 0;
    }

  if (irq == RX65N_TEI9_IRQ)
    {
       ICU.GRPBL1.BIT.IS26 = 0;
       ICU.GENBL1.BIT.EN26 = 0;
    }
#endif

#ifdef CONFIG_RX65N_SCI10
  if (irq == RX65N_RXI10_IRQ)
    {
      ICU.IER[10].BIT.IEN0 = 0;
    }

  if (irq == RX65N_TXI10_IRQ)
    {
      ICU.IER[10].BIT.IEN1 = 0;
    }

  if (irq == RX65N_ERI10_IRQ)
    {
      ICU.GRPAL0.BIT.IS9 = 0;
      ICU.GENAL0.BIT.EN9 = 0;
    }

  if (irq == RX65N_TEI10_IRQ)
    {
      ICU.GRPAL0.BIT.IS8 = 0;
      ICU.GENAL0.BIT.EN8 = 0;
    }
#endif

#ifdef CONFIG_RX65N_SCI11
  if (irq == RX65N_RXI11_IRQ)
    {
       ICU.IER[14].BIT.IEN2 = 0;
    }

  if (irq == RX65N_TXI11_IRQ)
    {
      ICU.IER[14].BIT.IEN3 = 0;
    }

  if (irq == RX65N_ERI11_IRQ)
    {
      ICU.GRPAL0.BIT.IS13 = 0;
      ICU.GENAL0.BIT.EN13 = 0;
    }

  if (irq == RX65N_TEI11_IRQ)
    {
      ICU.GRPAL0.BIT.IS12 = 0;
      ICU.GENAL0.BIT.EN12 = 0;
    }
#endif

#ifdef CONFIG_RX65N_SCI12
  if (irq == RX65N_RXI12_IRQ)
    {
      ICU.IER[14].BIT.IEN4 = 0;
    }

  if (irq == RX65N_TXI12_IRQ)
    {
      ICU.IER[14].BIT.IEN5 = 0;
    }

  if (irq == RX65N_ERI12_IRQ)
    {
      ICU.GRPBL0.BIT.IS17 = 0;
      ICU.GENBL0.BIT.EN17 = 0;
    }

  if (irq == RX65N_TEI12_IRQ)
    {
      ICU.GRPBL0.BIT.IS16 = 0;
      ICU.GENBL0.BIT.EN16 = 0;
    }
#endif

#ifdef CONFIG_RX65N_EMAC
  if (irq == RX65N_ETH_IRQ)
    {
       ICU.GRPAL1.BIT.IS4 = 0;
       ICU.GENAL1.BIT.EN4 = 0;
    }
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  if (irq == RX65N_CMI0_IRQ)
    {
      ICU.IER[3].BIT.IEN4 = 1;
    }

 #ifdef CONFIG_RX65N_SCI0
  if (irq == RX65N_RXI0_IRQ)
    {
      ICU.IER[7].BIT.IEN2 = 1;
    }

  if (irq == RX65N_TXI0_IRQ)
    {
      ICU.IER[7].BIT.IEN3 = 1;
    }

  if (irq == RX65N_ERI0_IRQ)
    {
      ICU.GRPBL0.BIT.IS1 = 1;
      ICU.GENBL0.BIT.EN1 = 1;
    }

  if (irq == RX65N_TEI0_IRQ)
    {
      ICU.GRPBL0.BIT.IS0 = 1;
      ICU.GENBL0.BIT.EN0 = 1;
    }
#endif

#ifdef CONFIG_RX65N_SCI1
  if (irq == RX65N_RXI1_IRQ)
    {
      ICU.IER[7].BIT.IEN4 = 1;
    }

  if (irq == RX65N_TXI1_IRQ)
    {
      ICU.IER[7].BIT.IEN5 = 1;
    }

  if (irq == RX65N_ERI1_IRQ)
    {
      ICU.GRPBL0.BIT.IS3 = 1;
      ICU.GENBL0.BIT.EN3 = 1;
    }

  if (irq == RX65N_TEI1_IRQ)
    {
      ICU.GRPBL0.BIT.IS2 = 1;
      ICU.GENBL0.BIT.EN2 = 1;
    }
#endif

#ifdef CONFIG_RX65N_SCI2
  if (irq == RX65N_RXI2_IRQ)
    {
      ICU.IER[7].BIT.IEN6 = 1;
    }

  if (irq == RX65N_TXI2_IRQ)
    {
      ICU.IER[7].BIT.IEN7 = 1;
    }

  if (irq == RX65N_ERI2_IRQ)
    {
      ICU.GRPBL0.BIT.IS5 = 1;
      ICU.GENBL0.BIT.EN5 = 1;
    }

  if (irq == RX65N_TEI2_IRQ)
    {
      ICU.GRPBL0.BIT.IS4 = 1;
      ICU.GENBL0.BIT.EN4 = 1;
    }
#endif
#ifdef CONFIG_RX65N_SCI3
  if (irq == RX65N_RXI3_IRQ)
    {
      ICU.IER[10].BIT.IEN0 = 1;
    }

  if (irq == RX65N_TXI3_IRQ)
    {
      ICU.IER[10].BIT.IEN1 = 1;
    }

  if (irq == RX65N_ERI3_IRQ)
    {
      ICU.GRPBL0.BIT.IS7 = 1;
      ICU.GENBL0.BIT.EN7 = 1;
    }

  if (irq == RX65N_TEI3_IRQ)
    {
      ICU.GRPBL0.BIT.IS6 = 1;
      ICU.GENBL0.BIT.EN6 = 1;
    }
#endif

#ifdef CONFIG_RX65N_SCI4
  if (irq == RX65N_RXI4_IRQ)
    {
      ICU.IER[10].BIT.IEN2 = 1;
    }

  if (irq == RX65N_TXI4_IRQ)
    {
      ICU.IER[10].BIT.IEN3 = 1;
    }

  if (irq == RX65N_ERI4_IRQ)
    {
      ICU.GRPBL0.BIT.IS9 = 1;
      ICU.GENBL0.BIT.EN9 = 1;
    }

  if (irq == RX65N_TEI4_IRQ)
    {
      ICU.GRPBL0.BIT.IS8 = 1;
      ICU.GENBL0.BIT.EN8 = 1;
    }
#endif

#ifdef CONFIG_RX65N_SCI5
  if (irq == RX65N_RXI5_IRQ)
    {
      ICU.IER[10].BIT.IEN4 = 1;
    }

  if (irq == RX65N_TXI5_IRQ)
    {
      ICU.IER[10].BIT.IEN5 = 1;
    }

  if (irq == RX65N_ERI5_IRQ)
    {
      ICU.GRPBL0.BIT.IS11 = 1;
      ICU.GENBL0.BIT.EN11 = 1;
    }

  if (irq == RX65N_TEI5_IRQ)
    {
      ICU.GRPBL0.BIT.IS10 = 1;
      ICU.GENBL0.BIT.EN10 = 1;
    }
#endif
#ifdef CONFIG_RX65N_SCI6
  if (irq == RX65N_RXI6_IRQ)
    {
      ICU.IER[10].BIT.IEN6 = 1;
    }

  if (irq == RX65N_TXI6_IRQ)
    {
      ICU.IER[10].BIT.IEN7 = 1;
    }

  if (irq == RX65N_ERI6_IRQ)
    {
      ICU.GRPBL0.BIT.IS13 = 1;
      ICU.GENBL0.BIT.EN13 = 1;
    }

  if (irq == RX65N_TEI6_IRQ)
    {
      ICU.GRPBL0.BIT.IS12 = 1;
      ICU.GENBL0.BIT.EN12 = 1;
    }
#endif

#ifdef CONFIG_RX65N_SCI7
  if (irq == RX65N_RXI7_IRQ)
    {
      ICU.IER[12].BIT.IEN2 = 1;
    }

  if (irq == RX65N_TXI7_IRQ)
    {
      ICU.IER[12].BIT.IEN3 = 1;
    }

  if (irq == RX65N_ERI7_IRQ)
    {
      ICU.GRPBL0.BIT.IS15 = 1;
      ICU.GENBL0.BIT.EN15 = 1;
    }

  if (irq == RX65N_TEI7_IRQ)
    {
      ICU.GRPBL0.BIT.IS14 = 1;
      ICU.GENBL0.BIT.EN14 = 1;
    }
#endif

#ifdef CONFIG_RX65N_SCI8
  if (irq == RX65N_RXI8_IRQ)
    {
      ICU.IER[12].BIT.IEN4 = 1;
    }

  if (irq == RX65N_TXI8_IRQ)
    {
      ICU.IER[12].BIT.IEN5 = 1;
    }

  if (irq == RX65N_ERI8_IRQ)
    {
      ICU.GRPBL1.BIT.IS25 = 1;
      ICU.GENBL1.BIT.EN25 = 1;
    }

  if (irq == RX65N_TEI8_IRQ)
    {
      ICU.GRPBL1.BIT.IS24 = 1;
      ICU.GENBL1.BIT.EN24 = 1;
    }
#endif

#ifdef CONFIG_RX65N_SCI9
  if (irq == RX65N_RXI9_IRQ)
    {
      ICU.IER[12].BIT.IEN6 = 1;
    }

  if (irq == RX65N_TXI9_IRQ)
    {
      ICU.IER[12].BIT.IEN7 = 1;
    }

  if (irq == RX65N_ERI9_IRQ)
    {
      ICU.GRPBL1.BIT.IS27 = 1;
      ICU.GENBL1.BIT.EN27 = 1;
    }

  if (irq == RX65N_TEI9_IRQ)
    {
      ICU.GRPBL1.BIT.IS26 = 1;
      ICU.GENBL1.BIT.EN26 = 1;
    }
#endif

#ifdef CONFIG_RX65N_SCI10
  if (irq == RX65N_RXI10_IRQ)
    {
      ICU.IER[10].BIT.IEN0 = 1;
    }

  if (irq == RX65N_TXI10_IRQ)
    {
      ICU.IER[10].BIT.IEN1 = 1;
    }

  if (irq == RX65N_ERI10_IRQ)
    {
      ICU.GRPAL0.BIT.IS9 = 1;
      ICU.GENAL0.BIT.EN9 = 1;
    }

  if (irq == RX65N_TEI10_IRQ)
    {
      ICU.GRPAL0.BIT.IS8 = 1;
      ICU.GENAL0.BIT.EN8 = 1;
    }
#endif

#ifdef CONFIG_RX65N_SCI11
  if (irq == RX65N_RXI11_IRQ)
    {
      ICU.IER[14].BIT.IEN2 = 1;
    }

  if (irq == RX65N_TXI11_IRQ)
    {
      ICU.IER[14].BIT.IEN3 = 1;
    }

  if (irq == RX65N_ERI11_IRQ)
    {
      ICU.GRPAL0.BIT.IS13 = 1;
      ICU.GENAL0.BIT.EN13 = 1;
    }

  if (irq == RX65N_TEI11_IRQ)
    {
      ICU.GRPAL0.BIT.IS12 = 1;
      ICU.GENAL0.BIT.EN12 = 1;
    }
#endif

#ifdef CONFIG_RX65N_SCI12
  if (irq == RX65N_RXI12_IRQ)
    {
      ICU.IER[14].BIT.IEN4 = 1;
    }

  if (irq == RX65N_TXI12_IRQ)
    {
      ICU.IER[14].BIT.IEN5 = 1;
    }

  if (irq == RX65N_ERI12_IRQ)
    {
      ICU.GRPBL0.BIT.IS17 = 1;
      ICU.GENBL0.BIT.EN17 = 1;
    }

  if (irq == RX65N_TEI12_IRQ)
    {
      ICU.GRPBL0.BIT.IS16 = 1;
      ICU.GENBL0.BIT.EN16 = 1;
    }
#endif

#ifdef CONFIG_RX65N_EMAC
  if (irq == RX65N_ETH_IRQ)
    {
      ICU.GRPAL1.BIT.IS4 = 1;
      ICU.GENAL1.BIT.EN4 = 1;
    }
#endif
}
