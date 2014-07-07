/************************************************************************************
 * configs/sama5d4-ek/src/sam_ethernet.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>

#include "sam_pio.h"
#include "sam_ethernet.h"

#ifdef HAVE_NETWORK

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_SAMA5_PIOE_IRQ
#ifdef CONFIG_SAMA5_EMAC0
static xcpt g_emac0_handler;
#endif
#ifdef CONFIG_SAMA5_EMAC1
static xcpt g_emac1_handler;
#endif
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ************************************************************************************/

void weak_function sam_netinitialize(void)
{
#ifdef CONFIG_SAMA4_EMAC0
  sam_configpio(PIO_INT_ETH0);
#endif

#ifdef CONFIG_SAMA4_EMAC1
  sam_configpio(PIO_INT_ETH1);
#endif
}

/************************************************************************************
 * Name: sam_phyirq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will be
 *   called when an interrupt is received from a PHY.
 *
 ************************************************************************************/

#ifdef CONFIG_SAMA5_PIOE_IRQ
xcpt_t sam_phyirq(int intf, xcpt_t irqhandler)
{
  irqstate_t flags;
  xcpt_t *handler;
  xcpt_t oldhandler;
  pio_pinset_t pinset;
  int irq;

#ifdef CONFIG_SAMA5_EMAC0
  if (intf == EMAC0_INTF)
    {
      handler = &g_emac0_handler;
      pinset  = PIO_INT_ETH0;
      irq     = IRQ_INT_ETH0;
    }
  else
#endif
#ifdef CONFIG_SAMA5_EMAC1
  if (intf == EMAC1_INTF)
    {
      handler = &g_emac1_handler;
      pinset  = PIO_INT_ETH1;
      irq     = IRQ_INT_ETH1;
    }
  else
#endif
    {
      ndbg("Unsupported interface: %d\n", intf);
      return NULL;
    }

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = irqsave();

  /* Get the old button interrupt handler and save the new one */

  oldhandler = *handler;
  *handler = irqhandler;

  /* Configure the interrupt */

  sam_pioirq(pinset);
  (void)irq_attach(irq, irqhandler);
  sam_pioirqenable(irq);

  /* Return the old button handler (so that it can be restored) */

  irqrestore(flags);
  return oldhandler;
}
#endif /* CONFIG_SAMA5_PIOE_IRQ */

#endif /* HAVE_NETWORK */
