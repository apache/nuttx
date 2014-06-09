/************************************************************************************
 * configs/sama5d3-xplained/src/sam_ethernet.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_SAMA5_EMAC
static xcpt g_emac_handler;
#endif
#ifdef CONFIG_SAMA5_GMAC
static xcpt g_gmac_handler;
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
#ifdef CONFIG_SAMA4_EMAC
  /* Ethernet 10/100 (EMAC) Port
   *
   * The main board contains a MICREL PHY device (KSZ8051) operating at 10/100 Mbps.
   * The board supports MII and RMII interface modes.
   *
   * The two independent PHY devices embedded on CM and MB boards are connected to
   * independent RJ-45 connectors with built-in magnetic and status LEDs.
   *
   * At the De-Assertion of Reset:
   *   PHY ADD[2:0]:001
   *   CONFIG[2:0]:001,Mode:RMII
   *   Duplex Mode:Half Duplex
   *   Isolate Mode:Disable
   *   Speed Mode:100Mbps
   *   Nway Auto-Negotiation:Enable
   *
   * The KSZ8051 PHY interrtup is available on PE30 INT_ETH1
  */

  sam_configpio(PIO_INT_ETH1);
#endif

#ifdef CONFIG_SAMA4_GMAC
  /* Tri-Speed Ethernet PHY
   *
   * The SAMA5D3 series-CM board is equipped with a MICREL PHY devices (MICREL
   * KSZ9021/31) operating at 10/100/1000 Mbps. The board supports RGMII interface
   * mode. The Ethernet interface consists of 4 pairs of low voltage differential
   * pair signals designated from GRX± and GTx± plus control signals for link
   * activity indicators. These signals can be used to connect to a 10/100/1000
   * BaseT RJ45 connector integrated on the main board.
   *
   * The KSZ9021/31 interrupt is available on PB35 INT_GETH0
   */

  sam_configpio(PIO_INT_ETH0);
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
  int irq;

#ifdef CONFIG_SAMA5_EMAC
  if (intf == EMAC_INTF)
    {
      handler = &g_emac_handler;
      irq     = IRQ_INT_ETH1;
    }
  else
#endif
#ifdef CONFIG_SAMA5_GMAC
  if (intf == GMAC_INTF)
    {
      handler = &g_gmac_handler;
      irq     = IRQ_INT_ETH0;
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

  sam_pioirq(irq);
  (void)irq_attach(irq, irqhandler);
  sam_pioirqenable(irq);

  /* Return the old button handler (so that it can be restored) */

  irqrestore(flags);
  return oldhandler;
}
#endif /* CONFIG_SAMA5_PIOE_IRQ */

#endif /* HAVE_NETWORK */
