/****************************************************************************
 * arch/arm/src/lm3s/lm3s_ethernet.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#if defined(CONFIG_NET) && defined(CONFIG_LM3S_ETHERNET)

#include <time.h>
#include <string.h>
#include <debug.h>
#include <wdog.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <net/uip/uip.h>
#include <net/uip/uip-arp.h>
#include <net/uip/uip-arch.h>

#include "lm3s_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define LM3S_WDDELAY   (1*CLK_TCK)
#define LM3S_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define LM3S_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)skel->ld_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lm3s_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct lm3s_driver_s
{
  boolean ld_bifup;            /* TRUE:ifup FALSE:ifdown */
  WDOG_ID ld_txpoll;           /* TX poll timer */
  WDOG_ID ld_txtimeout;        /* TX timeout timer */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s ld_dev;  /* Interface understood by uIP */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lm3s_driver_s g_lm3sdev;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  lm3s_transmit(struct lm3s_driver_s *skel);
static int  lm3s_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void lm3s_receive(struct lm3s_driver_s *skel);
static void lm3s_txdone(struct lm3s_driver_s *skel);
static int  lm3s_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void lm3s_polltimer(int argc, uint32 arg, ...);
static void lm3s_txtimeout(int argc, uint32 arg, ...);

/* NuttX callback functions */

static int lm3s_ifup(struct uip_driver_s *dev);
static int lm3s_ifdown(struct uip_driver_s *dev);
static int lm3s_txavail(struct uip_driver_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: lm3s_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   skel  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lm3s_transmit(struct lm3s_driver_s *skel)
{
  /* Verify that the hardware is ready to send another packet */
#warning "Missing logic"
  /* Increment statistics */

  /* Disable Ethernet interrupts */

  /* Send the packet: address=skel->ld_dev.d_buf, length=skel->ld_dev.d_len */

  /* Restore Ethernet interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(skel->ld_txtimeout, LM3S_TXTIMEOUT, lm3s_txtimeout, 1, (uint32)skel);
  return OK;
}

/****************************************************************************
 * Function: lm3s_uiptxpoll
 *
 * Description:
 *   The transmitter is available, check if uIP has any outgoing packets ready
 *   to send.  This is a callback from uip_poll().  uip_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lm3s_uiptxpoll(struct uip_driver_s *dev)
{
  struct lm3s_driver_s *skel = (struct lm3s_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (skel->ld_dev.d_len > 0)
    {
      uip_arp_out(&skel->ld_dev);
      lm3s_transmit(skel);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */
#warning "Missing logic"
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: lm3s_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   skel  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void lm3s_receive(struct lm3s_driver_s *skel)
{
  do
    {
      /* Check for errors and update statistics */
#warning "Missing logic"

      /* Check if the packet is a valid size for the uIP buffer configuration */

      /* Copy the data data from the hardware to skel->ld_dev.d_buf.  Set
       * amount of data in skel->ld_dev.d_len
       */

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          uip_arp_ipin();
          uip_input(&skel->ld_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (skel->ld_dev.d_len > 0)
           {
             uip_arp_out(&skel->ld_dev);
             lm3s_transmit(skel);
           }
         }
       else if (BUF->type == htons(UIP_ETHTYPE_ARP))
         {
           uip_arp_arpin(&skel->ld_dev);

           /* If the above function invocation resulted in data that should be
            * sent out on the network, the field  d_len will set to a value > 0.
            */

            if (skel->ld_dev.d_len > 0)
              {
                lm3s_transmit(skel);
              }
          }
      }
    }
  while (); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: lm3s_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   skel  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void lm3s_txdone(struct lm3s_driver_s *skel)
{
  /* Check for errors and update statistics */
#warning "Missing logic"

  /* If no further xmits are pending, then cancel the TX timeout */

  wd_cancel(skel->ld_txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&skel->ld_dev, lm3s_uiptxpoll);
}

/****************************************************************************
 * Function: lm3s_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lm3s_interrupt(int irq, FAR void *context)
{
  register struct lm3s_driver_s *skel = &g_lm3sdev;

  /* Disable Ethernet interrupts */
#warning "Missing logic"

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call lm3s_receive() */

  lm3s_receive(skel);

  /* Check is a packet transmission just completed.  If so, call lm3s_txdone */

  lm3s_txdone(skel);

  /* Enable Ethernet interrupts (perhaps excluding the TX done interrupt if 
   * there are no pending transmissions.
   */

  return OK;
}

/****************************************************************************
 * Function: lm3s_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void lm3s_txtimeout(int argc, uint32 arg, ...)
{
  struct lm3s_driver_s *skel = (struct lm3s_driver_s *)arg;

  /* Increment statistics and dump debug info */
#warning "Missing logic"

  /* Then reset the hardware */

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&skel->ld_dev, lm3s_uiptxpoll);
}

/****************************************************************************
 * Function: lm3s_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void lm3s_polltimer(int argc, uint32 arg, ...)
{
  struct lm3s_driver_s *skel = (struct lm3s_driver_s *)arg;

  /* Check if there is room in the send another Tx packet.  */
#warning "Missing logic"

  /* If so, update TCP timing states and poll uIP for new XMIT data */

  (void)uip_timer(&skel->ld_dev, lm3s_uiptxpoll, LM3S_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(skel->ld_txpoll, LM3S_WDDELAY, lm3s_polltimer, 1, arg);
}

/****************************************************************************
 * Function: lm3s_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided 
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lm3s_ifup(struct uip_driver_s *dev)
{
  struct lm3s_driver_s *skel = (struct lm3s_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initilize Ethernet interface */
#warning "Missing logic"

  /* Set and activate a timer process */

  (void)wd_start(skel->ld_txpoll, LM3S_WDDELAY, lm3s_polltimer, 1, (uint32)skel);

  /* Enable the Ethernet interrupt */

  skel->ld_bifup = TRUE;
  up_enable_irq(CONFIG_LM3S_IRQ);
  return OK;
}

/****************************************************************************
 * Function: lm3s_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lm3s_ifdown(struct uip_driver_s *dev)
{
  struct lm3s_driver_s *skel = (struct lm3s_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  up_disable_irq(CONFIG_LM3S_IRQ);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(skel->ld_txpoll);
  wd_cancel(skel->ld_txtimeout);

  /* Reset the device */

  skel->ld_bifup = FALSE;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: lm3s_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a 
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int lm3s_txavail(struct uip_driver_s *dev)
{
  struct lm3s_driver_s *skel = (struct lm3s_driver_s *)dev->d_private;
  irqstate_t flags;

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (skel->ld_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */
#warning "Missing logic"

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&skel->ld_dev, lm3s_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: lm3s_initialize
 *
 * Description:
 *   Initialize the Ethernet driver
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

/* Initialize the Ethernet controller and driver */

int lm3s_initialize(void)
{
  /* Check if the Ethernet module is present */

  DEBUGASSERT((getreg32(LM3S_SYSCON_DC4) & (SYSCON_DC4_EMAC0|SYSCON_DC4_EPHY0)) == (SYSCON_DC4_EMAC0|SYSCON_DC4_EPHY0));

  /* Enable Port F for Ethernet LEDs: LED0=Bit 3; LED1=Bit 2 */

#ifdef CONFIG_LM3S_ETHLEDS
  /* Configure the pins for the peripheral function */

  lm3s_configgpio(GPIO_ETHPHY_LED0 | GPIO_STRENGTH_2MA | GPIO_PADTYPE_STD);
  lm3s_configgpio(GPIO_ETHPHY_LED1 | GPIO_STRENGTH_2MA | GPIO_PADTYPE_STD);
#endif

#warning "Missing logic"

  /* Attach the IRQ to the driver */

  if (irq_attach(CONFIG_LM3S_IRQ, lm3s_interrupt))
    {
      /* We could not attach the ISR to the ISR */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(g_lm3sdev, 0, sizeof(struct lm3s_driver_s));
  g_lm3sdev.ld_dev.d_ifup    = lm3s_ifup;     /* I/F down callback */
  g_lm3sdev.ld_dev.d_ifdown  = lm3s_ifdown;   /* I/F up (new IP address) callback */
  g_lm3sdev.ld_dev.d_txavail = lm3s_txavail;  /* New TX data callback */
  g_lm3sdev.ld_dev.d_private = (void*)g_lm3sdev; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  g_lm3sdev.ld_txpoll       = wd_create();   /* Create periodic poll timer */
  g_lm3sdev.ld_txtimeout    = wd_create();   /* Create TX timeout timer */

  /* Read the MAC address from the hardware into g_lm3sdev.ld_dev.d_mac.ether_addr_octet */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&g_lm3sdev.ld_dev);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_LM3S_ETHERNET */

