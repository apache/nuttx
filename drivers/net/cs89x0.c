/****************************************************************************
 * drivers/net/cs89x0.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_CS89x0)

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

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* CONFIG_CS89x0_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_CS89x0_NINTERFACES
# define CONFIG_CS89x0_NINTERFACES 1
#endif

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define CS89x0_WDDELAY   (1*CLK_TCK)
#define CS89x0_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define CS89x0_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)cs89x0->sk_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The cs89x0_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct cs89x0_driver_s
{
  boolean sk_bifup;            /* TRUE:ifup FALSE:ifdown */
  WDOG_ID sk_txpoll;           /* TX poll timer */
  WDOG_ID sk_txtimeout;        /* TX timeout timer */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s sk_dev;  /* Interface understood by uIP */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cs89x0_driver_s g_cs89x0[CONFIG_CS89x0_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  cs89x0_transmit(struct cs89x0_driver_s *cs89x0);
static int  cs89x0_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static void cs89x0_receive(struct cs89x0_driver_s *cs89x0);
static void cs89x0_txdone(struct cs89x0_driver_s *cs89x0);
static int  cs89x0_interrupt(int irq, FAR void *context);

/* Watchdog timer expirations */

static void cs89x0_polltimer(int argc, uint32 arg, ...);
static void cs89x0_txtimeout(int argc, uint32 arg, ...);

/* NuttX callback functions */

static int cs89x0_ifup(struct uip_driver_s *dev);
static int cs89x0_ifdown(struct uip_driver_s *dev);
static int cs89x0_txavail(struct uip_driver_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: cs89x0_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   cs89x0  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int cs89x0_transmit(struct cs89x0_driver_s *cs89x0)
{
  /* Verify that the hardware is ready to send another packet */

  /* Increment statistics */

  /* Disable Ethernet interrupts */

  /* Send the packet: address=cs89x0->sk_dev.d_buf, length=cs89x0->sk_dev.d_len */

  /* Restore Ethernet interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(cs89x0->sk_txtimeout, CS89x0_TXTIMEOUT, cs89x0_txtimeout, 1, (uint32)cs89x0);
  return OK;
}

/****************************************************************************
 * Function: cs89x0_uiptxpoll
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

static int cs89x0_uiptxpoll(struct uip_driver_s *dev)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (cs89x0->sk_dev.d_len > 0)
    {
      uip_arp_out(&cs89x0->sk_dev);
      cs89x0_transmit(cs89x0);

      /* Check if there is room in the CS89x0 to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: cs89x0_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   cs89x0  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void cs89x0_receive(struct cs89x0_driver_s *cs89x0)
{
  do
    {
      /* Check for errors and update statistics */

      /* Check if the packet is a valid size for the uIP buffer configuration */

      /* Copy the data data from the hardware to cs89x0->sk_dev.d_buf.  Set
       * amount of data in cs89x0->sk_dev.d_len
       */

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          uip_arp_ipin();
          uip_input(&cs89x0->sk_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (cs89x0->sk_dev.d_len > 0)
           {
             uip_arp_out(&cs89x0->sk_dev);
             cs89x0_transmit(cs89x0);
           }
         }
       else if (BUF->type == htons(UIP_ETHTYPE_ARP))
         {
           uip_arp_arpin(&cs89x0->sk_dev);

           /* If the above function invocation resulted in data that should be
            * sent out on the network, the field  d_len will set to a value > 0.
            */

            if (cs89x0->sk_dev.d_len > 0)
              {
                cs89x0_transmit(cs89x0);
              }
          }
      }
    }
  while (); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: cs89x0_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   cs89x0  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void cs89x0_txdone(struct cs89x0_driver_s *cs89x0)
{
  /* Check for errors and update statistics */

  /* If no further xmits are pending, then cancel the TX timeout */

  wd_cancel(cs89x0->sk_txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&cs89x0->sk_dev, cs89x0_uiptxpoll);
}

/****************************************************************************
 * Function: cs89x0_interrupt
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

static int cs89x0_interrupt(int irq, FAR void *context)
{
  register struct cs89x0_driver_s *cs89x0 = &g_cs89x0[0];

  /* Disable Ethernet interrupts */

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call cs89x0_receive() */

  cs89x0_receive(cs89x0);

  /* Check is a packet transmission just completed.  If so, call cs89x0_txdone */

  cs89x0_txdone(cs89x0);

  /* Enable Ethernet interrupts (perhaps excluding the TX done interrupt if 
   * there are no pending transmissions.
   */

  return OK;
}

/****************************************************************************
 * Function: cs89x0_txtimeout
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

static void cs89x0_txtimeout(int argc, uint32 arg, ...)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)arg;

  /* Increment statistics and dump debug info */

  /* Then reset the hardware */

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&cs89x0->sk_dev, cs89x0_uiptxpoll);
}

/****************************************************************************
 * Function: cs89x0_polltimer
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

static void cs89x0_polltimer(int argc, uint32 arg, ...)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)arg;

  /* Check if there is room in the send another TXr packet.  */

  /* If so, update TCP timing states and poll uIP for new XMIT data */

  (void)uip_timer(&cs89x0->sk_dev, cs89x0_uiptxpoll, CS89x0_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(cs89x0->sk_txpoll, CS89x0_WDDELAY, cs89x0_polltimer, 1, arg);
}

/****************************************************************************
 * Function: cs89x0_ifup
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

static int cs89x0_ifup(struct uip_driver_s *dev)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initilize Ethernet interface */

  /* Set and activate a timer process */

  (void)wd_start(cs89x0->sk_txpoll, CS89x0_WDDELAY, cs89x0_polltimer, 1, (uint32)cs89x0);

  /* Enable the Ethernet interrupt */

  cs89x0->sk_bifup = TRUE;
  up_enable_irq(CONFIG_CS89x0_IRQ);
  return OK;
}

/****************************************************************************
 * Function: cs89x0_ifdown
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

static int cs89x0_ifdown(struct uip_driver_s *dev)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = irqsave();
  up_disable_irq(CONFIG_CS89x0_IRQ);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(cs89x0->sk_txpoll);
  wd_cancel(cs89x0->sk_txtimeout);

  /* Reset the device */

  cs89x0->sk_bifup = FALSE;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: cs89x0_txavail
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

static int cs89x0_txavail(struct uip_driver_s *dev)
{
  struct cs89x0_driver_s *cs89x0 = (struct cs89x0_driver_s *)dev->d_private;
  irqstate_t flags;

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (cs89x0->sk_bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&cs89x0->sk_dev, cs89x0_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: cs89x0_initialize
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

/* Initialize the CS89x0 chip and driver */

int cs89x0_initialize(void)
{
  /* Check if a Ethernet chip is recognized at its I/O base */

  /* Attach the IRQ to the driver */

  if (irq_attach(CONFIG_CS89x0_IRQ, cs89x0_interrupt))
    {
      /* We could not attach the ISR to the ISR */

      return -EAGAIN;
    }

  /* Initialize the driver structure */

  memset(g_cs89x0, 0, CONFIG_CS89x0_NINTERFACES*sizeof(struct cs89x0_driver_s));
  g_cs89x0[0].sk_dev.d_ifup    = cs89x0_ifup;     /* I/F down callback */
  g_cs89x0[0].sk_dev.d_ifdown  = cs89x0_ifdown;   /* I/F up (new IP address) callback */
  g_cs89x0[0].sk_dev.d_txavail = cs89x0_txavail;  /* New TX data callback */
  g_cs89x0[0].sk_dev.d_private = (void*)g_cs89x0; /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  g_cs89x0[0].sk_txpoll       = wd_create();   /* Create periodic poll timer */
  g_cs89x0[0].sk_txtimeout    = wd_create();   /* Create TX timeout timer */

  /* Read the MAC address from the hardware into g_cs89x0[0].sk_dev.d_mac.ether_addr_octet */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&g_cs89x0[0].sk_dev);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_CS89x0 */

