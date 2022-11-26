/****************************************************************************
 * arch/misoc/src/common/misoc_net.c
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
#if defined(CONFIG_NET) && defined(CONFIG_MISOC_ETHERNET)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#include <arch/board/board.h>
#include <arch/board/generated/csr.h>

#include "chip.h"
#include "hw/flags.h"
#include "hw/ethmac_mem.h"
#include "misoc.h"

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then high priority
 * work queue support is required.
 */

#if !defined(CONFIG_SCHED_HPWORK)
  /* REVISIT: The low priority work queue would be preferred if available */

#  error High priority work queue support is required
#endif

/* CONFIG_MISOC_NET_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_MISOC_NET_NINTERFACES
# define CONFIG_MISOC_NET_NINTERFACES 1
#endif

/* TX timeout = 1 minute */

#define MISOC_NET_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->misoc_net_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The misoc_net_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct misoc_net_driver_s
{
  bool misoc_net_bifup;               /* true:ifup false:ifdown */
  struct wdog_s misoc_net_txtimeout;  /* TX timeout timer */
  struct work_s misoc_net_irqwork;    /* For deferring interrupt work to the work queue */
  struct work_s misoc_net_pollwork;   /* For deferring poll work to the work queue */
  uint8_t *rx0_buf;                   /* 2 RX and 2 TX buffer */
  uint8_t *rx1_buf;
  uint8_t *tx0_buf;
  uint8_t *tx1_buf;
  uint8_t *tx_buf;
  uint8_t tx_slot;                    /* The slot from which we send packet (tx0/tx1) */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s misoc_net_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer is used */

static uint8_t g_pktbuf[CONFIG_MISOC_NET_NINTERFACES]
                       [MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* Driver state structure */

static struct misoc_net_driver_s g_misoc_net[CONFIG_MISOC_NET_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  misoc_net_transmit(struct misoc_net_driver_s *priv);
static int  misoc_net_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void misoc_net_receive(struct misoc_net_driver_s *priv);
static void misoc_net_txdone(struct misoc_net_driver_s *priv);

static void misoc_net_interrupt_work(void *arg);
static int  misoc_net_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void misoc_net_txtimeout_work(void *arg);
static void misoc_net_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int misoc_net_ifup(struct net_driver_s *dev);
static int misoc_net_ifdown(struct net_driver_s *dev);

static void misoc_net_txavail_work(void *arg);
static int misoc_net_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int misoc_net_addmac(struct net_driver_s *dev,
                            const uint8_t *mac);
#ifdef CONFIG_NET_MCASTGROUP
static int misoc_net_rmmac(struct net_driver_s *dev,
                           const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void misoc_net_ipv6multicast(struct misoc_net_driver_s *priv);
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: misoc_net_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   the network is locked.
 *
 ****************************************************************************/

static int misoc_net_transmit(struct misoc_net_driver_s *priv)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  /* Increment statistics */

  NETDEV_TXPACKETS(priv->misoc_net_dev);

  /* Send the packet: address=priv->misoc_net_dev.d_buf,
   * length=priv->misoc_net_dev.d_len
   *
   * NOTE: This memcpy could be avoided by setting tx_buf
   * to the d_buf pointer and setting d_buf to an alternate
   * buffer.  Some additional buffer management logic would
   * be required.
   */

  memcpy(priv->tx_buf, priv->misoc_net_dev.d_buf,
         priv->misoc_net_dev.d_len);

  /* Choose the slot on which we write */

  ethmac_sram_reader_slot_write(priv->tx_slot);

  /* Write the len */

  if (priv->misoc_net_dev.d_len < 60)
    {
      ethmac_sram_reader_length_write(60);
    }
  else
    {
      ethmac_sram_reader_length_write(priv->misoc_net_dev.d_len);
    }

  /* Trigger the writing */

  ethmac_sram_reader_start_write(1);

  /* switch tx slot */

  priv->tx_slot = (priv->tx_slot + 1) % 2;
  if (priv->tx_slot)
    {
      priv->tx_buf = priv->tx1_buf;
    }
  else
    {
      priv->tx_buf = priv->tx0_buf;
    }

  /* Enable Tx interrupts */

  ethmac_sram_reader_ev_enable_write(1);

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->misoc_net_txtimeout, MISOC_NET_TXTIMEOUT,
           misoc_net_txtimeout_expiry, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Function: misoc_net_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   the network is locked.
 *
 ****************************************************************************/

static int misoc_net_txpoll(struct net_driver_s *dev)
{
  struct misoc_net_driver_s *priv =
    (struct misoc_net_driver_s *)dev->d_private;

  /* Send the packet */

  misoc_net_transmit(priv);

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: misoc_net_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void misoc_net_receive(struct misoc_net_driver_s *priv)
{
  uint8_t rxslot;
  uint32_t rxlen;

  do
    {
      /* Check for errors and update statistics */

      /* Check if the packet is a valid size for the network buffer
       * configuration.
       */

      /* Find rx slot */

      rxslot = ethmac_sram_writer_slot_read();

      /* Get rx len */

      rxlen = ethmac_sram_writer_length_read();

      /* Copy the data data from the hardware to priv->misoc_net_dev.d_buf.
       * Set amount of data in priv->misoc_net_dev.d_len
       *
       * NOTE: These memcpy's could be avoided by simply setting the d_buf
       * pointer to the rx*_buf containing the received data. Some additional
       * buffer management logic would also be required.
       */

      misoc_flush_dcache();

      if (rxslot)
        {
          memcpy(priv->misoc_net_dev.d_buf, priv->rx1_buf, rxlen);
        }
      else
        {
          memcpy(priv->misoc_net_dev.d_buf, priv->rx0_buf, rxlen);
        }

      /* Clear event pending */

      ethmac_sram_writer_ev_pending_write(1);

      priv->misoc_net_dev.d_len = rxlen;

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

       pkt_input(&priv->misoc_net_dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(&priv->misoc_net_dev);

          /* Receive an IPv4 packet from the network device */

          ipv4_input(&priv->misoc_net_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->misoc_net_dev.d_len > 0)
            {
              /* And send the packet */

              misoc_net_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");
          NETDEV_RXIPV6(&priv->misoc_net_dev);

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->misoc_net_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->misoc_net_dev.d_len > 0)
            {
              /* And send the packet */

              misoc_net_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          arp_arpin(&priv->misoc_net_dev);
          NETDEV_RXARP(&priv->misoc_net_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->misoc_net_dev.d_len > 0)
            {
              misoc_net_transmit(priv);
            }
        }
#endif
      else
        {
          NETDEV_RXDROPPED(&priv->misoc_net_dev);
        }
    }
  while (ethmac_sram_writer_ev_pending_read() & ETHMAC_EV_SRAM_WRITER);
}

/****************************************************************************
 * Function: misoc_net_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void misoc_net_txdone(struct misoc_net_driver_s *priv)
{
  /* Check for errors and update statistics */

  NETDEV_TXDONE(priv->misoc_net_dev);

  /* Check if there are pending transmissions */

  /* If no further transmissions are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(&priv->misoc_net_txtimeout);

  /* And disable further TX interrupts. */

  ethmac_sram_reader_ev_enable_write(0);

  /* In any event, poll the network for new TX data */

  devif_poll(&priv->misoc_net_dev, misoc_net_txpoll);
}

/****************************************************************************
 * Function: misoc_net_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void misoc_net_interrupt_work(void *arg)
{
  struct misoc_net_driver_s *priv = (struct misoc_net_driver_s *)arg;

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Check if we received an incoming packet,
   * if so, call misoc_net_receive()
   */

  if (ethmac_sram_writer_ev_pending_read() & ETHMAC_EV_SRAM_WRITER)
    {
      misoc_net_receive(priv);
    }

  /* Check if a packet transmission just completed.  If so, call
   * misoc_net_txdone. This may disable further Tx interrupts if there are no
   * pending transmissions.
   */

  if (ethmac_sram_reader_ev_pending_read() & ETHMAC_EV_SRAM_READER)
    {
      misoc_net_txdone(priv);
      ethmac_sram_reader_ev_pending_write(1);
    }

  net_unlock();

  ethmac_sram_reader_ev_enable_write(1);
  ethmac_sram_writer_ev_enable_write(1);

  /* Re-enable Ethernet interrupts */

  up_enable_irq(ETHMAC_INTERRUPT);
}

/****************************************************************************
 * Function: misoc_net_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int misoc_net_interrupt(int irq, void *context, void *arg)
{
  struct misoc_net_driver_s *priv = &g_misoc_net[0];

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  ethmac_sram_reader_ev_enable_write(0);
  ethmac_sram_writer_ev_enable_write(0);

  /* TODO: Determine if a TX transfer just completed */

  if (ethmac_sram_reader_ev_pending_read() & ETHMAC_EV_SRAM_READER)
    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be do race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

       wd_cancel(&priv->misoc_net_txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(HPWORK, &priv->misoc_net_irqwork,
             misoc_net_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: misoc_net_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void misoc_net_txtimeout_work(void *arg)
{
  struct misoc_net_driver_s *priv = (struct misoc_net_driver_s *)arg;

  /* Increment statistics and dump debug info */

  net_lock();
  NETDEV_TXTIMEOUTS(priv->misoc_net_dev);

  /* Then reset the hardware */

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->misoc_net_dev, misoc_net_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: misoc_net_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void misoc_net_txtimeout_expiry(wdparm_t arg)
{
  struct misoc_net_driver_s *priv = (struct misoc_net_driver_s *)arg;

#if 0 /* REVISIT */
  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(ETHMAC_INTERRUPT);
#endif

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(HPWORK, &priv->misoc_net_irqwork,
             misoc_net_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: misoc_net_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int misoc_net_ifup(struct net_driver_s *dev)
{
  irqstate_t flags;
  struct misoc_net_driver_s *priv =
    (struct misoc_net_driver_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  /* Instantiate the MAC address from
   * priv->misoc_net_dev.d_mac.ether.ether_addr_octet
   */

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  misoc_net_ipv6multicast(priv);
#endif

  flags = enter_critical_section();

  priv->misoc_net_bifup = true;
  up_enable_irq(ETHMAC_INTERRUPT);

  /* Enable the RX Event Handler */

  ethmac_sram_writer_ev_enable_write(1);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: misoc_net_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int misoc_net_ifdown(struct net_driver_s *dev)
{
  struct misoc_net_driver_s *priv =
    (struct misoc_net_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(ETHMAC_INTERRUPT);

  ethmac_sram_reader_ev_enable_write(0);
  ethmac_sram_writer_ev_enable_write(0);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->misoc_net_txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the misoc_net_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->misoc_net_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: misoc_net_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void misoc_net_txavail_work(void *arg)
{
  struct misoc_net_driver_s *priv =
    (struct misoc_net_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->misoc_net_bifup)
    {
      /* Check if there is room in the hardware to hold another packet. */

      if (!ethmac_sram_reader_ready_read())
        {
          /* If so, then poll the network for new XMIT data */

          devif_poll(&priv->misoc_net_dev, misoc_net_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: misoc_net_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int misoc_net_txavail(struct net_driver_s *dev)
{
  struct misoc_net_driver_s *priv =
    (struct misoc_net_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->misoc_net_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(HPWORK, &priv->misoc_net_pollwork,
                 misoc_net_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: misoc_net_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int misoc_net_addmac(struct net_driver_s *dev,
                            const uint8_t *mac)
{
  struct misoc_net_driver_s *priv =
    (struct misoc_net_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: misoc_net_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int misoc_net_rmmac(struct net_driver_s *dev,
                           const uint8_t *mac)
{
  struct misoc_net_driver_s *priv =
    (struct misoc_net_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: misoc_net_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void misoc_net_ipv6multicast(struct misoc_net_driver_s *priv)
{
  struct net_driver_s *dev;
  uint16_t tmp16;
  uint8_t mac[6];

  /* For ICMPv6, we need to add the IPv6 multicast address
   *
   * For IPv6 multicast addresses, the Ethernet MAC is derived by
   * the four low-order octets OR'ed with the MAC 33:33:00:00:00:00,
   * so for example the IPv6 address FF02:DEAD:BEEF::1:3 would map
   * to the Ethernet MAC address 33:33:00:01:00:03.
   *
   * NOTES:  This appears correct for the ICMPv6 Router Solicitation
   * Message, but the ICMPv6 Neighbor Solicitation message seems to
   * use 33:33:ff:01:00:03.
   */

  mac[0] = 0x33;
  mac[1] = 0x33;

  dev    = &priv->dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  misoc_net_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  misoc_net_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  misoc_net_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: misoc_net_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int misoc_net_initialize(int intf)
{
  struct misoc_net_driver_s *priv;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < CONFIG_MISOC_NET_NINTERFACES);
  priv = &g_misoc_net[intf];

  /* Check if a Ethernet chip is recognized at its I/O base */

  /* Attach the IRQ to the driver */

  if (irq_attach(ETHMAC_INTERRUPT, misoc_net_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Clear pending int */

  ethmac_sram_reader_ev_pending_write(ETHMAC_EV_SRAM_READER);
  ethmac_sram_writer_ev_pending_write(ETHMAC_EV_SRAM_WRITER);

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct misoc_net_driver_s));
  priv->rx0_buf = (uint8_t *)ETHMAC_RX0_BASE;
  priv->rx1_buf = (uint8_t *)ETHMAC_RX1_BASE;
  priv->tx0_buf = (uint8_t *)ETHMAC_TX0_BASE;
  priv->tx1_buf = (uint8_t *)ETHMAC_TX1_BASE;
  priv->tx_buf  = priv->tx0_buf;
  priv->tx_slot = 0;

  priv->misoc_net_dev.d_buf     = g_pktbuf[intf];     /* Single packet buffer */
  priv->misoc_net_dev.d_ifup    = misoc_net_ifup;     /* I/F up (new IP address) callback */
  priv->misoc_net_dev.d_ifdown  = misoc_net_ifdown;   /* I/F down callback */
  priv->misoc_net_dev.d_txavail = misoc_net_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->misoc_net_dev.d_addmac  = misoc_net_addmac;   /* Add multicast MAC address */
  priv->misoc_net_dev.d_rmmac   = misoc_net_rmmac;    /* Remove multicast MAC address */
#endif
  priv->misoc_net_dev.d_private = g_misoc_net;        /* Used to recover private state from dev */

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling misoc_net_ifdown().
   */

  /* Read the MAC address from the hardware into
   * priv->misoc_net_dev.d_mac.ether.ether_addr_octet
   */

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->misoc_net_dev, NET_LL_ETHERNET);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_MISOC_NET_ETHERNET */
