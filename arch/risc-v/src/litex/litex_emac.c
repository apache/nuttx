/****************************************************************************
 * arch/risc-v/src/litex/litex_emac.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ioctl.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "riscv_internal.h"
#include "chip.h"
#include "hardware/litex_emac.h"

#ifdef CONFIG_LITEX_ETHMAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#else

  /* Select work queue.  Always use the LP work queue if available.  If not,
   * then LPWORK will re-direct to the HP work queue.
   *
   * NOTE:  However, the network should NEVER run on the high priority work
   * queue!  That queue is intended only to service short back end interrupt
   * processing that never suspends.  Suspending the high priority work queue
   * may bring the system to its knees!
   */

#  define ETHWORK LPWORK
#endif

/* TX timeout - in seconds */

#define LITEX_TXTIMEOUT           (60 * CLK_TCK)

/* Wait for Ethernet link Timeout - in ms */

#define LITEX_WAITLINKTIMEOUT     (5000)

/* PHY Reset Timeout - in ms */

#define LITEX_PHY_RESETTIMEOUT    (20)

/* LITEX MDIO register bit definitions */

#define LITEX_PHY_MDIO_CK         0x01
#define LITEX_PHY_MDIO_OE         0x02
#define LITEX_PHY_MDIO_DO         0x04

#define LITEX_PHY_MDIO_DI         0x01

#define LITEX_PHY_MDIO_PREAMBLE   0xffffffff
#define LITEX_PHY_MDIO_START      0x1
#define LITEX_PHY_MDIO_READ       0x2
#define LITEX_PHY_MDIO_WRITE      0x1
#define LITEX_PHY_MDIO_TURNAROUND 0x2

/* PHY definitions */

#if defined(CONFIG_ETH0_PHY_DP83848C)
#  define BOARD_PHY_NAME        "DP83848C"
#  define BOARD_PHYID1          MII_PHYID1_DP83848C
#  define BOARD_PHYID2          MII_PHYID2_DP83848C
#  define BOARD_PHY_STATUS      MII_DP83848C_STS
#  define BOARD_PHY_10BASET(s)  (((s) & MII_DP83848C_PHYSTS_SPEED) != 0)
#  define BOARD_PHY_100BASET(s) (((s) & MII_DP83848C_PHYSTS_SPEED) == 0)
#  define BOARD_PHY_ISDUPLEX(s) (((s) & MII_DP83848C_PHYSTS_DUPLEX) != 0)
#else
#  error EMAC PHY unrecognized
#endif

#ifdef CONFIG_NET_DUMPPACKET
#  define litex_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define litex_dumppacket(m,a,n)
#endif

/* Helpers ******************************************************************/

/* This is a helper pointer for accessing the contents of the Ethernet
 * header
 */

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The litex_emac_s encapsulates all state information for EMAC peripheral */

struct litex_emac_s
{
  bool                  ifup;        /* true:ifup false:ifdown */
  struct wdog_s         txpoll;      /* TX poll timer */
  struct wdog_s         txtimeout;   /* TX timeout timer */
  struct work_s         irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s         pollwork;    /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s   dev;         /* Interface understood by the network */
  uint8_t               phyaddr;     /* PHY address (pre-defined by pins on reset) */

  uint8_t               txslot;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The driver state singleton */

static struct litex_emac_s g_emac;

/* Global Rx/Tx Buffer */

static uint8_t g_buffer[ETHMAC_SLOT_SIZE]
               aligned_data(8);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void litex_txavail_work(void *arg);
static int  litex_txavail(struct net_driver_s *dev);
static int  litex_txpoll(struct net_driver_s *dev);
static void litex_txdone(struct litex_emac_s *priv);
static void litex_txtimeout_work(void *arg);
static void litex_txtimeout_expiry(wdparm_t arg);
static int  litex_transmit(struct litex_emac_s *priv);

static void litex_receive(struct litex_emac_s *priv);

static void litex_emac_interrupt_work(void *arg);
static int  litex_emac_interrupt(int irq, void *context, void *arg);

static int  litex_linkup(struct litex_emac_s *priv);
static int  litex_ifup(struct net_driver_s *dev);
static int  litex_ifdown(struct net_driver_s *dev);

static void litex_phywriteraw(uint32_t word, uint8_t bitcount);
static uint16_t litex_phyreadraw(void);
static void litex_phyturnaround(void);
static int  litex_phyread(struct litex_emac_s *priv, uint8_t phyaddr,
                          uint8_t regaddr, uint16_t *phyval);
#ifdef CONFIG_NETDEV_IOCTL
static int  litex_phywrite(struct litex_emac_s *priv, uint8_t phyaddr,
                           uint8_t regaddr, uint16_t phyval);
#endif
#if defined(CONFIG_DEBUG_NET) && defined(CONFIG_DEBUG_INFO)
static void litex_phydump(struct litex_emac_s *priv);
#else
#  define litex_phydump(priv)
#endif
static int  litex_phyfind(struct litex_emac_s *priv, uint8_t phyaddr);
static int  litex_phyinit(struct litex_emac_s *priv);

#ifdef CONFIG_NETDEV_IOCTL
static int  litex_ioctl(struct net_driver_s *dev, int cmd,
                        unsigned long arg);
#endif

static int  litex_emac_configure(struct litex_emac_s *priv);
static int  litex_buffer_init(struct litex_emac_s *priv);
static void litex_ethinitialize(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: litex_txavail_work
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

static void litex_txavail_work(void *arg)
{
  struct litex_emac_s *priv = (struct litex_emac_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();

  if (priv->ifup)
    {
      devif_poll(&priv->dev, litex_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Function: litex_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available. This is a
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

static int litex_txavail(struct net_driver_s *dev)
{
  struct litex_emac_s *priv = (struct litex_emac_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, litex_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: litex_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send. This is a callback from devif_poll().
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
 *   May or may not be called from an interrupt handler. In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int litex_txpoll(struct net_driver_s *dev)
{
  struct litex_emac_s *priv = (struct litex_emac_s *)dev->d_private;

  DEBUGASSERT(priv->dev.d_buf != NULL);

  /* Send the packet */

  litex_transmit(priv);

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: litex_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv - Reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *   The network is locked.
 *
 ****************************************************************************/

static void litex_txdone(struct litex_emac_s *priv)
{
  /* We are here because a transmission completed, so the watchdog can be
   * canceled.
   */

  wd_cancel(&priv->txtimeout);

  /* Update statistics */

  NETDEV_TXDONE(&priv->dev);

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  devif_poll(&priv->dev, litex_txpoll);
}

/****************************************************************************
 * Function: litex_txtimeout_work
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
 *
 ****************************************************************************/

static void litex_txtimeout_work(void *arg)
{
  struct litex_emac_s *priv = (struct litex_emac_s *)arg;

  /* Increment statistics and dump debug info */

  net_lock();

  nerr("ERROR: Timeout!\n");
  nerr("Resetting interface\n");

  NETDEV_TXTIMEOUTS(&priv->dev);

  /* Take the interface down and bring it back up.  That is the most
   * aggressive hardware reset.
   */

  litex_ifdown(&priv->dev);
  litex_ifup(&priv->dev);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->dev, litex_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: litex_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out. Called from the timer interrupt handler.
 *   The last TX never completed. Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void litex_txtimeout_expiry(wdparm_t arg)
{
  struct litex_emac_s *priv = (struct litex_emac_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

  up_disable_irq(LITEX_IRQ_ETHMAC);

  /* Schedule to perform the TX timeout processing on the worker thread,
   * canceling any pending interrupt work.
   */

  work_queue(ETHWORK, &priv->irqwork, litex_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: litex_transmit
 *
 * Description:
 *   Start hardware transmission. Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int litex_transmit(struct litex_emac_s *priv)
{
  irqstate_t flags;

  if (getreg32(LITEX_ETHMAC_SRAM_READER_READY) == 0)
    {
      nerr("Tx Busy\n");
      return -EBUSY;
    }

  ninfo("Sending packet, length: %d slot: %d\n",
         priv->dev.d_len, priv->txslot);
  litex_dumppacket("Tx Packet", g_buffer, priv->dev.d_len);

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

  /* Copy frame to the hardware buffer */

  memcpy((void *)(LITEX_ETHMAC_TXBASE + (priv->txslot * ETHMAC_SLOT_SIZE)),
         g_buffer, priv->dev.d_len);

  /* Make the following operations atomic */

  flags = spin_lock_irqsave(NULL);

  /* Now start transmission */

  putreg8(priv->txslot, LITEX_ETHMAC_SRAM_READER_SLOT);
  putreg16(priv->dev.d_len, LITEX_ETHMAC_SRAM_READER_LENGTH);
  putreg8(0x01, LITEX_ETHMAC_SRAM_READER_START);

  /* Increment the hardware TX slot -> wrap around */

  priv->txslot = (priv->txslot + 1) % ETHMAC_TX_SLOTS;

  /* Setup the TX timeout watchdog */

  wd_start(&priv->txtimeout, LITEX_TXTIMEOUT,
           litex_txtimeout_expiry, (wdparm_t)priv);

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

/****************************************************************************
 * Function: litex_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of one or more
 *   new RX packets in FIFO memory.
 *
 * Input Parameters:
 *   priv - Reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void litex_receive(struct litex_emac_s *priv)
{
  /* Update statistics */

  NETDEV_RXPACKETS(&priv->dev);

  priv->dev.d_len = getreg16(LITEX_ETHMAC_SRAM_WRITER_LENGTH);

  if (priv->dev.d_len == 0 || priv->dev.d_len > ETHMAC_SLOT_SIZE)
    {
      NETDEV_RXDROPPED(&priv->dev);
      return;
    }

  /* Copy frame from hardware buffer at the slot which activated
   * this reception
   */

  const uint8_t rxslot = getreg8(LITEX_ETHMAC_SRAM_WRITER_SLOT);
  memcpy(g_buffer,
         (void *)(LITEX_ETHMAC_RXBASE + (rxslot * ETHMAC_SLOT_SIZE)),
         priv->dev.d_len);
  litex_dumppacket("Rx Packet", g_buffer, priv->dev.d_len);

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the tap */

  pkt_input(&priv->dev);
#endif

#ifdef CONFIG_NET_IPv4
  /* Check for an IPv4 packet */

  if (BUF->type == HTONS(ETHTYPE_IP))
    {
      ninfo("IPv4 frame\n");
      NETDEV_RXIPV4(&priv->dev);

      /* Receive an IPv4 packet from the network device */

      ipv4_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          /* And send the packet */

          litex_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      ninfo("IPv6 frame\n");
      NETDEV_RXIPV6(&priv->dev);

      /* Give the IPv6 packet to the network layer */

      ipv6_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, d_len field will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          /* And send the packet */

          litex_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  /* Check for an ARP packet */

  if (BUF->type == HTONS(ETHTYPE_ARP))
    {
      ninfo("ARP frame\n");
      NETDEV_RXARP(&priv->dev);
      arp_input(&priv->dev);

      /* If the above function invocation resulted in data that should
       * be sent out on the network, the field d_len will set to a
       * value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          litex_transmit(priv);
        }
    }
#endif
  else
    {
      NETDEV_RXDROPPED(&priv->dev);
    }
}

/****************************************************************************
 * Function: litex_emac_interrupt_work
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
 *   Runs on a worker thread.
 *   The network is locked.
 *
 ****************************************************************************/

static void litex_emac_interrupt_work(void *arg)
{
  struct litex_emac_s *priv = (struct litex_emac_s *)arg;

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Rx Available */

  if (getreg32(LITEX_ETHMAC_SRAM_WRITER_EV_PENDING) & 0x01)
    {
      litex_receive(priv);

      /* ACK the pending flag AFTER receiving and processing data.
       * This transitions the receive slot in hardware.
       */

      putreg8(0x01, LITEX_ETHMAC_SRAM_WRITER_EV_PENDING);
    }

  /* Tx Done */

  if (getreg32(LITEX_ETHMAC_SRAM_READER_EV_PENDING) & 0x01)
    {
      litex_txdone(priv);

      /* ACK the pending flag AFTER processing Tx Done */

      putreg8(0x01, LITEX_ETHMAC_SRAM_READER_EV_PENDING);
    }

  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(LITEX_IRQ_ETHMAC);
}

/****************************************************************************
 * Function: litex_emac_interrupt
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

static int litex_emac_interrupt(int irq, void *context, void *arg)
{
  struct litex_emac_s *priv = &g_emac;

  /* Disable further Ethernet interrupts. Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  up_disable_irq(LITEX_IRQ_ETHMAC);

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->irqwork, litex_emac_interrupt_work, priv, 0);

  return OK;
}

/****************************************************************************
 * Function: litex_linkup
 *
 * Description:
 *   Check if the link is up
 *
 * Input Parameters:
 *   priv - Reference to the private driver state structure
 *
 * Returned Value:
 *   true: The link is up
 *
 ****************************************************************************/

static int litex_linkup(struct litex_emac_s *priv)
{
  int ret;
  uint16_t i;
  uint16_t msr;

  for (i = 0; i < LITEX_WAITLINKTIMEOUT; ++i)
    {
      nxsig_usleep(1000);

      ret = litex_phyread(priv, priv->phyaddr, MII_MSR, &msr);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read MSR: %d\n", ret);
          return ret;
        }

      if ((msr & MII_MSR_LINKSTATUS) != 0)
        {
          return OK;
        }
    }

  nerr("ERROR: Timeout to wait for PHY LINK UP\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Function: litex_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the EMAC interface when an IP address is
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

static int litex_ifup(struct net_driver_s *dev)
{
  struct litex_emac_s *priv = (struct litex_emac_s *)dev->d_private;
  int ret;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff),
        (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff),
        (int)(dev->d_ipaddr >> 24));

  /* Configure the EMAC interface for normal operation. */

  ninfo("Initialize the EMAC\n");
  litex_emac_configure(priv);

  /* Initialize for PHY access */

  ret = litex_phyinit(priv);
  if (ret < 0)
    {
      nerr("ERROR: litex_phyinit failed: %d\n", ret);
      return ret;
    }

  ninfo("Wait for link up\n");

  ret = litex_linkup(priv);
  if (ret != 0)
    {
      nerr("ERROR: Failed to wait LINK UP error=%d\n", ret);
      return ret;
    }

  ninfo("Link detected\n");

  /* Enable normal MAC operation */

  ninfo("Enable normal operation\n");

  /* Enable the EMAC interrupt */

  priv->ifup = true;
  up_enable_irq(LITEX_IRQ_ETHMAC);

  litex_phydump(priv);

  return OK;
}

/****************************************************************************
 * Function: litex_ifdown
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

static int litex_ifdown(struct net_driver_s *dev)
{
  struct litex_emac_s *priv = (struct litex_emac_s *)dev->d_private;
  irqstate_t flags;

  ninfo("Taking the network down\n");

  /* Disable the EMAC interrupt */

  flags = enter_critical_section();
  up_disable_irq(LITEX_IRQ_ETHMAC);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(&priv->txpoll);
  wd_cancel(&priv->txtimeout);

  /* Hold the PHY device in reset and mark the interface as "down" */

  putreg32(1, LITEX_ETHPHY_CRG_RESET);
  priv->ifup = false;
  leave_critical_section(flags);

  litex_phydump(priv);

  return OK;
}

/****************************************************************************
 * Function: litex_phywriteraw
 *
 * Description:
 * Write 1 word to the MDIO interface.
 * The hardware requires the data to be bitbashed in.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void litex_phywriteraw(uint32_t word, uint8_t bitcount)
{
  word <<= 32 - bitcount;

  while (bitcount > 0)
    {
      if (word & 0x80000000)
        {
          putreg32(LITEX_PHY_MDIO_DO | LITEX_PHY_MDIO_OE,
                   LITEX_ETHPHY_MDIO_W);
          up_udelay(1);
          putreg32(LITEX_PHY_MDIO_DO | LITEX_PHY_MDIO_OE | LITEX_PHY_MDIO_CK,
                   LITEX_ETHPHY_MDIO_W);
          up_udelay(1);
          putreg32(LITEX_PHY_MDIO_DO | LITEX_PHY_MDIO_OE,
                   LITEX_ETHPHY_MDIO_W);
        }
      else
        {
          putreg32(LITEX_PHY_MDIO_OE,
                   LITEX_ETHPHY_MDIO_W);
          up_udelay(1);
          putreg32(LITEX_PHY_MDIO_OE | LITEX_PHY_MDIO_CK,
                   LITEX_ETHPHY_MDIO_W);
          up_udelay(1);
          putreg32(LITEX_PHY_MDIO_OE,
                   LITEX_ETHPHY_MDIO_W);
        }

      word <<= 1;
      bitcount--;
    }
}

/****************************************************************************
 * Function: litex_phyreadraw
 *
 * Description:
 * Read 1 word from the MDIO interface.
 * The hardware requires this interface to be bitbashed to get the data out.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static uint16_t litex_phyreadraw(void)
{
  uint16_t word;
  uint8_t i;

  word = 0;

  for (i = 0; i < 16; ++i)
    {
      word <<= 1;

      if (getreg32(LITEX_ETHPHY_MDIO_R) & LITEX_PHY_MDIO_DI)
        {
          word |= 1;
        }

      putreg32(LITEX_PHY_MDIO_CK, LITEX_ETHPHY_MDIO_W);
      up_udelay(1);
      putreg32(0x00, LITEX_ETHPHY_MDIO_W);
      up_udelay(1);
    }

  return word;
}

/****************************************************************************
 * Function: litex_phyturnaround
 *
 * Description:
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void litex_phyturnaround(void)
{
  int i;

  for (i = 0; i < 2; ++i)
    {
      up_udelay(1);
      putreg32(LITEX_PHY_MDIO_CK, LITEX_ETHPHY_MDIO_W);
      up_udelay(1);
      putreg32(0x00, LITEX_ETHPHY_MDIO_W);
    }
}

/****************************************************************************
 * Function: litex_phyread
 *
 * Description:
 *  Read a PHY register.
 *
 * Input Parameters:
 *   priv    - Reference to the private driver state structure.
 *   phyaddr - The PHY device address.
 *   regaddr - The PHY register address.
 *   phyval  - The location to return the 16-bit PHY register value.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int litex_phyread(struct litex_emac_s *priv, uint8_t phyaddr,
                         uint8_t regaddr, uint16_t *phyval)
{
  putreg32(LITEX_PHY_MDIO_OE, LITEX_ETHPHY_MDIO_W);
  litex_phywriteraw(LITEX_PHY_MDIO_PREAMBLE, 32);
  litex_phywriteraw(LITEX_PHY_MDIO_START, 2);
  litex_phywriteraw(LITEX_PHY_MDIO_READ, 2);
  litex_phywriteraw(phyaddr, 5);
  litex_phywriteraw(regaddr, 5);
  litex_phyturnaround();
  *phyval = litex_phyreadraw();
  litex_phyturnaround();

  return OK;
}

/****************************************************************************
 * Function: litex_phywrite
 *
 * Description:
 *  Write to a PHY register.
 *
 * Input Parameters:
 *   priv    - Reference to the private driver state structure.
 *   phyaddr - The PHY device address.
 *   regaddr - The PHY register address.
 *   phyval  - The 16-bit value to write to the PHY register.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int litex_phywrite(struct litex_emac_s *priv, uint8_t phyaddr,
                        uint8_t regaddr, uint16_t phyval)
{
  putreg32(LITEX_PHY_MDIO_OE, LITEX_ETHPHY_MDIO_W);
  litex_phywriteraw(LITEX_PHY_MDIO_PREAMBLE, 32);
  litex_phywriteraw(LITEX_PHY_MDIO_START, 2);
  litex_phywriteraw(LITEX_PHY_MDIO_WRITE, 2);
  litex_phywriteraw(phyaddr, 5);
  litex_phywriteraw(regaddr, 5);
  litex_phywriteraw(LITEX_PHY_MDIO_TURNAROUND, 2);
  litex_phywriteraw(phyval, 16);
  litex_phyturnaround();

  return OK;
}
#endif

/****************************************************************************
 * Function: litex_phydump
 *
 * Description:
 *   Dump the contents of PHY registers
 *
 * Input Parameters:
 *   priv - Reference to the private driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_NET) && defined(CONFIG_DEBUG_INFO)
static void litex_phydump(struct litex_emac_s *priv)
{
  uint16_t phyval;

  ninfo("%s: MII Registers (Address %02x)\n", BOARD_PHY_NAME, priv->phyaddr);

  litex_phyread(priv, priv->phyaddr, MII_MCR, &phyval);
  ninfo("  MCR:       %04x\n", phyval);
  litex_phyread(priv, priv->phyaddr, MII_MSR, &phyval);
  ninfo("  MSR:       %04x\n", phyval);
  litex_phyread(priv, priv->phyaddr, MII_ADVERTISE, &phyval);
  ninfo("  ADVERTISE: %04x\n", phyval);
  litex_phyread(priv, priv->phyaddr, MII_LPA, &phyval);
  ninfo("  LPA:       %04x\n", phyval);
  litex_phyread(priv, priv->phyaddr, MII_EXPANSION, &phyval);
  ninfo("  EXPANSION: %04x\n", phyval);
  litex_phyread(priv, priv->phyaddr, MII_DP83848C_10BTSCR, &phyval);
  ninfo("  10BTCR:    %04x\n", phyval);
  litex_phyread(priv, priv->phyaddr, BOARD_PHY_STATUS, &phyval);
  ninfo("  PHYSR:     %04x\n", phyval);

  if (phyval == 0xffff)
    {
      return;
    }

  if (BOARD_PHY_ISDUPLEX(phyval))
    {
      ninfo("%s: Full duplex\n",  BOARD_PHY_NAME);
    }
  else
    {
      ninfo("%s: Half duplex\n",  BOARD_PHY_NAME);
    }

  if (BOARD_PHY_10BASET(phyval))
    {
      /* 10 Mbps */

      ninfo("%s: 10 Base-T\n",  BOARD_PHY_NAME);
    }
  else if (BOARD_PHY_100BASET(phyval))
    {
      /* 100 Mbps */

      ninfo("%s: 100 Base-T\n",  BOARD_PHY_NAME);
    }
  else
    {
      /* This might happen if Autonegotiation did not complete(?) */

      nerr("ERROR: Neither 10- nor 100-BaseT reported: PHY STATUS=%04x\n",
          phyval);
    }
}
#endif

/****************************************************************************
 * Function: litex_phyfind
 *
 * Description:
 *  Verify the PHY address.
 *
 * Input Parameters:
 *   priv    - Reference to the private driver state structure
 *   phyaddr - MDIO address to try and find confgiured PHY IC
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int litex_phyfind(struct litex_emac_s *priv, uint8_t phyaddr)
{
  uint16_t phyval[2];
  uint32_t oui;
  uint16_t model;
  uint16_t revision;

  int ret;

  ninfo("%s: Find a valid PHY at address: %d\n",
        BOARD_PHY_NAME, phyaddr);

  /* Read PHYID1 */

  ret = litex_phyread(priv, phyaddr, MII_PHYID1, &phyval[0]);

  /* Verify PHYID1 */

  if (ret != OK)
    {
      nerr("ERROR: litex_phyread failed for PHY address %02x: %d\n",
           phyaddr, ret);
      return -EIO;
    }

  if (phyval[0] != BOARD_PHYID1)
    {
      nerr("ERROR: unexpected PHYID - Expected: %02x Got: %02x\n",
           BOARD_PHYID1, phyval[0]);
      return -ESRCH;
    }

  /* Read PHYID2 */

  ret = litex_phyread(priv, phyaddr, MII_PHYID2, &phyval[1]);

  /* Verify PHYID2 */

  if (ret != OK)
    {
      nerr("ERROR: litex_phyread failed for PHY address %02x: %d\n",
           phyaddr, ret);
      return -EIO;
    }

  if (phyval[1] != BOARD_PHYID2)
    {
      nerr("ERROR: unexpected PHYID - Expected: %02x Got: %02x\n",
           BOARD_PHYID2, phyval[1]);
      return -ESRCH;
    }

  oui = ((phyval[0] << 16) | (phyval[1] & 0xfc00)) >> 10;
  model = (phyval[1] & 0x03f0) >> 4;
  revision = (phyval[1] & 0x000f);

  ninfo("%s: PHY Found - OUI: 0x%04" PRIx32 "MODEL: %u REV: %u\n",
        BOARD_PHY_NAME, oui, model, revision);

  return OK;
}

/****************************************************************************
 * Function: litex_phyinit
 *
 * Description:
 *  Configure the PHY
 *
 * Input Parameters:
 *   priv - Reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int litex_phyinit(struct litex_emac_s *priv)
{
  int ret;

  /* Reset PHY */

  putreg32(1, LITEX_ETHPHY_CRG_RESET);
  nxsig_usleep(LITEX_PHY_RESETTIMEOUT);
  putreg32(0, LITEX_ETHPHY_CRG_RESET);
  nxsig_usleep(LITEX_PHY_RESETTIMEOUT);

  /* Check the PHY responds at configured address */

  priv->phyaddr = CONFIG_LITEX_EMAC_PHYADDR;

  ninfo("PHY ADDR: %d\n", priv->phyaddr);

  ret = litex_phyfind(priv, priv->phyaddr);
  if (ret < 0)
    {
      nerr("ERROR: litex_phyfind failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Function: litex_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int litex_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  struct litex_emac_s *priv = (struct litex_emac_s *)dev->d_private;
#endif
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
      case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          req->phy_id = priv->phyaddr;
          ret = OK;
        }
        break;

      case SIOCGMIIREG: /* Get register from MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);

          /* Read from the requested register */

          ret = litex_phyread(priv, req->phy_id,
                              req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG: /* Set register in MII PHY */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);

          /* Write to the requested register */

          ret = litex_phywrite(priv, req->phy_id,
                               req->reg_num, req->val_in);
        }
        break;
#endif /* ifdef CONFIG_NETDEV_PHY_IOCTL */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: litex_emac_configure
 *
 * Description:
 *  Configure the EMAC interface for normal operation.
 *
 * Input Parameters:
 *   priv - Reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int litex_emac_configure(struct litex_emac_s *priv)
{
  /* Clear pending events */

  putreg8(0x01, LITEX_ETHMAC_SRAM_READER_EV_PENDING);
  putreg8(0x01, LITEX_ETHMAC_SRAM_WRITER_EV_PENDING);

  /* Enable Tx and Rx interrupts */

  putreg8(0x01, LITEX_ETHMAC_SRAM_READER_EV_ENABLE);
  putreg8(0x01, LITEX_ETHMAC_SRAM_WRITER_EV_ENABLE);

  return OK;
}

/****************************************************************************
 * Function: litex_buffer_init
 *
 * Description:
 *   Allocate aligned TX and RX descriptors and buffers.  For the case of
 *   pre-allocated structures, the function degenerates to a few assignments.
 *
 * Input Parameters:
 *   priv - Reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

static int litex_buffer_init(struct litex_emac_s *priv)
{
  priv->txslot = 0;
  priv->dev.d_buf = g_buffer;

  return OK;
}

/****************************************************************************
 * Function: litex_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void litex_ethinitialize(void)
{
  struct litex_emac_s *priv = &g_emac;
  int ret;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct litex_emac_s));
  priv->dev.d_ifup    = litex_ifup;       /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = litex_ifdown;     /* I/F down callback */
  priv->dev.d_txavail = litex_txavail;    /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = litex_ioctl;      /* Support PHY ioctl() calls */
#endif
  priv->dev.d_private = &g_emac;          /* Used to recover private state from dev */

  /* Init buffers */

  ret = litex_buffer_init(priv);
  if (ret < 0)
    {
      nerr("ERROR: litex_buffer_initialize failed: %d\n", ret);
      return;
    }

  /* Attach the IRQ to the driver.  It will not be enabled at the AIC until
   * the interface is in the 'up' state.
   */

  ret = irq_attach(LITEX_IRQ_ETHMAC, litex_emac_interrupt, NULL);
  if (ret < 0)
    {
      nerr("ERROR: Failed to attach the handler to the IRQ%d\n",
           LITEX_IRQ_ETHMAC);
    }

  /* Put the interface in the down state */

  ret = litex_ifdown(&priv->dev);
  if (ret < 0)
    {
      nerr("ERROR: Failed to put the interface in the down state: %d\n",
           ret);
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->dev, NET_LL_ETHERNET);
  if (ret >= 0)
    {
      return;
    }

  nerr("ERROR: netdev_register() failed: %d\n", ret);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: riscv_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in riscv_initialize.c.
 *
 * Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void riscv_netinitialize(void)
{
  litex_ethinitialize();
}
#endif

#endif //CONFIG_LITEX_ETHMAC
