/****************************************************************************
 * drivers/net/lan91c111.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include <nuttx/net/arp.h>
#include <nuttx/net/lan91c111.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/pkt.h>

#include "lan91c111.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Work queue support is required. */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define LAN91C111_WORK LPWORK

#ifdef CONFIG_NET_DUMPPACKET
#  define lan91c111_dumppacket  lib_dumpbuffer
#else
#  define lan91c111_dumppacket(m, b, l)
#endif

/* TX poll delay = 1 seconds.
 * CLK_TCK is the number of clock ticks per second
 */

#define LAN91C111_WDDELAY       (1*CLK_TCK)

/* MII busy delay = 1 microsecond */

#define LAN91C111_MIIDELAY      1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* lan91c111_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct lan91c111_driver_s
{
  uintptr_t base;                         /* Base address */
  int       irq;                          /* IRQ number */
  uint16_t  bank;                         /* Current bank */
  struct wdog_s txpoll;                   /* TX poll timer */
  struct work_s irqwork;                  /* For deferring interrupt work to the work queue */
  struct work_s pollwork;                 /* For deferring poll work to the work queue */
  uint8_t pktbuf[MAX_NETDEV_PKTSIZE + 4]; /* +4 due to getregs32/putregs32 */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  lan91c111_transmit(FAR struct net_driver_s *dev);
static int  lan91c111_txpoll(FAR struct net_driver_s *dev);

/* Interrupt handling */

static void lan91c111_reply(FAR struct net_driver_s *dev);
static void lan91c111_receive(FAR struct net_driver_s *dev);
static void lan91c111_txdone(FAR struct net_driver_s *dev);

static void lan91c111_interrupt_work(FAR void *arg);
static int  lan91c111_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void lan91c111_poll_work(FAR void *arg);
static void lan91c111_poll_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  lan91c111_ifup(FAR struct net_driver_s *dev);
static int  lan91c111_ifdown(FAR struct net_driver_s *dev);

static void lan91c111_txavail_work(FAR void *arg);
static int  lan91c111_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  lan91c111_addmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#ifdef CONFIG_NET_MCASTGROUP
static int  lan91c111_rmmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void lan91c111_ipv6multicast(FAR struct net_driver_s *dev);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  lan91c111_ioctl(FAR struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* MAC register access, assume lock hold by caller */

static uint16_t updatebank(FAR struct lan91c111_driver_s *priv,
                           uint16_t offset)
{
  uint16_t bank = offset >> 8;

  if (bank != priv->bank)
    {
      *(FAR volatile uint16_t *)(priv->base + BSR_REG) = bank;
      priv->bank = bank;
    }

  return offset & 0xff;
}

static uint8_t getreg8(FAR struct lan91c111_driver_s *priv, uint16_t offset)
{
  offset = updatebank(priv, offset);
  return *(FAR volatile uint8_t *)(priv->base + offset);
}

static uint16_t getreg16(FAR struct lan91c111_driver_s *priv,
                         uint16_t offset)
{
  offset = updatebank(priv, offset);
  return *(FAR volatile uint16_t *)(priv->base + offset);
}

static void putreg8(FAR struct lan91c111_driver_s *priv,
                    uint16_t offset, uint8_t value)
{
  offset = updatebank(priv, offset);
  *(FAR volatile uint8_t *)(priv->base + offset) = value;
}

static void putreg16(FAR struct lan91c111_driver_s *priv,
                     uint16_t offset, uint16_t value)
{
  offset = updatebank(priv, offset);
  *(FAR volatile uint16_t *)(priv->base + offset) = value;
}

static void modifyreg16(FAR struct lan91c111_driver_s *priv,
                        uint16_t offset, uint16_t clearbits,
                        uint16_t setbits)
{
  uint16_t value;

  offset = updatebank(priv, offset);
  value  = *(FAR volatile uint16_t *)(priv->base + offset);
  value &= ~clearbits;
  value |= setbits;
  *(FAR volatile uint16_t *)(priv->base + offset) = value;
}

static void getregs32(FAR struct lan91c111_driver_s *priv,
                      uint16_t offset, void *value_, size_t length)
{
  FAR uint32_t *value = value_;
  size_t i;

  offset = updatebank(priv, offset);
  for (i = 0; i < length; i += sizeof(*value))
    {
      *value++ = *(FAR volatile uint32_t *)(priv->base + offset);
    }
}

static void putregs32(FAR struct lan91c111_driver_s *priv,
                      uint16_t offset, const void *value_, size_t length)
{
  FAR const uint32_t *value = value_;
  size_t i;

  offset = updatebank(priv, offset);
  for (i = 0; i < length; i += sizeof(*value))
    {
      *(FAR volatile uint32_t *)(priv->base + offset) = *value++;
    }
}

static void copyfrom16(FAR struct lan91c111_driver_s *priv,
                       uint16_t offset, FAR void *value_, size_t length)
{
  FAR uint16_t *value = value_;
  size_t i;

  offset = updatebank(priv, offset);
  for (i = 0; i < length; i += sizeof(*value))
    {
      *value++ = *(FAR volatile uint16_t *)(priv->base + offset);
      offset  += sizeof(*value);
    }
}

static void copyto16(FAR struct lan91c111_driver_s *priv,
                     uint16_t offset, FAR const void *value_, size_t length)
{
  FAR const uint16_t *value = value_;
  size_t i;

  offset = updatebank(priv, offset);
  for (i = 0; i < length; i += sizeof(*value))
    {
      *(FAR volatile uint16_t *)(priv->base + offset) = *value++;
      offset += sizeof(*value);
    }
}

/* PHY register access, assume lock hold by caller */

static void outmii(FAR struct lan91c111_driver_s *priv,
                   uint32_t value, size_t bits)
{
  uint32_t mask;
  uint16_t mii;

  mii  = getreg16(priv, MII_REG);
  mii &= ~MII_MCLK;
  mii |= MII_MDOE;

  for (mask = 1 << (bits - 1); mask; mask >>= 1)
    {
      if (value & mask)
        {
          mii |= MII_MDO;
        }
      else
        {
          mii &= ~MII_MDO;
        }

      putreg16(priv, MII_REG, mii);
      up_udelay(LAN91C111_MIIDELAY);
      putreg16(priv, MII_REG, mii | MII_MCLK);
      up_udelay(LAN91C111_MIIDELAY);
    }
}

static uint32_t inmii(FAR struct lan91c111_driver_s *priv, size_t bits)
{
  uint32_t value = 0;
  uint32_t mask;
  uint16_t mii;

  mii  = getreg16(priv, MII_REG);
  mii &= ~(MII_MCLK | MII_MDOE);

  for (mask = 1 << (bits - 1); mask; mask >>= 1)
    {
      putreg16(priv, MII_REG, mii);
      up_udelay(LAN91C111_MIIDELAY);

      if (getreg16(priv, MII_REG) & MII_MDI)
        {
          value |= mask;
        }

      putreg16(priv, MII_REG, mii | MII_MCLK);
      up_udelay(LAN91C111_MIIDELAY);
    }

  return value;
}

static uint16_t getphy(FAR struct lan91c111_driver_s *priv, uint8_t offset)
{
  uint16_t value;

  /* Idle - 32 ones */

  outmii(priv, 0xffffffff, 32);

  /* Start(01) + read(10) + addr(00000) + offset(5bits) */

  outmii(priv, 6 << 10 | 0 << 5 | offset, 14);

  /* Turnaround(2bits) + value(16bits) */

  value = inmii(priv, 18); /* Cut the high 2bits */

  /* Return to idle state */

  modifyreg16(priv, MII_REG, MII_MCLK | MII_MDOE | MII_MDO, 0);

  return value;
}

static void putphy(FAR struct lan91c111_driver_s *priv,
                   uint8_t offset, uint16_t value)
{
  /* Idle - 32 ones */

  outmii(priv, 0xffffffff, 32);

  /* Start(01) + write(01) + addr(00000) + offset(5bits) + turnaround(10) +
   * value(16bits)
   */

  outmii(priv, 5 << 28 | 0 << 23 | offset << 18 | 2 << 16 | value, 32);

  /* Return to idle state */

  modifyreg16(priv, MII_REG, MII_MCLK | MII_MDOE | MII_MDO, 0);
}

/* Small utility function, assume lock hold by caller */

static void lan91c111_command_mmu(FAR struct lan91c111_driver_s *priv,
                                  uint16_t cmd)
{
  putreg16(priv, MMU_CMD_REG, cmd);
  while (getreg16(priv, MMU_CMD_REG) & MC_BUSY)
    {
      /* Wait until the current command finish */
    }
}

/****************************************************************************
 * Name: lan91c111_transmit
 *
 * Description:
 *   Start hardware transmission. Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int lan91c111_transmit(FAR struct net_driver_s *dev)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;
  uint32_t pages;
  uint8_t packet;

  /* Verify that the hardware is ready to send another packet. If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  /* The MMU wants the number of pages to be the number of 256 bytes
   * 'pages', minus 1 (since a packet can't ever have 0 pages :))
   *
   * Packet size for allocating is data length +6 (for additional status
   * words, length and ctl)
   *
   * If odd size then last byte is included in ctl word.
   */

  pages = ((dev->d_len & ~1) + 6) >> 8;

  while (1)
    {
      /* Release the received packet if no free memory */

      if (!(getreg16(priv, MIR_REG) & MIR_FREE_MASK) &&
          !(getreg8(priv, RXFIFO_REG) & RXFIFO_REMPTY))
        {
          lan91c111_command_mmu(priv, MC_RELEASE);
          NETDEV_RXERRORS(dev);
        }

      /* Now, try to allocate the memory */

      lan91c111_command_mmu(priv, MC_ALLOC | pages);
      while (1) /* Then wait the response */
        {
          if (getreg8(priv, INT_REG) & IM_ALLOC_INT)
            {
              /* Acknowledge the interrupt */

              putreg8(priv, INT_REG, IM_ALLOC_INT);
              break;
            }
        }

      /* Check the result */

      packet = getreg8(priv, AR_REG);
      if (!(packet & AR_FAILED))
        {
          break; /* Got the packet */
        }
    }

  /* Increment statistics */

  lan91c111_dumppacket("transmit", dev->d_buf, dev->d_len);
  NETDEV_TXPACKETS(dev);

  /* Send the packet: address=dev->d_buf, length=dev->d_len */

  /* Point to the beginning of the packet */

  putreg8(priv, PN_REG, packet);
  putreg16(priv, PTR_REG, PTR_AUTOINC);

  /* Send the status(set to zeros) and the packet
   * length(+6 for status, length and ctl byte)
   */

  putreg16(priv, DATA_REG, 0);
  putreg16(priv, DATA_REG, dev->d_len + 6);

  /* Append ctl byte */

  dev->d_buf[dev->d_len]     = TC_ODD;
  dev->d_buf[dev->d_len + 1] = 0x00;

  /* Copy and enqueue the buffer */

  putregs32(priv, DATA_REG, dev->d_buf, dev->d_len + 2);
  lan91c111_command_mmu(priv, MC_ENQUEUE);

  /* Assume the transmission no error, otherwise
   * revert the increment in lan91c111_txdone.
   */

  NETDEV_TXDONE(dev);
  return OK;
}

/****************************************************************************
 * Name: lan91c111_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send fail
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int lan91c111_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (dev->d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
      if (IFF_IS_IPv4(dev->d_flags))
        {
          arp_out(dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv6(dev->d_flags))
        {
          neighbor_out(dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(dev))
        {
          /* Send the packet */

          lan91c111_transmit(dev);

          /* Check if there is room in the device to hold another packet.  If
           * not, return a non-zero value to terminate the poll.
           */

          return !(getreg16(priv, MIR_REG) & MIR_FREE_MASK);
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: lan91c111_reply
 *
 * Description:
 *   After a packet has been received and dispatched to the network, it
 *   may return with an outgoing packet. This function checks for
 *   that case and performs the transmission if necessary.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void lan91c111_reply(FAR struct net_driver_s *dev)
{
  /* If the packet dispatch resulted in data that should be sent out on the
   * network, the field d_len will set to a value > 0.
   */

  if (dev->d_len > 0)
    {
      /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
      if (IFF_IS_IPv4(dev->d_flags))
        {
          arp_out(dev);
        }
#endif

#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv6(dev->d_flags))
        {
          neighbor_out(dev);
        }
#endif

      /* And send the packet */

      lan91c111_transmit(dev);
    }
}

/****************************************************************************
 * Name: lan91c111_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void lan91c111_receive(FAR struct net_driver_s *dev)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;
  FAR struct eth_hdr_s *eth = (FAR struct eth_hdr_s *)dev->d_buf;
  uint16_t status;
  uint16_t length;

  /* If the RX FIFO is empty then nothing to do */

  if (getreg8(priv, RXFIFO_REG) & RXFIFO_REMPTY)
    {
      return;
    }

  /* Read from start of packet */

  putreg16(priv, PTR_REG, PTR_RCV | PTR_AUTOINC | PTR_READ);

  /* Check for errors and update statistics */

  status = getreg16(priv, DATA_REG);
  if (status & RS_ERRORS)
    {
      lan91c111_command_mmu(priv, MC_RELEASE);
      NETDEV_RXERRORS(dev);
      return;
    }

  /* Check if the packet is a valid size for the network buffer
   * configuration.
   */

  length  = getreg16(priv, DATA_REG);
  length &= 0x07ff; /* Mask off top bits */

  /* Remove the header and tail space */

  length -= status & RS_ODDFRAME ? 5 : 6;

  if (length < ETH_HDRLEN || length > MAX_NETDEV_PKTSIZE)
    {
      lan91c111_command_mmu(priv, MC_RELEASE);
      NETDEV_RXERRORS(dev);
      return;
    }

  /* Copy the data from the hardware to dev->d_buf. Set
   * amount of data in dev->d_len
   */

  getregs32(priv, DATA_REG, dev->d_buf, length);
  dev->d_len = length;

  lan91c111_command_mmu(priv, MC_RELEASE);

  lan91c111_dumppacket("receive", dev->d_buf, dev->d_len);
  NETDEV_RXPACKETS(dev);

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the tap */

  pkt_input(dev);
#endif

  /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
  if (eth->type == HTONS(ETHTYPE_IP))
    {
      ninfo("IPv4 frame\n");
      NETDEV_RXIPV4(dev);

      /* Handle ARP on input, then dispatch IPv4 packet to the network
       * layer.
       */

      arp_ipin(dev);
      ipv4_input(dev);

      /* Check for a reply to the IPv4 packet */

      lan91c111_reply(dev);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (eth->type == HTONS(ETHTYPE_IP6))
    {
      ninfo("IPv6 frame\n");
      NETDEV_RXIPV6(dev);

      /* Dispatch IPv6 packet to the network layer */

      ipv6_input(dev);

      /* Check for a reply to the IPv6 packet */

      lan91c111_reply(dev);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (eth->type == htons(ETHTYPE_ARP))
    {
      ninfo("ARP frame\n");
      NETDEV_RXARP(dev);

      /* Dispatch ARP packet to the network layer */

      arp_arpin(dev);

      /* Check for a reply to the ARP packet */

      lan91c111_reply(dev);
    }
  else
#endif
    {
      NETDEV_RXDROPPED(dev);
    }
}

/****************************************************************************
 * Name: lan91c111_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void lan91c111_txdone(FAR struct net_driver_s *dev)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;
  uint16_t status;
  uint8_t  packet;

  /* If the TX FIFO is empty then nothing to do */

  packet = getreg8(priv, TXFIFO_REG);
  if (packet & TXFIFO_TEMPTY)
    {
      return;
    }

  /* Read the status word and free this packet */

  putreg8(priv, PN_REG, packet);
  putreg16(priv, PTR_REG, PTR_AUTOINC | PTR_READ);

  status = getreg16(priv, DATA_REG);
  lan91c111_command_mmu(priv, MC_FREEPKT);

  /* Check for errors and update statistics */

  if (status & ES_ERRORS)
    {
      /* Re-enable transmit */

      modifyreg16(priv, TCR_REG, 0, TCR_ENABLE);

#ifdef CONFIG_NETDEV_STATISTICS
      /* Revert the increment in lan91c111_transmit */

      dev->d_statistics.tx_done--;
#endif
      NETDEV_TXERRORS(dev);
    }
  else
    {
      DEBUGASSERT(0);
    }
}

/****************************************************************************
 * Name: lan91c111_phy_notify
 *
 * Description:
 *   An interrupt was received indicating that the phy has status change
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void lan91c111_phy_notify(FAR struct net_driver_s *dev)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;

  do
    {
      if (getphy(priv, MII_MSR) & MII_MSR_LINKSTATUS)
        {
          if (getphy(priv, MII_LPA) & MII_LPA_FULL)
            {
              modifyreg16(priv, TCR_REG, 0, TCR_SWFDUP);
            }
          else
            {
              modifyreg16(priv, TCR_REG, TCR_SWFDUP, 0);
            }

          netdev_carrier_on(dev);
        }
      else
        {
          netdev_carrier_off(dev);
        }
    }
  while (getphy(priv, PHY_INT_REG) & PHY_INT_INT);
}

/****************************************************************************
 * Name: lan91c111_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs on a worker thread.
 *
 ****************************************************************************/

static void lan91c111_interrupt_work(FAR void *arg)
{
  FAR struct net_driver_s *dev = arg;
  FAR struct lan91c111_driver_s *priv = dev->d_private;
  uint8_t status;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Process pending Ethernet interrupts */

  while (1)
    {
      /* Get interrupt status bits */

      status  = getreg8(priv, INT_REG);
      status &= getreg8(priv, IM_REG);
      if (!status)
        {
          break;
        }

      /* Handle interrupts according to status bit settings */

      /* Check if we received an incoming packet,
       * if so, call lan91c111_receive()
       */

      if (status & IM_RCV_INT)
        {
          lan91c111_receive(dev);
        }

      if (status & IM_RX_OVRN_INT)
        {
          NETDEV_RXERRORS(dev);
        }

      /* Check if a packet transmission just completed.
       * If so, call lan91c111_txdone.
       */

      if (status & IM_TX_INT)
        {
          lan91c111_txdone(dev);
        }

      /* Check if we have the phy interrupt,
       * if so, call lan91c111_phy_notify()
       */

      if (status & IM_MDINT)
        {
          lan91c111_phy_notify(dev);
        }

      /* Clear interrupt status bits */

      putreg8(priv, INT_REG, status);

      /* In any event, poll the network for new TX data */

      if (getreg16(priv, MIR_REG) & MIR_FREE_MASK)
        {
          devif_poll(dev, lan91c111_txpoll);
        }
    }

  net_unlock();

  /* Re-enable Ethernet interrupts */

  up_enable_irq(priv->irq);
}

/****************************************************************************
 * Name: lan91c111_interrupt
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
 *   Runs in the context of a the Ethernet interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 ****************************************************************************/

static int lan91c111_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct net_driver_s *dev = arg;
  FAR struct lan91c111_driver_s *priv = dev->d_private;

  /* Disable further Ethernet interrupts. */

  up_disable_irq(priv->irq);

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(LAN91C111_WORK, &priv->irqwork, lan91c111_interrupt_work,
             dev, 0);
  return OK;
}

/****************************************************************************
 * Name: lan91c111_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Run on a work queue thread.
 *
 ****************************************************************************/

static void lan91c111_poll_work(FAR void *arg)
{
  FAR struct net_driver_s *dev = arg;
  FAR struct lan91c111_driver_s *priv = dev->d_private;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Perform the poll */

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  if (getreg16(priv, MIR_REG) & MIR_FREE_MASK)
    {
      /* If so, update TCP timing states and poll the network for new XMIT
       * data.  Hmmm.. might be bug here.  Does this mean if there is a
       * transmit in progress, we will missing TCP time state updates?
       */

      devif_timer(dev, LAN91C111_WDDELAY, lan91c111_txpoll);
    }

  /* Setup the watchdog poll timer again */

  wd_start(&priv->txpoll, LAN91C111_WDDELAY,
           lan91c111_poll_expiry, (wdparm_t)dev);
  net_unlock();
}

/****************************************************************************
 * Name: lan91c111_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Runs in the context of a the timer interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 ****************************************************************************/

static void lan91c111_poll_expiry(wdparm_t arg)
{
  FAR struct net_driver_s *dev = (FAR struct net_driver_s *)arg;
  FAR struct lan91c111_driver_s *priv = dev->d_private;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(LAN91C111_WORK, &priv->pollwork, lan91c111_poll_work, dev, 0);
}

/****************************************************************************
 * Name: lan91c111_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int lan91c111_ifup(FAR struct net_driver_s *dev)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;

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

  net_lock();

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  putreg16(priv, CONFIG_REG, CONFIG_DEFAULT);
  putreg16(priv, CTL_REG, CTL_DEFAULT);
  putreg16(priv, TCR_REG, TCR_DEFAULT);
  putreg16(priv, RCR_REG, RCR_DEFAULT);
  putreg16(priv, RPC_REG, RPC_DEFAULT);

  putreg8(priv, IM_REG,
    IM_MDINT | IM_RX_OVRN_INT | IM_RCV_INT | IM_TX_INT);

  putphy(priv, PHY_MASK_REG, /* Interrupts listed here are disabled */
    PHY_INT_LOSSSYNC | PHY_INT_CWRD | PHY_INT_SSD | PHY_INT_ESD |
    PHY_INT_RPOL | PHY_INT_JAB | PHY_INT_SPDDET | PHY_INT_DPLXDET);

  putphy(priv, MII_MCR, MII_MCR_ANENABLE | MII_MCR_ANRESTART);

  lan91c111_phy_notify(dev); /* Check the initial phy state */

  /* Instantiate the MAC address from dev->d_mac.ether.ether_addr_octet */

  copyto16(priv, ADDR0_REG, &dev->d_mac.ether, sizeof(dev->d_mac.ether));

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  lan91c111_ipv6multicast(dev);
#endif

  /* Set and activate a timer process */

  wd_start(&priv->txpoll, LAN91C111_WDDELAY,
           lan91c111_poll_expiry, (wdparm_t)dev);
  net_unlock();

  /* Enable the Ethernet interrupt */

  up_enable_irq(priv->irq);
  return OK;
}

/****************************************************************************
 * Name: lan91c111_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int lan91c111_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(priv->irq);

  /* Cancel the TX poll timer and work */

  wd_cancel(&priv->txpoll);

  work_cancel(LAN91C111_WORK, &priv->irqwork);
  work_cancel(LAN91C111_WORK, &priv->pollwork);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the lan91c111_ifup() always
   * successfully brings the interface back up.
   */

  putreg8(priv, IM_REG, 0);

  putreg16(priv, RCR_REG, RCR_CLEAR);
  putreg16(priv, TCR_REG, TCR_CLEAR);
  putreg16(priv, CTL_REG, CTL_CLEAR);
  putreg16(priv, CONFIG_REG, CONFIG_CLEAR);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: lan91c111_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Runs on a work queue thread.
 *
 ****************************************************************************/

static void lan91c111_txavail_work(FAR void *arg)
{
  FAR struct net_driver_s *dev = arg;
  FAR struct lan91c111_driver_s *priv = dev->d_private;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (IFF_IS_UP(dev->d_flags))
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (getreg16(priv, MIR_REG) & MIR_FREE_MASK)
        {
          /* If so, then poll the network for new XMIT data */

          devif_poll(dev, lan91c111_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: lan91c111_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int lan91c111_txavail(FAR struct net_driver_s *dev)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(LAN91C111_WORK, &priv->pollwork, lan91c111_txavail_work,
                 dev, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: lan91c111_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 *   IEEE (CRC32) from http://www.microchip.com/wwwproducts/en/LAN91C111
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static uint32_t lan91c111_crc32(FAR const uint8_t *src, size_t len)
{
  uint32_t crc = 0xffffffff;
  uint8_t carry;
  uint8_t temp;
  size_t i;
  size_t j;

  for (i = 0; i < len; i++)
    {
      temp = *src++;
      for (j = 0; j < 8; j++)
        {
          carry = (crc & 0x80000000 ? 1 : 0) ^ (temp & 0x01);
          crc <<= 1;
          if (carry)
            {
              crc = (crc ^ 0x04c11db6) | carry;
            }

          temp >>= 1;
        }
    }

  return crc;
}

static int lan91c111_addmac(FAR struct net_driver_s *dev,
                            FAR const uint8_t *mac)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;
  uint16_t off;
  uint16_t bit;
  uint32_t hash;

  /* Calculate Ethernet CRC32 for MAC */

  hash = lan91c111_crc32(mac, ETHER_ADDR_LEN);

  /* The multicast table is a register array of 4 16-bit registers
   * and the hash value is defined as the six most significant bits
   * of the CRC of the destination addresses. The two msb's determine
   * the register to be used (MCAST1-MCAST4), while the other four
   * determine the bit within the register. If the appropriate bit in
   * the table is set, the packet is received.
   */

  off = (hash >> 29) & 0x06;
  bit = (hash >> 26) & 0x0f;

  /* Add the MAC address to the hardware multicast routing table */

  net_lock();
  modifyreg16(priv, MCAST_REG1 + off, 0, 1 << bit);
  net_unlock();

  return OK;
}
#endif

/****************************************************************************
 * Name: lan91c111_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int lan91c111_rmmac(FAR struct net_driver_s *dev,
                           FAR const uint8_t *mac)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;
  uint16_t off;
  uint16_t bit;
  uint32_t hash;

  /* Calculate Ethernet CRC32 for MAC */

  hash = lan91c111_crc32(mac, ETHER_ADDR_LEN);

  /* TODO: since the six most significant bits of the CRC from two
   * different destination addresses may have the same value. We
   * need a reference count here to avoid clear the bit prematurely.
   */

  off = (hash >> 29) & 0x06;
  bit = (hash >> 26) & 0x0f;

  /* Remove the MAC address from the hardware multicast routing table */

  net_lock();
  modifyreg16(priv, MCAST_REG1 + off, 1 << bit, 0);
  net_unlock();

  return OK;
}
#endif

/****************************************************************************
 * Name: lan91c111_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void lan91c111_ipv6multicast(FAR struct net_driver_s *dev)
{
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

  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  lan91c111_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  lan91c111_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  lan91c111_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: lan91c111_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int lan91c111_ioctl(FAR struct net_driver_s *dev, int cmd,
                           unsigned long arg)
{
  FAR struct lan91c111_driver_s *priv = dev->d_private;
  struct mii_ioctl_data_s *req = (void *)arg;
  int ret = OK;

  net_lock();

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
    case SIOCGMIIPHY: /* Get MII PHY address */
      req->phy_id = 0;
      break;

    case SIOCGMIIREG: /* Get register from MII PHY */
      req->val_out = getphy(priv, req->reg_num);
      break;

    case SIOCSMIIREG: /* Set register in MII PHY */
      putphy(priv, req->reg_num, req->val_in);
      break;

    default:
      nerr("ERROR: Unrecognized IOCTL command: %d\n", command);
      ret = -ENOTTY; /* Special return value for this case */
    }

  net_unlock();
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lan91c111_initialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Parameters:
 *   base - The controller base address
 *   irq  - The controller irq number
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called early in initialization before multi-tasking is initiated.
 *
 ****************************************************************************/

int lan91c111_initialize(uintptr_t base, int irq)
{
  FAR struct lan91c111_driver_s *priv;
  FAR struct net_driver_s *dev;
  uint16_t macrev;
  uint32_t phyid;
  int ret;

  /* Allocate the interface structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  dev = &priv->dev;

  priv->base = base;
  priv->irq  = irq;

  /* Check if a Ethernet chip is recognized at its I/O base */

  macrev = getreg16(priv, REV_REG);
  phyid  = getphy(priv, MII_PHYID1) << 16;
  phyid |= getphy(priv, MII_PHYID2);
  ninfo("base: %08x irq: %d rev: %04x phy: %08x\n",
        base, irq, macrev, phyid);

  if ((macrev >> 4 & 0x0f) != CHIP_91111FD || phyid != PHY_LAN83C183)
    {
      nerr("ERROR: Unsupported LAN91C111's MAC/PHY\n");
      ret = -ENODEV;
      goto err;
    }

  /* Attach the IRQ to the driver */

  ret = irq_attach(irq, lan91c111_interrupt, dev);
  if (ret < 0)
    {
      /* We could not attach the ISR to the interrupt */

      goto err;
    }

  /* Initialize the driver structure */

  dev->d_buf     = priv->pktbuf;      /* Single packet buffer */
  dev->d_ifup    = lan91c111_ifup;    /* I/F up (new IP address) callback */
  dev->d_ifdown  = lan91c111_ifdown;  /* I/F down callback */
  dev->d_txavail = lan91c111_txavail; /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  dev->d_addmac  = lan91c111_addmac;  /* Add multicast MAC address */
  dev->d_rmmac   = lan91c111_rmmac;   /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl   = lan91c111_ioctl;   /* Handle network IOCTL commands */
#endif
  dev->d_private = priv;              /* Used to recover private state from dev */

  /* Put the interface in the down state. This usually amounts to resetting
   * the device and/or calling lan91c111_ifdown().
   */

  putreg16(priv, RCR_REG, RCR_SOFTRST);
  up_udelay(10);                      /* Pause to make the chip happy */
  putreg16(priv, RCR_REG, RCR_CLEAR);
  putreg16(priv, CONFIG_REG, CONFIG_CLEAR);

  lan91c111_command_mmu(priv, MC_RESET);

  putphy(priv, MII_MCR, MII_MCR_RESET);
  while (getphy(priv, MII_MCR) & MII_MCR_RESET)
    {
      /* Loop, reset don't finish yet */
    }

  /* Read MAC address from the hardware into dev->d_mac.ether */

  copyfrom16(priv, ADDR0_REG, &dev->d_mac.ether, sizeof(dev->d_mac.ether));

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(dev, NET_LL_ETHERNET);
  return OK;

err:
  kmm_free(priv);
  return ret;
}
