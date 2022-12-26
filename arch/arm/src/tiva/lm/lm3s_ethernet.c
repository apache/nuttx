/****************************************************************************
 * arch/arm/src/tiva/lm/lm3s_ethernet.c
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
#if defined(CONFIG_NET) && defined(CONFIG_TIVA_ETHERNET)

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>

#include <arch/board/board.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "chip.h"
#include "arm_internal.h"
#include "tiva_gpio.h"
#include "tiva_ethernet.h"
#include "hardware/tiva_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

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

#define ETHWORK LPWORK

/* Half duplex can be forced if CONFIG_TIVA_ETHHDUPLEX is defined. */

#ifdef CONFIG_TIVA_ETHHDUPLEX
#  define TIVA_DUPLEX_SETBITS 0
#  define TIVA_DUPLEX_CLRBITS MAC_TCTL_DUPLEX
#else
#  define TIVA_DUPLEX_SETBITS MAC_TCTL_DUPLEX
#  define TIVA_DUPLEX_CLRBITS 0
#endif

/* Auto CRC generation can be suppressed
 * if CONFIG_TIVA_ETHNOAUTOCRC is defined
 */

#ifdef CONFIG_TIVA_ETHNOAUTOCRC
#  define TIVA_CRC_SETBITS 0
#  define TIVA_CRC_CLRBITS MAC_TCTL_CRC
#else
#  define TIVA_CRC_SETBITS MAC_TCTL_CRC
#  define TIVA_CRC_CLRBITS 0
#endif

/* Tx padding can be suppressed if CONFIG_TIVA_ETHNOPAD is defined */

#ifdef CONFIG_TIVA_ETHNOPAD
#  define TIVA_PADEN_SETBITS 0
#  define TIVA_PADEN_CLRBITS MAC_TCTL_PADEN
#else
#  define TIVA_PADEN_SETBITS MAC_TCTL_PADEN
#  define TIVA_PADEN_CLRBITS 0
#endif

#define TIVA_TCTCL_SETBITS (TIVA_DUPLEX_SETBITS|TIVA_CRC_SETBITS|TIVA_PADEN_SETBITS)
#define TIVA_TCTCL_CLRBITS (TIVA_DUPLEX_CLRBITS|TIVA_CRC_CLRBITS|TIVA_PADEN_CLRBITS)

/* Multicast frames can be enabled by defining CONFIG_TIVA_MULTICAST */

#ifdef CONFIG_TIVA_MULTICAST
#  define TIVA_AMUL_SETBITS MAC_RCTL_AMUL
#  define TIVA_AMUL_CLRBITS 0
#else
#  define TIVA_AMUL_SETBITS 0
#  define TIVA_AMUL_CLRBITS MAC_RCTL_AMUL
#endif

/* Promiscuous mode can be enabled by defining CONFIG_TIVA_PROMISCUOUS */

#ifdef CONFIG_TIVA_PROMISCUOUS
#  define TIVA_PRMS_SETBITS MAC_RCTL_PRMS
#  define TIVA_PRMS_CLRBITS 0
#else
#  define TIVA_PRMS_SETBITS 0
#  define TIVA_PRMS_CLRBITS MAC_RCTL_PRMS
#endif

/* Bad CRC rejection can be enabled by define CONFIG_TIVA_BADCRC */

#ifdef CONFIG_TIVA_BADCRC
#  define TIVA_BADCRC_SETBITS MAC_RCTL_BADCRC
#  define TIVA_BADCRC_CLRBITS 0
#else
#  define TIVA_BADCRC_SETBITS 0
#  define TIVA_BADCRC_CLRBITS MAC_RCTL_BADCRC
#endif

#define TIVA_RCTCL_SETBITS (TIVA_AMUL_SETBITS|TIVA_PRMS_SETBITS|TIVA_BADCRC_SETBITS)
#define TIVA_RCTCL_CLRBITS (TIVA_AMUL_CLRBITS|TIVA_PRMS_CLRBITS|TIVA_BADCRC_CLRBITS)

/* CONFIG_TIVA_DUMPPACKET will dump the contents of each packet. */

#ifdef CONFIG_TIVA_DUMPPACKET
#  define tiva_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define tiva_dumppacket(m,a,n)
#endif

/* TX timeout = 1 minute */

#define TIVA_TXTIMEOUT (60*CLK_TCK)

#define TIVA_MAX_MDCCLK 2500000

/* This is a helper pointer for accessing the contents of the Ethernet
 * header
 */

#define BUF ((struct eth_hdr_s *)&dev->d_buf[0])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The tiva_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct tiva_driver_s
{
  /* The following fields would only be necessary on chips that support
   * multiple Ethernet controllers.
   */

#if TIVA_NETHCONTROLLERS > 1
  uint32_t ld_base;             /* Ethernet controller base address */
  int      ld_irq;              /* Ethernet controller IRQ */
#endif

  bool     ld_bifup;           /* true:ifup false:ifdown */
  struct wdog_s ld_txtimeout;  /* TX timeout timer */
  struct work_s ld_irqwork;    /* For deferring interrupt work to the work queue */
  struct work_s ld_pollwork;   /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s ld_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer is used */

static uint8_t g_pktbuf[TIVA_NETHCONTROLLERS]
                       [MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* Ethernet peripheral state */

static struct tiva_driver_s g_lm3sdev[TIVA_NETHCONTROLLERS];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Miscellaneous low level helpers */

#if TIVA_NETHCONTROLLERS > 1
static uint32_t tiva_ethin(struct tiva_driver_s *priv, int offset);
static void tiva_ethout(struct tiva_driver_s *priv,
                        int offset, uint32_t value);
#else
static inline uint32_t tiva_ethin(struct tiva_driver_s *priv, int offset);
static inline void tiva_ethout(struct tiva_driver_s *priv,
                               int offset, uint32_t value);
#endif
static void tiva_ethreset(struct tiva_driver_s *priv);
#if 0 /* Not used */
static void tiva_phywrite(struct tiva_driver_s *priv,
                          int regaddr, uint16_t value);
#endif
#ifndef CONFIG_TIVA_WITH_QEMU
static uint16_t tiva_phyread(struct tiva_driver_s *priv, int regaddr);
#endif

/* Common TX logic */

static int  tiva_transmit(struct tiva_driver_s *priv);
static int  tiva_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void tiva_receive(struct tiva_driver_s *priv);
static void tiva_txdone(struct tiva_driver_s *priv);

static void tiva_interrupt_work(void *arg);
static int  tiva_interrupt(int irq, void *context, void *arg);

/* Watchdog timer expirations */

static void tiva_txtimeout_work(void *arg);
static void tiva_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  tiva_ifup(struct net_driver_s *dev);
static int  tiva_ifdown(struct net_driver_s *dev);

static void tiva_txavail_work(void *arg);
static int  tiva_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int  tiva_addmac(struct net_driver_s *dev, const uint8_t *mac);
static int  tiva_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tiva_ethin
 *
 * Description:
 *   Read a register from the Ethernet module
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   offset - Byte offset of the register from the ethernet base address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

#if TIVA_NETHCONTROLLERS > 1
static uint32_t tiva_ethin(struct tiva_driver_s *priv, int offset)
{
  return getreg32(priv->ld_base + offset);
}
#else
static inline uint32_t tiva_ethin(struct tiva_driver_s *priv, int offset)
{
  return getreg32(TIVA_ETHCON_BASE + offset);
}
#endif

/****************************************************************************
 * Function: tiva_ethout
 *
 * Description:
 *   Write a register to the Ethernet module
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   offset - Byte offset of the register from the ethernet base address
 *   value  - The value to write the Ethernet register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if TIVA_NETHCONTROLLERS > 1
static void tiva_ethout(struct tiva_driver_s *priv,
                        int offset, uint32_t value)
{
  putreg32(value, priv->ld_base + offset);
}
#else
static inline void tiva_ethout(struct tiva_driver_s *priv,
                               int offset, uint32_t value)
{
  putreg32(value, TIVA_ETHCON_BASE + offset);
}
#endif

/****************************************************************************
 * Function: tiva_ethreset
 *
 * Description:
 *   Configure and reset the Ethernet module, leaving it in a disabled state.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_ethreset(struct tiva_driver_s *priv)
{
  irqstate_t flags;
  uint32_t regval;

#if TIVA_NETHCONTROLLERS > 1
#  error "If multiple interfaces are supported, this function would have to be redesigned"
#endif

  /* Make sure that clocking is enabled for the Ethernet&PHY peripherals */

  flags   = enter_critical_section();
  regval  = getreg32(TIVA_SYSCON_RCGC2);
  regval |= (SYSCON_RCGC2_EMAC0 | SYSCON_RCGC2_EPHY0);
  putreg32(regval, TIVA_SYSCON_RCGC2);
  ninfo("RCGC2: %08" PRIx32 "\n", regval);

  /* Put the Ethernet controller into the reset state */

  regval  = getreg32(TIVA_SYSCON_SRCR2);
  regval |= (SYSCON_SRCR2_EMAC0 | SYSCON_SRCR2_EPHY0);
  putreg32(regval, TIVA_SYSCON_SRCR2);

  /* Wait just a bit.  This is a much longer delay than necessary */

  up_mdelay(2);

  /* Then take the Ethernet controller out of the reset state */

  regval &= ~(SYSCON_SRCR2_EMAC0 | SYSCON_SRCR2_EPHY0);
  putreg32(regval, TIVA_SYSCON_SRCR2);
  ninfo("SRCR2: %08" PRIx32 "\n", regval);

  /* Wait just a bit, again.
   * If we touch the ethernet too soon, we may busfault.
   */

  up_mdelay(2);

  /* Enable Port F for Ethernet LEDs: LED0=Bit 3; LED1=Bit 2 */

#ifdef CONFIG_TIVA_ETHLEDS
  /* Configure the pins for the peripheral function */

  tiva_configgpio(GPIO_ETHPHY_LED0 | GPIO_STRENGTH_2MA | GPIO_PADTYPE_STD);
  tiva_configgpio(GPIO_ETHPHY_LED1 | GPIO_STRENGTH_2MA | GPIO_PADTYPE_STD);
#endif

  /* Disable all Ethernet controller interrupts */

  regval = tiva_ethin(priv, TIVA_MAC_IM_OFFSET);
  regval &= ~MAC_IM_ALLINTS;
  tiva_ethout(priv, TIVA_MAC_IM_OFFSET, regval);

  /* Clear any pending interrupts (shouldn't be any) */

  regval = tiva_ethin(priv, TIVA_MAC_RIS_OFFSET);
  tiva_ethout(priv, TIVA_MAC_IACK_OFFSET, regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * Function: tiva_phywrite
 *
 * Description:
 *   Write a 16-bit word to a PHY register
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   regaddr - Address of the PHY register to write
 *   value   - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if 0 /* Not used */
static void tiva_phywrite(struct tiva_driver_s *priv,
                          int regaddr, uint16_t value)
{
  /* Wait for any MII transactions in progress to complete */

  while ((tiva_ethin(priv, TIVA_MAC_MCTL_OFFSET) & MAC_MCTL_START) != 0);

  /* Set up the data to be written */

  DEBUGASSERT(value < MAC_MTXD_MASK);
  tiva_ethout(priv, TIVA_MAC_MTXD_OFFSET, value);

  /* Set up the PHY register address and start the write operation */

  regaddr <<= MAC_MCTL_REGADR_SHIFT;
  DEBUGASSERT((regaddr & MAC_MTXD_MASK) == regaddr);
  tiva_ethout(priv, TIVA_MAC_MCTL_OFFSET,
              regaddr | MAC_MCTL_WRITE | MAC_MCTL_START);

  /* Wait for the write transaction to complete */

  while ((tiva_ethin(priv, TIVA_MAC_MCTL_OFFSET) & MAC_MCTL_START) != 0);
}
#endif

/****************************************************************************
 * Function: tiva_phyread
 *
 * Description:
 *   Write a 16-bit word to a PHY register
 *
 * Input Parameters:
 *   priv    - Reference to the driver state structure
 *   regaddr - Address of the PHY register to write
 *   value   - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_TIVA_WITH_QEMU
static uint16_t tiva_phyread(struct tiva_driver_s *priv, int regaddr)
{
  /* Wait for any MII transactions in progress to complete */

  while ((tiva_ethin(priv, TIVA_MAC_MCTL_OFFSET) & MAC_MCTL_START) != 0);

  /* Set up the PHY register address and start the read operation */

  regaddr <<= MAC_MCTL_REGADR_SHIFT;
  DEBUGASSERT((regaddr & MAC_MTXD_MASK) == regaddr);
  tiva_ethout(priv, TIVA_MAC_MCTL_OFFSET, regaddr | MAC_MCTL_START);

  /* Wait for the write transaction to complete */

  while ((tiva_ethin(priv, TIVA_MAC_MCTL_OFFSET) & MAC_MCTL_START) != 0);

  /* Read and return the PHY data */

  return (uint16_t)(tiva_ethin(priv, TIVA_MAC_MRXD_OFFSET) & MAC_MTRD_MASK);
}
#endif

/****************************************************************************
 * Function: tiva_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int tiva_transmit(struct tiva_driver_s *priv)
{
  irqstate_t flags;
  uint32_t   regval;
  uint8_t   *dbuf;
  int        pktlen;
  int        bytesleft;
  int        ret = -EBUSY;

  /* Verify that the hardware is ready to send another packet */

  flags = enter_critical_section();
  if ((tiva_ethin(priv, TIVA_MAC_TR_OFFSET) & MAC_TR_NEWTX) == 0)
    {
      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->ld_dev);
      tiva_dumppacket("Transmit packet",
                      priv->ld_dev.d_buf, priv->ld_dev.d_len);

      /* Transfer the packet into the Tx FIFO.  The LS 16-bits of the first
       * 32-bit word written to the Tx FIFO contains the Ethernet payload
       * data length.  That is the full length of the message (d_len) minus
       * the size of the Ethernet header (14).
       */

      pktlen     = priv->ld_dev.d_len;
      ninfo("Sending packet, pktlen: %d\n", pktlen);
      DEBUGASSERT(pktlen > ETH_HDRLEN);

      dbuf       = priv->ld_dev.d_buf;
      regval     = (uint32_t)(pktlen - 14);
      regval    |= ((uint32_t)(*dbuf++) << 16);
      regval    |= ((uint32_t)(*dbuf++) << 24);
      tiva_ethout(priv, TIVA_MAC_DATA_OFFSET, regval);

      /* Write all of the whole, 32-bit values in the middle of the packet */

      for (bytesleft = pktlen - 2; bytesleft > 3; bytesleft -= 4, dbuf += 4)
        {
          /* Transfer a whole word from the user buffer.  Note, the user
           * buffer may be un-aligned.
           */

          tiva_ethout(priv, TIVA_MAC_DATA_OFFSET, *(uint32_t *)dbuf);
        }

      /* Write the last, partial word in the FIFO */

      if (bytesleft > 0)
        {
          /* Write the last word */

          regval = 0;
          switch (bytesleft)
            {
              case 0:
              default:
                break;

              case 3:
                regval |= ((uint32_t)dbuf[2] << 16);
              case 2:
                regval |= ((uint32_t)dbuf[1] << 8);
              case 1:
                regval |= (uint32_t)dbuf[0];
                break;
            }

          tiva_ethout(priv, TIVA_MAC_DATA_OFFSET, regval);
        }

      /* Activate the transmitter */

      tiva_ethout(priv, TIVA_MAC_TR_OFFSET, MAC_TR_NEWTX);

      /* Setup the TX timeout watchdog (perhaps restarting the timer) */

      wd_start(&priv->ld_txtimeout, TIVA_TXTIMEOUT,
               tiva_txtimeout_expiry, (wdparm_t)priv);
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Function: tiva_txpoll
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
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_txpoll(struct net_driver_s *dev)
{
  struct tiva_driver_s *priv = (struct tiva_driver_s *)dev->d_private;

  /* Send the packet.  tiva_transmit() will return zero if the
   * packet was successfully handled.
   */

  return tiva_transmit(priv);
}

/****************************************************************************
 * Function: tiva_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_receive(struct tiva_driver_s *priv)
{
  struct net_driver_s *dev = &priv->ld_dev;
  uint32_t regval;
  uint8_t *dbuf;
  int bytesleft;
  int pktlen;

  /* Loop while there are incoming packets to be processed */

  while ((tiva_ethin(priv, TIVA_MAC_NP_OFFSET) & MAC_NP_MASK) != 0)
    {
      /* Update statistics */

      NETDEV_RXPACKETS(dev);

      /* Copy the data data from the hardware to dev->d_buf.  Set
       * amount of data in dev->d_len
       */

      dbuf = dev->d_buf;

      /* The packet frame length begins in the LS 16-bits of the first
       * word from the FIFO followed by the Ethernet header beginning
       * in the MS 16-bits of the first word.
       *
       * Pick off the packet length from the first word.  This packet length
       * includes the len/type field (size 2) and the FCS (size 4).
       */

      regval = tiva_ethin(priv, TIVA_MAC_DATA_OFFSET);
      pktlen = (int)(regval & 0x0000ffff);
      ninfo("Receiving packet, pktlen: %d\n", pktlen);

      /* Check if the pktlen is valid.  It should be large enough to hold
       * an Ethernet header and small enough to fit entirely in the I/O
       * buffer.  Six is subtracted to account for the 2-byte length/type
       * and 4 byte FCS that are not copied into the network packet.
       */

      if (pktlen > CONFIG_NET_ETH_PKTSIZE + 6 || pktlen <= ETH_HDRLEN + 6)
        {
          int wordlen;

          /* We will have to drop this packet */

          nwarn("WARNING: Bad packet size dropped (%d)\n", pktlen);
          NETDEV_RXERRORS(dev);

          /* The number of bytes and words left to read is pktlen - 4
           * (including, the final, possibly partial word) because we've
           * already read 4 bytes.
           */

          wordlen = (pktlen - 1) >> 2;

          /* Read and discard the remaining words in the FIFO */

          while (wordlen--)
            {
              tiva_ethin(priv, TIVA_MAC_DATA_OFFSET);
            }

          /* Check for another packet */

          continue;
        }

      /* Save the first two bytes from the first word */

      *dbuf++   = (uint8_t)((regval >> 16) & 0xff);
      *dbuf++   = (uint8_t)((regval >> 24) & 0xff);

      /* Read all of the whole, 32-bit values in the middle of the packet.
       * We've already read the length (2 bytes) plus the first two bytes
       * of data.
       */

      for (bytesleft = pktlen - 4; bytesleft > 7; bytesleft -= 4, dbuf += 4)
        {
          /* Transfer a whole word to the user buffer.  Note, the user
           * buffer may be un-aligned.
           */

          *(uint32_t *)dbuf = tiva_ethin(priv, TIVA_MAC_DATA_OFFSET);
        }

      /* Handle the last, partial word in the FIFO (0-3 bytes) and discard
       * the 4-byte FCS.
       */

      for (; bytesleft > 0; bytesleft -= 4)
        {
          /* Read the last word.  And transfer all but the last four
           * bytes of the FCS into the user buffer.
           */

          regval = tiva_ethin(priv, TIVA_MAC_DATA_OFFSET);
          switch (bytesleft)
            {
              default:
                break;

              case 7:
                dbuf[2] = (regval >> 16) & 0xff;
              case 6:
                dbuf[1] = (regval >> 8) & 0xff;
              case 5:
                dbuf[0] = regval & 0xff;
                break;
            }
        }

      /* Pass the packet length to the network MINUS 2 bytes for the length
       * and 4 bytes for the FCS.
       */

      dev->d_len = pktlen - 6;
      tiva_dumppacket("Received packet",
                      dev->d_buf, dev->d_len);

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

       pkt_input(dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(dev);

          /* Receive an IPv4 packet from the network device */

          ipv4_input(dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (dev->d_len > 0)
            {
              /* And send the packet */

              tiva_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");
          NETDEV_RXIPV6(dev);

          /* Give the IPv6 packet to the network layer */

          ipv6_input(dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              tiva_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP packet received (%02x)\n", BUF->type);
          NETDEV_RXARP(dev);

          arp_input(dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */

           if (dev->d_len > 0)
             {
               tiva_transmit(priv);
             }
        }
      else
#endif
        {
          nwarn("WARNING: Unsupported packet type dropped (%02x)\n",
                HTONS(BUF->type));
          NETDEV_RXDROPPED(dev);
        }
    }
}

/****************************************************************************
 * Function: tiva_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tiva_txdone(struct tiva_driver_s *priv)
{
  /* Cancel the TX timeout */

  wd_cancel(&priv->ld_txtimeout);

  /* Verify that the Tx FIFO is not in use.  The NEWTX bit initiates an
   * Ethernet transmission once the packet has been placed in the TX FIFO.
   * This bit is cleared once the transmission has been completed.  Since
   * we get here because of TXEMP which indicates that the packet was
   * transmitted and that the TX FIFO is empty, NEWTX should always be zero
   * at this point.
   */

  DEBUGASSERT((tiva_ethin(priv, TIVA_MAC_TR_OFFSET) & MAC_TR_NEWTX) == 0);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->ld_dev, tiva_txpoll);
}

/****************************************************************************
 * Function: tiva_interrupt_work
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

static void tiva_interrupt_work(void *arg)
{
  struct tiva_driver_s *priv = (struct tiva_driver_s *)arg;
  uint32_t ris;

  /* Process pending Ethernet interrupts */

  net_lock();

  /* Read the raw interrupt status register */

  ris = tiva_ethin(priv, TIVA_MAC_RIS_OFFSET);

  /* Clear all pending interrupts */

  tiva_ethout(priv, TIVA_MAC_IACK_OFFSET, ris);

  /* Check for errors */

#if defined(CONFIG_NETDEV_STATISTICS)
  if ((ris & MAC_RIS_TXER) != 0)
    {
      /* Tx error */

      NETDEV_TXERRORS(&priv->ld_dev);
    }

  if ((ris & MAC_RIS_FOV) != 0)
    {
      /* Rx FIFO overrun */

      NETDEV_RXERRORS(&priv->ld_dev);
    }

  if ((ris & MAC_RIS_RXER) != 0)
    {
      /* Rx error */

      NETDEV_RXERRORS(&priv->ld_dev);
    }
#endif

  /* Handle (unmasked) interrupts according to status bit settings */

  ris &= tiva_ethin(priv, TIVA_MAC_IM_OFFSET);

  /* Is this an Rx interrupt (meaning that a packet has been received)? */

  if ((ris & MAC_RIS_RXINT) != 0)
    {
      /* Handle the incoming packet */

      tiva_receive(priv);
    }

  /* Is this an Tx interrupt (meaning that the Tx FIFO is empty)? */

  if ((ris & MAC_RIS_TXEMP) != 0)
    {
      /* Handle the complete of the transmission */

      tiva_txdone(priv);
    }

  net_unlock();

  /* Re-enable Ethernet interrupts */

#if TIVA_NETHCONTROLLERS > 1
  up_enable_irq(priv->irq);
#else
  up_enable_irq(TIVA_IRQ_ETHCON);
#endif
}

/****************************************************************************
 * Function: tiva_interrupt
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

static int tiva_interrupt(int irq, void *context, void *arg)
{
  struct tiva_driver_s *priv;
  uint32_t ris;

#if TIVA_NETHCONTROLLERS > 1
# error "A mechanism to associate and interface with an IRQ is needed"
#else
  priv = &g_lm3sdev[0];
#endif

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

#if TIVA_NETHCONTROLLERS > 1
  up_disable_irq(priv->irq);
#else
  up_disable_irq(TIVA_IRQ_ETHCON);
#endif

  /* Read the raw interrupt status register (masking out any disabled
   * interrupts).
   */

  ris  = tiva_ethin(priv, TIVA_MAC_RIS_OFFSET);
  ris &= tiva_ethin(priv, TIVA_MAC_IM_OFFSET);

  /* Is this an Tx interrupt (meaning that the Tx FIFO is empty)? */

  if ((ris & MAC_RIS_TXEMP) != 0)
    {
      /* If a TX transfer just completed, then cancel the TX timeout so
       * there will be do race condition between any subsequent timeout
       * expiration and the deferred interrupt processing.
       */

       wd_cancel(&priv->ld_txtimeout);
    }

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(ETHWORK, &priv->ld_irqwork, tiva_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Function: tiva_txtimeout_work
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

static void tiva_txtimeout_work(void *arg)
{
  struct tiva_driver_s *priv = (struct tiva_driver_s *)arg;

  /* Increment statistics */

  net_lock();
  nerr("ERROR: Tx timeout\n");
  NETDEV_TXTIMEOUTS(&priv->ld_dev);

  /* Then reset the hardware */

  DEBUGASSERT(priv->ld_bifup);
  tiva_ifdown(&priv->ld_dev);
  tiva_ifup(&priv->ld_dev);

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->ld_dev, tiva_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: tiva_txtimeout_expiry
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

static void tiva_txtimeout_expiry(wdparm_t arg)
{
  struct tiva_driver_s *priv = (struct tiva_driver_s *)arg;

  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */

#if TIVA_NETHCONTROLLERS > 1
  up_disable_irq(priv->irq);
#else
  up_disable_irq(TIVA_IRQ_ETHCON);
#endif

  /* Schedule to perform the TX timeout processing on the worker thread. */

  work_queue(ETHWORK, &priv->ld_irqwork, tiva_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Function: tiva_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_ifup(struct net_driver_s *dev)
{
  struct tiva_driver_s *priv = (struct tiva_driver_s *)dev->d_private;
  irqstate_t flags;
  uint32_t regval;
  uint32_t div;
#ifndef CONFIG_TIVA_WITH_QEMU
  uint16_t phyreg;
#endif

  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff),
        (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff),
        (int)(dev->d_ipaddr >> 24));

  /* Enable and reset the Ethernet controller */

  flags = enter_critical_section();
  tiva_ethreset(priv);

  /* Set the management clock divider register for access to the PHY
   * register set. The MDC clock is divided down from the system clock per:
   *
   *   MDCCLK_FREQUENCY = SYSCLK_FREQUENCY / (2 * (div + 1))
   *   div = (SYSCLK_FREQUENCY / 2 / MDCCLK_FREQUENCY) - 1
   *
   * Where the maximum value for MDCCLK_FREQUENCY is 2,500,000.  We will
   * add 1 to assure the max TIVA_MAX_MDCCLK is not exceeded.
   */

  div = SYSCLK_FREQUENCY / 2 / TIVA_MAX_MDCCLK;
  tiva_ethout(priv, TIVA_MAC_MDV_OFFSET, div);
  ninfo("MDV:   %08" PRIx32 "\n", div);

  /* Then configure the Ethernet Controller for normal operation
   *
   * Setup the transmit control register (Full duplex, TX CRC Auto
   * Generation, TX Padding Enabled).
   */

  regval  = tiva_ethin(priv, TIVA_MAC_TCTL_OFFSET);
  regval &= ~TIVA_TCTCL_CLRBITS;
  regval |= TIVA_TCTCL_SETBITS;
  tiva_ethout(priv, TIVA_MAC_TCTL_OFFSET, regval);
  ninfo("TCTL:  %08" PRIx32 "\n", regval);

  /* Setup the receive control register (Disable multicast frames, disable
   * promiscuous mode, disable bad CRC rejection).
   */

  regval  = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
  regval &= ~TIVA_RCTCL_CLRBITS;
  regval |= TIVA_RCTCL_SETBITS;
  tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);
  ninfo("RCTL:  %08" PRIx32 "\n", regval);

  /* Setup the time stamp configuration register */

#ifdef TIVA_ETHTS
  regval = tiva_ethin(priv, TIVA_MAC_TS_OFFSET);
#ifdef CONFIG_TIVA_TIMESTAMP
  regval |= MAC_TS_EN;
#else
  regval &= ~(MAC_TS_EN);
#endif
  tiva_ethout(priv, TIVA_MAC_TS_OFFSET, regval);
  ninfo("TS:    %08" PRIx32 "\n", regval);
#endif

  /* Wait for the link to come up.  This following is not very conservative
   * of system resources -- it really should wait gracefully on a semaphore
   * and the interrupt handler should post the semaphore when LINKSTATUS is
   * set
   */

#ifndef CONFIG_TIVA_WITH_QEMU
  ninfo("Waiting for link\n");
  do
    {
      phyreg = tiva_phyread(priv, MII_MSR);
    }
  while ((phyreg & MII_MSR_LINKSTATUS) == 0);
  ninfo("Link established\n");
#endif

  /* Reset the receive FIFO */

  regval  = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
  regval |= MAC_RCTL_RSTFIFO;
  tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

  /* Enable the Ethernet receiver */

  regval  = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
  regval |= MAC_RCTL_RXEN;
  tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

  /* Enable the Ethernet transmitter */

  regval  = tiva_ethin(priv, TIVA_MAC_TCTL_OFFSET);
  regval |= MAC_TCTL_TXEN;
  tiva_ethout(priv, TIVA_MAC_TCTL_OFFSET, regval);

  /* Reset the receive FIFO (again) */

  regval  = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
  regval |= MAC_RCTL_RSTFIFO;
  tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

  /* Enable the Ethernet interrupt */

#if TIVA_NETHCONTROLLERS > 1
  up_enable_irq(priv->irq);
#else
  up_enable_irq(TIVA_IRQ_ETHCON);
#endif

  /* Enable the Ethernet RX packet receipt interrupt */

  regval = tiva_ethin(priv, TIVA_MAC_IM_OFFSET);
  regval |= MAC_IM_RXINTM;
  tiva_ethout(priv, TIVA_MAC_IM_OFFSET, regval);

  /* Program the hardware with it's MAC address (for filtering) */

  regval = (uint32_t)priv->ld_dev.d_mac.ether.ether_addr_octet[3] << 24 |
           (uint32_t)priv->ld_dev.d_mac.ether.ether_addr_octet[2] << 16 |
           (uint32_t)priv->ld_dev.d_mac.ether.ether_addr_octet[1] << 8 |
           (uint32_t)priv->ld_dev.d_mac.ether.ether_addr_octet[0];
  tiva_ethout(priv, TIVA_MAC_IA0_OFFSET, regval);

  regval = (uint32_t)priv->ld_dev.d_mac.ether.ether_addr_octet[5] << 8 |
           (uint32_t)priv->ld_dev.d_mac.ether.ether_addr_octet[4];
  tiva_ethout(priv, TIVA_MAC_IA1_OFFSET, regval);

  priv->ld_bifup = true;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: tiva_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.  The only way to restore normal
 *   behavior is to call tiva_ifup().
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tiva_ifdown(struct net_driver_s *dev)
{
  struct tiva_driver_s *priv = (struct tiva_driver_s *)dev->d_private;
  irqstate_t flags;
  uint32_t regval;

  ninfo("Taking down: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff),
        (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff),
        (int)(dev->d_ipaddr >> 24));

  /* Cancel the TX timeout timers */

  flags = enter_critical_section();
  wd_cancel(&priv->ld_txtimeout);

  /* Disable the Ethernet interrupt */

#if TIVA_NETHCONTROLLERS > 1
  up_disable_irq(priv->irq);
#else
  up_disable_irq(TIVA_IRQ_ETHCON);
#endif

  /* Disable all Ethernet controller interrupt sources */

  regval = tiva_ethin(priv, TIVA_MAC_IM_OFFSET);
  regval &= ~MAC_IM_ALLINTS;
  tiva_ethout(priv, TIVA_MAC_IM_OFFSET, regval);

  /* Reset the receive FIFO */

  regval  = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
  regval |= MAC_RCTL_RSTFIFO;
  tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

  /* Disable the Ethernet receiver */

  regval  = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
  regval &= ~MAC_RCTL_RXEN;
  tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

  /* Disable the Ethernet transmitter */

  regval  = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
  regval &= ~MAC_TCTL_TXEN;
  tiva_ethout(priv, TIVA_MAC_TCTL_OFFSET, regval);

  /* Reset the receive FIFO (again) */

  regval  = tiva_ethin(priv, TIVA_MAC_RCTL_OFFSET);
  regval |= MAC_RCTL_RSTFIFO;
  tiva_ethout(priv, TIVA_MAC_RCTL_OFFSET, regval);

  /* Clear any pending interrupts */

  regval = tiva_ethin(priv, TIVA_MAC_RIS_OFFSET);
  tiva_ethout(priv, TIVA_MAC_IACK_OFFSET, regval);

  /* The interface is now DOWN */

  priv->ld_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Function: tiva_txavail_work
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

static void tiva_txavail_work(void *arg)
{
  struct tiva_driver_s *priv = (struct tiva_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up or if the Tx FIFO
   * hardware is not available at this time.  The NEWTX bit initiates an
   * Ethernet transmission once the packet has been placed in the TX FIFO.
   * This bit is cleared once the transmission has been completed.  When the
   * transmission completes, tiva_txdone() will be called and the Tx polling
   * will occur at that time.
   */

  net_lock();
  if (priv->ld_bifup &&
      (tiva_ethin(priv, TIVA_MAC_TR_OFFSET) & MAC_TR_NEWTX) == 0)
    {
      /* If the interface is up and we can use the Tx FIFO, then poll the
       * network for new Tx data
       */

      devif_poll(&priv->ld_dev, tiva_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Function: tiva_txavail
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

static int tiva_txavail(struct net_driver_s *dev)
{
  struct tiva_driver_s *priv = (struct tiva_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->ld_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->ld_pollwork, tiva_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: tiva_addmac
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

#ifdef CONFIG_NET_MCASTGROUP
static int tiva_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct tiva_driver_s *priv = (struct tiva_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Multicast MAC support not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Function: tiva_rmmac
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
static int tiva_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct tiva_driver_s *priv = (struct tiva_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

#warning "Multicast MAC support not implemented"
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tiva_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if TIVA_NETHCONTROLLERS > 1 || defined(CONFIG_NETDEV_LATEINIT)
int tiva_ethinitialize(int intf)
#else
static inline int tiva_ethinitialize(int intf)
#endif
{
  struct tiva_driver_s *priv = &g_lm3sdev[intf];
  int ret;

  /* Check if the Ethernet module is present */

  ninfo("Setting up eth%d\n", intf);

#if TIVA_NETHCONTROLLERS > 1
# error "This debug check only works with one interface"
#else
  DEBUGASSERT((getreg32(TIVA_SYSCON_DC4) &
              (SYSCON_DC4_EMAC0 | SYSCON_DC4_EPHY0)) ==
              (SYSCON_DC4_EMAC0 | SYSCON_DC4_EPHY0));
#endif
  DEBUGASSERT((unsigned)intf < TIVA_NETHCONTROLLERS);

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct tiva_driver_s));
  priv->ld_dev.d_buf     = g_pktbuf[intf]; /* Single packet buffer */
  priv->ld_dev.d_ifup    = tiva_ifup;      /* I/F down callback */
  priv->ld_dev.d_ifdown  = tiva_ifdown;    /* I/F up (new IP address) callback */
  priv->ld_dev.d_txavail = tiva_txavail;   /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->ld_dev.d_addmac  = tiva_addmac;    /* Add multicast MAC address */
  priv->ld_dev.d_rmmac   = tiva_rmmac;     /* Remove multicast MAC address */
#endif
  priv->ld_dev.d_private = priv;           /* Used to recover private state from dev */

#if TIVA_NETHCONTROLLERS > 1
# error "A mechanism to associate base address an IRQ with an interface is needed"
  priv->ld_base          = ??;             /* Ethernet controller base address */
  priv->ld_irq           = ??;             /* Ethernet controller IRQ number */
#endif

#ifdef CONFIG_TIVA_BOARDMAC
  /* If the board can provide us with a MAC address, get the address
   * from the board now.  The MAC will not be applied until tiva_ifup()
   * is called (and the MAC can be overwritten with a netdev ioctl call).
   */

  tiva_ethernetmac(&priv->ld_dev.d_mac.ether);
#endif

  /* Perform minimal, one-time initialization -- just reset the controller
   * and leave it disabled.  The Ethernet controller will be reset and
   * properly re-initialized each time tiva_ifup() is called.
   */

  tiva_ethreset(priv);
  tiva_ifdown(&priv->ld_dev);

  /* Attach the IRQ to the driver */

#if TIVA_NETHCONTROLLERS > 1
  ret = irq_attach(priv->irq, tiva_interrupt, NULL);
#else
  ret = irq_attach(TIVA_IRQ_ETHCON, tiva_interrupt, NULL);
#endif
  if (ret != 0)
    {
      /* We could not attach the ISR to the IRQ */

      return -EAGAIN;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->ld_dev, NET_LL_ETHERNET);
  return OK;
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, Ethernet controllers should
 *   be initialized.
 *
 ****************************************************************************/

#if TIVA_NETHCONTROLLERS == 1 && !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
  tiva_ethinitialize(0);
}
#endif

#endif /* CONFIG_NET && CONFIG_TIVA_ETHERNET */
