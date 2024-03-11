/****************************************************************************
 * arch/xtensa/src/esp32/esp32_openeth.c
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
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <syslog.h>
#include <debug.h>

#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>
#include <netinet/if_ether.h>

#ifdef CONFIG_NET_PKT
#include <nuttx/net/pkt.h>
#endif

#include "hardware/esp32_soc.h"
#include "esp32_irq.h"

#ifdef CONFIG_ESP32_OPENETH

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* We need to have the work queue to handle interrupts */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#error Worker thread support is required (CONFIG_SCHED_WORKQUEUE)
#endif

/* These are the register definitions for the OpenCores Ethernet MAC. */

/* DMA buffers configuration */
#define DMA_BUF_SIZE 1600
#define RX_BUF_COUNT CONFIG_ESP32_OPENETH_DMA_RX_BUFFER_NUM
#define TX_BUF_COUNT CONFIG_ESP32_OPENETH_DMA_TX_BUFFER_NUM

/* This driver uses the interrupt source number of the internal EMAC
 * of the ESP32 chip, and uses the same register address base. This of
 * course only works in QEMU, where the OpenCores MAC is mapped to the
 * same register base and to the same interrupt source. This driver does
 * a sanity check that it is not running on the real ESP32 chip, using
 * the EMAC date register.
 */
#define OPENETH_BASE DR_REG_EMAC_BASE

/* OpenCores ethmac registers */
#define OPENETH_MODER_REG (OPENETH_BASE + 0x00)
#define OPENETH_MODER_DEFAULT 0xa000
/* OPENETH_RST: reset the MAC */
#define OPENETH_RST BIT(11)
/* OPENETH_PRO: enable promiscuous mode */
#define OPENETH_PRO BIT(5)
/*  OPENETH_TXEN: enable transmit */
#define OPENETH_TXEN BIT(1)
/*  OPENETH_RXEN: enable receive */
#define OPENETH_RXEN BIT(0)

#define OPENETH_INT_SOURCE_REG (OPENETH_BASE + 0x04)
#define OPENETH_INT_MASK_REG (OPENETH_BASE + 0x08)
/*  These bits apply to INT_SOURCE and INT_MASK registers: */

/*  OPENETH_INT_BUSY: Buffer was received and discarded due
 * to lack of buffers
 */
#define OPENETH_INT_BUSY BIT(4)
/*  OPENETH_INT_RXB: Frame received */
#define OPENETH_INT_RXB BIT(2)
/*  OPENETH_INT_TXB: Frame transmitted */
#define OPENETH_INT_TXB BIT(0)

/*  IPGT, IPGR1, IPGR2 registers are not implemented in QEMU,
 * hence not used here
 */
#define OPENETH_PACKETLEN_REG (OPENETH_BASE + 0x18)
/*  OPENETH_MINFL: minimum frame length */
#define OPENETH_MINFL_S 16
#define OPENETH_MINFL_V 0xffff
#define OPENETH_MINFL_M (OPENETH_MINFL_V << OPENETH_MINFL_S)
/*  OPENETH_MAXFL: maximum frame length */
#define OPENETH_MAXFL_S 0
#define OPENETH_MAXFL_V 0xffff
#define OPENETH_MAXFL_M (OPENETH_MAXFL_V << OPENETH_MAXFL_S)

/*  COLLCONF is not implemented in QEMU */
#define OPENETH_TX_BD_NUM_REG (OPENETH_BASE + 0x20)
/*  CTRLMODER, MIIMODER are not implemented in QEMU */
#define OPENETH_MIICOMMAND_REG (OPENETH_BASE + 0x2c)
/*  OPENETH_WCTRLDATA: write control data */
#define OPENETH_WCTRLDATA BIT(2)
/*  OPENETH_RSTAT: read status */
#define OPENETH_RSTAT BIT(1)
/*  OPENETH_SCANSTAT: scan status */
#define OPENETH_SCANSTAT BIT(0)

#define OPENETH_MIIADDRESS_REG (OPENETH_BASE + 0x30)
/*  OPENETH_RGAD: register address */
#define OPENETH_RGAD_S 8
#define OPENETH_RGAD_V 0x1f
#define OPENETH_RGAD_M (OPENETH_RGAD_V << OPENETH_RGAD_S)
/*  OPENETH_FIAD: PHY address */
#define OPENETH_FIAD_S 0
#define OPENETH_FIAD_V 0x1f
#define OPENETH_FIAD_N (OPENETH_FIAD_V << OPENETH_FIAD_S)

#define OPENETH_MIITX_DATA_REG (OPENETH_BASE + 0x34)
#define OPENETH_MIIRX_DATA_REG (OPENETH_BASE + 0x38)
#define OPENETH_MII_DATA_MASK 0xffff

#define OPENETH_MIISTATUS_REG (OPENETH_BASE + 0x3c)
/*  OPENETH_LINKFAIL: link is down */
#define OPENETH_LINKFAIL BIT(0)

/*  OPENETH_MAC_ADDR0_REG: bytes 2-5 of the MAC address (byte 5 in LSB) */
#define OPENETH_MAC_ADDR0_REG (OPENETH_BASE + 0x40)
/*  OPENETH_MAC_ADDR1_REG: bytes 0-1 of the MAC address (byte 1 in LSB) */
#define OPENETH_MAC_ADDR1_REG (OPENETH_BASE + 0x44)

#define OPENETH_HASH0_ADR_REG (OPENETH_BASE + 0x48)
#define OPENETH_HASH1_ADR_REG (OPENETH_BASE + 0x4c)

/*  Location of the DMA descriptors */
#define OPENETH_DESC_BASE (OPENETH_BASE + 0x400)
/*  Total number of (TX + RX) DMA descriptors */
#define OPENETH_DESC_CNT 128

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lo_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct openeth_driver_s
{
  struct work_s openeth_work; /* For deferring poll work to the work queue */

  int cpuint; /* EMAC interrupt ID */

  int cur_rx_desc;
  int cur_tx_desc;
  uint8_t addr[6];
  uint8_t *rx_buf[RX_BUF_COUNT];
  uint8_t *tx_buf[TX_BUF_COUNT];

  struct work_s rxwork; /* For deferring RX work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev; /* Interface understood by the network */
};

/*  Structures describing TX and RX descriptors. */

/*  The field names are same as in the OpenCores ethmac documentation. */

typedef struct
{
  uint16_t cs : 1;   /* !< Carrier sense lost (flag set by HW) */
  uint16_t df : 1;   /* !< Defer indication (flag set by HW) */
  uint16_t lc : 1;   /* !< Late collision occured (flag set by HW) */
  uint16_t rl : 1;   /* !< TX failed due to retransmission limit (flag set by HW) */
  uint16_t rtry : 4; /* !< Number of retries before the frame was sent (set by HW) */
  uint16_t ur : 1;   /* !< Underrun status (flag set by HW) */
  uint16_t rsv : 2;  /* !< Reserved */
  uint16_t crc : 1;  /* !< Add CRC at the end of the packet */
  uint16_t pad : 1;  /* !< Add padding to the end of short packets */
  uint16_t wr : 1;   /* !< Wrap-around. 0: not the last descriptor in the table, 1: last descriptor. */
  uint16_t irq : 1;  /* !< Generate interrupt after this descriptor is transmitted */
  uint16_t rd : 1;   /* !< Descriptor ready. 0: descriptor owned by SW, 1: descriptor owned by HW. Cleared by HW. */

  uint16_t len; /* !< Number of bytes to be transmitted */
  void *txpnt;  /* !< Pointer to the data to transmit */
} openeth_tx_desc_t;

static_assert(sizeof(openeth_tx_desc_t) == 8,
  "incorrect size of openeth_tx_desc_t");

typedef struct
{
  uint16_t lc : 1;  /* !< Late collision flag */
  uint16_t crc : 1; /* !< RX CRC error flag */
  uint16_t sf : 1;  /* !< Frame shorter than set in PACKETLEN register */
  uint16_t tl : 1;  /* !< Frame longer than set in PACKETLEN register */
  uint16_t dn : 1;  /* !< Dribble nibble (frame length not divisible by 8 bits) flag */
  uint16_t is : 1;  /* !< Invalid symbol flag */
  uint16_t or : 1;  /* !< Overrun flag */
  uint16_t m : 1;   /* !< Frame received because of the promiscuous mode */
  uint16_t rsv : 5; /* !< Reserved */
  uint16_t wr : 1;  /* !< Wrap-around. 0: not the last descriptor in the table, 1: last descriptor. */
  uint16_t irq : 1; /* !< Generate interrupt after this descriptor is transmitted */
  uint16_t e : 1;   /* !< The buffer is empty. 0: descriptor owned by SW, 1: descriptor owned by HW. */

  uint16_t len; /* !< Number of bytes received (filled by HW) */
  void *rxpnt;  /* !< Pointer to the receive buffer */
} openeth_rx_desc_t;

static_assert(sizeof(openeth_rx_desc_t) == 8,
  "incorrect size of openeth_rx_desc_t");

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct openeth_driver_s g_openeth;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* NuttX callback functions */

static int openeth_ifup(FAR struct net_driver_s *dev);
static int openeth_ifdown(FAR struct net_driver_s *dev);
static int openeth_txavail(FAR struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int openeth_addmac(FAR struct net_driver_s *dev,
  FAR const uint8_t *mac);
static int openeth_rmmac(FAR struct net_driver_s *dev,
  FAR const uint8_t *mac);
#endif
static int openeth_transmit(FAR struct net_driver_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline openeth_tx_desc_t *openeth_tx_desc(int idx)
{
  assert(idx < TX_BUF_COUNT);
  return &((openeth_tx_desc_t *)OPENETH_DESC_BASE)[idx];
}

static inline openeth_rx_desc_t *openeth_rx_desc(int idx)
{
  assert(idx < OPENETH_DESC_CNT - TX_BUF_COUNT);
  return &((openeth_rx_desc_t *)OPENETH_DESC_BASE)[idx + TX_BUF_COUNT];
}

static inline void openeth_enable(void)
{
  REG_SET_BIT(OPENETH_MODER_REG, OPENETH_TXEN | OPENETH_RXEN | OPENETH_PRO);
  REG_SET_BIT(OPENETH_INT_MASK_REG, OPENETH_INT_RXB);
}

static inline void openeth_disable(void)
{
  REG_CLR_BIT(OPENETH_INT_MASK_REG, OPENETH_INT_RXB);
  REG_CLR_BIT(OPENETH_MODER_REG, OPENETH_TXEN | OPENETH_RXEN | OPENETH_PRO);
}

static inline void openeth_reset(void)
{
  REG_SET_BIT(OPENETH_MODER_REG, OPENETH_RST);
  REG_CLR_BIT(OPENETH_MODER_REG, OPENETH_RST);
}

static inline void openeth_init_tx_desc(openeth_tx_desc_t *desc, void *buf)
{
  *desc = (openeth_tx_desc_t)
    {
      .rd = 0,
      .txpnt = buf,
    };
}

static inline void openeth_init_rx_desc(openeth_rx_desc_t *desc, void *buf)
{
  *desc = (openeth_rx_desc_t)
    {
      .e = 1,
      .irq = 1,
      .rxpnt = buf,
    };
}

static inline void openeth_set_tx_desc_cnt(int tx_desc_cnt)
{
  assert(tx_desc_cnt <= OPENETH_DESC_CNT);
  REG_WRITE(OPENETH_TX_BD_NUM_REG, tx_desc_cnt);
}

/****************************************************************************
 * Function: openeth_rx_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   0 on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void openeth_rx_interrupt_work(void *arg)
{
  struct openeth_driver_s *priv = (struct openeth_driver_s *)arg;
  struct net_driver_s *dev = &priv->dev;

  net_lock();

  uint8_t *buffer = NULL;
  uint32_t length = 0;

  while (1)
    {
      openeth_rx_desc_t *desc_ptr = openeth_rx_desc(priv->cur_rx_desc);
      openeth_rx_desc_t desc_val = *desc_ptr;

      if (!desc_val.len)
        break;
      ninfo("desc %d (%p) e=%d len=%d wr=%d",
        priv->cur_rx_desc, desc_ptr, desc_val.e, desc_val.len, desc_val.wr);
      if (desc_val.e)
        {
          nerr("ERROR %d", desc_val.e);
          goto err;
        }

      /* We don't need to copy this anywere. We can just feed it
       * to the stack from where it is
       */

      priv->dev.d_buf = desc_val.rxpnt;
      priv->dev.d_len = desc_val.len;
      struct eth_hdr_s *eth_hdr = (struct eth_hdr_s *)desc_val.rxpnt;

  #ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet tap
       */

      pkt_input(&priv->dev);
  #endif

      /* We only accept IP packets of the configured type and ARP packets
       */

  #ifdef CONFIG_NET_IPv4
      if (eth_hdr->type == HTONS(ETHTYPE_IP))
        {
          /* Receive an IPv4 packet from the network device */

          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field d_len will set to a value > 0
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              openeth_transmit(&priv->dev);
            }
        }
      else
  #endif
  #ifdef CONFIG_NET_IPv6
          if (eth_hdr->type == HTONS(ETHTYPE_IP6))
        {
          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field d_len will set to a value > 0
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              openeth_transmit(&priv->dev);
            }
        }
      else
  #endif
  #ifdef CONFIG_NET_ARP
          if (eth_hdr->type == HTONS(ETHTYPE_ARP))
        {
          /* Handle ARP packet */

          arp_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field d_len will set to a value > 0
           */

          if (priv->dev.d_len > 0)
            {
              /* And send the packet */

              openeth_transmit(&priv->dev);
            }
        }
      else
  #endif
        {
          nerr("ERROR: Dropped, Unknown type: %04x\n", eth_hdr->type);
        }

      desc_val.e = 1;
      desc_val.len = 0;
      *desc_ptr = desc_val;

      priv->cur_rx_desc = (priv->cur_rx_desc + 1) % RX_BUF_COUNT;
    }

err:

  net_unlock();
}

static IRAM_ATTR int openeth_isr_handler(int irq, void *context, void *arg)
{
  struct openeth_driver_s *priv = (struct openeth_driver_s *)arg;

  uint32_t status = REG_READ(OPENETH_INT_SOURCE_REG);

  if (status & OPENETH_INT_RXB)
    {
      work_queue(LPWORK, &priv->rxwork, openeth_rx_interrupt_work, priv, 0);
    }

  if (status & OPENETH_INT_BUSY)
    {
      ninfo("RX frame dropped (0x%x)", status);
    }

  /* Clear interrupt */

  REG_WRITE(OPENETH_INT_SOURCE_REG, status);

  return 0;
}

/****************************************************************************
 * Name: openeth_ifup
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

static int openeth_ifup(FAR struct net_driver_s *dev)
{
  irqstate_t flags;
  FAR struct openeth_driver_s *priv =
    (FAR struct openeth_driver_s *)dev->d_private;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();

  /* Enable TX and RX */

  openeth_enable();

  leave_critical_section(flags);

  netdev_carrier_on(dev);

  return OK;
}

/****************************************************************************
 * Name: openeth_ifdown
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

static int openeth_ifdown(FAR struct net_driver_s *dev)
{
  irqstate_t flags;
  FAR struct openeth_driver_s *priv =
    (FAR struct openeth_driver_s *)dev->d_private;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();

  /* Disable TX and RX */

  openeth_enable();

  leave_critical_section(flags);

  netdev_carrier_off(dev);

  return OK;
}

static int openeth_transmit(FAR struct net_driver_s *dev)
{
  FAR struct openeth_driver_s *priv =
    (FAR struct openeth_driver_s *)dev->d_private;

  int ret = 0;
  uint8_t *buf = priv->dev.d_buf;
  uint16_t length = priv->dev.d_len;

  if (!buf)
    {
      nerr("can't set buf to null");
      goto err;
    }

  if (!length)
    {
      nerr("buf length can't be zero");
      goto err;
    }

  if (length >= DMA_BUF_SIZE * TX_BUF_COUNT)
    {
      nerr("insufficient TX buffer size");
      goto err;
    }

  uint32_t bytes_remaining = length;

  /*  In QEMU, there never is a TX operation in progress,
   * so start with descriptor 0.
   */

  while (bytes_remaining > 0)
    {
      uint32_t will_write = MIN(bytes_remaining, DMA_BUF_SIZE);

      memcpy(priv->tx_buf[priv->cur_tx_desc], buf, will_write);
      openeth_tx_desc_t *desc_ptr = openeth_tx_desc(priv->cur_tx_desc);
      openeth_tx_desc_t desc_val = *desc_ptr;
      desc_val.wr = (priv->cur_tx_desc == TX_BUF_COUNT - 1);
      desc_val.len = will_write;
      desc_val.rd = 1;

      /*  TXEN is already set, and this triggers a TX operation
       * for the descriptor
       */

      ninfo("desc %d (%p) len=%d wr=%d",
        priv->cur_tx_desc, desc_ptr, will_write, desc_val.wr);
      *desc_ptr = desc_val;
      bytes_remaining -= will_write;
      buf += will_write;
      priv->cur_tx_desc = (priv->cur_tx_desc + 1) % TX_BUF_COUNT;
    }

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;
  return OK;

err:
  return ERROR;
}

/****************************************************************************
 * Name: openeth_txavail
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

static int openeth_txavail(FAR struct net_driver_s *dev)
{
  if (IFF_IS_UP(dev->d_flags))
    {
      /* In QEMU, the MAC is always available */

      net_lock();

      /* poll the network for new XMIT data */

      devif_poll(dev, openeth_transmit);

      net_unlock();
    }

  return 0;
}

/****************************************************************************
 * Name: openeth_addmac
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
 *   The default option can allow EMAC receive all multicast frame.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int openeth_addmac(FAR struct net_driver_s *dev,
  FAR const uint8_t *mac)
{
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  return 0;
}
#endif

/****************************************************************************
 * Name: openeth_rmmac
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
 *   The default option can allow EMAC receive all multicast frame.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int openeth_rmmac(FAR struct net_driver_s *dev,
  FAR const uint8_t *mac)
{
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  return 0;
}
#endif

static int openeth_set_addr(uint8_t *addr)
{
  ninfo("set mac\n");
  if (!addr)
    {
      nerr("can't set mac addr to null");
      goto err;
    }

  const uint8_t mac0[4] = {
    addr[5], addr[4], addr[3], addr[2]
  };

  const uint8_t mac1[4] = {
    addr[1], addr[0]
  };

  uint32_t mac0_u32;
  uint32_t mac1_u32;
  memcpy(&mac0_u32, &mac0, 4);
  memcpy(&mac1_u32, &mac1, 4);
  REG_WRITE(OPENETH_MAC_ADDR0_REG, mac0_u32);
  REG_WRITE(OPENETH_MAC_ADDR1_REG, mac1_u32);
  return 0;
err:
  return -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: openeth_initialize
 *
 * Description:
 *   Initialize the openeth driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32_openeth_initialize(void)
{
  int ret;
  FAR struct openeth_driver_s *priv;

  /* Sanity check */

  if (REG_READ(OPENETH_MODER_REG) != OPENETH_MODER_DEFAULT)
    {
      nerr("CONFIG_ESP32_OPENETH should only be used when running in QEMU.");
      nerr("When running the app on the ESP32, use ESP32 EMAC instead.");
      abort();
    }

  /* Get the interface structure associated with this interface number. */

  priv = &g_openeth;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct openeth_driver_s));
  priv->dev.d_ifup = openeth_ifup;       /* I/F up (new IP address) callback */
  priv->dev.d_ifdown = openeth_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = openeth_txavail; /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac = openeth_addmac; /* Add multicast MAC address */
  priv->dev.d_rmmac = openeth_rmmac;   /* Remove multicast MAC address */
#endif
  priv->dev.d_private = priv; /* Used to recover private state from dev */

  /* Allocate DMA buffers */

  for (int i = 0; i < RX_BUF_COUNT; i++)
    {
      priv->rx_buf[i] = calloc(1, DMA_BUF_SIZE);
      if (!(priv->rx_buf[i]))
        {
          nerr("ERROR: Failed allocate RX descriptors\n");
          ret = -ENOMEM;
          goto err;
        }

      openeth_init_rx_desc(openeth_rx_desc(i), priv->rx_buf[i]);
    }

  openeth_rx_desc(RX_BUF_COUNT - 1)->wr = 1;
  priv->cur_rx_desc = 0;

  for (int i = 0; i < TX_BUF_COUNT; i++)
    {
      priv->tx_buf[i] = calloc(1, DMA_BUF_SIZE);
      if (!(priv->tx_buf[i]))
        {
          nerr("ERROR: Failed allocate TX descriptors\n");
          ret = -ENOMEM;
          goto err;
        }

      openeth_init_tx_desc(openeth_tx_desc(i), priv->tx_buf[i]);
    }

  openeth_tx_desc(TX_BUF_COUNT - 1)->wr = 1;
  priv->cur_tx_desc = 0;

  /* Setup interrupts */

  priv->cpuint = esp32_setup_irq(0, ESP32_PERIPH_EMAC,
                                 1, ESP32_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      nerr("ERROR: Failed allocate interrupt\n");
      ret = -ENOMEM;
      goto err;
    }

  /* Initialize the MAC */

  openeth_reset();
  openeth_set_tx_desc_cnt(TX_BUF_COUNT);
  memcpy(priv->dev.d_mac.ether.ether_addr_octet,
    "\x00\x02\x03\x04\x05\x06\x07\x08", ETH_ALEN);
  openeth_set_addr(priv->dev.d_mac.ether.ether_addr_octet);

  /* Attach the interrupt */

  ret = irq_attach(ESP32_IRQ_EMAC, openeth_isr_handler, priv);

  /* Register the device with the OS so that socket IOCTLs can be
   * performed.
   */

  netdev_register(&priv->dev, NET_LL_ETHERNET);

  /* Put the network in the UP state */

  IFF_SET_UP(priv->dev.d_flags);
  return openeth_ifup(&priv->dev);

err:
  nerr("Failed initializing ret = %d", ret);
  abort();
}

#endif /* CONFIG_ESP32_OPENETH */
