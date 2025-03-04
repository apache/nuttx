/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_openeth.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <debug.h>

#include <nuttx/kmalloc.h>
#include <netinet/if_ether.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include <chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These are the register definitions for the OpenCores Ethernet MAC. */

/* DMA buffers configuration */
#define DMA_BUF_SIZE 1600

/* Only need 1 TX buf because packets are transmitted immediately */
#define TX_BUF_COUNT 1

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
/*  COLLCONF is not implemented in QEMU */
#define OPENETH_TX_BD_NUM_REG (OPENETH_BASE + 0x20)
/*  OPENETH_MAC_ADDR0_REG: bytes 2-5 of the MAC address (byte 5 in LSB) */
#define OPENETH_MAC_ADDR0_REG (OPENETH_BASE + 0x40)
/*  OPENETH_MAC_ADDR1_REG: bytes 0-1 of the MAC address (byte 1 in LSB) */
#define OPENETH_MAC_ADDR1_REG (OPENETH_BASE + 0x44)

/*  Location of the DMA descriptors */
#define OPENETH_DESC_BASE (OPENETH_BASE + 0x400)
/*  Total number of (TX + RX) DMA descriptors */
#define OPENETH_DESC_CNT 128

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct openeth_priv_s
{
  struct netdev_lowerhalf_s dev;

  int cpuint; /* EMAC interrupt ID */

  int cur_rx_desc;
  int cur_tx_desc;
  uint8_t *rx_buf[RX_BUF_COUNT];
  uint8_t *tx_buf[TX_BUF_COUNT];
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
 * Private Function Prototypes
 ****************************************************************************/

/* NuttX callback functions */

static int openeth_ifup(struct netdev_lowerhalf_s *dev);
static int openeth_ifdown(struct netdev_lowerhalf_s *dev);
static int openeth_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt);
static netpkt_t *openeth_receive(struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct openeth_priv_s g_openeth;

static const struct netdev_ops_s g_ops =
{
  .ifup     = openeth_ifup,
  .ifdown   = openeth_ifdown,
  .transmit = openeth_transmit,
  .receive  = openeth_receive,
};

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

static IRAM_ATTR int openeth_isr_handler(int irq, void *context, void *arg)
{
  struct netdev_lowerhalf_s *dev = &g_openeth.dev;

  uint32_t status = REG_READ(OPENETH_INT_SOURCE_REG);

  if (status & OPENETH_INT_RXB)
    {
      netdev_lower_rxready(dev);
    }

  if (status & OPENETH_INT_BUSY)
    {
      ninfo("RX frame dropped (0x%x)", status);
    }

  /* Clear interrupt */

  REG_WRITE(OPENETH_INT_SOURCE_REG, status);

  return 0;
}

static netpkt_t *openeth_receive(struct netdev_lowerhalf_s *dev)
{
  netpkt_t *pkt = netpkt_alloc(dev, NETPKT_RX);

  struct openeth_priv_s *priv = &g_openeth;

  if (pkt)
    {
      openeth_rx_desc_t *desc_ptr = openeth_rx_desc(priv->cur_rx_desc);
      openeth_rx_desc_t desc_val = *desc_ptr;

      ninfo("desc %d (%p) e=%d len=%d wr=%d",
        priv->cur_rx_desc, desc_ptr, desc_val.e, desc_val.len, desc_val.wr);

      if (desc_val.e)
        {
          ninfo("descriptor is not owned by HW (e=%d)", desc_val.e);
          goto err;
        }

      if (!desc_val.len)
        {
          nerr("ERROR desc_val.len is zero");
          goto err;
        }

      netpkt_copyin(dev, pkt, desc_val.rxpnt, desc_val.len, 0);

      /* Free up the descriptor */

      desc_val.e = 1;
      desc_val.len = 0;
      *desc_ptr = desc_val;

      priv->cur_rx_desc = (priv->cur_rx_desc + 1) % RX_BUF_COUNT;
    }

  return pkt;

err:
  netpkt_free(dev, pkt, NETPKT_RX);
  return NULL;
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

static int openeth_ifup(struct netdev_lowerhalf_s *dev)
{
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();

  /* Enable TX and RX */

  openeth_enable();

  leave_critical_section(flags);

  netdev_lower_carrier_on(dev);

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

static int openeth_ifdown(struct netdev_lowerhalf_s *dev)
{
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();

  /* Disable TX and RX */

  openeth_enable();

  leave_critical_section(flags);

  netdev_lower_carrier_off(dev);

  return OK;
}

static int openeth_transmit(struct netdev_lowerhalf_s *dev,
                            netpkt_t *pkt)
{
  struct openeth_priv_s *priv = (struct openeth_priv_s *)dev;
  unsigned int len = netpkt_getdatalen(dev, pkt);

  /*  In QEMU, there never is a TX operation in progress */

  /* Copyout the L2 data and transmit. */

  netpkt_copyout(dev, priv->tx_buf[priv->cur_tx_desc], pkt, len, 0);

  /* Do Transmit */

  openeth_tx_desc_t *desc_ptr = openeth_tx_desc(priv->cur_tx_desc);
  openeth_tx_desc_t desc_val = *desc_ptr;

  desc_val.wr = (priv->cur_tx_desc == TX_BUF_COUNT - 1);
  desc_val.len = len;
  desc_val.rd = 1;
  *desc_ptr = desc_val;

  /* Free the buffer and notify the upper layer */

  netpkt_free(dev, pkt, NETPKT_TX);
  netdev_lower_txdone(dev);

  return OK;
}

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
 * Name: esp_openeth_initialize
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

int esp_openeth_initialize(void)
{
  int ret;
  struct openeth_priv_s *priv = &g_openeth;
  struct netdev_lowerhalf_s *dev = &priv->dev;

  /* Sanity check */

  if (REG_READ(OPENETH_MODER_REG) != OPENETH_MODER_DEFAULT)
    {
      nerr("Openeth should only be used when running in QEMU.");
      nerr("When running the app on the real hardware,"
           "use the real MAC instead.");
      abort();
    }

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct openeth_priv_s));

  dev->ops = &g_ops;

  dev->quota[NETPKT_TX] = TX_BUF_COUNT;
  dev->quota[NETPKT_RX] = RX_BUF_COUNT;

  /* Allocate DMA buffers */

  for (int i = 0; i < RX_BUF_COUNT; i++)
    {
      priv->rx_buf[i] = kmm_calloc(1, DMA_BUF_SIZE);
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
      priv->tx_buf[i] = kmm_calloc(1, DMA_BUF_SIZE);
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

  priv->cpuint = OPENETH_SETUP_IRQ(0, OPENETH_PERIPH_MAC,
                                 1, OPENETH_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      nerr("ERROR: Failed allocate interrupt\n");
      ret = -ENOMEM;
      goto err;
    }

  /* Initialize the MAC */

  openeth_reset();
  openeth_set_tx_desc_cnt(TX_BUF_COUNT);
  memcpy(priv->dev.netdev.d_mac.ether.ether_addr_octet,
    "\x00\x02\x03\x04\x05\x06\x07\x08", ETH_ALEN);
  openeth_set_addr(priv->dev.netdev.d_mac.ether.ether_addr_octet);

  /* Attach the interrupt */

  ret = irq_attach(OPENETH_IRQ_MAC, openeth_isr_handler, priv);

  /* Register the device with the OS so that socket IOCTLs can be
   * performed.
   */

  ret = netdev_lower_register(dev, NET_LL_ETHERNET);
  if (ret)
    {
      nerr("ERROR: netdev_lower_register\n");
      goto err;
    }

  /* Put the network in the UP state */

  return openeth_ifup(dev);

err:
  nerr("Failed initializing ret = %d", ret);
  abort();
}
