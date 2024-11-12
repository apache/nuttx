/*****************************************************************************
 * drivers/net/e1000.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/addrenv.h>
#include <nuttx/spinlock.h>

#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/pci/pci.h>
#include <nuttx/net/e1000.h>

#include "e1000.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Packet buffer size */

#define E1000_PKTBUF_SIZE       2048
#define E1000_RCTL_BSIZE        E1000_RCTL_BSIZE_2048

/* TX and RX descriptors */

#define E1000_TX_DESC           256
#define E1000_RX_DESC           256

/* After RX packet is done, we provide free netpkt to the RX descriptor ring.
 * The upper-half network logic is responsible for freeing the RX packets
 * so we need some additional spare netpkt buffers to assure that it's
 * allways possible to allocate the new RX packet in the recevier logic.
 * It's hard to tell how many spare buffers is needed, for now it's set to 8.
 */

#define E1000_TX_QUOTA          (E1000_TX_DESC - 1)
#define E1000_RX_QUOTA          (E1000_RX_DESC + CONFIG_NET_E1000_RXSPARE)

/* NOTE: CONFIG_IOB_ALIGNMENT must match system D-CACHE line size */

#if CONFIG_IOB_NBUFFERS < (E1000_RX_QUOTA + E1000_TX_QUOTA)
#  error CONFIG_IOB_NBUFFERS must be > (E1000_RX_QUOTA + E1000_TX_QUOTA)
#endif

#if CONFIG_IOB_BUFSIZE < E1000_PKTBUF_SIZE
#  error CONFIG_IOB_BUFSIZE must be > E1000_PKTBUF_SIZE
#endif

/* PCI BARs */

#define E1000_MMIO_BAR          0
#define E1000_FLASH_BAR         1
#define E1000_IO_BAR            2
#define E1000_MSIX_BAR          3

/* E1000 interrupts */

#if CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD > 0
#  define E1000_INTERRUPTS      (E1000_IC_LSC)
#else
#  define E1000_INTERRUPTS      (E1000_IC_RXO    | E1000_IC_RXT0 |  \
                                 E1000_IC_RXDMT0 | E1000_IC_LSC |   \
                                 E1000_IC_TXDW)
#endif

/* For MSI-X we allocate all interrupts to MSI-X vector 0 */

#if CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD > 0
#  define E1000_MSIX_INTERRUPTS (E1000_IC_OTHER)
#  define E1000_MSIX_IVAR       (E1000_IVAR_OTHER_EN)
#else
#  define E1000_MSIX_INTERRUPTS (E1000_IC_RXQ0 |   \
                                 E1000_IC_TXQ0 |   \
                                 E1000_IC_OTHER)
#  define E1000_MSIX_IVAR       (E1000_IVAR_RXQ0_EN | \
                                 E1000_IVAR_TXQ0_EN | \
                                 E1000_IVAR_OTHER_EN)
#endif

/* NIC specific Flags */

#define E1000_RESET_BROKEN      (1 << 0)
#define E1000_HAS_MSIX          (1 << 1)

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* Extend default PCI devie type */

struct e1000_type_s
{
  uint32_t desc_align;          /* Descriptor alignment */
  uint32_t mta_regs;            /* MTA registers */
  uint32_t flags;               /* Device flags */
};

/* E1000 private data */

struct e1000_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s dev;

  /* Driver state */

  bool bifup;

  /* Packets list */

  FAR netpkt_t **tx_pkt;
  FAR netpkt_t **rx_pkt;

  /* Descriptors */

  FAR struct e1000_tx_leg_s *tx;
  FAR struct e1000_rx_leg_s *rx;

  size_t tx_now;
  size_t tx_done;
  size_t rx_now;

  /* PCI data */

  FAR struct pci_device_s       *pcidev;
  const FAR struct e1000_type_s *type;
  int                            irq;
  uint64_t                       base;
  uint32_t                       irqs;

#ifdef CONFIG_NET_MCASTGROUP
  /* MTA shadow */

  FAR uint32_t *mta;
#endif
};

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

/* Helpers */

static uint32_t e1000_getreg_mem(FAR struct e1000_driver_s *priv,
                                 unsigned int offset);
static void e1000_putreg_mem(FAR struct e1000_driver_s *priv,
                             unsigned int offset,
                             uint32_t value);
#ifdef CONFIG_DEBUG_NET_INFO
static void e1000_dump_reg(FAR struct e1000_driver_s *priv,
                           FAR const char *msg, unsigned int offset);
static void e1000_dump_mem(FAR struct e1000_driver_s *priv,
                           FAR const char *msg);
#endif

/* Common TX logic */

static int e1000_transmit(FAR struct netdev_lowerhalf_s *dev,
                          FAR netpkt_t *pkt);

/* Interrupt handling */

static FAR netpkt_t *e1000_receive(FAR struct netdev_lowerhalf_s *dev);
static void e1000_txdone(FAR struct netdev_lowerhalf_s *dev);

static void e1000_msi_interrupt(FAR struct e1000_driver_s *priv);
#ifdef CONFIG_PCI_MSIX
static void e1000_msix_interrupt(FAR struct e1000_driver_s *priv);
#endif
static int e1000_interrupt(int irq, FAR void *context, FAR void *arg);

/* NuttX callback functions */

static int e1000_ifup(FAR struct netdev_lowerhalf_s *dev);
static int e1000_ifdown(FAR struct netdev_lowerhalf_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static uint32_t e1000_hashmta(FAR struct e1000_driver_s *priv,
                              FAR const uint8_t *mac);
static int e1000_addmac(FAR struct netdev_lowerhalf_s *dev,
                        FAR const uint8_t *mac);
static int e1000_rmmac(FAR struct netdev_lowerhalf_s *dev,
                       FAR const uint8_t *mac);
#endif

/* Initialization */

static void e1000_disable(FAR struct e1000_driver_s *priv);
static void e1000_enable(FAR struct e1000_driver_s *priv);
static int e1000_initialize(FAR struct e1000_driver_s *priv);
static int e1000_probe(FAR struct pci_device_s *dev);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/* Intel I219 */

static const struct e1000_type_s g_e1000_i219 =
{
  .desc_align = 128,
  .mta_regs   = 32,
  .flags      = E1000_RESET_BROKEN
};

/* Intel 82801IB (QEMU -device e1000) */

static const struct e1000_type_s g_e1000_82540em =
{
  .desc_align = 16,
  .mta_regs   = 128,
  .flags      = 0
};

/* Intel 82574L (QEMU -device e1000e) */

static const struct e1000_type_s g_e1000_82574l =
{
  .desc_align = 128,
  .mta_regs   = 128,
  .flags      = E1000_HAS_MSIX
};

static const struct pci_device_id_s g_e1000_id_table[] =
{
  {
    PCI_DEVICE(0x8086, 0x1a1e),
    .driver_data = (uintptr_t)&g_e1000_i219
  },
  {
    PCI_DEVICE(0x8086, 0x100e),
    .driver_data = (uintptr_t)&g_e1000_82540em
  },
  {
    PCI_DEVICE(0x8086, 0x10d3),
    .driver_data = (uintptr_t)&g_e1000_82574l
  },
  { }
};

static struct pci_driver_s g_e1000_drv =
{
  .id_table = g_e1000_id_table,
  .probe    = e1000_probe,
};

static const struct netdev_ops_s g_e1000_ops =
{
  .ifup     = e1000_ifup,
  .ifdown   = e1000_ifdown,
  .transmit = e1000_transmit,
  .receive  = e1000_receive,
#ifdef CONFIG_NET_MCASTGROUP
  .addmac   = e1000_addmac,
  .rmmac    = e1000_rmmac,
#endif
#if CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD > 0
  .reclaim  = e1000_txdone,
#endif
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: e1000_getreg_mem
 *****************************************************************************/

static uint32_t e1000_getreg_mem(FAR struct e1000_driver_s *priv,
                                 unsigned int offset)
{
  uintptr_t addr = priv->base + offset;
  return *((FAR volatile uint32_t *)addr);
}

/*****************************************************************************
 * Name: e1000_putreg_mem
 *****************************************************************************/

static void e1000_putreg_mem(FAR struct e1000_driver_s *priv,
                             unsigned int offset,
                             uint32_t value)
{
  uintptr_t addr = priv->base + offset;
  *((FAR volatile uint32_t *)addr) = value;
}

#ifdef CONFIG_DEBUG_NET_INFO
/*****************************************************************************
 * Name: e1000_dump_reg
 *****************************************************************************/

static void e1000_dump_reg(FAR struct e1000_driver_s *priv,
                           FAR const char *msg, unsigned int offset)
{
  ninfo("\t%s:\t\t0x%" PRIx32 "\n", msg, e1000_getreg_mem(priv, offset));
}

/*****************************************************************************
 * Name: e1000_dump_mem
 *****************************************************************************/

static void e1000_dump_mem(FAR struct e1000_driver_s *priv,
                           FAR const char *msg)
{
  ninfo("Dump: %s\n", msg);

  ninfo("General registers:\n");
  e1000_dump_reg(priv, "CTRL", E1000_CTRL);
  e1000_dump_reg(priv, "STATUS", E1000_STATUS);

  ninfo("Interrupt registers:\n");
  e1000_dump_reg(priv, "ICS", E1000_ICS);
  e1000_dump_reg(priv, "IMS", E1000_IMS);
  e1000_dump_reg(priv, "IVAR", E1000_IVAR);

  ninfo("Transmit registers:\n");
  e1000_dump_reg(priv, "TCTL", E1000_TCTL);
  e1000_dump_reg(priv, "TIPG", E1000_TIPG);
  e1000_dump_reg(priv, "AIT", E1000_AIT);
  e1000_dump_reg(priv, "TDBAL", E1000_TDBAL);
  e1000_dump_reg(priv, "TDBAH", E1000_TDBAH);
  e1000_dump_reg(priv, "TDLEN", E1000_TDLEN);
  e1000_dump_reg(priv, "TDH", E1000_TDH);
  e1000_dump_reg(priv, "TDT", E1000_TDT);
  e1000_dump_reg(priv, "TARC", E1000_TARC);
  e1000_dump_reg(priv, "TIDV", E1000_TIDV);
  e1000_dump_reg(priv, "TXDCTL", E1000_TXDCTL);
  e1000_dump_reg(priv, "TADV", E1000_TADV);

  ninfo("Receive registers:\n");
  e1000_dump_reg(priv, "RCTL", E1000_RCTL);
  e1000_dump_reg(priv, "RDBAL", E1000_RDBAL);
  e1000_dump_reg(priv, "RDBAH", E1000_RDBAH);
  e1000_dump_reg(priv, "RDLEN", E1000_RDLEN);
  e1000_dump_reg(priv, "RDH", E1000_RDH);
  e1000_dump_reg(priv, "RDT", E1000_RDT);
  e1000_dump_reg(priv, "RDTR", E1000_RDTR);
  e1000_dump_reg(priv, "RXDCTL", E1000_RXDCTL);
  e1000_dump_reg(priv, "RADV", E1000_RADV);
  e1000_dump_reg(priv, "RSRPD", E1000_RSRPD);
  e1000_dump_reg(priv, "RAID", E1000_RAID);
  e1000_dump_reg(priv, "RXCSUM", E1000_RXCSUM);
  e1000_dump_reg(priv, "RFCTL", E1000_RFCTL);
  e1000_dump_reg(priv, "RAL", E1000_RAL);
  e1000_dump_reg(priv, "RAH", E1000_RAH);

  ninfo("Statistic registers:\n");
  e1000_dump_reg(priv, "CRCERRS", E1000_CRCERRS);
  e1000_dump_reg(priv, "ALGNERRC", E1000_ALGNERRC);
  e1000_dump_reg(priv, "RXERRC", E1000_RXERRC);
  e1000_dump_reg(priv, "MPC", E1000_MPC);
  e1000_dump_reg(priv, "SCC", E1000_SCC);
  e1000_dump_reg(priv, "ECOL", E1000_ECOL);
  e1000_dump_reg(priv, "MCC", E1000_MCC);
  e1000_dump_reg(priv, "LATECOL", E1000_LATECOL);
  e1000_dump_reg(priv, "COLC", E1000_COLC);
  e1000_dump_reg(priv, "DC", E1000_DC);
  e1000_dump_reg(priv, "TNCRS", E1000_TNCRS);
  e1000_dump_reg(priv, "CEXTERR", E1000_CEXTERR);
  e1000_dump_reg(priv, "RLEC", E1000_RLEC);
  e1000_dump_reg(priv, "XONRXC", E1000_XONRXC);
  e1000_dump_reg(priv, "XONTXC", E1000_XONTXC);
  e1000_dump_reg(priv, "XOFFRXC", E1000_XOFFRXC);
  e1000_dump_reg(priv, "XOFFTXC", E1000_XOFFTXC);
  e1000_dump_reg(priv, "FCRUC", E1000_FCRUC);
  e1000_dump_reg(priv, "PRC64", E1000_PRC64);
  e1000_dump_reg(priv, "PRC127", E1000_PRC127);
  e1000_dump_reg(priv, "PRC255", E1000_PRC255);
  e1000_dump_reg(priv, "PRC511", E1000_PRC511);
  e1000_dump_reg(priv, "PRC1023", E1000_PRC1023);
  e1000_dump_reg(priv, "PRC1522", E1000_PRC1522);
  e1000_dump_reg(priv, "GPRC", E1000_GPRC);
  e1000_dump_reg(priv, "BPRC", E1000_BPRC);
  e1000_dump_reg(priv, "MPRC", E1000_MPRC);
  e1000_dump_reg(priv, "GPTC", E1000_GPTC);
  e1000_dump_reg(priv, "GORCL", E1000_GORCL);
  e1000_dump_reg(priv, "GORCH", E1000_GORCH);
  e1000_dump_reg(priv, "GOTCL", E1000_GOTCL);
  e1000_dump_reg(priv, "GOTCH", E1000_GOTCH);
  e1000_dump_reg(priv, "RNBC", E1000_RNBC);
  e1000_dump_reg(priv, "RUC", E1000_RUC);
  e1000_dump_reg(priv, "RFC", E1000_RFC);
  e1000_dump_reg(priv, "ROC", E1000_ROC);
  e1000_dump_reg(priv, "RJC", E1000_RJC);
  e1000_dump_reg(priv, "MNGPRC", E1000_MNGPRC);
  e1000_dump_reg(priv, "MPDC", E1000_MPDC);
  e1000_dump_reg(priv, "MPTC", E1000_MPTC);
  e1000_dump_reg(priv, "TORL", E1000_TORL);
  e1000_dump_reg(priv, "TORH", E1000_TORH);
  e1000_dump_reg(priv, "TOT", E1000_TOT);
  e1000_dump_reg(priv, "TPR", E1000_TPR);
  e1000_dump_reg(priv, "TPT", E1000_TPT);
  e1000_dump_reg(priv, "PTC64", E1000_PTC64);
  e1000_dump_reg(priv, "PTC127", E1000_PTC127);
  e1000_dump_reg(priv, "PTC255", E1000_PTC255);
  e1000_dump_reg(priv, "PTC511", E1000_PTC511);
  e1000_dump_reg(priv, "PTC1023", E1000_PTC1023);
  e1000_dump_reg(priv, "PTC1522", E1000_PTC1522);
  e1000_dump_reg(priv, "MPTC", E1000_MPTC);
  e1000_dump_reg(priv, "BPTC", E1000_BPTC);
  e1000_dump_reg(priv, "TSCTC", E1000_TSCTC);
  e1000_dump_reg(priv, "TSCTFC", E1000_TSCTFC);
  e1000_dump_reg(priv, "IAC", E1000_IAC);

  ninfo("Management registers:\n");
  e1000_dump_reg(priv, "WUC", E1000_WUC);
  e1000_dump_reg(priv, "WUFC", E1000_WUFC);
  e1000_dump_reg(priv, "WUS", E1000_WUS);
  e1000_dump_reg(priv, "MFUTP01", E1000_MFUTP01);
  e1000_dump_reg(priv, "MFUTP23", E1000_MFUTP23);
  e1000_dump_reg(priv, "IPAV", E1000_IPAV);

  ninfo("Diagnostic registers:\n");
  e1000_dump_reg(priv, "POEMB", E1000_POEMB);
  e1000_dump_reg(priv, "RDFH", E1000_RDFH);
  e1000_dump_reg(priv, "FDFT", E1000_FDFT);
  e1000_dump_reg(priv, "RDFHS", E1000_RDFHS);
  e1000_dump_reg(priv, "RDFTS", E1000_RDFTS);
  e1000_dump_reg(priv, "RDFPC", E1000_RDFPC);
  e1000_dump_reg(priv, "TDFH", E1000_TDFH);
  e1000_dump_reg(priv, "TDFT", E1000_TDFT);
  e1000_dump_reg(priv, "TDFHS", E1000_TDFHS);
  e1000_dump_reg(priv, "TDFTS", E1000_TDFTS);
  e1000_dump_reg(priv, "TDFPC", E1000_TDFPC);
  e1000_dump_reg(priv, "PBM", E1000_PBM);
  e1000_dump_reg(priv, "PBS", E1000_PBS);
}
#endif

/*****************************************************************************
 * Name: e1000_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   Return OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static int e1000_transmit(FAR struct netdev_lowerhalf_s *dev,
                          FAR netpkt_t *pkt)
{
  FAR struct e1000_driver_s *priv = (FAR struct e1000_driver_s *)dev;
  uint64_t                   pa   = 0;
  int                        desc = priv->tx_now;
  size_t                     len  = netpkt_getdatalen(dev, pkt);

  ninfo("transmit\n");

  /* Check the send length */

  if (len > E1000_PKTBUF_SIZE)
    {
      nerr("net transmit buffer too large\n");
      return -EINVAL;
    }

  /* Store TX packet reference */

  priv->tx_pkt[priv->tx_now] = pkt;

  /* Prepare next TX descriptor */

  priv->tx_now = (priv->tx_now + 1) % E1000_TX_DESC;

  /* Setup TX descriptor */

  pa = up_addrenv_va_to_pa(netpkt_getdata(dev, pkt));

  priv->tx[desc].addr   = pa;
  priv->tx[desc].len    = len;
  priv->tx[desc].cmd    = (E1000_TDESC_CMD_EOP | E1000_TDESC_CMD_IFCS |
                           E1000_TDESC_CMD_RS | E1000_TDESC_CMD_RPS);
  priv->tx[desc].cso    = 0;
  priv->tx[desc].status = 0;

  SP_DSB();

  /* Update TX tail */

  e1000_putreg_mem(priv, E1000_TDT, priv->tx_now);

  ninfodumpbuffer("Transmitted:", netpkt_getdata(dev, pkt), len);

  return OK;
}

/*****************************************************************************
 * Name: e1000_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   A pointer to received packet
 *
 * Assumptions:
 *   The network is locked.
 *
 *****************************************************************************/

static FAR netpkt_t *e1000_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct e1000_driver_s *priv = (FAR struct e1000_driver_s *)dev;
  FAR netpkt_t              *pkt  = NULL;
  FAR struct e1000_rx_leg_s *rx   = NULL;
  int                        desc = desc = priv->rx_now;

  /* Get RX descriptor and RX packet */

  rx = &priv->rx[desc];
  pkt = priv->rx_pkt[desc];

  /* Check if descriptor done */

  if (!(rx->status & E1000_RDESC_STATUS_DD))
    {
      return NULL;
    }

  /* Next descriptor */

  priv->rx_now = (priv->rx_now + 1) % E1000_RX_DESC;

  /* Allocate new rx packet */

  priv->rx_pkt[desc] = netpkt_alloc(dev, NETPKT_RX);
  if (priv->rx_pkt[desc] == NULL)
    {
      nerr("alloc pkt_new failed\n");
      PANIC();
    }

  /* Set packet length */

  netpkt_setdatalen(dev, pkt, rx->len);

  /* Store new packet in RX descriptor ring */

  rx->addr   = up_addrenv_va_to_pa(
               netpkt_getdata(dev, priv->rx_pkt[desc]));
  rx->len    = 0;
  rx->status = 0;

  /* Update RX tail */

  e1000_putreg_mem(priv, E1000_RDT, desc);

  /* Handle errros */

  if (rx->errors)
    {
      nerr("RX error reported (%"PRIu8")\n", rx->errors);
      NETDEV_RXERRORS(&priv->dev.netdev);
      netpkt_free(dev, pkt, NETPKT_RX);
      return NULL;
    }

  return pkt;
}

/*****************************************************************************
 * Name: e1000_txdone
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
 *****************************************************************************/

static void e1000_txdone(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct e1000_driver_s *priv = (FAR struct e1000_driver_s *)dev;

  while (priv->tx_now != priv->tx_done)
    {
      if (priv->tx[priv->tx_done].status == 0)
        {
          break;
        }

      if (!(priv->tx[priv->tx_done].status & E1000_TDESC_STATUS_DD))
        {
          nerr("tx failed: 0x%" PRIx32 "\n", priv->tx[priv->tx_done].status);
          NETDEV_TXERRORS(&priv->dev.netdev);
        }

      /* Free net packet */

      netpkt_free(dev, priv->tx_pkt[priv->tx_done], NETPKT_TX);

      /* Next descriptor */

      priv->tx_done = (priv->tx_done + 1) % E1000_TX_DESC;
    }

  netdev_lower_txdone(dev);
}

/*****************************************************************************
 * Name: e1000_msi_interupt
 *
 * Description:
 *   Perform MSI/legacy interrupt work
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs on a worker thread.
 *
 *****************************************************************************/

static void e1000_msi_interrupt(FAR struct e1000_driver_s *priv)
{
  uint32_t status;
  uint32_t tmp;

  status = e1000_getreg_mem(priv, E1000_ICR);
  ninfo("irq status = 0x%" PRIx32 "\n", status);

  if (status == 0)
    {
      /* Ignore spurious interrupts */

      return;
    }

  /* Receiver Timer Interrupt or Receive Descriptor Minimum Threshold Reached
   */

  if ((status & E1000_IC_RXT0) || (status & E1000_IC_RXDMT0))
    {
      netdev_lower_rxready(&priv->dev);
    }

  /* Link Status Change */

  if (status & E1000_IC_LSC)
    {
      tmp = e1000_getreg_mem(priv, E1000_STATUS);
      if (tmp & E1000_STATUS_LU)
        {
          ninfo("Link up, status = 0x%x\n", tmp);
          netdev_lower_carrier_on(&priv->dev);
        }
      else
        {
          ninfo("Link down\n");
          netdev_lower_carrier_off(&priv->dev);
        }
    }

  /* Receiver Overrun */

  if (status & E1000_IC_RXO)
    {
      nerr("Receiver Overrun\n");
      netdev_lower_rxready(&priv->dev);
    }

  /* Transmit Descriptor Written Back */

  if (status & E1000_IC_TXDW)
    {
      e1000_txdone(&priv->dev);
    }
}

#ifdef CONFIG_PCI_MSIX
/*****************************************************************************
 * Name: e1000_msix_interrupt
 *
 * Description:
 *   Perform MSI-X interrupt work
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs on a worker thread.
 *
 *****************************************************************************/

static void e1000_msix_interrupt(FAR struct e1000_driver_s *priv)
{
  uint32_t status;

  status = e1000_getreg_mem(priv, E1000_ICR);
  ninfo("irq status = 0x%" PRIx32 "\n", status);

  if (status == 0)
    {
      /* Ignore spurious interrupts */

      return;
    }

  /* Receive Queue 0 Interrupt */

  if (status & E1000_IC_RXQ0)
    {
      netdev_lower_rxready(&priv->dev);
    }

  /* Link Status Change */

  if (status & E1000_IC_LSC)
    {
      if (e1000_getreg_mem(priv, E1000_STATUS) & E1000_STATUS_LU)
        {
          ninfo("Link up\n");
          netdev_lower_carrier_on(&priv->dev);
        }
      else
        {
          ninfo("Link down\n");
          netdev_lower_carrier_off(&priv->dev);
        }
    }

  /* Receiver Overrun */

  if (status & E1000_IC_RXO)
    {
      nerr("Receiver Overrun\n");
      netdev_lower_rxready(&priv->dev);
    }

  /* Transmit Descriptor Written Back */

  if (status & E1000_IC_TXQ0)
    {
      e1000_txdone(&priv->dev);
    }
}
#endif

/*****************************************************************************
 * Name: e1000_interrupt
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
 *   Runs in the context of a the Ethernet interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 *****************************************************************************/

static int e1000_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct e1000_driver_s *priv = (FAR struct e1000_driver_s *)arg;

  DEBUGASSERT(priv != NULL);

  ninfo("interrupt!\n");

  /* Schedule to perform the interrupt processing on the worker thread. */

#ifdef CONFIG_PCI_MSIX
  if (priv->type->flags & E1000_HAS_MSIX)
    {
      e1000_msix_interrupt(priv);
    }
  else
#endif
    {
      e1000_msi_interrupt(priv);
    }

  return OK;
}

/*****************************************************************************
 * Name: e1000_ifup
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
 *   The network is locked.
 *
 *****************************************************************************/

static int e1000_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct e1000_driver_s *priv = (FAR struct e1000_driver_s *)dev;
  irqstate_t flags;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %u.%u.%u.%u\n",
        ip4_addr1(dev->netdev.d_ipaddr), ip4_addr2(dev->netdev.d_ipaddr),
        ip4_addr3(dev->netdev.d_ipaddr), ip4_addr4(dev->netdev.d_ipaddr));
#endif

#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->netdev.d_ipv6addr[0], dev->netdev.d_ipv6addr[1],
        dev->netdev.d_ipv6addr[2], dev->netdev.d_ipv6addr[3],
        dev->netdev.d_ipv6addr[4], dev->netdev.d_ipv6addr[5],
        dev->netdev.d_ipv6addr[6], dev->netdev.d_ipv6addr[7]);
#endif

  /* Enable the Ethernet */

  flags = enter_critical_section();
  e1000_enable(priv);
  priv->bifup = true;
  leave_critical_section(flags);

  return OK;
}

/*****************************************************************************
 * Name: e1000_ifdown
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
 *   The network is locked.
 *
 *****************************************************************************/

static int e1000_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct e1000_driver_s *priv = (FAR struct e1000_driver_s *)dev;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the e1000_ifup() always
   * successfully brings the interface back up.
   */

  e1000_disable(priv);

  /* Mark the device "down" */

  priv->bifup = false;
  leave_critical_section(flags);
  return OK;
}

#ifdef CONFIG_NET_MCASTGROUP
/*****************************************************************************
 * Name: e1000_hashmta
 *
 * Note: This logic is based on freeBSD e1000 implementation
 *
 *****************************************************************************/

static uint32_t e1000_hashmta(FAR struct e1000_driver_s *priv,
                              FAR const uint8_t *mac)
{
  uint32_t hash_mask = 0;
  uint8_t  bit_shift = 0;

  /* Register count multiplied by bits per register */

  hash_mask = (priv->type->mta_regs * 32) - 1;

/* For a mc_filter_type of 0, bit_shift is the number of left-shifts
 * where 0xFF would still fall within the hash mask.
 */

  while (hash_mask >> bit_shift != 0xff)
    {
      bit_shift++;
    }

  /* bit_shift += 0 because we have MO set to 0 */

  return hash_mask & ((mac[4] >> (8 - bit_shift)) | (mac[5] << bit_shift));
}

/*****************************************************************************
 * Name: e1000_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 *****************************************************************************/

static int e1000_addmac(FAR struct netdev_lowerhalf_s *dev,
                        FAR const uint8_t *mac)
{
  FAR struct e1000_driver_s *priv = (FAR struct e1000_driver_s *)dev;
  uint16_t                   hash = 0;
  uint8_t                    row  = 0;
  uint8_t                    bit  = 0;
  int                        i    = 0;

  hash = e1000_hashmta(priv, mac);
  bit = hash & 31;
  row = (hash >> 5) & (priv->type->mta_regs - 1);

  /* Bits 4:0 indicate bit in row word */

  priv->mta[row] |= (1 << bit);

  /* Replace the entire MTA */

  for (i = priv->type->mta_regs - 1; i >= 0; i--)
    {
      e1000_putreg_mem(priv, E1000_MTA + (i << 2), priv->mta[i]);
    }

  return OK;
}

/*****************************************************************************
 * Name: e1000_rmmac
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
 *****************************************************************************/

static int e1000_rmmac(FAR struct netdev_lowerhalf_s *dev,
                       FAR const uint8_t *mac)
{
  FAR struct e1000_driver_s *priv = (FAR struct e1000_driver_s *)dev;
  uint16_t                   hash = 0;
  uint8_t                    row  = 0;
  uint8_t                    bit  = 0;
  int                        i    = 0;

  hash = e1000_hashmta(priv, mac);
  bit = hash & 31;
  row = (hash >> 5) & (priv->type->mta_regs - 1);

  /* Bits 4:0 indicate bit in row word */

  priv->mta[row] &= ~(1 << bit);

  /* Replace the entire MTA */

  for (i = priv->type->mta_regs - 1; i >= 0; i--)
    {
      e1000_putreg_mem(priv, E1000_MTA + (i << 2), priv->mta[i]);
    }

  return OK;
}
#endif  /* CONFIG_NET_MCASTGROUP */

/*****************************************************************************
 * Name: e1000_disable
 *
 * Description:
 *   Reset device to known state.
 *
 *****************************************************************************/

static void e1000_disable(FAR struct e1000_driver_s *priv)
{
  int i = 0;

  /* Reset Tx tail */

  e1000_putreg_mem(priv, E1000_TDH, 0);
  e1000_putreg_mem(priv, E1000_TDT, 0);

  /* Reset Rx tail */

  e1000_putreg_mem(priv, E1000_RDH, 0);
  e1000_putreg_mem(priv, E1000_RDT, 0);

  /* Disable interrupts */

  e1000_putreg_mem(priv, E1000_IMC, priv->irqs);
  up_disable_irq(priv->irq);

  /* Disable Transmiter */

  e1000_putreg_mem(priv, E1000_TCTL, 0);

  /* Disable Receiver */

  e1000_putreg_mem(priv, E1000_RCTL, 0);

  /* Free RX packets */

  for (i = 0; i < E1000_RX_DESC; i += 1)
    {
      netpkt_free(&priv->dev, priv->rx_pkt[i], NETPKT_RX);
    }
}

/*****************************************************************************
 * Name: e1000_enable
 *
 * Description:
 *   Enable device.
 *
 *****************************************************************************/

static void e1000_enable(FAR struct e1000_driver_s *priv)
{
  FAR struct netdev_lowerhalf_s *dev = (FAR struct netdev_lowerhalf_s *)priv;
  int      i         = 0;
  uint64_t pa        = 0;
  uint32_t regval    = 0;
  uint32_t fextnvm11 = 0;

  /* Errata for i219 - this is undocumented by Intel bug.
   * Linux and BSD drivers include this workaround which was optimised by
   * ipxe but no one except Intel engineers knows exactly wtha this does.
   * For some exaplanaition look at 546dd51de8459d4d09958891f426fa2c73ff090d
   * commit in ipxe.
   */

  if (priv->type->flags & E1000_RESET_BROKEN)
    {
      fextnvm11 = e1000_getreg_mem(priv, E1000_FEXTNVM11);
      fextnvm11 |= E1000_FEXTNVM11_MAGIC;
      e1000_putreg_mem(priv, E1000_FEXTNVM11, fextnvm11);
    }

  /* Reset Multicast Table Array */

  for (i = 0; i < priv->type->mta_regs; i++)
    {
      e1000_putreg_mem(priv, E1000_MTA + (i << 2), 0);
    }

  /* Allocate RX packets */

  for (i = 0; i < E1000_RX_DESC; i += 1)
    {
      priv->rx_pkt[i] = netpkt_alloc(dev, NETPKT_RX);
      if (priv->rx_pkt[i] == NULL)
        {
          nerr("alloc rx_pkt failed\n");
          PANIC();
        }

      /* Configure RX descriptor */

      priv->rx[i].addr   = up_addrenv_va_to_pa(
                           netpkt_getdata(dev, priv->rx_pkt[i]));
      priv->rx[i].len    = 0;
      priv->rx[i].status = 0;
    }

  /* Setup TX descriptor */

  /* The address passed to the NIC must be physical */

  pa = up_addrenv_va_to_pa(priv->tx);

  regval = (uint32_t)pa;
  e1000_putreg_mem(priv, E1000_TDBAL, regval);
  regval = (uint32_t)(pa >> 32);
  e1000_putreg_mem(priv, E1000_TDBAH, regval);

  regval = E1000_TX_DESC * sizeof(struct e1000_tx_leg_s);
  e1000_putreg_mem(priv, E1000_TDLEN, regval);

  priv->tx_now  = 0;

  /* Reset TX tail */

  e1000_putreg_mem(priv, E1000_TDH, 0);
  e1000_putreg_mem(priv, E1000_TDT, 0);

  /* Setup RX descriptor */

  /* The address passed to the NIC must be physical */

  pa = up_addrenv_va_to_pa(priv->rx);

  regval = (uint32_t)pa;
  e1000_putreg_mem(priv, E1000_RDBAL, regval);
  regval = (uint32_t)(pa >> 32);
  e1000_putreg_mem(priv, E1000_RDBAH, regval);

  regval = E1000_RX_DESC * sizeof(struct e1000_rx_leg_s);
  e1000_putreg_mem(priv, E1000_RDLEN, regval);

  priv->rx_now = 0;

  /* Reset RX tail */

  e1000_putreg_mem(priv, E1000_RDH, 0);
  e1000_putreg_mem(priv, E1000_RDT, E1000_RX_DESC);

  /* Enable interrupts */

  e1000_putreg_mem(priv, E1000_IMS, priv->irqs);
  up_enable_irq(priv->irq);

  /* Set link up and auto-detect speed */

  regval = E1000_CTRL_SLU | E1000_CTRL_ASDE;
  e1000_putreg_mem(priv, E1000_CTRL, regval);

  /* Setup and enable Transmiter */

  regval = e1000_getreg_mem(priv, E1000_TCTL);
  regval |= E1000_TCTL_EN | E1000_TCTL_PSP;
  e1000_putreg_mem(priv, E1000_TCTL, regval);

  /* Setup and enable Receiver */

  regval = E1000_RCTL_EN | E1000_RCTL_MPE |
           (E1000_RCTL_BSIZE << E1000_RCTL_BSIZE_SHIFT) |
           (E1000_RCTL_MO_4736 << E1000_RCTL_MO_SHIFT);
#ifdef CONFIG_NET_PROMISCUOUS
  regval |= E1000_RCTL_UPE | E1000_RCTL_MPE;
#endif
  e1000_putreg_mem(priv, E1000_RCTL, regval);

  /* REVISIT: Set granuality to Descriptors */

  regval = e1000_getreg_mem(priv, E1000_RXDCTL);
  regval |= E1000_RXDCTL_GRAN;
  e1000_putreg_mem(priv, E1000_RXDCTL, regval);

#ifdef CONFIG_DEBUG_NET_INFO
  /* Dump memory */

  e1000_dump_mem(priv, "enabled");
#endif
}

/*****************************************************************************
 * Name: e1000_initialize
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int e1000_initialize(FAR struct e1000_driver_s *priv)
{
  uint32_t regval = 0;
  uint64_t mac    = 0;
  int      ret    = OK;

  /* Make sure that interrupts are masked */

  e1000_putreg_mem(priv, E1000_IMC, 0xffffffff);

  /* Allocate MSI */

  ret = pci_alloc_irq(priv->pcidev, &priv->irq, 1);
  if (ret != 1)
    {
      nerr("Failed to allocate MSI %d\n", ret);
      return ret;
    }

  /* Connect MSI */

  ret = pci_connect_irq(priv->pcidev, &priv->irq, 1);
  if (ret < 0)
    {
      nerr("Failed to connect MSI %d\n", ret);
      pci_release_irq(priv->pcidev, &priv->irq, 1);

      /* Get legacy IRQ if MSI not supported */

      priv->irq = pci_get_irq(priv->pcidev);
    }

  /* Attach interupts */

  irq_attach(priv->irq, e1000_interrupt, priv);

#ifdef CONFIG_PCI_MSIX
  /* Configure MSI-X */

  if (priv->type->flags & E1000_HAS_MSIX)
    {
      e1000_putreg_mem(priv, E1000_IVAR, E1000_MSIX_IVAR);
      priv->irqs = E1000_MSIX_INTERRUPTS;
    }
  else
#endif
    {
      priv->irqs = E1000_INTERRUPTS;
    }

  /* Get MAC if valid */

  regval = e1000_getreg_mem(priv, E1000_RAH);
  if (regval & E1000_RAH_AV)
    {
      mac = ((uint64_t)regval & E1000_RAH_RAH_MASK) << 32;
      mac |= e1000_getreg_mem(priv, E1000_RAL);
      memcpy(&priv->dev.netdev.d_mac.ether, &mac, sizeof(struct ether_addr));
    }
  else
    {
      nwarn("Receive Address not vaild!\n");
    }

  return OK;
}

/*****************************************************************************
 * Name: e1000_probe
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int e1000_probe(FAR struct pci_device_s *dev)
{
  FAR const struct e1000_type_s *type   = NULL;
  FAR struct e1000_driver_s     *priv   = NULL;
  FAR struct netdev_lowerhalf_s *netdev = NULL;
  int                            ret    = -ENOMEM;

  /* Get type data associated with this PCI device card */

  type = (FAR const struct e1000_type_s *)dev->id->driver_data;

  /* Not found private data */

  if (type == NULL)
    {
      return -ENODEV;
    }

  /* Allocate the interface structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return ret;
    }

  priv->pcidev = dev;

  /* Allocate TX descriptors */

  priv->tx = kmm_memalign(type->desc_align,
                          E1000_TX_DESC * sizeof(struct e1000_tx_leg_s));
  if (priv->tx == NULL)
    {
      nerr("alloc tx failed\n");
      goto errout;
    }

  /* Allocate RX descriptors */

  priv->rx = kmm_memalign(type->desc_align,
                          E1000_RX_DESC * sizeof(struct e1000_rx_leg_s));
  if (priv->rx == NULL)
    {
      nerr("alloc rx failed\n");
      goto errout;
    }

  /* Allocate TX packet pointer array */

  priv->tx_pkt = kmm_zalloc(E1000_TX_DESC * sizeof(netpkt_t *));
  if (priv->tx_pkt == NULL)
    {
      nerr("alloc tx_pkt failed\n");
      goto errout;
    }

  /* Allocate RX packet pointer array */

  priv->rx_pkt = kmm_zalloc(E1000_RX_DESC * sizeof(netpkt_t *));
  if (priv->rx_pkt == NULL)
    {
      nerr("alloc rx_pkt failed\n");
      goto errout;
    }

#ifdef CONFIG_NET_MCASTGROUP
  /* Allocate MTA shadow */

  priv->mta = kmm_zalloc(type->mta_regs);
  if (priv->mta == NULL)
    {
      nerr("alloc mta failed\n");
      goto errout;
    }
#endif

  /* Get devices */

  netdev     = &priv->dev;
  priv->type = type;

  pci_set_master(dev);
  pciinfo("Enabled bus mastering\n");
  pci_enable_device(dev);
  pciinfo("Enabled memory resources\n");

  /* If the BAR is MMIO then it must be mapped */

  priv->base = (uintptr_t)pci_map_bar(dev, E1000_MMIO_BAR);
  if (!priv->base)
    {
      pcierr("Not found MMIO control bar\n");
      goto errout;
    }

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  ret = e1000_initialize(priv);
  if (ret != OK)
    {
      nerr("e1000_initialize failed %d\n", ret);
      goto errout;
    }

  /* Register the network device */

  netdev->quota[NETPKT_TX] = E1000_TX_QUOTA;
  netdev->quota[NETPKT_RX] = E1000_RX_QUOTA;
  netdev->ops = &g_e1000_ops;

  return netdev_lower_register(netdev, NET_LL_ETHERNET);

errout:
  kmm_free(priv->tx);
  kmm_free(priv->rx);
#ifdef CONFIG_NET_MCASTGROUP
  kmm_free(priv->mta);
#endif
  kmm_free(priv);

  return ret;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: pci_e1000_init
 *
 * Description:
 *   Register a pci driver
 *
 *****************************************************************************/

int pci_e1000_init(void)
{
  return pci_register_driver(&g_e1000_drv);
}
