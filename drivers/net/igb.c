/*****************************************************************************
 * drivers/net/igb.c
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
#include <nuttx/net/igb.h>

#include <arch/barriers.h>

#include "igb.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#if CONFIG_NET_IGB_TXDESC % 8 != 0
#  error CONFIG_NET_IGB_TXDESC must be multiple of 8
#endif

#if CONFIG_NET_IGB_RXDESC % 8 != 0
#  error CONFIG_NET_IGB_RXDESC must be multiple of 8
#endif

/* Packet buffer size */

#define IGB_PKTBUF_SIZE        2048
#define IGB_RCTL_BSIZE         IGB_RCTL_BSIZE_2048

/* TX and RX descriptors */

#define IGB_TX_DESC            CONFIG_NET_IGB_TXDESC
#define IGB_RX_DESC            CONFIG_NET_IGB_RXDESC

/* After RX packet is done, we provide free netpkt to the RX descriptor ring.
 * The upper-half network logic is responsible for freeing the RX packets
 * so we need some additional spare netpkt buffers to assure that it's
 * always possible to allocate the new RX packet in the receiver logic.
 * It's hard to tell how many spare buffers is needed, for now it's set to 8.
 */

#define IGB_TX_QUOTA           IGB_TX_DESC
#define IGB_RX_QUOTA           (IGB_RX_DESC + CONFIG_NET_IGB_RXSPARE)

/* NOTE: CONFIG_IOB_ALIGNMENT must match system D-CACHE line size */

#if CONFIG_IOB_NBUFFERS < IGB_RX_QUOTA + IGB_TX_QUOTA
#  error CONFIG_IOB_NBUFFERS must be > (IGB_RX_QUOTA + IGB_TX_QUOTA)
#endif

#if CONFIG_IOB_BUFSIZE < IGB_PKTBUF_SIZE
#  error CONFIG_IOB_BUFSIZE must be > IGB_PKTBUF_SIZE
#endif

/* PCI BARs */

#define IGB_MMIO_BAR           0
#define IGB_FLASH_BAR          1
#define IGB_IO_BAR             2
#define IGB_MSIX_BAR           3

/* For MSI-X we allocate all interrupts to MSI-X vector 0 */

#define IGB_GPIE_MSIX_SINGLE   (IGB_GPIE_NSICR | IGB_GPIE_EIAME | \
                                IGB_GPIE_PBASUPPORT)
#define IGB_MSIX_IMS           (IGB_IC_TXDW | IGB_IC_LSC | \
                                IGB_IC_RXMISS | IGB_IC_RXDW)
#define IGB_MSIX_EIMS          (IGB_EIMS_NOMSIX_OTHER | \
                                IGB_EIMS_NOMSIX_RXTX0)
#define IGB_MSIX_IVAR0         (IGB_IVAR0_RXQ0_VAL | IGB_IVAR0_TXQ0_VAL)
#define IGB_MSIX_IVARMSC       (IGB_IVARMSC_OTHER_VAL)

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* Extend default PCI devie type */

struct igb_type_s
{
  uint32_t desc_align;          /* Descriptor alignment */
  uint32_t mta_regs;            /* MTA registers */
};

/* IGB private data */

struct igb_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s dev;
  struct work_s work;

  /* Packets list */

  FAR netpkt_t **tx_pkt;
  FAR netpkt_t **rx_pkt;

  /* Descriptors */

  FAR struct igb_tx_leg_s *tx;
  FAR struct igb_rx_leg_s *rx;

  size_t tx_now;
  size_t tx_done;
  size_t rx_now;

  /* PCI data */

  FAR struct pci_device_s     *pcidev;
  FAR const struct igb_type_s *type;
  int                          irq;
  uint64_t                     base;

#ifdef CONFIG_NET_MCASTGROUP
  /* MTA shadow */

  FAR uint32_t *mta;
#endif
};

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

/* Helpers */

static uint32_t igb_getreg_mem(FAR struct igb_driver_s *priv,
                               unsigned int offset);
static void igb_putreg_mem(FAR struct igb_driver_s *priv,
                           unsigned int offset,
                           uint32_t value);
#ifdef CONFIG_DEBUG_NET_INFO
static void igb_dump_reg(FAR struct igb_driver_s *priv,
                         FAR const char *msg, unsigned int offset);
static void igb_dump_mem(FAR struct igb_driver_s *priv, FAR const char *msg);
#endif

/* Rings management */

static void igb_txclean(FAR struct igb_driver_s *priv);
static void igb_rxclean(FAR struct igb_driver_s *priv);

/* Common TX logic */

static int igb_transmit(FAR struct netdev_lowerhalf_s *dev,
                        FAR netpkt_t *pkt);

/* Interrupt handling */

static FAR netpkt_t *igb_receive(FAR struct netdev_lowerhalf_s *dev);
static void igb_txdone(FAR struct netdev_lowerhalf_s *dev);

static void igb_msix_interrupt(FAR struct igb_driver_s *priv);
static int igb_interrupt(int irq, FAR void *context, FAR void *arg);

/* NuttX callback functions */

static int igb_ifup(FAR struct netdev_lowerhalf_s *dev);
static int igb_ifdown(FAR struct netdev_lowerhalf_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static uint32_t igb_hashmta(FAR struct igb_driver_s *priv,
                            FAR const uint8_t *mac);
static int igb_addmac(FAR struct netdev_lowerhalf_s *dev,
                      FAR const uint8_t *mac);
static int igb_rmmac(FAR struct netdev_lowerhalf_s *dev,
                     FAR const uint8_t *mac);
#endif

/* Initialization */

static void igb_disable(FAR struct igb_driver_s *priv);
static void igb_enable(FAR struct igb_driver_s *priv);
static int igb_initialize(FAR struct igb_driver_s *priv);
static int igb_probe(FAR struct pci_device_s *dev);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/* Intel 82576 (QEMU -device igb) */

static const struct igb_type_s g_igb_82576 =
{
  .desc_align = 128,
  .mta_regs   = 128
};

/* Intel I211 */

static const struct igb_type_s g_igb_i211 =
{
  .desc_align = 128,
  .mta_regs   = 128
};

static const struct pci_device_id_s g_igb_id_table[] =
{
  {
    PCI_DEVICE(0x8086, 0x10c9),
    .driver_data = (uintptr_t)&g_igb_82576
  },

  {
    PCI_DEVICE(0x8086, 0x1539),
    .driver_data = (uintptr_t)&g_igb_i211
  },

  {
    PCI_DEVICE(0x8086, 0x1533),
    .driver_data = (uintptr_t)&g_igb_i211
  },
  { }
};

static struct pci_driver_s g_pci_igb_drv =
{
  .id_table = g_igb_id_table,
  .probe    = igb_probe,
};

static const struct netdev_ops_s g_igb_ops =
{
  .ifup     = igb_ifup,
  .ifdown   = igb_ifdown,
  .transmit = igb_transmit,
  .receive  = igb_receive,
#ifdef CONFIG_NET_MCASTGROUP
  .addmac   = igb_addmac,
  .rmmac    = igb_rmmac,
#endif
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: igb_getreg_mem
 *****************************************************************************/

static uint32_t igb_getreg_mem(FAR struct igb_driver_s *priv,
                               unsigned int offset)
{
  uintptr_t addr = priv->base + offset;
  return *((FAR volatile uint32_t *)addr);
}

/*****************************************************************************
 * Name: igb_putreg_mem
 *****************************************************************************/

static void igb_putreg_mem(FAR struct igb_driver_s *priv,
                           unsigned int offset,
                           uint32_t value)
{
  uintptr_t addr = priv->base + offset;
  *((FAR volatile uint32_t *)addr) = value;
}

#ifdef CONFIG_DEBUG_NET_INFO
/*****************************************************************************
 * Name: igb_dump_reg
 *****************************************************************************/

static void igb_dump_reg(FAR struct igb_driver_s *priv,
                         FAR const char *msg, unsigned int offset)
{
  ninfo("\t%s:\t\t0x%" PRIx32 "\n", msg, igb_getreg_mem(priv, offset));
}

/*****************************************************************************
 * Name: igb_dump_mem
 *****************************************************************************/

static void igb_dump_mem(FAR struct igb_driver_s *priv, FAR const char *msg)
{
  ninfo("\nDump: %s\n", msg);

  ninfo("General registers:\n");
  igb_dump_reg(priv, "CTRL", IGB_CTRL);
  igb_dump_reg(priv, "STATUS", IGB_STATUS);
  igb_dump_reg(priv, "CTRLEXT", IGB_CTRLEXT);
  igb_dump_reg(priv, "MDIC", IGB_MDIC);
  igb_dump_reg(priv, "SERDESCTL", IGB_SERDESCTL);
  igb_dump_reg(priv, "FCAL", IGB_FCAL);
  igb_dump_reg(priv, "FCAH", IGB_FCAH);
  igb_dump_reg(priv, "FCT", IGB_FCT);
  igb_dump_reg(priv, "CONNSW", IGB_CONNSW);
  igb_dump_reg(priv, "VET", IGB_VET);
  igb_dump_reg(priv, "FCTTV", IGB_FCTTV);

  ninfo("Interrupt registers:\n");
  igb_dump_reg(priv, "ICS", IGB_ICS);
  igb_dump_reg(priv, "IMS", IGB_IMS);
  igb_dump_reg(priv, "IAM", IGB_IAM);
  igb_dump_reg(priv, "EICS", IGB_EICS);
  igb_dump_reg(priv, "EIMS", IGB_EIMS);
  igb_dump_reg(priv, "EIAM", IGB_EIAM);
  igb_dump_reg(priv, "EIAC", IGB_EIAC);
  igb_dump_reg(priv, "IVAR0", IGB_IVAR0);
  igb_dump_reg(priv, "IVARMSC", IGB_IVARMSC);
  igb_dump_reg(priv, "EITR0", IGB_EITR0);
  igb_dump_reg(priv, "GPIE", IGB_GPIE);
  igb_dump_reg(priv, "PBACL", IGB_PBACL);

  ninfo("Receive registers:\n");
  igb_dump_reg(priv, "RCTL", IGB_RCTL);
  igb_dump_reg(priv, "PSRCTL", IGB_PSRCTL);
  igb_dump_reg(priv, "FCRTL0", IGB_FCRTL0);
  igb_dump_reg(priv, "FCRTH0", IGB_FCRTH0);
  igb_dump_reg(priv, "RXPBSIZE", IGB_RXPBSIZE);
  igb_dump_reg(priv, "FCRTV", IGB_FCRTV);
  igb_dump_reg(priv, "RDBAL0", IGB_RDBAL0);
  igb_dump_reg(priv, "RDBAH0", IGB_RDBAH0);
  igb_dump_reg(priv, "RDLEN0", IGB_RDLEN0);
  igb_dump_reg(priv, "SRRCTL0", IGB_SRRCTL0);
  igb_dump_reg(priv, "RDH0", IGB_RDH0);
  igb_dump_reg(priv, "RDT0", IGB_RDT0);
  igb_dump_reg(priv, "RXDCTL0", IGB_RXDCTL0);
  igb_dump_reg(priv, "RXCTL0", IGB_RXCTL0);
  igb_dump_reg(priv, "RXCSUM", IGB_RXCSUM);
  igb_dump_reg(priv, "RLPML", IGB_RLPML);
  igb_dump_reg(priv, "RFCTL", IGB_RFCTL);
  igb_dump_reg(priv, "MTA", IGB_MTA);
  igb_dump_reg(priv, "RAL", IGB_RAL);
  igb_dump_reg(priv, "RAH", IGB_RAH);

  ninfo("Transmit registers:\n");
  igb_dump_reg(priv, "TXPBSIZE", IGB_TXPBSIZE);
  igb_dump_reg(priv, "PBTWAC", IGB_PBTWAC);
  igb_dump_reg(priv, "TCTL", IGB_TCTL);
  igb_dump_reg(priv, "TCTLEXT", IGB_TCTLEXT);
  igb_dump_reg(priv, "TIPG", IGB_TIPG);
  igb_dump_reg(priv, "RETXCTL", IGB_RETXCTL);
  igb_dump_reg(priv, "DTXCTL", IGB_DTXCTL);
  igb_dump_reg(priv, "TDBAL0", IGB_TDBAL0);
  igb_dump_reg(priv, "TDBAH0", IGB_TDBAH0);
  igb_dump_reg(priv, "TDLEN0", IGB_TDLEN0);
  igb_dump_reg(priv, "TDH0", IGB_TDH0);
  igb_dump_reg(priv, "TDT0", IGB_TDT0);
  igb_dump_reg(priv, "TXDCTL0", IGB_TXDCTL0);
  igb_dump_reg(priv, "TXCTL0", IGB_TXCTL0);
  igb_dump_reg(priv, "TDWBAL0", IGB_TDWBAL0);
  igb_dump_reg(priv, "TDWBAH0", IGB_TDWBAH0);

  ninfo("Statistic registers:\n");
  igb_dump_reg(priv, "CRCERRS", IGB_CRCERRS);
  igb_dump_reg(priv, "ALGNERRC", IGB_ALGNERRC);
  igb_dump_reg(priv, "RXERRC", IGB_RXERRC);
  igb_dump_reg(priv, "MPC", IGB_MPC);
  igb_dump_reg(priv, "SCC", IGB_SCC);
  igb_dump_reg(priv, "ECOL", IGB_ECOL);
  igb_dump_reg(priv, "MCC", IGB_MCC);
  igb_dump_reg(priv, "LATECOL", IGB_LATECOL);
  igb_dump_reg(priv, "COLC", IGB_COLC);
  igb_dump_reg(priv, "DC", IGB_DC);
  igb_dump_reg(priv, "TNCRS", IGB_TNCRS);
  igb_dump_reg(priv, "RLEC", IGB_RLEC);
  igb_dump_reg(priv, "XONRXC", IGB_XONRXC);
  igb_dump_reg(priv, "XONTXC", IGB_XONTXC);
  igb_dump_reg(priv, "XOFFRXC", IGB_XOFFRXC);
  igb_dump_reg(priv, "XOFFTXC", IGB_XOFFTXC);
  igb_dump_reg(priv, "FCRUC", IGB_FCRUC);
  igb_dump_reg(priv, "PRC64", IGB_PRC64);
  igb_dump_reg(priv, "PRC127", IGB_PRC127);
  igb_dump_reg(priv, "PRC255", IGB_PRC255);
  igb_dump_reg(priv, "PRC511", IGB_PRC511);
  igb_dump_reg(priv, "PRC1023", IGB_PRC1023);
  igb_dump_reg(priv, "PRC1522", IGB_PRC1522);
  igb_dump_reg(priv, "GPRC", IGB_GPRC);
  igb_dump_reg(priv, "BPRC", IGB_BPRC);
  igb_dump_reg(priv, "MPRC", IGB_MPRC);
  igb_dump_reg(priv, "GPTC", IGB_GPTC);
  igb_dump_reg(priv, "GORCL", IGB_GORCL);
  igb_dump_reg(priv, "GORCH", IGB_GORCH);
  igb_dump_reg(priv, "GOTCL", IGB_GOTCL);
  igb_dump_reg(priv, "GOTCH", IGB_GOTCH);
  igb_dump_reg(priv, "RNBC", IGB_RNBC);
  igb_dump_reg(priv, "RUC", IGB_RUC);
  igb_dump_reg(priv, "RFC", IGB_RFC);
  igb_dump_reg(priv, "ROC", IGB_ROC);
  igb_dump_reg(priv, "RJC", IGB_RJC);
  igb_dump_reg(priv, "MNGPRC", IGB_MNGPRC);
  igb_dump_reg(priv, "MPDC", IGB_MPDC);
  igb_dump_reg(priv, "TORL", IGB_TORL);
  igb_dump_reg(priv, "TORH", IGB_TORH);
  igb_dump_reg(priv, "TPR", IGB_TPR);
  igb_dump_reg(priv, "TPT", IGB_TPT);
  igb_dump_reg(priv, "PTC64", IGB_PTC64);
  igb_dump_reg(priv, "PTC127", IGB_PTC127);
  igb_dump_reg(priv, "PTC255", IGB_PTC255);
  igb_dump_reg(priv, "PTC511", IGB_PTC511);
  igb_dump_reg(priv, "PTC1023", IGB_PTC1023);
  igb_dump_reg(priv, "PTC1522", IGB_PTC1522);
  igb_dump_reg(priv, "BPTC", IGB_BPTC);
  ninfo("Diagnostic registers:\n");
  igb_dump_reg(priv, "RDFT", IGB_RDFT);
  igb_dump_reg(priv, "RDFHS", IGB_RDFHS);
  igb_dump_reg(priv, "RDFTS", IGB_RDFTS);
  igb_dump_reg(priv, "RDFPC", IGB_RDFPC);
  igb_dump_reg(priv, "RPBECCSTS", IGB_RPBECCSTS);
  igb_dump_reg(priv, "TPBECCSTS", IGB_TPBECCSTS);
  igb_dump_reg(priv, "FCSTS0", IGB_FCSTS0);
  igb_dump_reg(priv, "RDHESTS", IGB_RDHESTS);
  igb_dump_reg(priv, "TDHESTS", IGB_TDHESTS);
  igb_dump_reg(priv, "TDFH", IGB_TDFH);
  igb_dump_reg(priv, "TDFT", IGB_TDFT);
  igb_dump_reg(priv, "TDFHS", IGB_TDFHS);
  igb_dump_reg(priv, "TDFTS", IGB_TDFTS);
  igb_dump_reg(priv, "TDFPC", IGB_TDFPC);
  igb_dump_reg(priv, "TDHMP", IGB_TDHMP);
  igb_dump_reg(priv, "CIRC", IGB_CIRC);
  igb_dump_reg(priv, "TXBDC", IGB_TXBDC);
  igb_dump_reg(priv, "TXIDLE", IGB_TXIDLE);
  igb_dump_reg(priv, "RXBDC", IGB_RXBDC);
  igb_dump_reg(priv, "RXIDLE", IGB_RXIDLE);
}
#endif

/*****************************************************************************
 * Name: igb_txclean
 *
 * Description:
 *   Clean transmission ring
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumption:
 *   This function can be called only after card reset and when TX is disabled
 *
 *****************************************************************************/

static void igb_txclean(FAR struct igb_driver_s *priv)
{
  FAR struct netdev_lowerhalf_s *netdev = &priv->dev;

  /* Reset ring */

  igb_putreg_mem(priv, IGB_TDH0, 0);
  igb_putreg_mem(priv, IGB_TDT0, 0);

  /* Free any pending TX */

  while (priv->tx_now != priv->tx_done)
    {
      /* Free net packet */

      netpkt_free(netdev, priv->tx_pkt[priv->tx_done], NETPKT_TX);

      /* Next descriptor */

      priv->tx_done = (priv->tx_done + 1) % IGB_TX_DESC;
    }

  priv->tx_now  = 0;
  priv->tx_done = 0;
}

/*****************************************************************************
 * Name: igb_rxclean
 *
 * Description:
 *   Clean receive ring
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumption:
 *   This function can be called only after card reset and when RX is disabled
 *
 *****************************************************************************/

static void igb_rxclean(FAR struct igb_driver_s *priv)
{
  priv->rx_now = 0;

  igb_putreg_mem(priv, IGB_RDH0, 0);
  igb_putreg_mem(priv, IGB_RDT0, IGB_RX_DESC - 1);
}

/*****************************************************************************
 * Name: igb_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
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

static int igb_transmit(FAR struct netdev_lowerhalf_s *dev,
                          FAR netpkt_t *pkt)
{
  FAR struct igb_driver_s *priv    = (FAR struct igb_driver_s *)dev;
  uint64_t                 pa      = 0;
  int                      desc    = priv->tx_now;
  size_t                   len     = netpkt_getdatalen(dev, pkt);
  size_t                   tx_next = (priv->tx_now + 1) % IGB_TX_DESC;

  ninfo("transmit\n");

  /* Check the send length */

  if (len > IGB_PKTBUF_SIZE)
    {
      nerr("net transmit buffer too large\n");
      return -EINVAL;
    }

  if (!IFF_IS_RUNNING(dev->netdev.d_flags))
    {
      return -ENETDOWN;
    }

  /* Drop packet if ring full */

  if (tx_next == priv->tx_done)
    {
      return -ENOMEM;
    }

  /* Store TX packet reference */

  priv->tx_pkt[priv->tx_now] = pkt;

  /* Prepare next TX descriptor */

  priv->tx_now = tx_next;

  /* Setup TX descriptor */

  pa = up_addrenv_va_to_pa(netpkt_getdata(dev, pkt));

  priv->tx[desc].addr   = pa;
  priv->tx[desc].len    = len;
  priv->tx[desc].cmd    = (IGB_TDESC_CMD_EOP | IGB_TDESC_CMD_IFCS |
                           IGB_TDESC_CMD_RS);
  priv->tx[desc].cso    = 0;
  priv->tx[desc].status = 0;

  UP_DSB();

  /* Update TX tail */

  igb_putreg_mem(priv, IGB_TDT0, priv->tx_now);

  ninfodumpbuffer("Transmitted:", netpkt_getdata(dev, pkt), len);

  return OK;
}

/*****************************************************************************
 * Name: igb_receive
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
 *****************************************************************************/

static FAR netpkt_t *igb_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct igb_driver_s *priv = (FAR struct igb_driver_s *)dev;
  FAR netpkt_t            *pkt  = NULL;
  FAR struct igb_rx_leg_s *rx   = NULL;
  int                      desc = 0;

  desc = priv->rx_now;

  /* Get RX descriptor and RX packet */

  rx = &priv->rx[desc];
  pkt = priv->rx_pkt[desc];

  /* Check if descriptor done */

  if (!(rx->status & IGB_RDESC_STATUS_DD))
    {
      return NULL;
    }

  /* Next descriptor */

  priv->rx_now = (priv->rx_now + 1) % IGB_RX_DESC;

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

  igb_putreg_mem(priv, IGB_RDT0, desc);

  /* Handle errors */

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
 * Name: igb_txdone
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

static void igb_txdone(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct igb_driver_s *priv = (FAR struct igb_driver_s *)dev;

  while (priv->tx_now != priv->tx_done)
    {
      if (priv->tx[priv->tx_done].status == 0)
        {
          break;
        }

      if (!(priv->tx[priv->tx_done].status & IGB_TDESC_STATUS_DD))
        {
          nerr("tx failed: 0x%" PRIx32 "\n", priv->tx[priv->tx_done].status);
          NETDEV_TXERRORS(&priv->dev.netdev);
        }

      /* Free net packet */

      netpkt_free(dev, priv->tx_pkt[priv->tx_done], NETPKT_TX);

      /* Next descriptor */

      priv->tx_done = (priv->tx_done + 1) % IGB_TX_DESC;
    }

  netdev_lower_txdone(dev);
}

/*****************************************************************************
 * Name: igb_link_work
 *
 * Description:
 *   Handle link status change.
 *
 * Input Parameters:
 *   arg - Reference to the lover half driver structure (cast to void *)
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

static void igb_link_work(FAR void *arg)
{
  FAR struct igb_driver_s *priv = arg;
  uint32_t tmp;

  tmp = igb_getreg_mem(priv, IGB_STATUS);
  if (tmp & IGB_STATUS_LU)
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

/*****************************************************************************
 * Name: igb_misx_interrupt
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

static void igb_msix_interrupt(FAR struct igb_driver_s *priv)
{
  uint32_t icr  = 0;
  uint32_t eicr = 0;

  /* Get interrupts */

  icr  = igb_getreg_mem(priv, IGB_ICR);
  eicr = igb_getreg_mem(priv, IGB_EICR);

  ninfo("eicr = 0x%" PRIx32 " icr = 0x%" PRIx32 "\n", eicr, icr);

  if (icr == 0)
    {
      /* Ignore spurious interrupts */

      return;
    }

  /* Receiver Descriptor Write Back */

  if (icr & IGB_IC_RXDW)
    {
      netdev_lower_rxready(&priv->dev);
    }

  /* Link Status Change */

  if (icr & IGB_IC_LSC)
    {
      if (work_available(&priv->work))
        {
          /* Schedule to work queue because netdev_lower_carrier_xxx API
           * can't be used in interrupt context
           */

          work_queue(LPWORK, &priv->work, igb_link_work, priv, 0);
        }
    }

  /* Receiver Miss */

  if (icr & IGB_IC_RXMISS)
    {
      nerr("Receiver Miss\n");
      netdev_lower_rxready(&priv->dev);
    }

  /* Transmit Descriptor Written Back */

  if (icr & IGB_IC_TXDW)
    {
      igb_txdone(&priv->dev);
    }
}

/*****************************************************************************
 * Name: igb_interrupt
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

static int igb_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct igb_driver_s *priv = (FAR struct igb_driver_s *)arg;

  DEBUGASSERT(priv != NULL);

  ninfo("interrupt!\n");

  /* Schedule to perform the interrupt processing on the worker thread. */

  igb_msix_interrupt(priv);

  return OK;
}

/*****************************************************************************
 * Name: igb_ifup
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

static int igb_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct igb_driver_s *priv = (FAR struct igb_driver_s *)dev;
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

  flags = enter_critical_section();

  /* Enable the Ethernet */

  igb_enable(priv);
  leave_critical_section(flags);

  /* Update link status in case link status interrupt is missing */

  igb_link_work(priv);

  return OK;
}

/*****************************************************************************
 * Name: igb_ifdown
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

static int igb_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct igb_driver_s *priv = (FAR struct igb_driver_s *)dev;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the igb_ifup() always
   * successfully brings the interface back up.
   */

  igb_disable(priv);
  leave_critical_section(flags);
  return OK;
}

#ifdef CONFIG_NET_MCASTGROUP
/*****************************************************************************
 * Name: igb_hashmta
 *
 * Note: This logic is based on freeBSD igb implementation
 *
 *****************************************************************************/

static uint32_t igb_hashmta(FAR struct igb_driver_s *priv,
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
 * Name: igb_addmac
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

static int igb_addmac(FAR struct netdev_lowerhalf_s *dev,
                        FAR const uint8_t *mac)
{
  FAR struct igb_driver_s *priv = (FAR struct igb_driver_s *)dev;
  uint16_t                 hash = 0;
  uint8_t                  row  = 0;
  uint8_t                  bit  = 0;
  int                      i    = 0;

  hash = igb_hashmta(priv, mac);
  bit = hash & 31;
  row = (hash >> 5) & (priv->type->mta_regs - 1);

  /* Bits 4:0 indicate bit in row word */

  priv->mta[row] |= (1 << bit);

  /* Replace the entire MTA */

  for (i = priv->type->mta_regs - 1; i >= 0; i--)
    {
      igb_putreg_mem(priv, IGB_MTA + (i << 2), priv->mta[i]);
    }

  return OK;
}

/*****************************************************************************
 * Name: igb_rmmac
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

static int igb_rmmac(FAR struct netdev_lowerhalf_s *dev,
                       FAR const uint8_t *mac)
{
  FAR struct igb_driver_s *priv = (FAR struct igb_driver_s *)dev;
  uint16_t                 hash = 0;
  uint8_t                  row  = 0;
  uint8_t                  bit  = 0;
  int                      i    = 0;

  hash = igb_hashmta(priv, mac);
  bit = hash & 31;
  row = (hash >> 5) & (priv->type->mta_regs - 1);

  /* Bits 4:0 indicate bit in row word */

  priv->mta[row] &= ~(1 << bit);

  /* Replace the entire MTA */

  for (i = priv->type->mta_regs - 1; i >= 0; i--)
    {
      igb_putreg_mem(priv, IGB_MTA + (i << 2), priv->mta[i]);
    }

  return OK;
}
#endif  /* CONFIG_NET_MCASTGROUP */

/*****************************************************************************
 * Name: igb_disable
 *
 * Description:
 *   Reset device to known state.
 *
 *****************************************************************************/

static void igb_disable(FAR struct igb_driver_s *priv)
{
  uint32_t regval;
  int      i = 0;

  /* Disable interrupts */

  igb_putreg_mem(priv, IGB_EIMC, IGB_MSIX_EIMS);
  igb_putreg_mem(priv, IGB_IMC, IGB_MSIX_IMS);
  up_disable_irq(priv->irq);

  /* Disable Transmitter */

  regval = igb_getreg_mem(priv, IGB_TCTL);
  regval &= ~IGB_TCTL_EN;
  igb_putreg_mem(priv, IGB_TCTL, regval);

  /* Disable Receiver */

  igb_putreg_mem(priv, IGB_RCTL, 0);

  /* We have to reset device, otherwise writing to RDH and THD corrupts
   * the device state.
   */

  igb_putreg_mem(priv, IGB_CTRL, IGB_CTRL_RST);

  /* Reset Tx tail */

  igb_txclean(priv);

  /* Reset Rx tail */

  igb_rxclean(priv);

  /* Free RX packets */

  for (i = 0; i < IGB_RX_DESC; i += 1)
    {
      netpkt_free(&priv->dev, priv->rx_pkt[i], NETPKT_RX);
    }
}

/*****************************************************************************
 * Name: igb_phy_reset
 *
 * Description:
 *   Reset PHY
 *
 *****************************************************************************/

static void igb_phy_reset(FAR struct igb_driver_s *priv)
{
  uint32_t regval = 0;

  regval = igb_getreg_mem(priv, IGB_CTRL);
  igb_putreg_mem(priv, IGB_CTRL, regval | IGB_CTRL_PHYRST);
  up_udelay(100);
  igb_putreg_mem(priv, IGB_CTRL, regval);
  up_udelay(100);
}

/*****************************************************************************
 * Name: igb_enable
 *
 * Description:
 *   Enable device.
 *
 *****************************************************************************/

static void igb_enable(FAR struct igb_driver_s *priv)
{
  FAR struct netdev_lowerhalf_s *dev = (FAR struct netdev_lowerhalf_s *)priv;
  uint64_t pa     = 0;
  uint32_t regval = 0;
  int      i      = 0;

  /* Reset PHY */

  igb_phy_reset(priv);

  /* Reset Multicast Table Array */

  for (i = 0; i < priv->type->mta_regs; i++)
    {
      igb_putreg_mem(priv, IGB_MTA + (i << 2), 0);
    }

  /* Allocate RX packets */

  for (i = 0; i < IGB_RX_DESC; i += 1)
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
  igb_putreg_mem(priv, IGB_TDBAL0, regval);
  regval = (uint32_t)(pa >> 32);
  igb_putreg_mem(priv, IGB_TDBAH0, regval);

  regval = IGB_TX_DESC * sizeof(struct igb_tx_leg_s);
  igb_putreg_mem(priv, IGB_TDLEN0, regval);

  /* Reset TX tail */

  igb_txclean(priv);

  /* Setup RX descriptor */

  /* The address passed to the NIC must be physical */

  pa = up_addrenv_va_to_pa(priv->rx);

  regval = (uint32_t)pa;
  igb_putreg_mem(priv, IGB_RDBAL0, regval);
  regval = (uint32_t)(pa >> 32);
  igb_putreg_mem(priv, IGB_RDBAH0, regval);

  regval = IGB_RX_DESC * sizeof(struct igb_rx_leg_s);
  igb_putreg_mem(priv, IGB_RDLEN0, regval);

  /* Enable interrupts */

  igb_putreg_mem(priv, IGB_EIMS, IGB_MSIX_EIMS);
  igb_putreg_mem(priv, IGB_IMS, IGB_MSIX_IMS);
  up_enable_irq(priv->irq);

  /* Set link up */

  igb_putreg_mem(priv, IGB_CTRL, IGB_CTRL_SLU);

  /* Setup and enable Transmitter */

  regval = igb_getreg_mem(priv, IGB_TCTL);
  regval |= IGB_TCTL_EN | IGB_TCTL_PSP;
  igb_putreg_mem(priv, IGB_TCTL, regval);

  /* Setup and enable Receiver */

  regval = (IGB_RCTL_EN | IGB_RCTL_MPE |
            (IGB_RCTL_BSIZE << IGB_RCTL_BSIZE_SHIFT));
#ifdef CONFIG_NET_PROMISCUOUS
  regval |= IGB_RCTL_UPE | IGB_RCTL_MPE;
#endif
  igb_putreg_mem(priv, IGB_RCTL, regval);

  /* Enable TX queeu */

  regval = igb_getreg_mem(priv, IGB_TXDCTL0);
  regval |= IGB_TXDCTL_ENABLE;
  igb_putreg_mem(priv, IGB_TXDCTL0, regval);

  /* Enable RX queue */

  regval = igb_getreg_mem(priv, IGB_RXDCTL0);
  regval |= IGB_RXDCTL_ENABLE;
  igb_putreg_mem(priv, IGB_RXDCTL0, regval);

  /* Reset RX tail - after queue is enabled */

  igb_rxclean(priv);

#ifdef CONFIG_DEBUG_NET_INFO
  /* Dump memory */

  igb_dump_mem(priv, "enabled");
#endif
}

/*****************************************************************************
 * Name: igb_initialize
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int igb_initialize(FAR struct igb_driver_s *priv)
{
  uint32_t regval = 0;
  uint64_t mac    = 0;
  int      ret    = OK;

  /* Allocate MSI */

  ret = pci_alloc_irq(priv->pcidev, &priv->irq, 1);
  if (ret != 1)
    {
      nerr("Failed to allocate MSI %d\n", ret);
      return ret;
    }

  /* Attach IRQ */

  irq_attach(priv->irq, igb_interrupt, priv);

  /* Connect MSI */

  ret = pci_connect_irq(priv->pcidev, &priv->irq, 1);
  if (ret != OK)
    {
      nerr("Failed to connect MSI %d\n", ret);
      pci_release_irq(priv->pcidev, &priv->irq, 1);

      return -ENOTSUP;
    }

  /* Clear previous Extended Interrupt Mask */

  igb_putreg_mem(priv, IGB_EIMC, 0xffffffff);
  igb_putreg_mem(priv, IGB_IMC, 0xffffffff);

  /* Configure MSI-X */

  igb_putreg_mem(priv, IGB_IVAR0, IGB_MSIX_IVAR0);
  igb_putreg_mem(priv, IGB_IVARMSC, IGB_MSIX_IVARMSC);

  /* Enable MSI-X Single Vector */

  igb_putreg_mem(priv, IGB_GPIE, IGB_GPIE_MSIX_SINGLE);
  igb_putreg_mem(priv, IGB_EIMS, IGB_MSIX_EIMS);

  /* Configure Other causes */

  igb_putreg_mem(priv, IGB_IMS, IGB_MSIX_IMS);

  /* Configure Interrupt Throttle */

  igb_putreg_mem(priv, IGB_EITR0, (CONFIG_NET_IGB_INT_INTERVAL << 2));

  /* Get MAC if valid */

  regval = igb_getreg_mem(priv, IGB_RAH);
  if (regval & IGB_RAH_AV)
    {
      mac = ((uint64_t)regval & IGB_RAH_RAH_MASK) << 32;
      mac |= igb_getreg_mem(priv, IGB_RAL);
      memcpy(&priv->dev.netdev.d_mac.ether, &mac, sizeof(struct ether_addr));
    }
  else
    {
      nwarn("Receive Address not valid!\n");
    }

  return OK;
}

/*****************************************************************************
 * Name: igb_probe
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int igb_probe(FAR struct pci_device_s *dev)
{
  FAR const struct igb_type_s   *type   = NULL;
  FAR struct igb_driver_s       *priv   = NULL;
  FAR struct netdev_lowerhalf_s *netdev = NULL;
  int                            ret    = -ENOMEM;

  /* Get type data associated with this PCI device card */

  type = (FAR const struct igb_type_s *)dev->id->driver_data;

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
                          IGB_TX_DESC * sizeof(struct igb_tx_leg_s));
  if (priv->tx == NULL)
    {
      nerr("alloc tx failed %d\n", errno);
      goto errout;
    }

  /* Allocate RX descriptors */

  priv->rx = kmm_memalign(type->desc_align,
                          IGB_RX_DESC * sizeof(struct igb_rx_leg_s));
  if (priv->rx == NULL)
    {
      nerr("alloc rx failed %d\n", errno);
      goto errout;
    }

  /* Allocate TX packet pointer array */

  priv->tx_pkt = kmm_zalloc(IGB_TX_DESC * sizeof(netpkt_t *));
  if (priv->tx_pkt == NULL)
    {
      nerr("alloc tx_pkt failed\n");
      goto errout;
    }

  /* Allocate RX packet pointer array */

  priv->rx_pkt = kmm_zalloc(IGB_RX_DESC * sizeof(netpkt_t *));
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

  priv->base = (uintptr_t)pci_map_bar(dev, IGB_MMIO_BAR);
  if (!priv->base)
    {
      pcierr("Not found MMIO control bar\n");
      goto errout;
    }

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  ret = igb_initialize(priv);
  if (ret != OK)
    {
      nerr("igb_initialize failed %d\n", ret);
      return ret;
    }

  /* Register the network device */

  netdev->quota[NETPKT_TX] = IGB_TX_QUOTA;
  netdev->quota[NETPKT_RX] = IGB_RX_QUOTA;
  netdev->ops = &g_igb_ops;

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
 * Name: pci_igb_init
 *
 * Description:
 *   Register a pci driver
 *
 *****************************************************************************/

int pci_igb_init(void)
{
  return pci_register_driver(&g_pci_igb_drv);
}
