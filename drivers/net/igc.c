/*****************************************************************************
 * drivers/net/igc.c
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
#include <nuttx/net/igc.h>

#include <arch/barriers.h>

#include "igc.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#if CONFIG_NET_IGC_TXDESC % 2 != 0
#  error CONFIG_NET_IGC_TXDESC must be multiple of 2
#endif

#if CONFIG_NET_IGC_RXDESC % 2 != 0
#  error CONFIG_NET_IGC_RXDESC must be multiple of 2
#endif

/* Packet buffer size */

#define IGC_PKTBUF_SIZE        2048
#define IGC_RCTL_BSIZE         IGC_RCTL_BSIZE_2048

/* TX and RX descriptors */

#define IGC_TX_DESC            CONFIG_NET_IGC_TXDESC
#define IGC_RX_DESC            CONFIG_NET_IGC_RXDESC

/* After RX packet is done, we provide free netpkt to the RX descriptor ring.
 * The upper-half network logic is responsible for freeing the RX packets
 * so we need some additional spare netpkt buffers to assure that it's
 * allways possible to allocate the new RX packet in the recevier logic.
 * It's hard to tell how many spare buffers is needed, for now it's set to 8.
 */

#define IGC_TX_QUOTA           IGC_TX_DESC
#define IGC_RX_QUOTA           (IGC_RX_DESC + CONFIG_NET_IGC_RXSPARE)

/* NOTE: CONFIG_IOB_ALIGNMENT must match system D-CACHE line size */

#if CONFIG_IOB_NBUFFERS < IGC_RX_QUOTA + IGC_TX_QUOTA
#  error CONFIG_IOB_NBUFFERS must be > (IGC_RX_QUOTA + IGC_TX_QUOTA)
#endif

#if CONFIG_IOB_BUFSIZE < IGC_PKTBUF_SIZE
#  error CONFIG_IOB_BUFSIZE must be > IGC_PKTBUF_SIZE
#endif

/* PCI BARs */

#define IGC_MMIO_BAR           0
#define IGC_FLASH_BAR          1
#define IGC_IO_BAR             2
#define IGC_MSIX_BAR           3

/* For MSI-X we allocate all interrupts to MSI-X vector 0 */

#define IGC_GPIE_MSIX_SINGLE   (IGC_GPIE_NSICR | IGC_GPIE_EIAME | \
                                IGC_GPIE_PBASUPPORT)
#define IGC_MSIX_IMS           (IGC_IC_TXDW | IGC_IC_LSC | \
                                IGC_IC_RXMISS | IGC_IC_RXDW)
#define IGC_MSIX_EIMS          (IGC_EIMS_NOMSIX_OTHER | \
                                IGC_EIMS_NOMSIX_RXTX0)
#define IGC_MSIX_IVAR0         (IGC_IVAR0_RXQ0_VAL | IGC_IVAR0_TXQ0_VAL)
#define IGC_MSIX_IVARMSC       (IGC_IVARMSC_OTHER_VAL)

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* Extend default PCI devie type */

struct igc_type_s
{
  uint32_t desc_align;          /* Descriptor alignment */
  uint32_t mta_regs;            /* MTA registers */
};

/* IGC private data */

struct igc_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s dev;
  struct work_s work;

  /* Driver state */

  bool bifup;

  /* Packets list */

  FAR netpkt_t **tx_pkt;
  FAR netpkt_t **rx_pkt;

  /* Descriptors */

  FAR struct igc_tx_leg_s *tx;
  FAR struct igc_rx_leg_s *rx;

  size_t tx_now;
  size_t tx_done;
  size_t rx_now;

  /* PCI data */

  FAR struct pci_device_s     *pcidev;
  FAR const struct igc_type_s *type;
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

static uint32_t igc_getreg_mem(FAR struct igc_driver_s *priv,
                               unsigned int offset);
static void igc_putreg_mem(FAR struct igc_driver_s *priv,
                           unsigned int offset,
                           uint32_t value);
#ifdef CONFIG_DEBUG_NET_INFO
static void igc_dump_reg(FAR struct igc_driver_s *priv,
                         FAR const char *msg, unsigned int offset);
static void igc_dump_mem(FAR struct igc_driver_s *priv, FAR const char *msg);
#endif

/* Rings management */

static void igc_txclean(FAR struct igc_driver_s *priv);
static void igc_rxclean(FAR struct igc_driver_s *priv);

/* Common TX logic */

static int igc_transmit(FAR struct netdev_lowerhalf_s *dev,
                        FAR netpkt_t *pkt);

/* Interrupt handling */

static FAR netpkt_t *igc_receive(FAR struct netdev_lowerhalf_s *dev);
static void igc_txdone(FAR struct netdev_lowerhalf_s *dev);

static void igc_msix_interrupt(FAR struct igc_driver_s *priv);
static int igc_interrupt(int irq, FAR void *context, FAR void *arg);

/* NuttX callback functions */

static int igc_ifup(FAR struct netdev_lowerhalf_s *dev);
static int igc_ifdown(FAR struct netdev_lowerhalf_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static uint32_t igc_hashmta(FAR struct igc_driver_s *priv,
                            FAR const uint8_t *mac);
static int igc_addmac(FAR struct netdev_lowerhalf_s *dev,
                      FAR const uint8_t *mac);
static int igc_rmmac(FAR struct netdev_lowerhalf_s *dev,
                     FAR const uint8_t *mac);
#endif

/* Initialization */

static void igc_disable(FAR struct igc_driver_s *priv);
static void igc_enable(FAR struct igc_driver_s *priv);
static int igc_initialize(FAR struct igc_driver_s *priv);
static int igc_probe(FAR struct pci_device_s *dev);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/* Intel I225LM */

static const struct igc_type_s g_igc_i225lm =
{
  .desc_align = 128,
  .mta_regs   = 128
};

static const struct igc_type_s g_igc_i226v =
{
  .desc_align = 128,
  .mta_regs   = 128
};

static const struct pci_device_id_s g_igc_id_table[] =
{
  {
    PCI_DEVICE(0x8086, 0x15f2),
    .driver_data = (uintptr_t)&g_igc_i225lm
  },
  {
    PCI_DEVICE(0x8086, 0x125c),
    .driver_data = (uintptr_t)&g_igc_i226v
  },
  { }
};

static struct pci_driver_s g_pci_igc_drv =
{
  .id_table = g_igc_id_table,
  .probe    = igc_probe,
};

static const struct netdev_ops_s g_igc_ops =
{
  .ifup     = igc_ifup,
  .ifdown   = igc_ifdown,
  .transmit = igc_transmit,
  .receive  = igc_receive,
#ifdef CONFIG_NET_MCASTGROUP
  .addmac   = igc_addmac,
  .rmmac    = igc_rmmac,
#endif
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: igc_getreg_mem
 *****************************************************************************/

static uint32_t igc_getreg_mem(FAR struct igc_driver_s *priv,
                               unsigned int offset)
{
  uintptr_t addr = priv->base + offset;
  return *((FAR volatile uint32_t *)addr);
}

/*****************************************************************************
 * Name: igc_putreg_mem
 *****************************************************************************/

static void igc_putreg_mem(FAR struct igc_driver_s *priv,
                           unsigned int offset,
                           uint32_t value)
{
  uintptr_t addr = priv->base + offset;
  *((FAR volatile uint32_t *)addr) = value;
}

#ifdef CONFIG_DEBUG_NET_INFO
/*****************************************************************************
 * Name: igc_dump_reg
 *****************************************************************************/

static void igc_dump_reg(FAR struct igc_driver_s *priv,
                         FAR const char *msg, unsigned int offset)
{
  ninfo("\t%s:\t\t0x%" PRIx32 "\n", msg, igc_getreg_mem(priv, offset));
}

/*****************************************************************************
 * Name: igc_dump_mem
 *****************************************************************************/

static void igc_dump_mem(FAR struct igc_driver_s *priv, FAR const char *msg)
{
  ninfo("Dump: %s\n", msg);

  ninfo("General registers:\n");
  igc_dump_reg(priv, "CTRL", IGC_CTRL);
  igc_dump_reg(priv, "STATUS", IGC_STATUS);
  igc_dump_reg(priv, "CTRLEXT", IGC_CTRLEXT);
  igc_dump_reg(priv, "IPCNFG", IGC_IPCNFG);
  igc_dump_reg(priv, "PHMP", IGC_PHMP);

  ninfo("Interrupt registers:\n");
  igc_dump_reg(priv, "ICS", IGC_ICS);
  igc_dump_reg(priv, "IMS", IGC_IMS);
  igc_dump_reg(priv, "IAM", IGC_IAM);
  igc_dump_reg(priv, "EICS", IGC_EICS);
  igc_dump_reg(priv, "EIMS", IGC_EIMS);
  igc_dump_reg(priv, "EIAM", IGC_EIAM);
  igc_dump_reg(priv, "EIAC", IGC_EIAC);
  igc_dump_reg(priv, "IVAR0", IGC_IVAR0);
  igc_dump_reg(priv, "IVARMSC", IGC_IVARMSC);
  igc_dump_reg(priv, "GPIE", IGC_GPIE);
  igc_dump_reg(priv, "PICAUSE", IGC_PICAUSE);
  igc_dump_reg(priv, "PIENA", IGC_PIENA);
  igc_dump_reg(priv, "PBACL", IGC_PBACL);
  igc_dump_reg(priv, "EITR0", IGC_EITR0);

  ninfo("Receive registers:\n");
  igc_dump_reg(priv, "RCTL", IGC_RCTL);
  igc_dump_reg(priv, "PSRCTL", IGC_PSRCTL);
  igc_dump_reg(priv, "FCRTL0", IGC_FCRTL0);
  igc_dump_reg(priv, "FCRTH0", IGC_FCRTH0);
  igc_dump_reg(priv, "RXPBSIZE", IGC_RXPBSIZE);
  igc_dump_reg(priv, "FCRTV", IGC_FCRTV);
  igc_dump_reg(priv, "RDBAL0", IGC_RDBAL0);
  igc_dump_reg(priv, "RDBAH0", IGC_RDBAH0);
  igc_dump_reg(priv, "RDLEN0", IGC_RDLEN0);
  igc_dump_reg(priv, "SRRCTL0", IGC_SRRCTL0);
  igc_dump_reg(priv, "RDH0", IGC_RDH0);
  igc_dump_reg(priv, "RDT0", IGC_RDT0);
  igc_dump_reg(priv, "RXDCTL0", IGC_RXDCTL0);
  igc_dump_reg(priv, "RXCTL0", IGC_RXCTL0);
  igc_dump_reg(priv, "RXCSUM", IGC_RXCSUM);
  igc_dump_reg(priv, "RLPML", IGC_RLPML);
  igc_dump_reg(priv, "RFCTL", IGC_RFCTL);
  igc_dump_reg(priv, "MTA", IGC_MTA);
  igc_dump_reg(priv, "RAL", IGC_RAL);
  igc_dump_reg(priv, "RAH", IGC_RAH);

  ninfo("Transmit registers:\n");
  igc_dump_reg(priv, "TCTL", IGC_TCTL);
  igc_dump_reg(priv, "TCTLEXT", IGC_TCTLEXT);
  igc_dump_reg(priv, "TIPG", IGC_TIPG);
  igc_dump_reg(priv, "RETXCTL", IGC_RETXCTL);
  igc_dump_reg(priv, "TXPBSIZE", IGC_TXPBSIZE);
  igc_dump_reg(priv, "DTXCTL", IGC_DTXCTL);
  igc_dump_reg(priv, "TDBAL0", IGC_TDBAL0);
  igc_dump_reg(priv, "TDBAH0", IGC_TDBAH0);
  igc_dump_reg(priv, "TDLEN0", IGC_TDLEN0);
  igc_dump_reg(priv, "TDH0", IGC_TDH0);
  igc_dump_reg(priv, "TDT0", IGC_TDT0);
  igc_dump_reg(priv, "TXDCTL0", IGC_TXDCTL0);
  igc_dump_reg(priv, "TXCTL0", IGC_TXCTL0);
  igc_dump_reg(priv, "TDWBAL0", IGC_TDWBAL0);
  igc_dump_reg(priv, "TDWBAH0", IGC_TDWBAH0);

  ninfo("Statistic registers:\n");
  igc_dump_reg(priv, "CRCERRS", IGC_CRCERRS);
  igc_dump_reg(priv, "ALGNERRC", IGC_ALGNERRC);
  igc_dump_reg(priv, "RXERRC", IGC_RXERRC);
  igc_dump_reg(priv, "MPC", IGC_MPC);
  igc_dump_reg(priv, "SCC", IGC_SCC);
  igc_dump_reg(priv, "ECOL", IGC_ECOL);
  igc_dump_reg(priv, "MCC", IGC_MCC);
  igc_dump_reg(priv, "LATECOL", IGC_LATECOL);
  igc_dump_reg(priv, "COLC", IGC_COLC);
  igc_dump_reg(priv, "DC", IGC_DC);
  igc_dump_reg(priv, "TNCRS", IGC_TNCRS);
  igc_dump_reg(priv, "CEXTERR", IGC_CEXTERR);
  igc_dump_reg(priv, "RLEC", IGC_RLEC);
  igc_dump_reg(priv, "XONRXC", IGC_XONRXC);
  igc_dump_reg(priv, "XONTXC", IGC_XONTXC);
  igc_dump_reg(priv, "XOFFRXC", IGC_XOFFRXC);
  igc_dump_reg(priv, "XOFFTXC", IGC_XOFFTXC);
  igc_dump_reg(priv, "FCRUC", IGC_FCRUC);
  igc_dump_reg(priv, "PRC64", IGC_PRC64);
  igc_dump_reg(priv, "PRC127", IGC_PRC127);
  igc_dump_reg(priv, "PRC255", IGC_PRC255);
  igc_dump_reg(priv, "PRC511", IGC_PRC511);
  igc_dump_reg(priv, "PRC1023", IGC_PRC1023);
  igc_dump_reg(priv, "PRC1522", IGC_PRC1522);
  igc_dump_reg(priv, "GPRC", IGC_GPRC);
  igc_dump_reg(priv, "BPRC", IGC_BPRC);
  igc_dump_reg(priv, "MPRC", IGC_MPRC);
  igc_dump_reg(priv, "GPTC", IGC_GPTC);
  igc_dump_reg(priv, "GORCL", IGC_GORCL);
  igc_dump_reg(priv, "GORCH", IGC_GORCH);
  igc_dump_reg(priv, "GOTCL", IGC_GOTCL);
  igc_dump_reg(priv, "GOTCH", IGC_GOTCH);
  igc_dump_reg(priv, "RNBC", IGC_RNBC);
  igc_dump_reg(priv, "RUC", IGC_RUC);
  igc_dump_reg(priv, "RFC", IGC_RFC);
  igc_dump_reg(priv, "ROC", IGC_ROC);
  igc_dump_reg(priv, "RJC", IGC_RJC);
  igc_dump_reg(priv, "MNGPRC", IGC_MNGPRC);
  igc_dump_reg(priv, "MPDC", IGC_MPDC);
  igc_dump_reg(priv, "MPTC", IGC_MPTC);
  igc_dump_reg(priv, "TORL", IGC_TORL);
  igc_dump_reg(priv, "TORH", IGC_TORH);
  igc_dump_reg(priv, "TOT", IGC_TOT);
  igc_dump_reg(priv, "TPR", IGC_TPR);
  igc_dump_reg(priv, "TPT", IGC_TPT);
  igc_dump_reg(priv, "PTC64", IGC_PTC64);
  igc_dump_reg(priv, "PTC127", IGC_PTC127);
  igc_dump_reg(priv, "PTC255", IGC_PTC255);
  igc_dump_reg(priv, "PTC511", IGC_PTC511);
  igc_dump_reg(priv, "PTC1023", IGC_PTC1023);
  igc_dump_reg(priv, "PTC1522", IGC_PTC1522);
  igc_dump_reg(priv, "MPTC", IGC_MPTC);
  igc_dump_reg(priv, "BPTC", IGC_BPTC);
  igc_dump_reg(priv, "TSCTC", IGC_TSCTC);
  igc_dump_reg(priv, "TSCTFC", IGC_TSCTFC);
  igc_dump_reg(priv, "IAC", IGC_IAC);
}
#endif

/*****************************************************************************
 * Name: igc_txclean
 *
 * Description:
 *   Clean transmition ring
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

static void igc_txclean(FAR struct igc_driver_s *priv)
{
  FAR struct netdev_lowerhalf_s *netdev = &priv->dev;

  /* Reset ring */

  igc_putreg_mem(priv, IGC_TDH0, 0);
  igc_putreg_mem(priv, IGC_TDT0, 0);

  /* Free any pending TX */

  while (priv->tx_now != priv->tx_done)
    {
      /* Free net packet */

      netpkt_free(netdev, priv->tx_pkt[priv->tx_done], NETPKT_TX);

      /* Next descriptor */

      priv->tx_done = (priv->tx_done + 1) % IGC_TX_DESC;
    }

  priv->tx_now  = 0;
  priv->tx_done = 0;
}

/*****************************************************************************
 * Name: igc_rxclean
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
 *****************************************************************************/

static void igc_rxclean(FAR struct igc_driver_s *priv)
{
  priv->rx_now = 0;

  igc_putreg_mem(priv, IGC_RDH0, 0);
  igc_putreg_mem(priv, IGC_RDT0, 0);
}

/*****************************************************************************
 * Name: igc_transmit
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

static int igc_transmit(FAR struct netdev_lowerhalf_s *dev,
                          FAR netpkt_t *pkt)
{
  FAR struct igc_driver_s *priv = (FAR struct igc_driver_s *)dev;
  uint64_t                 pa   = 0;
  int                      desc = priv->tx_now;
  size_t                   len  = netpkt_getdatalen(dev, pkt);

  ninfo("transmit\n");

  /* Check the send length */

  if (len > IGC_PKTBUF_SIZE)
    {
      nerr("net transmit buffer too large\n");
      return -EINVAL;
    }

  if (!IFF_IS_RUNNING(dev->netdev.d_flags))
    {
      return -ENETDOWN;
    }

  /* Store TX packet reference */

  priv->tx_pkt[priv->tx_now] = pkt;

  /* Prepare next TX descriptor */

  priv->tx_now = (priv->tx_now + 1) % IGC_TX_DESC;

  /* Setup TX descriptor */

  pa = up_addrenv_va_to_pa(netpkt_getdata(dev, pkt));

  priv->tx[desc].addr   = pa;
  priv->tx[desc].len    = len;
  priv->tx[desc].cmd    = (IGC_TDESC_CMD_EOP | IGC_TDESC_CMD_IFCS |
                           IGC_TDESC_CMD_RS);
  priv->tx[desc].cso    = 0;
  priv->tx[desc].status = 0;

  UP_DSB();

  /* Update TX tail */

  igc_putreg_mem(priv, IGC_TDT0, priv->tx_now);

  ninfodumpbuffer("Transmitted:", netpkt_getdata(dev, pkt), len);

  return OK;
}

/*****************************************************************************
 * Name: igc_receive
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

static FAR netpkt_t * igc_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct igc_driver_s *priv = (FAR struct igc_driver_s *)dev;
  FAR netpkt_t            *pkt  = NULL;
  FAR struct igc_rx_leg_s *rx   = NULL;
  int                      desc = 0;

  desc = priv->rx_now;

  /* Get RX descriptor and RX packet */

  rx = &priv->rx[desc];
  pkt = priv->rx_pkt[desc];

  /* Check if descriptor done */

  if (!(rx->status & IGC_RDESC_STATUS_DD))
    {
      return NULL;
    }

  /* Next descriptor */

  priv->rx_now = (priv->rx_now + 1) % IGC_RX_DESC;

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

  igc_putreg_mem(priv, IGC_RDT0, desc);

  /* Handle errros */

  if (rx->errors)
    {
      nerr("RX error reported (%"PRIu8")\n", rx->errors);
      NETDEV_RXERRORS(&priv->dev);
      netpkt_free(dev, pkt, NETPKT_RX);
      return NULL;
    }

  return pkt;
}

/*****************************************************************************
 * Name: igc_txdone
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

static void igc_txdone(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct igc_driver_s *priv = (FAR struct igc_driver_s *)dev;

  while (priv->tx_now != priv->tx_done)
    {
      if (priv->tx[priv->tx_done].status == 0)
        {
          break;
        }

      if (!(priv->tx[priv->tx_done].status & IGC_TDESC_STATUS_DD))
        {
          nerr("tx failed: 0x%" PRIx32 "\n", priv->tx[priv->tx_done].status);
          NETDEV_TXERRORS(priv->dev);
        }

      /* Free net packet */

      netpkt_free(dev, priv->tx_pkt[priv->tx_done], NETPKT_TX);

      /* Next descriptor */

      priv->tx_done = (priv->tx_done + 1) % IGC_TX_DESC;
    }

  netdev_lower_txdone(dev);
}

/*****************************************************************************
 * Name: igc_link_work
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

static void igc_link_work(FAR void *arg)
{
  FAR struct igc_driver_s *priv = arg;
  uint32_t tmp;

  tmp = igc_getreg_mem(priv, IGC_STATUS);
  if (tmp & IGC_STATUS_LU)
    {
      ninfo("Link up, status = 0x%x\n", tmp);
      netdev_lower_carrier_on(&priv->dev);

      /* Clear Tx and RX rings */

      igc_txclean(priv);
      igc_rxclean(priv);
    }
  else
    {
      ninfo("Link down\n");
      netdev_lower_carrier_off(&priv->dev);
    }
}

/*****************************************************************************
 * Name: igc_misx_interrupt
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

static void igc_msix_interrupt(FAR struct igc_driver_s *priv)
{
  uint32_t icr  = 0;
  uint32_t eicr = 0;

  /* Get interrupts */

  icr  = igc_getreg_mem(priv, IGC_ICR);
  eicr = igc_getreg_mem(priv, IGC_EICR);

  ninfo("eicr = 0x%" PRIx32 " icr = 0x%" PRIx32 "\n", eicr, icr);

  if (icr == 0)
    {
      /* Ignore spurious interrupts */

      return;
    }

  /* Receiver Descriptor Write Back */

  if (icr & IGC_IC_RXDW)
    {
      netdev_lower_rxready(&priv->dev);
    }

  /* Link Status Change */

  if (icr & IGC_IC_LSC)
    {
      if (work_available(&priv->work))
        {
          /* Schedule to work queue because netdev_lower_carrier_xxx API
           * can't be used in interrupt context
           */

          work_queue(LPWORK, &priv->work, igc_link_work, priv, 0);
        }
    }

  /* Receiver Miss */

  if (icr & IGC_IC_RXMISS)
    {
      nerr("Receiver Miss\n");
      netdev_lower_rxready(&priv->dev);
    }

  /* Transmit Descriptor Written Back */

  if (icr & IGC_IC_TXDW)
    {
      igc_txdone(&priv->dev);
    }
}

/*****************************************************************************
 * Name: igc_interrupt
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

static int igc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct igc_driver_s *priv = (FAR struct igc_driver_s *)arg;

  DEBUGASSERT(priv != NULL);

  ninfo("interrupt!\n");

  /* Schedule to perform the interrupt processing on the worker thread. */

  igc_msix_interrupt(priv);

  return OK;
}

/*****************************************************************************
 * Name: igc_ifup
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

static int igc_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct igc_driver_s *priv = (FAR struct igc_driver_s *)dev;
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
  igc_enable(priv);
  priv->bifup = true;
  leave_critical_section(flags);

  return OK;
}

/*****************************************************************************
 * Name: igc_ifdown
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

static int igc_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct igc_driver_s *priv = (FAR struct igc_driver_s *)dev;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the igc_ifup() always
   * successfully brings the interface back up.
   */

  igc_disable(priv);

  /* Mark the device "down" */

  priv->bifup = false;
  leave_critical_section(flags);
  return OK;
}

#ifdef CONFIG_NET_MCASTGROUP
/*****************************************************************************
 * Name: igc_hashmta
 *
 * Note: This logic is based on freeBSD igc implementation
 *
 *****************************************************************************/

static uint32_t igc_hashmta(FAR struct igc_driver_s *priv,
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
 * Name: igc_addmac
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

static int igc_addmac(FAR struct netdev_lowerhalf_s *dev,
                        FAR const uint8_t *mac)
{
  FAR struct igc_driver_s *priv = (FAR struct igc_driver_s *)dev;
  uint16_t                 hash = 0;
  uint8_t                  row  = 0;
  uint8_t                  bit  = 0;
  int                      i    = 0;

  hash = igc_hashmta(priv, mac);
  bit = hash & 31;
  row = (hash >> 5) & (priv->type->mta_regs - 1);

  /* Bits 4:0 indicate bit in row word */

  priv->mta[row] |= (1 << bit);

  /* Replace the entire MTA */

  for (i = priv->type->mta_regs - 1; i >= 0; i--)
    {
      igc_putreg_mem(priv, IGC_MTA + (i << 2), priv->mta[i]);
    }

  return OK;
}

/*****************************************************************************
 * Name: igc_rmmac
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

static int igc_rmmac(FAR struct netdev_lowerhalf_s *dev,
                       FAR const uint8_t *mac)
{
  FAR struct igc_driver_s *priv = (FAR struct igc_driver_s *)dev;
  uint16_t                 hash = 0;
  uint8_t                  row  = 0;
  uint8_t                  bit  = 0;
  int                      i    = 0;

  hash = igc_hashmta(priv, mac);
  bit = hash & 31;
  row = (hash >> 5) & (priv->type->mta_regs - 1);

  /* Bits 4:0 indicate bit in row word */

  priv->mta[row] &= ~(1 << bit);

  /* Replace the entire MTA */

  for (i = priv->type->mta_regs - 1; i >= 0; i--)
    {
      igc_putreg_mem(priv, IGC_MTA + (i << 2), priv->mta[i]);
    }

  return OK;
}
#endif  /* CONFIG_NET_MCASTGROUP */

/*****************************************************************************
 * Name: igc_disable
 *
 * Description:
 *   Reset device to known state.
 *
 *****************************************************************************/

static void igc_disable(FAR struct igc_driver_s *priv)
{
  int i = 0;

  /* Reset Tx tail */

  igc_txclean(priv);

  /* Reset Rx tail */

  igc_rxclean(priv);

  /* Disable interrupts */

  igc_putreg_mem(priv, IGC_EIMC, IGC_MSIX_EIMS);
  igc_putreg_mem(priv, IGC_IMC, IGC_MSIX_IMS);
  up_disable_irq(priv->irq);

  /* Disable Transmiter */

  igc_putreg_mem(priv, IGC_TCTL, 0);

  /* Disable Receiver */

  igc_putreg_mem(priv, IGC_RCTL, 0);

  /* Free RX packets */

  for (i = 0; i < IGC_RX_DESC; i += 1)
    {
      netpkt_free(&priv->dev, priv->rx_pkt[i], NETPKT_RX);
    }
}

/*****************************************************************************
 * Name: igc_phy_reset
 *
 * Description:
 *   Reset PHY
 *
 *****************************************************************************/

static void igc_phy_reset(FAR struct igc_driver_s *priv)
{
  uint32_t regval = 0;

  regval = igc_getreg_mem(priv, IGC_CTRL);
  igc_putreg_mem(priv, IGC_CTRL, regval | IGC_CTRL_PHYRST);
  up_udelay(100);
  igc_putreg_mem(priv, IGC_CTRL, regval);
  up_udelay(100);
}

/*****************************************************************************
 * Name: igc_enable
 *
 * Description:
 *   Enable device.
 *
 *****************************************************************************/

static void igc_enable(FAR struct igc_driver_s *priv)
{
  FAR struct netdev_lowerhalf_s *dev = (FAR struct netdev_lowerhalf_s *)priv;
  uint64_t pa     = 0;
  uint32_t regval = 0;
  int      i      = 0;

  /* Reset PHY */

  igc_phy_reset(priv);

  /* Reset Multicast Table Array */

  for (i = 0; i < priv->type->mta_regs; i++)
    {
      igc_putreg_mem(priv, IGC_MTA + (i << 2), 0);
    }

  /* Allocate RX packets */

  for (i = 0; i < IGC_RX_DESC; i += 1)
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
  igc_putreg_mem(priv, IGC_TDBAL0, regval);
  regval = (uint32_t)(pa >> 32);
  igc_putreg_mem(priv, IGC_TDBAH0, regval);

  regval = IGC_TX_DESC * sizeof(struct igc_tx_leg_s);
  igc_putreg_mem(priv, IGC_TDLEN0, regval);

  /* Reset TX tail */

  igc_txclean(priv);

  /* Setup RX descriptor */

  /* The address passed to the NIC must be physical */

  pa = up_addrenv_va_to_pa(priv->rx);

  regval = (uint32_t)pa;
  igc_putreg_mem(priv, IGC_RDBAL0, regval);
  regval = (uint32_t)(pa >> 32);
  igc_putreg_mem(priv, IGC_RDBAH0, regval);

  regval = IGC_RX_DESC * sizeof(struct igc_rx_leg_s);
  igc_putreg_mem(priv, IGC_RDLEN0, regval);

  /* Enable interrupts */

  igc_putreg_mem(priv, IGC_EIMS, IGC_MSIX_EIMS);
  igc_putreg_mem(priv, IGC_IMS, IGC_MSIX_IMS);
  up_enable_irq(priv->irq);

  /* Set link up */

  igc_putreg_mem(priv, IGC_CTRL, IGC_CTRL_SLU);

  /* Setup and enable Transmiter */

  regval = igc_getreg_mem(priv, IGC_TCTL);
  regval |= IGC_TCTL_EN | IGC_TCTL_PSP;
  igc_putreg_mem(priv, IGC_TCTL, regval);

  /* Setup and enable Receiver */

  regval = (IGC_RCTL_EN | IGC_RCTL_MPE |
            (IGC_RCTL_BSIZE << IGC_RCTL_BSIZE_SHIFT));
#ifdef CONFIG_NET_PROMISCUOUS
  regval |= IGC_RCTL_UPE | IGC_RCTL_MPE;
#endif
  igc_putreg_mem(priv, IGC_RCTL, regval);

  /* Enable TX queeu */

  regval = igc_getreg_mem(priv, IGC_TXDCTL0);
  regval |= IGC_TXDCTL_ENABLE;
  igc_putreg_mem(priv, IGC_TXDCTL0, regval);

  /* Enable RX queue */

  regval = igc_getreg_mem(priv, IGC_RXDCTL0);
  regval |= IGC_RXDCTL_ENABLE;
  igc_putreg_mem(priv, IGC_RXDCTL0, regval);

  /* Reset RX tail - after queue is enabled */

  igc_rxclean(priv);

  /* All RX descriptors availalbe */

  igc_putreg_mem(priv, IGC_RDT0, IGC_RX_DESC);

#ifdef CONFIG_DEBUG_NET_INFO
  /* Dump memory */

  igc_dump_mem(priv, "enabled");
#endif
}

/*****************************************************************************
 * Name: igc_initialize
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int igc_initialize(FAR struct igc_driver_s *priv)
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

  irq_attach(priv->irq, igc_interrupt, priv);

  /* Connect MSI */

  ret = pci_connect_irq(priv->pcidev, &priv->irq, 1);
  if (ret != OK)
    {
      nerr("Failed to connect MSI %d\n", ret);
      pci_release_irq(priv->pcidev, &priv->irq, 1);

      return -ENOTSUP;
    }

  /* Clear previous Extended Interrupt Mask */

  igc_putreg_mem(priv, IGC_EIMC, 0xffffffff);
  igc_putreg_mem(priv, IGC_IMC, 0xffffffff);

  /* Configure MSI-X */

  igc_putreg_mem(priv, IGC_IVAR0, IGC_MSIX_IVAR0);
  igc_putreg_mem(priv, IGC_IVARMSC, IGC_MSIX_IVARMSC);

  /* Enable MSI-X Single Vector */

  igc_putreg_mem(priv, IGC_GPIE, IGC_GPIE_MSIX_SINGLE);
  igc_putreg_mem(priv, IGC_EIMS, IGC_MSIX_EIMS);

  /* Configure Other causes */

  igc_putreg_mem(priv, IGC_IMS, IGC_MSIX_IMS);

  /* Configure Interrupt Throttle */

  igc_putreg_mem(priv, IGC_EITR0, (CONFIG_NET_IGC_INT_INTERVAL << 2));

  /* Get MAC if valid */

  regval = igc_getreg_mem(priv, IGC_RAH);
  if (regval & IGC_RAH_AV)
    {
      mac = ((uint64_t)regval & IGC_RAH_RAH_MASK) << 32;
      mac |= igc_getreg_mem(priv, IGC_RAL);
      memcpy(&priv->dev.netdev.d_mac.ether, &mac, sizeof(struct ether_addr));
    }
  else
    {
      nwarn("Receive Address not vaild!\n");
    }

  return OK;
}

/*****************************************************************************
 * Name: igc_probe
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int igc_probe(FAR struct pci_device_s *dev)
{
  FAR const struct igc_type_s   *type   = NULL;
  FAR struct igc_driver_s       *priv   = NULL;
  FAR struct netdev_lowerhalf_s *netdev = NULL;
  int                            ret    = -ENOMEM;

  /* Get type data associated with this PCI device card */

  type = (FAR const struct igc_type_s *)dev->id->driver_data;

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
                          IGC_TX_DESC * sizeof(struct igc_tx_leg_s));
  if (priv->tx == NULL)
    {
      nerr("alloc tx failed %d\n", errno);
      goto errout;
    }

  /* Allocate RX descriptors */

  priv->rx = kmm_memalign(type->desc_align,
                          IGC_RX_DESC * sizeof(struct igc_rx_leg_s));
  if (priv->rx == NULL)
    {
      nerr("alloc rx failed %d\n", errno);
      goto errout;
    }

  /* Allocate TX packet pointer array */

  priv->tx_pkt = kmm_zalloc(IGC_TX_DESC * sizeof(netpkt_t *));
  if (priv->tx_pkt == NULL)
    {
      nerr("alloc tx_pkt failed\n");
      goto errout;
    }

  /* Allocate RX packet pointer array */

  priv->rx_pkt = kmm_zalloc(IGC_RX_DESC * sizeof(netpkt_t *));
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

  priv->base = (uintptr_t)pci_map_bar(dev, IGC_MMIO_BAR);
  if (!priv->base)
    {
      pcierr("Not found MMIO control bar\n");
      goto errout;
    }

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  ret = igc_initialize(priv);
  if (ret != OK)
    {
      nerr("igc_initialize failed %d\n", ret);
      return ret;
    }

  /* Register the network device */

  netdev->quota[NETPKT_TX] = IGC_TX_QUOTA;
  netdev->quota[NETPKT_RX] = IGC_RX_QUOTA;
  netdev->ops = &g_igc_ops;

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
 * Name: pci_igc_init
 *
 * Description:
 *   Register a pci driver
 *
 *****************************************************************************/

int pci_igc_init(void)
{
  return pci_register_driver(&g_pci_igc_drv);
}
