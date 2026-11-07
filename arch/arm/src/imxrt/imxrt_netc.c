/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <sys/param.h>
#include <debug.h>
#include <endian.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "imxrt_netc_priv.h"
#include "imxrt_clockconfig.h"
#include "imxrt_dtcm.h"
#include "hardware/imxrt_netc.h"
#include "hardware/imxrt_src.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error NETC requires work queue support
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

#ifdef CONFIG_NET_PROMISCUOUS
#  error "NETC does not support promiscuous mode; disable CONFIG_NET_PROMISCUOUS"
#endif
#if CONFIG_NET_ETH_PKTSIZE > 1518
#  error "NETC requires CONFIG_NET_ETH_PKTSIZE <= 1518"
#endif
#if CONFIG_NET_LL_GUARDSIZE > 32
#  error "NETC requires CONFIG_NET_LL_GUARDSIZE <= 32"
#endif
#if CONFIG_IOB_BUFSIZE < 128
#  error "NETC requires CONFIG_IOB_BUFSIZE >= 128"
#endif
#if CONFIG_IOB_NBUFFERS < 96
#  error "NETC requires CONFIG_IOB_NBUFFERS >= 96"
#endif
#if CONFIG_IOB_THROTTLE > 16
#  error "NETC requires CONFIG_IOB_THROTTLE <= 16"
#endif

#define NETC_LINK_INTERVAL MSEC2TICK(CONFIG_IMXRT_NETC_LINK_POLL_MS)
#define NETC_TX_TIMEOUT    SEC2TICK(5)
#define NETC_TX_RING       1
#define TXREG(p, r)        ((p)->hw->si + NETC_TX_BASE(NETC_TX_RING) + (r))
#define RXREG(p, r)        ((p)->hw->si + NETC_RX_BASE(0) + (r))

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum imxrt_netc_state_e
{
  NETC_UNINITIALIZED = 0,
  NETC_DOWN,
  NETC_UP,
  NETC_RECOVERING,
  NETC_FAILED
};

struct imxrt_netc_dma_s
{
  struct imxrt_netc_txdesc_s tx[NETC_TX_SLOTS];
  struct imxrt_netc_rxdesc_s rx[NETC_RX_SLOTS]
    aligned_data(NETC_DMA_ALIGN);
  uint8_t txbuffer[NETC_TX_SLOTS][NETC_BUFFER_SIZE]
    aligned_data(NETC_DMA_ALIGN);
  uint8_t rxbuffer[NETC_RX_SLOTS][NETC_BUFFER_SIZE]
    aligned_data(NETC_DMA_ALIGN);
};

struct imxrt_netc_s
{
  struct netdev_lowerhalf_s lower;
  const struct imxrt_netc_hw_s *hw;
  struct imxrt_netc_enetc_s config;
  struct imxrt_netc_link_s link;
  struct imxrt_netc_dma_s *dma_cpu;
  uint32_t dma_bus;
  enum imxrt_netc_state_e state;
  struct work_s linkwork;
  netpkt_t *txpacket[NETC_TX_SLOTS];
  clock_t txtime[NETC_TX_SLOTS];
  uintptr_t msgintr;
  int irq;
  uint16_t txprod;
  uint16_t txclean;
  uint16_t rxcons;
  bool rxdiscard;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int imxrt_netc_ifup(struct netdev_lowerhalf_s *lower);
static int imxrt_netc_ifdown(struct netdev_lowerhalf_s *lower);
static int imxrt_netc_transmit(struct netdev_lowerhalf_s *lower,
                              netpkt_t *packet);
static netpkt_t *imxrt_netc_receive(struct netdev_lowerhalf_s *lower);
static void imxrt_netc_reclaim(struct netdev_lowerhalf_s *lower);
#ifdef CONFIG_NET_MCASTGROUP
static int imxrt_netc_multicast(struct netdev_lowerhalf_s *lower,
                               const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int imxrt_netc_ioctl(struct netdev_lowerhalf_s *lower, int cmd,
                            unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_netc_ops =
{
  .ifup     = imxrt_netc_ifup,
  .ifdown   = imxrt_netc_ifdown,
  .transmit = imxrt_netc_transmit,
  .receive  = imxrt_netc_receive,
  .reclaim  = imxrt_netc_reclaim,
#ifdef CONFIG_NET_MCASTGROUP
  .addmac   = imxrt_netc_multicast,
  .rmmac    = imxrt_netc_multicast,
#endif
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = imxrt_netc_ioctl,
#endif
};

static const struct imxrt_netc_hw_s g_netc_hw[IMXRT_NETC_NENETC] =
{
  {
    .si = IMXRT_ENETC0_SI0_BASE,
    .enetc = IMXRT_ENETC0_BASE,
    .port = IMXRT_ENETC0_PORT_BASE,
    .mac = IMXRT_ENETC0_ETH_MAC_BASE,
    .function = IMXRT_NETC_ENETC0_PF,
    .msix = IMXRT_NETC_ENETC0_MSIX_BASE,
    .attachment = IMXRT_NETC_DIRECT,
    .portcount = 1,
    .ports =
      {
        {
          .reg_idx = 4,
          .mdio = IMXRT_NETC_ENETC0_MDIO,
        },
      }
  },
  {
    .si = IMXRT_ENETC1_SI0_BASE,
    .enetc = IMXRT_ENETC1_BASE,
    .port = IMXRT_ENETC1_PORT_BASE,
    .mac = IMXRT_ENETC1_ETH_MAC_BASE,
    .function = IMXRT_NETC_ENETC1_PF,
    .msix = IMXRT_NETC_ENETC1_MSIX_BASE,
    .attachment = IMXRT_NETC_SWITCH,
    .portcount = IMXRT_NETC_SW0_NPORTS,
    .sw_cpu_port_reg_idx = 4,
    .ports =
      {
        {
          .reg_idx = 0,
          .mdio = IMXRT_NETC_PORT0_MDIO,
        },
        {
          .reg_idx = 1,
          .mdio = IMXRT_NETC_PORT1_MDIO,
        },
        {
          .reg_idx = 2,
          .mdio = IMXRT_NETC_PORT2_MDIO,
        },
        {
          .reg_idx = 3,
          .mdio = IMXRT_NETC_PORT3_MDIO,
        },
      },
  }
};

static const uintptr_t g_msgintr_base[6] =
{
  IMXRT_MSGINTR1_BASE, IMXRT_MSGINTR2_BASE, IMXRT_MSGINTR3_BASE,
  IMXRT_MSGINTR4_BASE, IMXRT_MSGINTR5_BASE, IMXRT_MSGINTR6_BASE
};

static const int g_msgintr_irq[6] =
{
  IMXRT_IRQ_MSGINTR1, IMXRT_IRQ_MSGINTR2, IMXRT_IRQ_MSGINTR3,
  IMXRT_IRQ_MSGINTR4, IMXRT_IRQ_MSGINTR5, IMXRT_IRQ_MSGINTR6
};

static struct imxrt_netc_s g_netc[IMXRT_NETC_NENETC];
static mutex_t g_initlock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool imxrt_netc_macvalid(const uint8_t *mac)
{
  static const uint8_t zero[6];

  return (mac[0] & 1) == 0 && memcmp(mac, zero, sizeof(zero)) != 0;
}

static int
imxrt_netc_shared_initialize(const struct imxrt_netc_config_s *config,
                          uint32_t clock_hz)
{
  uintptr_t prb = IMXRT_NETC_PRB_BASE;
  uintptr_t ierb = IMXRT_NETC_IERB_BASE;
  uint32_t mhz = clock_hz / 1000000;
  uint32_t period;
  uint32_t fraction;
  uint32_t status;
  uint8_t phyaddr[IMXRT_NETC_PHY_PORT_COUNT];
  const struct imxrt_netc_enetc_s *enetc;
  const struct imxrt_netc_hw_s *hw;
  const struct imxrt_netc_port_s *port;
  enum imxrt_netc_mdio_e mdio;
  unsigned int link;
  size_t index;
  size_t portindex;
  unsigned int function;
  bool locked;
  int ret;

  memset(phyaddr, UINT8_MAX, sizeof(phyaddr));
  for (index = 0; index < nitems(config->enetc); index++)
    {
      enetc = &config->enetc[index];
      if (!enetc->enabled)
        {
          continue;
        }

      hw = &g_netc_hw[index];
      for (portindex = 0; portindex < hw->portcount; portindex++)
        {
          port = &enetc->ports[portindex];
          mdio = hw->ports[portindex].mdio;

          if (!port->enabled || mdio == IMXRT_NETC_EMDIO)
            {
              continue;
            }

          link = hw->ports[portindex].reg_idx;
          DEBUGASSERT(link < nitems(phyaddr));
          DEBUGASSERT(phyaddr[link] == UINT8_MAX ||
              phyaddr[link] == port->phy.address_lo);

          phyaddr[link] = port->phy.address_lo;
        }
    }

  ret = imxrt_netc_wait(prb + NETC_PRB_NETCRR, NETC_PRB_SR, 0);
  if (ret < 0)
    {
      return ret;
    }

  locked = (getreg32(prb + NETC_PRB_NETCRR) & NETC_PRB_LOCK) != 0;
  if (!locked)
    {
      for (function = 0; function < 5; function++)
        {
          if ((getreg16(IMXRT_NETC_PF(function) + NETC_PCI_CMD) &
               NETC_PCI_CMD_ENABLE) != 0)
            {
              return -EBUSY;
            }
        }

      period = 1000000000 / clock_hz;
      fraction = ((uint64_t)(1000000000 % clock_hz) << 32) / clock_hz;
      putreg32(fraction, ierb + NETC_IERB_CLKFR);
      putreg32(NETC_IERB_CLOCK(mhz, period), ierb + NETC_IERB_CLKCR);
      for (link = 0; link < nitems(phyaddr); link++)
        {
          if (phyaddr[link] != UINT8_MAX)
            {
              modifyreg32(ierb + NETC_IERB_LINKBCR(link),
                          NETC_IERB_PHYADDR_MASK,
                          NETC_IERB_PHYADDR(phyaddr[link]));
            }
        }

      modifyreg32(prb + NETC_PRB_NETCRR, 0, NETC_PRB_LOCK);
    }
  else if ((getreg32(ierb + NETC_IERB_CLKCR) & 0x7ff) != mhz)
    {
      /* Unlocking would reset every function, including another owner. */

      return -EBUSY;
    }

  for (link = 0; locked && link < nitems(phyaddr); link++)
    {
      if (phyaddr[link] != UINT8_MAX &&
          (getreg32(ierb + NETC_IERB_LINKBCR(link)) &
           NETC_IERB_PHYADDR_MASK) != NETC_IERB_PHYADDR(phyaddr[link]))
        {
          return -EBUSY;
        }
    }

  ret = imxrt_netc_wait(prb + NETC_PRB_NETCSR, NETC_PRB_STATE, 0);
  if (ret == OK)
    {
      status = getreg32(prb + NETC_PRB_NETCSR);
      if ((status & NETC_PRB_ERROR) != 0)
        {
          nerr("NETC invalid IERB configuration: NETCSR=%08lx\n",
               (unsigned long)status);
          ret = -EIO;
        }
    }

  return ret;
}

static void imxrt_netc_fault(struct imxrt_netc_s *priv)
{
  priv->state = NETC_RECOVERING;
  up_disable_irq(priv->irq);
  netdev_lower_carrier_off(&priv->lower);
}

static void imxrt_netc_release_tx(struct imxrt_netc_s *priv)
{
  unsigned int slot;

  for (slot = 0; slot < NETC_TX_SLOTS; slot++)
    {
      if (priv->txpacket[slot] != NULL)
        {
          NETDEV_TXERRORS(&priv->lower.netdev);
          netpkt_free(&priv->lower, priv->txpacket[slot], NETPKT_TX);
          priv->txpacket[slot] = NULL;
        }
    }

  priv->txprod = 0;
  priv->txclean = 0;
}

static int imxrt_netc_stop(struct imxrt_netc_s *priv)
{
  int ret;

  up_disable_irq(priv->irq);
  netdev_lower_carrier_off(&priv->lower);
  if (priv->hw->attachment == IMXRT_NETC_SWITCH)
    {
      ret = imxrt_netc_switch_cpu(false, NULL);
    }
  else
    {
      /* A failed start may not have enabled register access yet. */

      putreg16(NETC_PCI_CMD_ENABLE, priv->hw->function + NETC_PCI_CMD);
      ret = imxrt_netc_wait(priv->hw->enetc + NETC_COMMON_OSR,
                            NETC_COMMON_STATE, 0);
      if (ret == OK)
        {
          putreg32(0, priv->hw->si + NETC_SI_SIMR);

          /* ERR051936: leave half duplex before quiescing and PF reset. */

          modifyreg32(priv->hw->mac + NETC_MAC_IFMODE,
                      NETC_MAC_IF_HALF, 0);
          ret = imxrt_netc_port_mode(priv->hw->port, priv->hw->mac,
                                      priv->config.ports[0].phy.iftype, 0);
          if (ret == OK)
            {
              ret = imxrt_netc_wait(priv->hw->port + NETC_PORT_PSR,
                                    NETC_PORT_BUSY, 0);
            }
        }

      priv->link.speed = 0;
    }

  if (ret < 0)
    {
      nerr("NETC port stop failed: %d\n", ret);
      priv->state = NETC_FAILED;
      return ret;
    }

  /* Only this PF is reset; no VFs are enabled. Pending DMA storage is
   * retained unless reset and readiness checks have completed.
   */

  ret = imxrt_netc_reset(priv->hw->function, priv->hw->enetc);
  if (ret < 0)
    {
      nerr("NETC function reset failed: %d\n", ret);
      priv->state = NETC_FAILED;
      return ret;
    }

  putreg16(0, priv->hw->function + NETC_PCI_CMD);
  imxrt_netc_release_tx(priv);
  priv->state = NETC_DOWN;
  return OK;
}

static void imxrt_netc_rxarm(struct imxrt_netc_s *priv, unsigned int slot)
{
  struct imxrt_netc_rxdesc_s *desc = &priv->dma_cpu->rx[slot];

  /* No dirty payload lines may survive the transfer back to DMA. */

  up_invalidate_dcache((uintptr_t)priv->dma_cpu->rxbuffer[slot],
                       (uintptr_t)priv->dma_cpu->rxbuffer[slot] +
                       NETC_BUFFER_SIZE);
  memset(desc, 0, sizeof(*desc));
  desc->address = htole32(priv->dma_bus +
                          offsetof(struct imxrt_netc_dma_s, rxbuffer) +
                          slot * NETC_BUFFER_SIZE);
  up_clean_dcache((uintptr_t)desc, (uintptr_t)desc + sizeof(*desc));
}

static int imxrt_netc_start(struct imxrt_netc_s *priv)
{
  const uint8_t *mac = priv->lower.netdev.d_mac.ether.ether_addr_octet;
  unsigned int slot;
  unsigned int reason;
  uint32_t address;
  uint32_t status;
  int ret;

  if (!imxrt_netc_macvalid(mac))
    {
      return -EINVAL;
    }

  putreg16(NETC_PCI_CMD_ENABLE, priv->hw->function + NETC_PCI_CMD);
  ret = imxrt_netc_wait(priv->hw->enetc + NETC_COMMON_OSR,
                        NETC_COMMON_STATE, 0);
  if (ret < 0)
    {
      return ret;
    }

  status = getreg32(priv->hw->enetc + NETC_COMMON_OSR);
  if ((status & NETC_COMMON_ITM_STATE) != 0)
    {
      nerr("NETC ENETC at %08lx index table memory overprovisioned: "
           "OSR=%08lx\n",
           (unsigned long)priv->hw->enetc, (unsigned long)status);
      return -EIO;
    }

  putreg32(NETC_PORT_DISABLE, priv->hw->port + NETC_PORT_POR);
  putreg32(0, priv->hw->si + NETC_SI_SIMR);

  /* TX0 is reserved for direct switch enqueue; normal traffic uses TX1. */

  putreg32(NETC_ENETC_RINGS(NETC_TX_RING + 1, 1),
           priv->hw->enetc + NETC_ENETC_PSICFGR0);
  putreg32(0, priv->hw->enetc + NETC_ENETC_PSICFGR2);
  putreg32(NETC_ENETC_SI0EN, priv->hw->enetc + NETC_ENETC_PMR);

  address = mac[0] | ((uint32_t)mac[1] << 8) |
            ((uint32_t)mac[2] << 16) | ((uint32_t)mac[3] << 24);
  putreg32(address, priv->hw->port + NETC_PORT_PMAR0);
  putreg32(mac[4] | ((uint32_t)mac[5] << 8),
           priv->hw->port + NETC_PORT_PMAR1);
  putreg32(NETC_ENETC_MCPROMISC, priv->hw->enetc + NETC_ENETC_PSIPMMR);
  putreg32(NETC_ENETC_VLANPROMISC | NETC_ENETC_VLANUNTAGGED,
           priv->hw->enetc + NETC_ENETC_PSIPVMR);
  putreg32(0, priv->hw->enetc + NETC_ENETC_PARCSCR);
  putreg32(0, priv->hw->enetc + NETC_ENETC_PRIO2TCMR0);
  if (priv->hw->attachment == IMXRT_NETC_SWITCH)
    {
      for (reason = 1; reason < 16; reason++)
        {
          putreg32(0, priv->hw->enetc + NETC_ENETC_SMHRBDRMR(reason));
        }

      imxrt_netc_pseudo(priv->hw->port);
      putreg32(NETC_PSEUDO_TXPAD, priv->hw->mac + NETC_PSEUDO_PPMCR);
    }
  else
    {
      /* Negotiated speed must be reapplied after every PF reset. Keep the
       * physical MAC and port gated until direct-link monitoring does so.
       */

      priv->link.speed = 0;
      modifyreg32(priv->hw->mac + NETC_MAC_CMD,
                  NETC_MAC_TX | NETC_MAC_RX, NETC_MAC_TXPAD);
      modifyreg32(priv->hw->mac + NETC_MAC_IFMODE, NETC_MAC_IF_MASK,
                  priv->config.ports[0].phy.iftype);
      putreg32(NETC_PACKET_SIZE + 4, priv->hw->mac + NETC_MAC_MAXFRM);
      putreg32(64, priv->hw->mac + NETC_MAC_MINFRM);
      imxrt_netc_port_sdu(priv->hw->port);
    }

  putreg32(NETC_DMA_NONCOHERENT, priv->hw->si + NETC_SI_SIBCAR);
  putreg32(0, priv->hw->si + NETC_SI_SIRBGCR);
  putreg32(0, priv->hw->si + NETC_SI_SIIPVBDRMR0);
  priv->txprod = 0;
  priv->txclean = 0;
  priv->rxcons = 0;
  priv->rxdiscard = false;
  memset(priv->dma_cpu, 0, sizeof(*priv->dma_cpu));
  up_clean_dcache((uintptr_t)priv->dma_cpu,
                 (uintptr_t)priv->dma_cpu + sizeof(*priv->dma_cpu));

  for (slot = 0; slot < NETC_RX_SLOTS; slot++)
    {
      imxrt_netc_rxarm(priv, slot);
    }

  putreg32(priv->dma_bus + offsetof(struct imxrt_netc_dma_s, tx),
           TXREG(priv, NETC_BDR_BAR0));
  putreg32(0, TXREG(priv, NETC_BDR_BAR1));
  putreg32(0, TXREG(priv, NETC_BDR_PIR));
  putreg32(0, TXREG(priv, NETC_TX_CIR));
  putreg32(CONFIG_IMXRT_NETC_NTXDESC, TXREG(priv, NETC_BDR_LENR));
  putreg32(0, TXREG(priv, NETC_BDR_ICR0));
  putreg32(0, TXREG(priv, NETC_BDR_ICR1));
  putreg32(priv->dma_bus + offsetof(struct imxrt_netc_dma_s, rx),
           RXREG(priv, NETC_BDR_BAR0));
  putreg32(0, RXREG(priv, NETC_BDR_BAR1));
  putreg32(0, RXREG(priv, NETC_BDR_PIR));
  putreg32(0, RXREG(priv, NETC_RX_CIR));
  putreg32(NETC_RX_SLOTS, RXREG(priv, NETC_BDR_LENR));
  putreg32(NETC_BUFFER_SIZE, RXREG(priv, NETC_RX_BSR));
  putreg32(0, RXREG(priv, NETC_BDR_ICR1));
  putreg32(NETC_RX_COALESCE | NETC_RX_THRESHOLD,
           RXREG(priv, NETC_BDR_ICR0));

  /* ERR051622: use the fixed EA MSI-X resource, not the reported BIR. */

  putreg32(NETC_MSIX_VECTOR_MASK, priv->hw->msix + NETC_MSIX_CONTROL);
  putreg32(priv->msgintr + NETC_MSGINTR_MSIIR,
           priv->hw->msix + NETC_MSIX_ADDRESS);
  putreg32(0, priv->hw->msix + NETC_MSIX_ADDRESS_HI);
  putreg32(0, priv->hw->msix + NETC_MSIX_DATA);
  putreg32(0, priv->hw->si + NETC_SI_SIMSITRVR(NETC_TX_RING));
  putreg32(0, priv->hw->si + NETC_SI_SIMSIRRVR(0));
  getreg32(priv->msgintr + NETC_MSGINTR_MSIR);
  getreg32(TXREG(priv, NETC_BDR_IDR));
  getreg32(RXREG(priv, NETC_BDR_IDR));
  putreg32(NETC_TX_FRAME_IE, TXREG(priv, NETC_BDR_IER));
  putreg32(NETC_RX_THRESHOLD_IE, RXREG(priv, NETC_BDR_IER));
  putreg32(0, priv->hw->msix + NETC_MSIX_CONTROL);
  modifyreg16(priv->hw->function + NETC_PCI_MSIXCTL,
              NETC_PCI_MSIX_MASK, NETC_PCI_MSIX_ENABLE);
  UP_DSB();
  putreg32(NETC_BDR_EN | NETC_RX_EXTENDED, RXREG(priv, NETC_BDR_MR));
  putreg32(NETC_BDR_EN, TXREG(priv, NETC_BDR_MR));
  putreg32(NETC_SI_EN, priv->hw->si + NETC_SI_SIMR);
  if (priv->hw->attachment == IMXRT_NETC_SWITCH)
    {
      putreg32(0, priv->hw->port + NETC_PORT_POR);
      ret = imxrt_netc_switch_cpu(true, mac);
      if (ret < 0)
        {
          return ret;
        }
    }

  priv->state = NETC_UP;
  up_enable_irq(priv->irq);
  return OK;
}

static int imxrt_netc_interrupt(int irq, void *context, void *arg)
{
  struct imxrt_netc_s *priv = arg;

  getreg32(priv->msgintr + NETC_MSGINTR_MSIR);

  /* Reading IDR acknowledges events and rearms coalescing. Hardware
   * generates a new interrupt on a simultaneous event/ack collision.
   * Reclamation and protocol dispatch remain with the upper-half worker.
   */

  getreg32(TXREG(priv, NETC_BDR_IDR));
  getreg32(RXREG(priv, NETC_BDR_IDR));
  netdev_lower_rxready(&priv->lower);
  return OK;
}

static void imxrt_netc_carrier(struct imxrt_netc_s *priv, uint8_t linkmask)
{
  if (priv->state == NETC_UP && linkmask != 0)
    {
      netdev_lower_carrier_on(&priv->lower);
    }
  else
    {
      netdev_lower_carrier_off(&priv->lower);
    }
}

static uint8_t imxrt_netc_direct_link(struct imxrt_netc_s *priv)
{
  uint16_t speed;
  int ret;

  if (priv->state != NETC_UP)
    {
      return 0;
    }

  ret = imxrt_netc_port_link(&priv->config.ports[0],
                            &priv->link, &speed);
  if (ret < 0)
    {
      nerr("NETC direct PHY link failed: %d\n", ret);
      speed = 0;
    }

  if (speed != priv->link.speed)
    {
      netdev_lower_carrier_off(&priv->lower);
      ret = imxrt_netc_port_mode(priv->hw->port, priv->hw->mac,
                                  priv->config.ports[0].phy.iftype, speed);
      if (ret < 0)
        {
          nerr("NETC direct MAC mode failed: %d\n", ret);
          priv->link.speed = 0;
          imxrt_netc_fault(priv);
          return 0;
        }

      priv->link.speed = speed;
      ninfo("NETC direct link: %u Mbps\n", speed);
    }

  return speed != 0;
}

static void imxrt_netc_linkwork(void *arg)
{
  struct imxrt_netc_s *priv = arg;
  uint8_t linkmask = 0;
  int ret;

  net_lock();
  if (priv->state == NETC_UNINITIALIZED)
    {
      net_unlock();
      return;
    }

  /* Serialize ring access and recovery with upper-half packet callbacks. */

  netdev_lock(&priv->lower.netdev);

  if (priv->hw->attachment == IMXRT_NETC_SWITCH)
    {
      imxrt_netc_switch_link(&linkmask);
    }

  if (priv->state == NETC_UP)
    {
      imxrt_netc_reclaim(&priv->lower);
      if (priv->state == NETC_UP && priv->txclean != priv->txprod &&
          clock_systime_ticks() - priv->txtime[priv->txclean] >
          NETC_TX_TIMEOUT)
        {
          nerr("NETC transmit timeout\n");
          NETDEV_TXTIMEOUTS(&priv->lower.netdev);
          imxrt_netc_fault(priv);
        }
    }

  if (priv->state == NETC_RECOVERING)
    {
      ret = imxrt_netc_stop(priv);
      if (ret == OK)
        {
          ret = imxrt_netc_start(priv);
        }

      if (ret < 0)
        {
          priv->state = NETC_FAILED;
          nerr("NETC recovery failed: %d\n", ret);
        }
      else
        {
          netdev_lower_rxready(&priv->lower);
        }
    }

  if (priv->hw->attachment == IMXRT_NETC_DIRECT)
    {
      linkmask = imxrt_netc_direct_link(priv);
    }

  imxrt_netc_carrier(priv, linkmask);
  ret = work_queue(ETHWORK, &priv->linkwork, imxrt_netc_linkwork, priv,
                   NETC_LINK_INTERVAL);
  if (ret < 0)
    {
      nerr("NETC link monitoring stopped: %d\n", ret);
      imxrt_netc_stop(priv);
      priv->state = NETC_FAILED;
    }

  netdev_unlock(&priv->lower.netdev);
  net_unlock();
}

static int imxrt_netc_ifup(struct netdev_lowerhalf_s *lower)
{
  struct imxrt_netc_s *priv = (struct imxrt_netc_s *)lower;
  uint8_t linkmask = 0;
  int ret;

  if (priv->state == NETC_UP)
    {
      return OK;
    }

  if (priv->state != NETC_DOWN)
    {
      return -EIO;
    }

  ret = imxrt_netc_start(priv);
  if (ret < 0)
    {
      nerr("NETC interface start failed: %d\n", ret);
      imxrt_netc_stop(priv);
      return ret;
    }

  if (priv->hw->attachment == IMXRT_NETC_SWITCH)
    {
      imxrt_netc_switch_link(&linkmask);
    }
  else
    {
      linkmask = imxrt_netc_direct_link(priv);
    }

  imxrt_netc_carrier(priv, linkmask);
  return OK;
}

static int imxrt_netc_ifdown(struct netdev_lowerhalf_s *lower)
{
  struct imxrt_netc_s *priv = (struct imxrt_netc_s *)lower;
  bool failed = priv->state == NETC_FAILED;
  int ret;

  if (priv->state == NETC_DOWN)
    {
      return OK;
    }

  ret = imxrt_netc_stop(priv);
  if (failed)
    {
      priv->state = NETC_FAILED;
    }

  if (ret < 0)
    {
      nerr("NETC interface stop failed: %d\n", ret);
    }

  return ret;
}

static void imxrt_netc_reclaim(struct netdev_lowerhalf_s *lower)
{
  struct imxrt_netc_s *priv = (struct imxrt_netc_s *)lower;
  struct imxrt_netc_txdesc_s *desc;
  unsigned int consumer;
  uint32_t flags;

  if (priv->state != NETC_UP)
    {
      return;
    }

  if ((getreg32(TXREG(priv, NETC_BDR_SR)) & NETC_BDR_SBE) != 0)
    {
      nerr("NETC transmit DMA bus error\n");
      imxrt_netc_fault(priv);
      return;
    }

  consumer = getreg32(TXREG(priv, NETC_TX_CIR)) & NETC_BDR_INDEX_MASK;
  if (consumer >= CONFIG_IMXRT_NETC_NTXDESC)
    {
      nerr("NETC invalid transmit consumer: %u\n", consumer);
      imxrt_netc_fault(priv);
      return;
    }

  /* The extension is the final BD. Never recycle half a descriptor pair. */

  consumer /= 2;
  if ((consumer + NETC_TX_SLOTS - priv->txclean) % NETC_TX_SLOTS >
      (priv->txprod + NETC_TX_SLOTS - priv->txclean) % NETC_TX_SLOTS)
    {
      nerr("NETC transmit consumer exceeds outstanding packets\n");
      imxrt_netc_fault(priv);
      return;
    }

  UP_DMB();
  while (priv->txclean != consumer)
    {
      desc = &priv->dma_cpu->tx[priv->txclean];
      up_invalidate_dcache((uintptr_t)desc, (uintptr_t)desc + sizeof(*desc));
      flags = le32toh(desc->flags);
      if ((flags & NETC_TX_BD_WRITTEN) != 0 &&
          (flags & NETC_TX_BD_ERROR_MASK) != 0)
        {
          nerr("NETC transmit descriptor error: %08lx\n",
               (unsigned long)flags);
          NETDEV_TXERRORS(&lower->netdev);
        }

      DEBUGASSERT(priv->txpacket[priv->txclean] != NULL);
      netpkt_free(lower, priv->txpacket[priv->txclean], NETPKT_TX);
      priv->txpacket[priv->txclean] = NULL;
      priv->txclean = (priv->txclean + 1) % NETC_TX_SLOTS;
      netdev_lower_txdone(lower);
    }
}

static int imxrt_netc_transmit(struct netdev_lowerhalf_s *lower,
                              netpkt_t *packet)
{
  struct imxrt_netc_s *priv = (struct imxrt_netc_s *)lower;
  struct imxrt_netc_txdesc_s *desc;
  unsigned int length = netpkt_getdatalen(lower, packet);
  unsigned int slot = priv->txprod;
  unsigned int next = (slot + 1) % NETC_TX_SLOTS;
  int ret;

  if (priv->state != NETC_UP)
    {
      return -ENETDOWN;
    }

  if (length < ETH_HDRLEN || length > NETC_PACKET_SIZE)
    {
      return -EMSGSIZE;
    }

  imxrt_netc_reclaim(lower);
  if (priv->state != NETC_UP)
    {
      return -EIO;
    }

  if (next == priv->txclean)
    {
      nerr("NETC TX quota exceeded\n");
      return -EBUSY;
    }

  ret = netpkt_copyout(lower, priv->dma_cpu->txbuffer[slot], packet,
                       length, 0);
  if (ret != length)
    {
      nerr("NETC TX packet copy failed: %d\n", ret);
      return ret < 0 ? ret : -EIO;
    }

  up_clean_dcache((uintptr_t)priv->dma_cpu->txbuffer[slot],
                 (uintptr_t)priv->dma_cpu->txbuffer[slot] +
                 NETC_BUFFER_SIZE);
  desc = &priv->dma_cpu->tx[slot];
  memset(desc, 0, sizeof(*desc));
  desc->address = htole32(priv->dma_bus +
                          offsetof(struct imxrt_netc_dma_s, txbuffer) +
                          slot * NETC_BUFFER_SIZE);
  desc->length = htole16(length);
  desc->frame_length = htole16(length);
  desc->flags = htole32(NETC_TX_BD_EXTENDED | NETC_TX_BD_INTERRUPT);
  desc->extension = htole32(NETC_BD_FINAL);
  up_clean_dcache((uintptr_t)desc, (uintptr_t)desc + sizeof(*desc));
  priv->txpacket[slot] = packet;
  priv->txtime[slot] = clock_systime_ticks();
  priv->txprod = next;
  UP_DSB();
  putreg32(next * 2, TXREG(priv, NETC_BDR_PIR));
  return OK;
}

static netpkt_t *imxrt_netc_receive(struct netdev_lowerhalf_s *lower)
{
  struct imxrt_netc_s *priv = (struct imxrt_netc_s *)lower;
  struct imxrt_netc_rxdesc_s *desc;
  netpkt_t *packet;
  unsigned int budget = NETC_RX_SLOTS;
  unsigned int producer;
  unsigned int slot;
  unsigned int length;
  uint32_t flags;
  bool discard;
  int ret;

  while (priv->state == NETC_UP && budget-- != 0)
    {
      producer = getreg32(RXREG(priv, NETC_BDR_PIR)) & NETC_BDR_INDEX_MASK;
      if (producer >= NETC_RX_SLOTS ||
          (getreg32(RXREG(priv, NETC_BDR_SR)) & NETC_BDR_SBE) != 0)
        {
          nerr("NETC receive DMA error, producer %u\n", producer);
          NETDEV_RXERRORS(&lower->netdev);
          imxrt_netc_fault(priv);
          return NULL;
        }

      slot = priv->rxcons;
      if (producer == slot)
        {
          return NULL;
        }

      desc = &priv->dma_cpu->rx[slot];
      up_invalidate_dcache((uintptr_t)desc, (uintptr_t)desc + sizeof(*desc));
      UP_DMB();
      flags = le32toh(desc->flags);
      length = le32toh(desc->length) & UINT16_MAX;
      discard = priv->rxdiscard || (flags & NETC_BD_FINAL) == 0;
      priv->rxdiscard = (flags & NETC_BD_FINAL) == 0;
      packet = NULL;
      if (discard || (flags & NETC_RX_BD_ERROR_MASK) != 0 ||
          length < ETH_HDRLEN || length > NETC_PACKET_SIZE)
        {
          nerr("NETC RX descriptor error: flags=%08lx length=%u "
               "discard=%u\n", (unsigned long)flags, length, discard);
          NETDEV_RXERRORS(&lower->netdev);
        }
      else
        {
          up_invalidate_dcache((uintptr_t)priv->dma_cpu->rxbuffer[slot],
                               (uintptr_t)priv->dma_cpu->rxbuffer[slot] +
                               NETC_BUFFER_SIZE);
          packet = netpkt_alloc(lower, NETPKT_RX);
          if (packet != NULL)
            {
              ret = netpkt_copyin(lower, packet,
                                  priv->dma_cpu->rxbuffer[slot], length, 0);
              if (ret != length)
                {
                  nerr("NETC RX packet copy failed: %d\n", ret);
                  netpkt_free(lower, packet, NETPKT_RX);
                  packet = NULL;
                }
            }
        }

      imxrt_netc_rxarm(priv, slot);
      priv->rxcons = (slot + 1) % NETC_RX_SLOTS;
      UP_DSB();
      putreg32(priv->rxcons, RXREG(priv, NETC_RX_CIR));
      if (packet != NULL)
        {
          return packet;
        }

      NETDEV_RXDROPPED(&lower->netdev);
    }

  return NULL;
}

#ifdef CONFIG_NET_MCASTGROUP
static int imxrt_netc_multicast(struct netdev_lowerhalf_s *lower,
                               const uint8_t *mac)
{
  /* The bridge floods multicast to members and SI0 accepts all multicast,
   * including mandatory IPv6 discovery without optional group callbacks.
   */

  return (mac[0] & 1) != 0 ? OK : -EINVAL;
}
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int imxrt_netc_ioctl(struct netdev_lowerhalf_s *lower, int cmd,
                            unsigned long arg)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  struct imxrt_netc_s *priv = (struct imxrt_netc_s *)lower;
  struct mii_ioctl_data_s *req = (struct mii_ioctl_data_s *)arg;

  if (priv->hw->attachment == IMXRT_NETC_SWITCH)
    {
      return imxrt_netc_switch_ioctl(cmd, arg);
    }

  if (cmd != SIOCGMIIPHY && cmd != SIOCGMIIREG && cmd != SIOCSMIIREG)
    {
      return -ENOTTY;
    }

  if (req == NULL || req->phy_id > 31)
    {
      return -EINVAL;
    }

  if (priv->state == NETC_UNINITIALIZED || priv->link.phyaddr > 31)
    {
      return -ENODEV;
    }

  if (priv->state == NETC_FAILED)
    {
      return -EIO;
    }

  if (priv->state == NETC_RECOVERING)
    {
      return -EBUSY;
    }

  if (cmd == SIOCGMIIPHY)
    {
      req->phy_id = priv->link.phyaddr;
      return OK;
    }

  if (req->phy_id != priv->link.phyaddr || req->reg_num > 31)
    {
      return -EINVAL;
    }

  /* netdev ioctl holds the network lock; the MDIO bus serializes access.
   * Never access the physical MDIO window after stop disabled this PF.
   */

  if ((getreg16(priv->hw->function + NETC_PCI_CMD) &
       NETC_PCI_CMD_MEM_ACCESS) == 0)
    {
      return -ENETDOWN;
    }

  if (cmd == SIOCGMIIREG)
    {
      return mdio_read(priv->link.bus, priv->link.phyaddr, req->reg_num,
                       &req->val_out);
    }

  return mdio_write(priv->link.bus, priv->link.phyaddr, req->reg_num,
                    req->val_in);
#else
  return -ENOTTY;
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imxrt_netc_wait(uintptr_t reg, uint32_t mask, uint32_t value)
{
  unsigned int count;

  for (count = 0; count < 1000; count++)
    {
      if ((getreg32(reg) & mask) == value)
        {
          return OK;
        }

      up_udelay(10);
    }

  return -ETIMEDOUT;
}

int imxrt_netc_reset(uintptr_t function, uintptr_t common)
{
  unsigned int count;
  uint32_t status;
  int ret;

  modifyreg16(function + NETC_PCI_DEVCTL, 0, NETC_PCI_FLR);
  for (count = 0; count < 10000; count++)
    {
      if ((getreg16(function + NETC_PCI_DEVCTL) & NETC_PCI_FLR) == 0)
        {
          putreg16(NETC_PCI_CMD_ENABLE, function + NETC_PCI_CMD);
          ret = imxrt_netc_wait(common + NETC_COMMON_OSR,
                                NETC_COMMON_STATE, 0);
          if (ret == OK)
            {
              status = getreg32(common + NETC_COMMON_OSR);
              if ((status & NETC_COMMON_ITM_STATE) != 0)
                {
                  nerr("NETC reset at %08lx index table memory "
                       "overprovisioned: OSR=%08lx\n",
                       (unsigned long)common, (unsigned long)status);
                  ret = -EIO;
                }
            }

          return ret;
        }

      up_udelay(10);
    }

  nerr("NETC function reset timed out at %08lx\n", (unsigned long)function);
  return -ETIMEDOUT;
}

void *imxrt_netc_alloc(size_t size, uint32_t *dma_bus)
{
  void *memory;

  memory = kmm_memalign(NETC_DMA_ALIGN, size);
  if (memory == NULL)
    {
      return NULL;
    }

  /* Convert CPU pointer to DMA address space.
   * Keep the CPU pointer for memory access and cache maintenance.
   */

  if (!imxrt_dma_address(memory, size, dma_bus))
    {
      nerr("NETC allocation cannot be mapped for DMA: %p\n", memory);
      kmm_free(memory);
      return NULL;
    }

  memset(memory, 0, size);
  return memory;
}

static int imxrt_netc_register(size_t index,
                               const struct imxrt_netc_enetc_s *config,
                               uint32_t clock_hz)
{
  struct imxrt_netc_s *priv;
  uint32_t status;
  uint16_t command;
  int cleanup;
  int ret;

  priv = &g_netc[index];
  priv->hw = &g_netc_hw[index];
  priv->config = *config;
  priv->link.speed = 0;
  priv->link.phyaddr = UINT8_MAX;
  priv->msgintr = g_msgintr_base[config->msgintr - 1];
  priv->irq = g_msgintr_irq[config->msgintr - 1];
  priv->lower.ops = &g_netc_ops;
  priv->lower.priority = ETHWORK;
  priv->lower.quota[NETPKT_TX] = NETC_TX_QUOTA;
  priv->lower.quota[NETPKT_RX] = 1;
  memcpy(priv->lower.netdev.d_mac.ether.ether_addr_octet, config->mac, 6);
  priv->dma_cpu = imxrt_netc_alloc(sizeof(*priv->dma_cpu), &priv->dma_bus);
  if (priv->dma_cpu == NULL)
    {
      ret = -ENOMEM;
      goto failed;
    }

  command = getreg16(priv->hw->function + NETC_PCI_CMD);
  if ((command & NETC_PCI_CMD_ENABLE) != 0 ||
      (getreg16(priv->hw->function + NETC_PCI_SRIOVCTL) &
       NETC_PCI_VF_ENABLE) != 0)
    {
      ret = -EBUSY;
      goto free_dma;
    }

  ret = irq_attach(priv->irq, imxrt_netc_interrupt, priv);
  if (ret < 0)
    {
      goto free_dma;
    }

  /* No switch or packet callbacks can overlap initialization. */

  net_lock();
  if (priv->hw->attachment == IMXRT_NETC_SWITCH)
    {
      ret = imxrt_netc_switch_initialize(priv->hw, &priv->config, clock_hz);
      if (ret < 0)
        {
          goto release_hw;
        }
    }
  else
    {
      putreg16(command | NETC_PCI_CMD_ENABLE,
                priv->hw->function + NETC_PCI_CMD);
      ret = imxrt_netc_wait(priv->hw->enetc + NETC_COMMON_OSR,
                            NETC_COMMON_STATE, 0);
      if (ret < 0)
        {
          goto release_hw;
        }

      status = getreg32(priv->hw->enetc + NETC_COMMON_OSR);
      if ((status & NETC_COMMON_ITM_STATE) != 0)
        {
          nerr("NETC ENETC%u index table memory overprovisioned: "
               "OSR=%08lx\n",
               (unsigned int)index, (unsigned long)status);
          ret = -EIO;
          goto release_hw;
        }

      putreg32(0, priv->hw->si + NETC_SI_SIMR);
      putreg32(NETC_PORT_DISABLE, priv->hw->port + NETC_PORT_POR);
      modifyreg32(priv->hw->mac + NETC_MAC_CMD,
                  NETC_MAC_RX | NETC_MAC_TX, 0);

      ret = imxrt_netc_port_prepare(&priv->hw->ports[0],
                                    &priv->config.ports[0]);
      if (ret < 0)
        {
          goto release_hw;
        }

      ret = imxrt_netc_port_initialize(&priv->hw->ports[0],
                                       &priv->config.ports[0],
                                       &priv->link, clock_hz);
      if (ret < 0)
        {
          goto release_hw;
        }
    }

  priv->state = NETC_DOWN;
  ret = work_queue(ETHWORK, &priv->linkwork, imxrt_netc_linkwork, priv,
                   NETC_LINK_INTERVAL);
  if (ret < 0)
    {
      goto release_hw;
    }

  ret = netdev_lower_register(&priv->lower, NET_LL_ETHERNET);
  if (ret < 0)
    {
      priv->state = NETC_UNINITIALIZED;
      net_unlock();
      work_cancel_sync(ETHWORK, &priv->linkwork);
      net_lock();
      goto release_hw;
    }

  net_unlock();
  return OK;

release_hw:
  priv->state = NETC_UNINITIALIZED;
  if (priv->hw->attachment == IMXRT_NETC_SWITCH)
    {
      cleanup = imxrt_netc_switch_uninitialize();
      if (cleanup < 0)
        {
          nerr("NETC switch cleanup failed: %d\n", cleanup);
          priv->state = NETC_FAILED;
        }
    }
  else
    {
      imxrt_netc_port_uninitialize(&priv->hw->ports[0], &priv->link);
      putreg16(command, priv->hw->function + NETC_PCI_CMD);
      if (getreg16(priv->hw->function + NETC_PCI_CMD) != command)
        {
          nerr("NETC ENETC%u function disable failed\n",
               (unsigned int)index);
          priv->state = NETC_FAILED;
        }
    }

  net_unlock();
  irq_detach(priv->irq);
free_dma:
  /* No ENETC ring addresses are published before ifup. Retain the allocation
   * if hardware cleanup failed, so the failed instance cannot be reused.
   */

  if (priv->state != NETC_FAILED)
    {
      kmm_free(priv->dma_cpu);
      priv->dma_cpu = NULL;
    }

failed:
  nerr("NETC ENETC%u initialization failed: %d\n", (unsigned int)index, ret);
  return ret;
}

int imxrt_netc_initialize(const struct imxrt_netc_config_s *config)
{
  uint32_t clock_hz = 0;
  uint32_t status;
  size_t index;
  int ret;

  ret = nxmutex_lock(&g_initlock);
  if (ret < 0)
    {
      return ret;
    }

  status = getreg32(IMXRT_SRC_SLICE_FUNC_STAT(IMXRT_NETC_MIX_SLICE_BASE));
  if ((status & (SRC_SLICE_FUNC_STAT_PSW_STAT |
                 SRC_SLICE_FUNC_STAT_RST_STAT |
                 SRC_SLICE_FUNC_STAT_ISO_STAT)) !=
      SRC_SLICE_FUNC_STAT_RST_STAT)
    {
      nerr("NETCMIX must be powered, released and deisolated: %#lx\n",
           (unsigned long)status);
      ret = -EHOSTDOWN;
      goto out;
    }

  ret = imxrt_get_rootclock(CCM_CR_NETC, &clock_hz);
  if (ret < 0)
    {
      nerr("NETC root clock query failed: %d\n", ret);
      goto out;
    }

  if (clock_hz < NETC_IERB_CLKCR_MIN_HZ ||
      clock_hz > NETC_IERB_CLKCR_MAX_HZ ||
      clock_hz % 1000000 != 0)
    {
      nerr("NETC unsupported root clock frequency: %lu Hz\n",
           (unsigned long)clock_hz);
      ret = -EINVAL;
      goto out;
    }

  ret = imxrt_netc_shared_initialize(config, clock_hz);
  if (ret < 0)
    {
      nerr("NETC shared infrastructure initialization failed: %d\n", ret);
      goto out;
    }

  for (index = 0; index < nitems(config->enetc); index++)
    {
      if (!config->enetc[index].enabled)
        {
          continue;
        }

      ret = imxrt_netc_register(index, &config->enetc[index], clock_hz);
      if (ret < 0)
        {
          goto out;
        }
    }

out:
  nxmutex_unlock(&g_initlock);
  return ret;
}
