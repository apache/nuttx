/****************************************************************************
 * arch/arm/src/imxrt/imxrt_netc_port.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <sys/param.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/net/gmii.h>
#include <nuttx/net/mdio.h>

#include "imxrt_netc_priv.h"
#include "hardware/imxrt_netc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_netc_mdio_s
{
  struct mdio_lowerhalf_s lower;
  struct mdio_bus_s *bus;
  enum imxrt_netc_mdio_e id;
  uint32_t config;
  unsigned int users;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int imxrt_netc_mdio_read(struct mdio_lowerhalf_s *lower,
                               uint8_t phyaddr, uint8_t reg,
                               uint16_t *value);
static int imxrt_netc_mdio_write(struct mdio_lowerhalf_s *lower,
                                uint8_t phyaddr, uint8_t reg,
                                uint16_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lock order: network lock, MDIO upper-half lock, then hardware lock.
 * The network lock also excludes other PHY users during paged sequences
 * and protects endpoint registration and reference counts.
 */

static mutex_t g_mdiolock = NXMUTEX_INITIALIZER;
static struct imxrt_netc_mdio_s g_mdio[IMXRT_NETC_MDIO_COUNT];

static const struct mdio_ops_s g_mdio_ops =
{
  .read = imxrt_netc_mdio_read,
  .write = imxrt_netc_mdio_write,
  .reset = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uintptr_t imxrt_netc_mdiobase(enum imxrt_netc_mdio_e bus)
{
  switch (bus)
    {
      case IMXRT_NETC_EMDIO:
        return IMXRT_NETC_EMDIO_REGS;

      case IMXRT_NETC_PORT0_MDIO:
        return IMXRT_NETC_PORT_MDIO(0);

      case IMXRT_NETC_PORT1_MDIO:
        return IMXRT_NETC_PORT_MDIO(1);

      case IMXRT_NETC_PORT2_MDIO:
        return IMXRT_NETC_PORT_MDIO(2);

      case IMXRT_NETC_PORT3_MDIO:
        return IMXRT_NETC_PORT_MDIO(3);

      case IMXRT_NETC_ENETC0_MDIO:
        return IMXRT_NETC_ENETC0_MDIO_REGS;

      default:
        return 0;
    }
}

static int imxrt_netc_mdio(struct imxrt_netc_mdio_s *priv,
                           uint8_t phyaddr, uint8_t reg, uint16_t *value,
                           bool read)
{
  uintptr_t base = imxrt_netc_mdiobase(priv->id);
  uint32_t status;
  int ret;

  DEBUGASSERT(phyaddr <= 31 && reg <= 31 && value != NULL);

  ret = nxmutex_lock(&g_mdiolock);
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_netc_wait(base + NETC_MDIO_CFG, NETC_MDIO_BUSY, 0);
  if (ret == OK)
    {
      /* Restore timing on every transaction, including after a PF reset. */

      putreg32(priv->config | NETC_MDIO_ADDR_ERROR | NETC_MDIO_COMPLETE,
               base + NETC_MDIO_CFG);
      putreg32(NETC_MDIO_PHY(phyaddr) | reg |
               (read ? NETC_MDIO_READ : 0), base + NETC_MDIO_CTL);
      if (!read)
        {
          putreg32(*value, base + NETC_MDIO_DATA);
        }

      ret = imxrt_netc_wait(base + NETC_MDIO_CFG, NETC_MDIO_BUSY, 0);
      if (ret == OK)
        {
          status = getreg32(base + NETC_MDIO_CFG);
          if ((status & NETC_MDIO_ADDR_ERROR) != 0)
            {
              ret = -EACCES;
            }
          else if (read && (status & NETC_MDIO_READ_ERROR) != 0)
            {
              ret = -ENODEV;
            }
          else if (read)
            {
              *value = getreg32(base + NETC_MDIO_DATA);
            }
        }
    }

  nxmutex_unlock(&g_mdiolock);
  if (ret < 0 && ret != -ENODEV)
    {
      nerr("MDIO bus %u PHY %u register %u: %d\n",
           priv->id, phyaddr, reg, ret);
    }

  return ret;
}

static int imxrt_netc_mdio_read(struct mdio_lowerhalf_s *lower,
                               uint8_t phyaddr, uint8_t reg,
                               uint16_t *value)
{
  struct imxrt_netc_mdio_s *priv =
    container_of(lower, struct imxrt_netc_mdio_s, lower);

  return imxrt_netc_mdio(priv, phyaddr, reg, value, true);
}

static int imxrt_netc_mdio_write(struct mdio_lowerhalf_s *lower,
                                uint8_t phyaddr, uint8_t reg,
                                uint16_t value)
{
  struct imxrt_netc_mdio_s *priv =
    container_of(lower, struct imxrt_netc_mdio_s, lower);

  return imxrt_netc_mdio(priv, phyaddr, reg, &value, false);
}

static int imxrt_netc_mdio_initialize(enum imxrt_netc_mdio_e id,
                                     uint32_t clock_hz,
                                     struct mdio_bus_s **bus)
{
  struct imxrt_netc_mdio_s *priv;
  uint32_t divisor;

  DEBUGASSERT((unsigned int)id < nitems(g_mdio));
  priv = &g_mdio[id];
  if (priv->users == 0)
    {
      /* MDC <= 2.5 MHz: NETC clock / (2 * divisor + 1). */

      divisor = (clock_hz + 4999999) / 5000000;
      if (divisor < 5)
        {
          divisor = 5;
        }

      DEBUGASSERT(divisor <= 511);
      priv->lower.ops = &g_mdio_ops;
      priv->id = id;
      priv->config = NETC_MDIO_NEG | NETC_MDIO_DIV(divisor) |
                     NETC_MDIO_HOLD(2);
      priv->bus = mdio_register(&priv->lower);
      if (priv->bus == NULL)
        {
          return -ENOMEM;
        }
    }

  priv->users++;
  *bus = priv->bus;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imxrt_netc_port_uninitialize(const struct imxrt_netc_ports_hw_s *hw,
                                 struct imxrt_netc_link_s *link)
{
  struct imxrt_netc_mdio_s *priv;

  if (link->bus == NULL)
    {
      return;
    }

  priv = &g_mdio[hw->mdio];
  DEBUGASSERT(priv->users > 0 && priv->bus == link->bus);

  link->bus = NULL;
  link->phyaddr = UINT8_MAX;
  link->speed = 0;
  if (--priv->users == 0)
    {
      mdio_unregister(priv->bus);
      priv->bus = NULL;
    }
}

int imxrt_netc_port_prepare(const struct imxrt_netc_ports_hw_s *hw,
                           const struct imxrt_netc_port_s *config)
{
  uint32_t mask = NETC_LINK_CFG_MII_PROT_MASK;
  uint32_t mode;
  uint32_t clock = NETC_PORT_MISC_RMII_CLK_OUT(hw->reg_idx);

  switch (config->phy.iftype)
    {
      case PHY_TYPE_MII:
        mode = NETC_LINK_CFG_MII;
        break;

      case PHY_TYPE_RMII:
        mode = NETC_LINK_CFG_RMII;
        break;

      case PHY_TYPE_RGMII:
        mode = NETC_LINK_CFG_RGMII;
        break;

      default:
        nerr("NETC port %u unsupported interface mode: %u\n",
             hw->reg_idx, config->phy.iftype);
        return -EOPNOTSUPP;
    }

  /* REVMII is reserved on ports 0 and 1. */

  if (hw->reg_idx >= 2)
    {
      mask |= NETC_LINK_CFG_REVMII;
    }

  modifyreg32(IMXRT_NETC_PORT_MISC_CFG, clock,
              config->phy.rmii_phy_clk_out ? 0 : clock);
  modifyreg32(IMXRT_NETC_LINK_CFG(hw->reg_idx), mask, mode);

#ifdef CONFIG_IMXRT_NETC_PHYINIT
  return board_netc_phyinit(hw->reg_idx, config);
#else
  return OK;
#endif
}

int imxrt_netc_port_initialize(const struct imxrt_netc_ports_hw_s *hw,
                              const struct imxrt_netc_port_s *config,
                              struct imxrt_netc_link_s *link,
                              uint32_t clock_hz)
{
  struct phy_desc_s phy = config->phy;
  unsigned int last = config->phy.address_high;
  unsigned int address;
  uint16_t id1;
  uint16_t id2;
  uint16_t advertise;
  uint16_t control;
  int ret;

  if (last == UINT16_MAX)
    {
      last = config->phy.address_lo;
    }

  DEBUGASSERT(phy.clause == 22);
  DEBUGASSERT(phy.address_lo <= 31 && last <= 31 &&
              last >= phy.address_lo);
  DEBUGASSERT(link->bus == NULL);

  ret = imxrt_netc_mdio_initialize(hw->mdio, clock_hz, &link->bus);
  if (ret < 0)
    {
      return ret;
    }

  for (address = config->phy.address_lo; address <= last; address++)
    {
      ret = mdio_read(link->bus, address, MII_PHYID1, &id1);
      if (ret == -ENODEV)
        {
          nerr("PHY %s MDIO bus %u address %u: no response to PHYID1\n",
               config->phy.name, hw->mdio, address);
          continue;
        }

      if (ret < 0)
        {
          return ret;
        }

      ret = mdio_read(link->bus, address, MII_PHYID2, &id2);
      if (ret < 0)
        {
          nerr("PHY %s MDIO bus %u address %u: PHYID2 read failed: %d\n",
               config->phy.name, hw->mdio, address, ret);
          return ret;
        }

      if (id1 == config->phy.id1 &&
          (id2 & ~MII_PHYID2_REV_MASK) ==
          (config->phy.id2 & ~MII_PHYID2_REV_MASK) &&
          id1 != UINT16_MAX && (id1 != 0 || id2 != 0))
        {
          break;
        }

      nerr("PHY %s MDIO bus %u address %u: ID %04x:%04x, "
           "expected %04x:%04x (revision ignored)\n",
           config->phy.name, hw->mdio, address,
           id1, id2, config->phy.id1, config->phy.id2);
    }

  if (address > last)
    {
      nerr("PHY %s not found on MDIO bus %u\n",
           config->phy.name, hw->mdio);
      return -ENODEV;
    }

  link->phyaddr = address;
  link->speed = 0;
  phy.address_lo = address;
  phy.address_high = address;
  if (phy.ops != NULL && phy.ops->phyinit != NULL)
    {
      ret = phy.ops->phyinit(&phy, link->bus);
      if (ret < 0)
        {
          nerr("PHY %s MDIO bus %u address %u initialization failed: %d\n",
               phy.name, hw->mdio, address, ret);
          return ret;
        }
    }

  ret = mdio_read(link->bus, address, MII_MCR, &control);
  if (ret < 0)
    {
      return ret;
    }

  if ((control & MII_MCR_RESET) != 0)
    {
      return -EBUSY;
    }

  advertise = MII_ADVERTISE_CSMA |
    MII_ADVERTISE_10BASETXFULL |
    MII_ADVERTISE_100BASETXFULL;

  ret = mdio_write(link->bus, address, MII_ADVERTISE, advertise);
  if (ret < 0)
    {
      return ret;
    }

  ret = mdio_read(link->bus, address, MII_MSR, &control);
  if (ret < 0)
    {
      return ret;
    }

  if ((control & MII_MSR_ANEGABLE) == 0)
    {
      return -EOPNOTSUPP;
    }

  if ((control & MII_MSR_ESTATEN) != 0)
    {
      ret = mdio_read(link->bus, address, MII_ESTATUS, &control);
      if (ret < 0)
        {
          return ret;
        }

      if ((control & MII_ESTATUS_1000BASETFULL) != 0 &&
          (config->phy.iftype == PHY_TYPE_GMII ||
           config->phy.iftype == PHY_TYPE_RGMII))
        {
          advertise = GMII_1000BTCR_1000BASETFULL;
          ret = mdio_write(link->bus, address, GMII_1000BTCR, advertise);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  return mdio_write(link->bus, address, MII_MCR,
                    MII_MCR_ANENABLE | MII_MCR_ANRESTART);
}

int imxrt_netc_port_link(const struct imxrt_netc_port_s *config,
                        struct imxrt_netc_link_s *link, uint16_t *speed)
{
  uint16_t status;
  uint16_t local;
  uint16_t partner;
  uint16_t mode;
  int ret;

  *speed = 0;

  /* BMSR link is latched low; always read twice. */

  ret = mdio_read(link->bus, link->phyaddr, MII_MSR, &status);
  if (ret == OK)
    {
      ret = mdio_read(link->bus, link->phyaddr, MII_MSR, &status);
    }

  if (ret < 0 || (status & (MII_MSR_LINKSTATUS | MII_MSR_ANEGCOMPLETE)) !=
                (MII_MSR_LINKSTATUS | MII_MSR_ANEGCOMPLETE))
    {
      return ret;
    }

  ret = mdio_read(link->bus, link->phyaddr, MII_MCR, &local);
  if (ret < 0)
    {
      return ret;
    }

  if ((local & MII_MCR_ANENABLE) == 0)
    {
      return -EPROTO;
    }

  if (config->phy.status != UINT16_MAX)
    {
      ret = mdio_read(link->bus, link->phyaddr, config->phy.status, &status);
      if (ret < 0)
        {
          return ret;
        }

      if ((status & config->phy.duplex) == 0)
        {
          return -EPROTO;
        }

      mode = status & config->phy.speed_mask;
      if (mode == config->phy.mbps1000)
        {
          *speed = 1000;
        }
      else if (mode == config->phy.mbps100)
        {
          *speed = 100;
        }
      else if (mode == config->phy.mbps10)
        {
          *speed = 10;
        }
      else
        {
          return -EPROTO;
        }

      return OK;
    }

  if (config->phy.iftype == PHY_TYPE_GMII ||
      config->phy.iftype == PHY_TYPE_RGMII)
    {
      ret = mdio_read(link->bus, link->phyaddr, GMII_1000BTCR, &local);
      if (ret == OK)
        {
          ret = mdio_read(link->bus, link->phyaddr, GMII_1000BTSR, &partner);
        }

      if (ret < 0)
        {
          return ret;
        }

      if ((partner & GMII_1000BTSR_MSFAULT) != 0)
        {
          return -EPROTO;
        }

      if ((local & GMII_1000BTCR_1000BASETFULL) != 0 &&
          (partner & GMII_1000BTSR_LP1000BASETFULL) != 0)
        {
          *speed = 1000;
          return OK;
        }
    }

  ret = mdio_read(link->bus, link->phyaddr, MII_ADVERTISE, &local);
  if (ret == OK)
    {
      ret = mdio_read(link->bus, link->phyaddr, MII_LPA, &partner);
    }

  if (ret < 0)
    {
      return ret;
    }

  mode = local & partner;
  if ((mode & MII_ADVERTISE_100BASETXFULL) != 0)
    {
      *speed = 100;
    }
  else if ((mode & MII_ADVERTISE_10BASETXFULL) != 0)
    {
      *speed = 10;
    }
  else
    {
      return -EPROTO;
    }

  return OK;
}

int imxrt_netc_port_mode(uintptr_t port, uintptr_t mac,
                        enum phy_iftype_e mode, uint16_t speed)
{
  uint32_t interface;
  int ret;

  switch (mode)
    {
      case PHY_TYPE_MII:
        interface = NETC_MAC_IF_MII;
        break;
      case PHY_TYPE_RMII:
        interface = NETC_MAC_IF_RMII;
        break;
      case PHY_TYPE_RGMII:
        interface = NETC_MAC_IF_RGMII;
        break;
      default:
        return -EINVAL;
    }

  modifyreg32(mac + NETC_MAC_CMD, NETC_MAC_RX, 0);
  ret = imxrt_netc_wait(mac + NETC_MAC_IEVENT, NETC_MAC_RXEMPTY,
                        NETC_MAC_RXEMPTY);
  if (ret == OK)
    {
      ret = imxrt_netc_wait(port + NETC_PORT_PSR, NETC_PORT_RXBUSY, 0);
    }

  if (ret == OK)
    {
      modifyreg32(port + NETC_PORT_POR, 0, NETC_PORT_DISABLE);
      ret = imxrt_netc_wait(mac + NETC_MAC_IEVENT, NETC_MAC_TXEMPTY,
                            NETC_MAC_TXEMPTY);
    }

  if (ret < 0)
    {
      modifyreg32(port + NETC_PORT_POR, 0, NETC_PORT_DISABLE);
      return ret;
    }

  /* Allow the final minimum frame to leave the MAC before disabling TX. */

  up_udelay(52);
  modifyreg32(mac + NETC_MAC_CMD, NETC_MAC_TX, 0);
  if (speed == 0)
    {
      return OK;
    }

  if (speed == 10)
    {
      interface |= NETC_MAC_IF_10M | NETC_MAC_IF_SSP(1);
    }
  else if (speed == 1000)
    {
      interface |= NETC_MAC_IF_SSP(2);
    }

  modifyreg32(mac + NETC_MAC_IFMODE, NETC_MAC_IF_MASK, interface);
  modifyreg32(port + NETC_PORT_PCR, NETC_PORT_PSPEED_MASK,
              NETC_PORT_PSPEED(speed - 1));
  putreg32(NETC_PACKET_SIZE + 4, mac + NETC_MAC_MAXFRM);
  putreg32(64, mac + NETC_MAC_MINFRM);
  modifyreg32(mac + NETC_MAC_CMD, 0,
              NETC_MAC_TXPAD | NETC_MAC_TX | NETC_MAC_RX);
  modifyreg32(port + NETC_PORT_POR, NETC_PORT_DISABLE, 0);
  return OK;
}

void imxrt_netc_port_sdu(uintptr_t port)
{
  unsigned int tc;

  putreg32(20, port + NETC_PORT_PRXSDUOR);
  putreg32(20, port + NETC_PORT_PTXSDUOR);
  for (tc = 0; tc < 8; tc++)
    {
      putreg32(NETC_PORT_MAXSDU_DISABLE, port + NETC_PORT_PTCTMSDUR(tc));
    }
}

void imxrt_netc_pseudo(uintptr_t port)
{
  modifyreg32(port + NETC_PORT_PCR, NETC_PORT_PSPEED_MASK,
              NETC_PORT_PSPEED(999));
  imxrt_netc_port_sdu(port);
}
