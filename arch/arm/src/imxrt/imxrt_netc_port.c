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
#include <nuttx/ethtool.h>

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
#ifdef CONFIG_IMXRT_NETC_PHYRESET
static int imxrt_netc_mdio_reset(struct mdio_lowerhalf_s *lower,
                                uint8_t phyaddr, int value);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lock order: network/device lock, PHY lock, MDIO lock, hardware lock.
 * Endpoint registration and reference counts use the network lock.
 * PHYLIB serializes paged accesses with the MDIO bus lock.
 */

static mutex_t g_mdiolock = NXMUTEX_INITIALIZER;
static struct imxrt_netc_mdio_s g_mdio[IMXRT_NETC_MDIO_COUNT];

static const struct mdio_ops_s g_mdio_ops =
{
  .read = imxrt_netc_mdio_read,
  .write = imxrt_netc_mdio_write,
#ifdef CONFIG_IMXRT_NETC_PHYRESET
  .reset = imxrt_netc_mdio_reset
#else
  .reset = NULL
#endif
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

#ifdef CONFIG_IMXRT_NETC_PHYRESET
static int imxrt_netc_mdio_reset(struct mdio_lowerhalf_s *lower,
                                uint8_t phyaddr, int value)
{
  struct imxrt_netc_mdio_s *priv =
    container_of(lower, struct imxrt_netc_mdio_s, lower);
  int ret;

  ret = board_netc_phyreset(priv->id, phyaddr, value);
  if (ret < 0)
    {
      nerr("MDIO bus %u PHY %u reset level %d failed: %d\n",
           priv->id, phyaddr, value, ret);
    }

  return ret;
}
#endif

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

  if (link->phydev != NULL)
    {
      phy_disconnect(link->phydev);
      link->phydev = NULL;
    }

  link->bus = NULL;
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

  switch (config->interface)
    {
      case PHY_INTERFACE_MII:
        mode = NETC_LINK_CFG_MII;
        break;

      case PHY_INTERFACE_RMII:
      case PHY_INTERFACE_RMII_PHY_CLK_OUT:
        mode = NETC_LINK_CFG_RMII;
        break;

      case PHY_INTERFACE_RGMII:
      case PHY_INTERFACE_RGMII_ID:
      case PHY_INTERFACE_RGMII_TXID:
      case PHY_INTERFACE_RGMII_RXID:
        mode = NETC_LINK_CFG_RGMII;
        break;

      default:
        nerr("NETC port %u unsupported interface mode: %u\n",
             hw->reg_idx, config->interface);
        return -EOPNOTSUPP;
    }

  /* REVMII is reserved on ports 0 and 1. */

  if (hw->reg_idx >= 2)
    {
      mask |= NETC_LINK_CFG_REVMII;
    }

  modifyreg32(IMXRT_NETC_PORT_MISC_CFG, clock,
              config->interface == PHY_INTERFACE_RMII_PHY_CLK_OUT ?
              0 : clock);
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
  struct phy_device_s *phydev;
  uint32_t supported;
  int ret;

  if (config->phyaddr > 31)
    {
      nerr("NETC port %u invalid PHY address: %u\n",
           hw->reg_idx, config->phyaddr);
      return -EINVAL;
    }

  DEBUGASSERT(link->bus == NULL);

  ret = imxrt_netc_mdio_initialize(hw->mdio, clock_hz, &link->bus);
  if (ret < 0)
    {
      return ret;
    }

  phydev = phy_device_create(link->bus, config->phyaddr);
  if (phydev == NULL)
    {
      nerr("NETC port %u PHY creation failed on bus %u address %u\n",
           hw->reg_idx, hw->mdio, config->phyaddr);
      ret = -ENODEV;
      goto failed;
    }

  phydev->interface = config->interface;
  ret = phy_connect(phydev, NULL);
  if (ret < 0)
    {
      phy_device_free(phydev);
      goto failed;
    }

  link->phydev = phydev;

  /* NETC supports full duplex only. Limit both capabilities and advertising
   * so ethtool cannot request a mode that the MAC cannot implement.
   */

  supported = SUPPORTED_10BASET_FULL | SUPPORTED_100BASET_FULL |
              SUPPORTED_AUTONEG | SUPPORTED_TP | SUPPORTED_MII;
  if (config->interface == PHY_INTERFACE_RGMII ||
      config->interface == PHY_INTERFACE_RGMII_ID ||
      config->interface == PHY_INTERFACE_RGMII_TXID ||
      config->interface == PHY_INTERFACE_RGMII_RXID)
    {
      supported |= SUPPORTED_1000BASET_FULL;
    }

  phydev->supported &= supported;
  phydev->advertising = phydev->supported;
  link->speed = 0;

  if (hw->reg_idx < IMXRT_NETC_SW0_NPORTS)
    {
      ret = phy_start(phydev);
      if (ret < 0)
        {
          goto failed;
        }
    }

  return OK;

failed:
  nerr("NETC port %u PHY initialization failed: %d\n", hw->reg_idx, ret);
  imxrt_netc_port_uninitialize(hw, link);
  return ret;
}

int imxrt_netc_port_link(struct imxrt_netc_link_s *link, uint16_t *speed)
{
  struct phy_device_s *phydev = link->phydev;
  int ret;

  *speed = 0;
  if (phydev == NULL)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&phydev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (phydev->state == PHY_ERROR)
    {
      ret = -EIO;
    }
  else if (phydev->state == PHY_RUNNING && phydev->link)
    {
      if (phydev->duplex != DUPLEX_FULL ||
          (phydev->speed != 10 && phydev->speed != 100 &&
           phydev->speed != 1000))
        {
          ret = -EPROTO;
        }
      else
        {
          *speed = phydev->speed;
        }
    }

  nxmutex_unlock(&phydev->lock);

  if (ret == -EIO)
    {
      /* PHYLIB halts polling after an MDIO error. Retry on the next NETC
       * update rather than leaving a transient bus failure unrecoverable.
       */

      nerr("NETC PHY %u restarting after an MDIO error\n",
           phydev->mdio_addr);
      phy_stop(phydev);
      ret = phy_start(phydev);
      return ret < 0 ? ret : -EIO;
    }

  return ret;
}

int imxrt_netc_port_mode(uintptr_t port, uintptr_t mac,
                        enum phy_interface_e mode, uint16_t speed)
{
  uint32_t interface;
  int ret;

  switch (mode)
    {
      case PHY_INTERFACE_MII:
        interface = NETC_MAC_IF_MII;
        break;
      case PHY_INTERFACE_RMII:
      case PHY_INTERFACE_RMII_PHY_CLK_OUT:
        interface = NETC_MAC_IF_RMII;
        break;
      case PHY_INTERFACE_RGMII:
      case PHY_INTERFACE_RGMII_ID:
      case PHY_INTERFACE_RGMII_TXID:
      case PHY_INTERFACE_RGMII_RXID:
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
