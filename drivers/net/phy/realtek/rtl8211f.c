/****************************************************************************
 * drivers/net/phy/realtek/rtl8211f.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <nuttx/net/realtek_phy.h>

#include "phy.h"
#include "rtl8211f.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rtl8211f_config_init(FAR struct phy_device_s *phydev);
static int rtl8211f_read_page(FAR struct phy_device_s *phydev);
static int rtl8211f_write_page(FAR struct phy_device_s *phydev, int page);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct phy_driver_s g_rtl8211f_drivers[] =
{
  {
    .name         = "Realtek RTL8211F-VD",
    .phy_id       = RTL8211F_VD_PHY_ID,
    .phy_id_mask  = RTL8211F_PHY_ID_MASK,
    .config_init  = rtl8211f_config_init,
    .config_aneg  = genphy_config_aneg,
    .read_status  = genphy_read_status,
    .config_power = genphy_config_power,
    .read_page    = rtl8211f_read_page,
    .write_page   = rtl8211f_write_page,
  },
  {
    .name         = "Realtek RTL8211F",
    .phy_id       = RTL8211F_PHY_ID,
    .phy_id_mask  = RTL8211F_PHY_ID_MASK,
    .config_init  = rtl8211f_config_init,
    .config_aneg  = genphy_config_aneg,
    .read_status  = genphy_read_status,
    .config_power = genphy_config_power,
    .read_page    = rtl8211f_read_page,
    .write_page   = rtl8211f_write_page,
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rtl8211f_read_page(FAR struct phy_device_s *phydev)
{
  uint16_t page;
  int ret;

  ret = mdio_read_nolock(phydev->bus, phydev->mdio_addr,
                         RTL8211F_PAGE_SELECT, &page);
  return ret < 0 ? ret : page;
}

static int rtl8211f_write_page(FAR struct phy_device_s *phydev, int page)
{
  return mdio_write_nolock(phydev->bus, phydev->mdio_addr,
                           RTL8211F_PAGE_SELECT, page);
}

static int rtl8211f_config_init(FAR struct phy_device_s *phydev)
{
  uint16_t txdelay = 0;
  uint16_t rxdelay = 0;
  int ret;

  switch (phydev->interface)
    {
      case PHY_INTERFACE_RGMII_ID:
        txdelay = RTL8211F_TX_DELAY;
        rxdelay = RTL8211F_RX_DELAY;
        break;

      case PHY_INTERFACE_RGMII_TXID:
        txdelay = RTL8211F_TX_DELAY;
        break;

      case PHY_INTERFACE_RGMII_RXID:
        rxdelay = RTL8211F_RX_DELAY;
        break;

      case PHY_INTERFACE_RGMII:
        break;

      default:
        return -EOPNOTSUPP;
    }

  ret = phy_modify_paged(phydev, RTL8211F_DELAY_PAGE,
                          RTL8211F_TX_DELAY_CTRL,
                          RTL8211F_TX_DELAY, txdelay);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_modify_paged(phydev, RTL8211F_DELAY_PAGE,
                          RTL8211F_RX_DELAY_CTRL,
                          RTL8211F_RX_DELAY, rxdelay);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write(phydev, RTL8211F_PAGE_SELECT, RTL8211F_DEFAULT_PAGE);

  return ret < 0 ? ret : genphy_config_init(phydev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rtl8211f_register(void)
{
  int ret;

  ret = phy_driver_register(&g_rtl8211f_drivers[0]);
  return ret < 0 ? ret : phy_driver_register(&g_rtl8211f_drivers[1]);
}
