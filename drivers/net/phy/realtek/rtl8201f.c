/****************************************************************************
 * drivers/net/phy/realtek/rtl8201f.c
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
#include "rtl8201f.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rtl8201f_config_init(FAR struct phy_device_s *phydev);
static int rtl8201f_read_page(FAR struct phy_device_s *phydev);
static int rtl8201f_write_page(FAR struct phy_device_s *phydev, int page);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct phy_driver_s g_rtl8201f_driver =
{
  .name         = "Realtek RTL8201F",
  .phy_id       = RTL8201F_PHY_ID,
  .phy_id_mask  = RTL8201F_PHY_ID_MASK,
  .config_init  = rtl8201f_config_init,
  .config_aneg  = genphy_config_aneg,
  .read_status  = genphy_read_status,
  .config_power = genphy_config_power,
  .read_page    = rtl8201f_read_page,
  .write_page   = rtl8201f_write_page,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rtl8201f_read_page(FAR struct phy_device_s *phydev)
{
  uint16_t page;
  int ret;

  ret = mdio_read_nolock(phydev->bus, phydev->mdio_addr,
                         RTL8201F_PAGE_SELECT, &page);
  return ret < 0 ? ret : page;
}

static int rtl8201f_write_page(FAR struct phy_device_s *phydev, int page)
{
  return mdio_write_nolock(phydev->bus, phydev->mdio_addr,
                           RTL8201F_PAGE_SELECT, page);
}

static int rtl8201f_config_init(FAR struct phy_device_s *phydev)
{
  uint16_t mode = 0;
  int ret;

  switch (phydev->interface)
    {
      case PHY_INTERFACE_MII:
        break;

      case PHY_INTERFACE_RMII:
        mode = RTL8201F_RMII_SEL | RTL8201F_REF_CLK_IN;
        break;

      case PHY_INTERFACE_RMII_PHY_CLK_OUT:
        mode = RTL8201F_RMII_SEL;
        break;

      default:
        return -EOPNOTSUPP;
    }

  ret = phy_write(phydev, RTL8201F_PAGE_SELECT, RTL8201F_DEFAULT_PAGE);
  if (ret < 0)
    {
      return ret;
    }

  /* Preserve oscillator configuration, reserved bits and RMII timing. */

  ret = phy_modify_paged(phydev, RTL8201F_RMII_PAGE, RTL8201F_RMII_MODE,
                          RTL8201F_RMII_SEL | RTL8201F_REF_CLK_IN, mode);
  return ret < 0 ? ret : genphy_config_init(phydev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rtl8201f_register(void)
{
  return phy_driver_register(&g_rtl8201f_driver);
}
