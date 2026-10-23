/****************************************************************************
 * drivers/net/phy/rtl8211f.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/config.h>
#include <nuttx/net/phy/rtl8211f.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTL8211FVS_DELAY_PAGE    0x0d08

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rtl8211f_phy_reset(FAR const struct phy_desc_s *desc,
                      FAR struct mdio_bus_s *bus)
{
  uint16_t value;
  unsigned int elapsed;
  int ret;

  ret = mdio_write(bus, desc->address_lo, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      return ret;
    }

  for (elapsed = 0; elapsed < RTL8211F_RESET_TIMEOUT_MS; elapsed++)
    {
      ret = mdio_read(bus, desc->address_lo, MII_MCR, &value);
      if (ret < 0)
        {
          return ret;
        }

      if ((value & MII_MCR_RESET) == 0)
        {
          return OK;
        }

      usleep(1000);
    }

  return -ETIMEDOUT;
}

int rtl8211f_phy_init(FAR const struct phy_desc_s *desc,
                     FAR struct mdio_bus_s *bus)
{
  uint16_t value;
  int restore;
  int ret;

  ret = mdio_write(bus, desc->address_lo, GMII_RTL8211F_PAGSR, 0);
  if (ret < 0)
    {
      goto out;
    }

  ret = rtl8211f_phy_reset(desc, bus);
  if (ret < 0)
    {
      goto out;
    }

  /* If PHY model is RTL8211F-VD, then RX&TX delays shall be set
   * manually from registers, otherwise (RTL8211FDI) the delays
   * are selected by hardware straps.
   */

  ret = mdio_read(bus, desc->address_lo, MII_PHYID2, &value);
  if (ret < 0)
    {
      goto out;
    }

  if (value == GMII_PHYID2_RTL8211F)
    {
      ret = mdio_write(bus, desc->address_lo, GMII_RTL8211F_PAGSR,
                       RTL8211FVS_DELAY_PAGE);
      if (ret < 0)
        {
          goto out;
        }

      ret = mdio_read(bus, desc->address_lo, GMII_RTL8211F_MIICR1_D08,
                      &value);
      if (ret < 0)
        {
          goto out;
        }

      value |= GMII_RTL8211F_MIICR1_TX_DELAY;
      ret = mdio_write(bus, desc->address_lo, GMII_RTL8211F_MIICR1_D08,
                       value);
      if (ret < 0)
        {
          goto out;
        }

      ret = mdio_read(bus, desc->address_lo, GMII_RTL8211F_MIICR2_D08,
                      &value);
      if (ret < 0)
        {
          goto out;
        }

      value |= GMII_RTL8211F_MIICR2_RX_DELAY;
      ret = mdio_write(bus, desc->address_lo, GMII_RTL8211F_MIICR2_D08,
                       value);
    }

out:
  restore = mdio_write(bus, desc->address_lo, GMII_RTL8211F_PAGSR, 0);
  if (restore < 0)
    {
      nerr("PHY %s address %u page restore failed: %d\n",
           desc->name, desc->address_lo, restore);
    }

  if (ret < 0)
    {
      nerr("PHY %s address %u initialization failed: %d\n",
           desc->name, desc->address_lo, ret);
      return ret;
    }

  return restore;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct phy_ops_s g_rtl8211f_phy_ops =
{
  .phyinit = rtl8211f_phy_init,
};
