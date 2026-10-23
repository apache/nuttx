/****************************************************************************
 * drivers/net/phy/rtl8201f.c
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
#include <nuttx/net/phy/rtl8201f.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rtl8201f_phy_reset(FAR const struct phy_desc_s *desc,
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

  for (elapsed = 0; elapsed < RTL8201F_RESET_TIMEOUT_MS; elapsed++)
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

int rtl8201f_phy_init(FAR const struct phy_desc_s *desc,
                     FAR struct mdio_bus_s *bus)
{
  uint16_t value;
  int restore;
  int ret;

  ret = mdio_write(bus, desc->address_lo, MII_RTL8201F_PSR, 0);
  if (ret < 0)
    {
      goto out;
    }

  ret = rtl8201f_phy_reset(desc, bus);
  if (ret < 0)
    {
      goto out;
    }

  ret = mdio_write(bus, desc->address_lo, MII_RTL8201F_PSR, 7);
  if (ret == OK)
    {
      ret = mdio_read(bus, desc->address_lo, MII_RTL8201F_RMSR_P7, &value);
    }

  if (ret == OK)
    {
      /* Preserve oscillator configuration, reserved bits and RMII
       * timing.
       */

      value &= ~(MII_RTL8201F_RMII_SEL |
                  MII_RTL8201F_TXC_REF_CLK_DIR);

      if (desc->iftype == PHY_TYPE_RMII)
        {
          value |= MII_RTL8201F_RMII_SEL;
          if (!desc->rmii_phy_clk_out)
            {
              value |= MII_RTL8201F_TXC_REF_CLK_DIR;
            }
        }

      ret = mdio_write(bus, desc->address_lo, MII_RTL8201F_RMSR_P7, value);
    }

out:
  restore = mdio_write(bus, desc->address_lo, MII_RTL8201F_PSR, 0);
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

const struct phy_ops_s g_rtl8201f_phy_ops =
{
  .phyinit = rtl8201f_phy_init,
};
