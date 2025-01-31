/****************************************************************************
 * boards/arm64/zynq-mpsoc/zcu111/src/zcu111_ethernet.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
/* Force verbose debug on in this file only to support unit-level testing. */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  undef  CONFIG_DEBUG_INFO
#  define CONFIG_DEBUG_INFO 1
#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>

#include "zynq_enet.h"

#include <arch/board/board.h>
#include "zcu111.h"

#if defined(CONFIG_ZYNQ_ENET) && defined(CONFIG_ZYNQ_ENET_PHYINIT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PHY CTRL bits */

#define DP83867_PHYCR_FIFO_DEPTH_3_B_NIB    0x00
#define DP83867_PHYCR_FIFO_DEPTH_4_B_NIB    0x01
#define DP83867_PHYCR_FIFO_DEPTH_6_B_NIB    0x02
#define DP83867_PHYCR_FIFO_DEPTH_8_B_NIB    0x03

/* RGMIIDCTL internal delay for rx and tx */

#define DP83867_RGMIIDCTL_250_PS    0x0
#define DP83867_RGMIIDCTL_500_PS    0x1
#define DP83867_RGMIIDCTL_750_PS    0x2
#define DP83867_RGMIIDCTL_1_NS      0x3
#define DP83867_RGMIIDCTL_1_25_NS   0x4
#define DP83867_RGMIIDCTL_1_50_NS   0x5
#define DP83867_RGMIIDCTL_1_75_NS   0x6
#define DP83867_RGMIIDCTL_2_00_NS   0x7
#define DP83867_RGMIIDCTL_2_25_NS   0x8
#define DP83867_RGMIIDCTL_2_50_NS   0x9
#define DP83867_RGMIIDCTL_2_75_NS   0xa
#define DP83867_RGMIIDCTL_3_00_NS   0xb
#define DP83867_RGMIIDCTL_3_25_NS   0xc
#define DP83867_RGMIIDCTL_3_50_NS   0xd
#define DP83867_RGMIIDCTL_3_75_NS   0xe
#define DP83867_RGMIIDCTL_4_00_NS   0xf

/* TI DP83867 */

#define DP83867_DEVADDR     0x1f

#define MII_DP83867_PHYCTRL 0x10
#define MII_DP83867_MICR    0x12
#define MII_DP83867_CFG2    0x14
#define MII_DP83867_BISCR   0x16
#define DP83867_CTRL        0x1f

/* Extended Registers */

#define DP83867_RGMIICTL   0x0032
#define DP83867_RGMIIDCTL  0x0086

#define DP83867_SW_RESET   BIT(15)
#define DP83867_SW_RESTART BIT(14)

/* MICR Interrupt bits */

#define MII_DP83867_MICR_AN_ERR_INT_EN          BIT(15)
#define MII_DP83867_MICR_SPEED_CHNG_INT_EN      BIT(14)
#define MII_DP83867_MICR_DUP_MODE_CHNG_INT_EN   BIT(13)
#define MII_DP83867_MICR_PAGE_RXD_INT_EN        BIT(12)
#define MII_DP83867_MICR_AUTONEG_COMP_INT_EN    BIT(11)
#define MII_DP83867_MICR_LINK_STS_CHNG_INT_EN   BIT(10)
#define MII_DP83867_MICR_FALSE_CARRIER_INT_EN   BIT(8)
#define MII_DP83867_MICR_SLEEP_MODE_CHNG_INT_EN BIT(4)
#define MII_DP83867_MICR_WOL_INT_EN             BIT(3)
#define MII_DP83867_MICR_XGMII_ERR_INT_EN       BIT(2)
#define MII_DP83867_MICR_POL_CHNG_INT_EN        BIT(1)
#define MII_DP83867_MICR_JABBER_INT_EN          BIT(0)

/* RGMIICTL bits */

#define DP83867_RGMII_TX_CLK_DELAY_EN    BIT(1)
#define DP83867_RGMII_RX_CLK_DELAY_EN    BIT(0)

/* PHY CTRL bits */

#define DP83867_PHYCR_FIFO_DEPTH_SHIFT 14
#define DP83867_MDI_CROSSOVER          5
#define DP83867_MDI_CROSSOVER_AUTO     2
#define DP83867_MDI_CROSSOVER_MDIX     2
#define DP83867_PHYCTRL_SGMIIEN        0x0800
#define DP83867_PHYCTRL_RXFIFO_SHIFT   12
#define DP83867_PHYCTRL_TXFIFO_SHIFT   14

/* RGMIIDCTL bits */

#define DP83867_RGMII_TX_CLK_DELAY_SHIFT  4

/* CFG2 bits */

#define MII_DP83867_CFG2_SPEEDOPT_10EN   0x0040
#define MII_DP83867_CFG2_SGMII_AUTONEGEN 0x0080
#define MII_DP83867_CFG2_SPEEDOPT_ENH    0x0100
#define MII_DP83867_CFG2_SPEEDOPT_CNT    0x0800
#define MII_DP83867_CFG2_SPEEDOPT_INTLOW 0x2000
#define MII_DP83867_CFG2_MASK            0x003F

#define MII_MMD_CTRL  0x0d /* MMD Access Control Register */
#define MII_MMD_DATA  0x0e /* MMD Access Data Register */

/* MMD Access Control register fields */

#define MII_MMD_CTRL_DEVAD_MASK 0x1f   /* Mask MMD DEVAD*/
#define MII_MMD_CTRL_ADDR       0x0000 /* Address */
#define MII_MMD_CTRL_NOINCR     0x4000 /* no post increment */
#define MII_MMD_CTRL_INCR_RDWT  0x8000 /* post increment on reads & writes */
#define MII_MMD_CTRL_INCR_ON_WT 0xC000 /* post increment on writes only */

/* User setting - can be taken from DTS */

#define DEFAULT_RX_ID_DELAY     DP83867_RGMIIDCTL_2_25_NS
#define DEFAULT_TX_ID_DELAY     DP83867_RGMIIDCTL_2_75_NS
#define DEFAULT_FIFO_DEPTH      DP83867_PHYCR_FIFO_DEPTH_4_B_NIB

/* Debug
 * Extra, in-depth debug output that is only available if
 * CONFIG_NETDEV_PHY_DEBUG us defined.
 */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  define phyerr    _err
#  define phywarn   _warn
#  define phyinfo   _info
#else
#  define phyerr(x...)
#  define phywarn(x...)
#  define phyinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dp83867_private
{
  int rx_id_delay;
  int tx_id_delay;
  int fifo_depth;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline bool phy_interface_is_rgmii(struct phy_s *phy)
{
  return ((phy->interface >= PHY_INTERFACE_MODE_RGMII) &&
          (phy->interface <= PHY_INTERFACE_MODE_RGMII_TXID));
}

static inline bool phy_interface_is_sgmii(struct phy_s *phy)
{
  return ((phy->interface >= PHY_INTERFACE_MODE_SGMII) &&
          (phy->interface <= PHY_INTERFACE_MODE_QSGMII));
}

/****************************************************************************
 * zynq_phyread_mmd_indirect - reads data from the MMD registers
 * @gmac: The PHY device bus
 * @prtad: MMD Address
 * @devad: MMD DEVAD
 * @addr: PHY address on the MII bus
 *
 * Description: it reads data from the MMD registers (clause 22 to access to
 * clause 45) of the specified phy address.
 * To read these registers we have:
 * 1) Write reg 13 // DEVAD
 * 2) Write reg 14 // MMD Address
 * 3) Write reg 13 // MMD Data Command for MMD DEVAD
 * 3) Read  reg 14 // Read MMD data
 ****************************************************************************/

uint16_t zynq_phyread_mmd_indirect(struct zynq_gmac_s *gmac, int prtad,
                                   int devad, int addr)
{
  uint16_t value = -1;

  /* Write the desired MMD Devad */

  zynq_phywrite(gmac, addr, MII_MMD_CTRL, devad);

  /* Write the desired MMD register address */

  zynq_phywrite(gmac, addr, MII_MMD_DATA, prtad);

  /* Select the Function : DATA with no post increment */

  zynq_phywrite(gmac, addr, MII_MMD_CTRL, (devad | MII_MMD_CTRL_NOINCR));

  /* Read the content of the MMD's selected register */

  zynq_phyread(gmac, addr, MII_MMD_DATA, &value);
  return value;
}

/****************************************************************************
 * zynq_phywrite_mmd_indirect - writes data to the MMD registers
 * @gmac: The PHY device
 * @prtad: MMD Address
 * @devad: MMD DEVAD
 * @addr: PHY address on the MII bus
 * @data: data to write in the MMD register
 *
 * Description: Write data from the MMD registers of the specified
 * phy address.
 * To write these registers we have:
 * 1) Write reg 13 // DEVAD
 * 2) Write reg 14 // MMD Address
 * 3) Write reg 13 // MMD Data Command for MMD DEVAD
 * 3) Write reg 14 // Write MMD data
 ****************************************************************************/

void zynq_phywrite_mmd_indirect(struct zynq_gmac_s *gmac, int prtad,
          int devad, int addr, uint16_t data)
{
  /* Write the desired MMD Devad */

  zynq_phywrite(gmac, addr, MII_MMD_CTRL, devad);

  /* Write the desired MMD register address */

  zynq_phywrite(gmac, addr, MII_MMD_DATA, prtad);

  /* Select the Function : DATA with no post increment */

  zynq_phywrite(gmac, addr, MII_MMD_CTRL, (devad | MII_MMD_CTRL_NOINCR));

  /* Write the data into MMD's selected register */

  zynq_phywrite(gmac, addr, MII_MMD_DATA, data);
}

static int dp83867_of_init(struct phy_s *phy)
{
  struct dp83867_private *dp83867 = (struct dp83867_private *)phy->priv;

  dp83867->rx_id_delay = DEFAULT_RX_ID_DELAY;
  dp83867->tx_id_delay = DEFAULT_TX_ID_DELAY;
  dp83867->fifo_depth = DEFAULT_FIFO_DEPTH;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: dp83867_config
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can
 *   be used.  This may include such things as configuring GPIOs, resetting
 *   the PHY, etc.  If CONFIG_ZYNQ_ENET_PHYINIT is defined in the
 *   configuration then the board specific logic must provide
 *   zynq_phyinitialize();  The ZYNQEthernet driver will call this
 *   function one time before it first uses the PHY.
 *
 * Input Parameters:
 *   gmac - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int dp83867_config(struct zynq_gmac_s *gmac)
{
  struct phy_s *phy = &gmac->phy;
  struct dp83867_private *dp83867 = (struct dp83867_private *)phy->priv;
  uint16_t val;
  uint16_t cfg2;
  unsigned int delay;
  int ret;

  /* Restart the PHY.  */

  zynq_phyread(gmac, phy->phyaddr, DP83867_CTRL, &val);
  zynq_phywrite(gmac, phy->phyaddr, DP83867_CTRL,
                val | DP83867_SW_RESTART);

  if (phy_interface_is_rgmii(phy))
    {
      ret = zynq_phywrite(gmac, phy->phyaddr, MII_DP83867_PHYCTRL,
                          (DP83867_MDI_CROSSOVER_AUTO <<
                           DP83867_MDI_CROSSOVER) |
                          (dp83867->fifo_depth <<
                           DP83867_PHYCR_FIFO_DEPTH_SHIFT));
      if (ret)
        {
          return ret;
        }
    }
  else if (phy_interface_is_sgmii(phy))
    {
      zynq_phywrite(gmac, phy->phyaddr, GMII_MCR,
                    (GMII_MCR_ANENABLE |
                     GMII_MCR_FULLDPLX |
                     GMII_MCR_SPEED1000));

      zynq_phyread(gmac, phy->phyaddr, MII_DP83867_CFG2, &cfg2);
      cfg2 &= MII_DP83867_CFG2_MASK;
      cfg2 |= (MII_DP83867_CFG2_SPEEDOPT_10EN |
               MII_DP83867_CFG2_SGMII_AUTONEGEN |
               MII_DP83867_CFG2_SPEEDOPT_ENH |
               MII_DP83867_CFG2_SPEEDOPT_CNT |
               MII_DP83867_CFG2_SPEEDOPT_INTLOW);

      zynq_phywrite(gmac, phy->phyaddr, MII_DP83867_CFG2, cfg2);

      zynq_phywrite_mmd_indirect(gmac, DP83867_RGMIICTL,
                 DP83867_DEVADDR, phy->phyaddr, 0x0);

      zynq_phywrite(gmac, phy->phyaddr, MII_DP83867_PHYCTRL,
                    DP83867_PHYCTRL_SGMIIEN |
                    (DP83867_MDI_CROSSOVER_MDIX << DP83867_MDI_CROSSOVER) |
                    (dp83867->fifo_depth << DP83867_PHYCTRL_RXFIFO_SHIFT) |
                    (dp83867->fifo_depth << DP83867_PHYCTRL_TXFIFO_SHIFT));
      zynq_phywrite(gmac, phy->phyaddr, MII_DP83867_BISCR, 0x0);
    }

  if ((phy->interface >= PHY_INTERFACE_MODE_RGMII_ID) &&
      (phy->interface <= PHY_INTERFACE_MODE_RGMII_RXID))
    {
      val = zynq_phyread_mmd_indirect(gmac, DP83867_RGMIICTL,
                                      DP83867_DEVADDR, phy->phyaddr);

      if (phy->interface == PHY_INTERFACE_MODE_RGMII_ID)
        {
          val |= (DP83867_RGMII_TX_CLK_DELAY_EN |
                  DP83867_RGMII_RX_CLK_DELAY_EN);
        }

      if (phy->interface == PHY_INTERFACE_MODE_RGMII_TXID)
        {
          val |= DP83867_RGMII_TX_CLK_DELAY_EN;
        }

      if (phy->interface == PHY_INTERFACE_MODE_RGMII_RXID)
        {
          val |= DP83867_RGMII_RX_CLK_DELAY_EN;
        }

      zynq_phywrite_mmd_indirect(gmac, DP83867_RGMIICTL,
                                 DP83867_DEVADDR, phy->phyaddr, val);

      delay = (dp83867->rx_id_delay |
               (dp83867->tx_id_delay << DP83867_RGMII_TX_CLK_DELAY_SHIFT));

      zynq_phywrite_mmd_indirect(gmac, DP83867_RGMIIDCTL,
                                 DP83867_DEVADDR, phy->phyaddr, delay);
    }

  return OK;
}

/****************************************************************************
 * Function: zynq_phy_boardinitialize
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can
 *   be used.  This may include such things as configuring GPIOs, resetting
 *   the PHY, etc.  If CONFIG_ZYNQ_ENET_PHYINIT is defined in the
 *   configuration then the board specific logic must provide
 *   zynq_phyinitialize();  The ZYNQEthernet driver will call this
 *   function one time before it first uses the PHY.
 *
 * Input Parameters:
 *   gmac - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int zynq_phy_boardinitialize(struct zynq_gmac_s *gmac)
{
  struct phy_s *phy = &gmac->phy;
  struct dp83867_private *dp83867;
  int ret;

  phy->uid = 0x2000a231;
  phy->interface = PHY_INTERFACE_MODE_RGMII_ID;

  if (!phy->priv)
    {
      dp83867 = kmm_malloc(sizeof(struct dp83867_private));
      if (!dp83867)
        {
          return -ENOMEM;
        }

      phy->priv = dp83867;
    }
  else
    {
      dp83867 = (struct dp83867_private *)phy->priv;
    }

  phy->config = dp83867_config;

  ret = dp83867_of_init(phy);

  return ret;
}

#endif /* CONFIG_ZYNQ_ENET && CONFIG_ZYNQ_ENET_PHYINIT */
