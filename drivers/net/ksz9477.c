/****************************************************************************
 * drivers/net/ksz9477.c
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
#include <debug.h>
#include <errno.h>
#include <nuttx/net/ksz9477.h>
#include "ksz9477_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NET_KSZ9477_PORT_VLAN

static uint8_t g_port_vlan_config[] =
{
  CONFIG_NET_KSZ9477_PORT_VLAN_PHY1,
  CONFIG_NET_KSZ9477_PORT_VLAN_PHY2,
  CONFIG_NET_KSZ9477_PORT_VLAN_PHY3,
  CONFIG_NET_KSZ9477_PORT_VLAN_PHY4,
  CONFIG_NET_KSZ9477_PORT_VLAN_PHY5,
  CONFIG_NET_KSZ9477_PORT_VLAN_RMII,
  CONFIG_NET_KSZ9477_PORT_VLAN_SGMII
};

static uint8_t g_port_mirror_config[7] =
{
  0
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t bswap16(uint16_t data)
{
  return (data << 8) | (data >> 8);
}

static uint32_t bswap32(uint32_t data)
{
  return (data << 24) | (data >> 24) | ((data << 8) & 0xff0000) |
    ((data >> 8) & 0xff00);
}

#if 0 /* Enable when needed */
static int ksz9477_reg_read8(uint16_t reg, uint8_t *data)
{
  int ret;
  struct ksz9477_transfer_s read_msg;
  read_msg.len = 3;
  read_msg.reg = bswap16(reg);
  ret = ksz9477_read(&read_msg);
  *data = read_msg.data;
  return ret;
}
#endif

static int ksz9477_reg_read16(uint16_t reg, uint16_t *data)
{
  int ret;
  struct ksz9477_transfer_s read_msg;
  read_msg.len = 4;
  read_msg.reg = bswap16(reg);
  ret = ksz9477_read(&read_msg);
  *data = bswap16(read_msg.data);
  return ret;
}

static int ksz9477_reg_read32(uint16_t reg, uint32_t *data)
{
  int ret;
  struct ksz9477_transfer_s read_msg;
  read_msg.len = 6;
  read_msg.reg = bswap16(reg);
  ret = ksz9477_read(&read_msg);
  *data = bswap32(read_msg.data);
  return ret;
}

static int ksz9477_reg_write8(uint16_t reg, uint8_t data)
{
  struct ksz9477_transfer_s write_msg;
  write_msg.len = 3;
  write_msg.reg = bswap16(reg);
  write_msg.data = data;
  return ksz9477_write(&write_msg);
}

static int ksz9477_reg_write32(uint16_t reg, uint32_t data)
{
  struct ksz9477_transfer_s write_msg;
  write_msg.len = 6;
  write_msg.reg = bswap16(reg);
  write_msg.data = bswap32(data);
  return ksz9477_write(&write_msg);
}

#if 0  /* Enable when needed */
static int ksz9477_reg_write16(uint16_t reg, uint16_t data)
{
  int ret;
  struct ksz9477_transfer_s write_msg;
  uint16_t addr = reg;
  uint32_t data32;

  /* Errata: 16-bit writes to registers 0xN120-0xN13f will corrupt the
   * adjacent regsters. Workaround: perform only 32-bit writes to this
   * area
   */

  if ((reg & 0xfff) >= 0x120 && (reg & 0xfff) <= 0x13f)
    {
      /* Align write on lower 16-byte boundary */

      addr = reg & (~1);

      /* Read the full 32-bit register */

      ret = ksz9477_reg_read32(addr, &data32);
      if (ret != OK)
        {
          return ret;
        }

      /* Inject the data to the 32-bit write data */

      if (reg & 1)
        {
          data32 = (data32 & 0xff0000ff) | ((uint32_t)data << 8);
        }
      else
        {
          data32 = (data32 & 0xffff0000) | data;
        }

      write_msg.len = 6;
      write_msg.data = bswap32(data32);
    }
  else
    {
      write_msg.len = 4;
      write_msg.data = bswap16(data);
    }

  write_msg.reg = bswap16(reg);

  return ksz9477_write(&write_msg);
}
#endif

static int ksz9477_sgmii_read_indirect(uint32_t address, uint16_t *value,
                                       unsigned len)
{
  int ret;
  uint32_t data;

  address |= SGMII_PORT_ADDRESS_AUTO_INC_ENB;
  ret = ksz9477_reg_write32(KSZ9477_SGMII_PORT_ADDRESS, address);
  while (len-- && ret == OK)
    {
      ret = ksz9477_reg_read32(KSZ9477_SGMII_PORT_DATA, &data);
      *value++ = (uint16_t)data;
    }

  return ret;
}

static int ksz9477_sgmii_write_indirect(uint32_t address, uint16_t *value,
                                        unsigned len)
{
  int ret;
  uint32_t data;

  address |= SGMII_PORT_ADDRESS_AUTO_INC_ENB;
  ret = ksz9477_reg_write32(KSZ9477_SGMII_PORT_ADDRESS, address);
  while (len-- && ret == OK)
    {
      data = *value++;
      ret = ksz9477_reg_write32(KSZ9477_SGMII_PORT_DATA, data);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ksz9477_enable_port_vlan
 *
 * Description:
 *   Enables static port-based VLAN, which can be configured in the switch
 *   queue management's port control registers
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK or negative error number
 *
 ****************************************************************************/

int ksz9477_enable_port_vlan(void)
{
  uint32_t reg;
  int ret = ksz9477_reg_read32(KSZ9477_Q_MGMT_CONTROL0, &reg);
  if (ret)
    {
      reg |= KSZ9477_Q_MGMT_PORT_VLAN_ENABLE;
      ret = ksz9477_reg_write32(KSZ9477_Q_MGMT_CONTROL0, reg);
    }

  return ret;
}

/****************************************************************************
 * Name: ksz9477_disable_port_vlan
 *
 * Description:
 *   Disables the static port-based VLAN
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK or negative error number
 *
 ****************************************************************************/

int ksz9477_disable_port_vlan(void)
{
  uint32_t reg;
  int ret = ksz9477_reg_read32(KSZ9477_Q_MGMT_CONTROL0, &reg);
  if (ret)
    {
      reg &= ~KSZ9477_Q_MGMT_PORT_VLAN_ENABLE;
      ret = ksz9477_reg_write32(KSZ9477_Q_MGMT_CONTROL0, reg);
    }

  return ret;
}

/****************************************************************************
 * Name: ksz9477_configure_port_vlan
 *
 * Description:
 *   Configures the static port-based VLAN for a single port
 *   The change will become effective next time when the switch is
 *   initialized.
 *
 * Input Parameters:
 *   port: The port being configured (1-7)
 *   disable: Bitmask of ports where frames may not be forwarded to.
 *            Bit 0 is for port 1, bit 1 for port 2 etc.
 *   enable: Bitmask of ports where frames may be forwarded to.
 *            Bit 0 is for port 1, bit 1 for port 2 etc.
 *
 * Returned Value:
 *   OK or negative error number
 *
 ****************************************************************************/

int ksz9477_configure_port_vlan(ksz9477_port_t port, uint8_t disable,
                                uint8_t enable)
{
  if (port < KSZ9477_PORT_PHY1 || port > KSZ9477_PORT_SGMII)
    {
      return -EINVAL;
    }

  g_port_vlan_config[port - KSZ9477_PORT_PHY1] &= ~disable;
  g_port_vlan_config[port - KSZ9477_PORT_PHY1] |= enable;
  return OK;
}

/****************************************************************************
 * Name: ksz9477_configure_port_mirroring
 *
 * Description:
 *   Configures the port mirroring and snooping.
 *   The change will become effective next time when the switch is
 *   initialized.
 *
 * Input Parameters:
 *   port: The port being configured (1-7)
 *   config: Bitmask to enable/disable rx/tx mirroring, sniffer port.
 *           See header file or Port Mirroring Control Register
 *           from datasheet for further details.
 *
 * Returned Value:
 *   OK or negative error number
 *
 ****************************************************************************/

int ksz9477_configure_port_mirroring(ksz9477_port_t port, uint8_t config)
{
  if (port < KSZ9477_PORT_PHY1 || port > KSZ9477_PORT_SGMII)
    {
      return -EINVAL;
    }

  g_port_mirror_config[port - KSZ9477_PORT_PHY1] = config;

  return OK;
}

/****************************************************************************
 * Name: ksz9477_init
 *
 * Description:
 *   Switches the ksz9477's SGMII port into PHY mode and sets the
 *   default settings to work directly with an external MAC
 *
 * Input Parameters:
 *   master_port: Port connected to the host MAC
 *
 * Returned Value:
 *   OK or negative error number
 *
 ****************************************************************************/

int ksz9477_init(ksz9477_port_t master_port)
{
  int ret;
  int i;
  uint16_t regval16;
  uint32_t regval32;

  /* Read the ID registers */

  ret = ksz9477_reg_read16(KSZ9477_ID1, &regval16);
  if (ret != OK || regval16 != KSZ9477_ID)
    {
      nerr("Device not found, id %x, ret %d\n", regval16, ret);
      return ret ? ret : -EINVAL;
    }

  /* Check that the SGMII block is alive and indirect accesses work */

  ret = ksz9477_sgmii_read_indirect(KSZ9477_SGMII_ID1,
                                    (FAR uint16_t *)&regval32, 2);
  if (ret != OK || regval32 != SGMII_PHY_ID)
    {
      nerr("SGMII port access failure, id %x, ret %d\n", regval32, ret);
      return ret ? ret : -EINVAL;
    }

  if (master_port == KSZ9477_PORT_SGMII)
    {
      /* Set the switch's SGMII port into "PHY" mode and enable link */

      regval16 = (SGMII_AUTONEG_CONTROL_PCS_SGMII |
                  SGMII_AUTONEG_CONTROL_TC_MASTER |
                  SGMII_AUTONEG_CONTROL_LINK_STATUS);
      ret = ksz9477_sgmii_write_indirect(KSZ9477_SGMII_AUTONEG_CONTROL,
                                         &regval16, 1);

      /* Write to autonegotiation advertisement register activates the new
       * setting. Advertise only full duplex.
       */

      regval16 = SGMII_AUTONEG_ADVERTISE_FD;
      ret = ksz9477_sgmii_write_indirect(KSZ9477_SGMII_AUTONEG_ADVERTISE,
                                         &regval16, 1);
    }

  /* Configure the static port-based VLANs */

#ifdef CONFIG_NET_KSZ9477_PORT_VLAN

  /* Restrict traffic according to Q_MGMT_PORT_CONTROL1 registers */

  ret = ksz9477_enable_port_vlan();

  /* Configure traffic control for each port */

  for (i = 0; ret == OK && i < 7; i++)
    {
      ret = ksz9477_reg_write32(
              KSZ9477_Q_MGMT_PORT_CONTROL1(KSZ9477_PORT_PHY1 + i),
              g_port_vlan_config[i]);
    }

#endif

#ifdef CONFIG_NET_KSZ9477_PORT_SNIFF

  /* Configure the port mirroring and snooping for each port */

  for (i = 0; ret == OK && i < 7; i++)
    {
      ret = ksz9477_reg_write8(
              KSZ9477_PORT_MIRROR_CONTROL(KSZ9477_PORT_PHY1 + i),
              g_port_mirror_config[i]);
    }

#endif

  return ret;
}
