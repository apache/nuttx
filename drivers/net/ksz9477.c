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

#define ERRATA_MOD1_REGS 7
#define ERRATA_MOD9_REGS 13

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

struct errata
{
  uint8_t dev;
  uint16_t reg;
  uint16_t value;
};

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

static int ksz9477_reg_write16(uint16_t reg, uint16_t data)
{
  int ret;
  struct ksz9477_transfer_s write_msg;
  uint16_t addr = reg;
  uint32_t data32;

  /* Errata: 16-bit writes to registers 0xN120-0xN13f will corrupt the
   * adjacent registers. Workaround: perform only 32-bit writes to this
   * area.
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

static int ksz9477_mmd_read_indirect(ksz9477_port_t port, uint16_t dev,
                                     uint16_t reg, uint16_t *value,
                                     uint16_t len)
{
  int ret;

  /* Set up MMD device address */

  ret = ksz9477_reg_write16(KSZ9477_PHY_MMD_SETUP(port),
                            KSZ9477_MMD_OP_MODE_REGISTER | dev);
  if (ret != OK)
    {
      goto out;
    }

  /* Select register */

  ret = ksz9477_reg_write16(KSZ9477_PHY_MMD_DATA(port), reg);
  if (ret != OK)
    {
      goto out;
    }

  /* Set MMD operation mode */

  if (len <= 1)
    {
      /* no post increment */

      ret = ksz9477_reg_write16(KSZ9477_PHY_MMD_SETUP(port),
                                KSZ9477_MMD_OP_MODE_NO_INCREMENT | dev);
      if (ret != OK)
        {
          goto out;
        }
    }
  else
    {
      /* post increment on reads and writes */

      ret = ksz9477_reg_write16(KSZ9477_PHY_MMD_SETUP(port),
                                KSZ9477_MMD_OP_MODE_RW_INCREMENT | dev);
      if (ret != OK)
        {
          goto out;
        }
    }

  /* Read value */

  while (len-- && ret == OK)
    {
      ret = ksz9477_reg_read16(KSZ9477_PHY_MMD_DATA(port), value);
      value++;
    }

out:

  return ret;
}

static int ksz9477_mmd_write_indirect(ksz9477_port_t port, uint8_t dev,
                                      uint16_t reg, const uint16_t *value,
                                      uint16_t len)
{
  int ret;

  /* Set up MMD device address */

  ret = ksz9477_reg_write16(KSZ9477_PHY_MMD_SETUP(port),
                            KSZ9477_MMD_OP_MODE_REGISTER | dev);
  if (ret != OK)
    {
      goto out;
    }

  /* Select register */

  ret = ksz9477_reg_write16(KSZ9477_PHY_MMD_DATA(port), reg);
  if (ret != OK)
    {
      goto out;
    }

  /* Set MMD operation mode */

  if (len <= 1)
    {
      /* no post increment */

      ret = ksz9477_reg_write16(KSZ9477_PHY_MMD_SETUP(port),
                                KSZ9477_MMD_OP_MODE_NO_INCREMENT | dev);
      if (ret != OK)
        {
          goto out;
        }
    }
  else
    {
      /* post increment on reads and writes */

      ret = ksz9477_reg_write16(KSZ9477_PHY_MMD_SETUP(port),
                                KSZ9477_MMD_OP_MODE_RW_INCREMENT | dev);
      if (ret != OK)
        {
          goto out;
        }
    }

  /* Write value */

  while (len-- && ret == OK)
    {
      ret = ksz9477_reg_write16(KSZ9477_PHY_MMD_DATA(port), *value);
      value++;
    }

out:

  return ret;
}

static int ksz9477_custom_error_fixes(ksz9477_port_t port)
{
  int ret = OK;
  int j;
  uint16_t regval16;
  const struct errata err1[] = {
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0xce, 0x0100},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0xcc, 0x0ff0},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0xca, 0x0141},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0xcb, 0x0fcf},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0xc8, 0x0010},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0xd9, 0x0100},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0xc9, 0x0280},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x8f, 0x6032},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x9d, 0x248c},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x75, 0x0060},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0xd3, 0x7777},
  };

  const struct errata err2[] = {
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x0, 0x9400},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x4, 0x00e2},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x6, 0x3100},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x9, 0xe01c},
  };

  const struct errata err3[] = {
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x79, 0x010a},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x7a, 0x00ed},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x7b, 0x00d3},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x7c, 0x00bc},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x7d, 0x00a8},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x7e, 0x0096},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x7f, 0x0085},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x80, 0x0077},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x81, 0x006a},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x82, 0x005e},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x83, 0x0054},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x84, 0x004b},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x85, 0x0043},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x86, 0x003c},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x87, 0x0035},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x88, 0x002f},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x89, 0x002a},
    {KSZ9477_MMD_DEV_SIGNAL_QUALITY, 0x8a, 0x0026},
  };

  struct errata mod9[] = {
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x13, 0x6eff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x14, 0xe6ff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x15, 0x6eff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x16, 0xe6ff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x17, 0x00ff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x18, 0x43ff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x19, 0xc3ff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x1a, 0x6fff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x1b, 0x07ff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x1c, 0x0fff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x1d, 0xe7ff},
    {KSZ9477_MMD_DEV_QUIET_WIRE, 0x1e, 0xefff},
  };

  /* First turn off autoneg */

  ret = ksz9477_reg_write16(KSZ9477_PHY_CONTROL(port), 0x2100);

  if (ret != OK)
    {
      nerr("PHY Control register write failure, ret %d\n", ret);
      return ret ? ret : -EINVAL;
    }

  /* Set Remote Loopback register */

  ret = ksz9477_reg_write16(KSZ9477_PHY_REMOTE_LP(port), 0x00f0);
  if (ret != OK)
    {
      nerr("PHY Remote Loopback register write failure, ret %d\n", ret);
      return ret ? ret : -EINVAL;
    }

  /* Needed configurations for MMD Signal Quality */

  for (j = 0; ret == OK && j < 11; j++)
    {
      ret = ksz9477_mmd_write_indirect(port, err1[j].dev, err1[j].reg,
                                       &(err1[j].value), 1);
    }

  /* Needed configurations for MMD Quiet-WIRE */

  for (j = 0; ret == OK && j < 4; j++)
    {
      ret = ksz9477_mmd_write_indirect(port, err2[j].dev, err2[j].reg,
                                       &(err2[j].value), 1);
    }

  /* More configurations for MMD Signal Quality */

  for (j = 0x0; ret == OK && j < 18; j++)
    {
      ret = ksz9477_mmd_write_indirect(port, err3[j].dev, err3[j].reg,
                                       &(err3[j].value), 1);
    }

  /* Module 9: Set various registers to get correct supply current values */

  for (j = 0; ret == OK && j < 12; j++)
    {
      ret = ksz9477_mmd_write_indirect(port, mod9[j].dev, mod9[j].reg,
                                       &(mod9[j].value), 1);
    }

  /* Disable EEE */

  regval16 = 0x0;
  ret = ksz9477_mmd_write_indirect(port, KSZ9477_MMD_DEV_EEE_ADVERTISEMENT,
                                   0x3c, &regval16, 1);

  /* Turn on autoneg */

  ret = ksz9477_reg_write16(KSZ9477_PHY_CONTROL(port), 0x1140);

  if (ret != OK)
    {
      nerr("PHY Control register write failure, ret %d\n", ret);
      return ret ? ret : -EINVAL;
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
  uint16_t backup;

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

  /* Errata 16: SGMII registers are not initialized by hardware reset
   * To ensure clean environment, reset the switch now.
   */

  regval16 = SGMII_CONTROL_SOFT_RESET;
  ret = ksz9477_sgmii_write_indirect(KSZ9477_SGMII_CONTROL,
                                     &regval16, 1);

  /* Check that indirect access to PHY MMD works.
   * Write LED mode to single-LED mode and verify access by
   * reading back the value.
   */

  /* 1. Read and backup the current LED MODE value */

  ret = ksz9477_mmd_read_indirect(KSZ9477_PORT_PHY1,
                                  KSZ9477_MMD_DEV_LED_MODE,
                                  0x00, &backup, 1);

  if (ret != OK)
    {
      nerr("MMD access failure. Failed to read LED_MODE "
           "register value: ret %d\n", ret);
      return ret ? ret : -EINVAL;
    }

  /* 2. Set LED mode to sigle-LED mode */

  regval16 = 0x10;
  ret = ksz9477_mmd_write_indirect(KSZ9477_PORT_PHY1,
                                   KSZ9477_MMD_DEV_LED_MODE,
                                   0x00, &regval16, 1);
  if (ret != OK)
    {
      nerr("MMD access failure. Failed write 0x%04x to "
           "LED_MODE register: ret %d\n", regval16, ret);
      return ret ? ret : -EINVAL;
    }

  /* 3. Verify read returns Single-LED mode (0x10) */

  ret = ksz9477_mmd_read_indirect(KSZ9477_PORT_PHY1,
                                  KSZ9477_MMD_DEV_LED_MODE,
                                  0x00, &regval16, 1);

  if (ret != OK || regval16 != 0x10)
    {
      nerr("MMD access failure. Failed to verify LED_MODE "
           "register value 0x10: ret %d, regval %04x\n", ret, regval16);
      return ret ? ret : -EINVAL;
    }

  /* 4. Set back to default value from backup */

  ret = ksz9477_mmd_write_indirect(KSZ9477_PORT_PHY1,
                                   KSZ9477_MMD_DEV_LED_MODE,
                                   0x00, &backup, 1);
  if (ret != OK)
    {
      nerr("MMD access failure. Failed to write 0x%04x to "
           "LED_MODE register: ret %d\n", backup, ret);
      return ret ? ret : -EINVAL;
    }

  /* Handle some erratas */

  for (i = KSZ9477_PORT_PHY1; i <= KSZ9477_PORT_PHY5; i++)
    {
      ret = ksz9477_custom_error_fixes(i);
    }

  if (ret != OK)
    {
      nerr("Errata handling failure, ret %d\n", ret);
      return ret ? ret : -EINVAL;
    }

  /* Set up SGMII port */

  if (master_port == KSZ9477_PORT_SGMII)
    {
      /* Set the switch's SGMII port into "PHY" mode and enable link */

      regval16 = (SGMII_AUTONEG_CONTROL_PCS_SGMII |
                  SGMII_AUTONEG_CONTROL_TC_MASTER |
                  SGMII_AUTONEG_CONTROL_LINK_STATUS);
      ret = ksz9477_sgmii_write_indirect(KSZ9477_SGMII_AUTONEG_CONTROL,
                                         &regval16, 1);

      if (ret != OK)
        {
          nerr("Failed to set SGMII port into PHY mode, ret %d\n", ret);
          return ret ? ret : -EINVAL;
        }

      /* Write to autonegotiation advertisement register activates the new
       * setting. Advertise only full duplex.
       */

      regval16 = SGMII_AUTONEG_ADVERTISE_FD;
      ret = ksz9477_sgmii_write_indirect(KSZ9477_SGMII_AUTONEG_ADVERTISE,
                                         &regval16, 1);

      if (ret != OK)
        {
          nerr("Failed to set autoneg, ret %d\n", ret);
          return ret ? ret : -EINVAL;
        }
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

  if (ret != OK)
    {
      nerr("Failed to configure VLANs, ret %d\n", ret);
      return ret ? ret : -EINVAL;
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

  if (ret != OK)
    {
      nerr("Failed to configure sniffer port, ret %d\n", ret);
      return ret ? ret : -EINVAL;
    }

#endif

  return ret;
}
