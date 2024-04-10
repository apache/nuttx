/****************************************************************************
 * drivers/net/ksz9477.c
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

#if 0  /* Enable when needed */
static int ksz9477_reg_write8(uint16_t reg, uint8_t data)
{
  struct ksz9477_transfer_s write_msg;
  write_msg.len = 3;
  write_msg.reg = bswap16(reg);
  write_msg.data = data;
  return ksz9477_write(&write_msg);
}
#endif

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
                                    (uint16_t *)&regval32, 2);
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

  return ret;
}
