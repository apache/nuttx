/****************************************************************************
 * drivers/net/phy/realtek/rtl9000b.c
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

#include <errno.h>
#include <debug.h>
#include <nuttx/arch.h>

#include <nuttx/net/phy.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/realtek_phy.h>
#include <nuttx/ethtool.h>

#include "phy.h"
#include "rtl9000b.h"
#include "rtl9000b_fw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Indirect window address iterations for uCode ready polling loops */

#define RTL_UCODE_ITERS       2000

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rtl9000b_config_init(FAR struct phy_device_s *phydev);
static int rtl9000b_config_aneg(FAR struct phy_device_s *phydev);
static int rtl9000b_read_status(FAR struct phy_device_s *phydev);
static int rtl9000b_config_power(FAR struct phy_device_s *phydev,
                                 bool enable);
static int rtl9000b_config_intr(FAR struct phy_device_s *phydev,
                                bool enable);
static int rtl9000b_handle_interrupt(FAR struct phy_device_s *phydev);
static int rtl9000b_read_page(FAR struct phy_device_s *phydev);
static int rtl9000b_write_page(FAR struct phy_device_s *phydev, int page);

static int rtl9000b_win_read(FAR struct phy_device_s *phydev,
                             uint16_t addr, FAR uint16_t *value);
static int rtl9000b_win_write(FAR struct phy_device_s *phydev,
                              uint16_t addr, uint16_t value);
static int rtl9000b_win_modify(FAR struct phy_device_s *phydev,
                               uint16_t addr, uint16_t mask,
                               uint16_t set);
static int rtl9000b_poll_ucode(FAR struct phy_device_s *phydev,
                               uint16_t expected, int timeout);
static int rtl9000b_load_dsp(FAR struct phy_device_s *phydev,
                             FAR const uint16_t *params, int count);
static int rtl9000b_wait_ready(FAR struct phy_device_s *phydev);
static int rtl9000b_write_config(FAR struct phy_device_s *phydev);
static int rtl9000b_check_config(FAR struct phy_device_s *phydev);
static int rtl9000b_set_dvddio(FAR struct phy_device_s *phydev,
                               rtl9000b_dvddio_e sel);
static int rtl9000b_soft_reset(FAR struct phy_device_s *phydev);
static int rtl9000b_set_op_mode(FAR struct phy_device_s *phydev,
                           uint8_t cmd);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct phy_driver_s g_rtl9000b_driver =
{
  .name             = "Realtek RTL9000B",
  .phy_id           = RTL9000B_PHY_ID,
  .phy_id_mask      = RTL9000B_PHY_ID_MASK,
  .config_init      = rtl9000b_config_init,
  .config_aneg      = rtl9000b_config_aneg,
  .read_status      = rtl9000b_read_status,
  .config_power     = rtl9000b_config_power,
  .config_intr      = rtl9000b_config_intr,
  .handle_interrupt = rtl9000b_handle_interrupt,
  .read_page        = rtl9000b_read_page,
  .write_page       = rtl9000b_write_page,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl9000b_win_read
 *
 * Description:
 *   Read a register through the indirect window (reg 0x1b/0x1c = reg 27/28).
 *   Holds the MDIO bus lock across the entire write-addr+read-data sequence.
 *
 ****************************************************************************/

static int rtl9000b_win_read(FAR struct phy_device_s *phydev,
                        uint16_t addr, FAR uint16_t *value)
{
  FAR struct mdio_bus_s *bus = phydev->bus;
  uint8_t pa = phydev->mdio_addr;
  int ret;

  mdio_lock(bus);
  ret = mdio_write_nolock(bus, pa, RTL_WIN_ADDR, addr);
  if (ret >= 0)
    {
      ret = mdio_read_nolock(bus, pa, RTL_WIN_DATA, value);
    }

  mdio_unlock(bus);
  return ret;
}

/****************************************************************************
 * Name: rtl9000b_win_write
 *
 * Description:
 *   Write a register through the indirect window.  Holds the MDIO bus
 *   lock across the entire write-addr+write-data sequence.
 *
 ****************************************************************************/

static int rtl9000b_win_write(FAR struct phy_device_s *phydev,
                         uint16_t addr, uint16_t value)
{
  FAR struct mdio_bus_s *bus = phydev->bus;
  uint8_t pa = phydev->mdio_addr;
  int ret;

  mdio_lock(bus);
  ret = mdio_write_nolock(bus, pa, RTL_WIN_ADDR, addr);
  if (ret >= 0)
    {
      ret = mdio_write_nolock(bus, pa, RTL_WIN_DATA, value);
    }

  mdio_unlock(bus);
  return ret;
}

/****************************************************************************
 * Name: rtl9000b_win_modify
 *
 * Description:
 *   Read-modify-write a register through the indirect window.
 *   Clears bits in 'mask' then sets bits in 'set'.
 *
 ****************************************************************************/

static int rtl9000b_win_modify(FAR struct phy_device_s *phydev,
                               uint16_t addr, uint16_t mask,
                               uint16_t set)
{
  uint16_t val;
  int ret;

  ret = rtl9000b_win_read(phydev, addr, &val);
  if (ret < 0)
    {
      return ret;
    }

  val &= ~mask;
  val |= set;

  return rtl9000b_win_write(phydev, addr, val);
}

/****************************************************************************
 * Name: rtl9000b_poll_ucode
 *
 * Description:
 *   Poll the uCode ready bit (bit6 of indirect window 0xb800) until it
 *   matches 'expected'.  Holds the bus lock for the entire poll loop,
 *   matching the original driver's single write27 + repeated read28 pattern.
 *
 ****************************************************************************/

static int rtl9000b_poll_ucode(FAR struct phy_device_s *phydev,
                               uint16_t expected, int timeout)
{
  FAR struct mdio_bus_s *bus = phydev->bus;
  uint8_t pa = phydev->mdio_addr;
  uint16_t val;
  int ret;

  mdio_lock(bus);
  ret = mdio_write_nolock(bus, pa, RTL_WIN_ADDR, 0xb800);
  if (ret < 0)
    {
      goto out;
    }

  do
    {
      ret = mdio_read_nolock(bus, pa, RTL_WIN_DATA, &val);
      if (ret < 0)
        {
          goto out;
        }

      if ((val & RTL_UCODE_RDY) == expected)
        {
          break;
        }
    }
  while (--timeout > 0);

  ret = timeout > 0 ? OK : -ETIMEDOUT;

out:
  mdio_unlock(bus);
  return ret;
}

/****************************************************************************
 * Name: rtl9000b_load_dsp
 *
 * Description:
 *   Write DSP parameter words sequentially through indirect window 0xa014.
 *   Holds the bus lock for the entire burst, matching the original driver's
 *   single write27 + repeated write28 pattern.
 *
 ****************************************************************************/

static int rtl9000b_load_dsp(FAR struct phy_device_s *phydev,
                        FAR const uint16_t *params, int count)
{
  FAR struct mdio_bus_s *bus = phydev->bus;
  uint8_t pa = phydev->mdio_addr;
  int ret;
  int i;

  mdio_lock(bus);
  ret = mdio_write_nolock(bus, pa, RTL_WIN_ADDR, 0xa014);
  if (ret < 0)
    {
      goto out;
    }

  for (i = 0; i < count; i++)
    {
      ret = mdio_write_nolock(bus, pa, RTL_WIN_DATA, params[i]);
      if (ret < 0)
        {
          goto out;
        }
    }

out:
  mdio_unlock(bus);
  return ret;
}

/****************************************************************************
 * Name: rtl9000b_wait_ready
 *
 * Description:
 *   Wait for the PHY to complete its internal power-on sequence.
 *   Polls page 0xa42 register 0x10 until bit [2:0] == 0x3.
 *
 ****************************************************************************/

static int rtl9000b_wait_ready(FAR struct phy_device_s *phydev)
{
  int timeout = RTL_READY_TO;
  uint16_t val;
  int ret;

  do
    {
      ret = phy_read_paged(phydev, RTL_PAGE_DEFAULT, 0x10, &val);
      if (ret < 0)
        {
          return ret;
        }

      if ((val & RTL_READY_MASK) == RTL_READY_VAL)
        {
          return OK;
        }
    }
  while (--timeout > 0);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rtl9000b_write_config
 *
 * Description:
 *   Write the vendor-provided initialization register sequence.
 *   Exact transliteration of RTL9000Bx_Initial_Configuration().
 *
 ****************************************************************************/

static int rtl9000b_write_config(FAR struct phy_device_s *phydev)
{
  int ret;

  /* Phase 1: indirect window writes + page-banked writes */

  rtl9000b_win_write(phydev, 0x8017, 0xfb03);
  rtl9000b_win_write(phydev, 0x8092, 0x6000);
  rtl9000b_win_write(phydev, 0x80af, 0x6000);
  rtl9000b_win_write(phydev, 0x807d, 0x4443);
  rtl9000b_win_write(phydev, 0x809a, 0x4443);
  rtl9000b_win_write(phydev, 0x81a3, 0x0f00);
  phy_write_paged(phydev, 0x0a81, 18, 0x0004);
  rtl9000b_win_write(phydev, 0x83c8, 0x0005);
  phy_write_paged(phydev, 0x0a5c, 18, 0x0003);

  /* Phase 2: start uCode loading */

  rtl9000b_win_write(phydev, 0xb820, 0x0010);
  rtl9000b_win_write(phydev, 0xb830, 0x8000);

  ret = rtl9000b_poll_ucode(phydev, RTL_UCODE_RDY, RTL_UCODE_TO);
  if (ret < 0)
    {
      return ret;
    }

  /* Phase 3: DSP parameter loading */

  rtl9000b_win_write(phydev, 0x8020, 0x3300);
  rtl9000b_win_write(phydev, 0xb82e, 0x0001);
  rtl9000b_win_write(phydev, 0xb820, 0x0290);
  rtl9000b_win_write(phydev, 0xa012, 0x0000);
  ret = rtl9000b_load_dsp(phydev, g_rtl9000b_param_check2, 5);
  if (ret < 0)
    {
      return ret;
    }

  rtl9000b_win_write(phydev, 0xa01a, 0x0000);
  rtl9000b_win_write(phydev, 0xa000, 0x11ef);

  /* Phase 4: wait for uCode loading to complete */

  rtl9000b_win_write(phydev, 0xb820, 0x0210);
  rtl9000b_win_write(phydev, 0xb82e, 0x0000);
  rtl9000b_win_write(phydev, 0x8020, 0x0000);
  rtl9000b_win_write(phydev, 0xb820, 0x0000);

  return rtl9000b_poll_ucode(phydev, 0, RTL_UCODE_TO);
}

/****************************************************************************
 * Name: rtl9000b_check_config
 *
 * Description:
 *   Read back and verify all configured registers.
 *   Exact transliteration of RTL9000Bx_Initial_Configuration_Check().
 *
 ****************************************************************************/

static int rtl9000b_check_config(FAR struct phy_device_s *phydev)
{
  uint16_t data;
  uint16_t data_temp;
  uint16_t expected;
  uint16_t page;
  uint16_t reg;
  uint16_t i;
  uint16_t param_addr;
  int ret;

  /* Pass 1: verify param_check[] 9 triplets */

  for (i = 0; i < 27; i += 3)
    {
      page     = g_rtl9000b_param_check[i];
      reg      = g_rtl9000b_param_check[i + 1];
      expected = g_rtl9000b_param_check[i + 2];

      if (page == 0)
        {
          ret = rtl9000b_win_read(phydev, reg, &data);
        }
      else
        {
          ret = phy_read_paged(phydev, page, reg, &data);
        }

      if (ret < 0)
        {
          return ret;
        }

      if (expected != data)
        {
          nerr("ERROR: RTL9000B param[%d] mismatch: "
               "page=0x%04x reg=0x%04x got=0x%04x exp=0x%04x\n",
               i / 3, page, reg, data, expected);
          return -EIO;
        }
    }

  /* Pass 2: re-start uCode */

  rtl9000b_win_write(phydev, 0xb820, 0x0010);
  rtl9000b_win_write(phydev, 0xb830, 0x8000);

  ret = rtl9000b_poll_ucode(phydev, RTL_UCODE_RDY, RTL_UCODE_ITERS);
  if (ret < 0)
    {
      return ret;
    }

  /* Pass 3: verify DSP parameters via page 0xa01 */

  rtl9000b_win_write(phydev, 0x8020, 0x3300);
  rtl9000b_win_write(phydev, 0xb82e, 0x0001);

  param_addr = 0;
  for (i = 0; i < 5; i++)
    {
      ret = phy_read_paged(phydev, 0x0a01, 17, &data_temp);
      if (ret < 0)
        {
          return ret;
        }

      data_temp &= ~0x01ff;
      param_addr &= 0x01ff;
      ret = phy_write_paged(phydev, 0x0a01, 17,
                            (data_temp | param_addr));
      if (ret < 0)
        {
          return ret;
        }

      ret = phy_read_paged(phydev, 0x0a01, 18, &data);
      if (ret < 0)
        {
          return ret;
        }

      expected = g_rtl9000b_param_check2[i];
      if (expected != data)
        {
          nerr("ERROR: RTL9000B dsp param[%d] mismatch: "
               "got=0x%04x exp=0x%04x\n",
               i, data, expected);
          return -EIO;
        }

      param_addr++;
    }

  /* Pass 4: verify remaining parameter */

  for (i = 0; i < 3; i += 3)
    {
      page     = g_rtl9000b_param_check3[i];
      reg      = g_rtl9000b_param_check3[i + 1];
      expected = g_rtl9000b_param_check3[i + 2];

      if (page == 0)
        {
          ret = rtl9000b_win_read(phydev, reg, &data);
        }
      else
        {
          ret = phy_read_paged(phydev, page, reg, &data);
        }

      if (ret < 0)
        {
          return ret;
        }

      if (expected != data)
        {
          nerr("ERROR: RTL9000B check3 mismatch: "
               "page=0x%04x reg=0x%04x got=0x%04x exp=0x%04x\n",
               page, reg, data, expected);
          return -EIO;
        }
    }

  /* Cleanup: release uCode */

  rtl9000b_win_write(phydev, 0xb82e, 0x0000);
  rtl9000b_win_write(phydev, 0x8020, 0x0000);
  rtl9000b_win_write(phydev, 0xb820, 0x0000);

  return rtl9000b_poll_ucode(phydev, 0, RTL_UCODE_ITERS);
}

/****************************************************************************
 * Name: rtl9000b_set_dvddio
 *
 * Description:
 *   Select the DVDDIO supply source (page 0xa4c reg 18 bits [12:10]).
 *   Transliteration of RTL9000Bx_ISR_SEL().
 *
 ****************************************************************************/

static int rtl9000b_set_dvddio(FAR struct phy_device_s *phydev,
                               rtl9000b_dvddio_e sel)
{
  uint16_t set_val;

  switch (sel)
    {
      case DVDDIO_LDO_1V8:
        set_val = 0x0800;
        break;
      case DVDDIO_LDO_2V5:
        set_val = 0x1000;
        break;
      case DVDDIO_EXTPWR_1V8:
        set_val = 0x2000;
        break;
      case DVDDIO_EXTPWR_2V5:
        set_val = 0x2800;
        break;
      case DVDDIO_EXTPWR_3V3:
        set_val = 0x3000;
        break;
      default:
        return -EINVAL;
    }

  return phy_modify_paged(phydev, RTL_PAGE_DVDDIO, 18, 0x3800, set_val);
}

/****************************************************************************
 * Name: rtl9000b_soft_reset
 *
 * Description:
 *   Soft-reset the PHY via MCR bit 15.  Polls until the reset completes
 *   (MCR reads back as 0x2100).
 *
 ****************************************************************************/

static int rtl9000b_soft_reset(FAR struct phy_device_s *phydev)
{
  int timeout = RTL_RST_TO;
  int ret;

  ret = phy_write(phydev, MII_MCR, RTL_MCR_RST);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      ret = phy_read(phydev, MII_MCR);
      if (ret < 0)
        {
          return ret;
        }

      if (ret == RTL_RST_DONE)
        {
          return OK;
        }
    }
  while (--timeout > 0);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rtl9000b_set_op_mode
 *
 * Description:
 *   Set the PHY operational mode (Normal / Sleep / Standby) via the OPCR
 *   registers (OPCR2 at indirect 0xdd00, OPCR4 at indirect 0xdd20).
 *   Transliteration of RTL9000Bx_Set_OP_state().
 *
 ****************************************************************************/

static int rtl9000b_set_op_mode(FAR struct phy_device_s *phydev, uint8_t cmd)
{
  uint16_t mode;
  int ret;

  switch (cmd)
    {
      case OP_CMD_NORMAL:
        mode = RTL_OPCR_NORMAL;
        break;
      case OP_CMD_SLEEP:
        mode = RTL_OPCR_SLEEP;
        break;
      case OP_CMD_STANDBY:
        mode = RTL_OPCR_STANDBY;
        break;
      default:
        return -EINVAL;
    }

  ret = rtl9000b_win_modify(phydev, RTL_OPCR2, RTL_OPCR_MODE_MSK, mode);
  if (ret < 0)
    {
      return ret;
    }

  /* Normal mode also requires OPCR4 configuration */

  if (cmd == OP_CMD_NORMAL)
    {
      ret = rtl9000b_win_modify(phydev, RTL_OPCR4, RTL_OPCR4_MSK,
                                RTL_OPCR4_NORMAL);
    }

  return ret;
}

/****************************************************************************
 * Name: rtl9000b_config_init
 *
 * Description:
 *   Initialize RTL9000B PHY: wait for the PHY to become ready, write the
 *   vendor initialization sequence (convergence parameters, uCode, DSP),
 *   verify all configured registers, set DVDDIO to external 3.3V, soft-
 *   reset and set Normal operation mode.  100BASE-T1 has no Clause 28 AN,
 *   so autoneg is disabled and DSP training completes automatically.
 *
 * Input Parameters:
 *   phydev - PHY device
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rtl9000b_config_init(FAR struct phy_device_s *phydev)
{
  int ret;

  /* 6-step vendor initialization sequence */

  ret = rtl9000b_wait_ready(phydev);
  if (ret < 0)
    {
      nerr("ERROR: RTL9000B not ready: %d\n", ret);
      return ret;
    }

  ret = rtl9000b_write_config(phydev);
  if (ret < 0)
    {
      nerr("ERROR: RTL9000B config write failed: %d\n", ret);
      return ret;
    }

  ret = rtl9000b_check_config(phydev);
  if (ret < 0)
    {
      nerr("ERROR: RTL9000B config check failed: %d\n", ret);
      return ret;
    }

  ret = rtl9000b_set_dvddio(phydev, DVDDIO_EXTPWR_3V3);
  if (ret < 0)
    {
      nerr("ERROR: RTL9000B DVDDIO set failed: %d\n", ret);
      return ret;
    }

  ret = rtl9000b_soft_reset(phydev);
  if (ret < 0)
    {
      nerr("ERROR: RTL9000B soft reset failed: %d\n", ret);
      return ret;
    }

  ret = rtl9000b_set_op_mode(phydev, OP_CMD_NORMAL);
  if (ret < 0)
    {
      nerr("ERROR: RTL9000B set OP mode failed: %d\n", ret);
      return ret;
    }

  /* Advertise 100BASE-T1 full-duplex capabilities */

  phydev->supported   = SUPPORTED_100BASET1_FULL |
                        SUPPORTED_TP;
  phydev->advertising = phydev->supported;
  phydev->autoneg     = false;

  ninfo("RTL9000B config_init done: supported=0x%lx\n",
        phydev->supported);
  return OK;
}

/****************************************************************************
 * Name: rtl9000b_config_aneg
 *
 * Description:
 *   Configure auto-negotiation.  100BASE-T1 has no Clause 28 AN; DSP
 *   training completes automatically in Normal mode.  When recovering from
 *   HALTED or ERROR state the full config_init sequence is re-run because
 *   the convergence parameters must be rewritten.
 *
 * Input Parameters:
 *   phydev - PHY device
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rtl9000b_config_aneg(FAR struct phy_device_s *phydev)
{
  /* 100BASE-T1 has no Clause 28 auto-negotiation.
   * "Re-trigger negotiation" = re-run the full vendor init sequence:
   * convergence parameters, DSP loading, DVDDIO, soft reset, Normal mode.
   *
   * Use phydev->state (not hardware OPCR2) to decide whether to
   * re-init: PHY_READY means config_init was just called by
   * phy_connect(), so skip.  PHY_HALTED/PHY_ERROR mean recovery
   * from stop or error, so full re-init is needed.
   */

  if (phydev->state == PHY_HALTED || phydev->state == PHY_ERROR)
    {
      return rtl9000b_config_init(phydev);
    }

  return OK;
}

/****************************************************************************
 * Name: rtl9000b_read_status
 *
 * Description:
 *   Poll the PHY for MDIO liveness and link status.  Uses PHYSR2 (reg 0x1a)
 *   bit2 for real-time link detection per vendor recommendation.  Returns
 *   -EIO when the PHY is unreachable (Sleep/Power-Off); otherwise updates
 *   phydev->link/speed/duplex and returns OK.
 *
 * Input Parameters:
 *   phydev - PHY device
 *
 * Returned Value:
 *   OK on success (link up or down); -EIO if the PHY is unreachable.
 *
 ****************************************************************************/

static int rtl9000b_read_status(FAR struct phy_device_s *phydev)
{
  int val;

  /* PHYSR2 (reg 0x1a) bit2: real-time link status.
   * Vendor note: this bit also implies local receiver OK (locok)
   * and descrambler OK, so separate locok/remok/PCS checks are
   * unnecessary.
   */

  val = phy_read(phydev, RTL_PHYSR2);
  if (val < 0 || val == 0xffff)
    {
      return -EIO;
    }

  val = phy_read(phydev, RTL_PHYSR2);
  if (val < 0 || val == 0xffff)
    {
      return -EIO;
    }

  phydev->link = (val & RTL_PHYSR2_LINK) != 0;

  /* Publish the result.
   * The caller (phy_check_link) holds phydev->lock, so no additional
   * locking is needed here.  link == false is a normal outcome
   * (cable unplugged) -- return OK so the framework continues polling.
   */

  if (phydev->link)
    {
      phydev->speed  = 100;
      phydev->duplex = DUPLEX_FULL;
    }
  else
    {
      phydev->link   = false;
      phydev->speed  = 0;
      phydev->duplex = DUPLEX_UNKNOWN;
    }

  return OK;
}

/****************************************************************************
 * Name: rtl9000b_config_power
 *
 * Description:
 *   Power the PHY up or down.  Power down enters Sleep mode; power up
 *   soft-resets, sets DVDDIO to external 3.3V, and returns to Normal
 *   operation mode.
 *
 * Input Parameters:
 *   phydev - PHY device
 *   enable - true to power up, false to power down
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rtl9000b_config_power(FAR struct phy_device_s *phydev,
                                 bool enable)
{
  if (enable)
    {
      int ret;

      ret = rtl9000b_soft_reset(phydev);
      if (ret < 0)
        {
          return ret;
        }

      ret = rtl9000b_set_dvddio(phydev, DVDDIO_EXTPWR_3V3);
      if (ret < 0)
        {
          return ret;
        }

      return rtl9000b_set_op_mode(phydev, OP_CMD_NORMAL);
    }

  return rtl9000b_set_op_mode(phydev, OP_CMD_SLEEP);
}

/****************************************************************************
 * Name: rtl9000b_config_intr
 *
 * Description:
 *   Enable or disable PHY interrupt output (INTB).
 *   Enable:  clear pending, program GINER/GINMR and OPINER/OPINMR.
 *   Disable: mask all, clear pending.
 *
 ****************************************************************************/

static int rtl9000b_config_intr(FAR struct phy_device_s *phydev,
                                bool enable)
{
  uint16_t dummy;

  if (enable)
    {
      /* Clear all pending interrupts first (read-clear) */

      phy_read(phydev, RTL_GINSR);
      rtl9000b_win_read(phydev, RTL_OPINSR1, &dummy);
      rtl9000b_win_read(phydev, RTL_OPINSR2, &dummy);
      rtl9000b_win_read(phydev, RTL_OPINSR3, &dummy);

      /* Enable General interrupts: Fatal + Link Change + PHY Status */

      phy_write_paged(phydev, RTL_PAGE_DEFAULT,
                      RTL_GINER, RTL_GINTR_ENABLE);
      phy_write_paged(phydev, RTL_PAGE_DEFAULT,
                      RTL_GINMR, ~RTL_GINTR_ENABLE);

      /* Enable only known OP interrupt sources (leave reserved bits
       * alone to avoid spurious interrupts from undefined bits).
       */

      rtl9000b_win_write(phydev, RTL_OPINER1, RTL_UV_MASK);
      rtl9000b_win_write(phydev, RTL_OPINER2,
                         RTL_OT_FLAG | RTL_REC_MASK);
      rtl9000b_win_write(phydev, RTL_OPINER3, 0x0000);

      /* Unmask corresponding bits (write 0 = unmask) */

      rtl9000b_win_write(phydev, RTL_OPINMR1,
                         ~RTL_UV_MASK);
      rtl9000b_win_write(phydev, RTL_OPINMR2,
                         ~(RTL_OT_FLAG | RTL_REC_MASK));
      rtl9000b_win_write(phydev, RTL_OPINMR3, 0xffff);
    }
  else
    {
      /* Mask all interrupts */

      phy_write_paged(phydev, RTL_PAGE_DEFAULT, RTL_GINMR, 0xffff);
      rtl9000b_win_write(phydev, RTL_OPINMR1, 0xffff);
      rtl9000b_win_write(phydev, RTL_OPINMR2, 0xffff);
      rtl9000b_win_write(phydev, RTL_OPINMR3, 0xffff);

      /* Clear any pending status */

      phy_read(phydev, RTL_GINSR);
    }

  return OK;
}

/****************************************************************************
 * Name: rtl9000b_handle_interrupt
 *
 * Description:
 *   Handle a pending PHY interrupt.  Reads GINSR (read-clear, releases
 *   INTB) and OPINSR1/2/3 for detailed source decode.
 *
 ****************************************************************************/

static int rtl9000b_handle_interrupt(FAR struct phy_device_s *phydev)
{
  int ginsr;
  uint16_t op1;
  uint16_t op2;
  uint16_t op3;
  bool link_event = false;

  ginsr = phy_read(phydev, RTL_GINSR);
  if (ginsr < 0)
    {
      return ginsr;
    }

  if (ginsr & RTL_GINTR_FATAL)
    {
      nerr("RTL9000B: PHY FATAL ERROR\n");
      return -EIO;
    }

  if (ginsr & RTL_GINTR_LINK)
    {
      link_event = true;
    }

  if (ginsr & RTL_GINTR_PHY)
    {
      /* Read OP interrupt status for detailed source decode */

      rtl9000b_win_read(phydev, RTL_OPINSR1, &op1);
      rtl9000b_win_read(phydev, RTL_OPINSR2, &op2);
      rtl9000b_win_read(phydev, RTL_OPINSR3, &op3);

      if (op1 & RTL_UV_MASK)
        {
          nwarn("WARNING: RTL9000B UV SLEEP OPINSR1=0x%04x\n", op1);
          link_event = true;
        }

      if (op2 & RTL_OT_FLAG)
        {
          nwarn("WARNING: RTL9000B OVER TEMP\n");
        }

      if (op2 & RTL_REC_MASK)
        {
          ninfo("RTL9000B UV recovered, restoring Normal mode\n");

          /* PHY auto-transitions Sleep → Standby on UV recovery.
           * Must explicitly switch back to Normal mode for link training.
           */

          rtl9000b_set_op_mode(phydev, OP_CMD_NORMAL);
          link_event = true;
        }

      if (op3)
        {
          link_event = true;
        }
    }

  return link_event;
}

/****************************************************************************
 * Name: rtl9000b_read_page
 *
 * Description:
 *   Return the currently selected page.  RTL9000B uses register 31 (0x1f)
 *   as the page-select register.  The MDIO bus lock is already held by the
 *   phylib core (phy_select_page), so use the lockless primitive.
 *
 ****************************************************************************/

static int rtl9000b_read_page(FAR struct phy_device_s *phydev)
{
  uint16_t page;
  int ret;

  ret = mdio_read_nolock(phydev->bus, phydev->mdio_addr, 31, &page);
  if (ret < 0)
    {
      return ret;
    }

  return page;
}

/****************************************************************************
 * Name: rtl9000b_write_page
 *
 * Description:
 *   Select 'page' by writing it to register 31 (0x1f).  The MDIO bus lock
 *   is already held by the phylib core (phy_select_page), so use the
 *   lockless primitive.
 *
 ****************************************************************************/

static int rtl9000b_write_page(FAR struct phy_device_s *phydev, int page)
{
  return mdio_write_nolock(phydev->bus, phydev->mdio_addr, 31, page);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl9000b_register
 *
 * Description:
 *   Register the RTL9000B PHY driver with the phylib framework.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rtl9000b_register(void)
{
  return phy_driver_register(&g_rtl9000b_driver);
}
