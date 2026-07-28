/****************************************************************************
 * drivers/net/phy/realtek/rtl8221b.c
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
#include <inttypes.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/realtek_phy.h>
#include <nuttx/ethtool.h>

#include "phy.h"
#include "rtl8221b.h"
#include "rtl8221b_fw.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rtl8221b_config_init(FAR struct phy_device_s *phydev);
static int rtl8221b_config_aneg(FAR struct phy_device_s *phydev);
static int rtl8221b_read_status(FAR struct phy_device_s *phydev);
static int rtl8221b_config_intr(FAR struct phy_device_s *phydev,
                                bool enable);
static int rtl8221b_handle_interrupt(FAR struct phy_device_s *phydev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct phy_driver_s g_rtl8221b_driver =
{
  .name        = "RTL8221B 2.5Gbps PHY",
  .phy_id      = RTL8221B_VB_CG_PHY_ID,
  .phy_id_mask = RTL8221B_PHY_ID_MASK,
  .config_init = rtl8221b_config_init,
  .config_aneg = rtl8221b_config_aneg,
  .read_status = rtl8221b_read_status,
  .config_intr = rtl8221b_config_intr,
  .handle_interrupt = rtl8221b_handle_interrupt,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8221b_write_ramcode
 *
 * Description:
 *   Write one firmware patch (ramcode) table to the PHY via MMD writes.
 *
 * Input Parameters:
 *   phydev - PHY device
 *   code   - Ramcode table (MMD device/address/value triples) to write
 *   len    - Number of entries in the ramcode table
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rtl8221b_write_ramcode(FAR struct phy_device_s *phydev,
                                  FAR const struct rtl8221b_mmd_reg_s *code,
                                  size_t len)
{
  size_t i;
  int ret;

  for (i = 0; i < len; i++)
    {
      ret = phy_write_mmd(phydev, MII_MMD_VEND2, code[i].addr,
                          code[i].value);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rtl8221b_apply_patch
 *
 * Description:
 *   Apply the RTL8221B firmware patch (NC0/NC1/NC2/UC2/UC ramcode plus the
 *   SDS patch and GPHY parameter tuning).  Mirrors the vendor reference
 *   initialization sequence, expressed purely through Clause-45 MMD
 *   register access.
 *
 * Input Parameters:
 *   phydev - PHY device
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rtl8221b_apply_patch(FAR struct phy_device_s *phydev)
{
  int ret;
  int val;
  int timeout;

  /* Patch request: MMD31.0xb820[4] = 1 */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_CTRL,
                       RTL_VEND2_PATCH_REQ, RTL_VEND2_PATCH_REQ);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for patch ready: MMD31.0xb800[6] = 1 (bounded timeout) */

  for (timeout = RTL_PATCH_READY_TIMEOUT_MS; timeout > 0; timeout--)
    {
      val = phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_STATUS);
      if (val < 0)
        {
          return val;
        }

      if (val & RTL_VEND2_PATCH_RDY)
        {
          break;
        }

      nxsig_usleep(USEC_PER_MSEC);
    }

  if (timeout == 0)
    {
      nerr("ERROR: RTL8221B patch ready (set) timeout\n");
      return -ETIMEDOUT;
    }

  /* Set patch_key & patch_lock */

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, 0xa436, 0x8024);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, 0xa438, 0x3701);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, 0xa436, 0xb82e);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, 0xa438, 0x0001);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable NC & UC patch: MMD31.0xb820[7] = 1 */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_CTRL,
                       RTL_VEND2_UCODE_EN, RTL_VEND2_UCODE_EN);
  if (ret < 0)
    {
      return ret;
    }

  /* Load NC ramcode banks */

  ret = rtl8221b_write_ramcode(phydev, g_rtl8221b_n0_ramcode,
                               g_rtl8221b_n0_ramcode_len);
  if (ret < 0)
    {
      return ret;
    }

  ret = rtl8221b_write_ramcode(phydev, g_rtl8221b_n1_ramcode,
                               g_rtl8221b_n1_ramcode_len);
  if (ret < 0)
    {
      return ret;
    }

  ret = rtl8221b_write_ramcode(phydev, g_rtl8221b_n2_ramcode,
                               g_rtl8221b_n2_ramcode_len);
  if (ret < 0)
    {
      return ret;
    }

  ret = rtl8221b_write_ramcode(phydev, g_rtl8221b_uc2_ramcode,
                               g_rtl8221b_uc2_ramcode_len);
  if (ret < 0)
    {
      return ret;
    }

  /* Disable NC & UC patch: MMD31.0xb820[7] = 0 */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_CTRL,
                       RTL_VEND2_UCODE_EN, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Load UC ramcode bank */

  ret = rtl8221b_write_ramcode(phydev, g_rtl8221b_uc_ramcode,
                               g_rtl8221b_uc_ramcode_len);
  if (ret < 0)
    {
      return ret;
    }

  /* GPHY OCP 0xb896[0] = 0 */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND2, 0xb896, 0x0001, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* GPHY OCP 0xb892[15:8] = 0 */

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, 0xb892, 0x0000);
  if (ret < 0)
    {
      return ret;
    }

  /* GPHY OCP 0xb896[0] = 1 */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND2, 0xb896, 0x0001, 0x0001);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear patch_key & patch_lock */

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, 0xa436, 0x0000);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, 0xa438, 0x0000);
  if (ret < 0)
    {
      return ret;
    }

  /* MMD31.0xb82e[0] = 0 */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND2, 0xb82e, 0x0001, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear patch key address */

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, 0xa436, 0x8024);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, 0xa438, 0x0000);
  if (ret < 0)
    {
      return ret;
    }

  /* Release patch request: MMD31.0xb820[4] = 0 */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_CTRL,
                       RTL_VEND2_PATCH_REQ, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for patch ready bit to clear: MMD31.0xb800[6] = 0 (bounded) */

  for (timeout = RTL_PATCH_READY_TIMEOUT_MS; timeout > 0; timeout--)
    {
      val = phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_STATUS);
      if (val < 0)
        {
          return val;
        }

      if (!(val & RTL_VEND2_PATCH_RDY))
        {
          break;
        }

      nxsig_usleep(USEC_PER_MSEC);
    }

  if (timeout == 0)
    {
      nerr("ERROR: RTL8221B patch ready (clear) timeout\n");
      return -ETIMEDOUT;
    }

  /* Apply GPHY tuning and OCP programming (all on VEND2). */

  ret = rtl8221b_write_ramcode(phydev, g_rtl8221b_gphy_tuning,
                               g_rtl8221b_gphy_tuning_len);
  if (ret < 0)
    {
      return ret;
    }

  /* SDS patch (MMD30 / VEND1) */

  ret = phy_write_mmd(phydev, MII_MMD_VEND1, 0x75b5, 0xe086);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND1, 0x75b6, 0x0024);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND1, 0x75b1, 0x0013);
  if (ret < 0)
    {
      return ret;
    }

  /* Update patch version marker (VEND2) */

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_REG_SEL,
                      RTL_VEND2_PATCH_VER_SEL);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_REG_DATA,
                      RTL_VEND2_PATCH_VERSION);
  if (ret < 0)
    {
      return ret;
    }

  /* PHY reset & restart Nway */

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PHYRST_REG,
                      RTL_VEND2_PHYRST_NWAY);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: rtl8221b_config_serdes
 *
 * Description:
 *   Configure the RTL8221B SerDes interface toward the MAC as a fixed
 *   2500BASE-X / SGMII link with auto-negotiation disabled, then wait for
 *   the SerDes link to come up.  All accesses are pure Clause-45 MMD
 *   register operations (MMD30 / VEND1 and MMD31 / VEND2).
 *
 * Input Parameters:
 *   phydev - PHY device
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rtl8221b_config_serdes(FAR struct phy_device_s *phydev)
{
  int val;
  int ret;
  int waited;

  /* --- SerDes option set: 2500BASE-X + SGMII --- */

  /* Clear MMD30.0x75f3[0] */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND1, RTL_VEND1_SDS_75F3, 0x0001, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Program SerDes mode select MMD30.0x697a[5:0] */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND1, RTL_VEND1_SDS_OPTION,
                       RTL_VEND1_SDS_MODE_MASK,
                       RTL_SDS_OPTION_2500BASEX_SGMII);
  if (ret < 0)
    {
      return ret;
    }

  /* 2500BASE-X specific SerDes tuning */

  ret = phy_write_mmd(phydev, MII_MMD_VEND1, RTL_VEND1_SDS_CTRL1, 0x0503);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND1, RTL_VEND1_SDS_6F10, 0xd455);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND1, RTL_VEND1_SDS_6F11, 0x8020);
  if (ret < 0)
    {
      return ret;
    }

  /* Toggle link state: set MMD31.0xa400[14] */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PHYRST_REG,
                       RTL_VEND2_LINK_CHG, RTL_VEND2_LINK_CHG);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for the SerDes link: MMD31.0xa434[2] (bounded timeout) */

  for (waited = 0; waited < RTL_SDS_LINK_TIMEOUT_MS; waited += 10)
    {
      val = phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PHYSR);
      if (val < 0)
        {
          return val;
        }

      if (val & RTL_PHYSR_LINK_RT)
        {
          break;
        }

      nxsig_usleep(10 * USEC_PER_MSEC);
    }

  if (waited >= RTL_SDS_LINK_TIMEOUT_MS)
    {
      nwarn("WARNING: RTL8221B SerDes link not up within %dms\n",
            RTL_SDS_LINK_TIMEOUT_MS);
    }

  /* Clear link-change bit: MMD31.0xa400[14] = 0 */

  ret = phy_modify_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PHYRST_REG,
                       RTL_VEND2_LINK_CHG, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* --- Disable SerDes auto-negotiation (fixed-speed link) --- */

  ret = phy_write_mmd(phydev, MII_MMD_VEND1, RTL_VEND1_SDS_ANEG_7588,
                      0x0002);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND1, RTL_VEND1_SDS_ANEG_7589,
                      0x71d0);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_write_mmd(phydev, MII_MMD_VEND1, RTL_VEND1_SDS_ANEG_7587,
                      0x0003);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: rtl8221b_config_init
 *
 * Description:
 *   Initialize RTL8221B PHY: wait for the PHY to become ready, apply the
 *   firmware patch (ramcode) when the running patch version does not match,
 *   then advertise the supported capabilities.
 *
 * Input Parameters:
 *   phydev - PHY device
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rtl8221b_config_init(FAR struct phy_device_s *phydev)
{
  int timeout;
  int val;
  int ret;

  /* Wait until the PHY reports ready: MMD31.0xa420[1:0] == 0x3 */

  for (timeout = RTL_PHY_READY_TIMEOUT_MS; timeout > 0; timeout--)
    {
      val = phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PHY_READY_REG);
      if (val < 0)
        {
          return val;
        }

      if (val != 0xffff && (val & 0x3) == 0x3)
        {
          break;
        }

      nxsig_usleep(USEC_PER_MSEC);
    }

  if (timeout == 0)
    {
      nerr("ERROR: RTL8221B not ready\n");
      return -ETIMEDOUT;
    }

  /* Check whether the firmware patch is already applied (idempotent):
   * read the running patch version from OCP 0x801e.
   */

  ret = phy_write_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_REG_SEL,
                      RTL_VEND2_PATCH_VER_SEL);
  if (ret < 0)
    {
      return ret;
    }

  val = phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_REG_DATA);
  if (val < 0)
    {
      return val;
    }

  if (val != RTL_VEND2_PATCH_VERSION)
    {
      /* Patch not present or stale: apply it */

      ret = rtl8221b_apply_patch(phydev);
      if (ret < 0)
        {
          nerr("ERROR: RTL8221B patch failed: %d\n", ret);
          return ret;
        }

      /* Verify the patch version was written back */

      ret = phy_write_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_REG_SEL,
                          RTL_VEND2_PATCH_VER_SEL);
      if (ret < 0)
        {
          return ret;
        }

      val = phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PATCH_REG_DATA);
      if (val < 0)
        {
          return val;
        }

      if (val != RTL_VEND2_PATCH_VERSION)
        {
          nerr("ERROR: RTL8221B patch verification failed (0x%04x)\n", val);
          return -EIO;
        }
    }

  phydev->supported = SUPPORTED_10BASET_FULL |
                      SUPPORTED_100BASET_FULL |
                      SUPPORTED_1000BASET_FULL |
                      SUPPORTED_AUTONEG |
                      SUPPORTED_TP |
                      SUPPORTED_MII;

  phydev->advertising = phydev->supported;

  /* The PHY was (potentially) soft-reset at the end of the firmware patch,
   * which clears any prior SerDes configuration.  Re-arm the one-shot
   * SerDes setup so it is (re)applied by read_status on the next link-up.
   */

  phydev->priv = NULL;

  /* NOTE: SerDes configuration toward the MAC is intentionally NOT done
   * here.  The SerDes "set option" sequence ends by toggling the link and
   * waiting for the real-time link bit, which only asserts once the UTP
   * link is established.  Doing it at config_init time (UTP link not yet
   * up) would stall and leave the SerDes data path unconfigured.  Instead
   * it is performed from read_status when link-up is first observed.
   */

  ninfo("RTL8221B config_init done: supported=0x%08" PRIx32 "\n",
        phydev->supported);
  return OK;
}

/****************************************************************************
 * Name: rtl8221b_config_aneg
 *
 * Description:
 *   Enable and restart auto-negotiation on RTL8221B via MMD7.
 *
 * Input Parameters:
 *   phydev - PHY device
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rtl8221b_config_aneg(FAR struct phy_device_s *phydev)
{
  return phy_write_mmd(phydev, MII_MMD_AN, MII_MMD_AN_CTRL1,
                       MII_MMD_AN_CTRL1_ENABLE | MII_MMD_AN_CTRL1_RESTART);
}

/****************************************************************************
 * Name: rtl8221b_read_status
 *
 * Description:
 *   Read link status and speed from RTL8221B vendor-specific registers.
 *   Uses MMD30 reg 0xa400 for link and MMD30 reg 0xa434 for speed.
 *
 * Input Parameters:
 *   phydev - PHY device
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rtl8221b_read_status(FAR struct phy_device_s *phydev)
{
  int val;
  int speed_hi;
  int speed_lo;
  int ret;

  /* Read PHYSR (MMD31 reg 0xa434): contains real-time link, speed and
   * duplex in a single register, so link/speed/duplex stay consistent.
   */

  val = phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PHYSR);
  if (val < 0)
    {
      return val;
    }

  /* bit2: real-time link status (use this rather than FESR.2 which is a
   * latched "link lost since last read" indication).
   */

  phydev->link = !!(val & RTL_PHYSR_LINK_RT);

  if (!phydev->link)
    {
      phydev->speed = 0;
      phydev->duplex = DUPLEX_UNKNOWN;

      /* Do NOT re-arm the SerDes setup here.  The SerDes configuration is
       * persistent in the PHY registers and only needs to be applied once
       * per config_init (which clears the flag).  Re-arming on every
       * link-down would cause the config_serdes link toggle to bounce the
       * link, creating a perpetual up/down loop.
       */

      return OK;
    }

  /* Link is up.  Configure the host-side SerDes once per link-up session.
   * This is deferred here (rather than config_init) because the SerDes
   * "set option" sequence waits on the real-time link bit, which is only
   * valid once the UTP link is established.
   *
   * No locking is needed around the SDS-done marker: read_status() for a
   * given PHY is only ever invoked from the phylib state machine via the
   * single state_work handle (the poll and interrupt paths are funnelled
   * onto the same work), so calls for the same phydev are strictly
   * serialized and never run concurrently.  The marker is stored directly
   * in phydev->priv as a boolean (NULL = not configured, non-NULL = SerDes
   * already set up for this session), so distinct PHYs keep independent
   * state, and rtl8221b_config_serdes() writes idempotent persistent PHY
   * registers, so even a redundant call would be harmless.
   */

  if (phydev->priv == NULL)
    {
      ret = rtl8221b_config_serdes(phydev);
      if (ret < 0)
        {
          nerr("ERROR: RTL8221B SerDes config failed: %d\n", ret);
          return ret;
        }

      phydev->priv = (FAR void *)1;

      /* Re-read PHYSR after SerDes (re)configuration to report a coherent
       * speed/duplex below.
       */

      val = phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_PHYSR);
      if (val < 0)
        {
          return val;
        }
    }

  speed_hi = (val & RTL_PHYSR_SPEED_HI_MASK) >> RTL_PHYSR_SPEED_HI_SHIFT;
  speed_lo = (val & RTL_PHYSR_SPEED_LO_MASK) >> RTL_PHYSR_SPEED_LO_SHIFT;

  /* PHYSR Speed_h(bit10:9) + Speed_l(bit5:4) per datasheet table:
   *   00 00 -> 10M   00 01 -> 100M   00 10 -> 1000M
   *   01 11 -> 2.5G Lite (1G data)   01 01 -> 2.5G
   * other combinations are reserved.
   */

  switch (speed_hi)
    {
      case 0:
        switch (speed_lo)
          {
            case 0:
              phydev->speed = 10;
              break;
            case 1:
              phydev->speed = 100;
              break;
            case 2:
              phydev->speed = 1000;
              break;
            default:
              phydev->speed = 0;
              break;
          }
        break;

      case 1:
        switch (speed_lo)
          {
            case 1:
              phydev->speed = 2500;
              break;
            case 3:
              phydev->speed = 2500; /* 2.5G lite */
              break;
            default:
              phydev->speed = 0;
              break;
          }
        break;

      default:
        phydev->speed = 0;
        break;
    }

  phydev->duplex = (val & RTL_PHYSR_DUPLEX) ? DUPLEX_FULL : DUPLEX_HALF;

  return OK;
}

/****************************************************************************
 * Name: rtl8221b_config_intr
 *
 * Description:
 *   Enable or disable the RTL8221B PHY interrupt output (INTB).  Programs
 *   the vendor interrupt-enable register (MMD31.0xa4d2) to assert INTB on
 *   link status changes.  When disabling, the pending interrupt status is
 *   also cleared.
 *
 * Input Parameters:
 *   phydev - PHY device
 *   enable - true to enable link-change interrupts, false to disable
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int rtl8221b_config_intr(FAR struct phy_device_s *phydev, bool enable)
{
  int ret;

  if (enable)
    {
      /* Clear any stale status first (read-to-clear), then enable the
       * link-change interrupt source.  The status read is a best-effort
       * clear; its value is intentionally ignored.
       */

      phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_INSR_REG);
      ret = phy_write_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_INER_REG,
                          RTL_VEND2_INTR_LINK_CHG);
    }
  else
    {
      /* Mask all interrupt sources, then clear any pending status.  The
       * trailing status read is a best-effort clear; its value is
       * intentionally ignored.
       */

      ret = phy_write_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_INER_REG, 0);
      phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_INSR_REG);
    }

  return ret;
}

/****************************************************************************
 * Name: rtl8221b_handle_interrupt
 *
 * Description:
 *   Handle a pending RTL8221B PHY interrupt.  Reads (and thereby clears) the
 *   vendor interrupt status register (MMD31.0xa4d4), releasing the INTB
 *   line, and decodes the pending sources.  The driver fully owns interrupt
 *   decode/ack.
 *
 * Input Parameters:
 *   phydev - PHY device
 *
 * Returned Value:
 *   >0 if a link-relevant event is pending (the phylib state machine then
 *      re-reads link status);
 *    0 if no link-relevant event was pending;
 *   <0 (negated errno) on MDIO/HW failure (the state machine enters
 *      PHY_ERROR).
 *
 ****************************************************************************/

static int rtl8221b_handle_interrupt(FAR struct phy_device_s *phydev)
{
  int val;

  /* Reading INSR clears the status and de-asserts INTB. */

  val = phy_read_mmd(phydev, MII_MMD_VEND2, RTL_VEND2_INSR_REG);
  if (val < 0)
    {
      return val;
    }

  /* Only the link-change source is enabled (see config_intr); report
   * whether a link-relevant event is pending so the state machine re-reads
   * link status.  Any other (currently unused) sources are simply cleared
   * by the read above.
   */

  return (val & RTL_VEND2_INTR_LINK_CHG) ? 1 : 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8221b_register
 *
 * Description:
 *   Register the RTL8221B PHY driver with the phylib framework.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rtl8221b_register(void)
{
  return phy_driver_register(&g_rtl8221b_driver);
}
