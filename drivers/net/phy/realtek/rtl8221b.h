/****************************************************************************
 * drivers/net/phy/realtek/rtl8221b.h
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

#ifndef __DRIVERS_NET_PHY_REALTEK_RTL8221B_H
#define __DRIVERS_NET_PHY_REALTEK_RTL8221B_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/phy.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PHY IDs */

#define RTL8221B_VB_CG_PHY_ID    0x001cc849
#define RTL8221B_PHY_ID_MASK     0xfffffff0

/* Standard IEEE 802.3 Clause 45 MMD addresses are defined in
 * <nuttx/net/mii.h> as MII_MMD_PMAPMD (1), MII_MMD_AN (7),
 * MII_MMD_VEND1 (30) and MII_MMD_VEND2 (31); this driver uses those
 * directly instead of redefining them here.
 */

/* Vendor registers (MMD31) */

#define RTL_VEND2_PHYSR          0xa434  /* PHY specific status register */

/* PHYSR (MMD31.0xa434) field definitions */

#define RTL_PHYSR_LINK_RT        (1 << 2)  /* bit2: real-time link status */
#define RTL_PHYSR_DUPLEX         (1 << 3)  /* bit3: 1=full, 0=half duplex */
#define RTL_PHYSR_SPEED_HI_MASK  0x0600    /* bits[10:9] Speed_h */
#define RTL_PHYSR_SPEED_HI_SHIFT 9
#define RTL_PHYSR_SPEED_LO_MASK  0x0030    /* bits[5:4] Speed_l */
#define RTL_PHYSR_SPEED_LO_SHIFT 4

/* Patch / init related vendor registers (MMD31) */

#define RTL_VEND2_PHY_READY_REG  0xa420    /* bit[1:0]==0x3 => PHY ready */
#define RTL_VEND2_PATCH_REG_SEL  0xa436    /* OCP register selector */
#define RTL_VEND2_PATCH_REG_DATA 0xa438    /* OCP register data */
#define RTL_VEND2_PATCH_VER_SEL  0x801e    /* selector for patch version */
#define RTL_VEND2_PATCH_VERSION  0x0020    /* expected driver patch version */
#define RTL_VEND2_PATCH_CTRL     0xb820    /* patch request/ucode control */
#define RTL_VEND2_PATCH_STATUS   0xb800    /* patch ready status */
#define RTL_VEND2_PATCH_REQ      0x0010    /* B820 bit4: patch request */
#define RTL_VEND2_PATCH_RDY      0x0040    /* B800 bit6: patch ready */
#define RTL_VEND2_UCODE_EN       0x0080    /* B820 bit7: NC/UC ucode enable */

/* PHY reset / restart auto-negotiation (MMD31.0xa400) */

#define RTL_VEND2_PHYRST_REG     0xa400
#define RTL_VEND2_PHYRST_NWAY    0x9200    /* PHY reset + restart Nway */
#define RTL_VEND2_LINK_CHG       0x4000    /* A400 bit14: change link state */

/* PHY interrupt registers (MMD31 / VEND2) */

#define RTL_VEND2_INER_REG       0xa4d2    /* Interrupt enable register   */
#define RTL_VEND2_INSR_REG       0xa4d4    /* Interrupt status (read-clr) */
#define RTL_VEND2_INTR_LINK_CHG  (1 << 2)  /* bit2: link status change    */

/* SerDes registers (MMD30 / VEND1) and option codes */

#define RTL_VEND1_SDS_OPTION     0x697a    /* SerDes mode select [5:0] */
#define RTL_VEND1_SDS_MODE_MASK  0x3f
#define RTL_VEND1_SDS_75F3       0x75f3
#define RTL_VEND1_SDS_CTRL1      0x6a04
#define RTL_VEND1_SDS_6F10       0x6f10
#define RTL_VEND1_SDS_6F11       0x6f11
#define RTL_VEND1_SDS_ANEG_7587  0x7587
#define RTL_VEND1_SDS_ANEG_7588  0x7588
#define RTL_VEND1_SDS_ANEG_7589  0x7589
#define RTL_VEND1_SDS_CTRL3      0x7580    /* SerDes mode status [4:0] */
#define RTL_VEND1_SDS_CTRL7      0x758b    /* SerDes valid status */
#define RTL_VEND1_SDS_CTRL9      0x758d    /* SerDes link status */

/* SerDes option code written into 0x697a[5:0]:
 * 0 = 2500BASE-X + SGMII (fixed, no auto-negotiation), per vendor enum.
 */

#define RTL_SDS_OPTION_2500BASEX_SGMII  0x00

/* SerDes link poll timeout (ms) when waiting for the PHY<->MAC link */

#define RTL_SDS_LINK_TIMEOUT_MS  10000

/* Timeout for PHY-ready polling (in 1ms iterations) */

#define RTL_PHY_READY_TIMEOUT_MS 5000

/* Timeout for patch request/ready handshake polling (in 1ms iterations) */

#define RTL_PATCH_READY_TIMEOUT_MS 1000

#endif /* __DRIVERS_NET_PHY_REALTEK_RTL8221B_H */
