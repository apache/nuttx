/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_netc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_NETC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_NETC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NETC PCI functions and SoC glue. */

#define IMXRT_IOMUXC_GPIO_AD_30_MUX       0x42a10184u
#define IMXRT_IOMUXC_GPIO_AD_31_MUX       0x42a10188u
#define IMXRT_IOMUXC_GPIO_AD_30_PAD       0x42a103ccu
#define IMXRT_IOMUXC_GPIO_AD_31_PAD       0x42a103d0u
#define IMXRT_IOMUXC_NETC_EMDIO_SELECT_INPUT 0x42a10798u

#define IMXRT_BLK_CTRL_WAKEUPMIX_BASE     0x42420000u
#define IMXRT_NETC_LINK0_CFG              (IMXRT_BLK_CTRL_WAKEUPMIX_BASE + 0x100u)
#define IMXRT_NETC_LINK2_CFG              (IMXRT_BLK_CTRL_WAKEUPMIX_BASE + 0x108u)

#define NETC_LINK_CFG_MII_PROTOCOL_MASK   0x0fu
#define NETC_LINK_CFG_MII_PROTOCOL_RGMII  2u

#define IMXRT_NETC_IERB_BASE              0x60800000u
#define IMXRT_NETC_IERB_RCMSIAMQR         (IMXRT_NETC_IERB_BASE + 0x20cu)
#define IMXRT_NETC_IERB_L0BCR             (IMXRT_NETC_IERB_BASE + 0x1010u)
#define IMXRT_NETC_IERB_L2BCR             (IMXRT_NETC_IERB_BASE + 0x1090u)

#define NETC_IERB_RCMSIAMQR_MSI_MASK      (7u << 27)
#define NETC_IERB_RCMSIAMQR_MSI_CM33      (1u << 27)
#define NETC_IERB_LINK_PHY_ADDRESS(n)      ((uint32_t)(n) & 0x1fu)

#define IMXRT_NETC_PRIV_BASE              0x60900000u
#define IMXRT_NETC_PRIV_NETCRR            (IMXRT_NETC_PRIV_BASE + 0x100u)
#define IMXRT_NETC_PRIV_NETCSR            (IMXRT_NETC_PRIV_BASE + 0x104u)

#define NETC_PRIV_NETCRR_LOCK             (1u << 1)
#define NETC_PRIV_NETCSR_STATE            (1u << 1)

#define IMXRT_NETC_F1_BASE                0x60001000u
#define IMXRT_NETC_F1_COMMAND             (IMXRT_NETC_F1_BASE + 0x0004u)
#define IMXRT_NETC_F1_DEVICE_CONTROL      (IMXRT_NETC_F1_BASE + 0x0048u)

#define IMXRT_NETC_F2_BASE                0x60002000u
#define IMXRT_NETC_F2_COMMAND             (IMXRT_NETC_F2_BASE + 0x0004u)
#define IMXRT_NETC_F2_DEVICE_CONTROL      (IMXRT_NETC_F2_BASE + 0x0048u)

#define IMXRT_NETC_F4_BASE                0x60004000u
#define IMXRT_NETC_F4_COMMAND             (IMXRT_NETC_F4_BASE + 0x0004u)
#define IMXRT_NETC_F4_DEVICE_CONTROL      (IMXRT_NETC_F4_BASE + 0x0048u)

#define NETC_PCI_COMMAND_MEMORY            (1u << 1)
#define NETC_PCI_COMMAND_MASTER            (1u << 2)
#define NETC_PCI_DEVICE_CONTROL_FLR        (1u << 15)

/* External MDIO register block. */

#define IMXRT_NETC_EMDIO_BASE             0x60ba0000u
#define IMXRT_NETC_EMDIO_CFG              (IMXRT_NETC_EMDIO_BASE + 0x1c00u)
#define IMXRT_NETC_EMDIO_CONTROL          (IMXRT_NETC_EMDIO_BASE + 0x1c04u)
#define IMXRT_NETC_EMDIO_DATA             (IMXRT_NETC_EMDIO_BASE + 0x1c08u)

#define NETC_EMDIO_CFG_READ_ERROR         (1u << 1)
#define NETC_EMDIO_CFG_HOLD_SHIFT         2
#define NETC_EMDIO_CFG_CLAUSE45           (1u << 6)
#define NETC_EMDIO_CFG_DIV_SHIFT          7
#define NETC_EMDIO_CFG_DIV_MASK           (0x1ffu << NETC_EMDIO_CFG_DIV_SHIFT)
#define NETC_EMDIO_CFG_ADDRESS_ERROR      (1u << 28)
#define NETC_EMDIO_CFG_BUSY               (1u << 31)

#define NETC_EMDIO_CONTROL_REG_SHIFT      0
#define NETC_EMDIO_CONTROL_PHY_SHIFT      5
#define NETC_EMDIO_CONTROL_READ           (1u << 15)

/* ENETC1 PSI0 is the internal CPU-facing endpoint for the NETC switch. */

#define IMXRT_NETC_ENETC1_SI_BASE         0x60b40000u
#define IMXRT_NETC_ENETC1_BASE            0x60b50000u
#define IMXRT_NETC_ENETC1_ECAPR2          (IMXRT_NETC_ENETC1_BASE + 0x0008u)
#define IMXRT_NETC_ENETC1_PMR             (IMXRT_NETC_ENETC1_BASE + 0x0010u)
#define IMXRT_NETC_ENETC1_PSIPMMR         (IMXRT_NETC_ENETC1_BASE + 0x0200u)
#define IMXRT_NETC_ENETC1_PSIPVMR         (IMXRT_NETC_ENETC1_BASE + 0x0204u)
#define IMXRT_NETC_ENETC1_PSIPMAR0        (IMXRT_NETC_ENETC1_BASE + 0x2000u)
#define IMXRT_NETC_ENETC1_PSIPMAR1        (IMXRT_NETC_ENETC1_BASE + 0x2004u)
#define IMXRT_NETC_ENETC1_PSICFGR0        (IMXRT_NETC_ENETC1_BASE + 0x2010u)

#define NETC_ENETC_PMR_SI0EN              (1u << 16)
#define NETC_ENETC_PSICFGR0_TXRINGS(n)    ((uint32_t)(n) & 0x7fu)
#define NETC_ENETC_PSICFGR0_RXRINGS(n)    (((uint32_t)(n) & 0x7fu) << 16)
#define NETC_ENETC_PSIPMMR_SI0_MAC_UP     (1u << 0)
#define NETC_ENETC_PSIPMMR_SI0_MAC_MP     (1u << 16)
#define NETC_ENETC_PSIPVMR_SI0_VLAN_P     (1u << 0)
#define NETC_ENETC_PSIPVMR_SI0_VLAN_UTA   (1u << 16)

#define IMXRT_NETC_SI_SIMR                (IMXRT_NETC_ENETC1_SI_BASE + 0x0000u)
#define IMXRT_NETC_SI_SIRBGCR             (IMXRT_NETC_ENETC1_SI_BASE + 0x0038u)
#define IMXRT_NETC_SI_BDR_BASE(n)         \
  (IMXRT_NETC_ENETC1_SI_BASE + 0x8000u + ((uintptr_t)(n) * 0x200u))
#define IMXRT_NETC_SI_TBMR(n)             (IMXRT_NETC_SI_BDR_BASE(n) + 0x000u)
#define IMXRT_NETC_SI_TBBAR0(n)           (IMXRT_NETC_SI_BDR_BASE(n) + 0x010u)
#define IMXRT_NETC_SI_TBBAR1(n)           (IMXRT_NETC_SI_BDR_BASE(n) + 0x014u)
#define IMXRT_NETC_SI_TBPIR(n)            (IMXRT_NETC_SI_BDR_BASE(n) + 0x018u)
#define IMXRT_NETC_SI_TBCIR(n)            (IMXRT_NETC_SI_BDR_BASE(n) + 0x01cu)
#define IMXRT_NETC_SI_TBLENR(n)           (IMXRT_NETC_SI_BDR_BASE(n) + 0x020u)
#define IMXRT_NETC_SI_TBIER(n)            (IMXRT_NETC_SI_BDR_BASE(n) + 0x0a0u)
#define IMXRT_NETC_SI_RBMR(n)             (IMXRT_NETC_SI_BDR_BASE(n) + 0x100u)
#define IMXRT_NETC_SI_RBBSR(n)            (IMXRT_NETC_SI_BDR_BASE(n) + 0x108u)
#define IMXRT_NETC_SI_RBCIR(n)            (IMXRT_NETC_SI_BDR_BASE(n) + 0x10cu)
#define IMXRT_NETC_SI_RBBAR0(n)           (IMXRT_NETC_SI_BDR_BASE(n) + 0x110u)
#define IMXRT_NETC_SI_RBBAR1(n)           (IMXRT_NETC_SI_BDR_BASE(n) + 0x114u)
#define IMXRT_NETC_SI_RBPIR(n)            (IMXRT_NETC_SI_BDR_BASE(n) + 0x118u)
#define IMXRT_NETC_SI_RBLENR(n)           (IMXRT_NETC_SI_BDR_BASE(n) + 0x120u)
#define IMXRT_NETC_SI_RBIER(n)            (IMXRT_NETC_SI_BDR_BASE(n) + 0x1a0u)

#define NETC_SI_SIMR_ENABLE               (1u << 31)

/* SIRBGCR.RINGS_PER_GROUP is encoded as (rings - 1). */

#define NETC_SI_SIRBGCR_NUM_GROUPS(n)     ((uint32_t)(n) & 0x3u)
#define NETC_SI_SIRBGCR_RINGS_PER_GROUP(n) \
  ((((uint32_t)(n) - 1u) & 0x7u) << 16)
#define NETC_SI_TBMR_ENABLE               (1u << 31)
#define NETC_SI_RBMR_ENABLE               (1u << 31)

/* Switch core and ports.  Port 4 is the internal ENETC1 conduit. */

#define IMXRT_NETC_SWITCH_BASE            0x60a00000u
#define IMXRT_NETC_SWITCH_VFHTDECR0       (IMXRT_NETC_SWITCH_BASE + 0x2010u)
#define IMXRT_NETC_SWITCH_VFHTDECR1       (IMXRT_NETC_SWITCH_BASE + 0x2014u)
#define IMXRT_NETC_SWITCH_VFHTDECR2       (IMXRT_NETC_SWITCH_BASE + 0x2018u)

#define NETC_SWITCH_PORT_BASE(n)          \
  (IMXRT_NETC_SWITCH_BASE + (((uintptr_t)(n) + 1u) * 0x4000u))
#define NETC_SWITCH_PORT_ETH_BASE(n)      (NETC_SWITCH_PORT_BASE(n) + 0x1000u)

#define NETC_SWITCH_PORT_PCR(n)           (NETC_SWITCH_PORT_BASE(n) + 0x0010u)
#define NETC_SWITCH_PORT_PMAR0(n)         (NETC_SWITCH_PORT_BASE(n) + 0x0020u)
#define NETC_SWITCH_PORT_PMAR1(n)         (NETC_SWITCH_PORT_BASE(n) + 0x0024u)
#define NETC_SWITCH_PORT_PSGCR(n)         (NETC_SWITCH_PORT_BASE(n) + 0x00a0u)
#define NETC_SWITCH_PORT_POR(n)           (NETC_SWITCH_PORT_BASE(n) + 0x0100u)
#define NETC_SWITCH_PORT_PRXSDUOR(n)      (NETC_SWITCH_PORT_BASE(n) + 0x0108u)
#define NETC_SWITCH_PORT_PTXSDUOR(n)      (NETC_SWITCH_PORT_BASE(n) + 0x010cu)
#define NETC_SWITCH_PORT_PTGSCR(n)        (NETC_SWITCH_PORT_BASE(n) + 0x0110u)
#define NETC_SWITCH_PORT_PDGSR(n)         (NETC_SWITCH_PORT_BASE(n) + 0x0138u)
#define NETC_SWITCH_PORT_RX_DISCARD(n)    (NETC_SWITCH_PORT_BASE(n) + 0x01c0u)
#define NETC_SWITCH_PORT_RX_DISCARD_R0(n) (NETC_SWITCH_PORT_BASE(n) + 0x01c8u)
#define NETC_SWITCH_PORT_RX_DISCARD_R1(n) (NETC_SWITCH_PORT_BASE(n) + 0x01ccu)
#define NETC_SWITCH_PORT_TX_DISCARD(n)    (NETC_SWITCH_PORT_BASE(n) + 0x01e0u)
#define NETC_SWITCH_PORT_TX_DISCARD_R0(n) (NETC_SWITCH_PORT_BASE(n) + 0x01e8u)
#define NETC_SWITCH_PORT_TX_DISCARD_R1(n) (NETC_SWITCH_PORT_BASE(n) + 0x01ecu)
#define NETC_SWITCH_PORT_PTCTMSDUR(n, tc) \
  (NETC_SWITCH_PORT_BASE(n) + 0x0208u + ((uintptr_t)(tc) * 0x20u))
#define NETC_SWITCH_PORT_BPCR(n)          (NETC_SWITCH_PORT_BASE(n) + 0x0500u)
#define NETC_SWITCH_PORT_BPDVR(n)         (NETC_SWITCH_PORT_BASE(n) + 0x0510u)
#define NETC_SWITCH_PORT_BPSTGSR(n)       (NETC_SWITCH_PORT_BASE(n) + 0x0520u)
#define NETC_SWITCH_PORT_BPSCR0(n)        (NETC_SWITCH_PORT_BASE(n) + 0x0528u)
#define NETC_SWITCH_PORT_BPSCR1(n)        (NETC_SWITCH_PORT_BASE(n) + 0x052cu)
#define NETC_SWITCH_PORT_PCTFCR(n)        (NETC_SWITCH_PORT_BASE(n) + 0x0450u)
#define NETC_SWITCH_PORT_BRIDGE_DISCARD(n) \
  (NETC_SWITCH_PORT_BASE(n) + 0x0580u)
#define NETC_SWITCH_PORT_BRIDGE_DISCARD_R0(n) \
  (NETC_SWITCH_PORT_BASE(n) + 0x0588u)
#define NETC_SWITCH_PORT_BRIDGE_DISCARD_R1(n) \
  (NETC_SWITCH_PORT_BASE(n) + 0x058cu)

#define NETC_SWITCH_PORT_COMMAND(n)       (NETC_SWITCH_PORT_ETH_BASE(n) + 0x0008u)
#define NETC_SWITCH_PORT_MAXFRM(n)        (NETC_SWITCH_PORT_ETH_BASE(n) + 0x0014u)
#define NETC_SWITCH_PORT_COMMAND_PM1(n)   (NETC_SWITCH_PORT_ETH_BASE(n) + 0x0408u)
#define NETC_SWITCH_PORT_MAXFRM_PM1(n)    (NETC_SWITCH_PORT_ETH_BASE(n) + 0x0414u)
#define NETC_SWITCH_PORT_RX_PACKETS(n)    (NETC_SWITCH_PORT_ETH_BASE(n) + 0x0160u)
#define NETC_SWITCH_PORT_TX_PACKETS(n)    (NETC_SWITCH_PORT_ETH_BASE(n) + 0x0260u)
#define NETC_SWITCH_PSEUDO_RX_BCAST(n)    (NETC_SWITCH_PORT_ETH_BASE(n) + 0x0098u)
#define NETC_SWITCH_PORT_IF_MODE(n)       (NETC_SWITCH_PORT_ETH_BASE(n) + 0x0300u)
#define NETC_SWITCH_PORT_IF_MODE_PM1(n)   (NETC_SWITCH_PORT_ETH_BASE(n) + 0x0700u)

/* FRDM: only ports 0, 2, and internal 4 are present
 * (Zephyr DSA membership).
 */
#define NETC_SWITCH_VLAN_PORTS_MASK       0x15u
#define NETC_SWITCH_FDB_LOOKUP_FLOOD      (2u << 27)
#define NETC_SWITCH_HW_MAC_LEARN          (2u << 24)
#define NETC_SWITCH_STG0_FORWARD          2u
#define NETC_SWITCH_STORM_ENTRY_DISABLE   0x0fff0fffu
#define NETC_SWITCH_PORT_SPEED_1G         (0x63u << 16)
#define NETC_SWITCH_PORT_ACCEPT_ALL_TAGS  (0x0fu << 20)
#define NETC_SWITCH_PORT_RXVAM            (1u << 24)
#define NETC_SWITCH_PORT_TC_GATES_OPEN    0xffu
#define NETC_SWITCH_PORT_TIME_GATE_ENABLE (1u << 31)
#define NETC_SWITCH_PORT_CUT_THROUGH_OFF  0x3u
/* PTCTMSDUR: measure the MPDU and disable the max-SDU check on
 * store-and-forward frames.  Cut-through frames are always checked.
 */

#define NETC_SWITCH_PORT_TC_SDU_TYPE_MPDU (1u << 16)
#define NETC_SWITCH_PORT_TC_SF_MAXSDU_DIS (1u << 24)
#define NETC_SWITCH_PORT_TC_MPDU_MAX      \
  (NETC_SWITCH_PORT_TC_SF_MAXSDU_DIS | \
   NETC_SWITCH_PORT_TC_SDU_TYPE_MPDU | 0x600u)
#define NETC_SWITCH_PORT_TXRX_DISABLE     0x3u
#define NETC_SWITCH_PORT_PPDU_BCO         20u

#define NETC_SWITCH_MAC_TX_ENABLE         (1u << 0)
#define NETC_SWITCH_MAC_RX_ENABLE         (1u << 1)
#define NETC_SWITCH_MAC_CNT_FRM_EN        (1u << 13)
#define NETC_SWITCH_MAC_TX_PAD            (1u << 15)
#define NETC_SWITCH_MAC_SWR               (1u << 26)
#define NETC_SWITCH_MAC_MAX_FRAME         0x600u
#define NETC_SWITCH_IFMODE_RGMII          0x4u
#define NETC_SWITCH_IFMODE_HALF_DUPLEX    (1u << 6)
#define NETC_SWITCH_IFMODE_SPEED_SHIFT    13
#define NETC_SWITCH_IFMODE_SPEED_MASK     (3u << NETC_SWITCH_IFMODE_SPEED_SHIFT)
#define NETC_SWITCH_IFMODE_SPEED_10       (0u << NETC_SWITCH_IFMODE_SPEED_SHIFT)
#define NETC_SWITCH_IFMODE_SPEED_100      (1u << NETC_SWITCH_IFMODE_SPEED_SHIFT)
#define NETC_SWITCH_IFMODE_SPEED_1000     (2u << NETC_SWITCH_IFMODE_SPEED_SHIFT)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_NETC_H */
