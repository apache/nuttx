/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_ethernet.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_ETHERNET_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Content of this file requires verification before it is used with other
 * families
 */

#if defined(CONFIG_STM32H7_STM32H7X3XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* MAC Registers */

#define STM32_ETH_MACCR_OFFSET       0x0000 /* Ethernet MAC configuration register */
#define STM32_ETH_MAECR_OFFSET       0x0004 /* Ethernet MAC Extended operating mode configuration register */
#define STM32_ETH_MACPFR_OFFSET      0x0008 /* Ethernet MAC Packet filtering control register */
#define STM32_ETH_MACWTR_OFFSET      0x000C /* Ethernet MAC Watchdog timeout register */
#define STM32_ETH_MACHT0R_OFFSET     0x0010 /* Ethernet MAC hash table high register */
#define STM32_ETH_MACHT1R_OFFSET     0x0014 /* Ethernet MAC hash table low register */
#define STM32_ETH_MACVTR_OFFSET      0x0050 /* Ethernet MAC VLAN tag register */
#define STM32_ETH_MACQTXFCR_OFFSET   0x0070 /* Ethernet MAC Tx Queue flow control register */
#define STM32_ETH_MACRXFCR_OFFSET    0x0090 /* Ethernet MAC Tx Queue flow control register */

#define STM32_ETH_MACISR_OFFSET      0x00B0 /* Ethernet MAC interrupt status register */
#define STM32_ETH_MACIER_OFFSET      0x00B4 /* Ethernet MAC interrupt enable register */

#define STM32_ETH_MACMDIOAR_OFFSET   0x0200 /* Ethernet MAC MDIO address register */
#define STM32_ETH_MACMDIODR_OFFSET   0x0204 /* Ethernet MAC MDIO data register */
#define STM32_ETH_MACA0HR_OFFSET     0x0300 /* Address 0 high register */
#define STM32_ETH_MACA0LR_OFFSET     0x0304 /* Address 0 low register */

/* MTL Registers */

#define STM32_ETH_MTLOMR_OFFSET      0x0C00 /* Operating mode Register */
#define STM32_ETH_MTLISR_OFFSET      0x0C20 /* Interrupt status Register */
#define STM32_ETH_MTLTXQOMR_OFFSET   0x0D00 /* Tx queue operating mode Register */
#define STM32_ETH_MTLTXQUR_OFFSET    0x0D04 /* Tx queue underflow register */
#define STM32_ETH_MTLTXQDR_OFFSET    0x0D08 /* Tx queue debug register */
#define STM32_ETH_MTLQICSR_OFFSET    0x0D2C /* Queue interrupt control status Register */
#define STM32_ETH_MTLRXQOMR_OFFSET   0x0D30 /* Rx queue operating mode register */
#define STM32_ETH_MTLRXQMPOCR_OFFSET 0x0D34 /* Rx queue missed packet and overflow counter register */
#define STM32_ETH_MTLRXQDR_OFFSE     0x0D38 /* Rx queue debug register */

/* DMA Registers */

#define STM32_ETH_DMAMR_OFFSET       0x1000 /* Ethernet DMA Mode register */
#define STM32_ETH_DMASBMR_OFFSET     0x1004 /* Ethernet DMA system bus mode register */
#define STM32_ETH_DMAISR_OFFSET      0x1008 /* Ethernet DMA Interrupt status register */
#define STM32_ETH_DMADSR_OFFSET      0x100C /* Ethernet DMA Debug status register */
#define STM32_ETH_DMACCR_OFFSET      0x1100 /* Ethernet DMA Channel control register */
#define STM32_ETH_DMACTXCR_OFFSET    0x1104 /* Ethernet DMA Channel transmit control register */
#define STM32_ETH_DMACRXCR_OFFSET    0x1108 /* Ethernet DMA Channel receive control register */
#define STM32_ETH_DMACTXDLAR_OFFSET  0x1114 /* Ethernet DMA Channel Tx descriptor list address register */
#define STM32_ETH_DMACRXDLAR_OFFSET  0x111C /* Ethernet DMA Channel Rx descriptor list address register */
#define STM32_ETH_DMACTXDTPR_OFFSET  0x1120 /* Ethernet DMA Channel Tx descriptor tail pointer register */
#define STM32_ETH_DMACRXDTPR_OFFSET  0x1128 /* Ethernet DMA Channel Rx descriptor tail pointer register */
#define STM32_ETH_DMACTXRLR_OFFSET   0x112C /* Ethernet DMA Channel Tx descriptor ring length register */
#define STM32_ETH_DMACRXRLR_OFFSET   0x1130 /* Ethernet DMA Channel Rx descriptor ring length register */
#define STM32_ETH_DMACIER_OFFSET     0x1134 /* Ethernet DMA Channel interrupt enable register */
#define STM32_ETH_DMACRXIWTR_OFFSET  0x1138 /* Ethernet DMA Channel Rx interrupt wdt register */
#define STM32_ETH_DMACCATXDR_OFFSET  0x1144 /* Ethernet DMA Channel current application transmit descriptor register */
#define STM32_ETH_DMACCARXDR_OFFSET  0x114C /* Ethernet DMA Channel current application receive descriptor register */
#define STM32_ETH_DMACCATXBR_OFFSET  0x1154 /* Ethernet DMA Channel current application transmit buffer register */
#define STM32_ETH_DMACCARXBR_OFFSET  0x115C /* Ethernet DMA Channel current application receive buffer register */
#define STM32_ETH_DMACSR_OFFSET      0x1160 /* Ethernet DMA Channel status register */
#define STM32_ETH_DMACMFCR_OFFSET    0x116C /* Ethernet DMA Channel missed frame count register */

/* Register Base Addresses **************************************************/

/* MAC Registers */

#define STM32_ETH_MACCR              (STM32_EMAC_BASE+STM32_ETH_MACCR_OFFSET)
#define STM32_ETH_MACPFR             (STM32_EMAC_BASE+STM32_ETH_MACPFR_OFFSET)
#define STM32_ETH_MACHT0R            (STM32_EMAC_BASE+STM32_ETH_MACHT0R_OFFSET)
#define STM32_ETH_MACHT1R            (STM32_EMAC_BASE+STM32_ETH_MACHT1R_OFFSET)
#define STM32_ETH_MACVTR             (STM32_EMAC_BASE+STM32_ETH_MACVTR_OFFSET)
#define STM32_ETH_MACQTXFCR          (STM32_EMAC_BASE+STM32_ETH_MACQTXFCR_OFFSET)
#define STM32_ETH_MACRXFCR           (STM32_EMAC_BASE+STM32_ETH_MACRXFCR_OFFSET)
#define STM32_ETH_MACISR             (STM32_EMAC_BASE+STM32_ETH_MACISR_OFFSET)
#define STM32_ETH_MACIER             (STM32_EMAC_BASE+STM32_ETH_MACIER_OFFSET)

#define STM32_ETH_MACMDIOAR          (STM32_EMAC_BASE+STM32_ETH_MACMDIOAR_OFFSET)
#define STM32_ETH_MACMDIODR          (STM32_EMAC_BASE+STM32_ETH_MACMDIODR_OFFSET)
#define STM32_ETH_MACA0LR            (STM32_EMAC_BASE+STM32_ETH_MACA0LR_OFFSET)
#define STM32_ETH_MACA0HR            (STM32_EMAC_BASE+STM32_ETH_MACA0HR_OFFSET)

/* MTL Registers */

#define STM32_ETH_MTLOMR             (STM32_EMAC_BASE+STM32_ETH_MTLOMR_OFFSET)
#define STM32_ETH_MTLISR             (STM32_EMAC_BASE+STM32_ETH_MTLISR_OFFSET)
#define STM32_ETH_MTLTXQOMR          (STM32_EMAC_BASE+STM32_ETH_MTLTXQOMR_OFFSET)
#define STM32_ETH_MTLTXQUR           (STM32_EMAC_BASE+STM32_ETH_MTLTXQUR_OFFSET)
#define STM32_ETH_MTLTXQDR           (STM32_EMAC_BASE+STM32_ETH_MTLTXQDR_OFFSET)
#define STM32_ETH_MTLQICSR           (STM32_EMAC_BASE+STM32_ETH_MTLQICSR_OFFSET)
#define STM32_ETH_MTLRXQOMR          (STM32_EMAC_BASE+STM32_ETH_MTLRXQOMR_OFFSET)
#define STM32_ETH_MTLRXQMPOCR        (STM32_EMAC_BASE+STM32_ETH_MTLRXQMPOCR_OFFSET)
#define STM32_ETH_MTLRXQDR           (STM32_EMAC_BASE+STM32_ETH_MTLRXQDR_OFFSE)

/* DMA Registers */

#define STM32_ETH_DMAMR              (STM32_EMAC_BASE+STM32_ETH_DMAMR_OFFSET)
#define STM32_ETH_DMASBMR            (STM32_EMAC_BASE+STM32_ETH_DMASBMR_OFFSET)
#define STM32_ETH_DMAISR             (STM32_EMAC_BASE+STM32_ETH_DMAISR_OFFSET)
#define STM32_ETH_DMADSR             (STM32_EMAC_BASE+STM32_ETH_DMADSR_OFFSET)
#define STM32_ETH_DMACCR             (STM32_EMAC_BASE+STM32_ETH_DMACCR_OFFSET)
#define STM32_ETH_DMACTXCR           (STM32_EMAC_BASE+STM32_ETH_DMACTXCR_OFFSET)
#define STM32_ETH_DMACRXCR           (STM32_EMAC_BASE+STM32_ETH_DMACRXCR_OFFSET)
#define STM32_ETH_DMACTXDLAR         (STM32_EMAC_BASE+STM32_ETH_DMACTXDLAR_OFFSET)
#define STM32_ETH_DMACRXDLAR         (STM32_EMAC_BASE+STM32_ETH_DMACRXDLAR_OFFSET)
#define STM32_ETH_DMACTXDTPR         (STM32_EMAC_BASE+STM32_ETH_DMACTXDTPR_OFFSET)
#define STM32_ETH_DMACRXDTPR         (STM32_EMAC_BASE+STM32_ETH_DMACRXDTPR_OFFSET)
#define STM32_ETH_DMACTXRLR          (STM32_EMAC_BASE+STM32_ETH_DMACTXRLR_OFFSET)
#define STM32_ETH_DMACRXRLR          (STM32_EMAC_BASE+STM32_ETH_DMACRXRLR_OFFSET)
#define STM32_ETH_DMACIER            (STM32_EMAC_BASE+STM32_ETH_DMACIER_OFFSET)
#define STM32_ETH_DMACRXIWTR         (STM32_EMAC_BASE+STM32_ETH_DMACRXIWTR_OFFSET)
#define STM32_ETH_DMACCATXDR         (STM32_EMAC_BASE+STM32_ETH_DMACCATXDR_OFFSET)
#define STM32_ETH_DMACCARXDR         (STM32_EMAC_BASE+STM32_ETH_DMACCARXDR_OFFSET)
#define STM32_ETH_DMACCATXBR         (STM32_EMAC_BASE+STM32_ETH_DMACCATXBR_OFFSET)
#define STM32_ETH_DMACCARXBR         (STM32_EMAC_BASE+STM32_ETH_DMACCARXBR_OFFSET)
#define STM32_ETH_DMACSR             (STM32_EMAC_BASE+STM32_ETH_DMACSR_OFFSET)
#define STM32_ETH_DMACMFCR           (STM32_EMAC_BASE+STM32_ETH_DMACMFCR_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* MAC Registers */

/* Ethernet MAC configuration register */

#define ETH_MACCR_RE                 (1 << 0)  /* Bit 0:  Receiver enable */
#define ETH_MACCR_TE                 (1 << 1)  /* Bit 1:  Transmitter enable */
#define ETH_MACCR_PRELEN_SHIFT       (2)       /* Bits 2-3: Preamble length for transmit packets */
#define ETH_MACCR_PRELEN_MASK        (3 << ETH_MACCR_PRELEN_SHIFT)
#  define ETH_MACCR_PRELEN_7         (0 << ETH_MACCR_PRELEN_SHIFT) /* 00: 7 bytes of preamble */
#  define ETH_MACCR_PRELEN_5         (1 << ETH_MACCR_PRELEN_SHIFT) /* 01: 5 bytes of preamble */
#  define ETH_MACCR_PRELEN_3         (2 << ETH_MACCR_PRELEN_SHIFT) /* 10: 3 bytes of preamble */

#define ETH_MACCR_DC                 (1 << 4)  /* Bit 4:  Deferral check */
#define ETH_MACCR_BL_SHIFT           (5)       /* Bits 5-6: Back-off limit */
#define ETH_MACCR_BL_MASK            (3 << ETH_MACCR_BL_SHIFT)
#  define ETH_MACCR_BL_10            (0 << ETH_MACCR_BL_SHIFT) /* 00: k = min (n, 10) */
#  define ETH_MACCR_BL_8             (1 << ETH_MACCR_BL_SHIFT) /* 01: k = min (n, 8) */
#  define ETH_MACCR_BL_4             (2 << ETH_MACCR_BL_SHIFT) /* 10: k = min (n, 4) */
#  define ETH_MACCR_BL_1             (3 << ETH_MACCR_BL_SHIFT) /* 11: k = min (n, 1) */

#define ETH_MACCR_DR                 (1 << 8)  /* Bit 8:  Retry disable */
#define ETH_MACCR_DCRS               (1 << 9)  /* Bit 9: Carrier sense disable */
#define ETH_MACCR_DO                 (1 << 10) /* Bit 10: Disable receive own */
#define ETH_MACCR_LM                 (1 << 12) /* Bit 12: Loopback mode */
#define ETH_MACCR_DM                 (1 << 13) /* Bit 13: Duplex mode */
#define ETH_MACCR_FES                (1 << 14) /* Bit 14: Fast Ethernet speed */
#define ETH_MACCR_JE                 (1 << 16) /* Bit 16: Jumbo packet enable */
#define ETH_MACCR_JD                 (1 << 17) /* Bit 17: Jabber disable */
#define ETH_MACCR_WD                 (1 << 19) /* Bit 19: Watchdog disable */
#define ETH_MACCR_ACS                (1 << 20) /* Bit 20:  Automatic pad/CRC stripping */
#define ETH_MACCR_CST                (1 << 21) /* Bit 21: CRC stripping for Type frames */
#define ETH_MACCR_S2KP               (1 << 22) /* Bit 22: IEEE 802.3as Support for 2K Packets */
#define ETH_MACCR_GPSLCE             (1 << 23) /* Bit 23: Giant Packet Size Limit Control Enable */
#define ETH_MACCR_IPG_SHIFT          (24)      /* Bits 24-26: Inter-packet gap */
#define ETH_MACCR_IPG_MASK           (7 << ETH_MACCR_IPG_SHIFT)
#  define ETH_MACCR_IPG(n)           ((12-((n) >> 3)) << ETH_MACCR_IPG_SHIFT) /* n bit times, n=40,48,..96 */

#define ETH_MACCR_IPC                (1 << 27) /* Bit 27: IPv4 checksum offload */
#define ETH_MACCR_SARC_SHIFT         (28)      /* Bits 28-30: Source Address Insertion or Replacement Control */
#define ETH_MACCR_SARC_MASK          (7 << ETH_MACCR_SARC_SHIFT)
#define ETH_MACCR_ARPEN              (1 << 31) /* Bit 31: ARP Offload Enable */

/* Ethernet MAC frame filter register */

#define ETH_MACPFR_PM                (1 << 0)  /* Bit 0: Promiscuous mode */
#define ETH_MACPFR_HUC               (1 << 1)  /* Bit 1: Hash unicast */
#define ETH_MACPFR_HMC               (1 << 2)  /* Bit 2: Hash multicast */
#define ETH_MACPFR_DAIF              (1 << 3)  /* Bit 3: Destination address inverse filtering */
#define ETH_MACPFR_PAM               (1 << 4)  /* Bit 4: Pass all multicast */
#define ETH_MACPFR_DBF               (1 << 5)  /* Bit 5: Disable broadcast packets */
#define ETH_MACPFR_PCF_SHIFT         (6)       /* Bits 6-7: Pass control frames */
#define ETH_MACPFR_PCF_MASK          (3 << ETH_MACPFR_PCF_SHIFT)
#  define ETH_MACPFR_PCF_NONE        (0 << ETH_MACPFR_PCF_SHIFT) /* Prevents all control frames */
#  define ETH_MACPFR_PCF_PAUSE       (1 << ETH_MACPFR_PCF_SHIFT) /* Prevents all except Pause control frames */
#  define ETH_MACPFR_PCF_ALL         (2 << ETH_MACPFR_PCF_SHIFT) /* Forwards all control frames */
#  define ETH_MACPFR_PCF_FILTER      (3 << ETH_MACPFR_PCF_SHIFT) /* Forwards all that pass address filter */

#define ETH_MACPFR_SAIF              (1 << 8)  /* Bit 8: Source address inverse filtering */
#define ETH_MACPFR_SAF               (1 << 9)  /* Bit 9: Source address filter */
#define ETH_MACPFR_HPF               (1 << 10) /* Bit 10: Hash or perfect filter */
#define ETH_MACPFR_VTFE              (1 << 16) /* Bit 16: VLAN Tag Filter Enable */
#define ETH_MACPFR_IPFE              (1 << 20) /* Bit 20: Layer 3 and Layer 4 Filter Enable */
#define ETH_MACPFR_DNTU              (1 << 21) /* Bit 21: Drop Non-TCP/UDP over IP Packets */
#define ETH_MACPFR_RA                (1 << 31) /* Bit 31: Receive all */

/* Ethernet MAC VLAN tag register */

#define ETH_MACVTR_VL_SHIFT     (0)       /* Bits 0-15: VLAN tag identifier (for receive frames) */
#define ETH_MACVTR_VL_MASK      (0xffff << ETH_MACVLANTR_VL_SHIFT)
#  define ETH_MACVTR_VL(n)      ((uint32_t)(n) << ETH_MACVLANTR_VL_SHIFT)
#define ETH_MACVLANTR_ETV       (1 << 16) /* Bit 16: Enable 12-Bit VLAN Tag Comparison */

/* Ethernet MAC Tx Queue flow control register */

#define ETH_MACQTXFCR_FCB_BPA           (1 << 0)  /* Bit 0: Flow control busy/back pressure activate */
#define ETH_MACQTXFCR_TFE               (1 << 1)  /* Bit 1: Transmit flow control enable */
#define ETH_MACQTXFCR_PLT_SHIFT         (4)       /* Bits 4-6: Pause low threshold */
#define ETH_MACQTXFCR_PLT_MASK          (7 << ETH_MACQTXFCR_PLT_SHIFT)
#  define ETH_MACQTXFCR_PLT_M4          (0 << ETH_MACQTXFCR_PLT_SHIFT) /* 000 PT - 4 slot times */
#  define ETH_MACQTXFCR_PLT_M28         (1 << ETH_MACQTXFCR_PLT_SHIFT) /* 001 PT - 28 slot times */
#  define ETH_MACQTXFCR_PLT_M36         (2 << ETH_MACQTXFCR_PLT_SHIFT) /* 010 PT - 36 slot times */
#  define ETH_MACQTXFCR_PLT_M144        (3 << ETH_MACQTXFCR_PLT_SHIFT) /* 011 PT - 144 slot times */
#  define ETH_MACQTXFCR_PLT_M256        (4 << ETH_MACQTXFCR_PLT_SHIFT) /* 100 PT - 256 slot times */
#  define ETH_MACQTXFCR_PLT_M512        (5 << ETH_MACQTXFCR_PLT_SHIFT) /* 101 PT - 512 slot times */

#define ETH_MACQTXFCR_DZPQ              (1 << 7)  /* Bit 7: Zero-quanta pause disable */
#define ETH_MACQTXFCR_PT_SHIFT          (16)      /* Bits 16-31: Pause Time */
#define ETH_MACQTXFCR_PT_MASK           (0xFFFF << ETH_MACQTXFCR_PT_SHIFT)
#define ETH_MACQTXFCR_PT(n)             ((uint32_t)(n) << ETH_MACQTXFCR_PT_SHIFT)

/* Ethernet MAC Rx flow control register */

#define ETH_MACRXFCR_RFE              (1 << 0)  /* Bit 0: Receive flow control enable */
#define ETH_MACRXFCR_UP               (1 << 1)  /* Bit 1: Unicast pause frame detect */

/* Ethernet MAC interrupt status register */

#define ETH_MACISR_PHYIS               (1 << 3)  /* Bit 3: PHY interrupt */
#define ETH_MACISR_PMTIS               (1 << 4)  /* Bit 4: PMT Interrupt Status */
#define ETH_MACISR_LPIIS               (1 << 5)  /* Bit 5: LPI Interrupt Status */
#define ETH_MACISR_MMCIS               (1 << 8)  /* Bit 8: MMC Interrupt Status */
#define ETH_MACISR_MMCRXIS             (1 << 9)  /* Bit 9: MMC Receive Interrupt Status */
#define ETH_MACISR_MMCTXIS             (1 << 10) /* Bit 10: MMC Transmit Interrupt Status */
#define ETH_MACISR_TSIS                (1 << 12) /* Bit 12: Timestamp Interrupt Status */
#define ETH_MACISR_TXTSIS              (1 << 13) /* Bit 13: Transmit Status Interrupt */
#define ETH_MACISR_RXSTSIS             (1 << 14) /* Bit 14: Receive Status Interrupt */

/* Ethernet MAC interrupt mask register */

#define ETH_MACIER_PHYIE             (1 << 3)  /* Bit 3: PHY interrupt mask */
#define ETH_MACIER_PMTIE             (1 << 4)  /* Bit 4: PMT Interrupt Enable */
#define ETH_MACIER_LPIIE             (1 << 5)  /* Bit 5: LPI Interrupt Enable */
#define ETH_MACIER_TSIE              (1 << 12) /* Bit 12: Timestamp Interrupt Enable */
#define ETH_MACIER_TXSTSIE           (1 << 13) /* Bit 13: Transmit Status Interrupt Enable */
#define ETH_MACIER_RXSTSIE           (1 << 14) /* Bit 14: Receive Status Interrupt Enable */
#define ETH_MACIER_ALLINTS           (ETH_MACIER_PHYIE | ETH_MACIER_PMTIE | ETH_MACIER_LPIIE |\
                                      ETH_MACIER_TSIE | ETH_MACIER_TXSTSIE | ETH_MACIER_RXSTSIE)

/* Ethernet MAC MDIO address register */

#define ETH_MACMDIOAR_MB              (1 << 0)  /* Bit 0: MII busy */
#define ETH_MACMDIOAR_C45E            (1 << 1)  /* Bit 1: Clause 45 PHY Enable */
#define ETH_MACMDIOAR_GOC_SHIFT       (2)       /* Bits 2-3: MII Operation Command */

#  define ETH_MACMDIOAR_GOC_WRITE     (1 << ETH_MACMDIOAR_GOC_SHIFT) /* Write */
#  define ETH_MACMDIOAR_GOC_PRIA      (2 << ETH_MACMDIOAR_GOC_SHIFT) /* Post Read Increment Address for Clause 45 PHY */
#  define ETH_MACMDIOAR_GOC_READ      (3 << ETH_MACMDIOAR_GOC_SHIFT) /* Read */

#define ETH_MACMDIOAR_SKAP            (1 << 4)  /* Bit 4: Skip Address Packet */
#define ETH_MACMDIOAR_CR_SHIFT        (8)       /* Bits 8-11: Clock range */
#define ETH_MACMDIOAR_CR_MASK         (15 << ETH_MACMDIOAR_CR_SHIFT)
#  define ETH_MACMDIOAR_CR_DIV42      (0 << ETH_MACMDIOAR_CR_SHIFT) /* 60-100  MHz HCLK/42 */
#  define ETH_MACMDIOAR_CR_DIV62      (1 << ETH_MACMDIOAR_CR_SHIFT) /* 100-150 MHz HCLK/62 */
#  define ETH_MACMDIOAR_CR_DIV16      (2 << ETH_MACMDIOAR_CR_SHIFT) /* 20-35   MHz HCLK/16 */
#  define ETH_MACMDIOAR_CR_DIV26      (3 << ETH_MACMDIOAR_CR_SHIFT) /* 35-60   MHz HCLK/26 */
#  define ETH_MACMDIOAR_CR_DIV102     (4 << ETH_MACMDIOAR_CR_SHIFT) /* 150-250 MHz HCLK/102 */
#  define ETH_MACMDIOAR_CR_DIV124     (5 << ETH_MACMDIOAR_CR_SHIFT) /* 250-300 MHz HCLK/124 */

#define ETH_MACMDIOAR_NTC_SHIFT       (12)       /* Bits 12-14: Number of Training Clocks */
#define ETH_MACMDIOAR_NTC_MASK        (7 << ETH_MACMDIOAR_NTC_SHIFT)
#  define ETH_MACMDIOAR_NTC(n)        ((uint32_t)(n) << ETH_MACMDIOAR_NTC_SHIFT)
#define ETH_MACMDIOAR_RDA_SHIFT       (16)       /* Bits 16-20: MII register */
#define ETH_MACMDIOAR_RDA_MASK        (31 << ETH_MACMDIOAR_RDA_SHIFT)
#  define ETH_MACMDIOAR_RDA(n)        ((uint32_t)(n) << ETH_MACMDIOAR_RDA_SHIFT)
#define ETH_MACMDIOAR_PA_SHIFT        (21)      /* Bits 21-25: PHY address */
#define ETH_MACMDIOAR_PA_MASK         (31 << ETH_MACMDIOAR_PA_SHIFT)
#  define ETH_MACMDIOAR_PA(n)         ((uint32_t)(n) << ETH_MACMDIOAR_PA_SHIFT)
#define ETH_MACMDIOAR_BTB             (1 << 26)  /* Bit 26: Back to Back transactions */
#define ETH_MACMDIOAR_PSE             (1 << 27)  /* Bit 27: Preamble Suppression Enable */

#define STM32_ETH_MACAxHR_MBC_SHIFT   (24) /* Bits 24-29: Mask Byte Control */
#define STM32_ETH_MACAxHR_MBC_MASK    (0x3f << STM32_ETH_MACA0HR_MBC_SHIFT)
#define STM32_ETH_MACAxHR_SA          (1 << 30) /* Bit 30: Source Address */
#define STM32_ETH_MACAxHR_AE          (1 << 31) /* Bit 31: Address Enable */

/* Ethernet MTL registers */

#define ETH_MTLOMR_DTXSTS             (1 << 0)   /* Bit 0: Drop Transmit Status */
#define ETH_MTLOMR_CNTPRST            (1 << 8)   /* Bit 8: Counters Preset */
#define ETH_MTLOMR_CNTCLR             (1 << 9)   /* Bit 9: Counters Reset */

#define ETH_MTLISR_Q0IS               (1 << 0)   /* Bit 0: Queue interrupt status */

#define ETH_MTLTXQOMR_FTQ             (1 << 0)   /* Bit 0: Flush Transmit Queue */
#define ETH_MTLTXQOMR_TSF             (1 << 1)   /* Bit 1: Transmit Store and Forward */
#define ETH_MTLTXQOMR_TXQEN_SHIFT     (2)        /* Bits 2-3: Transmit Queue Enable */
#define ETH_MTLTXQOMR_TXQEN_MASK      (0x3 << ETH_MTLTXQOMR_TXQEN_SHIFT)
#define ETH_MTLTXQOMR_TXQEN_DISABLED  (0)
#define ETH_MTLTXQOMR_TXQEN_ENABLED   (2)

#define ETH_MTLTXQOMR_TTC_SHIFT       (4)        /* Bits 4-6: Transmit Threshold Control */
#define ETH_MTLTXQOMR_TTC_MASK        (0x7 << ETH_MTLTXQOMR_TTC_SHIFT)
#  define ETH_MTLTXQOMR_TTC_32        (0x0 << ETH_MTLTXQOMR_TTC_SHIFT)
#  define ETH_MTLTXQOMR_TTC_64        (0x1 << ETH_MTLTXQOMR_TTC_SHIFT)
#  define ETH_MTLTXQOMR_TTC_96        (0x2 << ETH_MTLTXQOMR_TTC_SHIFT)
#  define ETH_MTLTXQOMR_TTC_128       (0x3 << ETH_MTLTXQOMR_TTC_SHIFT)
#  define ETH_MTLTXQOMR_TTC_192       (0x4 << ETH_MTLTXQOMR_TTC_SHIFT)
#  define ETH_MTLTXQOMR_TTC_256       (0x5 << ETH_MTLTXQOMR_TTC_SHIFT)
#  define ETH_MTLTXQOMR_TTC_364       (0x6 << ETH_MTLTXQOMR_TTC_SHIFT)
#  define ETH_MTLTXQOMR_TTC_512       (0x7 << ETH_MTLTXQOMR_TTC_SHIFT)
#define ETH_MTLTXQOMR_TQS_SHIFT       (16)       /* Bits 16-24: Transmit Queue Size */
#define ETH_MTLTXQOMR_TQS_MASK        (0x1ff << ETH_MTLTXQOMR_TQS_SHIFT)

#define ETH_MTLTXQUR_UFFRMCNT_SHIFT   (0)       /* Underflow Packet Counter */
#define ETH_MTLTXQUR_UFFRMCNT_MASK    (0x7ff << ETH_MTLTXQUR_UFFRMCNT_SHIFT)
#define ETH_MTLTXQUR_UFCNTOVF         (1 << 11)

#define ETH_MTLTXQDR_TXQPAUSED        (1 << 0)  /* Transmit Queue in Pause */
#define ETH_MTLTXQDR_TRCSTS_SHIFT     (1)       /* MTL Tx Queue Read Controller Status */
#define ETH_MTLTXQDR_TRCSTS_MASK      (0x3 << ETH_MTLTXQDR_TRCSTS_SHIFT)
#define ETH_MTLTXQDR_TWCSTS           (1 << 3)  /* Bit 3: MTL Tx Queue Write Controller Status */
#define ETH_MTLTXQDR_TXQSTS           (1 << 4)  /* Bit 4: MTL Tx Queue Not Empty Status */
#define ETH_MTLTXQDR_TXSTSFSTS        (1 << 5)  /* Bit 5: MTL Tx Status FIFO Full Status */
#define ETH_MTLTXQDR_PTXQ_SHIFT       (16)      /* Bits 16-18: Number of Packets in the Transmit Queue */
#define ETH_MTLTXQDR_PTXQ_MASK        (0x7 << ETH_MTLTXQDR_PTXQ_SHIFT)
#define ETH_MTLTXQDR_STXSTSF_SHIFT    (20)      /* Bits 20-22: Number of Status Words in Tx Status FIFO of Queue */
#define ETH_MTLTXQDR_STXSTSF_MASK     (0x7 << ETH_MTLTXQDR_STXSTSF_SHIFT)

#define ETH_MTLQICSR_TXUNFIS    (1 << 0)  /* Bit 0: Transmit Queue Underflow Interrupt Status */
#define ETH_MTLQICSR_TXUIE      (1 << 8)  /* Bit 8: Transmit Queue Underflow Interrupt Enable */
#define ETH_MTLQICSR_RXOVFIS    (1 << 16) /* Bit 16: Receive Queue Overflow Interrupt Status */
#define ETH_MTLQICSR_RXOIE      (1 << 24) /* Bit 24: Receive Queue Overflow Interrupt Enable */

#define ETH_MTLRXQOMR_RTC_SHIFT  (0)      /* Bits 0-1: Receive Queue Threshold Control */
#define ETH_MTLRXQOMR_RTC_MASK   (0x3 << ETH_MTLRXQOMR_RTC_SHIFT)
#  define ETH_MTLRXQOMR_RTC_64   (0x0 << ETH_MTLRXQOMR_RTC_SHIFT)
#  define ETH_MTLRXQOMR_RTC_32   (0x1 << ETH_MTLRXQOMR_RTC_SHIFT)
#  define ETH_MTLRXQOMR_RTC_96   (0x2 << ETH_MTLRXQOMR_RTC_SHIFT)
#  define ETH_MTLRXQOMR_RTC_128  (0x3 << ETH_MTLRXQOMR_RTC_SHIFT)
#define ETH_MTLRXQOMR_FUP        (1 << 3) /* Bit 3: Forward Undersized Good Packets */
#define ETH_MTLRXQOMR_FEP        (1 << 4) /* Bit 4: Forward Error Packets */
#define ETH_MTLRXQOMR_RSF        (1 << 5) /* Bit 5: Receive Queue Store and Forward */
#define ETH_MTLRXQOMR_DIS_TCP_EF (1 << 6) /* Bit 6: Disable Dropping of TCP/IP Checksum Error Packets */
#define ETH_MTLRXQOMR_EHFC       (1 << 7) /* Bit 7: Enable Hardware Flow Control */
#define ETH_MTLRXQOMR_RFA_SHIFT  (8)      /* Bits 8-10: Threshold for Activating Flow Control */
#define ETH_MTLRXQOMR_RFA_MASK   (0x7 << ETH_MTLRXQOMR_RFA_SHIFT)
#define ETH_MTLRXQOMR_RFD_SHIFT  (14)     /* Bits 14-16: Threshold for Deactivating Flow Control */
#define ETH_MTLRXQOMR_RFD_MASK   (0x7 << ETH_MTLRXQOMR_RFD_SHIFT)
#define ETH_MTLRXQOMR_RQS_SHIFT  (20)     /* Bits 20-22: Receive Queue Size */
#define ETH_MTLRXQOMR_RQS_MASK   (0x7 << ETH_MTLRXQOMR_RQS_SHIFT)

#define ETH_MTLRXQMPOCR_OVFPKTCNT_SHIFT (0)       /* Bits 0-10: Overflow Packet Counter */
#define ETH_MTLRXQMPOCR_OVFPKTCNT_MASK  (0x7ff << ETH_MTLRXQMPOCR_OVFPKTCNT_SHIFT)
#define ETH_MTLRXQMPOCR_OVFCNTOVF       (1 << 11) /* Bit 11: Overflow Counter Overflow Bit */
#define ETH_MTLRXQMPOCR_MISPKTCNT_SHIFT (16)      /* Bits 16-26: Missed Packet Counter */
#define ETH_MTLRXQMPOCR_MISPKTCNT_MASK  (0x7ff << ETH_MTLRXQMPOCR_MISPKTCNT_SHIFT)
#define ETH_MTLRXQMPOCR_MISCNTOVF       (1 << 27) /* Bit 27: Missed Packet Counter Overflow Bit */

#define ETH_MTLRXQDR_RWCSTS         (1 << 0) /* Bit 0: MTL Rx Queue Write Controller Active Status */
#define ETH_MTLRXQDR_RRCSTS_SHIFT   (1)      /* Bits 1-2: MTL Rx Queue Read Controller State */
#define ETH_MTLRXQDR_RRCSTS_MASK    (0x3 << ETH_MTLRXQDR_RRCSTS_SHIFT)
#define ETH_MTLRXQDR_RXQSTS_SHIFT   (4)      /* Bits 4-5: MTL Rx Queue Fill-Level Status */
#define ETH_MTLRXQDR_RXQSTS_MASK    (0x3 << ETH_MTLRXQDR_RXQSTS_SHIFT)
#define ETH_MTLRXQDR_PRXQ_SHIFT     (16)     /* Bits 16-29: Number of Packets in Receive Queue */
#define ETH_MTLRXQDR_PRXQ_MASK      (0x3fff << ETH_MTLRXQDR_PRXQ_SHIFT)

/* Ethernet MAC MDIO data register */

/* Ethernet DMA registers ***************************************************/

#define ETH_DMAMR_SWR          (1 << 0)  /* Bit 0: Software Reset */
#define ETH_DMAMR_DA           (1 << 1)  /* Bit 1: DMA Tx or Rx Arbitration Scheme */
#define ETH_DMAMR_TXPR         (1 << 11) /* Bit 11: Transmit priority */
#define ETH_DMAMR_PR_SHIFT     (12)      /* Bits 12-14: Priority ratio */
#define ETH_DMAMR_PR_MASK      (0x7 << ETH_DMAMR_PR_SHIFT)
#  define ETH_DMAMR_PR_1TO1    (0x0 << ETH_DMAMR_PR_SHIFT)
#  define ETH_DMAMR_PR_2TO1    (0x1 << ETH_DMAMR_PR_SHIFT)
#  define ETH_DMAMR_PR_3TO1    (0x2 << ETH_DMAMR_PR_SHIFT)
#  define ETH_DMAMR_PR_4TO1    (0x3 << ETH_DMAMR_PR_SHIFT)
#  define ETH_DMAMR_PR_5TO1    (0x4 << ETH_DMAMR_PR_SHIFT)
#  define ETH_DMAMR_PR_6TO1    (0x5 << ETH_DMAMR_PR_SHIFT)
#  define ETH_DMAMR_PR_7TO1    (0x6 << ETH_DMAMR_PR_SHIFT)
#  define ETH_DMAMR_PR_8TO1    (0x7 << ETH_DMAMR_PR_SHIFT)

#define ETH_DMAMR_INTM_SHIFT   (16)    /* Bits 16-17: Interrupt Mode */
#define ETH_DMAMR_INTM_MASK    (0x3 << ETH_DMAMR_INTM_SHIFT)

#define STM32_ETH_DMASBMR_FB   (1 << 0)  /* Bit 0: Fixed Burst Length */
#define STM32_ETH_DMASBMR_AAL  (1 << 12) /* Bit 12: Address-Aligned Beats */
#define STM32_ETH_DMASBMR_MB   (1 << 14) /* Bit 14: Mixed Burst */
#define STM32_ETH_DMASBMR_RB   (1 << 15) /* Bit 15: Rebuild INCRx Burst */

#define ETH_DMACIER_TIE              (1 << 0)  /* Bit 0:  Transmit interrupt enable */
#define ETH_DMACIER_TXSE             (1 << 1)  /* Bit 1:  Transmit process stopped interrupt enable */
#define ETH_DMACIER_TBUE             (1 << 2)  /* Bit 2:  Transmit buffer unavailable interrupt enable */

#define ETH_DMACIER_RIE              (1 << 6)  /* Bit 6:  Receive interrupt enable */
#define ETH_DMACIER_RBUE             (1 << 7)  /* Bit 7:  Receive buffer unavailable interrupt enable */
#define ETH_DMACIER_RSE              (1 << 8)  /* Bit 8:  Receive process stopped interrupt enable */
#define ETH_DMACIER_RWTE             (1 << 9)  /* Bit 9:  Receive watchdog timeout interrupt enable */
#define ETH_DMACIER_ETIE             (1 << 10) /* Bit 10: Early transmit interrupt enable */
#define ETH_DMACIER_ERIE             (1 << 11) /* Bit 11: Early receive interrupt enable*/
#define ETH_DMACIER_FBEE             (1 << 12) /* Bit 12: Fatal bus error interrupt enable*/
#define ETH_DMACIER_CDEE             (1 << 13) /* Bit 13: Context descriptor error interrupt enable */
#define ETH_DMACIER_AIE              (1 << 14) /* Bit 14: Abnormal interrupt summary interrupt enable */
#define ETH_DMACIER_NIE              (1 << 15) /* Bit 15: Normal interrupt summary enable */

#define ETH_DMACSR_TI                (1 << 0)  /* Bit 0:  Transmit interrupt */
#define ETH_DMACSR_TPS               (1 << 1)  /* Bit 1:  Transmit process stopped interrupt */
#define ETH_DMACSR_TBU               (1 << 2)  /* Bit 2:  Transmit buffer unavailable interrupt */

#define ETH_DMACSR_RI                (1 << 6)  /* Bit 6:  Receive interrupt */
#define ETH_DMACSR_RBU               (1 << 7)  /* Bit 7:  Receive buffer unavailable interrupt */
#define ETH_DMACSR_RPS               (1 << 8)  /* Bit 8:  Receive process stopped interrupt */
#define ETH_DMACSR_RWT               (1 << 9)  /* Bit 9:  Receive watchdog timeout interrupt */
#define ETH_DMACSR_ETI               (1 << 10) /* Bit 10: Early transmit interrupt */
#define ETH_DMACSR_ERI               (1 << 11) /* Bit 11: Early receive interrupt */
#define ETH_DMACSR_FBE               (1 << 12) /* Bit 12: Fatal bus error interrupt */
#define ETH_DMACSR_CDE               (1 << 13) /* Bit 13: Context descriptor error interrupt */
#define ETH_DMACSR_AIS               (1 << 14) /* Bit 14: Abnormal interrupt summary interrupt */
#define ETH_DMACSR_NIS               (1 << 15) /* Bit 15: Normal interrupt summary */
#define ETH_DMACSR_TEB_SHIFT         (16)      /* Bits 16-18: Tx DMA error bits */
#define ETH_DMACSR_TEB_MASK          (0x7 << ETH_DMACSR_TEB_SHIFT)
#define ETH_DMACSR_REB_SHIFT         (19)      /* Bits 19-21: Rx DMA error bits */
#define ETH_DMACSR_REB_MASK          (0x7 << ETH_DMACSR_REB_SHIFT)

#define ETH_DMACTXDTPR_TDT_SHIFT      (2) /* Transmit Descriptor Tail Pointer */
#define ETH_DMACTXDTPR_TDT_MASK       (0x3fffffff << ETH_DMACTXDTPR_TDT_SHIFT)

#define ETH_DMACCR_MSS_SHIFT        (0)       /* Bits 0-13: Maximum Segment Size */
#define ETH_DMACCR_MSS_MASK         (0x3fff << ETH_DMACCR_MSS_SHIFT)
#define ETH_DMACCR_PBLX8            (1 << 16) /* Bit 16: 8xPBL mode */
#define ETH_DMACCR_DSL_SHIFT        (18)      /* Bits 18-20: Descriptor Skip Length */
#define ETH_DMACCR_DSL_MASK         (0x7 << ETH_DMACCR_DSL_SHIFT)
#define ETH_DMACCR_DSL(n)           ((n) << ETH_DMACCR_DSL_SHIFT)

#define ETH_DMACTXCR_ST             (1 << 0)  /* Bit 0: Start or Stop Transmission Command */
#define ETH_DMACTXCR_OSF            (1 << 4)  /* Bit 4: Operate on Second Packet */
#define ETH_DMACTXCR_TSE            (1 << 12) /* Bit 12: TCP Segmentation Enabled */
#define ETH_DMACTXCR_TXPBL_SHIFT    (16)      /* Bits 16-21: Transmit Programmable Burst Length */
#define ETH_DMACTXCR_TXPBL_MASK     (0x3f << ETH_DMACTXCR_TXPBL_SHIFT)
#define ETH_DMACTXCR_TXPBL(n)       ((n) << ETH_DMACTXCR_TXPBL_SHIFT)

#define ETH_DMACRXCR_SR             (1 << 0)  /* Bit0: Start or Stop Receive */
#define ETH_DMACRXCR_RBSZ_SHIFT     (1)       /* Bits 1-14: Receive Buffer size */
#define ETH_DMACRXCR_RBSZ_MASK      (0x3fff << ETH_DMACRXCR_RBSZ_SHIFT)
#define ETH_DMACRXCR_RBSZ(n)        ((n) << ETH_DMACRXCR_RBSZ_SHIFT)
#define ETH_DMACRXCR_RXPBL_SHIFT    (16)      /* Bits 16-21: Receive Programmable Burst Length */
#define ETH_DMACRXCR_RXPBL_MASK     (0x3f << ETH_DMACRXCR_RXPBL_SHIFT)
#define ETH_DMACRXCR_RXPBL(n)        ((n) << ETH_DMACRXCR_RXPBL_SHIFT)
#define ETH_DMACRXCR_RPF            (1 << 31) /* Bit 31: DMA Rx Channel Packet Flush */

/* DMA Descriptors **********************************************************/

/* TDES0: TDES0 normal descriptor, read-format
 *         (header or buf 1 address[31:0])
 * TDES1: TDES1 normal descriptor, read-format
 *         (buf2 address[31:0] or buf1 address[63:32]
 * TDES2: TDES2 normal descriptor, read-format
 */

#define ETH_TDES2_RD_HL_BL1_SHIFT    (0) /* Header Length or Buffer 1 Length */
#define ETH_TDES2_RD_HL_BL1_MASK     (0x3fff << ETH_TDES2_RD_HL_BL1_SHIFT)
#define ETH_TDES2_RD_VTIR_SHIFT      (14) /* VLAN Tag Insertion or Replacement */
#define ETH_TDES2_RD_VTIR_MASK       (0x3 << ETH_TDES2_RD_VTIR_SHIFT)
#define ETH_TDES2_RD_B2L_SHIFT       (16) /* Buffer 2 Length */
#define ETH_TDES2_RD_B2L_MASK        (0x3fff << ETH_TDES2_RD_B2L_SHIFT)
#define ETH_TDES2_RD_TTSE            (1 << 30)  /* Bit 30: Transmit Timestamp Enable */
#define ETH_TDES2_RD_IOC             (1 << 31)  /* Bit 31: Interrupt on Completion */

/* TDES3: TDES3 normal descriptor, read-format */

#define ETH_TDES3_RD_FL_TPL_SHIFT    (0) /* Packet Length or TCP Payload Length */
#define ETH_TDES3_RD_FL_TPL_MASK     (0x7fff << ETH_TDES3_RD_FL_TPL_SHIFT)
#define ETH_TDES3_RD_TPL             (1 << 15) /* Bit 15: Reserved or TCP Payload Length */

#define ETH_TDES3_RD_CIC_TPL_SHIFT   (16) /* Checksum Insertion Control or TCP Payload Length */
#define ETH_TDES3_RD_CIC_TPL_MASK    (0x3 << ETH_TDES3_RD_CIC_TPL_SHIFT)
#define ETH_TDES3_RD_TSE             (1 << 18) /* Bit 18: TCP Segmentation Enable */

#define ETH_TDES3_RD_THL_SHIFT       (19) /* TCP Header Length */
#define ETH_TDES3_RD_THL_MASK        (0xf << ETH_TDES3_RD_THL_SHIFT)
#define ETH_TDES3_RD_SAIC_SHIFT      (23) /* SA Insertion Control */
#define ETH_TDES3_RD_SAIC_MASK       (0x7 << ETH_TDES3_RD_SAIC_SHIFT)
#define ETH_TDES3_RD_CPC_SHIFT       (26) /* CRC Pad Control */
#define ETH_TDES3_RD_CPC_MASK        (0x3 << ETH_TDES3_RD_CPC_SHIFT)
#define ETH_TDES3_RD_LD              (1 << 28) /* Bit 28: Last Descriptor */
#define ETH_TDES3_RD_FD              (1 << 29) /* Bit 29: First Descriptor */
#define ETH_TDES3_RD_CTXT            (1 << 30) /* Bit 30: Context Type  */
#define ETH_TDES3_RD_OWN             (1 << 31) /* Bit 31: Own bit */

/* TDES0: TDES0 normal descriptor, write-back format (time stamp low)
 * TDES1: TDES1 normal descriptor, write-back format (time stamp high)
 * TDES2: TDES2 normal descriptor, write-back format (reserved)
 * TDES3: TDES3 normal descriptor, write-back format
 */

#define ETH_TDES3_WB_IHE             (1 << 0)  /* Bit 0:  IP Header Error */
#define ETH_TDES3_WB_DB              (1 << 1)  /* Bit 1:  Deferred bit */
#define ETH_TDES3_WB_UF              (1 << 2)  /* Bit 1:  Underflow error */
#define ETH_TDES3_WB_ED              (1 << 3)  /* Bit 2:  Excessive deferral */
#define ETH_TDES3_WB_CC_SHIFT        (4)       /* Bits 4-7: Collision count */
#define ETH_TDES3_WB_CC_MASK         (15 << ETH_TDES3_CC_WB_SHIFT)
#define ETH_TDES3_WB_EC              (1 << 8)  /* Bit 8:  Excessive collision */
#define ETH_TDES3_WB_LC              (1 << 9)  /* Bit 9:  Late collision */
#define ETH_TDES3_WB_NC              (1 << 10) /* Bit 10: No carrier */
#define ETH_TDES3_WB_LOC             (1 << 11) /* Bit 11: Loss of carrier */
#define ETH_TDES3_WB_PCE             (1 << 12) /* Bit 12: Payload checksum error */
#define ETH_TDES3_WB_FF              (1 << 13) /* Bit 13: Packet flushed */
#define ETH_TDES3_WB_JT              (1 << 14) /* Bit 14: Jabber timeout */
#define ETH_TDES3_WB_ES              (1 << 15) /* Bit 15: Error summary */
#define ETH_TDES3_WB_TTSS            (1 << 17) /* Bit 17: Transmit time stamp status */
#define ETH_TDES3_WB_LD              (1 << 28) /* Bit 28: Last Descriptor */
#define ETH_TDES3_WB_FD              (1 << 29) /* Bit 29: First Descriptor */
#define ETH_TDES3_WB_CTXT            (1 << 30) /* Bit 30: Context Type  */
#define ETH_TDES0_WB_OWN             (1 << 31) /* Bit 31: Own bit */

/* TDES0: TDES0 context descriptor (time stamp low)
 * TDES1: TDES1 context descriptor (time stamp high)
 * TDES2: TDES2 context descriptor
 */

#define ETH_CTX_TDES2_MSS_SHIFT      (0) /* Maximum Segment Size */
#define ETH_CTX_TDES2_MSS_MASK       (0x3fff << ETH_CTX_TDES2_MSS_SHIFT)
#define ETH_CTX_TDES2_IVT_SHIFT      (16) /* Inner VLAN Tag */
#define ETH_CTX_TDES2_IVT_MASK       (0xffff << ETH_CTX_TDES2_IVT_SHIFT)

/* TDES3: TDES3 context descriptor */

#define ETH_CTX_TDES3_VT_SHIFT      (0) /* VLAN Tag */
#define ETH_CTX_TDES3_VT_MASK       (0xffff << ETH_CTX_TDES3_VT_SHIFT)
#define ETH_CTX_TDES3_VLTV          (1 << 16) /* Bit 16: VLAN Tag Valid */
#define ETH_CTX_TDES3_IVLTV         (1 << 17) /* Bit 17: Inner VLAN Tag Valid */

#define ETH_CTX_TDES3_IVTIR_SHIFT   (18) /* Inner VLAN Tag Insert or Replace */
#define ETH_CTX_TDES3_IVTIR_MASK    (0x3 << ETH_CTX_TDES3_IVTIR_SHIFT)
#define ETH_CTX_TDES3_CDE           (1 << 23) /* Bit 23: Context Descriptor Error */
#define ETH_CTX_TDES3_TCMSSV        (1 << 26) /* Bit 26: One-Step Timestamp Correction Input or MSS Valid */
#define ETH_CTX_TDES3_OSTC          (1 << 27) /* Bit 27: One-Step Timestamp Correction Enable */
#define ETH_CTX_TDES3_CTXT          (1 << 30) /* Bit 30: Context Type */
#define ETH_CTX_TDES3_OWN           (1 << 31) /* Bit 31: Own bit */

/* RDES0: Receive descriptor Word0, read-format (Buffer 1 address[31:0])

 * RDES1: Receive descriptor Word1, read-format
 *    (Reserved or Header/Buffer 1 Address[63:32] address)
 * RDES2: Receive descriptor Word2, read-format
 *    (Payload or Buffer 2 Address[31:0])
 * RDES3: Receive descriptor Word3, read-format
 */

#define ETH_RDES3_RD_BUF1V          (1 << 24) /* Bit 24: Buffer 1 Address Valid */
#define ETH_RDES3_RD_BUF2V          (1 << 25) /* Bit 25: Buffer 2 Address Valid */
#define ETH_RDES3_RD_IOC            (1 << 30) /* Bit 30: Interrupt Enabled on Completion */
#define ETH_RDES3_RD_OWN            (1 << 31) /* Bit 31: Own bit */

/* RDES0: Receive descriptor Word0, write-back format */

#define ETH_RDES0_WB_OVT_SHIFT      (0)       /* Bits 0-15: Outer VLAN Tag */
#define ETH_RDES0_WB_OVT_MASK       (0xffff << ETH_RDES0_WB_OVT_SHIFT)
#define ETH_RDES0_WB_IVT_SHIFT      (16)      /* Bits 16-31: Inner VLAN Tag */
#define ETH_RDES0_WB_IVT_MASK       (0xffff << ETH_RDES0_WB_IVT_SHIFT)

/* RDES1: Receive descriptor Word1, write-back format */

#define ETH_RDES1_WB_PT_SHIFT       (0)       /* Bits 0-2: Payload Type */
#define ETH_RDES1_WB_PT_MASK        (0x7 << ETH_RDES1_WB_PT_SHIFT)
#  define ETH_RDES1_WB_PT_UNKNOWN   (0x0)
#  define ETH_RDES1_WB_PT_UDP       (0x1)
#  define ETH_RDES1_WB_PT_TCP       (0x2)
#  define ETH_RDES1_WB_PT_ICMP      (0x3)
#define ETH_RDES1_WB_IPHE           (1 << 3)  /* Bit 3: IP Header Error */
#define ETH_RDES1_WB_IPV4           (1 << 4)  /* Bit 4: IPv4 Header Present */
#define ETH_RDES1_WB_IPV6           (1 << 5)  /* Bit 5: IPv6 Header Present */
#define ETH_RDES1_WB_IPCB           (1 << 6)  /* Bit 6: IP Checksum Bypassed */
#define ETH_RDES1_WB_IPCE           (1 << 7)  /* Bit 7: IP Payload Error */
#define ETH_RDES1_WB_PMT_SHIFT      (8)       /* Bits 8-11: PTP Message Type */
#define ETH_RDES1_WB_PMT_MASK       (0xf << ETH_RDES1_WB_PMT_SHIFT)
#define ETH_RDES1_WB_PFT            (1 << 12) /* Bit 12: PTP Packet Type */
#define ETH_RDES1_WB_PV             (1 << 13) /* Bit 13: PTP Version */
#define ETH_RDES1_WB_TSA            (1 << 14) /* Bit 14: Timestamp Available */
#define ETH_RDES1_WB_TD             (1 << 15) /* Bit 15: Timestamp Dropped */
#define ETH_RDES1_WB_OPC_SHIFT      (16)      /* Bits 16-31: OAM Sub-Type Code, or MAC Control Packet opcode */
#define ETH_RDES1_WB_OPC_MASK       (0xffff << ETH_RDES1_WB_OPC_SHIFT)

/* RDES2: Receive descriptor Word2, write-back format */

#define ETH_RDES2_WB_ARPNR          (1 << 10) /* Bit 10: ARP Reply Not Generated */
#define ETH_RDES2_WB_VF             (1 << 15) /* Bit 15: VLAN Filter Status */
#define ETH_RDES2_WB_SAF            (1 << 16) /* Bit 16: SA Address Filter Fail */
#define ETH_RDES2_WB_DAF            (1 << 17) /* Bit 17: Destination Address Filter Fail */
#define ETH_RDES2_WB_HF             (1 << 18) /* Bit 18: Hash Filter Status */
#define ETH_RDES2_WB_MADRM_SHIFT    (19)      /* Bits 19-26: MAC Address Match or Hash Value */
#define ETH_RDES2_WB_MADRM_MASK     (0xff << ETH_RDES1_WB_MADRM_SHIFT)
#define ETH_RDES2_WB_L3FM           (1 << 27) /* Bit 27: Layer 3 Filter Match */
#define ETH_RDES2_WB_L4FM           (1 << 28) /* Bit 28: Layer 4 Filter Match */
#define ETH_RDES2_WB_L3L4FM_SHIFT   (29)      /* Bits 29-31: Layer 3 and Layer 4 Filter Number Matched */
#define ETH_RDES2_WB_L3L4FM_MASK    (0x7 << ETH_RDES1_WB_L3L4FM_SHIFT)

/* RDES3: Receive descriptor Word3, write-back format */

#define ETH_RDES3_WB_PL_SHIFT       (0)      /* Bits 0-14: Packet Length */
#define ETH_RDES3_WB_PL_MASK        (0x7fff << ETH_RDES3_WB_PL_SHIFT)
#define ETH_RDES3_WB_ES             (1 << 15) /* Bit 15: Error Summary */
#define ETH_RDES3_WB_LT_SHIFT       (16)      /* Bits 16-18: Length/Type Field */
#define ETH_RDES3_WB_LT_MASK        (0x7 << ETH_RDES3_WB_LT_SHIFT)
#define ETH_RDES3_WB_DE             (1 << 19) /* Bit 19: Dribble Bit Error */
#define ETH_RDES3_WB_RE             (1 << 20) /* Bit 20: Receive Error */
#define ETH_RDES3_WB_OE             (1 << 21) /* Bit 21: Overflow Error */
#define ETH_RDES3_WB_RWT            (1 << 22) /* Bit 22: Receive Watchdog Timeout */
#define ETH_RDES3_WB_GP             (1 << 23) /* Bit 23: Giant Packet */
#define ETH_RDES3_WB_CE             (1 << 24) /* Bit 24: CRC Error */
#define ETH_RDES3_WB_RS0V           (1 << 25) /* Bit 25: Receive Status RDES0 Valid */
#define ETH_RDES3_WB_RS1V           (1 << 26) /* Bit 26: Receive Status RDES1 Valid */
#define ETH_RDES3_WB_RS2V           (1 << 27) /* Bit 27: Receive Status RDES2 Valid */
#define ETH_RDES3_WB_LD             (1 << 28) /* Bit 28: Last Descriptor */
#define ETH_RDES3_WB_FD             (1 << 29) /* Bit 29: First Descriptor */
#define ETH_RDES3_WB_CTXT           (1 << 30) /* Bit 30: Context Type  */
#define ETH_RDES3_WB_OWN            (1 << 31) /* Bit 31: Own bit */

/* RDES0: RDES0 context descriptor (time stamp low)
 * RDES1: RDES1 context descriptor (time stamp high)
 * RDES2: RDES2 context descriptor (reserved)
 * RDES3: RDES3 context descriptor
 */

#define ETH_CTX_RDES3_CTXT          (1 << 30) /* Bit 30: Context Type  */
#define ETH_CTX_RDES3_OWN           (1 << 31) /* Bit 31: Own bit */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Ethernet RX/TX DMA Descriptor */

struct eth_desc_s
{
  /* DMA descriptor words */

  volatile uint32_t des0;
  volatile uint32_t des1;
  volatile uint32_t des2;
  volatile uint32_t des3;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32H7_STM32H7X3XX */
#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_ETHERNET_H */
