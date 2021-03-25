/****************************************************************************
 * arch/arm/src/tiva/hardware/tm4c/tm4c_ethernet.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes some register bit definitions provided by:
 *
 *   Copyright (C) 2014 TRD2 Inc. All rights reserved.
 *   Author: Calvin Maguranis <calvin.maguranis@trd2inc.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C_ETHERNET_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ethernet Controller Register Offsets *************************************/

/* Ethernet MAC Register Offsets */

#define TIVA_EMAC_CFG_OFFSET           0x0000  /* Ethernet MAC Configuration */
#define TIVA_EMAC_FRAMEFLTR_OFFSET     0x0004  /* Ethernet MAC Frame Filter */
#define TIVA_EMAC_HASHTBLH_OFFSET      0x0008  /* Ethernet MAC Hash Table High */
#define TIVA_EMAC_HASHTBLL_OFFSET      0x000c  /* Ethernet MAC Hash Table Low */
#define TIVA_EMAC_MIIADDR_OFFSET       0x0010  /* Ethernet MAC MII Address */
#define TIVA_EMAC_MIIDATA_OFFSET       0x0014  /* Ethernet MAC MII Data Register */
#define TIVA_EMAC_FLOWCTL_OFFSET       0x0018  /* Ethernet MAC Flow Control */
#define TIVA_EMAC_VLANTG_OFFSET        0x001c  /* Ethernet MAC VLAN Tag */
#define TIVA_EMAC_STATUS_OFFSET        0x0024  /* Ethernet MAC Status */
#define TIVA_EMAC_RWUFF_OFFSET         0x0028  /* Ethernet MAC Remote Wake-Up Frame Filter */
#define TIVA_EMAC_PMTCTLSTAT_OFFSET    0x002c  /* Ethernet MAC PMT Control and Status Register */
#define TIVA_EMAC_RIS_OFFSET           0x0038  /* Ethernet MAC Raw Interrupt Status */
#define TIVA_EMAC_IM_OFFSET            0x003c  /* Ethernet MAC Interrupt Mask */
#define TIVA_EMAC_ADDR0H_OFFSET        0x0040  /* Ethernet MAC Address 0 High */
#define TIVA_EMAC_ADDR0L_OFFSET        0x0044  /* Ethernet MAC Address 0 Low Register */
#define TIVA_EMAC_ADDR1H_OFFSET        0x0048  /* Ethernet MAC Address 1 High */
#define TIVA_EMAC_ADDR1L_OFFSET        0x004c  /* Ethernet MAC Address 1 Low */
#define TIVA_EMAC_ADDR2H_OFFSET        0x0050  /* Ethernet MAC Address 2 High */
#define TIVA_EMAC_ADDR2L_OFFSET        0x0054  /* Ethernet MAC Address 2 Low */
#define TIVA_EMAC_ADDR3H_OFFSET        0x0058  /* Ethernet MAC Address 3 High */
#define TIVA_EMAC_ADDR3L_OFFSET        0x005c  /* Ethernet MAC Address 3 Low */
#define TIVA_EMAC_WDOGTO_OFFSET        0x00dc  /* Ethernet MAC Watchdog Timeout */

#define TIVA_EMAC_MMCCTRL_OFFSET       0x0100  /* Ethernet MAC MMC Control */
#define TIVA_EMAC_MMCRXRIS_OFFSET      0x0104  /* Ethernet MAC MMC Receive Raw Interrupt Status */
#define TIVA_EMAC_MMCTXRIS_OFFSET      0x0108  /* Ethernet MAC MMC Transmit Raw Interrupt Status */
#define TIVA_EMAC_MMCRXIM_OFFSET       0x010c  /* Ethernet MAC MMC Receive Interrupt Mask */
#define TIVA_EMAC_MMCTXIM_OFFSET       0x0110  /* Ethernet MAC MMC Transmit Interrupt Mask */
#define TIVA_EMAC_TXCNTGB_OFFSET       0x0118  /* Ethernet MAC Transmit Frame Count for Good and Bad Frames */
#define TIVA_EMAC_TXCNTSCOL_OFFSET     0x014c  /* Ethernet MAC Transmit Frame Count for Frames Transmitted after Single Collision */
#define TIVA_EMAC_TXCNTMCOL_OFFSET     0x0150  /* Ethernet MAC Transmit Frame Count for Frames Transmitted after Multiple Collisions */
#define TIVA_EMAC_TXOCTCNTG_OFFSET     0x0164  /* Ethernet MAC Transmit Octet Count Good */
#define TIVA_EMAC_RXCNTGB_OFFSET       0x0180  /* Ethernet MAC Receive Frame Count for Good and Bad Frames */
#define TIVA_EMAC_RXCNTCRCERR_OFFSET   0x0194  /* Ethernet MAC Receive Frame Count for CRC Error Frames */
#define TIVA_EMAC_RXCNTALGNERR_OFFSET  0x0198  /* Ethernet MAC Receive Frame Count for Alignment Error Frames */
#define TIVA_EMAC_RXCNTGUNI_OFFSET     0x01c4  /* Ethernet MAC Receive Frame Count for Good Unicast Frames */

#define TIVA_EMAC_VLNINCREP_OFFSET     0x0584  /* Ethernet MAC VLAN Tag Inclusion or Replacement */
#define TIVA_EMAC_VLANHASH_OFFSET      0x0588  /* Ethernet MAC VLAN Hash Table */

#define TIVA_EMAC_TIMSTCTRL_OFFSET     0x0700  /* Ethernet MAC Timestamp Control */
#define TIVA_EMAC_SUBSECINC_OFFSET     0x0704  /* Ethernet MAC Sub-Second Increment */
#define TIVA_EMAC_TIMSEC_OFFSET        0x0708  /* Ethernet MAC System Time - Seconds */
#define TIVA_EMAC_TIMNANO_OFFSET       0x070c  /* Ethernet MAC System Time - Nanoseconds */
#define TIVA_EMAC_TIMSECU_OFFSET       0x0710  /* Ethernet MAC System Time - Seconds Update */
#define TIVA_EMAC_TIMNANOU_OFFSET      0x0714  /* Ethernet MAC System Time - Nanoseconds Update */
#define TIVA_EMAC_TIMADD_OFFSET        0x0718  /* Ethernet MAC Timestamp Addend */
#define TIVA_EMAC_TARGSEC_OFFSET       0x071c  /* Ethernet MAC Target Time Seconds */
#define TIVA_EMAC_TARGNANO_OFFSET      0x0720  /* Ethernet MAC Target Time Nanoseconds */
#define TIVA_EMAC_HWORDSEC_OFFSET      0x0724  /* Ethernet MAC System Time-Higher Word Seconds */
#define TIVA_EMAC_TIMSTAT_OFFSET       0x0728  /* Ethernet MAC Timestamp Status */
#define TIVA_EMAC_PPSCTRL_OFFSET       0x072c  /* Ethernet MAC PPS Control */
#define TIVA_EMAC_PPS0INTVL_OFFSET     0x0760  /* Ethernet MAC PPS0 Interval */
#define TIVA_EMAC_PPS0WIDTH_OFFSET     0x0764  /* Ethernet MAC PPS0 Width */

#define TIVA_EMAC_DMABUSMOD_OFFSET     0x0c00  /* Ethernet MAC DMA Bus Mode */
#define TIVA_EMAC_TXPOLLD_OFFSET       0x0c04  /* Ethernet MAC Transmit Poll Demand */
#define TIVA_EMAC_RXPOLLD_OFFSET       0x0c08  /* Ethernet MAC Receive Poll Demand */
#define TIVA_EMAC_RXDLADDR_OFFSET      0x0c0c  /* Ethernet MAC Receive Descriptor List Address */
#define TIVA_EMAC_TXDLADDR_OFFSET      0x0c10  /* Ethernet MAC Transmit Descriptor List Address */
#define TIVA_EMAC_DMARIS_OFFSET        0x0c14  /* Ethernet MAC DMA Interrupt Status */
#define TIVA_EMAC_DMAOPMODE_OFFSET     0x0c18  /* Ethernet MAC DMA Operation Mode */
#define TIVA_EMAC_DMAIM_OFFSET         0x0c1c  /* Ethernet MAC DMA Interrupt Mask Register */
#define TIVA_EMAC_MFBOC_OFFSET         0x0c20  /* Ethernet MAC Missed Frame and Buffer Overflow Counter */
#define TIVA_EMAC_RXINTWDT_OFFSET      0x0c24  /* Ethernet MAC Receive Interrupt Watchdog Timer */
#define TIVA_EMAC_HOSTXDESC_OFFSET     0x0c48  /* Ethernet MAC Current Host Transmit Descriptor */
#define TIVA_EMAC_HOSRXDESC_OFFSET     0x0c4c  /* Ethernet MAC Current Host Receive Descriptor */
#define TIVA_EMAC_HOSTXBA_OFFSET       0x0c50  /* Ethernet MAC Current Host Transmit Buffer Address */
#define TIVA_EMAC_HOSRXBA_OFFSET       0x0c54  /* Ethernet MAC Current Host Receive Buffer Address */

#define TIVA_EMAC_PP_OFFSET            0x0fc0  /* Ethernet MAC Peripheral Property Register */
#define TIVA_EMAC_PC_OFFSET            0x0fc4  /* Ethernet MAC Peripheral Configuration Register */
#define TIVA_EMAC_CC_OFFSET            0x0fc8  /* Ethernet MAC Clock Configuration Register */
#define TIVA_EPHY_RIS_OFFSET           0x0fd0  /* Ethernet PHY Raw Interrupt Status */
#define TIVA_EPHY_IM_OFFSET            0x0fd4  /* Ethernet PHY Interrupt Mask */
#define TIVA_EPHY_MISC_OFFSET          0x0fd8  /* RW1C Ethernet PHY Masked Interrupt Status and Clear */

/* Ethernet Controller Register Addresses ***********************************/

#define TIVA_EMAC_CFG                  (TIVA_ETHCON_BASE + TIVA_EMAC_CFG_OFFSET)
#define TIVA_EMAC_FRAMEFLTR            (TIVA_ETHCON_BASE + TIVA_EMAC_FRAMEFLTR_OFFSET)
#define TIVA_EMAC_HASHTBLH             (TIVA_ETHCON_BASE + TIVA_EMAC_HASHTBLH_OFFSET)
#define TIVA_EMAC_HASHTBLL             (TIVA_ETHCON_BASE + TIVA_EMAC_HASHTBLL_OFFSET)
#define TIVA_EMAC_MIIADDR              (TIVA_ETHCON_BASE + TIVA_EMAC_MIIADDR_OFFSET)
#define TIVA_EMAC_MIIDATA              (TIVA_ETHCON_BASE + TIVA_EMAC_MIIDATA_OFFSET)
#define TIVA_EMAC_FLOWCTL              (TIVA_ETHCON_BASE + TIVA_EMAC_FLOWCTL_OFFSET)
#define TIVA_EMAC_VLANTG               (TIVA_ETHCON_BASE + TIVA_EMAC_VLANTG_OFFSET)
#define TIVA_EMAC_STATUS               (TIVA_ETHCON_BASE + TIVA_EMAC_STATUS_OFFSET)
#define TIVA_EMAC_RWUFF                (TIVA_ETHCON_BASE + TIVA_EMAC_RWUFF_OFFSET)
#define TIVA_EMAC_PMTCTLSTAT           (TIVA_ETHCON_BASE + TIVA_EMAC_PMTCTLSTAT_OFFSET)
#define TIVA_EMAC_RIS                  (TIVA_ETHCON_BASE + TIVA_EMAC_RIS_OFFSET)
#define TIVA_EMAC_IM                   (TIVA_ETHCON_BASE + TIVA_EMAC_IM_OFFSET)
#define TIVA_EMAC_ADDR0H               (TIVA_ETHCON_BASE + TIVA_EMAC_ADDR0H_OFFSET)
#define TIVA_EMAC_ADDR0L               (TIVA_ETHCON_BASE + TIVA_EMAC_ADDR0L_OFFSET)
#define TIVA_EMAC_ADDR1H               (TIVA_ETHCON_BASE + TIVA_EMAC_ADDR1H_OFFSET)
#define TIVA_EMAC_ADDR1L               (TIVA_ETHCON_BASE + TIVA_EMAC_ADDR1L_OFFSET)
#define TIVA_EMAC_ADDR2H               (TIVA_ETHCON_BASE + TIVA_EMAC_ADDR2H_OFFSET)
#define TIVA_EMAC_ADDR2L               (TIVA_ETHCON_BASE + TIVA_EMAC_ADDR2L_OFFSET)
#define TIVA_EMAC_ADDR3H               (TIVA_ETHCON_BASE + TIVA_EMAC_ADDR3H_OFFSET)
#define TIVA_EMAC_ADDR3L               (TIVA_ETHCON_BASE + TIVA_EMAC_ADDR3L_OFFSET)
#define TIVA_EMAC_WDOGTO               (TIVA_ETHCON_BASE + TIVA_EMAC_WDOGTO_OFFSET)
#define TIVA_EMAC_MMCCTRL              (TIVA_ETHCON_BASE + TIVA_EMAC_MMCCTRL_OFFSET)
#define TIVA_EMAC_MMCRXRIS             (TIVA_ETHCON_BASE + TIVA_EMAC_MMCRXRIS_OFFSET)
#define TIVA_EMAC_MMCTXRIS             (TIVA_ETHCON_BASE + TIVA_EMAC_MMCTXRIS_OFFSET)
#define TIVA_EMAC_MMCRXIM              (TIVA_ETHCON_BASE + TIVA_EMAC_MMCRXIM_OFFSET)
#define TIVA_EMAC_MMCTXIM              (TIVA_ETHCON_BASE + TIVA_EMAC_MMCTXIM_OFFSET)
#define TIVA_EMAC_TXCNTGB              (TIVA_ETHCON_BASE + TIVA_EMAC_TXCNTGB_OFFSET)
#define TIVA_EMAC_TXCNTSCOL            (TIVA_ETHCON_BASE + TIVA_EMAC_TXCNTSCOL_OFFSET)
#define TIVA_EMAC_TXCNTMCOL            (TIVA_ETHCON_BASE + TIVA_EMAC_TXCNTMCOL_OFFSET)
#define TIVA_EMAC_TXOCTCNTG            (TIVA_ETHCON_BASE + TIVA_EMAC_TXOCTCNTG_OFFSET)
#define TIVA_EMAC_RXCNTGB              (TIVA_ETHCON_BASE + TIVA_EMAC_RXCNTGB_OFFSET)
#define TIVA_EMAC_RXCNTCRCERR          (TIVA_ETHCON_BASE + TIVA_EMAC_RXCNTCRCERR_OFFSET)
#define TIVA_EMAC_RXCNTALGNERR         (TIVA_ETHCON_BASE + TIVA_EMAC_RXCNTALGNERR_OFFSET)
#define TIVA_EMAC_RXCNTGUNI            (TIVA_ETHCON_BASE + TIVA_EMAC_RXCNTGUNI_OFFSET)
#define TIVA_EMAC_VLNINCREP            (TIVA_ETHCON_BASE + TIVA_EMAC_VLNINCREP_OFFSET)
#define TIVA_EMAC_VLANHASH             (TIVA_ETHCON_BASE + TIVA_EMAC_VLANHASH_OFFSET)
#define TIVA_EMAC_TIMSTCTRL            (TIVA_ETHCON_BASE + TIVA_EMAC_TIMSTCTRL_OFFSET)
#define TIVA_EMAC_SUBSECINC            (TIVA_ETHCON_BASE + TIVA_EMAC_SUBSECINC_OFFSET)
#define TIVA_EMAC_TIMSEC               (TIVA_ETHCON_BASE + TIVA_EMAC_TIMSEC_OFFSET)
#define TIVA_EMAC_TIMNANO              (TIVA_ETHCON_BASE + TIVA_EMAC_TIMNANO_OFFSET)
#define TIVA_EMAC_TIMSECU              (TIVA_ETHCON_BASE + TIVA_EMAC_TIMSECU_OFFSET)
#define TIVA_EMAC_TIMNANOU             (TIVA_ETHCON_BASE + TIVA_EMAC_TIMNANOU_OFFSET)
#define TIVA_EMAC_TIMADD               (TIVA_ETHCON_BASE + TIVA_EMAC_TIMADD_OFFSET)
#define TIVA_EMAC_TARGSEC              (TIVA_ETHCON_BASE + TIVA_EMAC_TARGSEC_OFFSET)
#define TIVA_EMAC_TARGNANO             (TIVA_ETHCON_BASE + TIVA_EMAC_TARGNANO_OFFSET)
#define TIVA_EMAC_HWORDSEC             (TIVA_ETHCON_BASE + TIVA_EMAC_HWORDSEC_OFFSET)
#define TIVA_EMAC_TIMSTAT              (TIVA_ETHCON_BASE + TIVA_EMAC_TIMSTAT_OFFSET)
#define TIVA_EMAC_PPSCTRL              (TIVA_ETHCON_BASE + TIVA_EMAC_PPSCTRL_OFFSET)
#define TIVA_EMAC_PPS0INTVL            (TIVA_ETHCON_BASE + TIVA_EMAC_PPS0INTVL_OFFSET)
#define TIVA_EMAC_PPS0WIDTH            (TIVA_ETHCON_BASE + TIVA_EMAC_PPS0WIDTH_OFFSET)
#define TIVA_EMAC_DMABUSMOD            (TIVA_ETHCON_BASE + TIVA_EMAC_DMABUSMOD_OFFSET)
#define TIVA_EMAC_TXPOLLD              (TIVA_ETHCON_BASE + TIVA_EMAC_TXPOLLD_OFFSET)
#define TIVA_EMAC_RXPOLLD              (TIVA_ETHCON_BASE + TIVA_EMAC_RXPOLLD_OFFSET)
#define TIVA_EMAC_RXDLADDR             (TIVA_ETHCON_BASE + TIVA_EMAC_RXDLADDR_OFFSET)
#define TIVA_EMAC_TXDLADDR             (TIVA_ETHCON_BASE + TIVA_EMAC_TXDLADDR_OFFSET)
#define TIVA_EMAC_DMARIS               (TIVA_ETHCON_BASE + TIVA_EMAC_DMARIS_OFFSET)
#define TIVA_EMAC_DMAOPMODE            (TIVA_ETHCON_BASE + TIVA_EMAC_DMAOPMODE_OFFSET)
#define TIVA_EMAC_DMAIM                (TIVA_ETHCON_BASE + TIVA_EMAC_DMAIM_OFFSET)
#define TIVA_EMAC_MFBOC                (TIVA_ETHCON_BASE + TIVA_EMAC_MFBOC_OFFSET)
#define TIVA_EMAC_RXINTWDT             (TIVA_ETHCON_BASE + TIVA_EMAC_RXINTWDT_OFFSET)
#define TIVA_EMAC_HOSTXDESC            (TIVA_ETHCON_BASE + TIVA_EMAC_HOSTXDESC_OFFSET)
#define TIVA_EMAC_HOSRXDESC            (TIVA_ETHCON_BASE + TIVA_EMAC_HOSRXDESC_OFFSET)
#define TIVA_EMAC_HOSTXBA              (TIVA_ETHCON_BASE + TIVA_EMAC_HOSTXBA_OFFSET)
#define TIVA_EMAC_HOSRXBA              (TIVA_ETHCON_BASE + TIVA_EMAC_HOSRXBA_OFFSET)
#define TIVA_EMAC_PP                   (TIVA_ETHCON_BASE + TIVA_EMAC_PP_OFFSET)
#define TIVA_EMAC_PC                   (TIVA_ETHCON_BASE + TIVA_EMAC_PC_OFFSET)
#define TIVA_EMAC_CC                   (TIVA_ETHCON_BASE + TIVA_EMAC_CC_OFFSET)
#define TIVA_EPHY_RIS                  (TIVA_ETHCON_BASE + TIVA_EPHY_RIS_OFFSET)
#define TIVA_EPHY_IM                   (TIVA_ETHCON_BASE + TIVA_EPHY_IM_OFFSET)
#define TIVA_EPHY_MISC                 (TIVA_ETHCON_BASE + TIVA_EPHY_MISC_OFFSET)

/* MII Management Register Addresses */

#define TIVA_EPHY_BMCR                 0x00 /* Ethernet PHY Basic Mode Control */
#define TIVA_EPHY_BMSR                 0x01 /* Ethernet PHY Basic Mode Status */
#define TIVA_EPHY_ID1                  0x02 /* Ethernet PHY Identifier Register 1 */
#define TIVA_EPHY_ID2                  0x03 /* Ethernet PHY Identifier Register 2 */
#define TIVA_EPHY_ANA                  0x04 /* Ethernet PHY Auto-Negotiation Advertisement */
#define TIVA_EPHY_ANLPA                0x05 /* Ethernet PHY Auto-Negotiation Link Partner Ability */
#define TIVA_EPHY_ANER                 0x06 /* Ethernet PHY Auto-Negotiation Expansion */
#define TIVA_EPHY_ANNPTR               0x07 /* Ethernet PHY Auto-Negotiation Next Page TX */
#define TIVA_EPHY_ANLNPTR              0x08 /* Ethernet PHY Auto-Negotiation Link Partner Ability Next Page */
#define TIVA_EPHY_CFG1                 0x09 /* Ethernet PHY Configuration 1 */
#define TIVA_EPHY_CFG2                 0x0a /* Ethernet PHY Configuration 2 */
#define TIVA_EPHY_CFG3                 0x0b /* Ethernet PHY Configuration 3 */
#define TIVA_EPHY_REGCTL               0x0d /* Ethernet PHY Register Control */
#define TIVA_EPHY_ADDAR                0x0e /* Ethernet PHY Address or Data */
#define TIVA_EPHY_STS                  0x10 /* Ethernet PHY Status */
#define TIVA_EPHY_SCR                  0x11 /* Ethernet PHY Specific Control */
#define TIVA_EPHY_MISR1                0x12 /* Ethernet PHY MII Interrupt Status 1 */
#define TIVA_EPHY_MISR2                0x13 /* Ethernet PHY MII Interrupt Status 2 */
#define TIVA_EPHY_FCSCR                0x14 /* Ethernet PHY False Carrier Sense Counter */
#define TIVA_EPHY_RXERCNT              0x15 /* Ethernet PHY Receive Error Count */
#define TIVA_EPHY_BISTCR               0x16 /* Ethernet PHY BIST Control */
#define TIVA_EPHY_LEDCR                0x18 /* Ethernet PHY LED Control */
#define TIVA_EPHY_CTL                  0x19 /* Ethernet PHY Control */
#define TIVA_EPHY_10BTSC               0x1a /* Ethernet PHY 10Base-T Status/Control */
#define TIVA_EPHY_BICSR1               0x1b /* Ethernet PHY BIST Control and Status 1 */
#define TIVA_EPHY_BICSR2               0x1c /* Ethernet PHY BIST Control and Status 2 */
#define TIVA_EPHY_CDCR                 0x1e /* Ethernet PHY Cable Diagnostic Control */
#define TIVA_EPHY_RCR                  0x1f /* Ethernet PHY Reset Control */
#define TIVA_EPHY_LEDCFG               0x25 /* Ethernet PHY LED Configuration */

/* Ethernet Controller Register Bit Definitions *****************************/

/* Ethernet MAC Configuration */

#define EMAC_CFG_PRELEN_SHIFT          (0)       /* Bits 0-1: Preamble Length for Transmit */
#define EMAC_CFG_PRELEN_MASK           (3 << EMAC_CFG_PRELEN_SHIFT)
#  define EMAC_CFG_PRELEN_7            (0 << EMAC_CFG_PRELEN_SHIFT) /* 7 bytes of preamble */
#  define EMAC_CFG_PRELEN_5            (1 << EMAC_CFG_PRELEN_SHIFT) /* 5 bytes of preamble */
#  define EMAC_CFG_PRELEN_3            (2 << EMAC_CFG_PRELEN_SHIFT) /* 3 bytes of preamble */

#define EMAC_CFG_RE                    (1 << 2)  /* Bit 2:  Receiver Enable */
#define EMAC_CFG_TE                    (1 << 3)  /* Bit 3:  Transmitter Enable */
#define EMAC_CFG_DC                    (1 << 4)  /* Bit 4:  Deferral Check */
#define EMAC_CFG_BL_SHIFT              (5)       /* Bits 5-6: Back-Off Limit */
#define EMAC_CFG_BL_MASK               (3 << EMAC_CFG_BL_SHIFT)
#  define EMAC_CFG_BL_10               (0 << EMAC_CFG_BL_SHIFT) /* k = min (n,10) */
#  define EMAC_CFG_BL_8                (1 << EMAC_CFG_BL_SHIFT) /* k = min (n,8) */
#  define EMAC_CFG_BL_4                (2 << EMAC_CFG_BL_SHIFT) /* k = min (n,4) */
#  define EMAC_CFG_BL_1                (3 << EMAC_CFG_BL_SHIFT) /* k = min (n,1) */

#define EMAC_CFG_ACS                   (1 << 7)  /* Bit 7:  Automatic Pad or CRC Stripping */
#define EMAC_CFG_DR                    (1 << 9)  /* Bit 8:  Disable Retry */
#define EMAC_CFG_IPC                   (1 << 10) /* Bit 10: Checksum Offload */
#define EMAC_CFG_DUPM                  (1 << 11) /* Bit 11: Duplex Mode */
#define EMAC_CFG_LOOPBM                (1 << 12) /* Bit 12: Loopback Mode */
#define EMAC_CFG_DRO                   (1 << 13) /* Bit 13: Disable Receive Own */
#define EMAC_CFG_FES                   (1 << 14) /* Bit 14: Speed */
#define EMAC_CFG_PS                    (1 << 15) /* Bit 15: Port Select */
#define EMAC_CFG_DISCRS                (1 << 16) /* Bit 16: Disable Carrier Sense During Transmission */
#define EMAC_CFG_IFG_SHIFT             (17)      /* Bits 17-19: Inter-Frame Gap (IFG) */
#define EMAC_CFG_IFG_MASK              (7 << EMAC_CFG_IFG_SHIFT)
#  define EMAC_CFG_IFG(n)              ((12-((n) >> 3)) << EMAC_CFG_IFG_SHIFT) /* n bit times, n=40,48,..96 */

#  define EMAC_CFG_IFG_96              (0 << EMAC_CFG_IFG_SHIFT) /* 96 bit times */
#  define EMAC_CFG_IFG_88              (1 << EMAC_CFG_IFG_SHIFT) /* 88 bit times */
#  define EMAC_CFG_IFG_80              (2 << EMAC_CFG_IFG_SHIFT) /* 80 bit times */
#  define EMAC_CFG_IFG_72              (3 << EMAC_CFG_IFG_SHIFT) /* 72 bit times */
#  define EMAC_CFG_IFG_64              (4 << EMAC_CFG_IFG_SHIFT) /* 64 bit times */
#  define EMAC_CFG_IFG_56              (5 << EMAC_CFG_IFG_SHIFT) /* 56 bit times */
#  define EMAC_CFG_IFG_48              (6 << EMAC_CFG_IFG_SHIFT) /* 48 bit times */
#  define EMAC_CFG_IFG_40              (7 << EMAC_CFG_IFG_SHIFT) /* 40 bit times */

#define EMAC_CFG_JFEN                  (1 << 20) /* Bit 20: Jumbo Frame Enable */
#define EMAC_CFG_JD                    (1 << 22) /* Bit 21: Jabber Disable */
#define EMAC_CFG_WDDIS                 (1 << 23) /* Bit 23: Watchdog Disable */
#define EMAC_CFG_CST                   (1 << 25) /* Bit 25: CRC Stripping for Type Frames */
#define EMAC_CFG_TWOKPEN               (1 << 27) /* Bit 27: IEEE 802 */
#define EMAC_CFG_SADDR_SHIFT           (28)      /* Bits 28-30: Source Address Insertion or Replacement Control */
#define EMAC_CFG_SADDR_MASK            (7 << EMAC_CFG_SADDR_SHIFT)
#  define EMAC_CFG_SADDR_RADDR0        (2 << EMAC_CFG_SADDR_SHIFT) /* Insert EMACADDR0x */
#  define EMAC_CFG_SADDR_IADDR0        (3 << EMAC_CFG_SADDR_SHIFT) /* Replace with EMACADDR0x */
#  define EMAC_CFG_SADDR_RADDR1        (6 << EMAC_CFG_SADDR_SHIFT) /* Insert EMACADDR1x */
#  define EMAC_CFG_SADDR_IADDR1        (7 << EMAC_CFG_SADDR_SHIFT) /* Replace with EMACADDR1x */

/* Ethernet MAC Frame Filter */

#define EMAC_FRAMEFLTR_PR              (1 << 0)  /* Bit 0:  Promiscuous Mode */
#define EMAC_FRAMEFLTR_HUC             (1 << 1)  /* Bit 1:  Hash Unicast */
#define EMAC_FRAMEFLTR_HMC             (1 << 2)  /* Bit 2:  Hash Multicast */
#define EMAC_FRAMEFLTR_DAIF            (1 << 3)  /* Bit 3:  Destination Address (DA) Inverse Filtering */
#define EMAC_FRAMEFLTR_PM              (1 << 4)  /* Bit 4:  Pass All Multicast */
#define EMAC_FRAMEFLTR_DBF             (1 << 5)  /* Bit 5:  Disable Broadcast Frames */
#define EMAC_FRAMEFLTR_PCF_SHIFT       (6)       /* Bits 6-7: Pass Control Frames */
#define EMAC_FRAMEFLTR_PCF_MASK        (3 << EMAC_FRAMEFLTR_PCF_SHIFT)
#  define EMAC_FRAMEFLTR_PCF_NONE      (0 << EMAC_FRAMEFLTR_PCF_SHIFT) /* Prevents all control frames */
#  define EMAC_FRAMEFLTR_PCF_PAUSE     (1 << EMAC_FRAMEFLTR_PCF_SHIFT) /* Prevents all except PAUSE */
#  define EMAC_FRAMEFLTR_PCF_ALL       (2 << EMAC_FRAMEFLTR_PCF_SHIFT) /* Forward all control frames */
#  define EMAC_FRAMEFLTR_PCF_FILTER    (3 << EMAC_FRAMEFLTR_PCF_SHIFT) /* Forwards all that pass address filter */

#define EMAC_FRAMEFLTR_SAIF            (1 << 8)  /* Bit 8:  Source Address (SA) Inverse Filtering */
#define EMAC_FRAMEFLTR_SAF             (1 << 9)  /* Bit 9:  Source Address Filter Enable */
#define EMAC_FRAMEFLTR_HPF             (1 << 10) /* Bit 10: Hash or Perfect Filter */
#define EMAC_FRAMEFLTR_VTFE            (1 << 16) /* Bit 16: VLAN Tag Filter Enable */
#define EMAC_FRAMEFLTR_RA              (1 << 31) /* Bit 31: Receive All */

/* Ethernet MAC Hash Table High (32-bit data) */

/* Ethernet MAC Hash Table Low (32-bit data) */

/* Ethernet MAC MII Address */

#define EMAC_MIIADDR_MIIB              (1 << 0)  /* Bit 0:  MII Busy */
#define EMAC_MIIADDR_MIIW              (1 << 1)  /* Bit 1:  MII Write */
#define EMAC_MIIADDR_CR_SHIFT          (2)       /* Bits 2-5: Clock Reference Frequency Selection */
#define EMAC_MIIADDR_CR_MASK           (15 << EMAC_MIIADDR_CR_SHIFT)
#  define EMAC_MIIADDR_CR_60_100       (0 << EMAC_MIIADDR_CR_SHIFT) /* System Clock=60-100 MHz; MDIO clock=SYSCLK/42 */
#  define EMAC_MIIADDR_CR_100_150      (1 << EMAC_MIIADDR_CR_SHIFT) /* System Clock=100-150 MHz; MDIO clock=SYSCLK/62 */
#  define EMAC_MIIADDR_CR_20_35        (2 << EMAC_MIIADDR_CR_SHIFT) /* System Clock=20-35 MHz; MDIO clock=SYSCLK/16 */
#  define EMAC_MIIADDR_CR_35_60        (3 << EMAC_MIIADDR_CR_SHIFT) /* System Clock=35-60 MHz; MDIO clock=SYSCLK/26 */
#  define EMAC_MIIADDR_CR_150_168      (4 << EMAC_MIIADDR_CR_SHIFT) /* System Clock=150-168 MHz; MDIO clock=SYSCLK/102 */

#define EMAC_MIIADDR_MII_SHIFT         (6)       /* Bits 6-10: MII Register */
#define EMAC_MIIADDR_MII_MASK          (31 << EMAC_MIIADDR_MII_SHIFT)
#  define EMAC_MIIADDR_MII(n)          ((uint32_t)(n) << EMAC_MIIADDR_MII_SHIFT)
#define EMAC_MIIADDR_PLA_SHIFT         (11)      /* Bits 11-15: Physical Layer Address */
#define EMAC_MIIADDR_PLA_MASK          (31 << EMAC_MIIADDR_PLA_SHIFT)
#  define EMAC_MIIADDR_PLA(n)          ((uint32_t)(n) << EMAC_MIIADDR_PLA_SHIFT)

/* Ethernet MAC MII Data Register */

#define EMAC_MIIDATA_SHIFT             (0)       /* Bit 0-15: MII Data */
#define EMAC_MIIDATA_MASK              (0xffff << EMAC_MIIDATA_SHIFT)

/* Ethernet MAC Flow Control */

#define EMAC_FLOWCTL_FCBBPA            (1 << 0)  /* Bit 0:  Flow Control Busy or Back-pressure Activate */
#define EMAC_FLOWCTL_TFE               (1 << 1)  /* Bit 1:  Transmit Flow Control Enable */
#define EMAC_FLOWCTL_RFE               (1 << 2)  /* Bit 2:  Receive Flow Control Enable */
#define EMAC_FLOWCTL_UP                (1 << 3)  /* Bit 3:  Unicast Pause Frame Detect */
#define EMAC_FLOWCTL_PLT_SHIFT         (4)       /* Bits 4-5: Pause Low Threshold */
#define EMAC_FLOWCTL_PLT_MASK          (3 << EMAC_FLOWCTL_PLT_SHIFT)
#  define EMAC_FLOWCTL_PLT_M4          (0 << EMAC_FLOWCTL_PLT_SHIFT) /* Pause time minus 4 slot times */
#  define EMAC_FLOWCTL_PLT_M28         (1 << EMAC_FLOWCTL_PLT_SHIFT) /* Pause time minus 28 slot times */
#  define EMAC_FLOWCTL_PLT_M144        (2 << EMAC_FLOWCTL_PLT_SHIFT) /* Pause time minus 144 slot times */
#  define EMAC_FLOWCTL_PLT_M256        (3 << EMAC_FLOWCTL_PLT_SHIFT) /* Pause time minus 256 slot times */

#define EMAC_FLOWCTL_DZQP              (1 << 7)  /* Bit 7:  Disable Zero-Quanta Pause */
#define EMAC_FLOWCTL_PT_SHIFT          (16)      /* Bits 16-31: Pause Time */
#define EMAC_FLOWCTL_PT_MASK           (0xffff << EMAC_FLOWCTL_PT_SHIFT)
#  define EMAC_FLOWCTL_PT(n)           ((uint32_t)(n) << EMAC_FLOWCTL_PT_SHIFT)

/* Ethernet MAC VLAN Tag */

#define EMAC_VLANTG_VL_SHIFT           (0)       /* Bits 0-15: VLAN Tag Identifier for Receive Frames */
#define EMAC_VLANTG_VL_MASK            (0xffff << EMAC_VLANTG_VL_SHIFT)
#define EMAC_VLANTG_ETV                (1 << 16) /* Bit 16: Enable 12-Bit VLAN Tag Comparison */
#define EMAC_VLANTG_VTIM               (1 << 17) /* Bit 17: VLAN Tag Inverse Match Enable */
#define EMAC_VLANTG_ESVL               (1 << 18) /* Bit 18: Enable S-VLAN */
#define EMAC_VLANTG_VTHM               (1 << 19) /* Bit 19: VLAN Tag Hash Table Match Enable */

/* Ethernet MAC Status */

#define EMAC_STATUS_RPE                (1 << 0)  /* Bit 0:  MAC MII Receive Protocol Engine Status */
#define EMAC_STATUS_RFCFC_SHIFT        (1)       /* Bits 1-2: MAC Receive Frame Controller FIFO Status */
#define EMAC_STATUS_RFCFC_MASK         (3 << EMAC_STATUS_RFCFC_SHIFT)
#define EMAC_STATUS_RWC                (1 << 4)  /* Bit 4:  TX/RX Controller RX FIFO Write Controller Active Status */
#define EMAC_STATUS_RRC_SHIFT          (5)       /* Bits 5-6: TX/RX Controller Read Controller State */
#define EMAC_STATUS_RRC_MASK           (3 << EMAC_STATUS_RRC_SHIFT)
#  define EMAC_STATUS_RRC_IDLE         (0 << EMAC_STATUS_RRC_SHIFT) /* IDLE state */
#  define EMAC_STATUS_RRC_STATUS       (1 << EMAC_STATUS_RRC_SHIFT) /* Reading frame data */
#  define EMAC_STATUS_RRC_DATA         (2 << EMAC_STATUS_RRC_SHIFT) /* Reading frame status (or timestamp) */
#  define EMAC_STATUS_RRC_FLUSH        (3 << EMAC_STATUS_RRC_SHIFT) /* Flushing the frame data and status */

#define EMAC_STATUS_RXF_SHIFT          (8)       /* Bits 8-9: TX/RX Controller RX FIFO Fill-level Status */
#define EMAC_STATUS_RXF_MASK           (3 << EMAC_STATUS_RXF_SHIFT)
#  define EMAC_STATUS_RXF_EMPTY        (0 << EMAC_STATUS_RXF_SHIFT) /* RX FIFO Empty */
#  define EMAC_STATUS_RXF_BELOW        (1 << EMAC_STATUS_RXF_SHIFT) /* Below the flow-control deactivate threshold */
#  define EMAC_STATUS_RXF_ABOVE        (2 << EMAC_STATUS_RXF_SHIFT) /* Above the flow-control activate threshold */
#  define EMAC_STATUS_RXF_FULL         (3 << EMAC_STATUS_RXF_SHIFT) /* RX FIFO Full */

#define EMAC_STATUS_TPE                (1 << 16) /* Bit 16: MAC MII Transmit Protocol Engine Status */
#define EMAC_STATUS_TFC_SHIFT          (17)      /* Bits 17-18: MAC Transmit Frame Controller Status */
#define EMAC_STATUS_TFC_MASK           (3 << EMAC_STATUS_TFC_SHIFT)
#  define EMAC_STATUS_TFC_IDLE         (0 << EMAC_STATUS_TFC_SHIFT) /* IDLE state */
#  define EMAC_STATUS_TFC_STATUS       (1 << EMAC_STATUS_TFC_SHIFT) /* Waiting for status */
#  define EMAC_STATUS_TFC_PAUSE        (2 << EMAC_STATUS_TFC_SHIFT) /* Generating and transmitting a PAUSE control frame */
#  define EMAC_STATUS_TFC_INPUT        (3 << EMAC_STATUS_TFC_SHIFT) /* Transferring input frame for transmission */

#define EMAC_STATUS_TXPAUSED           (1 << 19) /* Bit 19: MAC Transmitter PAUSE */
#define EMAC_STATUS_TRC_SHIFT          (20)      /* Bits 20-21: TX/RX Controller's TX FIFO Read Controller Status */
#define EMAC_STATUS_TRC_MASK           (3 << EMAC_STATUS_TRC_SHIFT)
#  define EMAC_STATUS_TRC_IDLE         (0 << EMAC_STATUS_TRC_SHIFT) /* IDLE state */
#  define EMAC_STATUS_TRC_READ         (1 << EMAC_STATUS_TRC_SHIFT) /* READ state */
#  define EMAC_STATUS_TRC_WAIT         (2 << EMAC_STATUS_TRC_SHIFT) /* Waiting for TX Status from MAC transmitter */
#  define EMAC_STATUS_TRC_WRFLUSH      (3 << EMAC_STATUS_TRC_SHIFT) /* Writing received TX Status or flushing TX FIFO */

#define EMAC_STATUS_TWC                (1 << 22) /* Bit 22: TX/RX Controller TX FIFO Write Controller Active Status */
#define EMAC_STATUS_TXFE               (1 << 24) /* Bit 24: TX/RX Controller TX FIFO Not Empty Status */
#define EMAC_STATUS_TXFF               (1 << 25) /* Bit 25: TX/RX Controller TX FIFO Full Status */

/* Ethernet MAC Remote Wake-Up Frame Filter (32-bit data) */

/* Ethernet MAC PMT Control and Status Register */

#define EMAC_PMTCTLSTAT_PWRDWN         (1 << 0)  /* Bit 0:  Power Down */
#define EMAC_PMTCTLSTAT_MGKPKTEN       (1 << 1)  /* Bit 1:  Magic Packet Enable */
#define EMAC_PMTCTLSTAT_WUPFREN        (1 << 2)  /* Bit 2:  Wake-Up Frame Enable */
#define EMAC_PMTCTLSTAT_MGKPRX         (1 << 5)  /* Bit 3:  Magic Packet Received */
#define EMAC_PMTCTLSTAT_WUPRX          (1 << 6)  /* Bit 6:  Wake-Up Frame Received */
#define EMAC_PMTCTLSTAT_GLBLUCAST      (1 << 9)  /* Bit 7:  Global Unicast */
#define EMAC_PMTCTLSTAT_RWKPTR_SHIFT   (24)      /* Bits 24-26:  Remote Wake-Up FIFO Pointer */
#define EMAC_PMTCTLSTAT_RWKPTR_MASK    (7 << EMAC_PMTCTLSTAT_RWKPTR_SHIFT)
#  define EMAC_PMTCTLSTAT_RWKPTR(n)    ((uint32_t)(n) << EMAC_PMTCTLSTAT_RWKPTR_SHIFT)
#define EMAC_PMTCTLSTAT_WUPFRRST       (1 << 31) /* Bit 31: Wake-Up Frame Filter Register Pointer Reset */

/* Ethernet MAC Raw Interrupt Status */

#define EMAC_RIS_PMT                   (1 << 3)  /* Bit 3:  PMT Interrupt Status */
#define EMAC_RIS_MMC                   (1 << 4)  /* Bit 4:  MMC Interrupt Status */
#define EMAC_RIS_MMCRX                 (1 << 5)  /* Bit 5:  MMC Receive Interrupt Status */
#define EMAC_RIS_MMCTX                 (1 << 6)  /* Bit 6:  MMC Transmit Interrupt Status */
#define EMAC_RIS_TS                    (1 << 9)  /* Bit 9:  Timestamp Interrupt Status */

/* Ethernet MAC Interrupt Mask */

#define EMAC_IM_PMT                    (1 << 3)  /* Bit 3:  PMT Interrupt Mask */
#define EMAC_IM_TSI                    (1 << 9)  /* Bit 9:  Timestamp Interrupt Mask */
#define EMAC_IM_ALLINTS                (EMAC_IM_PMT|EMAC_IM_TSI)

/* Ethernet MAC Address 0 High */

/* Ethernet MAC Address 0 Low Register (32-bit MAC Address0 [31:0]) */

#define EMAC_ADDR0H_ADDRHI_SHIFT       (0)       /* Bits 0-15: MAC Address0 [47:32] */
#define EMAC_ADDR0H_ADDRHI_MASK        (0xffff << EMAC_ADDR0H_ADDRHI_SHIFT)
#  define EMAC_ADDR0H_ADDRHI(n)        ((uint32_t)(n) << EMAC_ADDR0H_ADDRHI_SHIFT)
#define EMAC_ADDR0H_AE                 (1 << 31) /* Bit 31: Address Enable */

/* Ethernet MAC Address 1 High */

/* Ethernet MAC Address 1 Low  (32-bit MAC Address0 [31:0]) */

#define EMAC_ADDR1H_ADDRHI_SHIFT       (0)       /* Bits 0-15: MAC Address0 [47:32] */
#define EMAC_ADDR1H_ADDRHI_MASK        (0xffff << EMAC_ADDR1H_ADDRHI_SHIFT)
#  define EMAC_ADDR1H_ADDRHI(n)        ((uint32_t)(n) << EMAC_ADDR1H_ADDRHI_SHIFT)
#define EMAC_ADDR1H_MBC_SHIFT          (24)      /* Bits 24-29: Mask Byte Control */
#define EMAC_ADDR1H_MBC_MASK           (0x3f << EMAC_ADDR1H_MBC_SHIFT)
#  define EMAC_ADDR1H_MBC(n)           ((uint32_t)(n) << EMAC_ADDR1H_MBC_SHIFT)
#  define EMAC_ADDR1H_EMACADDR1L_0     (1 << 24) /* ADDRLO [7:0] of EMACADDR1L register */
#  define EMAC_ADDR1H_EMACADDR1L_8     (1 << 25) /* ADDRLO [15:8] of EMACADDR1L register */
#  define EMAC_ADDR1H_EMACADDR1L_16    (1 << 26) /* ADDRLO [23:16] of EMACADDR1L register */
#  define EMAC_ADDR1H_EMACADDR1L_24    (1 << 27) /* ADDRLO [31:24] of EMACADDR1L register */
#  define EMAC_ADDR1H_EMACADDR1H_0     (1 << 28) /* ADDRHI [7:0] of EMACADDR1H */
#  define EMAC_ADDR1H_EMACADDR1H_8     (1 << 29) /* ADDRHI [15:8] of EMACADDR1H Register */
#define EMAC_ADDR1H_SA                 (1 << 30) /* Bit 30: Source Address */
#define EMAC_ADDR1H_AE                 (1 << 31) /* Bit 31: Address Enable */

/* Ethernet MAC Address 2 High */

/* Ethernet MAC Address 2 Low  (32-bit MAC Address0 [31:0]) */

#define EMAC_ADDR2H_ADDRHI_SHIFT       (0)       /* Bits 0-15: MAC Address0 [47:32] */
#define EMAC_ADDR2H_ADDRHI_MASK        (0xffff << EMAC_ADDR2H_ADDRHI_SHIFT)
#  define EMAC_ADDR2H_ADDRHI(n)        ((uint32_t)(n) << EMAC_ADDR2H_ADDRHI_SHIFT)
#define EMAC_ADDR2H_MBC_SHIFT          (24)      /* Bits 24-29: Mask Byte Control */
#define EMAC_ADDR2H_MBC_MASK           (0x3f << EMAC_ADDR2H_MBC_SHIFT)
#  define EMAC_ADDR2H_MBC(n)           ((uint32_t)(n) << EMAC_ADDR2H_MBC_SHIFT)
#  define EMAC_ADDR2H_EMACADDR2L_0     (1 << 24) /* ADDRLO [7:0] of EMACADDR2L register */
#  define EMAC_ADDR2H_EMACADDR2L_8     (1 << 25) /* ADDRLO [15:8] of EMACADDR2L register */
#  define EMAC_ADDR2H_EMACADDR2L_16    (1 << 26) /* ADDRLO [23:16] of EMACADDR2L register */
#  define EMAC_ADDR2H_EMACADDR2L_24    (1 << 27) /* ADDRLO [31:24] of EMACADDR2L register */
#  define EMAC_ADDR2H_EMACADDR2H_0     (1 << 28) /* ADDRHI [7:0] of EMACADDR2H */
#  define EMAC_ADDR2H_EMACADDR2H_8     (1 << 29) /* ADDRHI [15:8] of EMACADDR2H Register */
#define EMAC_ADDR2H_SA                 (1 << 30) /* Bit 30: Source Address */
#define EMAC_ADDR2H_AE                 (1 << 31) /* Bit 31: Address Enable */

/* Ethernet MAC Address 3 High */

/* Ethernet MAC Address 3 Low  (32-bit MAC Address0 [31:0]) */

#define EMAC_ADDR3H_ADDRHI_SHIFT       (0)       /* Bits 0-15: MAC Address0 [47:32] */
#define EMAC_ADDR3H_ADDRHI_MASK        (0xffff << EMAC_ADDR3H_ADDRHI_SHIFT)
#  define EMAC_ADDR3H_ADDRHI(n)        ((uint32_t)(n) << EMAC_ADDR3H_ADDRHI_SHIFT)
#define EMAC_ADDR3H_MBC_SHIFT          (24)      /* Bits 24-29: Mask Byte Control */
#define EMAC_ADDR3H_MBC_MASK           (0x3f << EMAC_ADDR3H_MBC_SHIFT)
#  define EMAC_ADDR3H_MBC(n)           ((uint32_t)(n) << EMAC_ADDR3H_MBC_SHIFT)
#  define EMAC_ADDR3H_EMACADDR3L_0     (1 << 24) /* ADDRLO [7:0] of EMACADDR3L register */
#  define EMAC_ADDR3H_EMACADDR3L_8     (1 << 25) /* ADDRLO [15:8] of EMACADDR3L register */
#  define EMAC_ADDR3H_EMACADDR3L_16    (1 << 26) /* ADDRLO [23:16] of EMACADDR3L register */
#  define EMAC_ADDR3H_EMACADDR3L_24    (1 << 27) /* ADDRLO [31:24] of EMACADDR3L register */
#  define EMAC_ADDR3H_EMACADDR3H_0     (1 << 28) /* ADDRHI [7:0] of EMACADDR3H */
#  define EMAC_ADDR3H_EMACADDR3H_8     (1 << 29) /* ADDRHI [15:8] of EMACADDR3H Register */
#define EMAC_ADDR3H_SA                 (1 << 30) /* Bit 30: Source Address */
#define EMAC_ADDR3H_AE                 (1 << 31) /* Bit 31: Address Enable */

/* Ethernet MAC Watchdog Timeout */

#define EMAC_WDOGTO_WTO_SHIFT          (0)       /* Bits 0-13: Watchdog Timeout */
#define EMAC_WDOGTO_WTO_MASK           (0x3fff << EMAC_WDOGTO_WTO_SHIFT)
#  define EMAC_WDOGTO_WTO(n)           ((uint32_t)(n) << EMAC_WDOGTO_WTO_SHIFT)
#define EMAC_WDOGTO_PWE                (1 << 16) /* Bit 16: Programmable Watchdog Enable */

/* Ethernet MAC MMC Control */

#define EMAC_MMCCTRL_CNTRST            (1 << 0)  /* Bit 0:  Counters Reset */
#define EMAC_MMCCTRL_CNTSTPRO          (1 << 1)  /* Bit 1:  Counters Stop Rollover */
#define EMAC_MMCCTRL_RSTONRD           (1 << 2)  /* Bit 2:  Reset on Read */
#define EMAC_MMCCTRL_CNTFREEZ          (1 << 3)  /* Bit 3:  MMC Counter Freeze */
#define EMAC_MMCCTRL_CNTPRST           (1 << 4)  /* Bit 4:  Counters Preset */
#define EMAC_MMCCTRL_CNTPRSTLVL        (1 << 5)  /* Bit 5:  Full/Half Preset Level Value */
#define EMAC_MMCCTRL_UCDBC             (1 << 8)  /* Bit 8:  Update MMC Counters for Dropped Broadcast Frames */

/* Ethernet MAC MMC Receive Raw Interrupt Status */

#define EMAC_MMCRXRIS_GBF              (1 << 0)  /* Bit 0:  MMC Receive Good Bad Frame Counter Interrupt Status */
#define EMAC_MMCRXRIS_CRCERR           (1 << 5)  /* Bit 5:  MMC Receive CRC Error Frame Counter Interrupt Status */
#define EMAC_MMCRXRIS_ALGNERR          (1 << 6)  /* Bit 6:  MMC Receive Alignment Error Frame Counter Interrupt Status */
#define EMAC_MMCRXRIS_UCGF             (1 << 17) /* Bit 17: MMC Receive Unicast Good Frame Counter Interrupt Status */

/* Ethernet MAC MMC Transmit Raw Interrupt Status */

#define EMAC_MMCTXRIS_GBF              (1 << 1)  /* Bit 1:  MMC Transmit Good Bad Frame Counter Interrupt Status */
#define EMAC_MMCTXRIS_SCOLLGF          (1 << 14) /* Bit 14: MMC Transmit Single Collision Good Frame Counter Interrupt Status */
#define EMAC_MMCTXRIS_MCOLLGF          (1 << 15) /* Bit 15: MMC Transmit Multiple Collision Good Frame Counter Interrupt Status */
#define EMAC_MMCTXRIS_OCTCNT           (1 << 20) /* Bit 20: Octet Counter Interrupt Status */

/* Ethernet MAC MMC Receive Interrupt Mask */

#define EMAC_MMCRXIM_GBF               (1 << 0)  /* Bit 0:  MMC Receive Good Bad Frame Counter Interrupt Mask */
#define EMAC_MMCRXIM_CRCERR            (1 << 5)  /* Bit 5:  MMC Receive CRC Error Frame Counter Interrupt Mask */
#define EMAC_MMCRXIM_ALGNERR           (1 << 6)  /* Bit 6:  MMC Receive Alignment Error Frame Counter Interrupt Mask */
#define EMAC_MMCRXIM_UCGF              (1 << 17) /* Bit 17: MMC Receive Unicast Good Frame Counter Interrupt Mask */

/* Ethernet MAC MMC Transmit Interrupt Mask */

#define EMAC_MMCTXIM_GBF               (1 << 1)  /* Bit 1:  MMC Transmit Good Bad Frame Counter Interrupt Mask */
#define EMAC_MMCTXIM_SCOLLGF           (1 << 14) /* Bit 14: MMC Transmit Single Collision Good Frame Counter Interrupt Mask */
#define EMAC_MMCTXIM_MCOLLGF           (1 << 15) /* Bit 15: MMC Transmit Multiple Collision Good Frame Counter Interrupt Mask */
#define EMAC_MMCTXIM_OCTCNT            (1 << 20) /* Bit 20: MMC Transmit Good Octet Counter Interrupt Mask */

/* Ethernet MAC Transmit Frame Count for Good and Bad Frames (32-bit data) */

/* Ethernet MAC Transmit Frame Count for Frames Transmitted after
 * Single Collision (32-bit data)
 */

/* Ethernet MAC Transmit Frame Count for Frames Transmitted after
 * Multiple Collisions (32-bit data)
 */

/* Ethernet MAC Transmit Octet Count Good (32-bit data) */

/* Ethernet MAC Receive Frame Count for Good and Bad Frames (32-bit data) */

/* Ethernet MAC Receive Frame Count for CRC Error Frames (32-bit data) */

/* Ethernet MAC Receive Frame Count for Alignment Error Frames
 * (32-bit data)
 */

/* Ethernet MAC Receive Frame Count for Good Unicast Frames (32-bit data) */

/* Ethernet MAC VLAN Tag Inclusion or Replacement */

#define EMAC_VLNINCREP_VLT_SHIFT       (0)       /* Bits 0-15: VLAN Tag for Transmit Frames */
#define EMAC_VLNINCREP_VLT_MASK        (0xffff << EMAC_VLNINCREP_VLT_SHIFT)
#  define EMAC_VLNINCREP_VLT(n)        ((uint32_t)(n) << EMAC_VLNINCREP_VLT_SHIFT)
#define EMAC_VLNINCREP_VLC_SHIFT       (16)      /* Bits 16-17: VLAN Tag Control in Transmit Frames */
#define EMAC_VLNINCREP_VLC_MASK        (3 << EMAC_VLNINCREP_VLC_SHIFT)
#  define EMAC_VLNINCREP_VLC_NONE      (0 << EMAC_VLNINCREP_VLC_SHIFT) /* No VLAN tag deletion, insertion, or replacement */
#  define EMAC_VLNINCREP_VLC_TAGDEL    (1 << EMAC_VLNINCREP_VLC_SHIFT) /* VLAN tag deletion */
#  define EMAC_VLNINCREP_VLC_TAGINS    (2 << EMAC_VLNINCREP_VLC_SHIFT) /* VLAN tag insertion */
#  define EMAC_VLNINCREP_VLC_TAGREP    (3 << EMAC_VLNINCREP_VLC_SHIFT) /* VLAN tag replacement */

#define EMAC_VLNINCREP_VLP             (1 << 18) /* Bit 18: VLAN Priority Control */
#define EMAC_VLNINCREP_CSVL            (1 << 19) /* Bit 19: C-VLAN or S-VLAN */

/* Ethernet MAC VLAN Hash Table */

#define EMAC_VLANHASH_VLHT_SHIFT       (0)       /* Bits 0-15: VLAN Hash Table */
#define EMAC_VLANHASH_VLHT_MASK        (0xffff << EMAC_VLANHASH_VLHT_SHIFT)

/* Ethernet MAC Timestamp Control */

#define EMAC_TIMSTCTRL_TSEN            (1 << 0)  /* Bit 0:  Timestamp Enable */
#define EMAC_TIMSTCTRL_TSFCUPDT        (1 << 1)  /* Bit 1:  Timestamp Fine or Coarse Update */
#define EMAC_TIMSTCTRL_TSINIT          (1 << 2)  /* Bit 2:  Timestamp Initialize */
#define EMAC_TIMSTCTRL_TSUPDT          (1 << 3)  /* Bit 3:  Timestamp Update */
#define EMAC_TIMSTCTRL_INTTRIG         (1 << 4)  /* Bit 4:  Timestamp Interrupt Trigger Enable */
#define EMAC_TIMSTCTRL_ADDREGUP        (1 << 5)  /* Bit 5:  Addend Register Update */
#define EMAC_TIMSTCTRL_ALLF            (1 << 8)  /* Bit 8:  Enable Timestamp For All Frames */
#define EMAC_TIMSTCTRL_DGTLBIN         (1 << 9)  /* Bit 9:  Timestamp Digital or Binary Rollover Control */
#define EMAC_TIMSTCTRL_PTPVER2         (1 << 10) /* Bit 10: Enable PTP Packet Processing For Version 2 Format */
#define EMAC_TIMSTCTRL_PTPETH          (1 << 11) /* Bit 11: Enable Processing of PTP Over Ethernet Frames */
#define EMAC_TIMSTCTRL_PTPIPV6         (1 << 12) /* Bit 12: Enable Processing of PTP Frames Sent Over IPv6-UDP */
#define EMAC_TIMSTCTRL_PTPIPV4         (1 << 13) /* Bit 13: Enable Processing of PTP Frames Sent over IPv4-UDP */
#define EMAC_TIMSTCTRL_TSEVNT          (1 << 14) /* Bit 14: Enable Timestamp Snapshot for Event Messages */
#define EMAC_TIMSTCTRL_TSMAST          (1 << 15) /* Bit 15: Enable Snapshot for Messages Relevant to Master */
#define EMAC_TIMSTCTRL_SELPTP_SHIFT    (16)      /* Bits 16-17: Select PTP packets for Taking Snapshots */
#define EMAC_TIMSTCTRL_SELPTP_MASK     (3 << EMAC_TIMSTCTRL_SELPTP_SHIFT)
#  define EMAC_TIMSTCTRL_SELPTP(n)     ((uint32_t)(n) << EMAC_TIMSTCTRL_SELPTP_SHIFT)
#define EMAC_TIMSTCTRL_PTPFLTR         (1 << 18) /* Bit 18: Enable MAC address for PTP Frame Filtering */

/* Ethernet MAC Sub-Second Increment */

#define EMAC_SUBSECINC_SSINC_SHIFT     (0)       /* Bits 0-7: Sub-second Increment Value */
#define EMAC_SUBSECINC_SSINC_MASK      (0xff << EMAC_SUBSECINC_SSINC_SHIFT)

/* Ethernet MAC System Time - Seconds (32-bit value) */

/* Ethernet MAC System Time - Nanoseconds */

#define EMAC_TIMNANO_TSSS_SHIFT        (0)       /* Bits 0-30: Timestamp Sub-Seconds */
#define EMAC_TIMNANO_TSSS_MASK         (0x7fffffff << EMAC_TIMNANO_TSSS_SHIFT)

/* Ethernet MAC System Time - Seconds Update (32-bit value) */

/* Ethernet MAC System Time - Nanoseconds Update */

#define EMAC_TIMNANOU_TSSS_SHIFT       (0)       /* Bits 0-30: Timestamp Sub-Second */
#define EMAC_TIMNANOU_TSSS_MASK        (0x7fffffff << EMAC_TIMNANOU_TSSS_SHIFT)
#define EMAC_TIMNANOU_ADDSUB           (1 << 31) /* Bit 31: Add or subtract time */

/* Ethernet MAC Timestamp Addend (32-bit value) */

/* Ethernet MAC Target Time Seconds (32-bit value) */

/* Ethernet MAC Target Time Nanoseconds */

#define EMAC_TARGNANO_TTSLO_SHIFT      (0)       /* Bits 0-30: Target Timestamp Low Register */
#define EMAC_TARGNANO_TTSLO_MASK       (0x7fffffff << EMAC_TARGNANO_TTSLO_SHIFT)
#define EMAC_TARGNANO_TRGTBUSY         (1 << 31) /* Bit 31: Target Time Register Busy */

/* Ethernet MAC System Time-Higher Word Seconds */

#define EMAC_HWORDSEC_TSHWR_SHIFT      (0)       /* Bits 0-15: Target Timestamp Higher Word Register */
#define EMAC_HWORDSEC_TSHWR_MASK       (0xffff << EMAC_HWORDSEC_TSHWR_SHIFT)

/* Ethernet MAC Timestamp Status */

#define EMAC_TIMSTAT_TSSOVF            (1 << 0)  /* Bit 0:  Timestamp Seconds Overflow */
#define EMAC_TIMSTAT_TSTARGT           (1 << 1)  /* Bit 1:  Timestamp Target Time Reached */

/* Ethernet MAC PPS Control */

#define EMAC_PPSCTRL_PPSCTRL_SHIFT     (0)       /* Bits 0-3: EN0PPS Output Frequency Control (PPSCTRL) or Command Control (PPSCMD) */
#define EMAC_PPSCTRL_PPSCTRL_MASK      (15 << EMAC_PPSCTRL_PPSCTRL_SHIFT)
#  define EMAC_PPSCTRL_PPSCTRL_1HZ     (0 << EMAC_PPSCTRL_PPSCTRL_SHIFT)  /* EN0PPS signal=PTP reference clock/sec */
#  define EMAC_PPSCTRL_PPSCTRL_2HZ     (1 << EMAC_PPSCTRL_PPSCTRL_SHIFT)  /* Binary rollover=2 Hz; digital rollover=1 Hz */
#  define EMAC_PPSCTRL_PPSCTRL_4HZ     (2 << EMAC_PPSCTRL_PPSCTRL_SHIFT)  /* Binary rollover=4 Hz; digital rollover=2 Hz */
#  define EMAC_PPSCTRL_PPSCTRL_8HZ     (3 << EMAC_PPSCTRL_PPSCTRL_SHIFT)  /* Binary rollover=8 Hz; digital rollover=4 Hz, */
#  define EMAC_PPSCTRL_PPSCTRL_16HZ    (4 << EMAC_PPSCTRL_PPSCTRL_SHIFT)  /* Binary rollover=16 Hz; digital rollover=8 Hz */
#  define EMAC_PPSCTRL_PPSCTRL_32HZ    (5 << EMAC_PPSCTRL_PPSCTRL_SHIFT)  /* Binary rollover=32 Hz; digital rollover=16 Hz */
#  define EMAC_PPSCTRL_PPSCTRL_64HZ    (6 << EMAC_PPSCTRL_PPSCTRL_SHIFT)  /* Binary rollover=64 Hz; digital rollover=32 Hz */
#  define EMAC_PPSCTRL_PPSCTRL_128HZ   (7 << EMAC_PPSCTRL_PPSCTRL_SHIFT)  /* Binary rollover=128 Hz; digital rollover=64 Hz */
#  define EMAC_PPSCTRL_PPSCTRL_256HZ   (8 << EMAC_PPSCTRL_PPSCTRL_SHIFT)  /* binary rollover=256 Hz; digital rollover=128 Hz */
#  define EMAC_PPSCTRL_PPSCTRL_512HZ   (9 << EMAC_PPSCTRL_PPSCTRL_SHIFT)  /* binary rollover=512 Hz; digital rollover=256 Hz */
#  define EMAC_PPSCTRL_PPSCTRL_1024HZ  (10 << EMAC_PPSCTRL_PPSCTRL_SHIFT) /* binary rollover=1.024 kHz; digital rollover=512 Hz */
#  define EMAC_PPSCTRL_PPSCTRL_2048HZ  (11 << EMAC_PPSCTRL_PPSCTRL_SHIFT) /* Binary rollover=2.048 kHz; digital rollover=1.024 kHz */
#  define EMAC_PPSCTRL_PPSCTRL_4096HZ  (12 << EMAC_PPSCTRL_PPSCTRL_SHIFT) /* Binary rollover=4.096 kHz; digital rollover=2.048 kHz */
#  define EMAC_PPSCTRL_PPSCTRL_8192HZ  (13 << EMAC_PPSCTRL_PPSCTRL_SHIFT) /* Binary rollover=8.192 kHz; digital rollover=4.096 kHz */
#  define EMAC_PPSCTRL_PPSCTRL_16384HZ (14 << EMAC_PPSCTRL_PPSCTRL_SHIFT) /* Binary rollover=16.384 kHz; digital rollover=8.092 kHz */
#  define EMAC_PPSCTRL_PPSCTRL_32768HZ (15 << EMAC_PPSCTRL_PPSCTRL_SHIFT) /* Binary rollover=32.768 KHz; digital rollover=16.384 KHz */

#define EMAC_PPSCTRL_PPSEN0            (1 << 4)  /* Bit 4:  Flexible PPS Output Mode Enable */
#define EMAC_PPSCTRL_TRGMODS0_SHIFT    (5)       /* Bits 5-6: Target Time Register Mode for PPS0 Output */
#define EMAC_PPSCTRL_TRGMODS0_MASK     (3 << EMAC_PPSCTRL_TRGMODS0_SHIFT)
#  define EMAC_PPSCTRL_TRGMODS0_INTONLY  (0 << EMAC_PPSCTRL_TRGMODS0_SHIFT) /* Generate the interrupt event */
#  define EMAC_PPSCTRL_TRGMODS0_INTPPS0  (2 << EMAC_PPSCTRL_TRGMODS0_SHIFT) /* Generate the interrupt event and control EN0PPS */
#  define EMAC_PPSCTRL_TRGMODS0_PPS0ONLY (3 << EMAC_PPSCTRL_TRGMODS0_SHIFT) /* Control ENOPPS output */

/* Ethernet MAC PPS0 Interval (32-bit value) */

/* Ethernet MAC PPS0 Width (32-bit value) */

/* Ethernet MAC DMA Bus Mode */

#define EMAC_DMABUSMOD_SWR             (1 << 0)  /* Bit 0:  DMA Software Reset */
#define EMAC_DMABUSMOD_DA              (1 << 1)  /* Bit 1:  DMA Arbitration Scheme */
#define EMAC_DMABUSMOD_DSL_SHIFT       (2)       /* Bits 2-6: Descriptor Skip Length */
#define EMAC_DMABUSMOD_DSL_MASK        (31 << EMAC_DMABUSMOD_DSL_SHIFT)
#  define EMAC_DMABUSMOD_DSL(n)        ((uint32_t)(n) << EMAC_DMABUSMOD_DSL_SHIFT)
#define EMAC_DMABUSMOD_ATDS            (1 << 7)  /* Bit 7:  Alternate Descriptor Size */
#define EMAC_DMABUSMOD_PBL_SHIFT       (8)       /* Bits 8-13: Programmable Burst Length */
#define EMAC_DMABUSMOD_PBL_MASK        (0x3f << EMAC_DMABUSMOD_PBL_SHIFT)
#  define EMAC_DMABUSMOD_PBL(n)        ((uint32_t)(n) << EMAC_DMABUSMOD_PBL_SHIFT)
#define EMAC_DMABUSMOD_PR_SHIFT        (14)      /* Bits 14-15: Priority Ratio */
#define EMAC_DMABUSMOD_PR_MASK         (3 << EMAC_DMABUSMOD_PR_SHIFT)
#  define EMAC_DMABUSMOD_PR_1TO1       (0 << EMAC_DMABUSMOD_PR_SHIFT) /* Priority Ratio is 1:1 */
#  define EMAC_DMABUSMOD_PR_2TO1       (1 << EMAC_DMABUSMOD_PR_SHIFT) /* Priority Ratio is 2:1 */
#  define EMAC_DMABUSMOD_PR_3TO1       (2 << EMAC_DMABUSMOD_PR_SHIFT) /* Priority Ratio is 3:1 */
#  define EMAC_DMABUSMOD_PR_4TO1       (3 << EMAC_DMABUSMOD_PR_SHIFT) /* Priority Ratio is 4:1 */

#define EMAC_DMABUSMOD_FB              (1 << 16) /* Bit 16: Fixed Burst */
#define EMAC_DMABUSMOD_RPBL_SHIFT      (17)      /* Bits 17:22: RX DMA Programmable Burst Length (PBL) */
#define EMAC_DMABUSMOD_RPBL_MASK       (0x3f << EMAC_DMABUSMOD_RPBL_SHIFT)
#  define EMAC_DMABUSMOD_RPBL(n)       ((uint32_t)(n) << EMAC_DMABUSMOD_RPBL_SHIFT)
#define EMAC_DMABUSMOD_USP             (1 << 23) /* Bit 23: Use Separate Programmable Burst Length (PBL) */
#define EMAC_DMABUSMOD_8XPBL           (1 << 24) /* Bit 24: 8 x Programmable Burst Length (PBL) Mode */
#define EMAC_DMABUSMOD_AAL             (1 << 25) /* Bit 25: Address Aligned Beats */
#define EMAC_DMABUSMOD_MB              (1 << 26) /* Bit 26: Mixed Burst */
#define EMAC_DMABUSMOD_TXPR            (1 << 27) /* Bit 27: Transmit Priority */
#define EMAC_DMABUSMOD_RIB             (1 << 31) /* Bit 31: Rebuild Burst */

/* Ethernet MAC Transmit Poll Demand (32-bit value) */

/* Ethernet MAC Receive Poll Demand (32-bit value) */

/* Ethernet MAC Receive Descriptor List Address */

#define EMAC_RXDLADDR_MASK             (0xfffffffc)

/* Ethernet MAC Transmit Descriptor List Address */

#define EMAC_TXDLADDR_MASK             (0xfffffffc)

/* Ethernet MAC DMA Interrupt Status and
 * Ethernet MAC DMA Interrupt Mask Register
 * (common bit definitions)
 */

#define EMAC_DMAINT_TI                 (1 << 0)  /* Bit 0:  Transmit interrupt */
#define EMAC_DMAINT_TPSI               (1 << 1)  /* Bit 1:  Transmit process stopped interrupt */
#define EMAC_DMAINT_TBUI               (1 << 2)  /* Bit 2:  Transmit buffer unavailable interrupt */
#define EMAC_DMAINT_TJTI               (1 << 3)  /* Bit 3:  Transmit jabber timeout interrupt */
#define EMAC_DMAINT_OVFI               (1 << 4)  /* Bit 4:  Overflow interrupt */
#define EMAC_EMAINT_UNFI               (1 << 5)  /* Bit 5:  Underflow interrupt */
#define EMAC_DMAINT_RI                 (1 << 6)  /* Bit 6:  Receive interrupt */
#define EMAC_DMAINT_RBUI               (1 << 7)  /* Bit 7:  Receive buffer unavailable interrupt */
#define EMAC_DMAINT_RPSI               (1 << 8)  /* Bit 8:  Receive process stopped interrupt */
#define EMAC_DMAINT_RWTI               (1 << 9)  /* Bit 9:  Receive watchdog timeout interrupt */
#define EMAC_DMAINT_ETI                (1 << 10) /* Bit 10: Early transmit interrupt */
#define EMAC_DMAINT_FBEI               (1 << 13) /* Bit 13: Fatal bus error interrupt */
#define EMAC_DMAINT_ERI                (1 << 14) /* Bit 14: Early receive interrupt */
#define EMAC_DMAINT_AIS                (1 << 15) /* Bit 15: Abnormal interrupt summary */
#define EMAC_DMAINT_NIS                (1 << 16) /* Bit 16: Normal interrupt summary */

/* Ethernet MAC DMA Interrupt Status (unique fields) */

#define EMAC_DMARIS_RS_SHIFT           (17)      /* Bits 17-19: Received Process State */
#define EMAC_DMARIS_RS_MASK            (7 << EMAC_DMARIS_RS_SHIFT)
#  define EMAC_DMARIS_RS_STOP          (0 << EMAC_DMARIS_RS_SHIFT) /* Stopped: Reset or stop receive command issued */
#  define EMAC_DMARIS_RS_RUNRXTD       (1 << EMAC_DMARIS_RS_SHIFT) /* Running: Fetching receive transfer descriptor */
#  define EMAC_DMARIS_RS_RUNRXD        (3 << EMAC_DMARIS_RS_SHIFT) /* Running: Waiting for receive packet */
#  define EMAC_DMARIS_RS_SUSPEND       (4 << EMAC_DMARIS_RS_SHIFT) /* Suspended: Receive descriptor unavailable */
#  define EMAC_DMARIS_RS_RUNCRD        (5 << EMAC_DMARIS_RS_SHIFT) /* Running: Closing receive descriptor */
#  define EMAC_DMARIS_RS_TSWS          (6 << EMAC_DMARIS_RS_SHIFT) /* Writing Timestamp */
#  define EMAC_DMARIS_RS_RUNTXD        (7 << EMAC_DMARIS_RS_SHIFT) /* Running: Transferring receive packet data from buffer */

#define EMAC_DMARIS_TS_SHIFT           (20)      /* Bits 20-22: Transmit Process State */
#define EMAC_DMARIS_TS_MASK            (7 << EMAC_DMARIS_TS_SHIFT)
#  define EMAC_DMARIS_TS_STOP          (0 << EMAC_DMARIS_TS_SHIFT) /* Stopped; Reset or Stop transmit command processed */
#  define EMAC_DMARIS_TS_RUNTXTD       (1 << EMAC_DMARIS_TS_SHIFT) /* Running; Fetching transmit transfer descriptor */
#  define EMAC_DMARIS_TS_STATUS        (2 << EMAC_DMARIS_TS_SHIFT) /* Running; Waiting for status */
#  define EMAC_DMARIS_TS_RUNTX         (3 << EMAC_DMARIS_TS_SHIFT) /* Running; Reading data from host buffer and queuing to TX FIFO */
#  define EMAC_DMARIS_TS_TSTAMP        (4 << EMAC_DMARIS_TS_SHIFT) /* Writing Timestamp */
#  define EMAC_DMARIS_TS_SUSPEND       (6 << EMAC_DMARIS_TS_SHIFT) /* Suspended; Transmit descriptor unavailable or transmit underflow */
#  define EMAC_DMARIS_TS_RUNCTD        (7 << EMAC_DMARIS_TS_SHIFT) /* Running; Closing transmit descriptor */

#define EMAC_DMARIS_AE_SHIFT           (23)      /* Bits 23-25: Access Error */
#define EMAC_DMARIS_AE_MASK            (7 << EMAC_DMARIS_AE_SHIFT)
#  define EMAC_DMARIS_AE_RXDMAWD       (0 << EMAC_DMARIS_AE_SHIFT) /* Error during RX DMA Write Data Transfer */
#  define EMAC_DMARIS_AE_TXDMARD       (3 << EMAC_DMARIS_AE_SHIFT) /* Error during TX DMA Read Data Transfer */
#  define EMAC_DMARIS_AE_RXDMADW       (4 << EMAC_DMARIS_AE_SHIFT) /* Error during RX DMA Descriptor Write Access */
#  define EMAC_DMARIS_AE_TXDMADW       (5 << EMAC_DMARIS_AE_SHIFT) /* Error during TX DMA Descriptor Write Access */
#  define EMAC_DMARIS_AE_RXDMADR       (6 << EMAC_DMARIS_AE_SHIFT) /* Error during RX DMA Descriptor Read Access */
#  define EMAC_DMARIS_AE_TXDMADR       (7 << EMAC_DMARIS_AE_SHIFT) /* Error during TX DMA Descriptor Read Access */

#define EMAC_DMARIS_MMC                (1 << 27) /* Bit 27: MAC MMC Interrupt */
#define EMAC_DMARIS_PMT                (1 << 28) /* Bit 28: MAC PMT Interrupt Status */
#define EMAC_DMARIS_TT                 (1 << 29) /* Bit 29: Timestamp Trigger Interrupt Status */

/* Ethernet MAC DMA Operation Mode */

#define EMAC_DMAOPMODE_SR              (1 << 1)  /* Bit 1:  Start or Stop Receive */
#define EMAC_DMAOPMODE_OSF             (1 << 2)  /* Bit 2:  Operate on Second Frame */
#define EMAC_DMAOPMODE_RTC_SHIFT       (3)       /* Bits 3-4: Receive Threshold Control */
#define EMAC_DMAOPMODE_RTC_MASK        (3 << EMAC_DMAOPMODE_RTC_SHIFT)
#  define EMAC_DMAOPMODE_RTC_64        (0 << EMAC_DMAOPMODE_RTC_SHIFT) /* 64 bytes */
#  define EMAC_DMAOPMODE_RTC_32        (1 << EMAC_DMAOPMODE_RTC_SHIFT) /* 32 bytes */
#  define EMAC_DMAOPMODE_RTC_96        (2 << EMAC_DMAOPMODE_RTC_SHIFT) /* 96 bytes */
#  define EMAC_DMAOPMODE_RTC_128       (3 << EMAC_DMAOPMODE_RTC_SHIFT) /* 128 bytes */

#define EMAC_DMAOPMODE_DGF             (1 << 5)  /* Bit 5:  Drop Giant Frame Enable */
#define EMAC_DMAOPMODE_FUF             (1 << 6)  /* Bit 6:  Forward Undersized Good Frames */
#define EMAC_DMAOPMODE_FEF             (1 << 7)  /* Bit 7:  Forward Error Frames */
#define EMAC_DMAOPMODE_ST              (1 << 13) /* Bit 13: Start or Stop Transmission Command */
#define EMAC_DMAOPMODE_TTC_SHIFT       (14)      /* Bits 14-16: Transmit Threshold Control */
#define EMAC_DMAOPMODE_TTC_MASK        (7 << EMAC_DMAOPMODE_TTC_SHIFT)
#  define EMAC_DMAOPMODE_TTC_64        (0 << EMAC_DMAOPMODE_TTC_SHIFT) /* 64 bytes */
#  define EMAC_DMAOPMODE_TTC_128       (1 << EMAC_DMAOPMODE_TTC_SHIFT) /* 128 bytes */
#  define EMAC_DMAOPMODE_TTC_192       (2 << EMAC_DMAOPMODE_TTC_SHIFT) /* 192 bytes */
#  define EMAC_DMAOPMODE_TTC_256       (3 << EMAC_DMAOPMODE_TTC_SHIFT) /* 256 bytes */
#  define EMAC_DMAOPMODE_TTC_40        (4 << EMAC_DMAOPMODE_TTC_SHIFT) /* 40 bytes */
#  define EMAC_DMAOPMODE_TTC_32        (5 << EMAC_DMAOPMODE_TTC_SHIFT) /* 32 bytes */
#  define EMAC_DMAOPMODE_TTC_24        (6 << EMAC_DMAOPMODE_TTC_SHIFT) /* 24 bytes */
#  define EMAC_DMAOPMODE_TTC_16        (7 << EMAC_DMAOPMODE_TTC_SHIFT) /* 16 bytes */

#define EMAC_DMAOPMODE_FTF             (1 << 20) /* Bit 20: Flush Transmit FIFO */
#define EMAC_DMAOPMODE_TSF             (1 << 21) /* Bit 21: Transmit Store and Forward */
#define EMAC_DMAOPMODE_DFF             (1 << 24) /* Bit 24: Disable Flushing of Received Frames */
#define EMAC_DMAOPMODE_RSF             (1 << 25) /* Bit 25: Receive Store and Forward */
#define EMAC_DMAOPMODE_DT              (1 << 26) /* Bit 26: Disable Dropping of TCP/IP Checksum Error Frames */

/* Ethernet MAC Missed Frame and Buffer Overflow Counter */

#define EMAC_MFBOC_MISFRMCNT_SHIFT     (0)       /* Bits 0-15: Missed Frame Counter */
#define EMAC_MFBOC_MISFRMCNT_MASK      (0xffff << EMAC_MFBOC_MISFRMCNT_SHIFT)
#define EMAC_MFBOC_MISCNTOVF           (1 << 16) /* Bit 16: Overflow bit for Missed Frame Counter */
#define EMAC_MFBOC_OVFFRMCNT_SHIFT     (17)      /* Bits 17-27: Overflow Frame Counter */
#define EMAC_MFBOC_OVFFRMCNT_MASK      (0x7ff << EMAC_MFBOC_OVFFRMCNT_SHIFT)
#define EMAC_MFBOC_OVFCNTOVF           (1 << 28) /* Bit 28: Overflow Bit for FIFO Overflow Counter */

/* Ethernet MAC Receive Interrupt Watchdog Timer */

#define EMAC_RXINTWDT_RIWT_SHIFT       (0)       /* Bits 0-7: Receive Interrupt Watchdog Timer Count */
#define EMAC_RXINTWDT_RIWT_MASK        (0xff << EMAC_RXINTWDT_RIWT_SHIFT)

/* Ethernet MAC Current Host Transmit Descriptor (32-bit value) */

/* Ethernet MAC Current Host Receive Descriptor (32-bit value) */

/* Ethernet MAC Current Host Transmit Buffer Address (32-bit value) */

/* Ethernet MAC Current Host Receive Buffer Address (32-bit value) */

/* Ethernet MAC Peripheral Property Register */

#define EMAC_PP_PHYTYPE_SHIFT          (0)       /* Bits 0-2: Ethernet PHY Type */
#define EMAC_PP_PHYTYPE_MASK           (7 << EMAC_PP_PHYTYPE_SHIFT)
#  define EMAC_PP_PHYTYPE_NONE         (0 << EMAC_PP_PHYTYPE_SHIFT) /* No PHY */
#  define EMAC_PP_PHYTYPE_FURY         (1 << EMAC_PP_PHYTYPE_SHIFT) /* Fury class PHY */
#  define EMAC_PP_PHYTYPE_TPFS         (2 << EMAC_PP_PHYTYPE_SHIFT) /* Tempest/Firestorm class PHY */
#  define EMAC_PP_PHYTYPE_SNOWFLAKE    (3 << EMAC_PP_PHYTYPE_SHIFT) /* Snowflake class PHY */

#define EMAC_PP_MACTYPE_SHIFT          (8)       /* Bits 8-11: Ethernet MAC Type */
#define EMAC_PP_MACTYPE_MASK           (7 << EMAC_PP_MACTYPE_SHIFT)
#  define EMAC_PP_MACTYPE_LM3S         (0 << EMAC_PP_MACTYPE_SHIFT) /* Stellaris® LM3S-class MAC */
#  define EMAC_PP_MACTYPE_TM4C129X     (1 << EMAC_PP_MACTYPE_SHIFT) /* Tiva TM4E129x-class MAC */

/* Ethernet MAC Peripheral Configuration Register */

#define EMAC_PC_PHYHOLD                (1 << 0)  /* Bit 0:  Ethernet PHY Hold */
#define EMAC_PC_ANMODE_SHIFT           (1)       /* Bits 1-2: Auto Negotiation Mode */
#define EMAC_PC_ANMODE_MASK            (3 << EMAC_PC_ANMODE_SHIFT)
#  define EMAC_PC_ANMODE_10HD          (0 << EMAC_PC_ANMODE_SHIFT)  /* When ANEN = 0x0, mode is 10Base-T, Half-Duplex */
#  define EMAC_PC_ANMODE_10FD          (1 << EMAC_PC_ANMODE_SHIFT)  /* When ANEN = 0x0, mode is 10Base-T, Full-Duplex */
#  define EMAC_PC_ANMODE_100HD         (2 << EMAC_PC_ANMODE_SHIFT)  /* When ANEN = 0x0, mode is 100Base-TX, Half-Duplex */
#  define EMAC_PC_ANMODE_100FD         (3 << EMAC_PC_ANMODE_SHIFT)  /* When ANEN = 0x0, mode is 100Base-TX, Full-Duplex */

#define EMAC_PC_ANEN                   (1 << 3)  /* Bit 3:  Auto Negotiation Enable */
#define EMAC_PC_FASTANSEL_SHIFT        (4)       /* Bits 4-5: Fast Auto Negotiation Select */
#define EMAC_PC_FASTANSEL_MASK         (3 << EMAC_PC_FASTANSEL_SHIFT)
#  define EMAC_PC_FASTANSEL(n)         ((uint32_t)(n) << EMAC_PC_FASTANSEL_SHIFT)
#define EMAC_PC_FASTANEN               (1 << 6)  /* Bit 6:  Fast Auto Negotiation Enable */
#define EMAC_PC_EXTFD                  (1 << 7)  /* Bit 7:  Extended Full Duplex Ability */
#define EMAC_PC_FASTLUPD               (1 << 8)  /* Bit 8:  FAST Link-Up in Parallel Detect */
#define EMAC_PC_FASTRXDV               (1 << 9)  /* Bit 9:  Fast RXDV Detection */
#define EMAC_PC_MDIXEN                 (1 << 10) /* Bit 10: MDIX Enable */
#define EMAC_PC_FASTMDIX               (1 << 11) /* Bit 11: Fast Auto MDI-X */
#define EMAC_PC_RBSTMDIX               (1 << 12) /* Bit 12: Robust Auto MDI-X */
#define EMAC_PC_MDISWAP                (1 << 13) /* Bit 13: MDI Swap */
#define EMAC_PC_POLSWAP                (1 << 14) /* Bit 14: Polarity Swap */
#define EMAC_PC_FASTLDMODE_SHIFT       (15)      /* Bits 15-19: Fast Link Down Mode */
#define EMAC_PC_FASTLDMODE_MASK        (31 << EMAC_PC_FASTLDMODE_SHIFT)
#  define EMAC_PC_FASTLDMODE(n)        ((uint32_t)(n) << EMAC_PC_FASTLDMODE_SHIFT)
#define EMAC_PC_TDRRUN                 (1 << 20) /* Bit 20: TDR Auto Run */
#define EMAC_PC_LRR                    (1 << 21) /* Bit 21: Link Loss Recovery */
#define EMAC_PC_ISOMIILL               (1 << 22) /* Bit 22: Isolate MII in Link Loss */
#define EMAC_PC_RXERIDLE               (1 << 23) /* Bit 23: RXER Detection During Idle */
#define EMAC_PC_NIBDETDIS              (1 << 24) /* Bit 24: Odd Nibble TXER Detection Disable */
#define EMAC_PC_DIGRESTART             (1 << 25) /* Bit 25: PHY Soft Restart */
#define EMAC_PC_PINTFS_SHIFT           (28)      /* Bits 28-30: Ethernet Interface Select */
#define EMAC_PC_PINTFS_MASK            (7 << EMAC_PC_PINTFS_SHIFT)
#  define EMAC_PC_PINTFS_MII           (0 << EMAC_PC_PINTFS_SHIFT) /* MII: Internal PHY or external PHY connected via MII */
#  define EMAC_PC_PINTFS_RMII          (4 << EMAC_PC_PINTFS_SHIFT) /* RMII: External PHY connected via RMII */

#define EMAC_PC_PHYEXT                 (1 << 31) /* Bit 31: PHY Select */

/* Ethernet MAC Clock Configuration Register */

#define EMAC_CC_CLKEN                  (1 << 16) /* Bit 16: EN0RREF_CLK Signal Enable */
#define EMAC_CC_POL                    (1 << 17) /* Bit 17: LED Polarity Control */
#define EMAC_CC_PTPCEN                 (1 << 18) /* Bit 18: PTP Clock Reference Enable */

/* Ethernet PHY Raw Interrupt Status */

#define EMAC_PHYRIS_INT                (1 << 0)  /* Bit 0:  Ethernet PHY Raw Interrupt Status */

/* Ethernet PHY Interrupt Mask */

#define EMAC_PHYIM_INT                 (1 << 0)  /* Bit 0:  Ethernet PHY Interrupt Mask */

/* RW1C Ethernet PHY Masked Interrupt Status and Clear */

#define EMAC_PHYMISC_INT              (1 << 0)   /* Bit 0: Ethernet PHY Status and Clear register */

/* MII Management Register Bit Definitions */

/* Ethernet PHY Basic Mode Control */

#define EPHY_BMCR_COLLTST              (1 << 7)  /* Bit 7:  Collision Test */
#define EPHY_BMCR_DUPLEXM              (1 << 8)  /* Bit 8:  Duplex Mode */
#define EPHY_BMCR_RESTARTAN            (1 << 9)  /* Bit 9:  Restart Auto-Negotiation */
#define EPHY_BMCR_ISOLATE              (1 << 10) /* Bit 10: Port Isolate */
#define EPHY_BMCR_PWRDWN               (1 << 11) /* Bit 11: Power Down */
#define EPHY_BMCR_ANEN                 (1 << 12) /* Bit 12: Auto-Negotiate Enable */
#define EPHY_BMCR_SPEED                (1 << 13) /* Bit 13: Speed Select */
#define EPHY_BMCR_MIILOOPBK            (1 << 14) /* Bit 14: MII Loopback */
#define EPHY_BMCR_MIIRESET             (1 << 15) /* Bit 15: MII Register reset */

/* Ethernet PHY Basic Mode Status */

#define EPHY_BMSR_EXTEN                (1 << 0)  /* Bit 0:  Extended Capability Enable */
#define EPHY_BMSR_JABBER               (1 << 1)  /* Bit 1:  Jabber Detect */
#define EPHY_BMSR_LINKSTAT             (1 << 2)  /* Bit 2:  Link Status */
#define EPHY_BMSR_ANEN                 (1 << 3)  /* Bit 3:  Auto Negotiation Enabled */
#define EPHY_BMSR_RFAULT               (1 << 4)  /* Bit 4:  Remote Fault */
#define EPHY_BMSR_ANC                  (1 << 5)  /* Bit 5:  Auto-Negotiation Complete */
#define EPHY_BMSR_MFPRESUP             (1 << 6)  /* Bit 6:  Preamble Suppression Capable */
#define EPHY_BMSR_10BTHD               (1 << 11) /* Bit 11: 10 Base-T Half Duplex Capable */
#define EPHY_BMSR_10BTFD               (1 << 12) /* Bit 12: 10 Base-T Full Duplex Capable */
#define EPHY_BMSR_100BTXHD             (1 << 13) /* Bit 13: 100Base-TX Half Duplex Capable */
#define EPHY_BMSR_100BTXFD             (1 << 14) /* Bit 14: 100Base-TX Full Duplex Capable */

/* Ethernet PHY Identifier Register 1
 * (Most significant 16 bits of the OUI)
 */

/* Ethernet PHY Identifier Register 2 */

#define EPHY_ID2_MDLREV_SHIFT          (0)       /* Bits 0-3:  Model Revision Number */
#define EPHY_ID2_MDLREV_MASK           (15 << EPHY_ID2_MDLREV_SHIFT)
#define EPHY_ID2_VNDRMDL_SHIFT         (4)       /* Bits 4-9:  Vendor Model Number */
#define EPHY_ID2_VNDRMDL_MASK          (0x3f << EPHY_ID2_VNDRMDL_SHIFT)
#define EPHY_ID2_OUILSB_SHIFT          (10)      /* Bits 10-15:  OUI Least Significant Bits */
#define EPHY_ID2_OUILSB_MASK           (0x3f << EPHY_ID2_OUILSB_SHIFT)

/* Ethernet PHY Auto-Negotiation Advertisement */

#define EPHY_ANA_SELECT_SHIFT          (0)       /* Bits 0-4:  Protocol Selection */
#define EPHY_ANA_SELECT_MASK           (31 << EPHY_ANA_SELECT_SHIFT)
#  define EPHY_ANA_SELECT(n)           ((uint16_t)(n) << EPHY_ANA_SELECT_SHIFT)
#  define EPHY_ANA_SELECT_802p3U       (1 << EPHY_ANA_SELECT_SHIFT)
#define EPHY_ANA_10BT                  (1 << 5)  /* Bit 5:  10Base-T Support */
#define EPHY_ANA_10BTFD                (1 << 6)  /* Bit 6:  10Base-T Full Duplex Support */
#define EPHY_ANA_100BTX                (1 << 7)  /* Bit 7:  100Base-TX Support */
#define EPHY_ANA_100BTXFD              (1 << 8)  /* Bit 8:  100Base-TX Full Duplex Support */
#define EPHY_ANA_100BT4                (1 << 9)  /* Bit 9:  100Base-T4 Support */
#define EPHY_ANA_PAUSE                 (1 << 10) /* Bit 10: PAUSE Support for Full Duplex Links */
#define EPHY_ANA_ASMDUP                (1 << 11) /* Bit 11: Asymmetric PAUSE support for Full Duplex Links */
#define EPHY_ANA_RF                    (1 << 13) /* Bit 13: Remote Fault */
#define EPHY_ANA_NP                    (1 << 15) /* Bit 15: Next Page Indication */

/* Ethernet PHY Auto-Negotiation Link Partner Ability */

#define EPHY_ANLPA_SELECT_SHIFT        (0)       /* Bits 0-4:  Protocol Selection */
#define EPHY_ANLPA_SELECT_MASK         (31 << EPHY_ANLPA_SELECT_SHIFT)
#  define EPHY_ANLPA_SELECT_802p3U     (1 << EPHY_ANA_SELECT_SHIFT)
#define EPHY_ANLPA_10BT                (1 << 5)  /* Bit 5:  10Base-T Support */
#define EPHY_ANLPA_10BTFD              (1 << 6)  /* Bit 6:  10Base-T Full Duplex Support */
#define EPHY_ANLPA_100BTX              (1 << 7)  /* Bit 7:  100Base-TX Support */
#define EPHY_ANLPA_100BTXFD            (1 << 8)  /* Bit 8:  100Base-TX Full Duplex Support */
#define EPHY_ANLPA_100BT4              (1 << 9)  /* Bit 9:  100Base-T4 Support */
#define EPHY_ANLPA_PAUSE               (1 << 10) /* Bit 10: PAUSE */
#define EPHY_ANLPA_ASMDUP              (1 << 11) /* Bit 11: Asymmetric PAUSE */
#define EPHY_ANLPA_RF                  (1 << 13) /* Bit 13: Remote Fault */
#define EPHY_ANLPA_ACK                 (1 << 14) /* Bit 14: Acknowledge */
#define EPHY_ANLPA_NP                  (1 << 15) /* Bit 15: Next Page Indication */

/* Ethernet PHY Auto-Negotiation Expansion */

#define EPHY_ANER_LPANABLE             (1 << 0)  /* Bit 0:  Link Partner Auto-Negotiation Able */
#define EPHY_ANER_PAGERX               (1 << 1)  /* Bit 1:  Link Code Word Page Received */
#define EPHY_ANER_NPABLE               (1 << 2)  /* Bit 2:  Next Page Able */
#define EPHY_ANER_LPNPABLE             (1 << 3)  /* Bit 3:  Link Partner Next Page Able */
#define EPHY_ANER_PDF                  (1 << 4)  /* Bit 4:  Parallel Detection Fault */

/* Ethernet PHY Auto-Negotiation Next Page TX */

#define EPHY_ANNPTR_CODE_SHIFT         (0)        /* Bits 0-10:  Code */
#define EPHY_ANNPTR_CODE_MASK          (0x7ff << EPHY_ANNPTR_CODE_SHIFT)
#  define EPHY_ANNPTR_CODE(n)          ((uint16_t)(n) << EPHY_ANNPTR_CODE_SHIFT)
#define EPHY_ANNPTR_TOGTX              (1 << 11)  /* Bit 11: Toggle */
#define EPHY_ANNPTR_ACK2               (1 << 12)  /* Bit 12: Acknowledge 2 */
#define EPHY_ANNPTR_MP                 (1 << 13)  /* Bit 13: Message Page */
#define EPHY_ANNPTR_NP                 (1 << 15)  /* Bit 15: Next Page Indication */

/* Ethernet PHY Auto-Negotiation Link Partner Ability Next Page */

#define EPHY_ANLNPTR_CODE_SHIFT        (0)  /* Bits 0-10:  Code */
#define EPHY_ANLNPTR_CODE_MASK         (0x7ff << EPHY_ANLNPTR_CODE_SHIFT)
#define EPHY_ANLNPTR_TOG               (1 << 11)  /* Bit 11: Toggle */
#define EPHY_ANLNPTR_ACK2              (1 << 12)  /* Bit 12: Acknowledge 2 */
#define EPHY_ANLNPTR_MP                (1 << 13)  /* Bit 13: Message Page */
#define EPHY_ANLNPTR_ACK               (1 << 14)  /* Bit 14: Acknowledge */
#define EPHY_ANLNPTR_NP                (1 << 15)  /* Bit 15: Next Page Indication */

/* Ethernet PHY Configuration 1 */

#define EPHY_CFG1_FRXDVDET             (1 << 1)  /* Bit 1:  FAST RXDV Detection */

#define EPHY_CFG1_FANSEL_SHIFT         (2)       /* Bits 2-3:  Fast Auto-Negotiation Select Configuration */
#define EPHY_CFG1_FANSEL_MASK          (3 << EPHY_CFG1_FANSEL_SHIFT)
#  define EPHY_CFG1_FANSEL_BLT80       (0 << EPHY_CFG1_FANSEL_SHIFT) /* Break Link Timer: 80 ms */
#  define EPHY_CFG1_FANSEL_BLT120      (1 << EPHY_CFG1_FANSEL_SHIFT) /* Break Link Timer: 120 ms */
#  define EPHY_CFG1_FANSEL_BLT240      (2 << EPHY_CFG1_FANSEL_SHIFT) /*  Break Link Timer: 240 ms */

#define EPHY_CFG1_FASTANEN             (1 << 4)  /* Bit 4:  Fast Auto Negotiation Enable */
#define EPHY_CFG1_RAMDIX               (1 << 5)  /* Bit 5:  Robust Auto MDI/MDIX */
#define EPHY_CFG1_FAMDIX               (1 << 6)  /* Bit 6:  Fast Auto MDI/MDIX */
#define EPHY_CFG1_LLR                  (1 << 7)  /* Bit 7:  Link Loss Recovery */
#define EPHY_CFG1_TDRAR                (1 << 8)  /* Bit 8:  TDR Auto-Run at Link Down */
#define EPHY_CFG1_DONE                 (1 << 15) /* Bit 15: Configuration Done */

/* Ethernet PHY Configuration 2 */

#define EPHY_CFG2_ODDNDETDIS           (1 << 1)  /* Bit 1:  Detection of Transmit Error */
#define EPHY_CFG2_RXERRIDLE            (1 << 2)  /* Bit 2:  Detection of Receive Symbol Error During IDLE State */
#define EPHY_CFG2_ISOMIILL             (1 << 3)  /* Bit 3:  Isolate MII outputs when Enhanced Link is not Achievable */
#define EPHY_CFG2_ENLEDLINK            (1 << 4)  /* Bit 4:  Enhanced LED Functionality */
#define EPHY_CFG2_EXTFD                (1 << 5)  /* Bit 5:  Extended Full-Duplex Ability */
#define EPHY_CFG2_FLUPPD               (1 << 6)  /* Bit 6:  Fast Link-Up in Parallel Detect Mode */

/* Ethernet PHY Configuration 3 */

#define EPHY_CFG3_FLDWNM_SHIFT         (0)       /* Bits 0-4:  Fast Link Down Modes */
#define EPHY_CFG3_FLDWNM_MASK          (31 << EPHY_CFG3_FLDWNM_SHIFT)
#  define EPHY_CFG3_FLDWNM(n)          ((uint16_t)(n) << EPHY_CFG3_FLDWNM_SHIFT)
#define EPHY_CFG3_MDIMDIXS             (1 << 6)  /* Bit 6:  MDI/MDIX Swap */
#define EPHY_CFG3_POLSWAP              (1 << 7)  /* Bit 7:  Polarity Swap */

/* Ethernet PHY Register Control */

#define EPHY_REGCTL_DEVAD_SHIFT        (0)       /* Bits 0-4:  Device Address */
#define EPHY_REGCTL_DEVAD_MASK         (31 << EPHY_REGCTL_DEVAD_SHIFT)
#  define EPHY_REGCTL_DEVAD(n)         ((uint16_t)(n) << EPHY_REGCTL_DEVAD_SHIFT)
#define EPHY_REGCTL_FUNC_SHIFT         (14)      /* Bits 14-15:  Function */
#define EPHY_REGCTL_FUNC_MASK          (3 << EPHY_REGCTL_FUNC_SHIFT)
#  define EPHY_REGCTL_FUNC_ADDR        (0 << EPHY_REGCTL_FUNC_SHIFT) /* Address */
#  define EPHY_REGCTL_FUNC_DATANI      (1 << EPHY_REGCTL_FUNC_SHIFT) /* Data, no post increment */
#  define EPHY_REGCTL_FUNC_DATAPIRW    (2 << EPHY_REGCTL_FUNC_SHIFT) /* Data, post increment on read and write */
#  define EPHY_REGCTL_FUNC_DATAPIWO    (3 << EPHY_REGCTL_FUNC_SHIFT) /* Data, post increment on write only */

/* Ethernet PHY Address or Data (16-bit value) */

/* Ethernet PHY Status */

#define EPHY_STS_LINK                  (1 << 0)  /* Bit 0:  Link Status */
#define EPHY_STS_SPEED                 (1 << 1)  /* Bit 1:  Speed Status */
#define EPHY_STS_DUPLEX                (1 << 2)  /* Bit 2:  Duplex Status */
#define EPHY_STS_MIILB                 (1 << 3)  /* Bit 3:  MII Loopback Status */
#define EPHY_STS_ANS                   (1 << 4)  /* Bit 4:  Auto-Negotiation Status */
#define EPHY_STS_JD                    (1 << 5)  /* Bit 5:  Jabber Detect */
#define EPHY_STS_RF                    (1 << 6)  /* Bit 6:  Remote Fault */
#define EPHY_STS_MIIREQ                (1 << 7)  /* Bit 7:  MII Interrupt Pending */
#define EPHY_STS_PAGERX                (1 << 8)  /* Bit 8:  Link Code Page Received */
#define EPHY_STS_DL                    (1 << 9)  /* Bit 9:  Descrambler Lock */
#define EPHY_STS_SD                    (1 << 10) /* Bit 10: Signal Detect */
#define EPHY_STS_FCSL                  (1 << 11) /* Bit 11: False Carrier Sense Latch */
#define EPHY_STS_POLSTAT               (1 << 12) /* Bit 12: Polarity Status */
#define EPHY_STS_RXLERR                (1 << 13) /* Bit 13: Receive Error Latch */
#define EPHY_STS_MDIXM                 (1 << 14) /* Bit 14: MDI-X Mode */

/* Ethernet PHY Specific Control */

#define EPHY_SCR_INTEN                 (1 << 1)  /* Bit 1:  Interrupt Enable */
#define EPHY_SCR_TINT                  (1 << 2)  /* Bit 2:  Test Interrupt */
#define EPHY_SCR_COLFDM                (1 << 4)  /* Bit 4:  Collision in Full-Duplex Mode */
#define EPHY_SCR_LBFIFO_SHIFT          (8)       /* Bits 8-9:  Loopback FIFO Depth */
#define EPHY_SCR_LBFIFO_MASK           (3 << EPHY_SCR_LBFIFO_SHIFT)
#  define EPHY_SCR_LBFIFO_4            (0 << EPHY_SCR_LBFIFO_SHIFT) /* Four nibble FIFO */
#  define EPHY_SCR_LBFIFO_5            (1 << EPHY_SCR_LBFIFO_SHIFT) /* Five nibble FIFO */
#  define EPHY_SCR_LBFIFO_6            (2 << EPHY_SCR_LBFIFO_SHIFT) /* Six nibble FIFO */
#  define EPHY_SCR_LBFIFO_8            (3 << EPHY_SCR_LBFIFO_SHIFT) /* Eight nibble FIFO */

#define EPHY_SCR_SBPYASS               (1 << 11) /* Bit 11: Scrambler Bypass */
#define EPHY_SCR_PSMODE_SHIFT          (12)      /* Bits 12-13:  Power Saving Modes */
#define EPHY_SCR_PSMODE_MASK           (3 << EPHY_SCR_PSMODE_SHIFT)
#  define EPHY_SCR_PSMODE_NORMAL       (0 << EPHY_SCR_PSMODE_SHIFT) /* Normal operation mode. PHY is fully functional */
#  define EPHY_SCR_PSMODE_LOWPWR       (1 << EPHY_SCR_PSMODE_SHIFT) /* IEEE Power Down */
#  define EPHY_SCR_PSMODE_ACTWOL       (2 << EPHY_SCR_PSMODE_SHIFT) /* Active Sleep */
#  define EPHY_SCR_PSMODE_PASWOL       (3 << EPHY_SCR_PSMODE_SHIFT) /* Passive Sleep */

#define EPHY_SCR_PSEN                  (1 << 14) /* Bit 14: Power Saving Modes Enable */
#define EPHY_SCR_DISCLK                (1 << 15) /* Bit 15: Disable CLK */

/* Ethernet PHY MII Interrupt Status 1 */

#define EPHY_MISR1_RXHFEN              (1 << 0)  /* Bit 0:  Receive Error Counter Register Half-Full Event Interrupt */
#define EPHY_MISR1_FCHFEN              (1 << 1)  /* Bit 1:  False Carrier Counter Register half-full Interrupt Enable */
#define EPHY_MISR1_ANCEN               (1 << 2)  /* Bit 2:  Auto-Negotiation Complete Interrupt Enable */
#define EPHY_MISR1_DUPLEXMEN           (1 << 3)  /* Bit 3:  Duplex Status Interrupt Enable */
#define EPHY_MISR1_SPEEDEN             (1 << 4)  /* Bit 4:  Speed Change Interrupt Enable */
#define EPHY_MISR1_LINKSTATEN          (1 << 5)  /* Bit 5:  Link Status Interrupt Enable */
#define EPHY_MISR1_RXHF                (1 << 8)  /* Bit 8:  Receive Error Counter Half-Full Interrupt */
#define EPHY_MISR1_FCHF                (1 << 9)  /* Bit 9:  False Carrier Counter Half-Full Interrupt */
#define EPHY_MISR1_ANC                 (1 << 10) /* Bit 10: Auto-Negotiation Complete Interrupt */
#define EPHY_MISR1_DUPLEXM             (1 << 11) /* Bit 11: Change of Duplex Status Interrupt */
#define EPHY_MISR1_SPEED               (1 << 12) /* Bit 12: Change of Speed Status Interrupt */
#define EPHY_MISR1_LINKSTAT            (1 << 13) /* Bit 13: Change of Link Status Interrupt */

/* Ethernet PHY MII Interrupt Status 2 */

#define EPHY_MISR2_JABBEREN            (1 << 0)  /* Bit 0:  Jabber Detect Event Interrupt Enable */
#define EPHY_MISR2_POLINTEN            (1 << 1)  /* Bit 1:  Polarity Changed Interrupt Enable */
#define EPHY_MISR2_SLEEPEN             (1 << 2)  /* Bit 2:  Sleep Mode Event Interrupt Enable */
#define EPHY_MISR2_MDICOEN             (1 << 3)  /* Bit 3:  MDI/MDIX Crossover Status Changed Interrupt Enable */
#define EPHY_MISR2_LBFIFOEN            (1 << 4)  /* Bit 4:  Loopback FIFO Overflow/Underflow Interrupt Enable */
#define EPHY_MISR2_PAGERXEN            (1 << 5)  /* Bit 5:  Page Receive Interrupt Enable */
#define EPHY_MISR2_ANERREN             (1 << 6)  /* Bit 6:  Auto-Negotiation Error Interrupt Enable */
#define EPHY_MISR2_JABBER              (1 << 8)  /* Bit 8:  Jabber Detect Event Interrupt */
#define EPHY_MISR2_POLINT              (1 << 9)  /* Bit 9:  Polarity Changed Interrupt */
#define EPHY_MISR2_SLEEP               (1 << 10) /* Bit 10: Sleep Mode Event Interrupt */
#define EPHY_MISR2_MDICO               (1 << 11) /* Bit 11: MDI/MDIX Crossover Status Changed Interrupt */
#define EPHY_MISR2_LBFIFO              (1 << 12) /* Bit 12: Loopback FIFO Overflow/Underflow Event Interrupt */
#define EPHY_MISR2_PAGERX              (1 << 13) /* Bit 13: Page Receive Interrupt */
#define EPHY_MISR2_ANERR               (1 << 14) /* Bit 14: Auto-Negotiation Error Interrupt */

/* Ethernet PHY False Carrier Sense Counter */

#define EPHY_FCSCR_FCSCNT_SHIFT        (0)       /* Bits 0-7:  False Carrier Event Counter */
#define EPHY_FCSCR_FCSCNT_MASK         (0xff << EPHY_FCSCR_FCSCNT_SHIFT)

/* Ethernet PHY Receive Error Count (16-bit value) */

/* Ethernet PHY BIST Control */

#define EPHY_BISTCR_LBMODE_SHIFT       (0)       /* Bits 0-4:  Loopback Mode Select */
#define EPHY_BISTCR_LBMODE_MASK        (31 << EPHY_BISTCR_LBMODE_SHIFT)
#  define EPHY_BISTCR_LBMODE_NPCSIN    (1 << 0)  /* Bit 0:  Near-end loopback: PCS Input Loopback */
#  define EPHY_BISTCR_LBMODE_NPCSOUT   (1 << 1)  /* Bit 1:  Near-end loopback: PCS Output Loopback (100Base-TX only) */
#  define EPHY_BISTCR_LBMODE_NDIG      (1 << 2)  /* Bit 2:  Near-end loopback: Digital Loopback */
#  define EPHY_BISTCR_LBMODE_NANA      (1 << 3)  /* Bit 3:  Near-end loopback: Analog Loopback (requires 100 Ohm termination) */
#  define EPHY_BISTCR_LBMODE_FREV      (1 << 4)  /* Bit 4:  Far-end Loopback: Reverse Loopback */
#define EPHY_BISTCR_TXMIILB            (1 << 6)  /* Bit 6:  Transmit Data in MII Loopback Mode */
#define EPHY_BISTCR_PWRMODE            (1 << 8)  /* Bit 8:  Power Mode Indication */
#define EPHY_BISTCR_PKTGENSTAT         (1 << 9)  /* Bit 9:  Packet Generator Status Indication */
#define EPHY_BISTCR_PRBSCHKSYNC        (1 << 10) /* Bit 10: PRBS Checker Lock Sync Loss Indication */
#define EPHY_BISTCR_PRBSCHKLK          (1 << 11) /* Bit 11: PRBS Checker Lock Indication */
#define EPHY_BISTCR_PKTEN              (1 << 12) /* Bit 12: Packet Generation Enable */
#define EPHY_BISTCR_PRBSPKT            (1 << 13) /* Bit 13: Generated PRBS Packets */
#define EPHY_BISTCR_PRBSM              (1 << 14) /* Bit 14: PRBS Single/Continuous Mode */

/* Ethernet PHY LED Control */

#define EPHY_LEDCR_BLINKRATE_SHIFT     (9)      /* Bits 9-10: LED Blinking Rate (ON/OFF duration): */
#define EPHY_LEDCR_BLINKRATE_MASK      (3 << EPHY_LEDCR_BLINKRATE_SHIFT)
#  define EPHY_LEDCR_BLINKRATE_20HZ    (0 << EPHY_LEDCR_BLINKRATE_SHIFT) /* 20 Hz (50 ms) */
#  define EPHY_LEDCR_BLINKRATE_10HZ    (1 << EPHY_LEDCR_BLINKRATE_SHIFT) /* 10 Hz (100 ms) */
#  define EPHY_LEDCR_BLINKRATE_5HZ     (2 << EPHY_LEDCR_BLINKRATE_SHIFT) /* 5 Hz (200 ms) */
#  define EPHY_LEDCR_BLINKRATE_2HZ     (3 << EPHY_LEDCR_BLINKRATE_SHIFT) /* 2 Hz (500 ms) */

/* Ethernet PHY Control */

#define EPHY_CTL_BYPLEDSTRCH           (1 << 7)  /* Bit 7:  Bypass LED Stretching */
#define EPHY_CTL_MIILNKSTAT            (1 << 11) /* Bit 11: MII Link Status */
#define EPHY_CTL_PAUSETX               (1 << 12) /* Bit 12: Pause Transmit Negotiated Status */
#define EPHY_CTL_PAUSERX               (1 << 13) /* Bit 13: Pause Receive Negotiated Status */
#define EPHY_CTL_FORCEMDI              (1 << 14) /* Bit 14: Force MDIX */
#define EPHY_CTL_AUTOMDI               (1 << 15) /* Bit 15: Auto-MDIX Enable */

/* Ethernet PHY 10Base-T Status/Control */

#define EPHY_10BTSC_JABBERD            (1 << 0)  /* Bit 0:  Jabber Disable */
#define EPHY_10BTSC_POLSTAT            (1 << 4)  /* Bit 4:  10 Mb Polarity Status */
#define EPHY_10BTSC_NLPDIS             (1 << 7)  /* Bit 7:  Normal Link Pulse (NLP) Transmission Control */
#define EPHY_10BTSC_SQUELCH_SHIFT      (9)       /* Bits 9-12:  Squelch Configuration */
#define EPHY_10BTSC_SQUELCH_MASK       (15 << EPHY_10BTSC_SQUELCH_SHIFT)
#  define EPHY_10BTSC_SQUELCH(n)       ((uint16_t)(n) << EPHY_10BTSC_SQUELCH_SHIFT)
#define EPHY_10BTSC_RXTHEN             (1 << 13) /* Bit 13: Lower Receiver Threshold Enable */

/* Ethernet PHY BIST Control and Status 1 */

#define EPHY_BICSR1_IPGLENGTH_SHIFT    (0)       /* Bits 0-7:  BIST IPG Length */
#define EPHY_BICSR1_IPGLENGTH_MASK     (0xff << EPHY_BICSR1_IPGLENGTH_SHIFT)
#  define EPHY_BICSR1_IPGLENGTH(n)     ((uint16_t)(n) << EPHY_BICSR1_IPGLENGTH_SHIFT)
#define EPHY_BICSR1_ERRCNT_SHIFT       (8)       /* Bits 8-15:  BIST Error Count */
#define EPHY_BICSR1_ERRCNT_MASK        (0xff << EPHY_BICSR1_ERRCNT_SHIFT)

/* Ethernet PHY BIST Control and Status 2 */

#define EPHY_BICSR2_PKTLENGTH_SHIFT    (0)  /* Bits 0-10:  BIST Packet Length */
#define EPHY_BICSR2_PKTLENGTH_MASK     (0x7ff << EPHY_BICSR2_PKTLENGTH_SHIFT)
#  define EPHY_BICSR2_PKTLENGTH(n)     ((uint16_t)(n << EPHY_BICSR2_PKTLENGTH_SHIFT)

/* Ethernet PHY Cable Diagnostic Control */

#define EPHY_CDCR_FAIL                 (1 << 0)  /* Bit 0:  Cable Diagnostic Process Fail */
#define EPHY_CDCR_DONE                 (1 << 1)  /* Bit 1:  Cable Diagnostic Process Done */
#define EPHY_CDCR_LINKQUAL_SHIFT       (8)       /* Bits 8-9:  Link Quality Indication */
#define EPHY_CDCR_LINKQUAL_MASK        (3 << EPHY_CDCR_LINKQUAL_SHIFT)
#  define EPHY_CDCR_LINKQUAL_GOOD      (1 << EPHY_CDCR_LINKQUAL_SHIFT) /* Good Quality Link Indication */
#  define EPHY_CDCR_LINKQUAL_MILD      (2 << EPHY_CDCR_LINKQUAL_SHIFT) /* Mid- Quality Link Indication */
#  define EPHY_CDCR_LINKQUAL_POOR      (3 << EPHY_CDCR_LINKQUAL_SHIFT) /* Poor Quality Link Indication */

#define EPHY_CDCR_START                (1 << 15) /* Bit 15: Cable Diagnostic Process Start */

/* Ethernet PHY Reset Control */

#define EPHY_RCR_SWRESTART             (1 << 14) /* Bit 14: Software Restart */
#define EPHY_RCR_SWRST                 (1 << 15) /* Bit 15: Software Reset */

/* Ethernet PHY LED Configuration */

#define EPHY_LEDCFG_LED0_SHIFT         (0)       /* Bit0-13:  LED0 Configuration */
#define EPHY_LEDCFG_LED0_MASK          (15 << EPHY_LEDCFG_LED0_SHIFT)
#  define EPHY_LEDCFG_LED0_LINK        (0 << EPHY_LEDCFG_LED0_SHIFT) /* Link OK */
#  define EPHY_LEDCFG_LED0_RXTX        (1 << EPHY_LEDCFG_LED0_SHIFT) /* RX/TX Activity */
#  define EPHY_LEDCFG_LED0_TX          (2 << EPHY_LEDCFG_LED0_SHIFT) /* TX Activity */
#  define EPHY_LEDCFG_LED0_RX          (3 << EPHY_LEDCFG_LED0_SHIFT) /* RX Activity */
#  define EPHY_LEDCFG_LED0_COL         (4 << EPHY_LEDCFG_LED0_SHIFT) /* Collision */
#  define EPHY_LEDCFG_LED0_100BT       (5 << EPHY_LEDCFG_LED0_SHIFT) /* 100-Base TX */
#  define EPHY_LEDCFG_LED0_10BT        (6 << EPHY_LEDCFG_LED0_SHIFT) /* 10-Base TX */
#  define EPHY_LEDCFG_LED0_FD          (7 << EPHY_LEDCFG_LED0_SHIFT) /* Full Duplex */
#  define EPHY_LEDCFG_LED0_LINKTXRX    (8 << EPHY_LEDCFG_LED0_SHIFT) /* Link OK/Blink on TX/RX Activity */

#define EPHY_LEDCFG_LED1_SHIFT         (4)       /* Bits 4-7:  LED1 Configuration */
#define EPHY_LEDCFG_LED1_MASK          (15 << EPHY_LEDCFG_LED1_SHIFT)
#  define EPHY_LEDCFG_LED1_LINK        (0 << EPHY_LEDCFG_LED1_SHIFT) /* Link OK */
#  define EPHY_LEDCFG_LED1_RXTX        (1 << EPHY_LEDCFG_LED1_SHIFT) /* RX/TX Activity */
#  define EPHY_LEDCFG_LED1_TX          (2 << EPHY_LEDCFG_LED1_SHIFT) /* TX Activity */
#  define EPHY_LEDCFG_LED1_RX          (3 << EPHY_LEDCFG_LED1_SHIFT) /* RX Activity */
#  define EPHY_LEDCFG_LED1_COL         (4 << EPHY_LEDCFG_LED1_SHIFT) /* Collision */
#  define EPHY_LEDCFG_LED1_100BT       (5 << EPHY_LEDCFG_LED1_SHIFT) /* 100-Base TX */
#  define EPHY_LEDCFG_LED1_10BT        (6 << EPHY_LEDCFG_LED1_SHIFT) /* 10-Base TX */
#  define EPHY_LEDCFG_LED1_FD          (7 << EPHY_LEDCFG_LED1_SHIFT) /* Full Duplex */
#  define EPHY_LEDCFG_LED1_LINKTXRX    (8 << EPHY_LEDCFG_LED1_SHIFT) /* Link OK/Blink on TX/RX Activity */

#define EPHY_LEDCFG_LED2_SHIFT         (8)       /* Bits 8-11:  LED2 Configuration */
#define EPHY_LEDCFG_LED2_MASK          (15 << EPHY_LEDCFG_LED2_SHIFT)
#  define EPHY_LEDCFG_LED2_LINK        (0 << EPHY_LEDCFG_LED2_SHIFT) /* Link OK */
#  define EPHY_LEDCFG_LED2_RXTX        (1 << EPHY_LEDCFG_LED2_SHIFT) /* RX/TX Activity */
#  define EPHY_LEDCFG_LED2_TX          (2 << EPHY_LEDCFG_LED2_SHIFT) /* TX Activity */
#  define EPHY_LEDCFG_LED2_RX          (3 << EPHY_LEDCFG_LED2_SHIFT) /* RX Activity */
#  define EPHY_LEDCFG_LED2_COL         (4 << EPHY_LEDCFG_LED2_SHIFT) /* Collision */
#  define EPHY_LEDCFG_LED2_100BT       (5 << EPHY_LEDCFG_LED2_SHIFT) /* 100-Base TX */
#  define EPHY_LEDCFG_LED2_10BT        (6 << EPHY_LEDCFG_LED2_SHIFT) /* 10-Base TX */
#  define EPHY_LEDCFG_LED2_FD          (7 << EPHY_LEDCFG_LED2_SHIFT) /* Full Duplex */
#  define EPHY_LEDCFG_LED2_LINKTXRX    (8 << EPHY_LEDCFG_LED2_SHIFT) /* Link OK/Blink on TX/RX Activity */

/* DMA Descriptors **********************************************************/

/* TDES0: Transmit descriptor Word0 */

#define EMAC_TDES0_DB                 (1 << 0)  /* Bit 0:  Deferred bit */
#define EMAC_TDES0_UF                 (1 << 1)  /* Bit 1:  Underflow error */
#define EMAC_TDES0_ED                 (1 << 2)  /* Bit 2:  Excessive deferral */
#define EMAC_TDES0_CC_SHIFT           (3)       /* Bits 3-6: Collision count */
#define EMAC_TDES0_CC_MASK            (15 << EMAC_TDES0_CC_SHIFT)
#define EMAC_TDES0_VF                 (1 << 7)  /* Bit 7:  VLAN frame */
#define EMAC_TDES0_EC                 (1 << 8)  /* Bit 8:  Excessive collision */
#define EMAC_TDES0_LCO                (1 << 9)  /* Bit 9:  Late collision */
#define EMAC_TDES0_NC                 (1 << 10) /* Bit 10: No carrier */
#define EMAC_TDES0_LCA                (1 << 11) /* Bit 11: Loss of carrier */
#define EMAC_TDES0_IPE                (1 << 12) /* Bit 12: IP payload error */
#define EMAC_TDES0_FF                 (1 << 13) /* Bit 13: Frame flushed */
#define EMAC_TDES0_JT                 (1 << 14) /* Bit 14: Jabber timeout */
#define EMAC_TDES0_ES                 (1 << 15) /* Bit 15: Error summary */
#define EMAC_TDES0_IHE                (1 << 16) /* Bit 16: IP header error */
#define EMAC_TDES0_TTSS               (1 << 17) /* Bit 17: Transmit time stamp status */
#define EMAC_TDES0_VLIC_SHIFT         (18)      /* Bits 18-19: VLAN Insertion Control */
#define EMAC_TDES0_VLIC_MASK          (3 << EMAC_TDES0_VLIC_SHIFT)
#  define EMAC_TDES0_VLIC_NOACTION    (0 << EMAC_TDES0_VLIC_SHIFT) /* Do not add a VLAN tag */
#  define EMAC_TDES0_VLIC_REMOVE      (1 << EMAC_TDES0_VLIC_SHIFT) /* Remove the VLAN tag before sending */
#  define EMAC_TDES0_VLIC_INSERT      (2 << EMAC_TDES0_VLIC_SHIFT) /* Insert a VLAN tag (EMACVLNINCREP) */
#  define EMAC_TDES0_VLIC_REPLACE     (3 << EMAC_TDES0_VLIC_SHIFT) /* Replace the VLAN tag i(EMACVLNINCREP) */

#define EMAC_TDES0_TCH                (1 << 20) /* Bit 20: Second address chained */
#define EMAC_TDES0_TER                (1 << 21) /* Bit 21: Transmit end of ring */
#define EMAC_TDES0_CIC_SHIFT          (22)      /* Bits 22-23: Checksum insertion control */
#define EMAC_TDES0_CIC_MASK           (3 << EMAC_TDES0_CIC_SHIFT)
#  define EMAC_TDES0_CIC_DISABLED     (0 << EMAC_TDES0_CIC_SHIFT) /* Checksum disabled */
#  define EMAC_TDES0_CIC_IH           (1 << EMAC_TDES0_CIC_SHIFT) /* Insert IPv4 header checksum */
#  define EMAC_TDES0_CIC_IHPL         (2 << EMAC_TDES0_CIC_SHIFT) /* Insert TCP/UDP/ICMP checksum */
#  define EMAC_TDES0_CIC_ALL          (3 << EMAC_TDES0_CIC_SHIFT) /* TCP/UDP/ICMP checksum fully calculated */

#define EMAC_TDES0_CRCR               (1 << 24) /* Bit 24: CRC Replacement Control */
#define EMAC_TDES0_TTSE               (1 << 25) /* Bit 25: Transmit time stamp enable */
#define EMAC_TDES0_DP                 (1 << 26) /* Bit 26: Disable pad */
#define EMAC_TDES0_DC                 (1 << 27) /* Bit 27: Disable CRC */
#define EMAC_TDES0_FS                 (1 << 28) /* Bit 28: First segment */
#define EMAC_TDES0_LS                 (1 << 29) /* Bit 29: Last segment */
#define EMAC_TDES0_IC                 (1 << 30) /* Bit 30: Interrupt on completion */
#define EMAC_TDES0_OWN                (1 << 31) /* Bit 31: Own bit */

/* TDES1: Transmit descriptor Word1 */

#define EMAC_TDES1_TBS1_SHIFT         (0)  /* Bits 0-12: Transmit buffer 1 size */
#define EMAC_TDES1_TBS1_MASK          (0x1fff << EMAC_TDES1_TBS1_SHIFT)
#  define EMAC_TDES1_TBS1(n)          ((uint32_t)(n) << EMAC_TDES1_TBS1_SHIFT)
#define EMAC_TDES1_TBS2_SHIFT         (16)  /* Bits 16-28: Transmit buffer 2 size */
#define EMAC_TDES1_TBS2_MASK          (0x1fff << EMAC_TDES1_TBS2_SHIFT)
#  define EMAC_TDES1_TBS2(n)          ((uint32_t)(n) << EMAC_TDES1_TBS2_SHIFT)
#define EMAC_TDES1_CTRL_SHIFT         (29)  /* Bits 29-31:SA Insertion Control */
#define EMAC_TDES1_CTRL_MASK          (7 << EMAC_TDES1_CTRL_SHIFT)
#  define EMAC_TDES1_CTRL_NOACTION    (0 << EMAC_TDES1_CTRL_SHIFT) /* Do not include the source address */
#  define EMAC_TDES1_CTRL_INSERT      (1 << EMAC_TDES1_CTRL_SHIFT) /* Insert the source address */
#  define EMAC_TDES1_CTRL_REPLACE     (2 << EMAC_TDES1_CTRL_SHIFT) /* Replace the source address */

/* TDES2: Transmit descriptor Word2 (32-bit address) */

/* TDES3: Transmit descriptor Word3 (32-bit address) */

/* TDES6: Transmit descriptor Word6 (32-bit time stamp) */

/* TDES7: Transmit descriptor Word7 (32-bit time stamp) */

/* RDES0: Receive descriptor Word0 */

#define EMAC_RDES0_ESA                (1 << 0)  /* Bit 0:  Extended status available */
#define EMAC_RDES0_CE                 (1 << 1)  /* Bit 1:  CRC error */
#define EMAC_RDES0_DBE                (1 << 2)  /* Bit 2:  Dribble bit error */
#define EMAC_RDES0_RE                 (1 << 3)  /* Bit 3:  Receive error */
#define EMAC_RDES0_RWT                (1 << 4)  /* Bit 4:  Receive watchdog timeout */
#define EMAC_RDES0_FT                 (1 << 5)  /* Bit 5:  Frame type */
#define EMAC_RDES0_LCO                (1 << 6)  /* Bit 6:  Late collision */
#define EMAC_RDES0_TSV                (1 << 7)  /* Bit 7:  Time stamp available */
#define EMAC_RDES0_GIANT              (1 << 7)  /* Bit 7:  Giant frame */
#define EMAC_RDES0_LS                 (1 << 8)  /* Bit 8:  Last descriptor */
#define EMAC_RDES0_FS                 (1 << 9)  /* Bit 9:  First descriptor */
#define EMAC_RDES0_VLAN               (1 << 10) /* Bit 10: VLAN tag */
#define EMAC_RDES0_OE                 (1 << 11) /* Bit 11: Overflow error */
#define EMAC_RDES0_LE                 (1 << 12) /* Bit 12: Length error */
#define EMAC_RDES0_SAF                (1 << 13) /* Bit 13: Source address filter fail */
#define EMAC_RDES0_DE                 (1 << 14) /* Bit 14: Descriptor error */
#define EMAC_RDES0_ES                 (1 << 15) /* Bit 15: Error summary */
#define EMAC_RDES0_FL_SHIFT           (16)      /* Bits 16-29: Frame length */
#define EMAC_RDES0_FL_MASK            (0x3fff << EMAC_RDES0_FL_SHIFT)
#define EMAC_RDES0_AFM                (1 << 30) /* Bit 30: Destination address filter fail */
#define EMAC_RDES0_OWN                (1 << 31) /* Bit 31: Own bit */

/* RDES1: Receive descriptor Word1 */

#define EMAC_RDES1_RBS1_SHIFT         (0)       /* Bits 0-12: Receive buffer 1 size */
#define EMAC_RDES1_RBS1_MASK          (0x1fff << EMAC_RDES1_RBS1_SHIFT)
                                                /* Bit 13: Reserved */
#define EMAC_RDES1_RCH                (1 << 14) /* Bit 14: Second address chained */
#define EMAC_RDES1_RER                (1 << 15) /* Bit 15: Receive end of ring */
#define EMAC_RDES1_RBS2_SHIFT         (16)      /* Bits 16-28: Receive buffer 2 size */
#define EMAC_RDES1_RBS2_MASK          (0x1fff << EMAC_RDES1_RBS2_SHIFT)
#define EMAC_RDES1_DIC                (1 << 31) /* Bit 31: Disable interrupt on completion */

/* RDES2: Receive descriptor Word2 (32-bit address) */

/* RDES3: Receive descriptor Word3 (32-bit address) */

/* RDES4: Receive descriptor Word4 */

#define EMAC_RDES4_IPPT_SHIFT         (0)       /* Bits 0-2: IP payload type */
#define EMAC_RDES4_IPPT_MASK          (7 << EMAC_RDES4_IPPT_SHIFT)
#  define EMAC_RDES4_IPPT_UNKNOWN     (0 << EMAC_RDES4_IPPT_SHIFT) /* Unknown */
#  define EMAC_RDES4_IPPT_UDP         (1 << EMAC_RDES4_IPPT_SHIFT) /* UDP payload in IP datagram */
#  define EMAC_RDES4_IPPT_TCP         (2 << EMAC_RDES4_IPPT_SHIFT) /* TCP payload in IP datagram */
#  define EMAC_RDES4_IPPT_ICMP        (3 << EMAC_RDES4_IPPT_SHIFT) /* ICMP payload in IP datagram */

#define EMAC_RDES4_IPHE               (1 << 3)  /* Bit 3:  IP header error */
#define EMAC_RDES4_IPPE               (1 << 4)  /* Bit 4:  IP payload error */
#define EMAC_RDES4_IPCB               (1 << 5)  /* Bit 5:  IP checksum bypassed */
#define EMAC_RDES4_IPV4PR             (1 << 6)  /* Bit 6:  IPv4 packet received */
#define EMAC_RDES4_IPV6PR             (1 << 7)  /* Bit 7:  IPv6 packet received */
#define EMAC_RDES4_PMT_SHIFT          (8)       /* Bits 8-11: PTP message type */
#define EMAC_RDES4_PMT_MASK           (15 << EMAC_RDES4_PMT_SHIFT)
#  define EMAC_RDES4_PMT_NONE         (0 << EMAC_RDES4_PMT_SHIFT)  /* No PTP message received */
#  define EMAC_RDES4_PMT_SYNC         (1 << EMAC_RDES4_PMT_SHIFT)  /* SYNC (all clock types) */
#  define EMAC_RDES4_PMT_FOLLOWUP     (2 << EMAC_RDES4_PMT_SHIFT)  /* Follow_Up (all clock types) */
#  define EMAC_RDES4_PMT_DELAYREQ     (3 << EMAC_RDES4_PMT_SHIFT)  /* Delay_Req (all clock types) */
#  define EMAC_RDES4_PMT_DELAYRESP    (4 << EMAC_RDES4_PMT_SHIFT)  /* Delay_Resp (all clock types) */
#  define EMAC_RDES4_PMT_PDELREQAM    (5 << EMAC_RDES4_PMT_SHIFT)  /* Pdelay_Req (in peer-to-peer
                                                                    * transparent clock) or Announce (in
                                                                    * ordinary or boundary clock) */
#  define EMAC_RDES4_PMT_PDELREQMM    (6 << EMAC_RDES4_PMT_SHIFT)  /* Pdelay_Resp (in peer-to-peer
                                                                    * transparent clock) or Management (in
                                                                    * ordinary or boundary clock) */
#  define EMAC_RDES4_PMT_PDELREQFUS   (7 << EMAC_RDES4_PMT_SHIFT)  /* Pdelay_Resp_Follow_Up (in
                                                                    * peer-to-peer transparent clock) or
                                                                    * Signaling (for ordinary or boundary
                                                                    * clock) */
#  define EMAC_RDES4_PMT_ANNOUNCE     (8 << EMAC_RDES4_PMT_SHIFT)  /* Announce */
#  define EMAC_RDES4_PMT_MANAGEMENT   (9 << EMAC_RDES4_PMT_SHIFT)  /* Management */
#  define EMAC_RDES4_PMT_SIGNALING    (10 << EMAC_RDES4_PMT_SHIFT) /* Signaling */
#  define EMAC_RDES4_PMT_PTP          (15 << EMAC_RDES4_PMT_SHIFT) /* PTP packet w/ Reserved message type */

#define EMAC_RDES4_PFT                (1 << 12) /* Bit 12: PTP frame type */
#define EMAC_RDES4_PV                 (1 << 13) /* Bit 13: PTP version */
#define EMAC_RDES4_TSD                (1 << 14) /* Bit 14: Time stampe dropped */

/* RDES5: Receive descriptor Word5 - Reserved */

/* RDES6: Receive descriptor Word6 (32-bit time stamp) */

/* RDES7: Receive descriptor Word7 (32-bit time stamp) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Ethernet TX DMA Descriptor */

struct emac_txdesc_s
{
  /* Normal DMA descriptor words */

  volatile uint32_t tdes0;   /* Status */
  volatile uint32_t tdes1;   /* Control and buffer1/2 lengths */
  volatile uint32_t tdes2;   /* Buffer1 address pointer */
  volatile uint32_t tdes3;   /* Buffer2 or next descriptor address pointer */

  /* Enhanced DMA descriptor words with time stamp */

#ifdef CONFIG_TIVA_EMAC_ENHANCEDDESC
  volatile uint32_t tdes4;   /* Reserved */
  volatile uint32_t tdes5;   /* Reserved */
  volatile uint32_t tdes6;   /* Time Stamp Low value for transmit and receive */
  volatile uint32_t tdes7;   /* Time Stamp High value for transmit and receive */
#endif
};

/* Ethernet RX DMA Descriptor */

struct emac_rxdesc_s
{
  volatile uint32_t rdes0;   /* Status */
  volatile uint32_t rdes1;   /* Control and buffer1/2 lengths */
  volatile uint32_t rdes2;   /* Buffer1 address pointer */
  volatile uint32_t rdes3;   /* Buffer2 or next descriptor address pointer */

  /* Enhanced DMA descriptor words with time stamp and PTP support */

#ifdef CONFIG_TIVA_EMAC_ENHANCEDDESC
  volatile uint32_t rdes4;   /* Extended status for PTP receive descriptor */
  volatile uint32_t rdes5;   /* Reserved */
  volatile uint32_t rdes6;   /* Time Stamp Low value for transmit and receive */
  volatile uint32_t rdes7;   /* Time Stamp High value for transmit and receive */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TM4C_TM4C_ETHERNET_H */
