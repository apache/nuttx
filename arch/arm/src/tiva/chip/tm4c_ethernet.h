/************************************************************************************
 * arch/arm/src/tiva/chip/tm4c_ethernet.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_TM4C_ETHERNET_H
#define __ARCH_ARM_SRC_TIVA_CHIP_TM4C_ETHERNET_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Ethernet Controller Register Offsets *********************************************/

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

/* Ethernet Controller Register Addresses *******************************************/

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

#define TIVA_EPHY_EPHYBMCR             0x00 /* Ethernet PHY Basic Mode Control */
#define TIVA_EPHY_EPHYBMSR             0x01 /* Ethernet PHY Basic Mode Status */
#define TIVA_EPHY_EPHYID1              0x02 /* Ethernet PHY Identifier Register 1 */
#define TIVA_EPHY_EPHYID2              0x03 /* Ethernet PHY Identifier Register 2 */
#define TIVA_EPHY_EPHYANA              0x04 /* Ethernet PHY Auto-Negotiation Advertisement */
#define TIVA_EPHY_EPHYANLPA            0x05 /* Ethernet PHY Auto-Negotiation Link Partner Ability */
#define TIVA_EPHY_EPHYANER             0x06 /* Ethernet PHY Auto-Negotiation Expansion */
#define TIVA_EPHY_EPHYANNPTR           0x07 /* Ethernet PHY Auto-Negotiation Next Page TX */
#define TIVA_EPHY_EPHYANLNPTR          0x08 /* Ethernet PHY Auto-Negotiation Link Partner Ability Next Page */
#define TIVA_EPHY_EPHYCFG1             0x09 /* Ethernet PHY Configuration 1 */
#define TIVA_EPHY_EPHYCFG2             0x0a /* Ethernet PHY Configuration 2 */
#define TIVA_EPHY_EPHYCFG3             0x0b /* Ethernet PHY Configuration 3 */
#define TIVA_EPHY_EPHYREGCTL           0x0d /* Ethernet PHY Register Control */
#define TIVA_EPHY_EPHYADDAR            0x0e /* Ethernet PHY Address or Data */
#define TIVA_EPHY_EPHYSTS              0x10 /* Ethernet PHY Status */
#define TIVA_EPHY_EPHYSCR              0x11 /* Ethernet PHY Specific Control */
#define TIVA_EPHY_EPHYMISR1            0x12 /* Ethernet PHY MII Interrupt Status 1 */
#define TIVA_EPHY_EPHYMISR2            0x13 /* Ethernet PHY MII Interrupt Status 2 */
#define TIVA_EPHY_EPHYFCSCR            0x14 /* Ethernet PHY False Carrier Sense Counter */
#define TIVA_EPHY_EPHYRXERCNT          0x15 /* Ethernet PHY Receive Error Count */
#define TIVA_EPHY_EPHYBISTCR           0x16 /* Ethernet PHY BIST Control */
#define TIVA_EPHY_EPHYLEDCR            0x18 /* Ethernet PHY LED Control */
#define TIVA_EPHY_EPHYCTL              0x19 /* Ethernet PHY Control */
#define TIVA_EPHY_EPHY10BTSC           0x1a /* Ethernet PHY 10Base-T Status/Control */
#define TIVA_EPHY_EPHYBICSR1           0x1b /* Ethernet PHY BIST Control and Status 1 */
#define TIVA_EPHY_EPHYBICSR2           0x1c /* Ethernet PHY BIST Control and Status 2 */
#define TIVA_EPHY_EPHYCDCR             0x1e /* Ethernet PHY Cable Diagnostic Control */
#define TIVA_EPHY_EPHYRCR              0x1f /* Ethernet PHY Reset Control */
#define TIVA_EPHY_EPHYLEDCFG           0x25 /* Ethernet PHY LED Configuration */

/* Ethernet Controller Register Bit Definitions *************************************/

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
#  define EMAC_CFG_BL_1024             (0 << EMAC_CFG_BL_SHIFT) /* k = min (n,10) */
#  define EMAC_CFG_BL_256              (1 << EMAC_CFG_BL_SHIFT) /* k = min (n,8) */
#  define EMAC_CFG_BL_8                (2 << EMAC_CFG_BL_SHIFT) /* k = min (n,4) */
#  define EMAC_CFG_BL_2                (3 << EMAC_CFG_BL_SHIFT) /* k = min (n,1) */
#define EMAC_CFG_ACS                   (1 << 7)  /* Bit 7:  Automatic Pad or CRC Stripping */
#define EMAC_CFG_DR                    (1 << 9)  /* Bit 8:  Disable Retry */
#define EMAC_CFG_IPC                   (1 << 10) /* Bit 10: Checksum Offload */
#define EMAC_CFG_DUPM                  (1 << 11) /* Bit 11: Duplex Mode */
#define EMAC_CFG_LOOPBM                (1 << 12) /* Bit 12: Loopback Mode */
#define EMAC_CFG_DRO                   (1 << 13) /* Bit 13: Disable Receive Own */
#define EMAC_CFG_FES                   (1 << 14) /* Bit 14: Speed */
#define EMAC_CFG_PS                    (1 << 15) /* Bit 15: Port Select */
#define EMAC_CFG_DISCRS                (1 << 16) /* Bit 16: Disable Carrier Sense During Transmission */
#define EMAC_CFG_IFG_SHIFT             (17)     /* Bits 17-19: Inter-Frame Gap (IFG) */
#define EMAC_CFG_IFG_MASK              (7 << EMAC_CFG_IFG_SHIFT)
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
#define EMAC_FRAMEFLTR_PCF_ALL         (0 << EMAC_FRAMEFLTR_PCF_SHIFT) /* Filter all control frames */
#define EMAC_FRAMEFLTR_PCF_PAUSE       (1 << EMAC_FRAMEFLTR_PCF_SHIFT) /* Forward all control frames except PAUSE */
#define EMAC_FRAMEFLTR_PCF_NONE        (2 << EMAC_FRAMEFLTR_PCF_SHIFT) /* Forward all control frames */
#define EMAC_FRAMEFLTR_PCF_ADDR        (3 << EMAC_FRAMEFLTR_PCF_SHIFT) /* Forward control frames that pass the address Filter */
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
#  define EMAC_FLOWCTL_PLT_4           (0 << EMAC_FLOWCTL_PLT_SHIFT) /* Pause time minus 4 slot times */
#  define EMAC_FLOWCTL_PLT_28          (1 << EMAC_FLOWCTL_PLT_SHIFT) /* Pause time minus 28 slot times */
#  define EMAC_FLOWCTL_PLT_144         (2 << EMAC_FLOWCTL_PLT_SHIFT) /* Pause time minus 144 slot times */
#  define EMAC_FLOWCTL_PLT_156         (3 << EMAC_FLOWCTL_PLT_SHIFT) /* Pause time minus 256 slot times */
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
/* Ethernet MAC Transmit Frame Count for Frames Transmitted after Single Collision (32-bit data) */
/* Ethernet MAC Transmit Frame Count for Frames Transmitted after Multiple Collisions (32-bit data) */
/* Ethernet MAC Transmit Octet Count Good (32-bit data) */
/* Ethernet MAC Receive Frame Count for Good and Bad Frames (32-bit data) */
/* Ethernet MAC Receive Frame Count for CRC Error Frames (32-bit data) */
/* Ethernet MAC Receive Frame Count for Alignment Error Frames (32-bit data) */
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
#  define EMAC_DMABUSMOD_PR_11         (0 << EMAC_DMABUSMOD_PR_SHIFT) /* Priority Ratio is 1:1 */
#  define EMAC_DMABUSMOD_PR_21         (1 << EMAC_DMABUSMOD_PR_SHIFT) /* Priority Ratio is 2:1 */
#  define EMAC_DMABUSMOD_PR_31         (2 << EMAC_DMABUSMOD_PR_SHIFT) /* Priority Ratio is 3:1 */
#  define EMAC_DMABUSMOD_PR_41         (3 << EMAC_DMABUSMOD_PR_SHIFT) /* Priority Ratio is 4:1 */
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

/* Ethernet MAC Transmit Poll Demand */
#define EMAC_TXPOLLD_
/* Ethernet MAC Receive Poll Demand */
#define EMAC_RXPOLLD_
/* Ethernet MAC Receive Descriptor List Address */
#define EMAC_RXDLADDR_
/* Ethernet MAC Transmit Descriptor List Address */
#define EMAC_TXDLADDR_
/* Ethernet MAC DMA Interrupt Status */
#define EMAC_DMARIS_
/* Ethernet MAC DMA Operation Mode */
#define EMAC_DMAOPMODE_
/* Ethernet MAC DMA Interrupt Mask Register */
#define EMAC_DMAIM_
/* Ethernet MAC Missed Frame and Buffer Overflow Counter */
#define EMAC_MFBOC_
/* Ethernet MAC Receive Interrupt Watchdog Timer */
#define EMAC_RXINTWDT_
/* Ethernet MAC Current Host Transmit Descriptor */
#define EMAC_HOSTXDESC_
/* Ethernet MAC Current Host Receive Descriptor */
#define EMAC_HOSRXDESC_
/* Ethernet MAC Current Host Transmit Buffer Address */
#define EMAC_HOSTXBA_
/* Ethernet MAC Current Host Receive Buffer Address */
#define EMAC_HOSRXBA_
/* Ethernet MAC Peripheral Property Register */
#define EMAC_PP_
/* Ethernet MAC Peripheral Configuration Register */
#define EMAC_PC_
/* Ethernet MAC Clock Configuration Register */
#define EMAC_CC_
/* Ethernet PHY Raw Interrupt Status */
#define EPHY_RIS_
/* Ethernet PHY Interrupt Mask */
#define EPHY_IM_
/* RW1C Ethernet PHY Masked Interrupt Status and Clear */
#define EPHY_MISC_

/* MII Management Register Bit Definitions */

/* Ethernet PHY Basic Mode Control */
#define EPHY_EPHYBMCR_
/* Ethernet PHY Basic Mode Status */
#define EPHY_EPHYBMSR_
/* Ethernet PHY Identifier Register 1 */
#define EPHY_EPHYID1_
/* Ethernet PHY Identifier Register 2 */
#define EPHY_EPHYID2_
/* Ethernet PHY Auto-Negotiation Advertisement */
#define EPHY_EPHYANA_
/* Ethernet PHY Auto-Negotiation Link Partner Ability */
#define EPHY_EPHYANLPA_
/* Ethernet PHY Auto-Negotiation Expansion */
#define EPHY_EPHYANER_
/* Ethernet PHY Auto-Negotiation Next Page TX */
#define EPHY_EPHYANNPTR_
/* Ethernet PHY Auto-Negotiation Link Partner Ability Next Page */
#define EPHY_EPHYANLNPTR_
/* Ethernet PHY Configuration 1 */
#define EPHY_EPHYCFG1_
/* Ethernet PHY Configuration 2 */
#define EPHY_EPHYCFG2_
/* Ethernet PHY Configuration 3 */
#define EPHY_EPHYCFG3_
/* Ethernet PHY Register Control */
#define EPHY_EPHYREGCTL_
/* Ethernet PHY Address or Data */
#define EPHY_EPHYADDAR_
/* Ethernet PHY Status */
#define EPHY_EPHYSTS_
/* Ethernet PHY Specific Control */
#define EPHY_EPHYSCR_
/* Ethernet PHY MII Interrupt Status 1 */
#define EPHY_EPHYMISR1_
/* Ethernet PHY MII Interrupt Status 2 */
#define EPHY_EPHYMISR2_
/* Ethernet PHY False Carrier Sense Counter */
#define EPHY_EPHYFCSCR_
/* Ethernet PHY Receive Error Count */
#define EPHY_EPHYRXERCNT_
/* Ethernet PHY BIST Control */
#define EPHY_EPHYBISTCR_
/* Ethernet PHY LED Control */
#define EPHY_EPHYLEDCR_
/* Ethernet PHY Control */
#define EPHY_EPHYCTL_
/* Ethernet PHY 10Base-T Status/Control */
#define EPHY_EPHY10BTSC_
/* Ethernet PHY BIST Control and Status 1 */
#define EPHY_EPHYBICSR1_
/* Ethernet PHY BIST Control and Status 2 */
#define EPHY_EPHYBICSR2_
/* Ethernet PHY Cable Diagnostic Control */
#define EPHY_EPHYCDCR_
/* Ethernet PHY Reset Control */
#define EPHY_EPHYRCR_
/* Ethernet PHY LED Configuration */
#define EPHY_EPHYLEDCFG_

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_CHIP_TM4C_ETHERNET_H */
