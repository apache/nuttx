/************************************************************************************
 * arch/arm/src/tiva/chip/tm4c_ethernet.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#define EMAC_CFG_
/* Ethernet MAC Frame Filter */
#define EMAC_FRAMEFLTR_
/* Ethernet MAC Hash Table High */
#define EMAC_HASHTBLH_
/* Ethernet MAC Hash Table Low */
#define EMAC_HASHTBLL_
/* Ethernet MAC MII Address */
#define EMAC_MIIADDR_
/* Ethernet MAC MII Data Register */
#define EMAC_MIIDATA_
/* Ethernet MAC Flow Control */
#define EMAC_FLOWCTL_
/* Ethernet MAC VLAN Tag */
#define EMAC_VLANTG_
/* Ethernet MAC Status */
#define EMAC_STATUS_
/* Ethernet MAC Remote Wake-Up Frame Filter */
#define EMAC_RWUFF_
/* Ethernet MAC PMT Control and Status Register */
#define EMAC_PMTCTLSTAT_
/* Ethernet MAC Raw Interrupt Status */
#define EMAC_RIS_
/* Ethernet MAC Interrupt Mask */
#define EMAC_IM_
/* Ethernet MAC Address 0 High */
#define EMAC_ADDR0H_
/* Ethernet MAC Address 0 Low Register */
#define EMAC_ADDR0L_
/* Ethernet MAC Address 1 High */
#define EMAC_ADDR1H_
/* Ethernet MAC Address 1 Low */
#define EMAC_ADDR1L_
/* Ethernet MAC Address 2 High */
#define EMAC_ADDR2H_
/* Ethernet MAC Address 2 Low */
#define EMAC_ADDR2L_
/* Ethernet MAC Address 3 High */
#define EMAC_ADDR3H_
/* Ethernet MAC Address 3 Low */
#define EMAC_ADDR3L_
/* Ethernet MAC Watchdog Timeout */
#define EMAC_WDOGTO_
/* Ethernet MAC MMC Control */
#define EMAC_MMCCTRL_
/* Ethernet MAC MMC Receive Raw Interrupt Status */
#define EMAC_MMCRXRIS_
/* Ethernet MAC MMC Transmit Raw Interrupt Status */
#define EMAC_MMCTXRIS_
/* Ethernet MAC MMC Receive Interrupt Mask */
#define EMAC_MMCRXIM_
/* Ethernet MAC MMC Transmit Interrupt Mask */
#define EMAC_MMCTXIM_
/* Ethernet MAC Transmit Frame Count for Good and Bad Frames */
#define EMAC_TXCNTGB_
/* Ethernet MAC Transmit Frame Count for Frames Transmitted after Single Collision */
#define EMAC_TXCNTSCOL_
/* Ethernet MAC Transmit Frame Count for Frames Transmitted after Multiple Collisions */
#define EMAC_TXCNTMCOL_
/* Ethernet MAC Transmit Octet Count Good */
#define EMAC_TXOCTCNTG_
/* Ethernet MAC Receive Frame Count for Good and Bad Frames */
#define EMAC_RXCNTGB_
/* Ethernet MAC Receive Frame Count for CRC Error Frames */
#define EMAC_RXCNTCRCERR_
/* Ethernet MAC Receive Frame Count for Alignment Error Frames */
#define EMAC_RXCNTALGNERR_
/* Ethernet MAC Receive Frame Count for Good Unicast Frames */
#define EMAC_RXCNTGUNI_
/* Ethernet MAC VLAN Tag Inclusion or Replacement */
#define EMAC_VLNINCREP_
/* Ethernet MAC VLAN Hash Table */
#define EMAC_VLANHASH_
/* Ethernet MAC Timestamp Control */
#define EMAC_TIMSTCTRL_
/* Ethernet MAC Sub-Second Increment */
#define EMAC_SUBSECINC_
/* Ethernet MAC System Time - Seconds */
#define EMAC_TIMSEC_
/* Ethernet MAC System Time - Nanoseconds */
#define EMAC_TIMNANO_
/* Ethernet MAC System Time - Seconds Update */
#define EMAC_TIMSECU_
/* Ethernet MAC System Time - Nanoseconds Update */
#define EMAC_TIMNANOU_
/* Ethernet MAC Timestamp Addend */
#define EMAC_TIMADD_
/* Ethernet MAC Target Time Seconds */
#define EMAC_TARGSEC_
/* Ethernet MAC Target Time Nanoseconds */
#define EMAC_TARGNANO_
/* Ethernet MAC System Time-Higher Word Seconds */
#define EMAC_HWORDSEC_
/* Ethernet MAC Timestamp Status */
#define EMAC_TIMSTAT_
/* Ethernet MAC PPS Control */
#define EMAC_PPSCTRL_
/* Ethernet MAC PPS0 Interval */
#define EMAC_PPS0INTVL_
/* Ethernet MAC PPS0 Width */
#define EMAC_PPS0WIDTH_
/* Ethernet MAC DMA Bus Mode */
#define EMAC_DMABUSMOD_
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
