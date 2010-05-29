/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_ethernet.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_ETHERNET_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_ETHERNET_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "lp17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* MAC registers */

#define LPC17_ETH_MAC1_OFFSET       0x0000 /* MAC configuration register 1 */
#define LPC17_ETH_MAC2_OFFSET       0x0004 /* MAC configuration register 2 */
#define LPC17_ETH_IPGT_OFFSET       0x0008 /* Back-to-Back Inter-Packet-Gap register */
#define LPC17_ETH_IPGR_OFFSET       0x000c /* Non Back-to-Back Inter-Packet-Gap register */
#define LPC17_ETH_CLRT_OFFSET       0x0010 /* Collision window / Retry register */
#define LPC17_ETH_MAXF_OFFSET       0x0014 /* Maximum Frame register */
#define LPC17_ETH_SUPP_OFFSET       0x0018 /* PHY Support register */
#define LPC17_ETH_TEST_OFFSET       0x001c /* Test register */
#define LPC17_ETH_MCFG_OFFSET       0x0020 /* MII Mgmt Configuration register */
#define LPC17_ETH_MCMD_OFFSET       0x0024 /* MII Mgmt Command register */
#define LPC17_ETH_MADR_OFFSET       0x0028 /* MII Mgmt Address register */
#define LPC17_ETH_MWTD_OFFSET       0x002c /* MII Mgmt Write Data register */
#define LPC17_ETH_MRDD_OFFSET       0x0030 /* MII Mgmt Read Data register */
#define LPC17_ETH_MIND_OFFSET       0x0034 /* MII Mgmt Indicators register */
#define LPC17_ETH_SA0_OFFSET        0x0040 /* Station Address 0 register */
#define LPC17_ETH_SA1_OFFSET        0x0044 /* Station Address 1 register */
#define LPC17_ETH_SA2_OFFSET        0x0048 /* Station Address 2 register */

/* Control registers */

#define LPC17_ETH_COMMAND_OFFSET    0x0100 /* Command register */
#define LPC17_ETH_STAT_OFFSET       0x0104 /* Status register */
#define LPC17_ETH_RXDESC_OFFSET     0x0108 /* Receive descriptor base address register */
#define LPC17_ETH_RXSTAT_OFFSET     0x010c /* Receive status base address register */
#define LPC17_ETH_RXDESCNO_OFFSET   0x0110 /* Receive number of descriptors register */
#define LPC17_ETH_RXPRODIDX_OFFSET  0x0114 /* Receive produce index register */
#define LPC17_ETH_RXCONSIDX_OFFSET  0x0118 /* Receive consume index register */
#define LPC17_ETH_TXDESC_OFFSET     0x011c /* Transmit descriptor base address register */
#define LPC17_ETH_TXSTAT_OFFSET     0x0120 /* Transmit status base address register */
#define LPC17_ETH_TXDESCRNO_OFFSET  0x0124 /* Transmit number of descriptors register */
#define LPC17_ETH_TXPRODIDX_OFFSET  0x0128 /* Transmit produce index register */
#define LPC17_ETH_TXCONSIDX_OFFSET  0x012c /* Transmit consume index register */
#define LPC17_ETH_TSV0_OFFSET       0x0158 /* Transmit status vector 0 register */
#define LPC17_ETH_TSV1_OFFSET       0x015c /* Transmit status vector 1 register */
#define LPC17_ETH_RSV_OFFSET        0x0160 /* Receive status vector register */
#define LPC17_ETH_FCCNTR_OFFSET     0x0170 /* Flow control counter register */
#define LPC17_ETH_FCSTAT_OFFSET     0x0174 /* Flow control status register */

/* Rx filter registers */

#define LPC17_ETH_RXFLCTRL_OFFSET   0x0200 /* Receive filter control register */
#define LPC17_ETH_RXFLWOLST_OFFSET  0x0204 /* Receive filter WoL status register */
#define LPC17_ETH_RXFLWOLCLR_OFFSET 0x0208 /* Receive filter WoL clear register */
#define LPC17_ETH_HASHFLL_OFFSET    0x0210 /* Hash filter table LSBs register */
#define LPC17_ETH_HASHFLH_OFFSET    0x0214 /* Hash filter table MSBs register */

/* Module control registers */

#define LPC17_ETH_INTST_OFFSET      0x0fe0 /* Interrupt status register */
#define LPC17_ETH_INTEN_OFFSET      0x0fe4 /* Interrupt enable register */
#define LPC17_ETH_INTCLR_OFFSET     0x0fe8 /* Interrupt clear register */
#define LPC17_ETH_INTSET_OFFSET     0x0fec /* Interrupt set register */
#define LPC17_ETH_PWRDOWN_OFFSET    0x0ff4 /* Power-down register */

/* Register addresses ***************************************************************/
/* MAC registers */

#define LPC17_ETH_MAC1              (LPC17_ETH_BASE+LPC17_ETH_MAC1_OFFSET)
#define LPC17_ETH_MAC2              (LPC17_ETH_BASE+LPC17_ETH_MAC2_OFFSET)
#define LPC17_ETH_IPGT              (LPC17_ETH_BASE+LPC17_ETH_IPGT_OFFSET)
#define LPC17_ETH_IPGR              (LPC17_ETH_BASE+LPC17_ETH_IPGR_OFFSET)
#define LPC17_ETH_CLRT              (LPC17_ETH_BASE+LPC17_ETH_CLRT_OFFSET)
#define LPC17_ETH_MAXF              (LPC17_ETH_BASE+LPC17_ETH_MAXF_OFFSET)
#define LPC17_ETH_SUPP              (LPC17_ETH_BASE+LPC17_ETH_SUPP_OFFSET)
#define LPC17_ETH_TEST              (LPC17_ETH_BASE+LPC17_ETH_TEST_OFFSET)
#define LPC17_ETH_MCFG              (LPC17_ETH_BASE+LPC17_ETH_MCFG_OFFSET)
#define LPC17_ETH_MCMD              (LPC17_ETH_BASE+LPC17_ETH_MCMD_OFFSET)
#define LPC17_ETH_MADR              (LPC17_ETH_BASE+LPC17_ETH_MADR_OFFSET)
#define LPC17_ETH_MWTD              (LPC17_ETH_BASE+LPC17_ETH_MWTD_OFFSET)
#define LPC17_ETH_MRDD              (LPC17_ETH_BASE+LPC17_ETH_MRDD_OFFSET)
#define LPC17_ETH_MIND              (LPC17_ETH_BASE+LPC17_ETH_MIND_OFFSET)
#define LPC17_ETH_SA0               (LPC17_ETH_BASE+LPC17_ETH_SA0_OFFSET)
#define LPC17_ETH_SA1               (LPC17_ETH_BASE+LPC17_ETH_SA1_OFFSET)
#define LPC17_ETH_SA2               (LPC17_ETH_BASE+LPC17_ETH_SA2_OFFSET)

/* Control registers */

#define LPC17_ETH_COMMAND           (LPC17_ETH_BASE+LPC17_ETH_COMMAND_OFFSET)
#define LPC17_ETH_STAT              (LPC17_ETH_BASE+LPC17_ETH_STAT_OFFSET)
#define LPC17_ETH_RXDESC            (LPC17_ETH_BASE+LPC17_ETH_RXDESC_OFFSET)
#define LPC17_ETH_RXSTAT            (LPC17_ETH_BASE+LPC17_ETH_RXSTAT_OFFSET)
#define LPC17_ETH_RXDESCNO          (LPC17_ETH_BASE+LPC17_ETH_RXDESCNO_OFFSET)
#define LPC17_ETH_RXPRODIDX         (LPC17_ETH_BASE+LPC17_ETH_RXPRODIDX_OFFSET)
#define LPC17_ETH_RXCONSIDX         (LPC17_ETH_BASE+LPC17_ETH_RXCONSIDX_OFFSET)
#define LPC17_ETH_TXDESC            (LPC17_ETH_BASE+LPC17_ETH_TXDESC_OFFSET)
#define LPC17_ETH_TXSTAT            (LPC17_ETH_BASE+LPC17_ETH_TXSTAT_OFFSET)
#define LPC17_ETH_TXDESCRNO         (LPC17_ETH_BASE+LPC17_ETH_TXDESCRNO_OFFSET)
#define LPC17_ETH_TXPRODIDX         (LPC17_ETH_BASE+LPC17_ETH_TXPRODIDX_OFFSET)
#define LPC17_ETH_TXCONSIDX         (LPC17_ETH_BASE+LPC17_ETH_TXCONSIDX_OFFSET)
#define LPC17_ETH_TSV0              (LPC17_ETH_BASE+LPC17_ETH_TSV0_OFFSET)
#define LPC17_ETH_TSV1              (LPC17_ETH_BASE+LPC17_ETH_TSV1_OFFSET)
#define LPC17_ETH_RSV               (LPC17_ETH_BASE+LPC17_ETH_RSV_OFFSET)
#define LPC17_ETH_FCCNTR            (LPC17_ETH_BASE+LPC17_ETH_FCCNTR_OFFSET)
#define LPC17_ETH_FCSTAT            (LPC17_ETH_BASE+LPC17_ETH_FCSTAT_OFFSET)

/* Rx filter registers */

#define LPC17_ETH_RXFLCTRL          (LPC17_ETH_BASE+LPC17_ETH_RXFLCTRL_OFFSET)
#define LPC17_ETH_RXFLWOLST         (LPC17_ETH_BASE+LPC17_ETH_RXFLWOLST_OFFSET)
#define LPC17_ETH_RXFLWOLCLR        (LPC17_ETH_BASE+LPC17_ETH_RXFLWOLCLR_OFFSET)
#define LPC17_ETH_HASHFLL           (LPC17_ETH_BASE+LPC17_ETH_HASHFLL_OFFSET)
#define LPC17_ETH_HASHFLH           (LPC17_ETH_BASE+LPC17_ETH_HASHFLH_OFFSET)

/* Module control registers */

#define LPC17_ETH_INTST             (LPC17_ETH_BASE+LPC17_ETH_INTST_OFFSET)
#define LPC17_ETH_INTEN             (LPC17_ETH_BASE+LPC17_ETH_INTEN_OFFSET)
#define LPC17_ETH_INTCLR            (LPC17_ETH_BASE+LPC17_ETH_INTCLR_OFFSET)
#define LPC17_ETH_INTSET            (LPC17_ETH_BASE+LPC17_ETH_INTSET_OFFSET)
#define LPC17_ETH_PWRDOWN           (LPC17_ETH_BASE+LPC17_ETH_PWRDOWN_OFFSET)

/* Register bit definitions *********************************************************/
/* MAC registers */

/* MAC configuration register 1 */
#define ETH_MAC1_
/* MAC configuration register 2 */
#define ETH_MAC2_
/* Back-to-Back Inter-Packet-Gap register */
#define ETH_IPGT_
/* Non Back-to-Back Inter-Packet-Gap register */
#define ETH_IPGR_
/* Collision window / Retry register */
#define ETH_CLRT_
/* Maximum Frame register */
#define ETH_MAXF_
/* PHY Support register */
#define ETH_SUPP_
/* Test register */
#define ETH_TEST_
/* MII Mgmt Configuration register */
#define ETH_MCFG_
/* MII Mgmt Command register */
#define ETH_MCMD_
/* MII Mgmt Address register */
#define ETH_MADR_
/* MII Mgmt Write Data register */
#define ETH_MWTD_
/* MII Mgmt Read Data register */
#define ETH_MRDD_
/* MII Mgmt Indicators register */
#define ETH_MIND_
/* Station Address 0 register */
#define ETH_SA0_
/* Station Address 1 register */
#define ETH_SA1_
/* Station Address 2 register */
#define ETH_SA2_

/* Control registers */

/* Command register */
#define ETH_COMMAND_
/* Status register */
#define ETH_STAT_
/* Receive descriptor base address register */
#define ETH_RXDESC_
/* Receive status base address register */
#define ETH_RXSTAT_
/* Receive number of descriptors register */
#define ETH_RXDESCNO_
/* Receive produce index register */
#define ETH_RXPRODIDX_
/* Receive consume index register */
#define ETH_RXCONSIDX_
/* Transmit descriptor base address register */
#define ETH_TXDESC_
/* Transmit status base address register */
#define ETH_TXSTAT_
/* Transmit number of descriptors register */
#define ETH_TXDESCRNO_
/* Transmit produce index register */
#define ETH_TXPRODIDX_
/* Transmit consume index register */
#define ETH_TXCONSIDX_
/* Transmit status vector 0 register */
#define ETH_TSV0_
/* Transmit status vector 1 register */
#define ETH_TSV1_
/* Receive status vector register */
#define ETH_RSV_
/* Flow control counter register */
#define ETH_FCCNTR_
/* Flow control status register */
#define ETH_FCSTAT_

/* Rx filter registers */

/* Receive filter control register */
#define ETH_RXFLCTRL_
/* Receive filter WoL status register */
#define ETH_RXFLWOLST_
/* Receive filter WoL clear register */
#define ETH_RXFLWOLCLR_
/* Hash filter table LSBs register */
#define ETH_HASHFLL_
/* Hash filter table MSBs register */
#define ETH_HASHFLH_

/* Module control registers */

/* Interrupt status register */
#define ETH_INTST_
/* Interrupt enable register */
#define ETH_INTEN_
/* Interrupt clear register */
#define ETH_INTCLR_
/* Interrupt set register */
#define ETH_INTSET_
/* Power-down register */
#define ETH_PWRDOWN_

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_ETHERNET_H */
