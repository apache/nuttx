/************************************************************************************
 * arch/arm/src/xmc4/chip/xmc4_scu.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: XMC4500 Reference Manual V1.5 2014-07 Microcontrollers.
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
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 *   Infineon Technologies AG (Infineon) is supplying this software for use with
 *   Infineon's microcontrollers.  This file can be freely distributed within
 *   development tools that are supporting such microcontrollers.
 *
 *   THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *   OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *   INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 *   OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_XMC4_CHIP_XMC4_SCU_H
#define __ARCH_ARM_SRC_XMC4_CHIP_XMC4_SCU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/xmc4_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/
/* General SCU Registers */

#define XMC4_SCU_ID_OFFSET          0x0000    /* Module Identification Register */
#define XMC4_SCU_IDCHIP_OFFSET      0x0004    /* Chip ID */
#define XMC4_SCU_IDMANUF_OFFSET     0x0008    /* Manufactory ID */
#define XMC4_SCU_STCON_OFFSET       0x0010    /* Start-up Control */
#define XMC4_SCU_GPR0_OFFSET        0x002c    /* General Purpose Register 0 */
#define XMC4_SCU_GPR1_OFFSET        0x0030    /* General Purpose Register 1 */
#define XMC4_SCU_ETH0CON_OFFSET     0x0040    /* Ethernet 0 Port Control */
#define XMC4_SCU_CCUCON_OFFSET      0x004c    /* CCUx Global Start Control Register */
#define XMC4_SCU_DTSCON_OFFSET      0x008c    /* DTS Control */
#define XMC4_SCU_DTSSTAT_OFFSET     0x0090    /* DTS Status */
#define XMC4_SCU_SDMMCDEL_OFFSET    0x009c    /* SD-MMC Delay Control Register */
#define XMC4_SCU_G0ORCEN_OFFSET     0x00a0    /* Out-Of-Range Comparator Enable Register 0 */
#define XMC4_SCU_G1ORCEN_OFFSET     0x00a4    /* Out-Of-Range Comparator Enable Register 1 */
#define XMC4_SCU_MIRRSTS_OFFSET     0x00c4    /* Mirror Update Status Register */
#define XMC4_SCU_RMACR_OFFSET       0x00c8    /* Retention Memory Access Control Register */
#define XMC4_SCU_RMADATA_OFFSET     0x00cc    /* Retention Memory Access Data Register */

/* Ethernet Control SCU Resters */

#define XMC4_SCU_ETHCON_OFFSET      0x0000    /* Ethernet 0 Port Control Register */

/* Interrupt Control SCU Registers */

#define XMC4_SCU_SRSTAT_OFFSET      0x0000    /* Service Request Status */
#define XMC4_SCU_SRRAW_OFFSET       0x0004    /* RAW Service Request Status */
#define XMC4_SCU_SRMSK_OFFSET       0x0008    /* Service Request Mask */
#define XMC4_SCU_SRCLR_OFFSET       0x000c    /* Service Request Clear */
#define XMC4_SCU_SRSET_OFFSET       0x0010    /* Service Request Set */
#define XMC4_SCU_NMIREQEN_OFFSET    0x0014    /* Enable Promoting Events to NMI Request */

/* SDMMC Control SCU Registers */

#define XMC4_SCU_SDMMCCON_OFFSET    0x0000    /* SDMMC Configuration */

/* Parity Control Registers */

#define XMC4_SCU_PEEN_OFFSET        0x0000    /* Parity Error Enable Register */
#define XMC4_SCU_MCHKCON_OFFSET     0x0004    /* Memory Checking Control Register */
#define XMC4_SCU_PETE_OFFSET        0x0008    /* Parity Error Trap Enable Register */
#define XMC4_SCU_PERSTEN_OFFSET     0x000c    /* Reset upon Parity Error Enable Register */
#define XMC4_SCU_PEFLAG_OFFSET      0x0014    /* Parity Error Control Register */
#define XMC4_SCU_PMTPR_OFFSET       0x0018    /* Parity Memory Test Pattern Register */
#define XMC4_SCU_PMTSR_OFFSET       0x001c    /* Parity Memory Test Select Register */

/* Trap Control Registers */

#define XMC4_SCU_TRAPSTAT_OFFSET    0x0000    /* Trap Status Register */
#define XMC4_SCU_TRAPRAW_OFFSET     0x0004    /* Trap Raw Status Register */
#define XMC4_SCU_TRAPDIS_OFFSET     0x0008    /* Trap Mask Register */
#define XMC4_SCU_TRAPCLR_OFFSET     0x000c    /* Trap Clear Register */
#define XMC4_SCU_TRAPSET_OFFSET     0x0010    /* Trap Set Register */

/* Power Control SCU Registers */

#define XMC4_SCU_PWRSTAT_OFFSET     0x0000    /* Power Status Register */
#define XMC4_SCU_PWRSET_OFFSET      0x0004    /* Power Set Control Register */
#define XMC4_SCU_PWRCLR_OFFSET      0x0008    /* Power Clear Control Register */
#define XMC4_SCU_EVRSTAT_OFFSET     0x0010    /* EVR Status Register */
#define XMC4_SCU_EVRVADCSTAT_OFFSET 0x0014    /* EVR VADC Status Register */
#define XMC4_SCU_PWRMON_OFFSET      0x002c    /* Power Monitor Value */

/* Hibernation SCU Registers */

#define XMC4_SCU_HDSTAT_OFFSET      0x0000    /* Hibernate Domain Status Register */
#define XMC4_SCU_HDCLR_OFFSET       0x0004    /* Hibernate Domain Status Clear Register */
#define XMC4_SCU_HDSET_OFFSET       0x0008    /* Hibernate Domain Status Set Register */
#define XMC4_SCU_HDCR_OFFSET        0x000c    /* Hibernate Domain Control Register */
#define XMC4_SCU_OSCSICTRL_OFFSET   0x0014    /* Internal 32.768 kHz Clock Source Control Register */
#define XMC4_SCU_OSCULSTAT_OFFSET   0x0018    /* OSC_ULP Status Register */
#define XMC4_SCU_OSCULCTRL_OFFSET   0x001c    /* OSC_ULP Control Register */

/* Reset SCU Registers */

#define XMC4_SCU_RSTSTAT_OFFSET     0x0000    /* System Reset Status */
#define XMC4_SCU_RSTSET_OFFSET      0x0004    /* Reset Set Register */
#define XMC4_SCU_RSTCLR_OFFSET      0x0008    /* Reset Clear Register */
#define XMC4_SCU_PRSTAT0_OFFSET     0x000c    /* Peripheral Reset Status Register 0 */
#define XMC4_SCU_PRSET0_OFFSET      0x0010    /* Peripheral Reset Set Register 0 */
#define XMC4_SCU_PRCLR0_OFFSET      0x0014    /* Peripheral Reset Clear Register 0 */
#define XMC4_SCU_PRSTAT1_OFFSET     0x0018    /* Peripheral Reset Status Register 1 */
#define XMC4_SCU_PRSET1_OFFSET      0x001c    /* Peripheral Reset Set Register 1 */
#define XMC4_SCU_PRCLR1_OFFSET      0x0020    /* Peripheral Reset Clear Register 1 */
#define XMC4_SCU_PRSTAT2_OFFSET     0x0024    /* Peripheral Reset Status Register 2 */
#define XMC4_SCU_PRSET2_OFFSET      0x0028    /* Peripheral Reset Set Register 2 */
#define XMC4_SCU_PRCLR2_OFFSET      0x002c    /* Peripheral Reset Clear Register 2 */
#define XMC4_SCU_PRSTAT3_OFFSET     0x0030    /* Peripheral Reset Status Register 3 */
#define XMC4_SCU_PRSET3_OFFSET      0x0034    /* Peripheral Reset Set Register 3 */
#define XMC4_SCU_PRCLR3_OFFSET      0x0038    /* Peripheral Reset Clear Register 3 */

/* Clock Control SCU Registers */

#define XMC4_SCU_CLKSTAT_OFFSET     0x0000    /* Clock Status Register */
#define XMC4_SCU_CLKSET_OFFSET      0x0004    /* Clock Set Control Register */
#define XMC4_SCU_CLKCLR_OFFSET      0x0008    /* Clock clear Control Register */
#define XMC4_SCU_SYSCLKCR_OFFSET    0x000c    /* System Clock Control */
#define XMC4_SCU_CPUCLKCR_OFFSET    0x0010    /* CPU Clock Control */
#define XMC4_SCU_PBCLKCR_OFFSET     0x0014    /* Peripheral Bus Clock Control */
#define XMC4_SCU_USBCLKCR_OFFSET    0x0018    /* USB Clock Control */
#define XMC4_SCU_EBUCLKCR_OFFSET    0x001c    /* EBU Clock Control */
#define XMC4_SCU_CCUCLKCR_OFFSET    0x0020    /* CCU Clock Control */
#define XMC4_SCU_WDTCLKCR_OFFSET    0x0024    /* WDT Clock Control */
#define XMC4_SCU_EXTCLKCR_OFFSET    0x0028    /* External clock Control Register */
#define XMC4_SCU_SLEEPCR_OFFSET     0x0030    /* Sleep Control Register */
#define XMC4_SCU_DSLEEPCR_OFFSET    0x0034    /* Deep Sleep Control Register */
#define XMC4_SCU_CGATSTAT0_OFFSET   0x0040    /* Peripheral 0 Clock Gating Status */
#define XMC4_SCU_CGATSET0_OFFSET    0x0044    /* Peripheral 0 Clock Gating Set */
#define XMC4_SCU_CGATCLR0_OFFSET    0x0048    /* Peripheral 0 Clock Gating Clear */
#define XMC4_SCU_CGATSTAT1_OFFSET   0x004c    /* Peripheral 1 Clock Gating Status */
#define XMC4_SCU_CGATSET1_OFFSET    0x0050    /* Peripheral 1 Clock Gating Set */
#define XMC4_SCU_CGATCLR1_OFFSET    0x0054    /* Peripheral 1 Clock Gating Clear */
#define XMC4_SCU_CGATSTAT2_OFFSET   0x0058    /* Peripheral 2 Clock Gating Status */
#define XMC4_SCU_CGATSET2_OFFSET    0x005c    /* Peripheral 2 Clock Gating Set */
#define XMC4_SCU_CGATCLR2_OFFSET    0x0060    /* Peripheral 2 Clock Gating Clear */
#define XMC4_SCU_CGATSTAT3_OFFSET   0x0064    /* Peripheral 3 Clock Gating Status */
#define XMC4_SCU_CGATSET3_OFFSET    0x0068    /* Peripheral 3 Clock Gating Set */
#define XMC4_SCU_CGATCLR3_OFFSET    0x006c    /* Peripheral 3 Clock Gating Clear */

/* Oscillator Control SCU Registers */

#define XMC4_OCU_OSCHPSTAT_OFFSET   0x0000    /* OSC_HP Status Register */
#define XMC4_OCU_OSCHPCTRL_OFFSET   0x0004    /* OSC_HP Control Register */
#define XMC4_OCU_CLKCALCONST_OFFSET 0x000c    /* Clock Calibration Constant Register */

/* PLL Control SCU Registers */

#define XMC4_SCU_PLLSTAT_OFFSET     0x0000    /* System PLL Status Register */
#define XMC4_SCU_PLLCON0_OFFSET     0x0004    /* System PLL Configuration 0 Register */
#define XMC4_SCU_PLLCON1_OFFSET     0x0008    /* System PLL Configuration 1 Register */
#define XMC4_SCU_PLLCON2_OFFSET     0x000c    /* System PLL Configuration 2 Register */
#define XMC4_SCU_USBPLLSTAT_OFFSET  0x0010    /* USB PLL Status Register */
#define XMC4_SCU_USBPLLCON_OFFSET   0x0014    /* USB PLL Control Register */
#define XMC4_SCU_CLKMXSTAT_OFFSET   0x0028    /* Clock Multiplexing Status Register */

/* Register Addresses ***************************************************************/
/* General SCU Registers */

#define XMC4_SCU_ID                 (XMC4_SCU_GENERAL_BASE+XMC4_SCU_ID_OFFSET)
#define XMC4_SCU_IDCHIP             (XMC4_SCU_GENERAL_BASE+XMC4_SCU_IDCHIP_OFFSET)
#define XMC4_SCU_IDMANUF            (XMC4_SCU_GENERAL_BASE+XMC4_SCU_IDMANUF_OFFSET)
#define XMC4_SCU_STCON              (XMC4_SCU_GENERAL_BASE+XMC4_SCU_STCON_OFFSET)
#define XMC4_SCU_GPR0               (XMC4_SCU_GENERAL_BASE+XMC4_SCU_GPR0_OFFSET)
#define XMC4_SCU_GPR1               (XMC4_SCU_GENERAL_BASE+XMC4_SCU_GPR1_OFFSET)
#define XMC4_SCU_ETH0CON            (XMC4_SCU_GENERAL_BASE+XMC4_SCU_ETH0CON_OFFSET)
#define XMC4_SCU_CCUCON             (XMC4_SCU_GENERAL_BASE+XMC4_SCU_CCUCON_OFFSET)
#define XMC4_SCU_DTSCON             (XMC4_SCU_GENERAL_BASE+XMC4_SCU_DTSCON_OFFSET)
#define XMC4_SCU_DTSSTAT            (XMC4_SCU_GENERAL_BASE+XMC4_SCU_DTSSTAT_OFFSET)
#define XMC4_SCU_SDMMCDEL           (XMC4_SCU_GENERAL_BASE+XMC4_SCU_SDMMCDEL_OFFSET)
#define XMC4_SCU_G0ORCEN            (XMC4_SCU_GENERAL_BASE+XMC4_SCU_G0ORCEN_OFFSET)
#define XMC4_SCU_G1ORCEN            (XMC4_SCU_GENERAL_BASE+XMC4_SCU_G1ORCEN_OFFSET)
#define XMC4_SCU_MIRRSTS            (XMC4_SCU_GENERAL_BASE+XMC4_SCU_MIRRSTS_OFFSET)
#define XMC4_SCU_RMACR              (XMC4_SCU_GENERAL_BASE+XMC4_SCU_RMACR_OFFSET)
#define XMC4_SCU_RMADATA            (XMC4_SCU_GENERAL_BASE+XMC4_SCU_RMADATA_OFFSET)

/* Ethernet Control SCU Registers */

#define XMC4_SCU_ETHCON             (XMC4_ETH0_CON_BASE+XMC4_SCU_ETHCON_OFFSET)

/* Parity Control Registers */

#define XMC4_SCU_PEEN               (XMC4_SCU_PARITY_BASE+XMC4_SCU_PEEN_OFFSET)
#define XMC4_SCU_MCHKCON            (XMC4_SCU_PARITY_BASE+XMC4_SCU_MCHKCON_OFFSET)
#define XMC4_SCU_PETE               (XMC4_SCU_PARITY_BASE+XMC4_SCU_PETE_OFFSET)
#define XMC4_SCU_PERSTEN            (XMC4_SCU_PARITY_BASE+XMC4_SCU_PERSTEN_OFFSET)
#define XMC4_SCU_PEFLAG             (XMC4_SCU_PARITY_BASE+XMC4_SCU_PEFLAG_OFFSET)
#define XMC4_SCU_PMTPR              (XMC4_SCU_PARITY_BASE+XMC4_SCU_PMTPR_OFFSET)
#define XMC4_SCU_PMTSR              (XMC4_SCU_PARITY_BASE+XMC4_SCU_PMTSR_OFFSET)

/* Trap Control Registers */

#define XMC4_SCU_TRAPSTAT           (XMC4_SCU_TRAP_BASE+XMC4_SCU_TRAPSTAT_OFFSET)
#define XMC4_SCU_TRAPRAW            (XMC4_SCU_TRAP_BASE+XMC4_SCU_TRAPRAW_OFFSET)
#define XMC4_SCU_TRAPDIS            (XMC4_SCU_TRAP_BASE+XMC4_SCU_TRAPDIS_OFFSET)
#define XMC4_SCU_TRAPCLR            (XMC4_SCU_TRAP_BASE+XMC4_SCU_TRAPCLR_OFFSET)
#define XMC4_SCU_TRAPSET            (XMC4_SCU_TRAP_BASE+XMC4_SCU_TRAPSET_OFFSET)

/* Ethernet Control SCU Resters */

#define XMC4_SCU_ETHCON_OFFSET      0x0000    /* Ethernet 0 Port Control Register */
#define XMC4_SCU_ETHCON_OFFSET      0x0000    /* Ethernet 0 Port Control Register */

/* Interrupt Control SCU Registers */

#define XMC4_SCU_SRSTAT             (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_SRSTAT_OFFSET)
#define XMC4_SCU_SRRAW              (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_SRRAW_OFFSET)
#define XMC4_SCU_SRMSK              (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_SRMSK_OFFSET)
#define XMC4_SCU_SRCLR              (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_SRCLR_OFFSET)
#define XMC4_SCU_SRSET              (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_SRSET_OFFSET)
#define XMC4_SCU_NMIREQEN           (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_NMIREQEN_OFFSET)

/* SDMMC Control SCU Registers */

#define XMC4_SCU_SDMMCCON           (XMC4_SDMMC_CON_BASE+XMC4_SCU_SDMMCCON_OFFSET)

/* Power control SCU Registers */

#define XMC4_SCU_PWRSTAT            (XMC4_SCU_POWER_BASE+XMC4_SCU_PWRSTAT_OFFSET)
#define XMC4_SCU_PWRSET             (XMC4_SCU_POWER_BASE+XMC4_SCU_PWRSET_OFFSET)
#define XMC4_SCU_PWRCLR             (XMC4_SCU_POWER_BASE+XMC4_SCU_PWRCLR_OFFSET)
#define XMC4_SCU_EVRSTAT            (XMC4_SCU_POWER_BASE+XMC4_SCU_EVRSTAT_OFFSET)
#define XMC4_SCU_EVRVADCSTAT        (XMC4_SCU_POWER_BASE+XMC4_SCU_EVRVADCSTAT_OFFSET)
#define XMC4_SCU_PWRMON             (XMC4_SCU_POWER_BASE+XMC4_SCU_PWRMON_OFFSET)

/* Hibernation SCU Registers */

#define XMC4_SCU_HDSTAT             (XMC4_SCU_HIBERNATE_BASE+XMC4_SCU_HDSTAT_OFFSET)
#define XMC4_SCU_HDCLR              (XMC4_SCU_HIBERNATE_BASE+XMC4_SCU_HDCLR_OFFSET)
#define XMC4_SCU_HDSET              (XMC4_SCU_HIBERNATE_BASE+XMC4_SCU_HDSET_OFFSET)
#define XMC4_SCU_HDCR               (XMC4_SCU_HIBERNATE_BASE+XMC4_SCU_HDCR_OFFSET)
#define XMC4_SCU_OSCSICTRL          (XMC4_SCU_HIBERNATE_BASE+XMC4_SCU_OSCSICTRL_OFFSET)
#define XMC4_SCU_OSCULSTAT          (XMC4_SCU_HIBERNATE_BASE+XMC4_SCU_OSCULSTAT_OFFSET)
#define XMC4_SCU_OSCULCTRL          (XMC4_SCU_HIBERNATE_BASE+XMC4_SCU_OSCULCTRL_OFFSET)

/* Reset SCU Registers */

#define XMC4_SCU_RSTSTAT            (XMC4_SCU_RESET_BASE+XMC4_SCU_RSTSTAT_OFFSET)
#define XMC4_SCU_RSTSET             (XMC4_SCU_RESET_BASE+XMC4_SCU_RSTSET_OFFSET)
#define XMC4_SCU_RSTCLR             (XMC4_SCU_RESET_BASE+XMC4_SCU_RSTCLR_OFFSET)
#define XMC4_SCU_PRSTAT0            (XMC4_SCU_RESET_BASE+XMC4_SCU_PRSTAT0_OFFSET)
#define XMC4_SCU_PRSET0             (XMC4_SCU_RESET_BASE+XMC4_SCU_PRSET0_OFFSET)
#define XMC4_SCU_PRCLR0             (XMC4_SCU_RESET_BASE+XMC4_SCU_PRCLR0_OFFSET)
#define XMC4_SCU_PRSTAT1            (XMC4_SCU_RESET_BASE+XMC4_SCU_PRSTAT1_OFFSET)
#define XMC4_SCU_PRSET1             (XMC4_SCU_RESET_BASE+XMC4_SCU_PRSET1_OFFSET)
#define XMC4_SCU_PRCLR1             (XMC4_SCU_RESET_BASE+XMC4_SCU_PRCLR1_OFFSET)
#define XMC4_SCU_PRSTAT2            (XMC4_SCU_RESET_BASE+XMC4_SCU_PRSTAT2_OFFSET)
#define XMC4_SCU_PRSET2             (XMC4_SCU_RESET_BASE+XMC4_SCU_PRSET2_OFFSET)
#define XMC4_SCU_PRCLR2             (XMC4_SCU_RESET_BASE+XMC4_SCU_PRCLR2_OFFSET)
#define XMC4_SCU_PRSTAT3            (XMC4_SCU_RESET_BASE+XMC4_SCU_PRSTAT3_OFFSET)
#define XMC4_SCU_PRSET3             (XMC4_SCU_RESET_BASE+XMC4_SCU_PRSET3_OFFSET)
#define XMC4_SCU_PRCLR3             (XMC4_SCU_RESET_BASE+XMC4_SCU_PRCLR3_OFFSET)

/* Clock Control SCU Registers */

#define XMC4_SCU_CLKSTAT            (XMC4_SCU_CLK_BASE+XMC4_SCU_CLKSTAT_OFFSET)
#define XMC4_SCU_CLKSET             (XMC4_SCU_CLK_BASE+XMC4_SCU_CLKSET_OFFSET)
#define XMC4_SCU_CLKCLR             (XMC4_SCU_CLK_BASE+XMC4_SCU_CLKCLR_OFFSET)
#define XMC4_SCU_SYSCLKCR           (XMC4_SCU_CLK_BASE+XMC4_SCU_SYSCLKCR_OFFSET)
#define XMC4_SCU_CPUCLKCR           (XMC4_SCU_CLK_BASE+XMC4_SCU_CPUCLKCR_OFFSET)
#define XMC4_SCU_PBCLKCR            (XMC4_SCU_CLK_BASE+XMC4_SCU_PBCLKCR_OFFSET)
#define XMC4_SCU_USBCLKCR           (XMC4_SCU_CLK_BASE+XMC4_SCU_USBCLKCR_OFFSET)
#define XMC4_SCU_EBUCLKCR           (XMC4_SCU_CLK_BASE+XMC4_SCU_EBUCLKCR_OFFSET)
#define XMC4_SCU_CCUCLKCR           (XMC4_SCU_CLK_BASE+XMC4_SCU_CCUCLKCR_OFFSET)
#define XMC4_SCU_WDTCLKCR           (XMC4_SCU_CLK_BASE+XMC4_SCU_WDTCLKCR_OFFSET)
#define XMC4_SCU_EXTCLKCR           (XMC4_SCU_CLK_BASE+XMC4_SCU_EXTCLKCR_OFFSET)
#define XMC4_SCU_SLEEPCR            (XMC4_SCU_CLK_BASE+XMC4_SCU_SLEEPCR_OFFSET)
#define XMC4_SCU_DSLEEPCR           (XMC4_SCU_CLK_BASE+XMC4_SCU_DSLEEPCR_OFFSET)
#define XMC4_SCU_CGATSTAT0          (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSTAT0_OFFSET)
#define XMC4_SCU_CGATSET0           (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSET0_OFFSET)
#define XMC4_SCU_CGATCLR0           (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATCLR0_OFFSET)
#define XMC4_SCU_CGATSTAT1          (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSTAT1_OFFSET)
#define XMC4_SCU_CGATSET1           (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSET1_OFFSET)
#define XMC4_SCU_CGATCLR1           (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATCLR1_OFFSET)
#define XMC4_SCU_CGATSTAT2          (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSTAT2_OFFSET)
#define XMC4_SCU_CGATSET2           (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSET2_OFFSET
#define XMC4_SCU_CGATCLR2           (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATCLR2_OFFSET
#define XMC4_SCU_CGATSTAT3          (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSTAT3_OFFSET
#define XMC4_SCU_CGATSET3           (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSET3_OFFSET
#define XMC4_SCU_CGATCLR3           (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATCLR3_OFFSET_

/* Oscillator Control SCU Registers */

#define XMC4_OSCU_OSCHPSTAT         (XMC4_SCU_OSC_BASE+XMC4_OSCU_OSCHPSTAT_OFFSET)
#define XMC4_OSCU_OSCHPCTRL         (XMC4_SCU_OSC_BASE+XMC4_OSCU_OSCHPCTRL_OFFSET)
#define XMC4_OSCU_CLKCALCONST       (XMC4_SCU_OSC_BASE+XMC4_OSCU_CLKCALCONST_OFFSET)

/* PLL Control SCU Registers */

#define XMC4_SCU_PLLSTAT            (XMC4_SCU_PLL_BASE+XMC4_SCU_PLLSTAT_OFFSET)
#define XMC4_SCU_PLLCON0            (XMC4_SCU_PLL_BASE+XMC4_SCU_PLLCON0_OFFSET)
#define XMC4_SCU_PLLCON1            (XMC4_SCU_PLL_BASE+XMC4_SCU_PLLCON1_OFFSET)
#define XMC4_SCU_PLLCON2            (XMC4_SCU_PLL_BASE+XMC4_SCU_PLLCON2_OFFSET)
#define XMC4_SCU_USBPLLSTAT         (XMC4_SCU_PLL_BASE+XMC4_SCU_USBPLLSTAT_OFFSET)
#define XMC4_SCU_USBPLLCON          (XMC4_SCU_PLL_BASE+XMC4_SCU_USBPLLCON_OFFSET)
#define XMC4_SCU_CLKMXSTAT          (XMC4_SCU_PLL_BASE+XMC4_SCU_CLKMXSTAT_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* General SCU Registers */

/* Module Identification Register */
#define SCU_ID_
/* Chip ID */
#define SCU_IDCHIP_
/* Manufactory ID */
#define SCU_IDMANUF_
/* Start-up Control */
#define SCU_STCON_
/* General Purpose Register 0 */
#define SCU_GPR0_
/* General Purpose Register 1 */
#define SCU_GPR1_
/* Ethernet 0 Port Control */
#define SCU_ETH0CON_
/* CCUx Global Start Control Register */
#define SCU_CCUCON_
/* DTS Control */
#define SCU_DTSCON_
/* DTS Status */
#define SCU_DTSSTAT_
/* SD-MMC Delay Control Register */
#define SCU_SDMMCDEL_
/* Out-Of-Range Comparator Enable Register 0 */
#define SCU_G0ORCEN_
/* Out-Of-Range Comparator Enable Register 1 */
#define SCU_G1ORCEN_
/* Mirror Update Status Register */
#define SCU_MIRRSTS_

/* Ethernet Control SCU Resters */

/* Ethernet 0 Port Control Register */
#define SCU_ETHCON_

/* Interrupt Control SCU Registers */

/* Service Request Status */
#define SCU_SRSTAT_
/* RAW Service Request Status */
#define SCU_SRRAW_
/* Service Request Mask */
#define SCU_SRMSK_
/* Service Request Clear */
#define SCU_SRCLR_
/* Service Request Set */
#define SCU_SRSET_
/* Enable Promoting Events to NMI Request */
#define SCU_NMIREQEN_
/* Retention Memory Access Control Register */
#define SCU_RMACR_
/* Retention Memory Access Data Register */
#define SCU_RMADATA_
/* Parity Error Enable Register */

/* SDMMC Control SCU Registers */

/* SDMMC Configuration */
#define SCU_SDMMCCON_

/* Parity Control Registers */

#define SCU_PEEN_
/* Memory Checking Control Register */
#define SCU_MCHKCON_
/* Parity Error Trap Enable Register */
#define SCU_PETE_
/* Reset upon Parity Error Enable Register */
#define SCU_PERSTEN_
/* Parity Error Control Register */
#define SCU_PEFLAG_
/* Parity Memory Test Pattern Register */
#define SCU_PMTPR_
/* Parity Memory Test Select Register */
#define SCU_PMTSR_

/* Trap Control Registers */

/* Trap Status Register, Trap Raw Status Register, Trap Mask Register, Trap Clear
 * Register, and Trap Set Register
 */

#define SCU_TRAP_SOSCWDGT           (1 << 0)  /* Bit 0:  OSC_HP Oscillator Watchdog Trap */
#define SCU_TRAP_SVCOLCKT           (1 << 2)  /* Bit 2:  System VCO Lock Trap */
#define SCU_TRAP_UVCOLCKT           (1 << 3)  /* Bit 3:  USB VCO Lock Trap */
#define SCU_TRAP_PET                (1 << 4)  /* Bit 4:  Parity Error Trap */
#define SCU_TRAP_BRWNT              (1 << 5)  /* Bit 5:  Brown Out Trap */
#define SCU_TRAP_ULPWDGT            (1 << 6)  /* Bit 6:  OSC_ULP Oscillator Watchdog Trap */
#define SCU_TRAP_BWERR0T            (1 << 7)  /* Bit 7:  Peripheral Bridge 0 Trap */
#define SCU_TRAP_BWERR1T            (1 << 8)  /* Bit 8:  Peripheral Bridge 1 Trap */

/* Power Control SCU Registers */

/* Power Status Register, Power Set Control Register, and Power Clear
 * Control Register
 */

#define SCU_PWR_HIBEN               (1 << 0)  /* Bit 0:  Hibernate Domain Enable State */
#define SCU_PWR_USBPHYPDQ           (1 << 16) /* Bit 16: USB PHY Transceiver State */
#define SCU_PWR_USBOTGEN            (1 << 17) /* Bit 17: USB On-The-Go Comparators State */
#define SCU_PWR_USBPUWQ             (1 << 18) /* Bit 18: USB Weak Pull-Up at PADN State */

/* EVR Status Register */
#define SCU_EVRSTAT_
/* EVR VADC Status Register */
#define SCU_EVRVADCSTAT_
/* Power Monitor Value */
#define SCU_PWRMON_

/* HCU Registers */

/* Hibernate Domain Status Register */
#define SCU_HDSTAT_
/* Hibernate Domain Status Clear Register */
#define SCU_HDCLR_
/* Hibernate Domain Status Set Register */
#define SCU_HDSET_
/* Hibernate Domain Control Register */
#define SCU_HDCR_
/* Internal 32.768 kHz Clock Source Control Register */
#define SCU_OSCSICTRL_
/* OSC_ULP Status Register */
#define SCU_OSCULSTAT_
/* OSC_ULP Control Register */
#define SCU_OSCULCTRL_

/* Reset SCU Registers */

/* System Reset Status */

#define SCU_RSTSTAT_RSTSTAT_SHIFT   (0)     /* Bits 0-7: Reset Status Information */
#define SCU_RSTSTAT_RSTSTAT_MASK    (0xff << SCU_RSTSTAT_RSTSTAT_SHIFT)
#  define SCU_RSTSTAT_RSTSTAT_PORST   (1 << SCU_RSTSTAT_RSTSTAT_SHIFT)   /* PORST reset */
#  define SCU_RSTSTAT_RSTSTAT_SWD     (2 << SCU_RSTSTAT_RSTSTAT_SHIFT)   /* SWD reset */
#  define SCU_RSTSTAT_RSTSTAT_PV      (4 << SCU_RSTSTAT_RSTSTAT_SHIFT)   /* PV reset */
#  define SCU_RSTSTAT_RSTSTAT_CPUSYS  (8 << SCU_RSTSTAT_RSTSTAT_SHIFT)   /* CPU system reset */
#  define SCU_RSTSTAT_RSTSTAT_CPULOCK (16 << SCU_RSTSTAT_RSTSTAT_SHIFT)  /* CPU lockup reset */
#  define SCU_RSTSTAT_RSTSTAT_WDT     (32 << SCU_RSTSTAT_RSTSTAT_SHIFT)  /* WDT reset */
#  define SCU_RSTSTAT_RSTSTAT_PERR    (128 << SCU_RSTSTAT_RSTSTAT_SHIFT) /* Parity Error reset */

#define SCU_RSTSTAT_HIBWK           (1 << 8)  /* Bit 8:  Hibernate Wake-up Status */
#define SCU_RSTSTAT_HIBRS           (1 << 9)  /* Bit 9:  Hibernate Reset Status */
#define SCU_RSTSTAT_LCKEN           (1 << 10) /* Bit 10: Enable Lockup Status */

/* Reset Set Register */

#define SCU_RSTSET_HIBWK            (1 << 8)  /* Bit 8:  Hibernate Wake-up Reset Status */
#define SCU_RSTSET_HIBRS            (1 << 9)  /* Bit 9:  Hibernate Reset Reset Status */
#define SCU_RSTSET_LCKEN            (1 << 10) /* Bit 10: Enable Lockup Reset Status */

/* Reset Clear Register */

#define SCU_RSTCLR_RSCLR            (1 << 0)  /* Bit 0:  Clear Reset Status */
#define SCU_RSTCLR_HIBWK            (1 << 8)  /* Bit 8:  Clear Hibernate Wake-up Reset Status */
#define SCU_RSTCLR_HIBRS            (1 << 9)  /* Bit 9:  Clear Hibernate Reset */
#define SCU_RSTCLR_LCKEN            (1 << 10) /* Bit 10: Clear Hibernate Reset */

/* Peripheral Reset Status Register 0 */
#define SCU_PRSTAT0_
/* Peripheral Reset Set Register 0 */
#define SCU_PRSET0_
/* Peripheral Reset Clear Register 0 */
#define SCU_PRCLR0_
/* Peripheral Reset Status Register 1 */
#define SCU_PRSTAT1_
/* Peripheral Reset Set Register 1 */
#define SCU_PRSET1_
/* Peripheral Reset Clear Register 1 */
#define SCU_PRCLR1_
/* Peripheral Reset Status Register 2 */
#define SCU_PRSTAT2_
/* Peripheral Reset Set Register 2 */
#define SCU_PRSET2_
/* Peripheral Reset Clear Register 2 */
#define SCU_PRCLR2_
/* Peripheral Reset Status Register 3 */
#define SCU_PRSTAT3_
/* Peripheral Reset Set Register 3 */
#define SCU_PRSET3_
/* Peripheral Reset Clear Register 3 */
#define SCU_PRCLR3_

/* Clock Control SCU Registers */

/* Clock Status Register */
#define SCU_CLKSTAT_
/* Clock Set Control Register */
#define SCU_CLKSET_
/* Clock clear Control Register */
#define SCU_CLKCLR_

/* System Clock Control */

#define SCU_SYSCLKCR_SYSDIV_SHIFT   (0)  /* Bits 0-7: System Clock Division Value */
#define SCU_SYSCLKCR_SYSDIV_MASK    (0xff << SCU_CLK_SYSCLKCR_SYSDIV_SHIFT)
#  define SCU_SYSCLKCR_SYSDIV(n)    ((uint32_t)((n)-1) << SCU_CLK_SYSCLKCR_SYSDIV_SHIFT)

#define SCU_SYSCLKCR_SYSSEL         (1 << 16)  /* Bit 16: System Clock Selection Value */
#  define SCU_SYSCLKCR_SYSSEL_OFI   (0)        /*         0=OFI clock */
#  define SCU_SYSCLKCR_SYSSEL_PLL   (1 << 16)  /*         1=PLL clock */

/* CPU Clock Control */

#define SCU_CPUCLKCR_CPUDIV         (1 << 0)   /* Bit 0: CPU Clock Divider Enable */

/* Peripheral Bus Clock Control */
#define SCU_PBCLKCR_
/* USB Clock Control */
#define SCU_USBCLKCR_
/* EBU Clock Control */
#define SCU_EBUCLKCR_
/* CCU Clock Control */
#define SCU_CCUCLKCR_
/* WDT Clock Control */
#define SCU_WDTCLKCR_
/* External clock Control Register */
#define SCU_EXTCLKCR_
/* Sleep Control Register */
#define SCU_SLEEPCR_
/* Deep Sleep Control Register */
#define SCU_DSLEEPCR_
/* Peripheral 0 Clock Gating Status */
#define SCU_CGATSTAT0_
/* Peripheral 0 Clock Gating Set */
#define SCU_CGATSET0_
/* Peripheral 0 Clock Gating Clear */
#define SCU_CGATCLR0_
/* Peripheral 1 Clock Gating Status */
#define SCU_CGATSTAT1_
/* Peripheral 1 Clock Gating Set */
#define SCU_CGATSET1_
/* Peripheral 1 Clock Gating Clear */
#define SCU_CGATCLR1_
/* Peripheral 2 Clock Gating Status */
#define SCU_CGATSTAT2_
/* Peripheral 2 Clock Gating Set */
#define SCU_CGATSET2_
/* Peripheral 2 Clock Gating Clear */
#define SCU_CGATCLR2_
/* Peripheral 3 Clock Gating Status */
#define SCU_CGATSTAT3_
/* Peripheral 3 Clock Gating Set */
#define SCU_CGATSET3_
/* Peripheral 3 Clock Gating Clear */
#define SCU_CGATCLR3_

/* Oscillator Control SCU Registers */

/* OSC_HP Status Register */
#define OSCU_OSCHPSTAT_
/* OSC_HP Control Register */
#define OSCU_OSCHPCTRL_
/* Clock Calibration Constant Register */
#define OSCU_CLKCALCONST_

/* PLL Control SCU Registers */

/* System PLL Status Register */

#define SCU_PLLSTAT_VCOBYST         (1 << 0)  /* Bit 0:  VCO Bypass Status */
#define SCU_PLLSTAT_PWDSTAT         (1 << 1)  /* Bit 1:  PLL Power-saving Mode Status */
#define SCU_PLLSTAT_VCOLOCK         (1 << 2)  /* Bit 2:  PLL LOCK Status */
#define SCU_PLLSTAT_K1RDY           (1 << 4)  /* Bit 4:  K1 Divider Ready Status */
#define SCU_PLLSTAT_K2RDY           (1 << 5)  /* Bit 5:  K2 Divider Ready Status */
#define SCU_PLLSTAT_BY              (1 << 6)  /* Bit 6:  Bypass Mode Status */
#define SCU_PLLSTAT_PLLLV           (1 << 7)  /* Bit 7:  Oscillator for PLL Valid Low Status */
#define SCU_PLLSTAT_PLLHV           (1 << 8)  /* Bit 8:  Oscillator for PLL Valid High Status */
#define SCU_PLLSTAT_PLLSP           (1 << 9)  /* Bit 9:  Oscillator for PLL Valid Spike Status */

/* System PLL Configuration 0 Register */

#define SCU_PLLCON0_VCOBYP          (1 << 0)  /* Bit 0:  VCO Bypass */
#define SCU_PLLCON0_VCOPWD          (1 << 1)  /* Bit 1:  VCO Power Saving Mode */
#define SCU_PLLCON0_VCOTR           (1 << 2)  /* Bit 2:  VCO Trim Control */
#define SCU_PLLCON0_FINDIS          (1 << 4)  /* Bit 4:  Disconnect Oscillator from VCO */
#define SCU_PLLCON0_OSCDISCDIS      (1 << 6)  /* Bit 6:  Oscillator Disconnect Disable */
#define SCU_PLLCON0_PLLPWD          (1 << 16) /* Bit 16: PLL Power Saving Mode */
#define SCU_PLLCON0_OSCRES          (1 << 17) /* Bit 17: Oscillator Watchdog Reset */
#define SCU_PLLCON0_RESLD           (1 << 18) /* Bit 18: Restart VCO Lock Detection */
#define SCU_PLLCON0_AOTREN          (1 << 19) /* Bit 19: Automatic Oscillator Calibration Enable */
#define SCU_PLLCON0_FOTR            (1 << 20) /* Bit 20: Factory Oscillator Calibration */

/* System PLL Configuration 1 Register */

#define SCU_PLLCON1_K1DIV_SHIFT     (0)       /* Bits 0-6: K1-Divider Value */
#define SCU_PLLCON1_K1DIV_MASK      (0x7f << SCU_PLLCON1_K1DIV_SHIFT)
#  define SCU_PLLCON1_K1DIV(n)      ((uint32_t)((n)-1) << SCU_PLLCON1_K1DIV_SHIFT)
#define SCU_PLLCON1_NDIV_SHIFT      (8)       /* Bits 8-14: N-Divider Value */
#define SCU_PLLCON1_NDIV_MASK       (0x7f << SCU_PLLCON1_NDIV_SHIFT)
#  define SCU_PLLCON1_NDIV(n)       ((uint32_t)((n)-1) << SCU_PLLCON1_NDIV_SHIFT)
#define SCU_PLLCON1_K2DIV_SHIFT     (16)      /* Bit 16-22: K2-Divider Value */
#define SCU_PLLCON1_K2DIV_MASK      (0x7f << SCU_PLLCON1_K2DIV_SHIFT)
#  define SCU_PLLCON1_K2DIV(n)      ((uint32_t)((n)-1) << SCU_PLLCON1_K2DIV_SHIFT)
#define SCU_PLLCON1_PDIV_SHIFT      (24)      /* Bits 24-27: P-Divider Value */
#define SCU_PLLCON1_PDIV_MASK       (0x7f << SCU_PLLCON1_PDIV_SHIFT)
#  define SCU_PLLCON1_PDIV(n)       ((uint32_t)((n)-1) << SCU_PLLCON1_PDIV_SHIFT)

/* System PLL Configuration 2 Register */

#define SCU_PLLCON2_PINSEL          (1 << 0)  /* Bit 0: P-Divider Input Selection */
#  define SCU_PLLCON2_PINSEL_PLL    (0)       /*        0=PLL external oscillator selected */
#  define SCU_PLLCON2_PINSEL_OFI    (1 << 0)  /*        1=Backup clock source selected */
#define SCU_PLLCON2_K1INSEL         (1 << 8)  /* Bit 8: K1-Divider Input */
#  define SCU_PLLCON2_K1INSEL_PLL   (0)       /*        0=PLL external oscillator selected */
#  define SCU_PLLCON2_K1INSEL_OFI   (1 << 8)  /*        1=Backup clock source selected */

/* USB PLL Status Register */
#define SCU_USBPLLSTAT_
/* USB PLL Control Register */
#define SCU_USBPLLCON_
/* Clock Multiplexing Status Register */
#define SCU_CLKMXSTAT_

#endif /* __ARCH_ARM_SRC_XMC4_CHIP_XMC4_SCU_H */
