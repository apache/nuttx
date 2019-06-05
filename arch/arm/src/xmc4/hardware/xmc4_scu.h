/************************************************************************************
 * arch/arm/src/xmc4/hardware/xmc4_scu.h
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

#ifndef __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_SCU_H
#define __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_SCU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/xmc4_memorymap.h"

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
#define XMC4_SCU_SDMMCCON_OFFSET    0x00b4    /* SDMMC Configuration */
#define XMC4_SCU_MIRRSTS_OFFSET     0x00c4    /* Mirror Update Status Register */
#define XMC4_SCU_RMACR_OFFSET       0x00c8    /* Retention Memory Access Control Register */
#define XMC4_SCU_RMADATA_OFFSET     0x00cc    /* Retention Memory Access Data Register */

/* Interrupt Control SCU Registers */

#define XMC4_SCU_SRSTAT_OFFSET      0x0000    /* Service Request Status */
#define XMC4_SCU_SRRAW_OFFSET       0x0004    /* RAW Service Request Status */
#define XMC4_SCU_SRMSK_OFFSET       0x0008    /* Service Request Mask */
#define XMC4_SCU_SRCLR_OFFSET       0x000c    /* Service Request Clear */
#define XMC4_SCU_SRSET_OFFSET       0x0010    /* Service Request Set */
#define XMC4_SCU_NMIREQEN_OFFSET    0x0014    /* Enable Promoting Events to NMI Request */

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
#ifdef XMC4_SCU_GATING
#  define XMC4_SCU_CGATSTAT0_OFFSET 0x0040    /* Peripheral 0 Clock Gating Status */
#  define XMC4_SCU_CGATSET0_OFFSET  0x0044    /* Peripheral 0 Clock Gating Set */
#  define XMC4_SCU_CGATCLR0_OFFSET  0x0048    /* Peripheral 0 Clock Gating Clear */
#  define XMC4_SCU_CGATSTAT1_OFFSET 0x004c    /* Peripheral 1 Clock Gating Status */
#  define XMC4_SCU_CGATSET1_OFFSET  0x0050    /* Peripheral 1 Clock Gating Set */
#  define XMC4_SCU_CGATCLR1_OFFSET  0x0054    /* Peripheral 1 Clock Gating Clear */
#  define XMC4_SCU_CGATSTAT2_OFFSET 0x0058    /* Peripheral 2 Clock Gating Status */
#  define XMC4_SCU_CGATSET2_OFFSET  0x005c    /* Peripheral 2 Clock Gating Set */
#  define XMC4_SCU_CGATCLR2_OFFSET  0x0060    /* Peripheral 2 Clock Gating Clear */
#  define XMC4_SCU_CGATSTAT3_OFFSET 0x0064    /* Peripheral 3 Clock Gating Status */
#  define XMC4_SCU_CGATSET3_OFFSET  0x0068    /* Peripheral 3 Clock Gating Set */
#  define XMC4_SCU_CGATCLR3_OFFSET  0x006c    /* Peripheral 3 Clock Gating Clear */
#endif

/* Oscillator Control SCU Registers */

#define XMC4_SCU_OSCHPSTAT_OFFSET   0x0000    /* OSC_HP Status Register */
#define XMC4_SCU_OSCHPCTRL_OFFSET   0x0004    /* OSC_HP Control Register */
#define XMC4_SCU_CLKCALCONST_OFFSET 0x000c    /* Clock Calibration Constant Register */

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
#define XMC4_SCU_SDMMCCON           (XMC4_SDMMC_CON_BASE+XMC4_SCU_SDMMCCON_OFFSET)
#define XMC4_SCU_MIRRSTS            (XMC4_SCU_GENERAL_BASE+XMC4_SCU_MIRRSTS_OFFSET)
#define XMC4_SCU_RMACR              (XMC4_SCU_GENERAL_BASE+XMC4_SCU_RMACR_OFFSET)
#define XMC4_SCU_RMADATA            (XMC4_SCU_GENERAL_BASE+XMC4_SCU_RMADATA_OFFSET)

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

/* Interrupt Control SCU Registers */

#define XMC4_SCU_SRSTAT             (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_SRSTAT_OFFSET)
#define XMC4_SCU_SRRAW              (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_SRRAW_OFFSET)
#define XMC4_SCU_SRMSK              (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_SRMSK_OFFSET)
#define XMC4_SCU_SRCLR              (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_SRCLR_OFFSET)
#define XMC4_SCU_SRSET              (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_SRSET_OFFSET)
#define XMC4_SCU_NMIREQEN           (XMC4_SCU_INTERRUPT_BASE+XMC4_SCU_NMIREQEN_OFFSET)

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
#ifdef XMC4_SCU_GATING
#  define XMC4_SCU_CGATSTAT0        (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSTAT0_OFFSET)
#  define XMC4_SCU_CGATSET0         (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSET0_OFFSET)
#  define XMC4_SCU_CGATCLR0         (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATCLR0_OFFSET)
#  define XMC4_SCU_CGATSTAT1        (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSTAT1_OFFSET)
#  define XMC4_SCU_CGATSET1         (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSET1_OFFSET)
#  define XMC4_SCU_CGATCLR1         (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATCLR1_OFFSET)
#  define XMC4_SCU_CGATSTAT2        (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSTAT2_OFFSET)
#  define XMC4_SCU_CGATSET2         (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSET2_OFFSET)
#  define XMC4_SCU_CGATCLR2         (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATCLR2_OFFSET)
#  define XMC4_SCU_CGATSTAT3        (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSTAT3_OFFSET)
#  define XMC4_SCU_CGATSET3         (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATSET3_OFFSET)
#  define XMC4_SCU_CGATCLR3         (XMC4_SCU_CLK_BASE+XMC4_SCU_CGATCLR3_OFFSET)
#endif

/* Oscillator Control SCU Registers */

#define XMC4_SCU_OSCHPSTAT          (XMC4_SCU_OSC_BASE+XMC4_SCU_OSCHPSTAT_OFFSET)
#define XMC4_SCU_OSCHPCTRL          (XMC4_SCU_OSC_BASE+XMC4_SCU_OSCHPCTRL_OFFSET)
#define XMC4_SCU_CLKCALCONST        (XMC4_SCU_OSC_BASE+XMC4_SCU_CLKCALCONST_OFFSET)

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

/* Module Identification Register (32-bit Chip ID) */

#define SCU_ID_MOD_REV_SHIFT        (0)       /* Bits 0-7: Module Revision Number */
#define SCU_ID_MOD_REV_MASK         (0xff << SCU_ID_MOD_REV_SHIFT)
#define SCU_ID_MOD_TYPE_SHIFT       (8)       /* Bits 8-15: Module Type */
#define SCU_ID_MOD_TYPE_MASK        (0xff << SCU_ID_MOD_REV_SHIFT)
#define SCU_ID_MOD_NUMBER_SHIFT     (16)      /* Bits 16-31: Module Number Value */
#define SCU_ID_MOD_NUMBER_MASK      (0xffff << SCU_ID_MOD_NUMBER_SHIFT)

/* Chip ID (32-bit Chip ID) */

/* Manufactory ID */

#define SCU_IDMANUF_DEPT_SHIFT      (0)       /* Bits 0-4: Department Identification Number */
#define SCU_IDMANUF_DEPT_MASK       (31 << SCU_IDMANUF_MOD_DEPT_SHIFT)
#define SCU_IDMANUF_MANUF_SHIFT     (5)       /* Bits 5-15: Manufacturer Identification Number */
#define SCU_IDMANUF_MANUF_MASK      (0x7ff << SCU_IDMANUF_MOD_MANUF_SHIFT)

/* Start-up Control */

#define SCU_STCON_HWCON_SHIFT       (0)       /* Bits 0-1: HW Configuration */
#define SCU_STCON_HWCON_MASK        (3 << SCU_STCON_HWCON_SHIFT)
#  define SCU_STCON_HWCON_JTAG      (0 << SCU_STCON_HWCON_SHIFT)  /* Normal mode, JTAG */
#  define SCU_STCON_HWCON_ACBSL     (1 << SCU_STCON_HWCON_SHIFT)  /* ASC BSL enabled */
#  define SCU_STCON_HWCON_BMI       (2 << SCU_STCON_HWCON_SHIFT)  /* BMI customized boot enabled */
#  define SCU_STCON_HWCON_CANBSL    (3 << SCU_STCON_HWCON_SHIFT)  /* CAN BSL enabled */
#define SCU_STCON_SWCON_SHIFT       (8)       /* Bits 8-11: SW Configuration */
#define SCU_STCON_SWCON_MASK        (15 << SCU_STCON_SWCON_SHIFT)
#  define SCU_STCON_SWCON_ ROM      (0 << SCU_STCON_SWCON_SHIFT)  /* Normal boot from Boot ROM */
#  define SCU_STCON_SWCON_ASCBSL    (1 << SCU_STCON_SWCON_SHIFT)  /* ASC BSL enabled */
#  define SCU_STCON_SWCON_BMI       (2 << SCU_STCON_SWCON_SHIFT)  /* BMI customized boot enabled */
#  define SCU_STCON_SWCON_CANBSL    (3 << SCU_STCON_SWCON_SHIFT)  /* CAN BSL enabled */
#  define SCU_STCON_SWCON_SRAM      (4 << SCU_STCON_SWCON_SHIFT)  /* Boot from Code SRAM */
#  define SCU_STCON_SWCON_FLASH0    (8 << SCU_STCON_SWCON_SHIFT)  /* Boot from alternate Flash Address 0 */
#  define SCU_STCON_SWCON_FLASH1    (12 << SCU_STCON_SWCON_SHIFT) /* Boot from alternate Flash Address 1 */
#  define SCU_STCON_SWCON_ABM       (15 << SCU_STCON_SWCON_SHIFT) /* Enable fallback Alternate Boot Mode (ABM) */

/* General Purpose Register 0 and General Purpose Register 1 (32-bit data) */

/* Ethernet 0 Port Control */

#define SCU_ETH0CON_RXD0_SHIFT      (0)       /* Bits 0-1: MAC Receive Input 0 */
#define SCU_ETH0CON_RXD0_MASK       (3 << SCU_ETH0CON_RXD0_SHIFT)
#  define SCU_ETH0CON_RXD0A         (0 << SCU_ETH0CON_RXD0_SHIFT) /* Data input RXD0A is selected */
#  define SCU_ETH0CON_RXD0B         (1 << SCU_ETH0CON_RXD0_SHIFT) /* Data input RXD0B is selected */
#  define SCU_ETH0CON_RXD0C         (2 << SCU_ETH0CON_RXD0_SHIFT) /* Data input RXD0C is selected */
#  define SCU_ETH0CON_RXD0D         (3 << SCU_ETH0CON_RXD0_SHIFT) /* Data input RXD0D is selected */
#define SCU_ETH0CON_RXD1_SHIFT      (2)       /* Bits 2-3: MAC Receive Input 1 */
#define SCU_ETH0CON_RXD1_MASK       (3 << SCU_ETH0CON_RXD1_SHIFT)
#  define SCU_ETH0CON_RXD1A         (0 << SCU_ETH0CON_RXD1_SHIFT) /* Data input RXD1A is selected */
#  define SCU_ETH0CON_RXD1B         (1 << SCU_ETH0CON_RXD1_SHIFT) /* Data input RXD1B is selected */
#  define SCU_ETH0CON_RXD1C         (2 << SCU_ETH0CON_RXD1_SHIFT) /* Data input RXD1C is selected */
#  define SCU_ETH0CON_RXD1D         (3 << SCU_ETH0CON_RXD1_SHIFT) /* Data input RXD1D is selected */
#define SCU_ETH0CON_RXD2_SHIFT      (4)       /* Bits 4-5: MAC Receive Input 2 */
#define SCU_ETH0CON_RXD2_MASK       (3 << SCU_ETH0CON_RXD2_SHIFT)
#  define SCU_ETH0CON_RXD2A         (0 << SCU_ETH0CON_RXD2_SHIFT) /* Data input RXD2A is selected */
#  define SCU_ETH0CON_RXD2B         (1 << SCU_ETH0CON_RXD2_SHIFT) /* Data input RXD2B is selected */
#  define SCU_ETH0CON_RXD2C         (2 << SCU_ETH0CON_RXD2_SHIFT) /* Data input RXD2C is selected */
#  define SCU_ETH0CON_RXD2D         (3 << SCU_ETH0CON_RXD2_SHIFT) /* Data input RXD2D is selected */
#define SCU_ETH0CON_RXD3_SHIFT      (6)       /* Bits 6-7: MAC Receive Input 3 */
#define SCU_ETH0CON_RXD3_MASK       (3 << SCU_ETH0CON_RXD3_SHIFT)
#  define SCU_ETH0CON_RXD3A         (0 << SCU_ETH0CON_RXD3_SHIFT) /* Data input RXD3A is selected */
#  define SCU_ETH0CON_RXD3B         (1 << SCU_ETH0CON_RXD3_SHIFT) /* Data input RXD3B is selected */
#  define SCU_ETH0CON_RXD3C         (2 << SCU_ETH0CON_RXD3_SHIFT) /* Data input RXD3C is selected */
#  define SCU_ETH0CON_RXD3D         (3 << SCU_ETH0CON_RXD3_SHIFT) /* Data input RXD3D is selected */
#define SCU_ETH0CON_CLKRMII_SHIFT   (8)       /* Bits 8-9: RMII clock input */
#define SCU_ETH0CON_CLKRMII_MASK    (3 << SCU_ETH0CON_CLKRMII_SHIFT)
#  define SCU_ETH0CON_CLKRMIIA      (0 << SCU_ETH0CON_CLKRMII_SHIFT) /* Data input RMIIA is selected */
#  define SCU_ETH0CON_CLKRMIIB      (1 << SCU_ETH0CON_CLKRMII_SHIFT) /* Data input RMIIB is selected */
#  define SCU_ETH0CON_CLKRMIIC      (2 << SCU_ETH0CON_CLKRMII_SHIFT) /* Data input RMIIC is selected */
#  define SCU_ETH0CON_CLKRMIID      (3 << SCU_ETH0CON_CLKRMII_SHIFT) /* Data input RMIID is selected */
#define SCU_ETH0CON_CRSDV_SHIFT     (10)      /* Bits 10-11: CRS_DV input */
#define SCU_ETH0CON_CRSDV_MASK      (3 << SCU_ETH0CON_CRSDV_SHIFT)
#  define SCU_ETH0CON_CRSDVA        (0 << SCU_ETH0CON_CRSDV_SHIFT) /* Data input CRS_DVA is selected */
#  define SCU_ETH0CON_CRSDVB        (1 << SCU_ETH0CON_CRSDV_SHIFT) /* Data input CRS_DVB is selected */
#  define SCU_ETH0CON_CRSDVC        (2 << SCU_ETH0CON_CRSDV_SHIFT) /* Data input CRS_DVC is selected */
#  define SCU_ETH0CON_CRSDVD        (3 << SCU_ETH0CON_CRSDV_SHIFT) /* Data input CRS_DVD is selected */
#define SCU_ETH0CON_CRS_SHIFT       (12)      /* Bits 12-13: CRS input */
#define SCU_ETH0CON_CRS_MASK        (3 << SCU_ETH0CON_CRS_SHIFT)
#  define SCU_ETH0CON_CRSA          (0 << SCU_ETH0CON_CRS_SHIFT) /* Data input CRSA is selected */
#  define SCU_ETH0CON_CRSB          (1 << SCU_ETH0CON_CRS_SHIFT) /* Data input CRSB is selected */
#  define SCU_ETH0CON_CRSC          (2 << SCU_ETH0CON_CRS_SHIFT) /* Data input CRSC is selected */
#  define SCU_ETH0CON_CRSD          (3 << SCU_ETH0CON_CRS_SHIFT) /* Data input CRSD is selected */
#define SCU_ETH0CON_RXER_SHIFT      (14)      /* Bits 14-15: RXER Input */
#define SCU_ETH0CON_RXER_MASK       (3 << SCU_ETH0CON_RXER_SHIFT)
#  define SCU_ETH0CON_RXERA         (0 << SCU_ETH0CON_RXER_SHIFT) /* Data input RXERA is selected */
#  define SCU_ETH0CON_RXERB         (1 << SCU_ETH0CON_RXER_SHIFT) /* Data input RXERB is selected */
#  define SCU_ETH0CON_RXERC         (2 << SCU_ETH0CON_RXER_SHIFT) /* Data input RXERC is selected */
#  define SCU_ETH0CON_RXERD         (3 << SCU_ETH0CON_RXER_SHIFT) /* Data input RXERD is selected */
#define SCU_ETH0CON_COL_SHIFT       (16)      /* Bits 16-17: COL input */
#define SCU_ETH0CON_COL_MASK        (3 << SCU_ETH0CON_COL_SHIFT)
#  define SCU_ETH0CON_COLA          (0 << SCU_ETH0CON_COL_SHIFT) /* Data input COLA is selected */
#  define SCU_ETH0CON_COLB          (1 << SCU_ETH0CON_COL_SHIFT) /* Data input COLB is selected */
#  define SCU_ETH0CON_COLC          (2 << SCU_ETH0CON_COL_SHIFT) /* Data input COLC is selected */
#  define SCU_ETH0CON_COLD          (3 << SCU_ETH0CON_COL_SHIFT) /* Data input COLD is selected */
#define SCU_ETH0CON_CLKTX_SHIFT     (18)      /* Bits 18-19: CLK_TX input */
#define SCU_ETH0CON_CLKTX_MASK      (3 << SCU_ETH0CON_CLKTX_SHIFT)
#  define SCU_ETH0CON_CLKTXA        (0 << SCU_ETH0CON_CLKTX_SHIFT) /* Data input CLK_TXA is selected */
#  define SCU_ETH0CON_CLKTXB        (1 << SCU_ETH0CON_CLKTX_SHIFT) /* Data input CLK_TXB is selected */
#  define SCU_ETH0CON_CLKTXC        (2 << SCU_ETH0CON_CLKTX_SHIFT) /* Data input CLK_TXC is selected */
#  define SCU_ETH0CON_CLKTXD        (3 << SCU_ETH0CON_CLKTX_SHIFT) /* Data input CLK_TXD is selected */
#define SCU_ETH0CON_MDIO_SHIFT      (22)      /* Bits 22-23: MDIO Input Select */
#define SCU_ETH0CON_MDIO_MASK       (3 << SCU_ETH0CON_MDIO_SHIFT)
#  define SCU_ETH0CON_MDIOA         (0 << SCU_ETH0CON_MDIO_SHIFT) /* Data input MDIOA is selected */
#  define SCU_ETH0CON_MDIOB         (1 << SCU_ETH0CON_MDIO_SHIFT) /* Data input MDIOB is selected */
#  define SCU_ETH0CON_MDIOC         (2 << SCU_ETH0CON_MDIO_SHIFT) /* Data input MDIOC is selected */
#  define SCU_ETH0CON_MDIOD         (3 << SCU_ETH0CON_MDIO_SHIFT) /* Data input MDIOD is selected */
#define SCU_ETH0CON_INFSEL          (1 << 26) /* Bit 26: Ethernet MAC Interface Selection */
#  define SCU_ETH0CON_INFSEL_MII    (0)       /*         0=MII */
#  define SCU_ETH0CON_INFSEL_RMII   (1 << 26) /*         1=RMII */

/* CCUx Global Start Control Register */

#define SCU_CCUCON_GSC40            (1 << 0)  /* Bit 0:  Global Start Control CCU40 */
#define SCU_CCUCON_GSC41            (1 << 1)  /* Bit 1:  Global Start Control CCU41 */
#define SCU_CCUCON_GSC42            (1 << 2)  /* Bit 2:  Global Start Control CCU42 */
#define SCU_CCUCON_GSC43            (1 << 3)  /* Bit 3:  Global Start Control CCU43 */
#define SCU_CCUCON_GSC80            (1 << 8)  /* Bit 8:  Global Start Control CCU80 */
#define SCU_CCUCON_GSC81            (1 << 9)  /* Bit 9:  Global Start Control CCU81 */

/* DTS Control */

#define SCU_DTSCON_PWD              (1 << 0)  /* Bit 0:  Sensor Power Down */
#define SCU_DTSCON_START            (1 << 1)  /* Bit 1:  Sensor Measurement Start */
#define SCU_DTSCON_OFFSET_SHIFT     (4)       /* Bits 4-10: Offset Calibration Value */
#define SCU_DTSCON_OFFSET_MASK      (0x7f << SCU_DTSCON_OFFSET_SHIFT)
#  define SCU_DTSCON_OFFSET(n)      ((uint32_t)(n) << SCU_DTSCON_OFFSET_SHIFT)
#define SCU_DTSCON_GAIN_SHIFT       (11)      /* Bits 11-16: Gain Calibration Value */
#define SCU_DTSCON_GAIN_MASK        (0x3f << SCU_DTSCON_GAIN_SHIFT)
#  define SCU_DTSCON_GAIN(n)        ((uint32_t)(n) << SCU_DTSCON_GAIN_SHIFT)
#define SCU_DTSCON_REFTRIM_SHIFT    (17)      /* Bits 17-19: Reference Trim Calibration Value */
#define SCU_DTSCON_REFTRIM_MASK     (7 << SCU_DTSCON_REFTRIM_SHIFT)
#  define SCU_DTSCON_REFTRIM(n)     ((uint32_t)(n) << SCU_DTSCON_REFTRIM_SHIFT)
#define SCU_DTSCON_BGTRIM_SHIFT     (20)      /* Bits 20-23: Bandgap Trim Calibration Value */
#define SCU_DTSCON_BGTRIM_MASK      (15 << SCU_DTSCON_BGTRIM_SHIFT)
#  define SCU_DTSCON_BGTRIM(n)      ((uint32_t)(n) << SCU_DTSCON_BGTRIM_SHIFT)

/* DTS Status */

#define SCU_DTSSTAT_RESULT_SHIFT    (0)       /* Bits 0-9: Result of the DTS Measurement */
#define SCU_DTSSTAT_RESULT_MASK     (0x3ff << SCU_DTSSTAT_RESULT_SHIFT)
#define SCU_DTSSTAT_RDY             (1 << 14) /* Bit 14: Sensor Ready Status */
#define SCU_DTSSTAT_BUSY            (1 << 15) /* Bit 15: Sensor Busy Status */

/* SD-MMC Delay Control Register */

#define SCU_SDMMCDEL_TAPEN          (1 << 0)  /* Bit 0: Enable delay on the CMD/DAT out lines */
#define SCU_SDMMCDEL_TAPDEL_SHIFT   (4)       /* Bitx 4-7: Number of Delay Elements Select */
#define SCU_SDMMCDEL_TAPDEL_MASK    (15 << SCU_SDMMCDEL_TAPDEL_SHIFT)
#  define SCU_SDMMCDEL_TAPDEL(n)    ((uint32_t)((n)-1) << SCU_SDMMCDEL_TAPDEL_SHIFT)

/* Out-Of-Range Comparator Enable Register 0 and Out-Of-Range Comparator Enable Register 1 */

#define SCU_GORCEN_ENORC6           (1 << 6)  /* Bit 6:  Enable Out of Range Comparator, Channel 6 */
#define SCU_GORCEN_ENORC7           (1 << 7)  /* Bit 7:  Enable Out of Range Comparator, Channel 7 */

/* SDMMC Configuration */

#define SCU_SDMMCCON_WPSEL          (1 << 0)  /* Bit 0:  SDMMC Write Protection Input Multiplexer Control */
#define SCU_SDMMCCON_WPSVAL         (1 << 4)  /* Bit 4:  SDMMC Write Protect Software Control */
#define SCU_SDMMCCON_CDSEL          (1 << 16) /* Bit 16: SDMMC Card Detection Control */
#define SCU_SDMMCCON_CDSVAL         (1 << 20) /* Bit 20: SDMMC Write Protect Software Control */

/* Mirror Update Status Register */

#define SCU_MIRRSTS_HDCLR           (1 << 1)  /* Bit 1:  HDCLR Mirror Register Write Status */
#define SCU_MIRRSTS_HDSET           (1 << 2)  /* Bit 2:  HDSET Mirror Register Write Status */
#define SCU_MIRRSTS_HDCR            (1 << 3)  /* Bit 3:  HDCR Mirror Register Write Status */
#define SCU_MIRRSTS_OSCSICTRL       (1 << 5)  /* Bit 5:  OSCSICTRL Mirror Register Write Status */
#define SCU_MIRRSTS_OSCULSTAT       (1 << 6)  /* Bit 6:  OSCULSTAT Mirror Register Write Status */
#define SCU_MIRRSTS_OSCULCTRL       (1 << 7)  /* Bit 7:  OSCULCTRL Mirror Register Write Status */
#define SCU_MIRRSTS_RTC_CTR         (1 << 8)  /* Bit 8:  RTC CTR Mirror Register Write Status */
#define SCU_MIRRSTS_RTC_ATIM0       (1 << 9)  /* Bit 9:  RTC ATIM0 Mirror Register Write Status */
#define SCU_MIRRSTS_RTC_ATIM1       (1 << 10) /* Bit 10: RTC ATIM1 Mirror Register Write Status */
#define SCU_MIRRSTS_RTC_TIM0        (1 << 11) /* Bit 11: RTC TIM0 Mirror Register Write Status */
#define SCU_MIRRSTS_RTC_TIM1        (1 << 12) /* Bit 12: RTC TIM1 Mirror Register Write Status */
#define SCU_MIRRSTS_RMX             (1 << 13) /* Bit 13: Retention Memory Access Register Update Status */
#define SCU_MIRRSTS_RTC_MSKSR       (1 << 14) /* Bit 14: RTC MSKSSR Mirror Register Write Status */
#define SCU_MIRRSTS_RTC_CLRSR       (1 << 15) /* Bit 15: RTC CLRSR Mirror Register Write Status */

/* Interrupt Control SCU Registers */

/* Service Request Status, RAW Service Request Status, Service Request Mask, Service
 * Request Clear, Service Request Set
 */

#define SCU_INT_PRWARN              (1 << 0)  /* Bit 0:  WDT pre-warning Interrupt */
#define SCU_INT_PI                  (1 << 1)  /* Bit 1:  RTC Periodic Interrupt */
#define SCU_INT_AI                  (1 << 2)  /* Bit 2:  Alarm Interrupt */
#define SCU_INT_DLROVR              (1 << 3)  /* Bit 3:  DLR Request Overrun Interrupt */
#define SCU_INT_HDSTAT              (1 << 16) /* Bit 16: HDSTAT Mirror Register Update */
#define SCU_INT_HDCLR               (1 << 17) /* Bit 17: HDCLR Mirror Register Update */
#define SCU_INT_HDSET               (1 << 18) /* Bit 18: HDSET Mirror Register Update */
#define SCU_INT_HDCR                (1 << 19) /* Bit 19: HDCR Mirror Register Update */
#define SCU_INT_OSCSICTRL           (1 << 21) /* Bit 21: OSCSICTRL Mirror Register Update */
#define SCU_INT_OSCULSTAT           (1 << 22) /* Bit 22: OSCULTAT Mirror Register Update */
#define SCU_INT_OSCULCTRL           (1 << 23) /* Bit 23: OSCULCTRL Mirror Register Update */
#define SCU_INT_RTC_CTR             (1 << 24) /* Bit 24: RTC CTR Mirror Register Update */
#define SCU_INT_RTC_ATIM0           (1 << 25) /* Bit 25: RTC ATIM0 Mirror Register Update */
#define SCU_INT_RTC_ATIM1           (1 << 26) /* Bit 26: RTC ATIM1 Mirror Register Update */
#define SCU_INT_RTC_TIM0            (1 << 27) /* Bit 27: RTC TIM0 Mirror Register Update */
#define SCU_INT_RTC_TIM1            (1 << 28) /* Bit 28: RTC TIM1 Mirror Register Update */
#define SCU_INTT_RMX                (1 << 29) /* Bit 29: Retention Memory Mirror Register */

/* Enable Promoting Events to NMI Request */

#define SCU_NMIREQEN_PRWARN         (1 << 0)  /* Bit 0:  Promote Pre-Warning Interrupt Request to NMI Request */
#define SCU_NMIREQEN_PI             (1 << 1)  /* Bit 1:  Promote RTC Periodic Interrupt request to NMI Request */
#define SCU_NMIREQEN_AI             (1 << 2)  /* Bit 2:  Promote RTC Alarm Interrupt Request to NMIRequest */
#define SCU_NMIREQEN_ERU00          (1 << 16) /* Bit 16: Promote Channel 0 Interrupt of ERU0 Request to NMI Request */
#define SCU_NMIREQEN_ERU01          (1 << 17) /* Bit 17: Promote Channel 1 Interrupt of ERU0 Request to NMI Request */
#define SCU_NMIREQEN_ERU02          (1 << 18) /* Bit 18: Promote Channel 2 Interrupt of ERU0 Request to NMI Request */
#define SCU_NMIREQEN_ERU03          (1 << 19) /* Bit 19: Promote Channel 3 Interrupt of ERU0 Request to NMI Request */

/* Retention Memory Access Control Register */

#define SCU_RMACR_RDWR              (1 << 0)  /* Bit 0: Hibernate Retention Memory Register Update Control */
#define SCU_RMACR_ADDR_SHIFT        (16)      /* Bits 16-19: Hibernate Retention Memory Register Address Select */
#define SCU_RMACR_ADDR_MASK         (15 << SCU_RMACR_ADDR_SHIFT)
#  define SCU_RMACR_ADDR(n)         ((uint32_t)(n) << SCU_RMACR_ADDR_SHIFT)

/* Retention Memory Access Data Register (32-bit data) */

/* Parity Control Registers */

/* Parity Error Enable Register */

#define SCU_PEEN_PEENPS             (1 << 0)  /* Bit 0:  Parity Error Enable for PSRAM */
#define SCU_PEEN_PEENDS1            (1 << 1)  /* Bit 1:  Parity Error Enable for DSRAM1 */
#define SCU_PEEN_PEENDS2            (1 << 2)  /* Bit 2:  Parity Error Enable for DSRAM2 */
#define SCU_PEEN_PEENU0             (1 << 8)  /* Bit 8:  Parity Error Enable for USIC0 Memory */
#define SCU_PEEN_PEENU1             (1 << 9)  /* Bit 9:  Parity Error Enable for USIC1 Memory */
#define SCU_PEEN_PEENU2             (1 << 10) /* Bit 10: Parity Error Enable for USIC2 Memory */
#define SCU_PEEN_PEENMC             (1 << 12) /* Bit 12: Parity Error Enable for MultiCAN Memory */
#define SCU_PEEN_PEENPPRF           (1 << 13) /* Bit 13: Parity Error Enable for PMU Prefetch Memory */
#define SCU_PEEN_PEENUSB            (1 << 16) /* Bit 16: Parity Error Enable for USB Memory */
#define SCU_PEEN_PEENETH0TX         (1 << 17) /* Bit 17: Parity Error Enable for ETH TX Memory */
#define SCU_PEEN_PEENETH0RX         (1 << 18) /* Bit 18: Parity Error Enable for ETH RX Memory */
#define SCU_PEEN_PEENSD0            (1 << 19) /* Bit 19: Parity Error Enable for SDMMC Memory 0 */
#define SCU_PEEN_PEENSD1            (1 << 20) /* Bit 20: Parity Error Enable for SDMMC Memory 1 */

/* Memory Checking Control Register */

#define SCU_MCHKCON_SELPS           (1 << 0)  /* Bit 0:  Select Memory Check for PSRAM */
#define SCU_MCHKCON_SELDS1          (1 << 1)  /* Bit 1:  Select Memory Check for DSRAM1 */
#define SCU_MCHKCON_SELDS2          (1 << 2)  /* Bit 2:  Select Memory Check for DSRAM2 */
#define SCU_MCHKCON_USIC0DRA        (1 << 8)  /* Bit 8:  Select Memory Check for USIC0 */
#define SCU_MCHKCON_USIC1DRA        (1 << 9)  /* Bit 9:  Select Memory Check for USIC1 */
#define SCU_MCHKCON_USIC2DRA        (1 << 10) /* Bit 10: Select Memory Check for USIC2 */
#define SCU_MCHKCON_MCANDRA         (1 << 12) /* Bit 12: Select Memory Check for MultiCAN */
#define SCU_MCHKCON_PPRFDRA         (1 << 13) /* Bit 13: Select Memory Check for PMU */
#define SCU_MCHKCON_SELUSB          (1 << 16) /* Bit 16: Select Memory Check for USB SRAM */
#define SCU_MCHKCON_SELETH0TX       (1 << 17) /* Bit 17: Select Memory Check for ETH0 TX SRAM */
#define SCU_MCHKCON_SELETH0RX       (1 << 18) /* Bit 18: Select Memory Check for ETH0 RX SRAM */
#define SCU_MCHKCON_SELSD0          (1 << 19) /* Bit 19: Select Memory Check for SDMMC SRAM 0 */
#define SCU_MCHKCON_SELSD1          (1 << 20) /* Bit 20: Select Memory Check for SDMMC SRAM 1 */

/* Parity Error Trap Enable Register */

#define SCU_PETE_PETEPS             (1 << 0)  /* Bit 0:  Parity Error Trap Enable for PSRAM */
#define SCU_PETE_PETEDS1            (1 << 1)  /* Bit 1:  Parity Error Trap Enable for DSRAM1 */
#define SCU_PETE_PETEDS2            (1 << 2)  /* Bit 2:  Parity Error Trap Enable for DSRAM2 */
#define SCU_PETE_PETEU0             (1 << 8)  /* Bit 8:  Parity Error Trap Enable for USIC0 Memory */
#define SCU_PETE_PETEU1             (1 << 9)  /* Bit 9:  Parity Error Trap Enable for USIC1 Memory */
#define SCU_PETE_PETEU2             (1 << 10) /* Bit 10: Parity Error Trap Enable for USIC2 Memory */
#define SCU_PETE_PETEMC             (1 << 12) /* Bit 12: Parity Error Trap Enable for MultiCAN Memory */
#define SCU_PETE_PETEPPRF           (1 << 13) /* Bit 13: Parity Error Trap Enable for PMU Prefetch Memory */
#define SCU_PETE_PETEUSB            (1 << 16) /* Bit 16: Parity Error Trap Enable for USB Memory */
#define SCU_PETE_PETEETH0TX         (1 << 17) /* Bit 17: Parity Error Trap Enable for ETH0 TX Memory */
#define SCU_PETE_PETEETH0RX         (1 << 18) /* Bit 18: Parity Error Trap Enable for ETH0 RX Memory */
#define SCU_PETE_PETESD0            (1 << 19) /* Bit 19: Parity Error Trap Enable for SDMMC SRAM 0 Memory */
#define SCU_PETE_PETESD1            (1 << 20) /* Bit 20: Parity Error Trap Enable for SDMMC SRAM 1 Memory */

/* Reset upon Parity Error Enable Register */

#define SCU_PERSTEN_RSEN            (1 << 0)  /* Bit 0: System Reset Enable upon Parity Error Trap */

/* Parity Error Control Register */

#define SCU_PEFLAG_PEFPS            (1 << 0)  /* Bit 0:  Parity Error Flag for PSRAM */
#define SCU_PEFLAG_PEFDS1           (1 << 1)  /* Bit 1:  Parity Error Flag for DSRAM1 */
#define SCU_PEFLAG_PEFDS2           (1 << 2)  /* Bit 2:  Parity Error Flag for DSRAM2 */
#define SCU_PEFLAG_PEFU0            (1 << 8)  /* Bit 8:  Parity Error Flag for USIC0 Memory */
#define SCU_PEFLAG_PEFU1            (1 << 9)  /* Bit 9:  Parity Error Flag for USIC1 Memory */
#define SCU_PEFLAG_PEFU2            (1 << 10) /* Bit 10: Parity Error Flag for USIC2 Memory */
#define SCU_PEFLAG_PEFMC            (1 << 12) /* Bit 12: Parity Error Flag for MultiCAN Memory */
#define SCU_PEFLAG_PEFPPRF          (1 << 13) /* Bit 13: Parity Error Flag for PMU Prefetch Memory */
#define SCU_PEFLAG_PEUSB            (1 << 16) /* Bit 16: Parity Error Flag for USB Memory */
#define SCU_PEFLAG_PEETH0TX         (1 << 17) /* Bit 17: Parity Error Flag for ETH TX Memory */
#define SCU_PEFLAG_PEETH0RX         (1 << 18) /* Bit 18: Parity Error Flag for ETH RX Memory */
#define SCU_PEFLAG_PESD0            (1 << 19) /* Bit 19: Parity Error Flag for SDMMC Memory 0 */
#define SCU_PEFLAG_PESD1            (1 << 20) /* Bit 20: Parity Error Flag for SDMMC Memory 1 */

/* Parity Memory Test Pattern Register */

#define SCU_PMTPR_PWR_SHIFT         (0)       /* Bits 0-7: Parity Read Values for Memory Test */
#define SCU_PMTPR_PWR_MASK          (0xff << SCU_PMTPR_PWR_SHIFT)
#  define SCU_PMTPR_PWR(n)          ((uint32_t)(n) << SCU_PMTPR_PWR_SHIFT)
#define SCU_PMTPR_PRD_SHIFT         (8)       /* Bits 8-15: Parity Write Values for Memory Test */
#define SCU_PMTPR_PRD_MASK          (0xff << SCU_PMTPR_PRD_SHIFT)
#  define SCU_PMTPR_PRD(n)          ((uint32_t)(n) << SCU_PMTPR_PRD_SHIFT)

/* Parity Memory Test Select Register */

#define SCU_PMTSR_MTENPS            (1 << 0)  /* Bit 0:  Test Enable Control for PSRAM */
#define SCU_PMTSR_MTENDS1           (1 << 1)  /* Bit 1:  Test Enable Control for DSRAM1 */
#define SCU_PMTSR_MTENDS2           (1 << 2)  /* Bit 2:  Test Enable Control for DSRAM2 */
#define SCU_PMTSR_MTEU0             (1 << 8)  /* Bit 8:  Test Enable Control for USIC0 Memory */
#define SCU_PMTSR_MTEU1             (1 << 9)  /* Bit 9:  Test Enable Control for USIC1 Memory */
#define SCU_PMTSR_MTEU2             (1 << 10) /* Bit 10: Test Enable Control for USIC2 Memory */
#define SCU_PMTSR_MTEMC             (1 << 12) /* Bit 12: Test Enable Control for MultiCAN Memory */
#define SCU_PMTSR_MTEPPRF           (1 << 13) /* Bit 13: Test Enable Control for PMU Prefetch Memory */
#define SCU_PMTSR_MTUSB             (1 << 16) /* Bit 16: Test Enable Control for USB Memory */
#define SCU_PMTSR_MTETH0TX          (1 << 17) /* Bit 17: Test Enable Control for ETH TX Memory */
#define SCU_PMTSR_MTETH0RX          (1 << 18) /* Bit 18: Test Enable Control for ETH RX Memory */
#define SCU_PMTSR_MTSD0             (1 << 19) /* Bit 19: Test Enable Control for SDMMC Memory 0 */
#define SCU_PMTSR_MTSD1             (1 << 20) /* Bit 20: Test Enable Control for SDMMC Memory 1 */

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

#define SCU_EVRSTAT_OV13            (1 << 1)  /* Bit 1:  Regulator Overvoltage for 1.3 V */

/* EVR VADC Status Register */

#define SCU_EVRVADCSTAT_VADC13V_SHIFT (0)     /* Bits 0-7: VADC 1.3 V Conversion Result */
#define SCU_EVRVADCSTAT_VADC13V_MASK  (0xff << SCU_EVRVADCSTAT_VADC13V_SHIFT)
#define SCU_EVRVADCSTAT_VADC33V_SHIFT (8)     /* Bits 8-15: VADC 3.3 V Conversion Result */
#define SCU_EVRVADCSTAT_VADC33V_MASK  (0xff << SCU_EVRVADCSTAT_VADC33V_SHIFT)

/* Power Monitor Value */

#define SCU_PWRMON_THRS_SHIFT       (0)       /* Bits 0-7: Threshold */
#define SCU_PWRMON_THRS_MASK        (0xff << SCU_POWER_PWRMON_THRS_SHIFT)
#  define SCU_PWRMON_THRS(n)        ((uint32_t)(n) << SCU_POWER_PWRMON_THRS_SHIFT)
#define SCU_PWRMON_INTV_SHIFT       (8)       /* Bits 8-15: Interval */
#define SCU_PWRMON_INTV_MASK        (0xff << SCU_POWER_PWRMON_INTV_SHIFT)
#  define SCU_PWRMON_INTV(n)        ((uint32_t)(n) << SCU_POWER_PWRMON_INTV_SHIFT)
#define SCU_PWRMON_ENB              (1 << 16) /* Bit 16: Enable */

/* Hibernation SCU Registers */
/* Hibernate Domain Status Register */

#define SCU_HDSTAT_EPEV             (1 << 0)  /* Bit 0:  Wake-up Pin Event Positive Edge Status */
#define SCU_HDSTAT_ENEV             (1 << 1)  /* Bit 1:  Wake-up Pin Event Negative Edge Status */
#define SCU_HDSTAT_RTCEV            (1 << 2)  /* Bit 2:  RTC Event Status */
#define SCU_HDSTAT_ULPWDG           (1 << 3)  /* Bit 3:  ULP WDG Alarm Status */
#define SCU_HDSTAT_HIBNOUT          (1 << 4)  /* Bit 3:  Hibernate Control Status */

/* Hibernate Domain Status Clear Register */

#define SCU_HDCLR_EPEV              (1 << 0)  /* Bit 0:  Wake-up Pin Event Positive Edge Clear */
#define SCU_HDCLR_ENEV              (1 << 1)  /* Bit 1:  Wake-up Pin Event Negative Edge Clear */
#define SCU_HDCLR_RTCEV             (1 << 2)  /* Bit 2:  RTC Event Clear */
#define SCU_HDCLR_ULPWDG            (1 << 3)  /* Bit 3:  ULP WDG Alarm Clear */

/* Hibernate Domain Status Set Register */

#define SCU_HDSET_EPEV              (1 << 0)  /* Bit 0:  Wake-up Pin Event Positive Edge Set */
#define SCU_HDSET_ENEV              (1 << 1)  /* Bit 1:  Wake-up Pin Event Negative Edge Set */
#define SCU_HDSET_RTCEV             (1 << 2)  /* Bit 2:  RTC Event Set */
#define SCU_HDSET_ULPWDG            (1 << 3)  /* Bit 3:  ULP WDG Alarm Set */

/* Hibernate Domain Control Register */

#define SCU_HDCR_WKPEP              (1 << 0)  /* Bit 0:  Wake-Up on Pin Event Positive Edge Enable */
#define SCU_HDCR_WKPEN              (1 << 1)  /* Bit 1:  Wake-up on Pin Event Negative Edge Enable */
#define SCU_HDCR_RTCE               (1 << 2)  /* Bit 2:  Wake-up on RTC Event Enable */
#define SCU_HDCR_ULPWDGEN           (1 << 3)  /* Bit 3:  ULP WDG Alarm Enable */
#define SCU_HDCR_HIB                (1 << 4)  /* Bit 4:  Hibernate Request Value Set */
#define SCU_HDCR_RCS                (1 << 6)  /* Bit 6:  fRTC Clock Selection */
#  define SCU_HDCR_RCS_OSI          (0)       /*         0=fOSI */
#  define SCU_HDCR_RCS_ULP          (1 << 6)  /*         1=fULP */
#define SCU_HDCR_STDBYSEL           (1 << 7)  /* Bit 7:  fSTDBY Clock Selection */
#  define SCU_HDCR_STDBYSEL_OSI     (0)       /*         0=fOSI */
#  define SCU_HDCR_STDBYSEL_ULP     (1 << 7)  /*         1=fULP */
#define SCU_HDCR_WKUPSEL            (1 << 8)  /* Bit 8:  Wake-Up from Hibernate Trigger Input Select */
#  define SCU_HDCR_WKUPSEL_HIBIO1   (0)       /*         0=HIB_IO_1 pin selected */
#  define SCU_HDCR_WKUPSEL_HIBIO0   (1 << 8)  /*         1=HIB_IO_0 pin selected */
#define SCU_HDCR_GPI0SEL            (1 << 10) /* Bit 10: General Purpose Input 0 Selection */
#  define SCU_HDCR_GPIOSEL_HIBIO1   (0)       /*         0=HIB_IO_1 pin selected */
#  define SCU_HDCR_GPIOSEL_HIBIO0   (1 << 10) /*         1=HIB_IO_0 pin selected */
#define SCU_HDCR_HIBIO0POL          (1 << 12) /* Bit 12: HIBIO0 Polarity Set */
#  define SCU_HDCR_HIBIO0POL_DIR    (0)       /*         0=Direct */
#  define SCU_HDCR_HIBIO0POL_INV    (1 << 12) /*         1=Inverted */
#define SCU_HDCR_HIBIO1POL          (1 << 13) /* Bit 13: HIBIO1 Polarity Set */
#  define SCU_HDCR_HIBIO1POL_DIR    (0)       /*         0=Direct */
#  define SCU_HDCR_HIBIO1POL_INV    (1 << 13) /*         1=Inverted */
#define SCU_HDCR_HIBIO0SEL_SHIFT    (16)      /* Bits 16-19: HIB_IO_0 Pin I/O Control */
#define SCU_HDCR_HIBIO0SEL_MASK     (15 << SCU_HDCR_HIBIO0SEL_SHIFT)
#  define SCU_HDCR_HIBIO0SEL_DIR    (0 << SCU_HDCR_HIBIO0SEL_SHIFT)  /* Direct input */
#  define SCU_HDCR_HIBIO0SEL_DIRPD  (1 << SCU_HDCR_HIBIO0SEL_SHIFT)  /* Direct input, Input pull-down */
#  define SCU_HDCR_HIBIO0SEL_DIRPU  (2 << SCU_HDCR_HIBIO0SEL_SHIFT)  /* Direct input, Input pull-up */
#  define SCU_HDCR_HIBIO0SEL_PP     (8 << SCU_HDCR_HIBIO0SEL_SHIFT)  /*  Push-pull HIB Control output */
#  define SCU_HDCR_HIBIO0SEL_PPWDT  (9 << SCU_HDCR_HIBIO0SEL_SHIFT)  /* Push-pull WDT service output */
#  define SCU_HDCR_HIBIO0SEL_PPGPIO (10 << SCU_HDCR_HIBIO0SEL_SHIFT) /* Push-pull GPIO output */
#  define SCU_HDCR_HIBIO0SEL_OD     (12 << SCU_HDCR_HIBIO0SEL_SHIFT) /* Open-drain HIB Control output */
#  define SCU_HDCR_HIBIO0SEL_ODWDT  (13 << SCU_HDCR_HIBIO0SEL_SHIFT) /* Open-drain WDT service output */
#  define SCU_HDCR_HIBIO0SEL_ODGPIO (14 << SCU_HDCR_HIBIO0SEL_SHIFT) /* Open-drain GPIO output */
#define SCU_HDCR_HIBIO1SEL_SHIFT    (20)      /* Bits 20-23: HIB_IO_1 Pin I/O Control */
#define SCU_HDCR_HIBIO1SEL_MASK     (15 << SCU_HDCR_HIBIO1SEL_SHIFT)
#  define SCU_HDCR_HIBIO1SEL_DIR    (0 << SCU_HDCR_HIBIO1SEL_SHIFT)  /* Direct input */
#  define SCU_HDCR_HIBIO1SEL_DIRPD  (1 << SCU_HDCR_HIBIO1SEL_SHIFT)  /* Direct input, Input pull-down */
#  define SCU_HDCR_HIBIO1SEL_DIRPU  (2 << SCU_HDCR_HIBIO1SEL_SHIFT)  /* Direct input, Input pull-up */
#  define SCU_HDCR_HIBIO1SEL_PP     (8 << SCU_HDCR_HIBIO1SEL_SHIFT)  /*  Push-pull HIB Control output */
#  define SCU_HDCR_HIBIO1SEL_PPWDT  (9 << SCU_HDCR_HIBIO1SEL_SHIFT)  /* Push-pull WDT service output */
#  define SCU_HDCR_HIBIO1SEL_PPGPIO (10 << SCU_HDCR_HIBIO1SEL_SHIFT) /* Push-pull GPIO output */
#  define SCU_HDCR_HIBIO1SEL_OD     (12 << SCU_HDCR_HIBIO1SEL_SHIFT) /* Open-drain HIB Control output */
#  define SCU_HDCR_HIBIO1SEL_ODWDT  (13 << SCU_HDCR_HIBIO1SEL_SHIFT) /* Open-drain WDT service output */
#  define SCU_HDCR_HIBIO1SEL_ODGPIO (14 << SCU_HDCR_HIBIO1SEL_SHIFT) /* Open-drain GPIO output */

/* Internal 32.768 kHz Clock Source Control Register */

#define SCU_OSCSICTRL_PWD           (1 << 0)  /* Bit 0:  Turn OFF the fOSI Clock Source */

/* OSC_ULP Status Register */

#define SCU_OSCULSTAT_X1D           (1 << 0)  /* Bit 0: XTAL1 Data Value */

/* OSC_ULP Control Register */

#define SCU_OSCULCTRL_X1DEN         (1 << 0)  /* Bit 0: XTAL1 Data General Purpose Input Enable */
#define SCU_OSCULCTRL_MODE_SHIFT    (4)       /* Bits 4-5: Oscillator Mode */
#define SCU_OSCULCTRL_MODE_MASK     (3 << SCU_OSCULCTRL_MODE_SHIFT)
#  define SCU_OSCULCTRL_MODE_OPER   (0 << SCU_OSCULCTRL_MODE_SHIFT) /* OSC enabled in operation */
#  define SCU_OSCULCTRL_MODE_BYPASS (1 << SCU_OSCULCTRL_MODE_SHIFT) /* OSC enabled in bypass */
#  define SCU_OSCULCTRL_MODE_PDN    (2 << SCU_OSCULCTRL_MODE_SHIFT) /* OSC power down */
#  define SCU_OSCULCTRL_MODE_PDNGPI (3 << SCU_OSCULCTRL_MODE_SHIFT) /* OSC power down, GPI possible */

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

/* Peripheral Reset Status Register 0, Peripheral Reset Set Register 0, Peripheral
 * Reset Clear Register 0
 */

#define SCU_PR0_VADCRS              (1 << 0)  /* Bit 0:  VADC Reset */
#define SCU_PR0_DSDRS               (1 << 1)  /* Bit 1:  DSD Reset */
#define SCU_PR0_CCU40RS             (1 << 2)  /* Bit 2:  CCU40 Reset */
#define SCU_PR0_CCU41RS             (1 << 3)  /* Bit 3:  CCU41 Reset */
#define SCU_PR0_CCU42RS             (1 << 4)  /* Bit 4:  CCU42 Reset */
#define SCU_PR0_CCU80RS             (1 << 7)  /* Bit 7:  CCU80 Reset */
#define SCU_PR0_CCU81RS             (1 << 8)  /* Bit 8:  CCU81 Reset */
#define SCU_PR0_POSIF0RS            (1 << 9)  /* Bit 9:  POSIF0 Reset */
#define SCU_PR0_POSIF1RS            (1 << 10) /* Bit 10: POSIF1 Reset */
#define SCU_PR0_USIC0RS             (1 << 11) /* Bit 11: USIC0 Reset */
#define SCU_PR0_ERU1RS              (1 << 16) /* Bit 16: ERU1 Reset */

/* Peripheral Reset Status Register 1, Peripheral Reset Set Register 1, Peripheral
 * Reset Clear Register 1
 */

#define SCU_PR1_CCU43RS             (1 << 0)  /* Bit 0:  CCU43 Reset */
#define SCU_PR1_LEDTSCU0RS          (1 << 3)  /* Bit 3:  LEDTS Reset */
#define SCU_PR1_MCAN0RS             (1 << 4)  /* Bit 4:  MultiCAN Reset */
#define SCU_PR1_DACRS               (1 << 5)  /* Bit 5:  DAC Reset */
#define SCU_PR1_MMCIRS              (1 << 6)  /* Bit 6:  MMC Interface Reset */
#define SCU_PR1_USIC1RS             (1 << 7)  /* Bit 7:  USIC1 Reset */
#define SCU_PR1_USIC2RS             (1 << 8)  /* Bit 8:  USIC2 Reset */
#define SCU_PR1_PPORTSRS            (1 << 9)  /* Bit 9:  PORTS Reset */

/* Peripheral Reset Status Register 1, Peripheral Reset Set Register 1, Peripheral
 * Reset Clear Register 1
 */

#define SCU_PR2_WDTRS               (1 << 1)  /* Bit 1:  WDT Reset */
#define SCU_PR2_ETH0RS              (1 << 2)  /* Bit 2:  ETH0 Reset */
#define SCU_PR2_DMA0RS              (1 << 4)  /* Bit 4:  DMA0 Reset */
#define SCU_PR2_DMA1RS              (1 << 5)  /* Bit 5:  DMA1 Reset */
#define SCU_PR2_FCERS               (1 << 6)  /* Bit 6:  FCE Reset */
#define SCU_PR2_USBRS               (1 << 7)  /* Bit 7:  USB Reset */

/* Peripheral Reset Status Register 3, Peripheral Reset Set Register 3, Peripheral
 * Reset Clear Register 3
 */

#define SCU_PR3_EBURS               (1 << 2)  /* Bit 2:  EBU Reset */

/* Clock Control SCU Registers */

/* Clock Status Register, Clock Set Control Register, Clock clear Control Register */

#define SCU_CLK_USBC                (1 << 0)  /* Bit 0:  USB Clock */
#define SCU_CLK_MMCC                (1 << 1)  /* Bit 1:  MMC Clock */
#define SCU_CLK_ETH0C               (1 << 2)  /* Bit 2:  Ethernet Clock */
#define SCU_CLK_EBUC                (1 << 3)  /* Bit 3:  EBU Clock */
#define SCU_CLK_CCUC                (1 << 4)  /* Bit 4:  CCU Clock */
#define SCU_CLK_WDTC                (1 << 5)  /* Bit 5:  WDT Clock */

/* System Clock Control */

#define SCU_SYSCLKCR_SYSDIV_SHIFT   (0)  /* Bits 0-7: System Clock Division Value */
#define SCU_SYSCLKCR_SYSDIV_MASK    (0xff << SCU_SYSCLKCR_SYSDIV_SHIFT)
#  define SCU_SYSCLKCR_SYSDIV(n)    ((uint32_t)((n)-1) << SCU_SYSCLKCR_SYSDIV_SHIFT)

#define SCU_SYSCLKCR_SYSSEL         (1 << 16) /* Bit 16: System Clock Selection Value */
#  define SCU_SYSCLKCR_SYSSEL_OFI   (0)       /*         0=OFI clock */
#  define SCU_SYSCLKCR_SYSSEL_PLL   (1 << 16) /*         1=PLL clock */

/* CPU Clock Control */

#define SCU_CPUCLKCR_CPUDIV         (1 << 0)  /* Bit 0: CPU Clock Divider Enable */

/* Peripheral Bus Clock Control */

#define SCU_PBCLKCR_PBDIV           (1 << 0)  /* Bit 0:  PB Clock Divider Enable */
#  define SCU_PBCLKCR_PBDIV_FCPU    (0)       /*         0=fCPU */
#  define SCU_PBCLKCR_PBDIV_DIV2    (1 << 0)  /*         1=fCPU/2 */

/* USB Clock Control */

#define SCU_USBCLKCR_USBDIV_SHIFT   (0)       /* Bits 0-2: USB Clock Divider Value */
#define SCU_USBCLKCR_USBDIV_MASK    (7 << SCU_USBCLKCR_USBDIV_SHIFT)
#  define SCU_SYSCLKCR_USBDIV(n)    ((uint32_t)((n)-1) << SCU_USBCLKCR_USBDIV_SHIFT)
#define SCU_USBCLKCR_USBSEL         (1 << 16) /* Bit 16: USB Clock Selection Value */
#  define SCU_USBCLKCR_USBSEL_USBPLL (0)      /*         0=USB PLL Clock */
#  define SCU_USBCLKCR_USBSEL_PLL   (1 << 16) /*         1= PLL Clock */

/* EBU Clock Control */

#define SCU_EBUCLKCR_EBUDIV_SHIFT   (0)       /* Bitx 0-5: EBU Clock Divider Value */
#define SCU_EBUCLKCR_EBUDIV_MASK    (0x3f << SCU_EBUCLKCR_EBUDIV_SHIFT)
#  define SCU_EBUCLKCR_EBUDIV(n)    ((uint32_t)((n)-1) << SCU_EBUCLKCR_EBUDIV_SHIFT)

/* CCU Clock Control */

#define SCU_CCUCLKCR_CCUDIV         (1 << 0)  /* Bit 0:  CCU Clock Divider Enable */
#  define SCU_CCUCLKCR_CCUDIV_FSYS  (0)       /*         0= SYS */
#  define SCU_CCUCLKCR_CCUDIV_DIV2  (1 << 0)  /*         1=fSYS/2 */

/* WDT Clock Control */

#define SCU_WDTCLKCR_WDTDIV_SHIFT   (0)       /* Bits 0-7: WDT Clock Divider Value */
#define SCU_WDTCLKCR_WDTDIV_MASK    (0xff << SCU_WDTCLKCR_WDTDIV_SHIFT)
#  define SCU_WDTCLKCR_WDTDIV(n)    ((uint32_t)((n)-1) << SCU_WDTCLKCR_WDTDIV_SHIFT)
#define SCU_WDTCLKCR_WDTSEL_SHIFT   (16)      /* Bits 16-17: WDT Clock Selection Value */
#define SCU_WDTCLKCR_WDTSEL_MASK    (3 << SCU_WDTCLKCR_WDTSEL_SHIFT)
#  define SCU_WDTCLKCR_WDTSEL_FOFI  (0 << SCU_WDTCLKCR_WDTSEL_SHIFT) /* fOFI clock */
#  define SCU_WDTCLKCR_WDTSEL_FSTDY (1 << SCU_WDTCLKCR_WDTSEL_SHIFT) /* fSTDBY clock */
#  define SCU_WDTCLKCR_WDTSEL_FPLL  (2 << SCU_WDTCLKCR_WDTSEL_SHIFT) /* fPLL clock */

/* External clock Control Register */

#define SCU_EXTCLKCR_ECKSEL_SHIFT   (0)       /* Bits 0-1: External Clock Selection Value */
#define SCU_EXTCLKCR_ECKSEL_MASK    (3 << SCU_EXTCLKCR_ECKSEL_SHIFT)
#  define SCU_EXTCLKCR_ECKSEL_FSYS  (0 << SCU_EXTCLKCR_ECKSEL_SHIFT) /* fSYS clock */
#  define SCU_EXTCLKCR_ECKSEL_FUSB  (2 << SCU_EXTCLKCR_ECKSEL_SHIFT) /* fUSB clock divided by ECKDIV */
#  define SCU_EXTCLKCR_ECKSEL_FPLL  (3 << SCU_EXTCLKCR_ECKSEL_SHIFT) /* fPLL clock divided by ECKDIV */
#define SCU_EXTCLKCR_ECKDIV_SHIFT   (16)      /* Bits 16-24: External Clock Divider Value */
#define SCU_EXTCLKCR_ECKDIV_MASK    (0x1ff << SCU_EXTCLKCR_ECKDIV_SHIFT)
#  define SCU_EXTCLKCR_ECKDIV(n)    ((uint32_t)((n)-1) << SCU_EXTCLKCR_ECKDIV_SHIFT)

/* Sleep Control Register */

#define SCU_SLEEPCR_SYSSEL          (1 << 0)  /* Bit 0:  System Clock Selection Value */
#  define SCU_SLEEPCR_SYSSEL_OFI    (0)       /*         0=fOFI */
#  define SCU_SLEEPCR_SYSSEL_FPLL   (1 << 0)  /*         1=fPLL */
#define SCU_SLEEPCR_USBCR           (1 << 16) /* Bit 6:  USB Clock Control in Sleep Mode */
#define SCU_SLEEPCR_MMCCR           (1 << 17) /* Bit 17: MMC Clock Control in Sleep Mode */
#define SCU_SLEEPCR_ETH0CR          (1 << 18) /* Bit 18: Ethernet Clock Control in Sleep Mode */
#define SCU_SLEEPCR_EBUCR           (1 << 19) /* Bit 19: EBU Clock Control in Sleep Mode */
#define SCU_SLEEPCR_CCUCR           (1 << 20) /* Bit 20: CCU Clock Control in Sleep Mode */
#define SCU_SLEEPCR_WDTCR           (1 << 21) /* Bit 21: WDT Clock Control in Sleep Mode */

/* Deep Sleep Control Register */

#define SCU_DSLEEPCR_SYSSEL         (1 << 0)  /* Bit 0: System Clock Selection Value */
#  define SCU_DSLEEPCR_SYSSEL_FOFI  (0)       /*        0=fOFI */
#  define SCU_DSLEEPCR_SYSSEL_FPLL  (1 << 0)  /*        1=fPLL */
#define SCU_DSLEEPCR_FPDN           (1 << 11) /* Bit 11: Flash Power Down */
#define SCU_DSLEEPCR_PLLPDN         (1 << 12) /* Bit 12: PLL Power Down */
#define SCU_DSLEEPCR_VCOPDN         (1 << 13) /* Bit 13: PLL Power Down */
#define SCU_DSLEEPCR_USBCR          (1 << 16) /* Bit 16: USB Clock Control in Deep Sleep Mode */
#define SCU_DSLEEPCR_MMCCR          (1 << 17) /* Bit 17: MMC Clock Control in Deep Sleep Mode */
#define SCU_DSLEEPCR_ETH0CR         (1 << 18) /* Bit 18: Ethernet Clock Control in Deep Sleep Mode */
#define SCU_DSLEEPCR_EBUCR          (1 << 19) /* Bit 19: EBU Clock Control in Deep Sleep Mode */
#define SCU_DSLEEPCR_CCUCR          (1 << 20) /* Bit 20: CCU Clock Control in Deep Sleep Mod */
#define SCU_DSLEEPCR_WDTCR          (1 << 21) /* Bit 21: WDT Clock Control in Deep Sleep Mode */

/* Peripheral 0 Clock Gating Status, Peripheral 0 Clock Gating Set, Peripheral 0 Clock Gating Clear */

#ifdef XMC4_SCU_GATING
#  define SCU_CGAT0_VADC            (1 << 0)  /* Bit 0:  VADC Gating Status */
#  define SCU_CGAT0_DSD             (1 << 1)  /* Bit 1:  DSD Gating Status */
#  define SCU_CGAT0_CCU40           (1 << 2)  /* Bit 2:  CCU40 Gating Status */
#  define SCU_CGAT0_CCU41           (1 << 3)  /* Bit 3:  CCU41 Gating Status */
#  define SCU_CGAT0_CCU42           (1 << 4)  /* Bit 4:  CCU42 Gating Status */
#  define SCU_CGAT0_CCU80           (1 << 7)  /* Bit 7:  CCU80 Gating Status */
#  define SCU_CGAT0_CCU81           (1 << 8)  /* Bit 8:  CCU81 Gating Status */
#  define SCU_CGAT0_POSIF0          (1 << 9)  /* Bit 9:  POSIF0 Gating Status */
#  define SCU_CGAT0_POSIF1          (1 << 10) /* Bit 10: POSIF1 Gating Status */
#  define SCU_CGAT0_USIC0           (1 << 11) /* Bit 11: USIC0 Gating Status */
#  define SCU_CGAT0_ERU1            (1 << 16) /* Bit 16: ERU1 Gating Status */
#endif

/* Peripheral 1 Clock Gating Status, Peripheral 1 Clock Gating Set, Peripheral 1 Clock Gating Clear */

#ifdef XMC4_SCU_GATING
#  define SCU_CGAT1_CCU43           (1 << 0)  /* Bit 0:  CCU43 Gating Status */
#  define SCU_CGAT1_LEDTSCU0        (1 << 3)  /* Bit 3:  LEDTS Gating Status */
#  define SCU_CGAT1_MCAN0           (1 << 4)  /* Bit 4:  MultiCAN Gating Status */
#  define SCU_CGAT1_DAC             (1 << 5)  /* Bit 5:  DAC Gating Status */
#  define SCU_CGAT1_MMCI            (1 << 6)  /* Bit 6:  MMC Interface Gating Status */
#  define SCU_CGAT1_USIC1           (1 << 7)  /* Bit 7:  USIC1 Gating Status */
#  define SCU_CGAT1_USIC2           (1 << 8)  /* Bit 8:  USIC1 Gating Status */
#  define SCU_CGAT1_PPORTS          (1 << 9)  /* Bit 9:  PORTS Gating Status */
#endif

/* Peripheral 2 Clock Gating Status, Peripheral 2 Clock Gating Set, Peripheral 2 Clock Gating Clear */

#ifdef XMC4_SCU_GATING
#  define SCU_CGAT2_WDT             (1 << 1)  /* Bit 1:  WDT Gating Status */
#  define SCU_CGAT2_ETH0            (1 << 2)  /* Bit 2:  ETH0 Gating Status */
#  define SCU_CGAT2_DMA0            (1 << 4)  /* Bit 4:  DMA0 Gating Status */
#  define SCU_CGAT2_DMA1            (1 << 5)  /* Bit 5:  DMA1 Gating Status */
#  define SCU_CGAT2_FCE             (1 << 6)  /* Bit 6:  FCE Gating Status */
#  define SCU_CGAT2_USB             (1 << 7)  /* Bit 7:  USB Gating Status */
#  define SCU_CGAT2_ECAT            (1 << 10) /* Bit 10: ECAT Gating Status */
#endif

/* Peripheral 3 Clock Gating Status, Peripheral 3 Clock Gating Set, Peripheral 3 Clock Gating Clear */

#ifdef XMC4_SCU_GATING
#  define SCU_CGAT3_EBU             (1 << 2)  /* Bit 2:  EBU Gating Status */
#endif

/* Oscillator Control SCU Registers */

/* OSC_HP Status Register */

#define SCU_OSCHPSTAT_X1D           (1 << 0)  /* Bit 0:  XTAL1 Data Value */

/* OSC_HP Control Register */

#define SCU_OSCHPCTRL_X1DEN         (1 << 0)  /* Bit 0:  XTAL1 Data Enable */
#define SCU_OSCHPCTRL_SHBY          (1 << 1)  /* Bit 1:  Shaper Bypass */
#define SCU_OSCHPCTRL_GAINSEL_SHIFT (2)       /* Bits 2-3: */
#define SCU_OSCHPCTRL_GAINSEL_MASK  (3 << SCU_OSCHPCTRL_GAINSEL_SHIFT)
#  define SCU_OSCHPCTRL_GAINSEL(n)  ((uint32_t)(n) << SCU_OSCHPCTRL_GAINSEL_SHIFT)
#define SCU_OSCHPCTRL_MODE_SHIFT    (4)
#define SCU_OSCHPCTRL_MODE_MASK     (3 << SCU_OSCHPCTRL_MODE_SHIFT)
#  define SCU_OSCHPCTRL_MODE_XTAL   (0 << SCU_OSCHPCTRL_MODE_SHIFT) /* External Crystal Mode */
#  define SCU_OSCHPCTRL_MODE_DIS    (1 << SCU_OSCHPCTRL_MODE_SHIFT) /* OSC is disabled */
#  define SCU_OSCHPCTRL_MODE_EXTIN  (2 << SCU_OSCHPCTRL_MODE_SHIFT) /*  External Input Clock Mode */
#  define SCU_OSCHPCTRL_MODE_DISPSM (3 << SCU_OSCHPCTRL_MODE_SHIFT) /* OSC is disabled, Power-Saving Mode */
#define SCU_OSCHPCTRL_OSCVAL_SHIFT  (16)
#define SCU_OSCHPCTRL_OSCVAL_MASK   (15 << SCU_OSCHPCTRL_OSCVAL_SHIFT)
#  define SCU_OSCHPCTRL_OSCVAL(n)   ((uint32_t)((n)-1) << SCU_OSCHPCTRL_OSCVAL_SHIFT)

/* Clock Calibration Constant Register */

#define SCU_CLKCALCONST_CALIBCONST_SHIFT (0)  /* Bits 0-3: Clock Calibration Constant Value */
#define SCU_CLKCALCONST_CALIBCONST_MASK  (15 << SCU_CLKCALCONST_CALIBCONST_SHIFT)
#  define SCU_CLKCALCONST_CALIBCONST(n)  ((uint32_t)(n) << SCU_CLKCALCONST_CALIBCONST_SHIFT)

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

#define SCU_USBPLLSTAT_VCOBYST      (1 << 0)  /* Bit 0:  VCO Bypass Status */
#define SCU_USBPLLSTAT_PWDSTAT      (1 << 1)  /* Bit 1:  PLL Power-saving Mode Status */
#define SCU_USBPLLSTAT_VCOLOCK      (1 << 2)  /* Bit 2:  PLL VCO Lock Status */
#define SCU_USBPLLSTAT_BY           (1 << 6)  /* Bit 6:  Bypass Mode Status */
#define SCU_USBPLLSTAT_VCOLOCKED    (1 << 7)  /* Bit 7:  PLL LOCK Status */

/* USB PLL Control Register */

#define SCU_USBPLLCON_VCOBYP        (1 << 0)  /* Bit 0:  VCO Bypass */
#define SCU_USBPLLCON_VCOPWD        (1 << 1)  /* Bit 1:  VCO Power Saving Mode */
#define SCU_USBPLLCON_VCOTR         (1 << 2)  /* Bit 2:  VCO Trim Control */
#define SCU_USBPLLCON_FINDIS        (1 << 4)  /* Bit 4:  Disconnect Oscillator from VCO */
#define SCU_USBPLLCON_OSCDISCDIS    (1 << 6)  /* Bit 6:  Oscillator Disconnect Disable */
#define SCU_USBPLLCON_NDIV_SHIFT    (8)       /* Bits 8-14: N-Divider Val */
#define SCU_USBPLLCON_NDIV_MASK     (0x7f << SCU_USBPLLCON_NDIV_SHIFT)
#  define SCU_USBPLLCON_NDIV(n)     ((uint32_t)((n)-1) << SCU_USBPLLCON_NDIV_SHIFT)
#define SCU_USBPLLCON_PLLPWD        (1 << 16) /* Bit 16: PLL Power Saving Mode */
#define SCU_USBPLLCON_RESLD         (1 << 18) /* Bit 18: Restart VCO Lock Detection */
#define SCU_USBPLLCON_PDIV_SHIFT    (24)      /* Bits 24-27: P-Divider Value */
#define SCU_USBPLLCON_PDIV_MASK     (15 << SCU_USBPLLCON_PDIV_SHIFT)
#  define SCU_USBPLLCON_PDIV(n)     ((uint32_t)((n)-1) << SCU_USBPLLCON_PDIV_SHIFT)

/* Clock Multiplexing Status Register */

#define SCU_CLKMXSTAT_SYSCLKMUX_SHIFT (0)     /* Bits 0-1: System Clock Multiplexing Status */
#define SCU_CLKMXSTAT_SYSCLKMUX_MASK  (3 << SCU_CLKMXSTAT_SYSCLKMUX_SHIFT)
#  define SCU_CLKMXSTAT_SYSCLKMUX_OFI (1 << SCU_CLKMXSTAT_SYSCLKMUX_SHIFT)
#  define SCU_CLKMXSTAT_SYSCLKMUX_PLL (2 << SCU_CLKMXSTAT_SYSCLKMUX_SHIFT)

#endif /* __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_SCU_H */
