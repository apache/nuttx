/************************************************************************************
 * arch/arm/src/xmc4/chip/xmc4_scu.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#define XMC4_GCU_OFFSET             0x0000    /* Offset address of General Control Unit */
#define XMC4_PCU_OFFSET             0x0200    /* Offset address of Power Control Unit */
#define XMC4_HCU_OFFSET             0x0300    /* Offset address of Hibernate Control Unit */
#define XMC4_RCU_OFFSET             0x0400    /* Offset address of Reset Control Unit */
#define XMC4_CCU_OFFSET             0x0600    /* Offset address of Clock Control Unit */

/* General SCU Registers */

#define XMC4_GCU_ID_OFFSET          0x0000    /* Module Identification Register */
#define XMC4_GCU_IDCHIP_OFFSET      0x0004    /* Chip ID */
#define XMC4_GCU_IDMANUF_OFFSET     0x0008    /* Manufactory ID */
#define XMC4_GCU_STCON_OFFSET       0x0010    /* Start-up Control */
#define XMC4_GCU_GPR0_OFFSET        0x002c    /* General Purpose Register 0 */
#define XMC4_GCU_GPR1_OFFSET        0x0030    /* General Purpose Register 1 */
#define XMC4_GCU_ETH0CON_OFFSET     0x0040    /* Ethernet 0 Port Control */
#define XMC4_GCU_CCUCON_OFFSET      0x004c    /* CCUx Global Start Control Register */
#define XMC4_GCU_SRSTAT_OFFSET      0x0074    /* Service Request Status */
#define XMC4_GCU_SRRAW_OFFSET       0x0078    /* RAW Service Request Status */
#define XMC4_GCU_SRMSK_OFFSET       0x007c    /* Service Request Mask */
#define XMC4_GCU_SRCLR_OFFSET       0x0080    /* Service Request Clear */
#define XMC4_GCU_SRSET_OFFSET       0x0084    /* Service Request Set */
#define XMC4_GCU_NMIREQEN_OFFSET    0x0088    /* Enable Promoting Events to NMI Request */
#define XMC4_GCU_DTSCON_OFFSET      0x008c    /* DTS Control */
#define XMC4_GCU_DTSSTAT_OFFSET     0x0090    /* DTS Status */
#define XMC4_GCU_SDMMCDEL_OFFSET    0x009c    /* SD-MMC Delay Control Register */ 
#define XMC4_GCU_G0ORCEN_OFFSET     0x00a0    /* Out-Of-Range Comparator Enable Register 0 */
#define XMC4_GCU_G1ORCEN_OFFSET     0x00a4    /* Out-Of-Range Comparator Enable Register 1 */
#define XMC4_GCU_MIRRSTS_OFFSET     0x00c4    /* Mirror Update Status Register */
#define XMC4_GCU_RMACR_OFFSET       0x00c8    /* Retention Memory Access Control Register */
#define XMC4_GCU_RMADATA_OFFSET     0x00cc    /* Retention Memory Access Data Register */
#define XMC4_GCU_PEEN_OFFSET        0x013c    /* Parity Error Enable Register */
#define XMC4_GCU_MCHKCON_OFFSET     0x0140    /* Memory Checking Control Register */
#define XMC4_GCU_PETE_OFFSET        0x0144    /* Parity Error Trap Enable Register */
#define XMC4_GCU_PERSTEN_OFFSET     0x0148    /* Reset upon Parity Error Enable Register */
#define XMC4_GCU_PEFLAG_OFFSET      0x0150    /* Parity Error Control Register */
#define XMC4_GCU_PMTPR_OFFSET       0x0154    /* Parity Memory Test Pattern Register */
#define XMC4_GCU_PMTSR_OFFSET       0x0158    /* Parity Memory Test Select Register */
#define XMC4_GCU_TRAPSTAT_OFFSET    0x0160    /* Trap Status Register */
#define XMC4_GCU_TRAPRAW_OFFSET     0x0164    /* Trap Raw Status Register */
#define XMC4_GCU_TRAPDIS_OFFSET     0x0168    /* Trap Mask Register */
#define XMC4_GCU_TRAPCLR_OFFSET     0x016c    /* Trap Clear Register */
#define XMC4_GCU_TRAPSET_OFFSET     0x0170    /* Trap Set Register */

/* PCU Registers */

#define XMC4_PCU_PWRSTAT_OFFSET     0x0000    /* Power Status Register */
#define XMC4_PCU_PWRSET_OFFSET      0x0004    /* Power Set Control Register */
#define XMC4_PCU_PWRCLR_OFFSET      0x0008    /* Power Clear Control Register */
#define XMC4_PCU_EVRSTAT_OFFSET     0x0010    /* EVR Status Register */
#define XMC4_PCU_EVRVADCSTAT_OFFSET 0x0014    /* EVR VADC Status Register */
#define XMC4_PCU_PWRMON_OFFSET      0x002c    /* Power Monitor Value */

/* HCU Registers */

#define XMC4_HCU_HDSTAT_OFFSET      0x0000    /* Hibernate Domain Status Register */
#define XMC4_HCU_HDCLR_OFFSET       0x0004    /* Hibernate Domain Status Clear Register */
#define XMC4_HCU_HDSET_OFFSET       0x0008    /* Hibernate Domain Status Set Register */
#define XMC4_HCU_HDCR_OFFSET        0x000c    /* Hibernate Domain Control Register */
#define XMC4_HCU_OSCSICTRL_OFFSET   0x0014    /* Internal 32.768 kHz Clock Source Control Register */
#define XMC4_HCU_OSCULSTAT_OFFSET   0x0018    /* OSC_ULP Status Register */
#define XMC4_HCU_OSCULCTRL_OFFSET   0x001c    /* OSC_ULP Control Register */

/* RCU Registers */

#define XMC4_RCU_RSTSTAT_OFFSET     0x0000    /* System Reset Status */
#define XMC4_RCU_RSTSET_OFFSET      0x0004    /* Reset Set Register */
#define XMC4_RCU_RSTCLR_OFFSET      0x0008    /* Reset Clear Register */
#define XMC4_RCU_PRSTAT0_OFFSET     0x000c    /* Peripheral Reset Status Register 0 */
#define XMC4_RCU_PRSET0_OFFSET      0x0010    /* Peripheral Reset Set Register 0 */
#define XMC4_RCU_PRCLR0_OFFSET      0x0014    /* Peripheral Reset Clear Register 0 */
#define XMC4_RCU_PRSTAT1_OFFSET     0x0018    /* Peripheral Reset Status Register 1 */
#define XMC4_RCU_PRSET1_OFFSET      0x001c    /* Peripheral Reset Set Register 1 */
#define XMC4_RCU_PRCLR1_OFFSET      0x0020    /* Peripheral Reset Clear Register 1 */
#define XMC4_RCU_PRSTAT2_OFFSET     0x0024    /* Peripheral Reset Status Register 2 */
#define XMC4_RCU_PRSET2_OFFSET      0x0028    /* Peripheral Reset Set Register 2 */
#define XMC4_RCU_PRCLR2_OFFSET      0x002c    /* Peripheral Reset Clear Register 2 */
#define XMC4_RCU_PRSTAT3_OFFSET     0x0030    /* Peripheral Reset Status Register 3 */
#define XMC4_RCU_PRSET3_OFFSET      0x0034    /* Peripheral Reset Set Register 3 */
#define XMC4_RCU_PRCLR3_OFFSET      0x0038    /* Peripheral Reset Clear Register 3 */

/* CCU Registers */

#define XMC4_CCU_CLKSTAT_OFFSET     0x0000    /* Clock Status Register */
#define XMC4_CCU_CLKSET_OFFSET      0x0004    /* Clock Set Control Register */
#define XMC4_CCU_CLKCLR_OFFSET      0x0008    /* Clock clear Control Register */
#define XMC4_CCU_SYSCLKCR_OFFSET    0x000c    /* System Clock Control */
#define XMC4_CCU_CPUCLKCR_OFFSET    0x0010    /* CPU Clock Control */
#define XMC4_CCU_PBCLKCR_OFFSET     0x0014    /* Peripheral Bus Clock Control */
#define XMC4_CCU_USBCLKCR_OFFSET    0x0018    /* USB Clock Control */
#define XMC4_CCU_EBUCLKCR_OFFSET    0x001c    /* EBU Clock Control */
#define XMC4_CCU_CCUCLKCR_OFFSET    0x0020    /* CCU Clock Control */
#define XMC4_CCU_WDTCLKCR_OFFSET    0x0024    /* WDT Clock Control */
#define XMC4_CCU_EXTCLKCR_OFFSET    0x0028    /* External clock Control Register */
#define XMC4_CCU_SLEEPCR_OFFSET     0x0030    /* Sleep Control Register */
#define XMC4_CCU_DSLEEPCR_OFFSET    0x0034    /* Deep Sleep Control Register */
#define XMC4_CCU_OSCHPSTAT_OFFSET   0x0100    /* OSC_HP Status Register */
#define XMC4_CCU_OSCHPCTRL_OFFSET   0x0104    /* OSC_HP Control Register */
#define XMC4_CCU_CLKCALCONST_OFFSET 0x010c    /* Clock Calibration Constant Register */
#define XMC4_CCU_PLLSTAT_OFFSET     0x0110    /* System PLL Status Register */
#define XMC4_CCU_PLLCON0_OFFSET     0x0114    /* System PLL Configuration 0 Register */
#define XMC4_CCU_PLLCON1_OFFSET     0x0118    /* System PLL Configuration 1 Register */
#define XMC4_CCU_PLLCON2_OFFSET     0x011c    /* System PLL Configuration 2 Register */
#define XMC4_CCU_USBPLLSTAT_OFFSET  0x0120    /* USB PLL Status Register */
#define XMC4_CCU_USBPLLCON_OFFSET   0x0124    /* USB PLL Control Register */
#define XMC4_CCU_CLKMXSTAT_OFFSET   0x0138    /* Clock Multiplexing Status Register */

/* Register Addresses ***************************************************************/

#define XMC4_GCU_BASE               (XMC4_SCU_BASE+XMC4_GCU_OFFSET)
#define XMC4_PCU_BASE               (XMC4_SCU_BASE+XMC4_PCU_OFFSET)
#define XMC4_HCU_BASE               (XMC4_SCU_BASE+XMC4_HCU_OFFSET)
#define XMC4_RCU_BASE               (XMC4_SCU_BASE+XMC4_RCU_OFFSET)
#define XMC4_CCU_BASE               (XMC4_SCU_BASE+XMC4_CCU_OFFSET)

/* General SCU Registers */

#define XMC4_GCU_ID                 (XMC4_GCU_BASE+XMC4_GCU_ID_OFFSET)
#define XMC4_GCU_IDCHIP             (XMC4_GCU_BASE+XMC4_GCU_IDCHIP_OFFSET)
#define XMC4_GCU_IDMANUF            (XMC4_GCU_BASE+XMC4_GCU_IDMANUF_OFFSET)
#define XMC4_GCU_STCON              (XMC4_GCU_BASE+XMC4_GCU_STCON_OFFSET)
#define XMC4_GCU_GPR0               (XMC4_GCU_BASE+XMC4_GCU_GPR0_OFFSET)
#define XMC4_GCU_GPR1               (XMC4_GCU_BASE+XMC4_GCU_GPR1_OFFSET)
#define XMC4_GCU_ETH0CON            (XMC4_GCU_BASE+XMC4_GCU_ETH0CON_OFFSET)
#define XMC4_GCU_CCUCON             (XMC4_GCU_BASE+XMC4_GCU_CCUCON_OFFSET)
#define XMC4_GCU_SRSTAT             (XMC4_GCU_BASE+XMC4_GCU_SRSTAT_OFFSET)
#define XMC4_GCU_SRRAW              (XMC4_GCU_BASE+XMC4_GCU_SRRAW_OFFSET)
#define XMC4_GCU_SRMSK              (XMC4_GCU_BASE+XMC4_GCU_SRMSK_OFFSET)
#define XMC4_GCU_SRCLR              (XMC4_GCU_BASE+XMC4_GCU_SRCLR_OFFSET)
#define XMC4_GCU_SRSET              (XMC4_GCU_BASE+XMC4_GCU_SRSET_OFFSET)
#define XMC4_GCU_NMIREQEN           (XMC4_GCU_BASE+XMC4_GCU_NMIREQEN_OFFSET)
#define XMC4_GCU_DTSCON             (XMC4_GCU_BASE+XMC4_GCU_DTSCON_OFFSET)
#define XMC4_GCU_DTSSTAT            (XMC4_GCU_BASE+XMC4_GCU_DTSSTAT_OFFSET)
#define XMC4_GCU_SDMMCDEL           (XMC4_GCU_BASE+XMC4_GCU_SDMMCDEL_OFFSET)
#define XMC4_GCU_G0ORCEN            (XMC4_GCU_BASE+XMC4_GCU_G0ORCEN_OFFSET)
#define XMC4_GCU_G1ORCEN            (XMC4_GCU_BASE+XMC4_GCU_G1ORCEN_OFFSET)
#define XMC4_GCU_MIRRSTS            (XMC4_GCU_BASE+XMC4_GCU_MIRRSTS_OFFSET)
#define XMC4_GCU_RMACR              (XMC4_GCU_BASE+XMC4_GCU_RMACR_OFFSET)
#define XMC4_GCU_RMADATA            (XMC4_GCU_BASE+XMC4_GCU_RMADATA_OFFSET)
#define XMC4_GCU_PEEN               (XMC4_GCU_BASE+XMC4_GCU_PEEN_OFFSET)
#define XMC4_GCU_MCHKCON            (XMC4_GCU_BASE+XMC4_GCU_MCHKCON_OFFSET)
#define XMC4_GCU_PETE               (XMC4_GCU_BASE+XMC4_GCU_PETE_OFFSET)
#define XMC4_GCU_PERSTEN            (XMC4_GCU_BASE+XMC4_GCU_PERSTEN_OFFSET)
#define XMC4_GCU_PEFLAG             (XMC4_GCU_BASE+XMC4_GCU_PEFLAG_OFFSET)
#define XMC4_GCU_PMTPR              (XMC4_GCU_BASE+XMC4_GCU_PMTPR_OFFSET)
#define XMC4_GCU_PMTSR              (XMC4_GCU_BASE+XMC4_GCU_PMTSR_OFFSET)
#define XMC4_GCU_TRAPSTAT           (XMC4_GCU_BASE+XMC4_GCU_TRAPSTAT_OFFSET)
#define XMC4_GCU_TRAPRAW            (XMC4_GCU_BASE+XMC4_GCU_TRAPRAW_OFFSET)
#define XMC4_GCU_TRAPDIS            (XMC4_GCU_BASE+XMC4_GCU_TRAPDIS_OFFSET)
#define XMC4_GCU_TRAPCLR            (XMC4_GCU_BASE+XMC4_GCU_TRAPCLR_OFFSET)
#define XMC4_GCU_TRAPSET            (XMC4_GCU_BASE+XMC4_GCU_TRAPSET_OFFSET)

/* PCU Registers */

#define XMC4_PCU_PWRSTAT            (XMC4_PCU_BASE+XMC4_PCU_PWRSTAT_OFFSET)
#define XMC4_PCU_PWRSET             (XMC4_PCU_BASE+XMC4_PCU_PWRSET_OFFSET)
#define XMC4_PCU_PWRCLR             (XMC4_PCU_BASE+XMC4_PCU_PWRCLR_OFFSET)
#define XMC4_PCU_EVRSTAT            (XMC4_PCU_BASE+XMC4_PCU_EVRSTAT_OFFSET)
#define XMC4_PCU_EVRVADCSTAT        (XMC4_PCU_BASE+XMC4_PCU_EVRVADCSTAT_OFFSET)
#define XMC4_PCU_PWRMON             (XMC4_PCU_BASE+XMC4_PCU_PWRMON_OFFSET)

/* HCU Registers */

#define XMC4_HCU_HDSTAT             (XMC4_HCU_BASE+XMC4_HCU_HDSTAT_OFFSET)
#define XMC4_HCU_HDCLR              (XMC4_HCU_BASE+XMC4_HCU_HDCLR_OFFSET)
#define XMC4_HCU_HDSET              (XMC4_HCU_BASE+XMC4_HCU_HDSET_OFFSET)
#define XMC4_HCU_HDCR               (XMC4_HCU_BASE+XMC4_HCU_HDCR_OFFSET)
#define XMC4_HCU_OSCSICTRL          (XMC4_HCU_BASE+XMC4_HCU_OSCSICTRL_OFFSET)
#define XMC4_HCU_OSCULSTAT          (XMC4_HCU_BASE+XMC4_HCU_OSCULSTAT_OFFSET)
#define XMC4_HCU_OSCULCTRL          (XMC4_HCU_BASE+XMC4_HCU_OSCULCTRL_OFFSET)

/* RCU Registers */

#define XMC4_RCU_RSTSTAT            (XMC4_RCU_BASE+XMC4_RCU_RSTSTAT_OFFSET)
#define XMC4_RCU_RSTSET             (XMC4_RCU_BASE+XMC4_RCU_RSTSET_OFFSET)
#define XMC4_RCU_RSTCLR             (XMC4_RCU_BASE+XMC4_RCU_RSTCLR_OFFSET)
#define XMC4_RCU_PRSTAT0            (XMC4_RCU_BASE+XMC4_RCU_PRSTAT0_OFFSET)
#define XMC4_RCU_PRSET0             (XMC4_RCU_BASE+XMC4_RCU_PRSET0_OFFSET)
#define XMC4_RCU_PRCLR0             (XMC4_RCU_BASE+XMC4_RCU_PRCLR0_OFFSET)
#define XMC4_RCU_PRSTAT1            (XMC4_RCU_BASE+XMC4_RCU_PRSTAT1_OFFSET)
#define XMC4_RCU_PRSET1             (XMC4_RCU_BASE+XMC4_RCU_PRSET1_OFFSET)
#define XMC4_RCU_PRCLR1             (XMC4_RCU_BASE+XMC4_RCU_PRCLR1_OFFSET)
#define XMC4_RCU_PRSTAT2            (XMC4_RCU_BASE+XMC4_RCU_PRSTAT2_OFFSET)
#define XMC4_RCU_PRSET2             (XMC4_RCU_BASE+XMC4_RCU_PRSET2_OFFSET)
#define XMC4_RCU_PRCLR2             (XMC4_RCU_BASE+XMC4_RCU_PRCLR2_OFFSET)
#define XMC4_RCU_PRSTAT3            (XMC4_RCU_BASE+XMC4_RCU_PRSTAT3_OFFSET)
#define XMC4_RCU_PRSET3             (XMC4_RCU_BASE+XMC4_RCU_PRSET3_OFFSET)
#define XMC4_RCU_PRCLR3             (XMC4_RCU_BASE+XMC4_RCU_PRCLR3_OFFSET)

/* CCU Registers */

#define XMC4_CCU_CLKSTAT            (XMC4_CCU_BASE+XMC4_CCU_CLKSTAT_OFFSET)
#define XMC4_CCU_CLKSET             (XMC4_CCU_BASE+XMC4_CCU_CLKSET_OFFSET)
#define XMC4_CCU_CLKCLR             (XMC4_CCU_BASE+XMC4_CCU_CLKCLR_OFFSET)
#define XMC4_CCU_SYSCLKCR           (XMC4_CCU_BASE+XMC4_CCU_SYSCLKCR_OFFSET)
#define XMC4_CCU_CPUCLKCR           (XMC4_CCU_BASE+XMC4_CCU_CPUCLKCR_OFFSET)
#define XMC4_CCU_PBCLKCR            (XMC4_CCU_BASE+XMC4_CCU_PBCLKCR_OFFSET)
#define XMC4_CCU_USBCLKCR           (XMC4_CCU_BASE+XMC4_CCU_USBCLKCR_OFFSET)
#define XMC4_CCU_EBUCLKCR           (XMC4_CCU_BASE+XMC4_CCU_EBUCLKCR_OFFSET)
#define XMC4_CCU_CCUCLKCR           (XMC4_CCU_BASE+XMC4_CCU_CCUCLKCR_OFFSET)
#define XMC4_CCU_WDTCLKCR           (XMC4_CCU_BASE+XMC4_CCU_WDTCLKCR_OFFSET)
#define XMC4_CCU_EXTCLKCR           (XMC4_CCU_BASE+XMC4_CCU_EXTCLKCR_OFFSET)
#define XMC4_CCU_SLEEPCR            (XMC4_CCU_BASE+XMC4_CCU_SLEEPCR_OFFSET)
#define XMC4_CCU_DSLEEPCR           (XMC4_CCU_BASE+XMC4_CCU_DSLEEPCR_OFFSET)
#define XMC4_CCU_OSCHPSTAT          (XMC4_CCU_BASE+XMC4_CCU_OSCHPSTAT_OFFSET)
#define XMC4_CCU_OSCHPCTRL          (XMC4_CCU_BASE+XMC4_CCU_OSCHPCTRL_OFFSET)
#define XMC4_CCU_CLKCALCONST        (XMC4_CCU_BASE+XMC4_CCU_CLKCALCONST_OFFSET)
#define XMC4_CCU_PLLSTAT            (XMC4_CCU_BASE+XMC4_CCU_PLLSTAT_OFFSET)
#define XMC4_CCU_PLLCON0            (XMC4_CCU_BASE+XMC4_CCU_PLLCON0_OFFSET)
#define XMC4_CCU_PLLCON1            (XMC4_CCU_BASE+XMC4_CCU_PLLCON1_OFFSET)
#define XMC4_CCU_PLLCON2            (XMC4_CCU_BASE+XMC4_CCU_PLLCON2_OFFSET)
#define XMC4_CCU_USBPLLSTAT         (XMC4_CCU_BASE+XMC4_CCU_USBPLLSTAT_OFFSET)
#define XMC4_CCU_USBPLLCON          (XMC4_CCU_BASE+XMC4_CCU_USBPLLCON_OFFSET)
#define XMC4_CCU_CLKMXSTAT          (XMC4_CCU_BASE+XMC4_CCU_CLKMXSTAT_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* General SCU Registers */

/* Module Identification Register */
#define GCU_ID_
/* Chip ID */
#define GCU_IDCHIP_
/* Manufactory ID */
#define GCU_IDMANUF_
/* Start-up Control */
#define GCU_STCON_
/* General Purpose Register 0 */
#define GCU_GPR0_
/* General Purpose Register 1 */
#define GCU_GPR1_
/* Ethernet 0 Port Control */
#define GCU_ETH0CON_
/* CCUx Global Start Control Register */
#define GCU_CCUCON_
/* Service Request Status */
#define GCU_SRSTAT_
/* RAW Service Request Status */
#define GCU_SRRAW_
/* Service Request Mask */
#define GCU_SRMSK_
/* Service Request Clear */
#define GCU_SRCLR_
/* Service Request Set */
#define GCU_SRSET_
/* Enable Promoting Events to NMI Request */
#define GCU_NMIREQEN_
/* DTS Control */
#define GCU_DTSCON_
/* DTS Status */
#define GCU_DTSSTAT_
/* SD-MMC Delay Control Register */ 
#define GCU_SDMMCDEL_
/* Out-Of-Range Comparator Enable Register 0 */
#define GCU_G0ORCEN_
/* Out-Of-Range Comparator Enable Register 1 */
#define GCU_G1ORCEN_
/* Mirror Update Status Register */
#define GCU_MIRRSTS_
/* Retention Memory Access Control Register */
#define GCU_RMACR_
/* Retention Memory Access Data Register */
#define GCU_RMADATA_
/* Parity Error Enable Register */
#define GCU_PEEN_
/* Memory Checking Control Register */
#define GCU_MCHKCON_
/* Parity Error Trap Enable Register */
#define GCU_PETE_
/* Reset upon Parity Error Enable Register */
#define GCU_PERSTEN_
/* Parity Error Control Register */
#define GCU_PEFLAG_
/* Parity Memory Test Pattern Register */
#define GCU_PMTPR_
/* Parity Memory Test Select Register */
#define GCU_PMTSR_
/* Trap Status Register */
#define GCU_TRAPSTAT_
/* Trap Raw Status Register */
#define GCU_TRAPRAW_
/* Trap Mask Register */
#define GCU_TRAPDIS_
/* Trap Clear Register */
#define GCU_TRAPCLR_
/* Trap Set Register */
#define GCU_TRAPSET_

/* PCU Registers */

/* Power Status Register */
#define PCU_PWRSTAT_
/* Power Set Control Register */
#define PCU_PWRSET_
/* Power Clear Control Register */
#define PCU_PWRCLR_
/* EVR Status Register */
#define PCU_EVRSTAT_
/* EVR VADC Status Register */
#define PCU_EVRVADCSTAT_
/* Power Monitor Value */
#define PCU_PWRMON_

/* HCU Registers */

/* Hibernate Domain Status Register */
#define HCU_HDSTAT_
/* Hibernate Domain Status Clear Register */
#define HCU_HDCLR_
/* Hibernate Domain Status Set Register */
#define HCU_HDSET_
/* Hibernate Domain Control Register */
#define HCU_HDCR_
/* Internal 32.768 kHz Clock Source Control Register */
#define HCU_OSCSICTRL_
/* OSC_ULP Status Register */
#define HCU_OSCULSTAT_
/* OSC_ULP Control Register */
#define HCU_OSCULCTRL_

/* RCU Registers */

/* System Reset Status */
#define RCU_RSTSTAT_
/* Reset Set Register */
#define RCU_RSTSET_
/* Reset Clear Register */
#define RCU_RSTCLR_
/* Peripheral Reset Status Register 0 */
#define RCU_PRSTAT0_
/* Peripheral Reset Set Register 0 */
#define RCU_PRSET0_
/* Peripheral Reset Clear Register 0 */
#define RCU_PRCLR0_
/* Peripheral Reset Status Register 1 */
#define RCU_PRSTAT1_
/* Peripheral Reset Set Register 1 */
#define RCU_PRSET1_
/* Peripheral Reset Clear Register 1 */
#define RCU_PRCLR1_
/* Peripheral Reset Status Register 2 */
#define RCU_PRSTAT2_
/* Peripheral Reset Set Register 2 */
#define RCU_PRSET2_
/* Peripheral Reset Clear Register 2 */
#define RCU_PRCLR2_
/* Peripheral Reset Status Register 3 */
#define RCU_PRSTAT3_
/* Peripheral Reset Set Register 3 */
#define RCU_PRSET3_
/* Peripheral Reset Clear Register 3 */
#define RCU_PRCLR3_

/* CCU Registers */

/* Clock Status Register */
#define CCU_CLKSTAT_
/* Clock Set Control Register */
#define CCU_CLKSET_
/* Clock clear Control Register */
#define CCU_CLKCLR_
/* System Clock Control */
#define CCU_SYSCLKCR_
/* CPU Clock Control */
#define CCU_CPUCLKCR_
/* Peripheral Bus Clock Control */
#define CCU_PBCLKCR_
/* USB Clock Control */
#define CCU_USBCLKCR_
/* EBU Clock Control */
#define CCU_EBUCLKCR_
/* CCU Clock Control */
#define CCU_CCUCLKCR_
/* WDT Clock Control */
#define CCU_WDTCLKCR_
/* External clock Control Register */
#define CCU_EXTCLKCR_
/* Sleep Control Register */
#define CCU_SLEEPCR_
/* Deep Sleep Control Register */
#define CCU_DSLEEPCR_
/* OSC_HP Status Register */
#define CCU_OSCHPSTAT_
/* OSC_HP Control Register */
#define CCU_OSCHPCTRL_
/* Clock Calibration Constant Register */
#define CCU_CLKCALCONST_
/* System PLL Status Register */
#define CCU_PLLSTAT_
/* System PLL Configuration 0 Register */
#define CCU_PLLCON0_
/* System PLL Configuration 1 Register */
#define CCU_PLLCON1_
/* System PLL Configuration 2 Register */
#define CCU_PLLCON2_
/* USB PLL Status Register */
#define CCU_USBPLLSTAT_
/* USB PLL Control Register */
#define CCU_USBPLLCON_
/* Clock Multiplexing Status Register */
#define CCU_CLKMXSTAT_

#endif /* __ARCH_ARM_SRC_XMC4_CHIP_XMC4_SCU_H */
