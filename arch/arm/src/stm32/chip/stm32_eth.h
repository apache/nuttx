/****************************************************************************************************
 * arch/arm/src/stm32/chip/stm32_eth.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_ETH_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_ETH_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if STM32_NETHERNET > 1

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/
/* MAC Registers */

#define STM32_ETH_MACCR_OFFSET       0x0000 /* Ethernet MAC configuration register */
#define STM32_ETH_MACFFR_OFFSET      0x0004 /* Ethernet MAC frame filter register */
#define STM32_ETH_MACHTHR_OFFSET     0x0008 /* Ethernet MAC hash table high register */
#define STM32_ETH_MACHTLR_OFFSET     0x000c /* Ethernet MAC hash table low register */
#define STM32_ETH_MACMIIAR_OFFSET    0x0010 /* Ethernet MAC MII address register */
#define STM32_ETH_MACMIIDR_OFFSET    0x0014 /* Ethernet MAC MII data register */
#define STM32_ETH_MACFCR_OFFSET      0x0018 /* Ethernet MAC flow control register */
#define STM32_ETH_MACVLANTR_OFFSET   0x001c /* Ethernet MAC VLAN tag register */
#define STM32_ETH_MACRWUFFR_OFFSET   0x0028 /* Ethernet MAC remote wakeup frame filter reg */
#define STM32_ETH_MACPMTCSR_OFFSET   0x002c /* Ethernet MAC PMT control and status register */
#define STM32_ETH_MACDBGR_OFFSET     0x0034 /* Ethernet MAC debug register */
#define STM32_ETH_MACSR_OFFSET       0x0038 /* Ethernet MAC interrupt status register */
#define STM32_ETH_MACIMR_OFFSET      0x003c /* Ethernet MAC interrupt mask register */
#define STM32_ETH_MACA0HR_OFFSET     0x0040 /* Ethernet MAC address 0 high register */
#define STM32_ETH_MACA0LR_OFFSET     0x0044 /* Ethernet MAC address 0 low register */
#define STM32_ETH_MACA1HR_OFFSET     0x0048 /* Ethernet MAC address 1 high register */
#define STM32_ETH_MACA1LR_OFFSET     0x004c /* Ethernet MAC address1 low register */
#define STM32_ETH_MACA2HR_OFFSET     0x0050 /* Ethernet MAC address 2 high register */
#define STM32_ETH_MACA2LR_OFFSET     0x0054 /* Ethernet MAC address 2 low register */
#define STM32_ETH_MACA3HR_OFFSET     0x0058 /* Ethernet MAC address 3 high register */
#define STM32_ETH_MACA3LR_OFFSET     0x005c /* Ethernet MAC address 3 low register */

/* MMC Registers */

#define STM32_ETH_MMCC_OFFSET        0x0100 /* Ethernet MMC control register */
#define STM32_ETH_MMCRIR_OFFSET      0x0104 /* Ethernet MMC receive interrupt register */
#define STM32_ETH_MMCTIR_OFFSET      0x0108 /* Ethernet MMC transmit interrupt register */
#define STM32_ETH_MMCRIMR_OFFSET     0x010c /* Ethernet MMC receive interrupt mask register */
#define STM32_ETH_MMCTIMR_OFFSET     0x0110 /* Ethernet MMC transmit interrupt mask register */
#define STM32_ETH_MMCTGFSCCR_OFFSET  0x014c /* Ethernet MMC transmitted good frames counter register (single collision) */
#define STM32_ETH_MMCTGFMSCCR_OFFSET 0x0150 /* Ethernet MMC transmitted good frames counter register (multiple-collision) */
#define STM32_ETH_MMCTGFCR_OFFSET    0x0168 /* Ethernet MMC transmitted good frames counter register */
#define STM32_ETH_MMCRFCECR_OFFSET   0x0194 /* Ethernet MMC received frames with CRC error counter register */
#define STM32_ETH_MMCRFAECR_OFFSET   0x0198 /* Ethernet MMC received frames with alignment error counter */
#define STM32_ETH_MMCRGUFCR_OFFSET   0x01c4 /* MMC received good unicast frames counter register */

/* IEEE 1588 time stamp registers */

#define STM32_ETH_PTPTSCR_OFFSET     0x0700 /* Ethernet PTP time stamp control register */
#define STM32_ETH_PTPSSIR_OFFSET     0x0704 /* Ethernet PTP subsecond increment register */
#define STM32_ETH_PTPTSHR_OFFSET     0x0708 /* Ethernet PTP time stamp high register */
#define STM32_ETH_PTPTSLR_OFFSET     0x070c /* Ethernet PTP time stamp low register */
#define STM32_ETH_PTPTSHUR_OFFSET    0x0710 /* Ethernet PTP time stamp high update register */
#define STM32_ETH_PTPTSLUR_OFFSET    0x0714 /* Ethernet PTP time stamp low update register */
#define STM32_ETH_PTPTSAR_OFFSET     0x0718 /* Ethernet PTP time stamp addend register */
#define STM32_ETH_PTPTTHR_OFFSET     0x071c /* Ethernet PTP target time high register */
#define STM32_ETH_PTPTTLR_OFFSET     0x0720 /* Ethernet PTP target time low register */
#define STM32_ETH_PTPTSSR_OFFSET     0x0728 /* Ethernet PTP time stamp status register */

/* DMA Registers */

#define STM32_ETH_DMABMR_OFFSET      0x1000 /* Ethernet DMA bus mode register */
#define STM32_ETH_DMATPDR_OFFSET     0x1004 /* Ethernet DMA transmit poll demand register */
#define STM32_ETH_DMARPDR_OFFSET     0x1008 /* EHERNET DMA receive poll demand register */
#define STM32_ETH_DMARDLAR_OFFSET    0x100c /* Ethernet DMA receive descriptor list address register */
#define STM32_ETH_DMATDLAR_OFFSET    0x1010 /* Ethernet DMA transmit descriptor list address register */
#define STM32_ETH_DMASR_OFFSET       0x1014 /* Ethernet DMA status register */
#define STM32_ETH_DMAOMR_OFFSET      0x1018 /* Ethernet DMA operation mode register */
#define STM32_ETH_DMAIER_OFFSET      0x101c /* Ethernet DMA interrupt enable register */
#define STM32_ETH_DMAMFBOC_OFFSET    0x1020 /* Ethernet DMA missed frame and buffer overflow counter register */
#define STM32_ETH_DMARSWTR_OFFSET    0x1024 /* Ethernet DMA receive status watchdog timer register */
#define STM32_ETH_DMACHTDR_OFFSET    0x1048 /* Ethernet DMA current host transmit descriptor register */
#define STM32_ETH_DMACHRDR_OFFSET    0x104c /* Ethernet DMA current host receive descriptor register */
#define STM32_ETH_DMACHTBAR_OFFSET   0x1050 /* Ethernet DMA current host transmit buffer address register */
#define STM32_ETH_DMACHRBAR_OFFSET   0x1054 /* Ethernet DMA current host receive buffer address register */

/* Register Base Addresses **************************************************************************/
/* MAC Registers */

#define STM32_ETH_MACCR              (STM32_ETHERNET_BASE+STM32_ETH_MACCR_OFFSET)
#define STM32_ETH_MACFFR             (STM32_ETHERNET_BASE+STM32_ETH_MACFFR_OFFSET)
#define STM32_ETH_MACHTHR            (STM32_ETHERNET_BASE+STM32_ETH_MACHTHR_OFFSET)
#define STM32_ETH_MACHTLR            (STM32_ETHERNET_BASE+STM32_ETH_MACHTLR_OFFSET)
#define STM32_ETH_MACMIIAR           (STM32_ETHERNET_BASE+STM32_ETH_MACMIIAR_OFFSET)
#define STM32_ETH_MACMIIDR           (STM32_ETHERNET_BASE+STM32_ETH_MACMIIDR_OFFSET)
#define STM32_ETH_MACFCR             (STM32_ETHERNET_BASE+STM32_ETH_MACFCR_OFFSET)
#define STM32_ETH_MACVLANTR          (STM32_ETHERNET_BASE+STM32_ETH_MACVLANTR_OFFSET)
#define STM32_ETH_MACRWUFFR          (STM32_ETHERNET_BASE+STM32_ETH_MACRWUFFR_OFFSET)
#define STM32_ETH_MACPMTCSR          (STM32_ETHERNET_BASE+STM32_ETH_MACPMTCSR_OFFSET)
#define STM32_ETH_MACDBGR            (STM32_ETHERNET_BASE+STM32_ETH_MACDBGR_OFFSET)
#define STM32_ETH_MACSR              (STM32_ETHERNET_BASE+STM32_ETH_MACSR_OFFSET)
#define STM32_ETH_MACIMR             (STM32_ETHERNET_BASE+STM32_ETH_MACIMR_OFFSET)
#define STM32_ETH_MACA0HR            (STM32_ETHERNET_BASE+STM32_ETH_MACA0HR_OFFSET)
#define STM32_ETH_MACA0LR            (STM32_ETHERNET_BASE+STM32_ETH_MACA0LR_OFFSET)
#define STM32_ETH_MACA1HR            (STM32_ETHERNET_BASE+STM32_ETH_MACA1HR_OFFSET)
#define STM32_ETH_MACA1LR            (STM32_ETHERNET_BASE+STM32_ETH_MACA1LR_OFFSET)
#define STM32_ETH_MACA2HR            (STM32_ETHERNET_BASE+STM32_ETH_MACA2HR_OFFSET)
#define STM32_ETH_MACA2LR            (STM32_ETHERNET_BASE+STM32_ETH_MACA2LR_OFFSET)
#define STM32_ETH_MACA3HR            (STM32_ETHERNET_BASE+STM32_ETH_MACA3HR_OFFSET)
#define STM32_ETH_MACA3LR            (STM32_ETHERNET_BASE+STM32_ETH_MACA3LR_OFFSET)

/* MMC Registers */

#define STM32_ETH_MMCC               (STM32_ETHERNET_BASE+STM32_ETH_MMCC_OFFSET)
#define STM32_ETH_MMCRIR             (STM32_ETHERNET_BASE+STM32_ETH_MMCRIR_OFFSET)
#define STM32_ETH_MMCTIR             (STM32_ETHERNET_BASE+STM32_ETH_MMCTIR_OFFSET)
#define STM32_ETH_MMCRIMR            (STM32_ETHERNET_BASE+STM32_ETH_MMCRIMR_OFFSET)
#define STM32_ETH_MMCTIMR            (STM32_ETHERNET_BASE+STM32_ETH_MMCTIMR_OFFSET)
#define STM32_ETH_MMCTGFSCCR         (STM32_ETHERNET_BASE+STM32_ETH_MMCTGFSCCR_OFFSET)
#define STM32_ETH_MMCTGFMSCCR        (STM32_ETHERNET_BASE+STM32_ETH_MMCTGFMSCCR_OFFSET)
#define STM32_ETH_MMCTGFCR           (STM32_ETHERNET_BASE+STM32_ETH_MMCTGFCR_OFFSET)
#define STM32_ETH_MMCRFCECR          (STM32_ETHERNET_BASE+STM32_ETH_MMCRFCECR_OFFSET)
#define STM32_ETH_MMCRFAECR          (STM32_ETHERNET_BASE+STM32_ETH_MMCRFAECR_OFFSET)
#define STM32_ETH_MMCRGUFCR          (STM32_ETHERNET_BASE+STM32_ETH_MMCRGUFCR_OFFSET)

/* IEEE 1588 time stamp registers */

#define STM32_ETH_PTPTSCR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTSCR_OFFSET)
#define STM32_ETH_PTPSSIR            (STM32_ETHERNET_BASE+STM32_ETH_PTPSSIR_OFFSET)
#define STM32_ETH_PTPTSHR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTSHR_OFFSET)
#define STM32_ETH_PTPTSLR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTSLR_OFFSET)
#define STM32_ETH_PTPTSHUR           (STM32_ETHERNET_BASE+STM32_ETH_PTPTSHUR_OFFSET)
#define STM32_ETH_PTPTSLUR           (STM32_ETHERNET_BASE+STM32_ETH_PTPTSLUR_OFFSET)
#define STM32_ETH_PTPTSAR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTSAR_OFFSET)
#define STM32_ETH_PTPTTHR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTTHR_OFFSET)
#define STM32_ETH_PTPTTLR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTTLR_OFFSET)
#define STM32_ETH_PTPTSSR            (STM32_ETHERNET_BASE+STM32_ETH_PTPTSSR_OFFSET)

/* DMA Registers */

#define STM32_ETH_DMABMR             (STM32_ETHERNET_BASE+STM32_ETH_DMABMR_OFFSET)
#define STM32_ETH_DMATPDR            (STM32_ETHERNET_BASE+STM32_ETH_DMATPDR_OFFSET)
#define STM32_ETH_DMARPDR            (STM32_ETHERNET_BASE+STM32_ETH_DMARPDR_OFFSET)
#define STM32_ETH_DMARDLAR           (STM32_ETHERNET_BASE+STM32_ETH_DMARDLAR_OFFSET)
#define STM32_ETH_DMATDLAR           (STM32_ETHERNET_BASE+STM32_ETH_DMATDLAR_OFFSET)
#define STM32_ETH_DMASR              (STM32_ETHERNET_BASE+STM32_ETH_DMASR_OFFSET)
#define STM32_ETH_DMAOMR             (STM32_ETHERNET_BASE+STM32_ETH_DMAOMR_OFFSET)
#define STM32_ETH_DMAIER             (STM32_ETHERNET_BASE+STM32_ETH_DMAIER_OFFSET)
#define STM32_ETH_DMAMFBOC           (STM32_ETHERNET_BASE+STM32_ETH_DMAMFBOC_OFFSET)
#define STM32_ETH_DMARSWTR           (STM32_ETHERNET_BASE+STM32_ETH_DMARSWTR_OFFSET)
#define STM32_ETH_DMACHTDR           (STM32_ETHERNET_BASE+STM32_ETH_DMACHTDR_OFFSET)
#define STM32_ETH_DMACHRDR           (STM32_ETHERNET_BASE+STM32_ETH_DMACHRDR_OFFSET)
#define STM32_ETH_DMACHTBAR          (STM32_ETHERNET_BASE+STM32_ETH_DMACHTBAR_OFFSET)
#define STM32_ETH_DMACHRBAR          (STM32_ETHERNET_BASE+STM32_ETH_DMACHRBAR_OFFSET)

/* Register Bit-Field Definitions *******************************************************************/
/* MAC Registers */

/* Ethernet MAC configuration register */
#define ETH_MACCR_
/* Ethernet MAC frame filter register */
#define ETH_MACFFR_
/* Ethernet MAC hash table high register */
#define ETH_MACHTHR_
/* Ethernet MAC hash table low register */
#define ETH_MACHTLR_
/* Ethernet MAC MII address register */
#define ETH_MACMIIAR_
/* Ethernet MAC MII data register */
#define ETH_MACMIIDR_
/* Ethernet MAC flow control register */
#define ETH_MACFCR_
/* Ethernet MAC VLAN tag register */
#define ETH_MACVLANTR_
/* Ethernet MAC remote wakeup frame filter reg */
#define ETH_MACRWUFFR_
/* Ethernet MAC PMT control and status register */
#define ETH_MACPMTCSR_
/* Ethernet MAC debug register */
#define ETH_MACDBGR_
/* Ethernet MAC interrupt status register */
#define ETH_MACSR_
/* Ethernet MAC interrupt mask register */
#define ETH_MACIMR_
/* Ethernet MAC address 0 high register */
#define ETH_MACA0HR_
/* Ethernet MAC address 0 low register */
#define ETH_MACA0LR_
/* Ethernet MAC address 1 high register */
#define ETH_MACA1HR_
/* Ethernet MAC address1 low register */
#define ETH_MACA1LR_
/* Ethernet MAC address 2 high register */
#define ETH_MACA2HR_
/* Ethernet MAC address 2 low register */
#define ETH_MACA2LR_
/* Ethernet MAC address 3 high register */
#define ETH_MACA3HR_
/* Ethernet MAC address 3 low register */
#define ETH_MACA3LR_

/* MMC Registers */

/* Ethernet MMC control register */
#define ETH_MMCC_
/* Ethernet MMC receive interrupt register */
#define ETH_MMCRIR_
/* Ethernet MMC transmit interrupt register */
#define ETH_MMCTIR_
/* Ethernet MMC receive interrupt mask register */
#define ETH_MMCRIMR_
/* Ethernet MMC transmit interrupt mask register */
#define ETH_MMCTIMR_
/* Ethernet MMC transmitted good frames counter register (single collision) */
#define ETH_MMCTGFSCCR_
/* Ethernet MMC transmitted good frames counter register (multiple-collision) */
#define ETH_MMCTGFMSCCR_
/* Ethernet MMC transmitted good frames counter register */
#define ETH_MMCTGFCR_
/* Ethernet MMC received frames with CRC error counter register */
#define ETH_MMCRFCECR_
/* Ethernet MMC received frames with alignment error counter */
#define ETH_MMCRFAECR_
/* MMC received good unicast frames counter register */
#define ETH_MMCRGUFCR_

/* IEEE 1588 time stamp registers */

/* Ethernet PTP time stamp control register */
#define ETH_PTPTSCR_
/* Ethernet PTP subsecond increment register */
#define ETH_PTPSSIR_
/* Ethernet PTP time stamp high register */
#define ETH_PTPTSHR_
/* Ethernet PTP time stamp low register */
#define ETH_PTPTSLR_
/* Ethernet PTP time stamp high update register */
#define ETH_PTPTSHUR_
/* Ethernet PTP time stamp low update register */
#define ETH_PTPTSLUR_
/* Ethernet PTP time stamp addend register */
#define ETH_PTPTSAR_
/* Ethernet PTP target time high register */
#define ETH_PTPTTHR_
/* Ethernet PTP target time low register */
#define ETH_PTPTTLR_
/* Ethernet PTP time stamp status register */
#define ETH_PTPTSSR_

/* DMA Registers */

/* Ethernet DMA bus mode register */
#define ETH_DMABMR_
/* Ethernet DMA transmit poll demand register */
#define ETH_DMATPDR_
/* EHERNET DMA receive poll demand register */
#define ETH_DMARPDR_
/* Ethernet DMA receive descriptor list address register */
#define ETH_DMARDLAR_
/* Ethernet DMA transmit descriptor list address register */
#define ETH_DMATDLAR_
/* Ethernet DMA status register */
#define ETH_DMASR_
/* Ethernet DMA operation mode register */
#define ETH_DMAOMR_
/* Ethernet DMA interrupt enable register */
#define ETH_DMAIER_
/* Ethernet DMA missed frame and buffer overflow counter register */
#define ETH_DMAMFBOC_
/* Ethernet DMA receive status watchdog timer register */
#define ETH_DMARSWTR_
/* Ethernet DMA current host transmit descriptor register */
#define ETH_DMACHTDR_
/* Ethernet DMA current host receive descriptor register */
#define ETH_DMACHRDR_
/* Ethernet DMA current host transmit buffer address register */
#define ETH_DMACHTBAR_
/* Ethernet DMA current host receive buffer address register */
#define ETH_DMACHRBAR_

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif


#endif /* __ASSEMBLY__ */
#endif /* STM32_NETHERNET > 1 */
#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_ETH_H */

