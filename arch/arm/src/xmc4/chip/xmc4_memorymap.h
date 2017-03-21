/************************************************************************************
 * arch/arm/src/xmc4/chip/xmc4_memorymap.h
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
 * Infineon Technologies AG (Infineon) is supplying this software for use with
 * Infineon's microcontrollers.  This file can be freely distributed within
 * development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_XMC4_CHIP_XMC4_MEMORYMAP_H
#define __ARCH_ARM_SRC_XMC4_CHIP_XMC4_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Peripheral Memory Map ************************************************************/
/* Acronyms:
 *  ADC   - Analog to Digital Converter
 *  CCU   - Capture Compare Unit
 *  DAC   - Digital to Analog Converter
 *  DSD   - Delta Sigmoid Demodulator
 *  ERU   - External Request Unit
 *  FCE   - Flexible CRC Engine
 *  GPDMA - General Purpose DMA
 *  LEDTS - LED and Touch Sense Control Unit
 *  PMU   - Program Management Unit
 *  POSIF - Position Interface
 *  SDMMC - Multi Media Card Interface
 *  USB   - Universal Serial Bus
 *  USCI  - Universal Serial Interface
 */

#define XMC4_PBA0_BASE              0x40000000 /* PBA0 */
#define XMC4_VADC_BASE              0x40004000 /* VADC */
#define XMC4_VADC_G0_BASE           0x40004400
#define XMC4_VADC_G1_BASE           0x40004800
#define XMC4_VADC_G2_BASE           0x40004c00
#define XMC4_VADC_G3_BASE           0x40005000
#define XMC4_DSD_BASE               0x40008000 /* DSD */
#define XMC4_DSD_CH0_BASE           0x40008100
#define XMC4_DSD_CH1_BASE           0x40008200
#define XMC4_DSD_CH2_BASE           0x40008300
#define XMC4_DSD_CH3_BASE           0x40008400
#define XMC4_CCU40_BASE             0x4000c000 /* CCU40 */
#define XMC4_CCU40_CC40_BASE        0x4000c100
#define XMC4_CCU40_CC41_BASE        0x4000c200
#define XMC4_CCU40_CC42_BASE        0x4000c300
#define XMC4_CCU40_CC43_BASE        0x4000c400
#define XMC4_CCU41_BASE             0x40010000 /* CCU41 */
#define XMC4_CCU41_CC40_BASE        0x40010100
#define XMC4_CCU41_CC41_BASE        0x40010200
#define XMC4_CCU41_CC42_BASE        0x40010300
#define XMC4_CCU41_CC43_BASE        0x40010400
#define XMC4_CCU42_BASE             0x40014000 /* CCU42 */
#define XMC4_CCU42_CC40_BASE        0x40014100
#define XMC4_CCU42_CC41_BASE        0x40014200
#define XMC4_CCU42_CC42_BASE        0x40014300
#define XMC4_CCU42_CC43_BASE        0x40014400
#define XMC4_CCU80_BASE             0x40020000 /* CCU80 */
#define XMC4_CCU80_CC80_BASE        0x40020100
#define XMC4_CCU80_CC81_BASE        0x40020200
#define XMC4_CCU80_CC82_BASE        0x40020300
#define XMC4_CCU80_CC83_BASE        0x40020400
#define XMC4_CCU81_BASE             0x40024000 /* CCU81 */
#define XMC4_CCU81_CC80_BASE        0x40024100
#define XMC4_CCU81_CC81_BASE        0x40024200
#define XMC4_CCU81_CC82_BASE        0x40024300
#define XMC4_CCU81_CC83_BASE        0x40024400
#define XMC4_POSIF0_BASE            0x40028000 /* POSIF0 */
#define XMC4_POSIF1_BASE            0x4002c000 /* POSIF1 */
#define XMC4_USIC0_BASE             0x40030000 /* USIC0 */
#define XMC4_USIC0_CH0_BASE         0x40030000
#define XMC4_USIC0_CH1_BASE         0x40030200
#define XMC4_USIC0_RAM_BASE         0x40030400
#define XMC4_ERU1_BASE              0x40044000 /* ERU1 */

#define XMC4_PBA1_BASE              0x48000000 /* PBA1 */
#define XMC4_CCU43_BASE             0x48004000 /* CCU43 */
#define XMC4_CCU43_CC40_BASE        0x48004100
#define XMC4_CCU43_CC41_BASE        0x48004200
#define XMC4_CCU43_CC42_BASE        0x48004300
#define XMC4_CCU43_CC43_BASE        0x48004400
#define XMC4_LEDTS0_BASE            0x48010000 /* LEDTS0 */
#define XMC4_CAN_BASE               0x48014000 /* MultiCAN */
#define XMC4_CAN_NODE0_BASE         0x48014200
#define XMC4_CAN_NODE1_BASE         0x48014300
#define XMC4_CAN_NODE2_BASE         0x48014400
#define XMC4_CAN_NODE3_BASE         0x48014500
#define XMC4_CAN_NODE4_BASE         0x48014600
#define XMC4_CAN_NODE5_BASE         0x48014700
#define XMC4_CAN_MO_BASE            0x48015000
#define XMC4_DAC_BASE               0x48018000 /* DAC */
#define XMC4_SDMMC_BASE             0x4801c000 /* SDMMC */
#define XMC4_USIC1_BASE             0x48020000 /* USIC1 */
#define XMC4_USIC1_CH0_BASE         0x48020000
#define XMC4_USIC1_CH1_BASE         0x48020200
#define XMC4_USIC1_RAM_BASE         0x48020400
#define XMC4_USIC2_BASE             0x48024000 /* USIC2 */
#define XMC4_USIC2_CH0_BASE         0x48024000
#define XMC4_USIC2_CH1_BASE         0x48024200
#define XMC4_USIC2_RAM_BASE         0x48024400
#define XMC4_PORT_BASE(n)           (0x48028000 + ((n) << 8))
#define XMC4_PORT0_BASE             0x48028000 /* PORTS */
#define XMC4_PORT1_BASE             0x48028100
#define XMC4_PORT2_BASE             0x48028200
#define XMC4_PORT3_BASE             0x48028300
#define XMC4_PORT4_BASE             0x48028400
#define XMC4_PORT5_BASE             0x48028500
#define XMC4_PORT6_BASE             0x48028600
#define XMC4_PORT7_BASE             0x48028700
#define XMC4_PORT8_BASE             0x48028800
#define XMC4_PORT9_BASE             0x48028900
#define XMC4_PORT14_BASE            0x48028e00
#define XMC4_PORT15_BASE            0x48028f00

#define XMC4_PBA2_BASE              0x50000000 /* PBA2 */
#define XMC4_SCU_GENERAL_BASE       0x50004000 /* SCU & RTC */
#define XMC4_ETH0_CON_BASE          0x50004040
#define XMC4_SCU_INTERRUPT_BASE     0x50004074
#define XMC4_SDMMC_CON_BASE         0x500040b4
#define XMC4_SCU_PARITY_BASE        0x5000413c
#define XMC4_SCU_TRAP_BASE          0x50004160
#define XMC4_SCU_POWER_BASE         0x50004200
#define XMC4_SCU_HIBERNATE_BASE     0x50004300
#define XMC4_SCU_RESET_BASE         0x50004400
#define XMC4_SCU_CLK_BASE           0x50004600
#define XMC4_SCU_OSC_BASE           0x50004700
#define XMC4_SCU_PLL_BASE           0x50004710
#define XMC4_ERU0_BASE              0x50004800
#define XMC4_DLR_BASE               0x50004900
#define XMC4_RTC_BASE               0x50004a00
#define XMC4_WDT_BASE               0x50008000 /* WDT */
#define XMC4_ETH0_BASE              0x5000c000 /* ETH */
#define XMC4_GPDMA0_CH0_BASE        0x50014000 /* GPDMA0 */
#define XMC4_GPDMA0_CH1_BASE        0x50014058
#define XMC4_GPDMA0_CH2_BASE        0x500140b0
#define XMC4_GPDMA0_CH3_BASE        0x50014108
#define XMC4_GPDMA0_CH4_BASE        0x50014160
#define XMC4_GPDMA0_CH5_BASE        0x500141b8
#define XMC4_GPDMA0_CH6_BASE        0x50014210
#define XMC4_GPDMA0_CH7_BASE        0x50014268
#define XMC4_GPDMA0_BASE            0x500142c0
#define XMC4_GPDMA1_CH0_BASE        0x50018000 /* GPDMA1 */
#define XMC4_GPDMA1_CH1_BASE        0x50018058
#define XMC4_GPDMA1_CH2_BASE        0x500180b0
#define XMC4_GPDMA1_CH3_BASE        0x50018108
#define XMC4_GPDMA1_BASE            0x500182c0
#define XMC4_FCE_BASE               0x50020000 /* FCE */
#define XMC4_FCE_KE0_BASE           0x50020020
#define XMC4_FCE_KE1_BASE           0x50020040
#define XMC4_FCE_KE2_BASE           0x50020060
#define XMC4_FCE_KE3_BASE           0x50020080
#define XMC4_USB0_BASE              0x50040000 /* USB0 */
#define XMC4_USB0_CH0_BASE          0x50040500
#define XMC4_USB0_CH1_BASE          0x50040520
#define XMC4_USB0_CH2_BASE          0x50040540
#define XMC4_USB0_CH3_BASE          0x50040560
#define XMC4_USB0_CH4_BASE          0x50040580
#define XMC4_USB0_CH5_BASE          0x500405a0
#define XMC4_USB0_CH6_BASE          0x500405c0
#define XMC4_USB0_CH7_BASE          0x500405e0
#define XMC4_USB0_CH8_BASE          0x50040600
#define XMC4_USB0_CH9_BASE          0x50040620
#define XMC4_USB0_CH10_BASE         0x50040640
#define XMC4_USB0_CH11_BASE         0x50040660
#define XMC4_USB0_CH12_BASE         0x50040680
#define XMC4_USB0_CH13_BASE         0x500406a0
#define XMC4_USB_EP_BASE            0x50040900
#define XMC4_USB0_EP1_BASE          0x50040920
#define XMC4_USB0_EP2_BASE          0x50040940
#define XMC4_USB0_EP3_BASE          0x50040960
#define XMC4_USB0_EP4_BASE          0x50040980
#define XMC4_USB0_EP5_BASE          0x500409a0
#define XMC4_USB0_EP6_BASE          0x500409c0
#define XMC4_ECAT0_BASE             0x50100000 /* ECAT0 */

#define XMC4_PMU0_BASE              0x58000000 /* PMU0 registers */
#define XMC4_FLASH0_BASE            0x58001000
#define XMC4_PREF_BASE              0x58004000 /* PMU0 prefetch */
#define XMC4_EBU_BASE               0x58008000 /* EBU registers */

#define XMC4_EBUMEM_CS0             0x60000000 /* EBU memory CS0 */
#define XMC4_EBUMEM_CS1             0x64000000 /* EBU memory CS1 */
#define XMC4_EBUMEM_CS2             0x68000000 /* EBU memory CS2 */
#define XMC4_EBUMEM_CS3             0x6c000000 /* EBU memory CS3 */

#define XMC4_EBUDEV_CS0             0xa0000000 /* EBU devices CS0 */
#define XMC4_EBUDEV_CS1             0xa4000000 /* EBU devices CS1 */
#define XMC4_EBUDEV_CS2             0xa8000000 /* EBU devices CS2 */
#define XMC4_EBUDEV_CS3             0xac000000 /* EBU devices CS3 */

#define XMC4_PPB_BASE               0xe000e000

#endif /* __ARCH_ARM_SRC_XMC4_CHIP_XMC4_MEMORYMAP_H */
