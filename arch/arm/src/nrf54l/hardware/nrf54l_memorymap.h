/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_memorymap.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_MEMORYMAP_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory */

#define NRF54L_RRAM_BASE   0x00000000
#define NRF54L_ICACHE_BASE 0xe0082000
#define NRF54L_SRAM_BASE   0x20000000

/* Peripheral bases */

#define NRF54L_FICR_BASE          0x00ffc000
#define NRF54L_UICR_BASE          0x00ffd000
#define NRF54L_SICR_BASE          0x00ffe000
#define NRF54L_USBHSCORE_BASE     0x50020000
#define NRF54L_SPU00_BASE         0x50040000
#define NRF54L_MPC00_BASE         0x50041000
#define NRF54L_DPPIC00_BASE       0x50042000
#define NRF54L_VPR00_BASE         0x5004c000
#define NRF54L_GPIOHSPADCTRL_BASE 0x50050400
#define NRF54L_GPIO_P2_BASE       0x50050400
#define NRF54L_CTRLAP_BASE        0x50052000
#define NRF54L_TAD_BASE           0x50053000
#define NRF54L_TIMER00_BASE       0x50055000
#define NRF54L_AXONS_BASE         0x50056000
#define NRF54L_EGU00_BASE         0x50058000
#define NRF54L_USBHS_BASE         0x5005a000
#define NRF54L_SPU10_BASE         0x50080000
#define NRF54L_DPPIC10_BASE       0x50082000
#define NRF54L_PPIB10_BASE        0x50083000
#define NRF54L_PPIB11_BASE        0x50084000
#define NRF54L_TIMER10_BASE       0x50085000
#define NRF54L_EGU10_BASE         0x50087000
#define NRF54L_RADIO_BASE         0x5008a000
#define NRF54L_SPU20_BASE         0x500c0000
#define NRF54L_DPPIC20_BASE       0x500c2000
#define NRF54L_PPIB20_BASE        0x500c3000
#define NRF54L_PPIB21_BASE        0x500c4000
#define NRF54L_PPIB22_BASE        0x500c5000
#define NRF54L_SPIM20_BASE        0x500c6000
#define NRF54L_SPIS20_BASE        0x500c6000
#define NRF54L_TWIM20_BASE        0x500c6000
#define NRF54L_TWIS20_BASE        0x500c6000
#define NRF54L_UART0_BASE         0x500c6000
#define NRF54L_SPIM21_BASE        0x500c7000
#define NRF54L_SPIS21_BASE        0x500c7000
#define NRF54L_TWIM21_BASE        0x500c7000
#define NRF54L_TWIS21_BASE        0x500c7000
#define NRF54L_UART1_BASE         0x500c7000
#define NRF54L_SPIM22_BASE        0x500c8000
#define NRF54L_SPIS22_BASE        0x500c8000
#define NRF54L_TWIM22_BASE        0x500c8000
#define NRF54L_TWIS22_BASE        0x500c8000
#define NRF54L_UART2_BASE         0x500c8000
#define NRF54L_EGU20_BASE         0x500c9000
#define NRF54L_TIMER20_BASE       0x500ca000
#define NRF54L_TIMER21_BASE       0x500cb000
#define NRF54L_TIMER22_BASE       0x500cc000
#define NRF54L_TIMER23_BASE       0x500cd000
#define NRF54L_TIMER24_BASE       0x500ce000
#define NRF54L_MEMCONF_BASE       0x500cf000
#define NRF54L_PDM20_BASE         0x500d0000
#define NRF54L_PDM21_BASE         0x500d1000
#define NRF54L_PWM20_BASE         0x500d2000
#define NRF54L_PWM21_BASE         0x500d3000
#define NRF54L_PWM22_BASE         0x500d4000
#define NRF54L_SAADC_BASE         0x500d5000
#define NRF54L_NFCT_BASE          0x500d6000
#define NRF54L_TEMP_BASE          0x500d7000
#define NRF54L_GPIO_P1_BASE       0x500d8200
#define NRF54L_GPIO_P3_BASE       0x500d8600
#define NRF54L_GPIOTE20_BASE      0x500da000
#define NRF54L_I2S20_BASE         0x500dd000
#define NRF54L_QDEC20_BASE        0x500e0000
#define NRF54L_QDEC21_BASE        0x500e1000
#define NRF54L_GRTC_BASE          0x500e2000
#define NRF54L_TDM_BASE           0x500e8000
#define NRF54L_SPIM23_BASE        0x500ed000
#define NRF54L_SPIS23_BASE        0x500ed000
#define NRF54L_TWIM23_BASE        0x500ed000
#define NRF54L_TWIS23_BASE        0x500ed000
#define NRF54L_UART5_BASE         0x500ed000
#define NRF54L_SPIM24_BASE        0x500ee000
#define NRF54L_SPIS24_BASE        0x500ee000
#define NRF54L_TWIM24_BASE        0x500ee000
#define NRF54L_TWIS24_BASE        0x500ee000
#define NRF54L_UART6_BASE         0x500ee000
#define NRF54L_SPU30_BASE         0x50100000
#define NRF54L_DPPIC30_BASE       0x50102000
#define NRF54L_PPIB30_BASE        0x50103000
#define NRF54L_SPIM30_BASE        0x50104000
#define NRF54L_SPIS30_BASE        0x50104000
#define NRF54L_TWIM30_BASE        0x50104000
#define NRF54L_TWIS30_BASE        0x50104000
#define NRF54L_UART3_BASE         0x50104000
#define NRF54L_COMP_BASE          0x50106000
#define NRF54L_LPCOMP_BASE        0x50106000
#define NRF54L_WDT30_BASE         0x50108000
#define NRF54L_WDT31_BASE         0x50109000
#define NRF54L_GPIO_P0_BASE       0x5010a000
#define NRF54L_GPIOTE30_BASE      0x5010c000
#define NRF54L_CLOCK_BASE         0x5010e000
#define NRF54L_POWER_BASE         0x5010e000
#define NRF54L_RESET_BASE         0x5010e000
#define NRF54L_OSCILLATORS_BASE   0x50120000
#define NRF54L_REGULATORS_BASE    0x50120000
#define NRF54L_VREGUSB_BASE       0x50121000

/* Peripheral bases that differ between L15 and LM20 */

#ifdef CONFIG_ARCH_CHIP_NRF54L15
#  define NRF54L_AAR00_BASE      0x50046000
#  define NRF54L_CCM00_BASE      0x50046000
#  define NRF54L_CRACEN_BASE     0x50048000
#  define NRF54L_CRACENCORE_BASE 0x51800000
#  define NRF54L_ECB00_BASE      0x50047000
#  define NRF54L_GLITCHDET_BASE  0x5004b000
#  define NRF54L_KMU_BASE        0x50045000
#  define NRF54L_PPIB00_BASE     0x50043000
#  define NRF54L_PPIB01_BASE     0x50044000
#  define NRF54L_RRAMC_BASE      0x5004b000
#  define NRF54L_SPIM00_BASE     0x5004a000
#  define NRF54L_SPIS00_BASE     0x5004a000
#  define NRF54L_TAMPC_BASE      0x500dc000
#  define NRF54L_UART4_BASE      0x5004a000
#else
#  define NRF54L_AAR00_BASE      0x5004a000
#  define NRF54L_CCM00_BASE      0x5004a000
#  define NRF54L_CRACEN_BASE     0x50059000
#  define NRF54L_CRACENCORE_BASE 0x50010000
#  define NRF54L_ECB00_BASE      0x5004b000
#  define NRF54L_GLITCHDET_BASE  0x5004e000
#  define NRF54L_KMU_BASE        0x50049000
#  define NRF54L_PPIB00_BASE     0x50044000
#  define NRF54L_PPIB01_BASE     0x50045000
#  define NRF54L_RRAMC_BASE      0x5004e000
#  define NRF54L_SPIM00_BASE     0x5004d000
#  define NRF54L_SPIS00_BASE     0x5004d000
#  define NRF54L_TAMPC_BASE      0x500ef000
#  define NRF54L_UART4_BASE      0x5004d000
#endif

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_MEMORYMAP_H */
