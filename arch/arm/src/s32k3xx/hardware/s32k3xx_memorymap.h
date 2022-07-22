/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_memorymap.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MEMORYMAP_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AIPS-Lite Peripheral Bridges */

#define S32K3XX_AIPS0_BASE       (0x40000000) /* AIPS-Lite Peripheral Bridge 0 */
#define S32K3XX_AIPS1_BASE       (0x40200000) /* AIPS-Lite Peripheral Bridge 1 */
#define S32K3XX_AIPS2_BASE       (0x40400000) /* AIPS-Lite Peripheral Bridge 2 */

/* Peripheral Bridge 0 (AIPS0) **********************************************/

#define S32K3XX_TRGMUX_BASE      (0x40080000) /* Trigger Multiplexer */
#define S32K3XX_BCTU_BASE        (0x40084000) /* Body Cross-Triggering Unit */
#define S32K3XX_EMIOS0_BASE      (0x40088000) /* Enhanced Modular I/O Subsystem 0 */
#define S32K3XX_EMIOS1_BASE      (0x4008c000) /* Enhanced Modular I/O Subsystem 1 */
#define S32K3XX_EMIOS2_BASE      (0x40090000) /* Enhanced Modular I/O Subsystem 2 */

                               /* 0x40094000-0x40097fff Reserved */

#define S32K3XX_LCU0_BASE        (0x40098000) /* Logic Control Unit 0 */
#define S32K3XX_LCU1_BASE        (0x4009c000) /* Logic Control Unit 1 */
#define S32K3XX_ADC0_BASE        (0x400a0000) /* Analog-to-Digital Converter 0 */
#define S32K3XX_ADC1_BASE        (0x400a4000) /* Analog-to-Digital Converter 1 */
#define S32K3XX_ADC2_BASE        (0x400a8000) /* Analog-to-Digital Converter 2 */

                               /* 0x400ac000-0x400afff Reserved */

#define S32K3XX_PIT0_BASE        (0x400b0000) /* Periodic Interrupt Timer 0 */
#define S32K3XX_PIT1_BASE        (0x400b4000) /* Periodic Interrupt Timer 1 */
#define S32K3XX_MU2_MUA_BASE     (0x400b8000) /* Messaging Unit 2, Interface A */
#define S32K3XX_MU2_MUB_BASE     (0x400bc000) /* Messaging Unit 2, Interface B */
#define S32K3XX_I3C_BASE         (0x400c0000) /* Improved Inter-Integrated Circuit */

/* Peripheral Bridge 1 (AIPS1) **********************************************/

#define S32K3XX_AXBS_BASE        (0x40200000) /* Crossbar Switch */
#define S32K3XX_XBIC0_BASE       (0x40204000) /* Crossbar Integrity Checker 0 */
#define S32K3XX_XBIC1_BASE       (0x40208000) /* Crossbar Integrity Checker 1 */
#define S32K3XX_EDMA_BASE        (0x4020c000) /* Enhanced Direct Memory Access */
#define S32K3XX_EDMA_TCD_BASE    (0x40210000) /* eDMA Transfer Control Descriptor */

                               /* 0x40240000-0x40257fff Debug */

#define S32K3XX_EIM_BASE         (0x40258000) /* Error Injection Module */
#define S32K3XX_ERM_BASE         (0x4025c000) /* Error Reporting Module */
#define S32K3XX_MSCM_BASE        (0x40260000) /* Miscellaneous System Control Module */
#define S32K3XX_PRAMC0_BASE      (0x40264000) /* Platform RAM Controller 0 */
#define S32K3XX_PFLASH_BASE      (0x40268000) /* Platform Flash Memory Controller */

                               /* 0x4026c000-0x4026ffff PFLASH Alt. */

#define S32K3XX_SWT0_BASE        (0x40270000) /* Software Watchdog Timer 0 */
#define S32K3XX_STM0_BASE        (0x40274000) /* System Timer Module 0 */
#define S32K3XX_XRDC_BASE        (0x40278000) /* Extended Resource Domain Controller */
#define S32K3XX_INTM_BASE        (0x4027c000) /* Interrupt Monitor */
#define S32K3XX_DMAMUX0_BASE     (0x40280000) /* Direct Memory Access Multiplexer 0 */
#define S32K3XX_DMAMUX1_BASE     (0x40284000) /* Direct Memory Access Multiplexer 1 */
#define S32K3XX_RTC_BASE         (0x40288000) /* Real Time Clock */
#define S32K3XX_MC_RGM_BASE      (0x4028c000) /* Reset Generation Module */
#define S32K3XX_SIUL2_BASE       (0x40290000) /* System Integration Unit Lite */

                               /* 0x40294000-0x402a7fff SIUL2 */

#define S32K3XX_VIRTWRAPPER_BASE (0x402a8000) /* Virtualization Wrapper */
#define S32K3XX_DCM_BASE         (0x402ac000) /* Device Configuration Module */

                               /* 0x402b0000-0x402b3fff Reserved */

#define S32K3XX_WKPU_BASE        (0x402b4000) /* Wakeup Unit */

                               /* 0x402b8000-0x402bbfff Reserved */

#define S32K3XX_CMU0_BASE        (0x402bc000) /* Clock Monitoring Unit 0 */
#define S32K3XX_CMU1_BASE        (0x402bc020) /* Clock Monitoring Unit 1 */
#define S32K3XX_CMU2_BASE        (0x402bc040) /* Clock Monitoring Unit 2 */
#define S32K3XX_CMU3_BASE        (0x402bc060) /* Clock Monitoring Unit 3 */
#define S32K3XX_CMU4_BASE        (0x402bc080) /* Clock Monitoring Unit 4 */
#define S32K3XX_CMU5_BASE        (0x402bc0a0) /* Clock Monitoring Unit 5 */

                               /* 0x402c0000-0x402c3fff Reserved */

#define S32K3XX_TSPC_BASE        (0x402c4000) /* Touch Sensing Pin Coupling */
#define S32K3XX_SIRC_BASE        (0x402c8000) /* Slow Internal RC Oscillator */
#define S32K3XX_SXOSC_BASE       (0x402cc000) /* Slow Crystal Oscillator */
#define S32K3XX_FIRC_BASE        (0x402d0000) /* Fast Internal RC Oscillator */
#define S32K3XX_FXOSC_BASE       (0x402d4000) /* Fast Crystal Oscillator */
#define S32K3XX_MC_CGM_BASE      (0x402d8000) /* Clock Generation Module */
#define S32K3XX_MC_ME_BASE       (0x402dc000) /* Mode Entry Module */
#define S32K3XX_PLL_BASE         (0x402e0000) /* Phase-Locked Loop */

                               /* 0x402e4000-0x402e7fff Reserved */

#define S32K3XX_PMC_BASE         (0x402e8000) /* Power Management Controller */
#define S32K3XX_FMU_BASE         (0x402ec000) /* Flash Management Unit */

                               /* 0x402f0000-0x402f3fff FMU Alt. */

                               /* 0x402f4000-0x402fbfff Reserved */

#define S32K3XX_PIT2_BASE        (0x402fc000) /* Periodic Interrupt Timer 2 */

                               /* 0x40300000-0x40303fff Reserved  */

#define S32K3XX_FLEXCAN0_BASE    (0x40304000) /* FlexCAN 0 */
#define S32K3XX_FLEXCAN1_BASE    (0x40308000) /* FlexCAN 1 */
#define S32K3XX_FLEXCAN2_BASE    (0x4030c000) /* FlexCAN 2 */
#define S32K3XX_FLEXCAN3_BASE    (0x40310000) /* FlexCAN 3 */
#define S32K3XX_FLEXCAN4_BASE    (0x40314000) /* FlexCAN 4 */
#define S32K3XX_FLEXCAN5_BASE    (0x40318000) /* FlexCAN 5 */

                               /* 0x4031c000-0x40323fff Reserved */

#define S32K3XX_FLEXIO_BASE      (0x40324000) /* Flexible I/O */
#define S32K3XX_LPUART0_BASE     (0x40328000) /* Low Power Universal Asynchronous Receiver/Transmitter 0 */
#define S32K3XX_LPUART1_BASE     (0x4032c000) /* Low Power Universal Asynchronous Receiver/Transmitter 1 */
#define S32K3XX_LPUART2_BASE     (0x40330000) /* Low Power Universal Asynchronous Receiver/Transmitter 2 */
#define S32K3XX_LPUART3_BASE     (0x40334000) /* Low Power Universal Asynchronous Receiver/Transmitter 3 */
#define S32K3XX_LPUART4_BASE     (0x40338000) /* Low Power Universal Asynchronous Receiver/Transmitter 4 */
#define S32K3XX_LPUART5_BASE     (0x4033c000) /* Low Power Universal Asynchronous Receiver/Transmitter 5 */
#define S32K3XX_LPUART6_BASE     (0x40340000) /* Low Power Universal Asynchronous Receiver/Transmitter 6 */
#define S32K3XX_LPUART7_BASE     (0x40344000) /* Low Power Universal Asynchronous Receiver/Transmitter 7 */

                               /* 0x40348000-0x4034ffff Reserved */

#define S32K3XX_LPI2C0_BASE      (0x40350000) /* Low Power Inter-Integrated Circuit 0 */
#define S32K3XX_LPI2C1_BASE      (0x40354000) /* Low Power Inter-Integrated Circuit 1 */
#define S32K3XX_LPSPI0_BASE      (0x40358000) /* Low Power Serial Peripheral Interface 0 */
#define S32K3XX_LPSPI1_BASE      (0x4035c000) /* Low Power Serial Peripheral Interface 1 */
#define S32K3XX_LPSPI2_BASE      (0x40360000) /* Low Power Serial Peripheral Interface 2 */
#define S32K3XX_LPSPI3_BASE      (0x40364000) /* Low Power Serial Peripheral Interface 3 */

                               /* 0x40368000-0x4036bfff Reserved */

#define S32K3XX_SAI0_BASE        (0x4036c000) /* Synchronous Audio Interface 0 */
#define S32K3XX_LPCMP0_BASE      (0x40370000) /* Low Power Comparator 0 */
#define S32K3XX_LPCMP1_BASE      (0x40374000) /* Low Power Comparator 1 */

                               /* 0x40378000-0x4037bfff Reserved */

#define S32K3XX_TMU_BASE         (0x4037c000) /* Temperature Sensor Unit */
#define S32K3XX_CRC_BASE         (0x40380000) /* Cyclic Redundancy Check */
#define S32K3XX_FCCU_BASE        (0x40384000) /* Fault Collection and Control Unit */

                               /* 0x40388000-0x4038bfff Self-test */

#define S32K3XX_MU0_MUB_BASE     (0x4038c000) /* Messaging Unit 0, Interface B */

#if defined(CONFIG_ARCH_CHIP_S32K312) || defined(CONFIG_ARCH_CHIP_S32K311)
#  define S32K3XX_MU1_MUB_BASE   (0x40390000) /* Messaging Unit 1, Interface B (S32K312/S32K311) */
#endif

#define S32K3XX_JDC_BASE         (0x40394000) /* JTAG Data Communication */
#define S32K3XX_HSE_BASE         (0x4039c000) /* HSE_B Configuration_GPR */

/* Peripheral Bridge 2 (AIPS2) **********************************************/

#define S32K3XX_XBIC2_BASE       (0x40400000) /* Crossbar Integrity Checker 2 */
#define S32K3XX_XBIC3_BASE       (0x40404000) /* Crossbar Integrity Checker 3 */

                               /* 0x40408000-0x4040ffff Reserved */

                               /* 0x40410000-0x4045ffff EDMA TCD */

#define S32K3XX_SEMA42_BASE      (0x40460000) /* Semaphores */
#define S32K3XX_PRAMC1_BASE      (0x40464000) /* Platform RAM Controller 1 */

                               /* 0x40468000-0x4046bfff Reserved */

#define S32K3XX_SWT1_BASE        (0x4046c000) /* Software Watchdog Timer 1 */

                               /* 0x40470000-0x40473fff Reserved */

#define S32K3XX_STM1_BASE        (0x40474000) /* System Timer Module 1 */

                               /* 0x40478000-0x4047ffff Reserved */

#define S32K3XX_EMAC_BASE        (0x40480000) /* Ethernet Media Access Controller */

                               /* 0x40484000-0x4048bfff Reserved */

#define S32K3XX_LPUART8_BASE     (0x4048c000) /* Low Power Universal Asynchronous Receiver/Transmitter 8 */
#define S32K3XX_LPUART9_BASE     (0x40490000) /* Low Power Universal Asynchronous Receiver/Transmitter 9 */
#define S32K3XX_LPUART10_BASE    (0x40494000) /* Low Power Universal Asynchronous Receiver/Transmitter 10 */
#define S32K3XX_LPUART11_BASE    (0x40498000) /* Low Power Universal Asynchronous Receiver/Transmitter 11 */
#define S32K3XX_LPUART12_BASE    (0x4049c000) /* Low Power Universal Asynchronous Receiver/Transmitter 12 */
#define S32K3XX_LPUART13_BASE    (0x404a0000) /* Low Power Universal Asynchronous Receiver/Transmitter 13 */
#define S32K3XX_LPUART14_BASE    (0x404a4000) /* Low Power Universal Asynchronous Receiver/Transmitter 14 */
#define S32K3XX_LPUART15_BASE    (0x404a8000) /* Low Power Universal Asynchronous Receiver/Transmitter 15 */

                               /* 0x404ac000-0x404bbfff Reserved */

#define S32K3XX_LPSPI4_BASE      (0x404bc000) /* Low Power Serial Peripheral Interface 4 */
#define S32K3XX_LPSPI5_BASE      (0x404c0000) /* Low Power Serial Peripheral Interface 5 */

                               /* 0x404c4000-0x404cbfff Reserved */

#define S32K3XX_QSPI_BASE        (0x404cc000) /* Quad Serial Peripheral Interface */

                               /* 0x404d0000-0x404dbfff Reserved */

#define S32K3XX_SAI1_BASE        (0x404dc000) /* Synchronous Audio Interface 1 */

                               /* 0x404e0000-0x404e7fff Reserved */

#define S32K3XX_LPCMP2_BASE      (0x404e8000) /* Low Power Comparator 2 */

#if !defined(CONFIG_ARCH_CHIP_S32K312) && !defined(CONFIG_ARCH_CHIP_S32K311)
#  define S32K3XX_MU1_MUB_BASE   (0x404ec000) /* Messaging Unit 1, Interface B (S32K344/S32K342/S32K324/S32K322/S32K314) */
#endif

/* Private Peripheral Bus (PPB) *********************************************/

#define S32K3XX_ITM_BASE         (0xe0000000) /* Instrumentation Trace Macrocell */
#define S32K3XX_DWT_BASE         (0xe0001000) /* Data Watchpoint and Trace */
#define S32K3XX_FPB_BASE         (0xe0002000) /* Flash Patch and Breakpoint */
#define S32K3XX_SCS_BASE         (0xe000e000) /* System Control Space */
#define S32K3XX_TPIU_BASE        (0xe0040000) /* Trace Port Interface Unit */
#define S32K3XX_ETM_BASE         (0xe0041000) /* Embedded Trace Macrocell */
#define S32K3XX_CTI_BASE         (0xe0042000) /* Cross Trigger Interface */
#define S32K3XX_MCM_BASE         (0xe0080000) /* Miscellaneous Control Module */
#define S32K3XX_ROMTABLE_BASE    (0xe00ff000) /* Cortex-M7 PPB ROM Table */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MEMORYMAP_H */
