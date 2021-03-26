/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_memorymap.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_MEMORYMAP_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define S32K1XX_AIPS_LITE_BASE   0x40000000  /* Peripheral bridge (AIPS-Lite) */
#  define S32K1XX_FLASHCFG_BASE  0x40000400  /* FLASH Configuration bytes */
#  define S32K1XX_MSCM_BASE      0x40001000  /* MSCM */
#  define S32K1XX_DMAC_BASE      0x40008000  /* DMA controller */
#  define S32K1XX_DMADESC_BASE   0x40008000  /* DMA transfer control descriptors */
#  define S32K1XX_MPU_BASE       0x4000d000  /* MPU */
#  define S32K1XX_GPIOCTL_BASE   0x4000f000  /* GPIO controller */
#  define S32K1XX_GPIOALIAS_BASE 0x400ff000  /* GPIO controller (alias) */
#  define S32K1XX_ERM_BASE       0x40018000  /* ERM */
#  define S32K1XX_EIM_BASE       0x40019000  /* EIM */
#define S32K1XX_FTFC_BASE        0x40020000  /* Flash memory */
#define S32K1XX_DMAMUX_BASE      0x40021000  /* DMA Channel Multiplexer  */
#define S32K1XX_FLEXCAN0_BASE    0x40024000  /* FlexCAN 0 */
#define S32K1XX_FLEXCAN1_BASE    0x40025000  /* FlexCAN 1 */
#define S32K1XX_FTM3_BASE        0x40026000  /* FlexTimer 3 */
#define S32K1XX_ADC1_BASE        0x40027000  /* Analog-to-digital converter 1 */
#define S32K1XX_FLEXCAN2_BASE    0x4002b000  /* FlexCAN 2 */
#define S32K1XX_LPSPI0_BASE      0x4002c000  /* Low Power SPI 0 */
#define S32K1XX_LPSPI1_BASE      0x4002d000  /* Low Power SPI 1 */
#define S32K1XX_LPSPI2_BASE      0x4002e000  /* Low Power SPI 2 */
#define S32K1XX_PDB1_BASE        0x40031000  /* Programmable delay block 1 */
#define S32K1XX_CRC_BASE         0x40032000  /* CRC */
#define S32K1XX_LPIT0_BASE       0x40037000  /* Low power periodic interrupt timer */
#define S32K1XX_FTM0_BASE        0x40038000  /* FlexTimer 0 */
#define S32K1XX_FTM1_BASE        0x40039000  /* FlexTimer 1 */
#define S32K1XX_FTM2_BASE        0x4003a000  /* FlexTimer 2 */
#define S32K1XX_ADC0_BASE        0x4003b000  /* Analog-to-digital converter 0 */
#define S32K1XX_RTC_BASE         0x4003d000  /* Real-time counter */
#define S32K1XX_CMU0_BASE        0x4003e000  /* Clock Monitor Unit 0 */
#define S32K1XX_CMU1_BASE        0x4003f000  /* Clock Monitor Unit 1 */
#define S32K1XX_LPTMR0_BASE      0x40040000  /* Low-power timer 0 */
#define S32K1XX_SIM_BASE         0x40048000  /* System integration module */

#define S32K1XX_PORT_BASE(n)     (0x40049000 + ((n) << 12)) /* Port n multiplexing control */
#  define S32K1XX_PORTA_BASE     0x40049000                 /* Port A multiplexing control */
#  define S32K1XX_PORTB_BASE     0x4004a000                 /* Port B multiplexing control */
#  define S32K1XX_PORTC_BASE     0x4004b000                 /* Port C multiplexing control */
#  define S32K1XX_PORTD_BASE     0x4004c000                 /* Port D multiplexing control */
#  define S32K1XX_PORTE_BASE     0x4004d000                 /* Port E multiplexing control */

#define S32K1XX_WDOG_BASE        0x40052000  /* Software watchdog */
#define S32K1XX_SAI0_BASE        0x40054000  /* Synchronous Audio Interface 0 */
#define S32K1XX_SAI1_BASE        0x40055000  /* Synchronous Audio Interface 1 */
#define S32K1XX_FLEXIO_BASE      0x4005a000  /* Flexible IO */
#define S32K1XX_EWM_BASE         0x40061000  /* External watchdog  */
#define S32K1XX_TRGMUX_BASE      0x40063000  /* Trigger Multiplexing Control  */
#define S32K1XX_SCG_BASE         0x40064000  /* System Clock Generator  */
#define S32K1XX_PCC_BASE         0x40065000  /* Peripheral Clock Control */
#define S32K1XX_LPI2C0_BASE      0x40066000  /* Low Power I2C 0 */
#define S32K1XX_LPI2C1_BASE      0x40067000  /* Low Power I2C 1 */
#define S32K1XX_LPUART0_BASE     0x4006a000  /* Low Power UART 0 */
#define S32K1XX_LPUART1_BASE     0x4006b000  /* Low Power UART 1 */
#define S32K1XX_LPUART2_BASE     0x4006c000  /* Low Power UART 2 */
#define S32K1XX_FTM4_BASE        0x4006e000  /* FlexTimer 4 */
#define S32K1XX_FTM5_BASE        0x4006f000  /* FlexTimer 5 */
#define S32K1XX_FTM6_BASE        0x40070000  /* FlexTimer 6 */
#define S32K1XX_FTM7_BASE        0x40071000  /* FlexTimer 7 */
#define S32K1XX_CMP0_BASE        0x40073000  /* Analog comparator 0 */
#define S32K1XX_QUADSPI_BASE     0x40076000  /* QuadSPI */
#define S32K1XX_ENET_BASE        0x40079000  /* Ethernet */
#define S32K1XX_PMC_BASE         0x4007d000  /* Power management controller */
#define S32K1XX_SMC_BASE         0x4007e000  /* System Mode controller */
#define S32K1XX_RCM_BASE         0x4007f000  /* Reset Control Module */

#define S32K1XX_GPIO_BASE(n)     (0x400ff000 +((n) << 6)) /* GPIO controller */
#  define S32K1XX_GPIOA_BASE     0x400ff000               /* GPIOA controller */
#  define S32K1XX_GPIOB_BASE     0x400ff040               /* GPIOB controller */
#  define S32K1XX_GPIOC_BASE     0x400ff080               /* GPIOC controller */
#  define S32K1XX_GPIOD_BASE     0x400ff0c0               /* GPIOD controller */
#  define S32K1XX_GPIOE_BASE     0x400ff100               /* GPIOE controller */

#if defined(CONFIG_ARCH_CHIP_S32K14X)
#  define S32K1XX_ITM_BASE       0xe0000000  /* Instrumentation Trace Macrocell */
#  define S32K1XX_DWT_BASE       0xe0001000  /* Data Watchpoint and Trace */
#  define S32K1XX_FPB_BASE       0xe0002000  /* Flash Patch and Breakpoint */
#  define S32K1XX_SCS_BASE       0xe000e000  /* System Control Space */
#  define S32K1XX_TPIU_BASE      0xe0040000  /* Trace Port Interface Unit */
#  define S32K1XX_MCM_BASE       0xe0080000  /* Miscellaneous Control Module */
#  define S32K1XX_LMEM_BASE      0xe0082000  /* Cache Controller */
#  define S32K1XX_ROMTABLE1_BASE 0xe00ff000  /* Arm Core ROM Table */
#elif defined(CONFIG_ARCH_CHIP_S32K11X)
#  define S32K1XX_DWT_BASE       0xe0001000  /* Data Watchpoint and Trace */
#  define S32K1XX_ROMTABLE_BASE  0xe0002000  /* Arm Core ROM Table */
#  define S32K1XX_SCS_BASE       0xe000e000  /* System Control Space */
#endif

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_MEMORYMAP_H */
