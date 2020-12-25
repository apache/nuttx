/******************************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_k28memorymap.h
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
 ******************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_K28MEMORYMAP_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_K28MEMORYMAP_H

/******************************************************************************************
 * Included Files
 ******************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef KINETIS_K28

/******************************************************************************************
 * Pre-processor Definitions
 ******************************************************************************************/

/* Memory Map *****************************************************************************/

/* K28 Family
 *
 * The memory map for the following parts is defined in NXP document
 * K28P210M150SF5RM Rev. 4, August 2017
 */

#if defined(CONFIG_ARCH_CHIP_MK28FN2M0VMI15) || defined(CONFIG_ARCH_CHIP_MK28FN2M0CAU15R)

#  define KINETIS_FLASH_BASE         0x00000000 /* Program flash and read-only data
                                                 * (includes exception vectors in
                                                 * first 1024 bytes) */
#  define KINETIS_QSPI0ALIAS_BASE    0x04000000 /* QSPI0 (Aliased area) Mapped to the
                                                 * same access space of 0x68000000 to
                                                 * 0x6bffffff */
#  define KINETIS_SDRAMALIAS_BASE    0x08000000 /* SDRAM (Aliased area) Mapped to the
                                                 * same access space of 0x88000000 to
                                                 * 0x8fffffff */
#  define KINETIS_PRGACCLRAM_BASE    0x14000000 /* FlexRAM (Flash Programming
                                                 * Acceleration RAM) */
#  define KINETIS_FBALIAS_BASE       0x18000000 /* FlexBus (Aliased Area) Mapped to the
                                                 * same access space of 0x98000000 to
                                                 * 0x9bffffff */
#  define KINETIS_ROM_BASE           0x1c000000 /* ROM */
#  define KINETIS_SRAML_BASE         0x1ffc0000 /* Tightly Coupled Memory Lower (TCML)
                                                 * SRAM_L: Lower SRAM (ICODE/DCODE) */
#  define KINETIS_SRAMU_BASE         0x20000000 /* Tightly Coupled Memory Upper (TCMU)
                                                 * SRAM_U: Upper SRAM bitband region */
#  define KINETIS_SALIAS_BASE        0x22000000 /* Aliased to TCMU SRAM bitband */
#  define KINETIS_FDATALIAS_BASE     0x30000000 /* Flash Data Alias */
#  define KINETIS_OCRAM_BASE         0x34000000 /* OCRAM */
#  define KINETIS_BRIDGE0_BASE       0x40000000 /* Bitband region for AIPS0 */
#  define KINETIS_BRIDGE1_BASE       0x40080000 /* Bitband region for AIPS1 */
#  define KINETIS_GPIOBB_BASE        0x400ff000 /* Bitband region for GPIO */
#  define KINETIS_AGALIAS_BASE       0x42000000 /* Aliased to AIPS and GPIO bitband */
#  define KINETIS_BME2_BASE          0x44000000 /* Bit Manipulation Engine (BME2) access
                                                 * to AIPS0 peripheral slots 0-127 and
                                                 * AIPS1 peripheral slots 0-123 */
#  define KINETIS_FLEXBUS_BASE       0x60000000 /* FlexBus
                                                 * (External Memory - Write-back) */
#  define KINETIS_QSPIRX_BASE        0x67000000 /* QuadSPI0 Rx Buffer */
#  define KINETIS_QSPI0_BASE         0x68000000 /* QuadSPI0 (External Memory) */
#  define KINETIS_SDRAMWB_BASE       0x70000000 /* SDRAM
                                                 * (External Memory - Write-back) */
#  define KINETIS_SDRAMWT1_BASE      0x80000000 /* SDRAM
                                                 * (External Memory - Write-through) */
#  define KINETIS_SDRAMWT2_BASE      0x88000000 /* SDRAM
                                                 * (External Memory - Write-through) */
#  define KINETIS_FLEXBUSWT_BASE     0x98000000 /* FlexBus
                                                 * (External Memory - Write-through) */
#  define KINETIS_FLEXBUSEP_BASE     0xa0000000 /* FlexBus
                                                 * External Peripheral - Not executable) */
#  define KINETIS_PERIPH_BASE        0xe0000000 /* Private peripherals */

/* Peripheral Bridge 0 Memory Map *********************************************************/

#  define KINETIS_AIPS0_BASE         0x40000000 /* Peripheral bridge 0 (AIPS-Lite 0) */
#  define KINETIS_XBAR_BASE          0x40004000 /* Crossbar switch */
#  define KINETIS_DMAC_BASE          0x40008000 /* DMA controller */
#  define KINETIS_DMADESC_BASE       0x40009000 /* DMA controller  transfer control
                                                 * descriptors */
#  define KINETIS_FLEXBUSC_BASE      0x4000c000 /* FlexBus controller */
#  define KINETIS_MPU_BASE           0x4000d000 /* System MPU */
#  define KINETIS_SDRAMC_BASE        0x4000f000 /* SDRAMC */
#  define KINETIS_FMC_BASE           0x4001f000 /* Flash memory controller */
#  define KINETIS_FTFE_BASE          0x40020000 /* Flash memory */
#  define KINETIS_DMAMUX0_BASE       0x40021000 /* DMA channel multiplexer 0 */
#  define KINETIS_SPI0_BASE          0x4002c000 /* DSPI 0 */
#  define KINETIS_SPI1_BASE          0x4002d000 /* DSPI 1 */
#  define KINETIS_SAI0_BASE          0x4002f000 /* SAI 0 (12S/SSI) */
#  define KINETIS_CRC_BASE           0x40032000 /* CRC */
#  define KINETIS_USBDCD_BASE        0x40035000 /* USB DCD */
#  define KINETIS_PDB0_BASE          0x40036000 /* Programmable delay block */
#  define KINETIS_PIT_BASE           0x40037000 /* Periodic interrupt timers (PIT) */
#  define KINETIS_FTM0_BASE          0x40038000 /* FlexTimer (FTM) 0 */
#  define KINETIS_FTM1_BASE          0x40039000 /* FlexTimer (FTM) 1 */
#  define KINETIS_FTM2_BASE          0x4003a000 /* FlexTimer (FTM) 2 */
#  define KINETIS_ADC0_BASE          0x4003b000 /* Analog-to-digital converter (ADC) 0 */
#  define KINETIS_RTC_BASE           0x4003d000 /* Real time clock */
#  define KINETIS_VBATR_BASE         0x4003e000 /* VBAT register file */
#  define KINETIS_DAC0_BASE          0x4003f000 /* DAC0 */
#  define KINETIS_LPTMR0_BASE        0x40040000 /* Low power timer 0 (LPTMR0) */
#  define KINETIS_SYSR_BASE          0x40041000 /* System register file */
#  define KINETIS_LPTMR1_BASE        0x40044000 /* Low power timer 1 (LPTMR1) */
#  define KINETIS_SIMLP_BASE         0x40047000 /* SIM low-power logic */
#  define KINETIS_SIM_BASE           0x40048000 /* System integration module (SIM) */

#  define KINETIS_PORT_BASE(n)       (0x40049000 + ((n) << 12))
#  define KINETIS_PORTA_BASE         0x40049000 /* Port A multiplexing control */
#  define KINETIS_PORTB_BASE         0x4004a000 /* Port B multiplexing control */
#  define KINETIS_PORTC_BASE         0x4004b000 /* Port C multiplexing control */
#  define KINETIS_PORTD_BASE         0x4004c000 /* Port D multiplexing control */
#  define KINETIS_PORTE_BASE         0x4004d000 /* Port E multiplexing control */

#  define KINETIS_WDOG_BASE          0x40052000 /* Software watchdog */
#  define KINETIS_EWM_BASE           0x40061000 /* External watchdog */
#  define KINETIS_CMT_BASE           0x40062000 /* Carrier modulator timer (CMT) */
#  define KINETIS_MCG_BASE           0x40064000 /* Multi-purpose Clock Generator (MCG) */
#  define KINETIS_OSC_BASE           0x40065000 /* System oscillator (OSC) */
#  define KINETIS_I2C0_BASE          0x40066000 /* I2C 0 */
#  define KINETIS_I2C1_BASE          0x40067000 /* I2C 1 */
#  define KINETIS_USB0_BASE          0x40072000 /* USB OTG FS/LS */
#  define KINETIS_2C2D_BASE          0x40073000 /* 2C2D (Analog comparator (CMP) /
                                                 * 6-bit digital-to-analog
                                                 * converter (DAC)) */
#  define KINETIS_VREF_BASE          0x40074000 /* Voltage reference (VREF) */
#  define KINETIS_LLWU_BASE          0x4007c000 /* Low-leakage wakeup unit (LLWU) */
#  define KINETIS_PMC_BASE           0x4007d000 /* Power management controller (PMC) */
#  define KINETIS_SMC_BASE           0x4007e000 /* System Mode controller (SMC) */
#  define KINETIS_RCM_BASE           0x4007f000 /* Reset Control Module (RCM) */

/* Peripheral Bridge 1 Memory Map *********************************************************/

#  define KINETIS_AIPS1_BASE         0x40080000 /* Peripheral bridge 1 (AIPS-Lite 1) */
#  define KINETIS_RNGA_BASE          0x400a0000 /* True Random Number Generator (TRNG) */
#  define KINETIS_USBHS_BASE         0x400a1000 /* USB OTG HS/FS/LS */
#  define KINETIS_USBHSPHY_BASE      0x400a2000 /* USBHS PHY */
#  define KINETIS_USBHSDCD_BASE      0x400a3000 /* USBHS DCD */
#  define KINETIS_SPI2_BASE          0x400ac000 /* SPI 2 */
#  define KINETIS_SPI3_BASE          0x400ad000 /* SPI 3 */
#  define KINETIS_SAI1_BASE          0x400af000 /* SAI 1 (12S/SSI) */
#  define KINETIS_SDHC_BASE          0x400b1000 /* eSDHC */
#  define KINETIS_FTM2_ALT_BASE      0x400b8000 /* Alternate address FlexTimer 2 */
#  define KINETIS_FTM3_BASE          0x400b9000 /* FlexTimer 3 */
#  define KINETIS_LPUART0_BASE       0x400c4000 /* LPUART0 */
#  define KINETIS_LPUART1_BASE       0x400c5000 /* LPUART1 */
#  define KINETIS_LPUART2_BASE       0x400c6000 /* LPUART2 */
#  define KINETIS_LPUART3_BASE       0x400c7000 /* LPUART3 */
#  define KINETIS_TPM1_BASE          0x400c9000 /* TPM1 */
#  define KINETIS_TPM2_BASE          0x400ca000 /* TPM2 */
#  define KINETIS_DAC0_ALT_BASE      0x400cc000 /* Alternate address 12-bit
                                                 * digital-to-analog converter (DAC) 0 */
#  define KINETIS_LPUART4_BASE       0x400d6000 /* LPUART4 */
#  define KINETIS_QSPI0C_BASE        0x400da000 /* QSPI0 controller */
#  define KINETIS_FLEXIO0_BASE       0x400df000 /* FlexIO0 */
#  define KINETIS_I2C2_BASE          0x400e6000 /* I2C 2 */
#  define KINETIS_I2C3_BASE          0x400e7000 /* I2C 3 */
#  define KINETIS_XBARSS_BASE        0x400ff000 /* Not an AIPS-Lite slot. The 32-bit
                                                 * general purpose input/output module
                                                 * that shares the crossbar switch slave
                                                 * port with the AIPS-Lite is accessed at
                                                 * this address. */
#  define KINETIS_GPIO_BASE(n)       (0x400ff000 + ((n) << 6))
#  define KINETIS_GPIOA_BASE         0x400ff000 /* GPIO PORTA registers */
#  define KINETIS_GPIOB_BASE         0x400ff040 /* GPIO PORTB registers */
#  define KINETIS_GPIOC_BASE         0x400ff080 /* GPIO PORTC registers */
#  define KINETIS_GPIOD_BASE         0x400ff0c0 /* GPIO PORTD registers */
#  define KINETIS_GPIOE_BASE         0x400ff100 /* GPIO PORTE registers */

/* Private Peripheral Bus (PPB) Memory Map ************************************************/

#  define KINETIS_ITM_BASE           0xe0000000 /* Instrumentation Trace Macrocell (ITM) */
#  define KINETIS_DWT_BASE           0xe0001000 /* Data Watchpoint and Trace (DWT) */
#  define KINETIS_FPB_BASE           0xe0002000 /* Flash Patch and Breakpoint (FPB) */
#  define KINETIS_SCS_BASE           0xe000e000 /* System Control Space (SCS)
                                                 * (for NVIC and FPU) */
#  define KINETIS_TPIU_BASE          0xe0040000 /* Trace Port Interface Unit (TPIU) */
#  define KINETIS_ETM_BASE           0xe0041000 /* Embedded Trace Macrocell (ETM) */
#  define KINETIS_MCM_BASE           0xe0080000 /* Miscellaneous Control Module (MTM) */
#  define KINETIS_MMCAU_BASE         0xe0081000 /* Memory Mapped Cryptographic
                                                 * Acceleration Unit (MMCAU) */
#  define KINETIS_CACHECTL_BASE      0xe0082000 /* Cache Controller */
#  define KINETIS_ROMTAB_BASE        0xe00ff000 /* ROM Table - allows auto-detection
                                                 * of debug components */

#else
  /* The memory map for other parts is defined in other documents and may or may not
   * be the same as above (the family members are all very similar)  This error just
   * means that you have to look at the document and determine for yourself if the
   * memory map is the same.
   */

#  error "No memory map for this K28 part"
#endif

#endif /* KINETIS_K28 */
#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_K28MEMORYMAP_H */
