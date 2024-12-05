/****************************************************************************
 * arch/arm/src/at32/hardware/at32f43xxx_memorymap.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_MEMORYMAP_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AT32F43XXX Address Blocks ************************************************/

#define AT32_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define AT32_SRAM_BASE      0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define AT32_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define AT32_FSMC_BANK1     0x60000000     /* 0x60000000-0x6fffffff: 256Mb NOR/PSRMA/SRAM */
                                           /* 0x70000000-0x7fffffff: Reserved */
#define AT32_FSMC_BANK3     0x80000000     /* 0x80000000-0x8fffffff: 256Mb NAND FLASH */
#define AT32_QSPI_BANK1     0x90000000     /* 0x90000000-0x9fffffff: 256Mb QUADSPI */

#define AT32_FMC_QSPI_BASE  0xa0000000     /* 0xa0000000-0xa0002fff: FMC and QUADSPI registers */
                                           /* 0xa0003000-0xa7ffffff: Reserved */

#define AT32_FMC_PC_CARD    0xa8000000     /* 0xa8000000-0xafffffff: 128Mb PC card */

#define AT32_FMC_QSPI2      0xb0000000     /* 0xb0000000-0xbfffffff: 128Mb FMC QSPI2 */

#define AT32_FMX_SDRAM      0xc0000000     /* 0xc0000000-0xdfffffff: 256Mb SDRAM */

#define AT32_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define AT32_REGION_MASK    0xf0000000
#define AT32_IS_SRAM(a)     ((((uint32_t)(a)) & AT32_REGION_MASK) == AT32_SRAM_BASE)

/* Code Base Addresses ******************************************************/

#define AT32_BOOT_BASE      0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
                                           /* 0x00100000-0x07ffffff: Reserved */
#define AT32_FLASH_BASE     0x08000000     /* 0x08000000-0x080fffff: FLASH memory */
                                           /* 0x08100000-0x0fffffff: Reserved */
#define AT32_CCMRAM_BASE    0x10000000     /* 0x10000000-0x1000ffff: 64Kb CCM data RAM */
                                           /* 0x10010000-0x1ffeffff: Reserved */
#define AT32_SYSMEM_BASE    0x1fff0000     /* 0x1fff0000-0x1fff7a0f: System memory */
                                           /* 0x1fff7a10-0x1fff7fff: Reserved */
#define AT32_OPTION_BASE    0x1fffc000     /* 0x1fffc000-0x1fffc007: Option bytes */
                                           /* 0x1fffc008-0x1fffffff: Reserved */

/* System Memory Addresses **************************************************/

#define AT32_SYSMEM_UID     0x1ffff7e8     /* The 96-bit unique device identifier */
#define AT32_SYSMEM_FSIZE   0x1ffff7e0     /* This bitfield indicates the size of
                                             * the device Flash memory expressed in
                                             * Kbytes. Example: 0x0400 corresponds
                                             * to 1024 Kbytes.
                                             */

/* SRAM Base Addresses ******************************************************/

/*                                             0x20000000-0x2001bfff:
 *                                             112Kb aliased by bit-banding
 */

/*                                             0x2001c000-0x2001ffff:
 *                                             16Kb aliased by bit-banding
 */

#define AT32_SRAMBB_BASE    0x22000000     /* 0x22000000-          : SRAM bit-band region */

/* Peripheral Base Addresses ************************************************/

#define AT32_APB1_BASE      0x40000000     /* 0x40000000-0x400023ff: APB1 */
                                           /* 0x40002400-0x400027ff: Reserved */
                                           /* 0x40002800-0x400077ff: APB1 */
                                           /* 0x40007800-0x4000ffff: Reserved */
#define AT32_APB2_BASE      0x40010000     /* 0x40010000-0x400023ff: APB2 */
                                           /* 0x40013400-0x400137ff: Reserved */
                                           /* 0x40013800-0x40013bff: SYSCFG */
#define AT32_EXINT_BASE     0x40013c00     /* 0x40013c00-0x40013fff: EXINT */
                                           /* 0x40014000-0x40014bff: APB2 */
                                           /* 0x40014c00-0x4001ffff: Reserved */
#define AT32_AHB1_BASE      0x40020000     /* 0x40020000-0x400223ff: APB1 */
                                           /* 0x40022400-0x40022fff: Reserved */
                                           /* 0x40023000-0x400233ff: CRC */
                                           /* 0x40023400-0x400237ff: Reserved */
                                           /* 0x40023800-0x40023bff: Reset and Clock control RCC */
                                           /* 0x40023c00-0x400293ff: AHB1 (?) */
                                           /* 0x40029400-0x4fffffff: Reserved (?) */
#define AT32_AHB2_BASE      0x50000000     /* 0x50000000-0x5003ffff: AHB2 */
                                           /* 0x50040000-0x5004ffff: Reserved */
                                           /* 0x50050000-0x500503ff: AHB2 */
                                           /* 0x50050400-0x500607ff: Reserved */
                                           /* 0x50060800-0x50060bff: AHB2 */
                                           /* 0x50060c00-0x5fffffff: Reserved */

/* FSMC Base Addresses ******************************************************/

#define AT32_AHB3_BASE      0x60000000     /* 0x60000000-0xa0000fff: AHB3 */

/* APB1 Base Addresses ******************************************************/

#define AT32_TMR2_BASE      0x40000000     /* 0x40000000-0x400003ff: TMR2 timer */
#define AT32_TMR3_BASE      0x40000400     /* 0x40000400-0x400007ff: TMR3 timer */
#define AT32_TMR4_BASE      0x40000800     /* 0x40000800-0x40000bff: TMR4 timer */
#define AT32_TMR5_BASE      0x40000c00     /* 0x40000c00-0x40000fff: TMR5 timer */
#define AT32_TMR6_BASE      0x40001000     /* 0x40001000-0x400013ff: TMR6 timer */
#define AT32_TMR7_BASE      0x40001400     /* 0x40001400-0x400017ff: TMR7 timer */
#define AT32_TMR12_BASE     0x40001800     /* 0x40001800-0x40001bff: TMR12 timer */
#define AT32_TMR13_BASE     0x40001c00     /* 0x40001c00-0x40001fff: TMR13 timer */
#define AT32_TMR14_BASE     0x40002000     /* 0x40002000-0x400023ff: TMR14 timer */
#define AT32_ERTC_BASE      0x40002800     /* 0x40002800-0x40002bff: ERTC & BKP registers */
#define AT32_WWDT_BASE      0x40002c00     /* 0x40002c00-0x40002fff: Window watchdog (WWDG) */
#define AT32_WDT_BASE       0x40003000     /* 0x40003000-0x400033ff: Independent watchdog (IWDG) */
#define AT32_SPI2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2/I2S2 */
#define AT32_I2S2_BASE      0x40003800
#define AT32_SPI3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3/I2S3 */
#define AT32_I2S3_BASE      0x40003c00
#define AT32_USART2_BASE    0x40004400     /* 0x40004400-0x400047ff: USART2 */
#define AT32_USART3_BASE    0x40004800     /* 0x40004800-0x40004bff: USART3 */
#define AT32_UART4_BASE     0x40004c00     /* 0x40004c00-0x40004fff: UART4 */
#define AT32_UART5_BASE     0x40005000     /* 0x40005000-0x400053ff: UART5 */
#define AT32_I2C1_BASE      0x40005400     /* 0x40005400-0x400057ff: I2C1 */
#define AT32_I2C2_BASE      0x40005800     /* 0x40005800-0x40005Bff: I2C2 */
#define AT32_I2C3_BASE      0x40005c00     /* 0x40005c00-0x40005fff: I2C3 */
#define AT32_CAN1_BASE      0x40006400     /* 0x40006400-0x400067ff: bxCAN1 */
#define AT32_CAN2_BASE      0x40006800     /* 0x40006800-0x40006bff: bxCAN2 */
#define AT32_PWC_BASE       0x40007000     /* 0x40007000-0x400073ff: Power control PWR */
#define AT32_DAC_BASE       0x40007400     /* 0x40007400-0x400077ff: DAC */
#define AT32_UART7_BASE     0x40007800     /* 0x40007800-0x40007bff: UART7 */
#define AT32_UART8_BASE     0x40007c00     /* 0x40007c00-0x40007fff: UART8 */

/* APB2 Base Addresses ******************************************************/

#define AT32_TMR1_BASE      0x40010000     /* 0x40010000-0x400103ff: TMR1 timer */
#define AT32_TMR8_BASE      0x40010400     /* 0x40010400-0x400107ff: TMR8 timer */
#define AT32_USART1_BASE    0x40011000     /* 0x40011000-0x400113ff: USART1 */
#define AT32_USART6_BASE    0x40011400     /* 0x40011400-0x400117ff: USART6 */
#define AT32_ADC_BASE       0x40012000     /* 0x40012000-0x400123ff: ADC1-3 */
#define AT32_SPI1_BASE      0x40013000     /* 0x40013000-0x400133ff: SPI1 */
#define AT32_I2S1_BASE      0x40013000
#define AT32_SPI4_BASE      0x40013400     /* 0x40013000-0x400137ff: SPI4 */
#define AT32_I2S4_BASE      0x40013400
#define AT32_SCFG_BASE      0x40013800     /* 0x40013800-0x40013bff: SCFG */
#define AT32_EXINT_BASE     0x40013c00     /* 0x40013c00-0x40013fff: EXINT */
#define AT32_TMR9_BASE      0x40014000     /* 0x40014000-0x400143ff: TMR9 timer */
#define AT32_TMR10_BASE     0x40014400     /* 0x40014400-0x400147ff: TMR10 timer */
#define AT32_TMR11_BASE     0x40014800     /* 0x40014800-0x40014bff: TMR11 timer */
#define AT32_TMR20_BASE     0x40014C00     /* 0x40014C00-0x400173ff: TMR20 timer */
#define AT32_ACC_BASE       0x40017400     /* 0x40017400-0x400177ff: ACC */
#define AT32_I2S2EXT_BASE   0x40017800     /* 0x40017800-0x40017bff: I2S2EXT */
#define AT32_I2S3EXT_BASE   0x40017C00     /* 0x40017C00-0x40017fff: I2S3EXT */

/* AHB1 Base Addresses ******************************************************/

#define AT32_GPIOA_BASE     0x40020000     /* 0x40020000-0x400203ff: GPIO Port A */
#define AT32_GPIOB_BASE     0x40020400     /* 0x40020400-0x400207ff: GPIO Port B */
#define AT32_GPIOC_BASE     0x40020800     /* 0x40020800-0x40020bff: GPIO Port C */
#define AT32_GPIOD_BASE     0X40020C00     /* 0x40020c00-0x40020fff: GPIO Port D */
#define AT32_GPIOE_BASE     0x40021000     /* 0x40021000-0x400213ff: GPIO Port E */
#define AT32_GPIOF_BASE     0x40021400     /* 0x40021400-0x400217ff: GPIO Port F */
#define AT32_GPIOG_BASE     0x40021800     /* 0x40021800-0x40021bff: GPIO Port G */
#define AT32_GPIOH_BASE     0x40021C00     /* 0x40021C00-0x40021fff: GPIO Port H */

#define AT32_CRC_BASE       0x40023000     /* 0x40023000-0x400233ff: CRC */
#define AT32_CRM_BASE       0x40023800     /* 0x40023800-0x40023bff: Reset and Clock control RCC */
#define AT32_FLASHIF_BASE   0x40023c00     /* 0x40023c00-0x40023fff: Flash memory interface */

#define AT32_EDMA_BASE      0x40026000     /* 0x40026000-0x400263ff: EDMA  */
#define AT32_DMA1_BASE      0x40026400     /* 0x40026400-0x400265ff: DMA1  */
#define AT32_DMA2_BASE      0x40026600     /* 0x40026600-0x400267ff: DMA2  */

#define AT32_DMAMUX1_BASE   0x40026500     /* 0x40026500-0x400265ff: DMA1 MUX */
#define AT32_DMAMUX2_BASE   0x40026700     /* 0x40026700-0x400267ff: DMA2 MUX */

#define AT32_EMAC_BASE      0x40028000     /* 0x40028000-0x400283ff: Ethernet MAC */
                                           /* 0x40028400-0x400287ff: Ethernet MAC */
                                           /* 0x40028800-0x40028bff: Ethernet MAC */
                                           /* 0x40028c00-0x40028fff: Ethernet MAC */
                                           /* 0x40029000-0x400293ff: Ethernet MAC */

#define AT32_OTGFS2_BASE    0x40040000     /* 0x40040000-0x4007ffff: USB OTG FS2 */
#define AT32_SDIO1_BASE     0x4002C400     /* 0x4002c400-0x4002c7ff: SDIO1 */

#define AT32_PERIPHBB_BASE  0x42000000     /* Peripheral bit-band region */

/* AHB2 Base Addresses ******************************************************/

#define AT32_OTGFS1_BASE    0x50000000     /* 0x50000000-0x5003ffff: USB OTG FS1 */
#define AT32_DVP_BASE       0x50050000     /* 0x50050000-0x500503ff: DVP */
#define AT32_SDIO2_BASE     0x50061000     /* 0x50061000-0x500613ff: SDIO2 */

/* Cortex-M4 Base Addresses *************************************************/

/* Other registers -- see armv7-m/nvic.h for standard Cortex-M3 registers in
 * this address range
 */

#define AT32_SCS_BASE      0xe000e000
#define AT32_DEBUGMCU_BASE 0xe0042000

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_MEMORYMAP_H */
