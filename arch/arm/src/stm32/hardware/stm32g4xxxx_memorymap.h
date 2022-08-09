/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32g4xxxx_memorymap.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32G4xxxx Address Blocks ***********************************************/

#define STM32_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32_SRAM_BASE      0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define STM32_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define STM32_FMC_BANK1      0x60000000     /* 0x60000000-0x6fffffff: 256Mb NOR/PSRMA/SRAM */
                                            /* 0x70000000-0x7fffffff: Reserved */
#define STM32_FMC_BANK3      0x80000000     /* 0x80000000-0x8fffffff: 256Mb NAND FLASH */
#define STM32_QSPI_BANK1     0x90000000     /* 0x90000000-0x9fffffff: 256Mb QUADSPI */

#define STM32_FMC_QSPI_BASE  0xa0000000     /* 0xa0000000-0xbfffffff: 256Mb FMC and QUADSPI registers */
                                            /* 0xc0000000-0xdfffffff: Reserved */
#define STM32_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define STM32_REGION_MASK    0xf0000000
#define STM32_IS_SRAM(a)     ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)

/* Code Base Addresses ******************************************************/

#define STM32_BOOT_BASE      0x00000000     /* 0x00000000-0x0007ffff: Aliased boot memory */
                                            /* 0x00080000-0x07ffffff: Reserved */
#define STM32_FLASH_BASE     0x08000000     /* 0x08000000-0x807ffff: Up to 512Kb FLASH memory */
                                            /* 0x08080000-0xfffffff: Reserved */
#define STM32_CCMRAM_BASE    0x10000000     /* 0x10000000-0x10007fff: 32Kb CCM data SRAM */
                                            /* 0x10008000-0x1ffeffff: Reserved */
#define STM32_SYSMEM_BASE    0x1fff0000     /* 0x1fff0000-0x1fff6fff: 28Kb System memory */
#define STM32_OTP_AREA_BASE  0x1fff7000     /* 0x1fff7000-0x1fff73ff: 1Kb OTP area */
                                            /* 0x1fff7400-0x1fff77ff: Reserved */
#define STM32_OPTION_BASE    0x1fff7800     /* 0x1fff7800-0x1fff782f: 48 Option bytes */
                                            /* 0x1fff7830-0x1fff7fff: Reserved */
#define STM32_SYSMEM_BASE2   0x1fff8000     /* 0x1fff8000-0x1fffefff: 28Kb System memory */
                                            /* 0x1ffff000-0x1ffff7ff: Reserved */
#define STM32_OPTION_BASE2   0x1ffff800     /* 0x1ffff800-0x1ffff82f: 48 Option bytes */
                                            /* 0x1fff7830-0x1fff7fff: Reserved */

/* System Memory Addresses **************************************************/

#define STM32_PACKAGE_INFO   0x1fff7500     /* Package data register */
#define STM32_SYSMEM_UID     0x1fff7590     /* The 96-bit unique device identifier */
#define STM32_SYSMEM_FSIZE   0x1fff75e0     /* This bitfield indicates the size of
                                             * the device Flash memory expressed in
                                             * Kbytes.  Example: 0x040 corresponds
                                             * to 64 Kbytes
                                             */

/* Peripheral Base Addresses ************************************************/

#define STM32_APB1_BASE      0x40000000     /* 0x40000000-0x400097ff: APB1 */
                                            /* 0x40009800-0x4000ffff: Reserved */
#define STM32_APB2_BASE      0x40010000     /* 0x40010000-0x400163ff: APB2 */
                                            /* 0x40016400-0x4001ffff: Reserved */
#define STM32_AHB1_BASE      0x40020000     /* 0x40020000-0x400243ff: APB1 */
                                            /* 0x40024400-0x47ffffff: Reserved */
#define STM32_AHB2_BASE      0x48000000     /* 0x48000000-0x50060bff: AHB2 */
                                            /* 0x50060c00-0x5fffffff: Reserved */

/* APB1 Base Addresses ******************************************************/

#define STM32_TIM2_BASE      0x40000000     /* 0x40000000-0x400003ff: TIM2 */
#define STM32_TIM3_BASE      0x40000400     /* 0x40000400-0x400007ff: TIM3 */
#define STM32_TIM4_BASE      0x40000800     /* 0x40000800-0x40000bff: TIM4 */
#define STM32_TIM5_BASE      0x40000c00     /* 0x40000c00-0x40000fff: TIM5 */
#define STM32_TIM6_BASE      0x40001000     /* 0x40001000-0x400013ff: TIM6 */
#define STM32_TIM7_BASE      0x40001400     /* 0x40001400-0x400017ff: TIM7 */
#define STM32_CRS_BASE       0x40002000     /* 0x40002000-0x400023ff: CRS */
#define STM32_TAMP_BASE      0x40002400     /* 0x40002400-0x400027ff: TAMP */
#define STM32_RTC_BASE       0x40002800     /* 0x40002800-0x40002bff: RTC */
#define STM32_WWDG_BASE      0x40002c00     /* 0x40002c00-0x40002fff: WWDG */
#define STM32_IWDG_BASE      0x40003000     /* 0x40003000-0x400033ff: IWDG */
#define STM32_SPI2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2 */
#define STM32_SPI3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3 */
#define STM32_USART2_BASE    0x40004400     /* 0x40004400-0x400047ff: USART2 */
#define STM32_USART3_BASE    0x40004800     /* 0x40004800-0x40004bff: USART3 */
#define STM32_UART4_BASE     0x40004c00     /* 0x40004c00-0x40004fff: UART4 */
#define STM32_UART5_BASE     0x40005000     /* 0x40005000-0x400053ff: UART5 */
#define STM32_I2C1_BASE      0x40005400     /* 0x40005400-0x400057ff: I2C1 */
#define STM32_I2C2_BASE      0x40005800     /* 0x40005800-0x40005bff: I2C2 */
#define STM32_USB_BASE       0x40005c00     /* 0x40005c00-0x40005fff: USB */
#define STM32_USBRAM_BASE    0x40006000     /* 0x40006000-0x400063ff: USB SRAM */
#define STM32_FDCAN1_BASE    0x40006400     /* 0x40006400-0x400067ff: FDCAN1 */
#define STM32_FDCAN2_BASE    0x40006800     /* 0x40006800-0x40006bff: FDCAN2 */
#define STM32_FDCAN3_BASE    0x40006c00     /* 0x40006c00-0x40006fff: FDCAN3 */
#define STM32_PWR_BASE       0x40007000     /* 0x40007000-0x400073ff: PWR */
#define STM32_I2C3_BASE      0x40007800     /* 0x40007800-0x40007bff: I2C3 */
#define STM32_LPTIM1_BASE    0x40007c00     /* 0x40007c00-0x40007fff: LPTIM1 */
#define STM32_LPUART1_BASE   0x40008000     /* 0x40008000-0x400083ff: LPUART1 */
#define STM32_I2C4_BASE      0x40008400     /* 0x40008400-0x400087ff: I2C4 */
#define STM32_UCPD1_BASE     0x4000a000     /* 0x4000a000-0x4000a3ff: UCPD1 */
#define STM32_CANRAM_BASE    0x4000a400     /* 0x4000a400-0x4000afff: FDCANs Message RAM */

/* APB2 Base Addresses ******************************************************/

#define STM32_SYSCFG_BASE    0x40010000     /* 0x40010000-0x40010029: SYSCFG */
#define STM32_VREFBUF_BASE   0x40010030     /* 0x40010030-0x400101ff: VREFBUF */
#define STM32_COMP1_BASE     0x40010200     /* 0x40010200-0x40010203: COMP1 */
#define STM32_COMP2_BASE     0x40010204     /* 0x40010204-0x40010207: COMP2 */
#define STM32_COMP3_BASE     0x40010208     /* 0x40010208-0x4001020b: COMP3 */
#define STM32_COMP4_BASE     0x4001020c     /* 0x4001020c-0x4001020f: COMP4 */
#define STM32_COMP5_BASE     0x40010210     /* 0x40010210-0x40010213: COMP5 */
#define STM32_COMP6_BASE     0x40010214     /* 0x40010214-0x40010217: COMP6 */
#define STM32_COMP7_BASE     0x40010218     /* 0x40010218-0x400102ff: COMP7 */
#define STM32_OPAMP_BASE     0x40010300     /* 0x40010300-0x40010303: OPAMP */
#define STM32_EXTI_BASE      0x40010400     /* 0x40010400-0x400107ff: EXTI */
#define STM32_TIM1_BASE      0x40012c00     /* 0x40012c00-0x40012fff: TIM1 */
#define STM32_SPI1_BASE      0x40013000     /* 0x40013000-0x400133ff: SPI1 */
#define STM32_TIM8_BASE      0x40013400     /* 0x40013400-0x400137ff: TIM8 */
#define STM32_USART1_BASE    0x40013800     /* 0x40013800-0x40013bff: USART1 */
#define STM32_SPI4_BASE      0x40013c00     /* 0x40013c00-0x40013fff: SPI4 */
#define STM32_TIM15_BASE     0x40014000     /* 0x40014000-0x400143ff: TIM15 */
#define STM32_TIM16_BASE     0x40014400     /* 0x40014400-0x400147ff: TIM16 */
#define STM32_TIM17_BASE     0x40014800     /* 0x40014800-0x40014bff: TIM17 */
#define STM32_TIM20_BASE     0x40015000     /* 0x40015000-0x400153ff: TIM20 */
#define STM32_SAI1_BASE      0x40015400     /* 0x40015400-0x400157ff: SAI1 */
#define STM32_HRTIM1_BASE    0x40016800     /* 0x40016800-0x400167ff: HRTIM1 */

/* AHB1 Base Addresses ******************************************************/

#define STM32_DMA1_BASE      0x40020000     /* 0x40020000-0x400203ff: DMA1 */
#define STM32_DMA2_BASE      0x40020400     /* 0x40020400-0x400207ff: DMA2 */
#define STM32_DMAMUX1_BASE   0x40020800     /* 0x40020800-0x40020bff: DMAMUX1 */
#define STM32_CORDIC_BASE    0x40020c00     /* 0x40020c00-0x40020fff: CORDIC */
#define STM32_RCC_BASE       0x40021000     /* 0x40021000-0x400213ff: Reset and Clock Control (RCC) */
#define STM32_FMAC_BASE      0x40021400     /* 0x40021400-0x40021fff: FMAC */
#define STM32_FLASHIF_BASE   0x40022000     /* 0x40022000-0x400223ff: Flash memory I/F */
#define STM32_CRC_BASE       0x40023000     /* 0x40023000-0x400233ff: CRC */

/* AHB2 Base Addresses ******************************************************/

#define STM32_GPIOA_BASE     0x48000000     /* 0x48000000-0x480003ff: GPIO Port A */
#define STM32_GPIOB_BASE     0x48000400     /* 0x48000400-0x480007ff: GPIO Port B */
#define STM32_GPIOC_BASE     0x48000800     /* 0x48000800-0x48000bff: GPIO Port C */
#define STM32_GPIOD_BASE     0x48000c00     /* 0X48000C00-0x48000fff: GPIO Port D */
#define STM32_GPIOE_BASE     0x48001000     /* 0X48001000-0x480013ff: GPIO Port E */
#define STM32_GPIOF_BASE     0x48001400     /* 0x48001400-0x480017ff: GPIO Port F */
#define STM32_GPIOG_BASE     0x48001800     /* 0x48001800-0x48001bff: GPIO Port G */
#define STM32_ADC12_BASE     0x50000000     /* 0x50000000-0x500003ff: ADC12 */
#define STM32_ADC345_BASE    0x50000400     /* 0x50000400-0x500007ff: ADC345 */
#define STM32_DAC_BASE       0x50000800     /* 0x50000800-0x50000bff: DAC */
#define STM32_DAC1_BASE      0x50000800     /* 0x50000800-0x50000bff: DAC1 */
#define STM32_DAC2_BASE      0x50000c00     /* 0x50000c00-0x50000fff: DAC2 */
#define STM32_DAC3_BASE      0x50001000     /* 0x50001000-0x500013ff: DAC3 */
#define STM32_DAC4_BASE      0x50001400     /* 0x50001400-0x500017ff: DAC4 */

/* Compatibility defines */

#define STM32_ADC34_BASE     STM32_ADC345_BASE

/* Cortex-M4 Base Addresses *************************************************/

/* Other registers -- see armv7-m/nvic.h for standard Cortex-M4 registers in
 * this address range
 */

#define STM32_DEBUGMCU_BASE  0xe0042000

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32G4XXXX_MEMORYMAP_H */
