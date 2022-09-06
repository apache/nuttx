/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f450_memorymap.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F450_MEMORYMAP_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F450_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GD32F4XX Address Blocks **************************************************/

#define GD32_CODE_BASE       0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define GD32_SRAM_BASE       0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define GD32_PERIPH_BASE     0x40000000     /* 0x40000000-0x5fffffff: 512Mb peripheral block */

#define GD32_EXMC_REG_BASE   0xa0000000     /* 0xa0000000-0xbfffffff: 512Mb EXMC register block */
#define GD32_EXMC_BASE       0x60000000     /* 0x60000000-0x7fffffff: EXMC base */
#define GD32_EXMC_BANK0      0x60000000     /* 0x60000000-0x6fffffff: 256Mb NOR/PSRAM/SRAM */
#define GD32_EXMC_BANK1      0x70000000     /* 0x70000000-0x7fffffff: 256Mb NAND FLASH */
#define GD32_EXMC_BANK2      0x80000000     /* 0x80000000-0x8fffffff: 256Mb NAND FLASH */
#define GD32_EXMC_BANK3      0x90000000     /* 0x90000000-0x9fffffff: 256Mb PC CARD*/
#define GD32_EXMC_BANK4      0xc0000000     /* 0xc0000000-0xcfffffff: 256Mb SDRAM */
#define GD32_EXMC_BANK5      0xd0000000     /* 0xd0000000-0xdfffffff: 256Mb SDRAM */

#define GD32_CORTEX_BASE     0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define GD32_REGION_MASK     0xf0000000
#define GD32_IS_SRAM(a)      ((((uint32_t)(a)) & GD32_REGION_MASK) == GD32_SRAM_BASE)
#define GD32_IS_EXTSRAM(a)   ((((uint32_t)(a)) & GD32_REGION_MASK) == GD32_EXMC_BANK0)

/* Code Base Addresses ******************************************************/

#define GD32_BOOT_BASE       0x00000000     /* 0x00000000-0x007ffffff: Aliased to the boot device */
#define GD32_FLASH_BASE      0x08000000     /* 0x08000000-0x082fffff: Main flash */
                                            /* 0x08100000-0x0fffffff: Reserved */
#define GD32_TCMSRAM_BASE    0x10000000     /* 0x10000000-0x1000ffff: 64Kb TCMSRAM */
                                            /* 0x10010000-0x1ffebfff: Reserved */
#define GD32_OPBYTE_BANK1    0x1ffec000     /* 0x1ffec000-0x1ffec00f: 16b Option bytes */
                                            /* 0x1ffec010-0x1ffeffff: Reserved */
#define GD32_BOOTLD_BASE     0x1fff0000     /* 0x1fff0000-0x1fff77ff: 30Kb Boot loader */
#define GD32_OTP_BASE        0x1fff7800     /* 0x1fff7800-0x1fff7a0f: 528b OTP block */
                                            /* 0x1fff7a10-0x1fffbfff: Reserved */
#define GD32_OPBYTE_BANK0    0x1fffc000     /* 0x1fffc000-0x1fffc00f: 16b Option bytes */
                                            /* 0x1fffc010-0x1fffffff: Reserved */

/* System Memory Addresses **************************************************/

#define GD32_UNIQUE_ID       0x1fff7a10     /* The 96-bit unique device ID */
#define GD32_FLASH_DENSITY   0x1fff7a20     /* The memory density information, indicates the size of
                                             * the device Flash or RAM memory expressed in
                                             * Kbytes. Example: 0x020 corresponds
                                             * to 32 Kbytes.
                                             */

/* SRAM Base Addresses ******************************************************/

/*                                             0x20000000-0x2001bfff:
 *                                               112Kb aliased by bit-banding
 */

/*                                             0x2001c000-0x2001ffff:
 *                                               16Kb aliased by bit-banding
 */

#define GD32_SRAMBB_BASE     0x22000000     /* 0x22000000-          : SRAM bit-band region */

/* Peripheral Base Addresses ************************************************/

#define GD32_APB1_BUS_BASE   0x40000000     /* APB1 base address */                                          
#define GD32_APB2_BUS_BASE   0x40010000     /* APB2 base address */
#define GD32_AHB1_BUS_BASE   0x40020000     /* AHB1 base address */
#define GD32_AHB2_BUS_BASE   0x50000000     /* AHB2 base address */

/* APB1 Base Addresses ******************************************************/

#define GD32_TIMER_BASE      (GD32_APB1_BUS_BASE + 0x00000000U)     /* TIMER base address */
#define GD32_RTC_BASE        (GD32_APB1_BUS_BASE + 0x00002800U)     /* RTC base address */
#define GD32_WWDGT_BASE      (GD32_APB1_BUS_BASE + 0x00002C00U)     /* WWDGT base address */
#define GD32_FWDGT_BASE      (GD32_APB1_BUS_BASE + 0x00003000U)     /* FWDGT base address */
#define GD32_I2S_ADD_BASE    (GD32_APB1_BUS_BASE + 0x00003400U)     /* I2S1_add base address */
#define GD32_SPI_BASE        (GD32_APB1_BUS_BASE + 0x00003800U)     /* SPI base address */
#define GD32_USART_BASE      (GD32_APB1_BUS_BASE + 0x00004400U)     /* USART base address */
#define GD32_I2C_BASE        (GD32_APB1_BUS_BASE + 0x00005400U)     /* I2C base address */
#define GD32_CAN_BASE        (GD32_APB1_BUS_BASE + 0x00006400U)     /* CAN base address */
#define GD32_CTC_BASE        (GD32_APB1_BUS_BASE + 0x00006C00U)     /* CTC base address */
#define GD32_PMU_BASE        (GD32_APB1_BUS_BASE + 0x00007000U)     /* PMU base address */
#define GD32_DAC_BASE        (GD32_APB1_BUS_BASE + 0x00007400U)     /* DAC base address */
#define GD32_IREF_BASE       (GD32_APB1_BUS_BASE + 0x0000C400U)     /* IREF base address */

/* APB2 Base Addresses ******************************************************/

#define GD32_TLI_BASE        (GD32_APB2_BUS_BASE + 0x00006800U)     /* TLI base address */
#define GD32_SYSCFG_BASE     (GD32_APB2_BUS_BASE + 0x00003800U)     /* SYSCFG base address */
#define GD32_EXTI_BASE       (GD32_APB2_BUS_BASE + 0x00003C00U)     /* EXTI base address*/
#define GD32_SDIO_BASE       (GD32_APB2_BUS_BASE + 0x00002C00U)     /* SDIO base address */
#define GD32_ADC_BASE        (GD32_APB2_BUS_BASE + 0x00002000U)     /* ADC base address */

/* AHB1 Base Addresses ******************************************************/

#define GD32_GPIO_BASE       (GD32_AHB1_BUS_BASE + 0x00000000U)     /* GPIO base address*/
#define GD32_CRC_BASE        (GD32_AHB1_BUS_BASE + 0x00003000U)     /* CRC base address */
#define GD32_RCU_BASE        (GD32_AHB1_BUS_BASE + 0x00003800U)     /* RCU base address */
#define GD32_FMC_BASE        (GD32_AHB1_BUS_BASE + 0x00003C00U)     /* FMC base address */
#define GD32_BKPSRAM_BASE    (GD32_AHB1_BUS_BASE + 0x00004000U)     /* BKPSRAM base address */
#define GD32_DMA_BASE        (GD32_AHB1_BUS_BASE + 0x00006000U)     /* DMA base address */
#define GD32_ENET_BASE       (GD32_AHB1_BUS_BASE + 0x00008000U)     /* ENET base address */
#define GD32_IPA_BASE        (GD32_AHB1_BUS_BASE + 0x0000B000U)     /* IPA base address */
#define GD32_USBHS_BASE      (GD32_AHB1_BUS_BASE + 0x00020000U)     /* USBHS base address */

/* AHB2 Base Addresses ******************************************************/

#define GD32_USBFS_BASE      (GD32_AHB2_BUS_BASE + 0x00000000U)     /* USBFS base address */
#define GD32_DCI_BASE        (GD32_AHB2_BUS_BASE + 0x00050000U)     /* DCI base address */
#define GD32_TRNG_BASE       (GD32_AHB2_BUS_BASE + 0x00060800U)     /* TRNG base address */

/* DBG register Base Addresses **********************************************/

#define GD32_DBG_BASE      0xe0042000

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F450_MEMORYMAP_H */
