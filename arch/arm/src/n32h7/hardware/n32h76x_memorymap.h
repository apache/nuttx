/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h76x_memorymap.h
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

 #ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_MEMORYMAP_H
 #define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* N32H76X Address Blocks ***************************************************/

#define N32_VECTOR_SIZE    0x000003E4

#define N32_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb CODE block */
#define N32_RAM_BASE       0x20000000     /* 0x20000000-0x3fffffff: 512Mb RAM block */
#define N32_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb Peripheral blocks */
#define N32_EXTMEM_BASE    0x60000000     /* 0x60000000-0x9fffffff: 1Gb   External memory blocks */
#define N32_EXTDEV_BASE    0xa0000000     /* 0xa0000000-0xdfffffff: 1Gb   External device blocks */
#define N32_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M7 block */

#define N32_PERIPH_SIZE    0x20000000
#define N32_ITCRAM_SIZE    0x00020000
#define N32_DTCRAM_SIZE    0x000E0000

#define N32_REGION_MASK    0xff000000
#define N32_IS_RAM(a)     ((((uint32_t)(a)) & N32_REGION_MASK) == N32_RAM_BASE)
#define N32_IS_EXTMEM(a)  ((((uint32_t)(a)) & N32_REGION_MASK) == N32_EXTMEM_BASE)

/* Code Base Addresses ******************************************************/

#define N32_ITCRAM_BASE    0x00000000     /* 0x00000000-0x0001ffff: ITCM */
#define N32_FLASH_BANK1    0x15000000     /* 0x15000000-0x151fffff: FLASH memory 1 */
#define N32_FLASH_BANK2    0x17000000     /* 0x17000000-0x18ffffff: FLASH memory 2 */
#define N32_FLASH_BASE     N32_FLASH_BANK1

#if defined(CONFIG_N32H7_FLASH_CONFIG_I)
 #define N32_FLASH_SIZE     (0x00200000-0x00020000)
#endif
#if defined(CONFIG_N32H7_FLASH_CONFIG_K)
 #define N32_FLASH_SIZE     (0x00400000-0x00020000)
#endif

/* SRAM Base Addresses ******************************************************/

#define N32_DTCRAM_BASE    0x20000000     /* 0x20000000-0x200cffff: DTCM-RAM on TCM interface */
#define N32_AXISRAM_BASE   0x24000000     /* 0x24000000-0x2401ffff: System AXI SRAM */

#ifdef CONFIG_ARCH_CHIP_N32H7_CORTEXM7
#  define N32_AHBSRAM_BASE 0x30000000     /* 0x30000000-0x30057fff: System SRAM1 */
#else
#  error "CONFIG_ARCH_CHIP_N32H7_CORTEXM7 not defined"
#endif
#define N32_BSRAM_BASE     0x38000000     /* 0x38800000-0x38800fff: System Backup SRAM */

/* Peripheral Base Addresses ************************************************/

#define N32_PREGION_MASK   0xff000000
#define N32_D2_BASE        0x40000000     /* 0x40000000-0x401403ff: D2 domain */
#  define N32_APB1_BASE    0x40000000     /* 0x40000000-0x4003ffff: APB1 */
#  define N32_AHB1_BASE    0x40040000     /* 0x40040000-0x400affff: AHB1 */
#  define N32_AHB9_BASE    0x400b0000     /* 0x400b0000-0x400cffff: AHB9 */
#  define N32_APB2_BASE    0x400d0000     /* 0x400d0000-0x400effff: APB2 */
#  define N32_AHB2_BASE    0x400f0000     /* 0x400f0000-0x401403ff: AHB2 */
#define N32_D1_BASE        0x50000000     /* 0x50000000-0x51120fff: D1 domain */
#  define N32_APB3_BASE    0x50000000     /* 0x50000000-0x50090fff: APB6 */
#  define N32_AHB3_BASE    0x51000000     /* 0x51000000-0x51120fff: AHB6 */
#define N32_D3_BASE        0x58000000     /* 0x58000000-0x58036fff: D3 domain */
#  define N32_APB4_BASE    0x58000000     /* 0x58000000-0x58005bff: APB5 */
#  define N32_AHB4_BASE    0x58030000     /* 0x58030000-0x58036fff: AHB5 */

/* APB1 Base Addresses ******************************************************/

#define N32_FDCAN1_BASE        0x40000000
#define N32_FDCAN2_BASE        0x40000400
#define N32_FDCAN5_BASE        0x40000800
#define N32_FDCAN6_BASE        0x40000c00
#define N32_AHB_ICACHE_BASE    0x40008000
#define N32_AHB_DCACHE_BASE    0x40009000
#define N32_BTIMER1_BASE       0x4000a000
#define N32_BTIMER2_BASE       0x4000a400
#define N32_GTIMERB1_BASE      0x4000a800
#define N32_GTIMERB2_BASE      0x4000ac00
#define N32_GTIMERB3_BASE      0x4000b000
#define N32_GTIMERA4_BASE      0x4000b400
#define N32_GTIMERA5_BASE      0x4000b800
#define N32_GTIMERA6_BASE      0x4000bc00
#define N32_GTIMERA7_BASE      0x4000c000
#define N32_USART1_BASE        0x4000c400
#define N32_USART2_BASE        0x4000c800
#define N32_USART3_BASE        0x4000cc00
#define N32_USART4_BASE        0x4000d000
#define N32_UART9_BASE         0x4000d400
#define N32_UART10_BASE        0x4000d800
#define N32_UART11_BASE        0x4000dc00
#define N32_UART12_BASE        0x4000e000
#define N32_SPI3_BASE          0x4000e400
#define N32_I2C1_BASE          0x4000e800
#define N32_I2C2_BASE          0x4000ec00
#define N32_I2C3_BASE          0x4000f000
#define N32_I2S3_BASE          0x4000f400
#define N32_I2S4_BASE          0x4000f800
#define N32_DAC12_BASE         0x4000fc00
#define N32_WWDG2_BASE         0x40010000
#define N32_BTIMER3_BASE       0x40010800
#define N32_BTIMER4_BASE       0x40010c00

/* AHB1 Base Addresses ******************************************************/

#define N32_ETH2_BASE                0x40042000
#define N32_ADC1_BASE                0x40044000
#define N32_ADC2_BASE                0x40044400
#define N32_ADC3_BASE                0x40044800
#define N32_AHB_CACHE_PARMON_BASE    0x40045000
#define N32_DMAMUX1_BASE             0x40046400
#define N32_DMA1_BASE                0x40046800
#define N32_DMA2_BASE                0x40046C00
#define N32_DMA3_BASE                0x40047000
#define N32_SDMMC2_CFG_BASE          0x4004A000
#define N32_SDHOST2_BASE             0x40050000
#define N32_USBCTRL2_BASE            0x40060000
#define N32_USBCTRL2_WRAPPER_BASE    0x400A0000

/* AHB9 Base Addresses ******************************************************/

#define N32_ESC_BASE             0x400B0000
#define N32_ESC_WRAPPER_BASE     0x400C0000

/* APB2 Base Addresses ******************************************************/

#define N32_FDCAN3_BASE      0x400D0000
#define N32_FDCAN4_BASE      0x400D0400
#define N32_FDCAN7_BASE      0x400D0800
#define N32_FDCAN8_BASE      0x400D0C00
#define N32_SHRTIM1_BASE     0x400D8000
#define N32_SHRTIM2_BASE     0x400D9000
#define N32_DSMU_BASE        0x400DA000
#define N32_ATIMER1_BASE     0x400DB000
#define N32_ATIMER2_BASE     0x400DB400
#define N32_I2S1_BASE        0x400DB800
#define N32_I2S2_BASE        0x400DBC00
#define N32_SPI1_BASE        0x400DC000
#define N32_SPI2_BASE        0x400DC400
#define N32_GTIMERA1_BASE    0x400DC800
#define N32_GTIMERA2_BASE    0x400DCC00
#define N32_GTIMERA3_BASE    0x400DD000
#define N32_I2C4_BASE        0x400DD400
#define N32_I2C5_BASE        0x400DD800
#define N32_I2C6_BASE        0x400DDC00
#define N32_USART5_BASE      0x400DE000
#define N32_USART6_BASE      0x400DE400
#define N32_USART7_BASE      0x400DE800
#define N32_USART8_BASE      0x400DEC00
#define N32_UART13_BASE      0x400DF000
#define N32_UART14_BASE      0x400DF400
#define N32_UART15_BASE      0x400DF800

/* AHB2 Base Addresses ******************************************************/

#define N32_ECCMON2_BASE           0x400F0000
#define N32_CORDIC_BASE            0x400F1000
#define N32_FMAC_BASE              0x400F1400
#define N32_DAC34_BASE             0x400F1800
#define N32_DAC56_BASE             0x400F1C00
#define N32_SDPU_BASE              0x400F2000
#define N32_ETH1_BASE              0x400F4000
#define N32_SEMA4_BASE             0x400F6000
#define N32_DCMUA_BASE             0x400F7000
#define N32_DCMUB_BASE             0x400F8000
#define N32_USBCTRL1_BASE          0x40100000
#define N32_USBCTRL1_WRAPPER_BASE  0x40140000

/* APB6 Base Addresses ******************************************************/

#define N32_DSIHOST_BASE         0x50000000
#define N32_2_5D_GPU_BASE        0x50040000
#define N32_DVP1_BASE            0x50048000
#define N32_DVP2_BASE            0x50049000
#define N32_LCDC_BASE            0x5004a000
#define N32_WWDG1_BASE           0x5004a800
#define N32_DSIHOST_WRAPPER_BASE 0x5004ac00
#define N32_TCMSRAMC_BASE        0x5004b000
#define N32_FEMC_BASE            0x5004c000
#define N32_JPEG_ENC_BASE        0x50060000
#define N32_JPEG_RBC_BASE        0x50070000
#define N32_JPEG_H2P_SGDMA_BASE  0x50070400
#define N32_JPEG_DEC_BASE        0x50080000
#define N32_JPEG_BRC_BASE        0x50090000
#define N32_JPEG_P2H_SGDMA_BASE  0x50090400
#define N32_JPEG_CTRL_BASE       0x50090800

/* AHB6 Base Addresses ******************************************************/

#define N32_GPV_BASE          0x51000000
#define N32_xSPI1_BASE        0x51100000
#define N32_xSPI2_BASE        0x51101000
#define N32_MDMA_BASE         0x51102000
#define N32_ECCMON1_BASE      0x51104000
#define N32_MMU_BASE          0x51105000
#define N32_SDRAM_BASE        0x51106000
#define N32_SDMMC1_CFG_BASE   0x51107000
#define N32_SDHOST1_BASE      0x51110000
#define N32_OTPC_BASE         0x51118000
#define N32_DMAMUX2_BASE      0x51120000
#define N32_MDMA_WRAPPER_BASE 0x51120800

/* APB5 Base Addresses ******************************************************/
#define N32_EXTI_WKUP_BASE    0x58000000
#define N32_AFEC_BASE         0x58000400
#define N32_LPUART1_BASE      0x58000800
#define N32_LPUART2_BASE      0x58000C00
#define N32_LPTIMER1_BASE     0x58001000
#define N32_LPTIMER_BASE      0x58001400
#define N32_LPTIMER3_BASE     0x58001800
#define N32_LPTIMER4_BASE     0x58001C00
#define N32_SPI4_BASE         0x58002000
#define N32_SPI5_BASE         0x58002400
#define N32_SPI6_BASE         0x58002800
#define N32_SPI7_BASE         0x58002C00
#define N32_I2C7_BASE         0x58003000
#define N32_I2C8_BASE         0x58003400
#define N32_I2C9_BASE         0x58003800
#define N32_I2C10_BASE        0x58003C00
#define N32_ATIMER3_BASE      0x58004000
#define N32_ATIMER4_BASE      0x58004400
#define N32_COMPCTRL_BASE     0x58004800
#define N32_IWDG1_BASE        0x58004C00
#define N32_IWDG2_BASE        0x58005000
#define N32_RTC_BASE          0x58005400
#define N32_LPTIMER5_BASE     0x58005800

/* AHB5 Base Addresses ******************************************************/
#define N32_AFIO_BASE         0x58032400
#define N32_CRC_BASE          0x58032000
#define N32_PWR_BASE          0x58031000
#define N32_RCC_BASE          0x58030000
#define N32_GPIOA_BASE        0x58032800
#define N32_GPIOB_BASE        0x58032C00
#define N32_GPIOC_BASE        0x58033000
#define N32_GPIOD_BASE        0x58033400
#define N32_GPIOE_BASE        0x58033800
#define N32_GPIOF_BASE        0x58033C00
#define N32_GPIOG_BASE        0x58034000
#define N32_GPIOH_BASE        0x58034400
#define N32_GPIOI_BASE        0x58034800
#define N32_GPIOJ_BASE        0x58034C00
#define N32_GPIOK_BASE        0x58035000
#define N32_ECCMON3_BASE      0x58036000

#define N32_SYSMEM_UID        0x0238

/* The DBGMCU registers are accessible to the debugger via the APB-D bus
 * at base address 0xe00e1000. They are also accessible by the processor
 * core at base address 0x58035400.
 */

#define N32_DEBUGMCU_BASE     0x58035400

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_MEMORYMAP_H */
