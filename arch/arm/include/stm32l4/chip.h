/****************************************************************************
 * arch/arm/include/stm32l4/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32L4_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32L4_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* STM32L475, STM32L476, STM32L486, STM32L496, STM32L4A6
 *
 * Differences between family members:
 *  - L475 has no TSC, no LCD, no AES, no I2C4, no CAN2, No Hash/CRS, no
 *    DCMI, no DMA2D
 *  - L486 has AES
 *  - L496, L4A6 has 320 Kib SRAM, 2xCAN and CameraIF. Most (all?) of these
 *    have I2C4.
 *  - L4A6 has AES and HASH
 *
 *   ----------- ---------------- ----- ------ ------ ---- ---- -----
 *   PART        PACKAGE          GPIOs LCD    Tamper FSMC CapS AdcCh
 *   ----------- ---------------- ----- ------ ------ ---- ---- -----
 *   STM32L475Rx LQFP100            82          3     Yes   21   16
 *   STM32L475Vx LQFP64             51          2     No    12   16
 *   STM32L4x6Jx WLCSP72L           57   8x28   2     No    12   16
 *   STM32L476Mx WLCSP81L           65   ?      ?     ?     ?    ?
 *   STM32L4x6Qx UFBGA132L         109   8x40   3     Yes   24   16
 *   STM32L4x6Rx LQFP64             51   8x28   2     No    12   16
 *   STM32L4x6Vx LQFP100            82   8x40   3     Yes   21   16
 *   STM32L4x6Zx LQFP144           114   8x40   3     Yes   24   24
 *   STM32L4x6Ax UFBGA169          132   8x40   3     Yes   24   24
 *   ----------- ---------------- ----- ------ ------ ---- ---- -----
 *
 * Parts STM32L4x6xC have 256Kb of FLASH
 * Parts STM32L4x6xE have 512Kb of FLASH
 * Parts STM32L4x6xG have 1024Kb of FLASH
 *
 * The correct FLASH size must be set with a CONFIG_STM32L4_FLASH_CONFIG_*
 * selection.
 */

#if defined(CONFIG_STM32L4_STM32L4XR)
#  define STM32L4_SRAM1_SIZE       (192*1024)  /* 192Kb SRAM1 on AHB bus Matrix */
#  define STM32L4_SRAM2_SIZE       (64*1024)   /* 64Kb  SRAM2 on AHB bus Matrix */
#  define STM32L4_SRAM3_SIZE       (384*1024)  /* 384Kb SRAM3 on AHB bus Matrix */
#elif defined(CONFIG_STM32L4_STM32L496XX)
#  define STM32L4_SRAM1_SIZE       (256*1024)  /* 256Kb SRAM1 on AHB bus Matrix */
#  define STM32L4_SRAM2_SIZE       (64*1024)   /* 64Kb  SRAM2 on AHB bus Matrix */
#elif defined(CONFIG_STM32L4_STM32L475XX) || defined(CONFIG_STM32L4_STM32L476XX) || \
      defined(CONFIG_STM32L4_STM32L486XX)
#  define STM32L4_SRAM1_SIZE       (96*1024)   /* 96Kb SRAM1 on AHB bus Matrix */
#  define STM32L4_SRAM2_SIZE       (32*1024)   /* 32Kb SRAM2 on AHB bus Matrix */
#elif defined(CONFIG_STM32L4_STM32L451XX) || defined(CONFIG_STM32L4_STM32L452XX) || \
      defined(CONFIG_STM32L4_STM32L462XX)
#  define STM32L4_SRAM1_SIZE       (128*1024)  /* 128Kb SRAM1 on AHB bus Matrix */
#  define STM32L4_SRAM2_SIZE       (32*1024)   /* 32Kb SRAM2 on AHB bus Matrix */
#elif defined(CONFIG_STM32L4_STM32L432XX) || defined(CONFIG_STM32L4_STM32L433XX)
#  define STM32L4_SRAM1_SIZE       (48*1024)   /* 48Kb SRAM1 on AHB bus Matrix */
#  define STM32L4_SRAM2_SIZE       (16*1024)   /* 16Kb SRAM2 on AHB bus Matrix */
#elif defined(CONFIG_STM32L4_STM32L412XX) || defined(CONFIG_STM32L4_STM32L422XX)
#  define STM32L4_SRAM1_SIZE       (32*1024)   /* 32Kb SRAM1 on AHB bus Matrix */
#  define STM32L4_SRAM2_SIZE       (8*1024)    /* 8Kb SRAM2 on AHB bus Matrix */
#else
#  error "Unsupported STM32L4 chip"
#endif

#if defined(CONFIG_STM32L4_STM32L4XR)
#  define STM32L4_NFSMC                    1   /* Have FSMC memory controller */
#  define STM32L4_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32L4_NGTIM32                  2   /* 32-bit general timers TIM2 and 5 with DMA */
#  define STM32L4_NGTIM16                  2   /* 16-bit general timers TIM3 and 4 with DMA */
#  define STM32L4_NGTIMNDMA                3   /* 16-bit general timers TIM15-17 without DMA */
#  define STM32L4_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32L4_NLPTIM                   2   /* Two low-power timers, LPTIM1-2 */
#  define STM32L4_NRNG                     1   /* Random number generator (RNG) */
#  define STM32L4_NUART                    2   /* UART 4-5 */
#  define STM32L4_NUSART                   3   /* USART 1-3 */
#  define STM32L4_NLPUART                  1   /* LPUART 1 */
#  define STM32L4_QSPI                     0   /* No QuadSPI1 */
#  define STM32L4_OCTOSPI                  2   /* OCTOSPI1-2 */
#  define STM32L4_NSPI                     3   /* SPI1-3 */
#  define STM32L4_NI2C                     4   /* I2C1-4 */
#  define STM32L4_NSWPMI                   0   /* No SWPMI1 */
#  define STM32L4_NUSBOTGFS                1   /* USB OTG FS */
#  define STM32L4_NUSBFS                   0   /* No USB FS */
#  define STM32L4_NCAN                     1   /* CAN1 */
#  define STM32L4_NSAI                     2   /* SAI1-2 */
#  define STM32L4_NSDMMC                   1   /* SDMMC interface */
#  define STM32L4_NDMA                     2   /* DMA1-2 */
#  define STM32L4_NPORTS                   9   /* 9 GPIO ports, GPIOA-I */
#  define STM32L4_NADC                     1   /* 12-bit ADC1, up to 20 channels */
#  define STM32L4_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32L4_NCRC                     1   /* CRC */
#  define STM32L4_NCOMP                    2   /* Comparators */
#  define STM32L4_NOPAMP                   2   /* Operational Amplifiers */
#endif /* CONFIG_STM32L4_STM32L4XR */

#if defined(CONFIG_STM32L4_STM32L4X5)
#  define STM32L4_NFSMC                    1   /* Have FSMC memory controller */
#  define STM32L4_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32L4_NGTIM32                  2   /* 32-bit general timers TIM2 and 5 with DMA */
#  define STM32L4_NGTIM16                  2   /* 16-bit general timers TIM3 and 4 with DMA */
#  define STM32L4_NGTIMNDMA                3   /* 16-bit general timers TIM15-17 without DMA */
#  define STM32L4_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32L4_NLPTIM                   2   /* Two low-power timers, LPTIM1-2 */
#  define STM32L4_NRNG                     1   /* Random number generator (RNG) */
#  define STM32L4_NUART                    2   /* UART 4-5 */
#  define STM32L4_NUSART                   3   /* USART 1-3 */
#  define STM32L4_NLPUART                  1   /* LPUART 1 */
#  define STM32L4_QSPI                     1   /* QuadSPI1 */
#  define STM32L4_NSPI                     3   /* SPI1-3 */
#  define STM32L4_NI2C                     3   /* I2C1-3 */
#  define STM32L4_NSWPMI                   1   /* SWPMI1 */
#  define STM32L4_NUSBOTGFS                1   /* USB OTG FS */
#  define STM32L4_NUSBFS                   0   /* No USB FS */
#  define STM32L4_NCAN                     1   /* CAN1 */
#  define STM32L4_NSAI                     2   /* SAI1-2 */
#  define STM32L4_NSDMMC                   1   /* SDMMC interface */
#  define STM32L4_NDMA                     2   /* DMA1-2 */
#  define STM32L4_NPORTS                   8   /* 8 GPIO ports, GPIOA-H */
#  define STM32L4_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define STM32L4_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32L4_NCRC                     1   /* CRC */
#  define STM32L4_NCOMP                    2   /* Comparators */
#  define STM32L4_NOPAMP                   2   /* Operational Amplifiers */
#endif /* CONFIG_STM32L4_STM32L4X5 */

#if defined(CONFIG_STM32L4_STM32L4X6)
#  define STM32L4_NFSMC                    1   /* Have FSMC memory controller */
#  define STM32L4_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32L4_NGTIM32                  2   /* 32-bit general timers TIM2 and 5 with DMA */
#  define STM32L4_NGTIM16                  2   /* 16-bit general timers TIM3 and 4 with DMA */
#  define STM32L4_NGTIMNDMA                3   /* 16-bit general timers TIM15-17 without DMA */
#  define STM32L4_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32L4_NLPTIM                   2   /* Two low-power timers, LPTIM1-2 */
#  define STM32L4_NRNG                     1   /* Random number generator (RNG) */
#  define STM32L4_NUART                    2   /* UART 4-5 */
#  define STM32L4_NUSART                   3   /* USART 1-3 */
#  define STM32L4_NLPUART                  1   /* LPUART 1 */
#  define STM32L4_QSPI                     1   /* QuadSPI1 */
#  define STM32L4_NSPI                     3   /* SPI1-3 */
#if defined(CONFIG_STM32L4_STM32L496XX)
#  define STM32L4_NI2C                     4   /* I2C1-4 */
#else
#  define STM32L4_NI2C                     3   /* I2C1-3 */
#endif
#  define STM32L4_NSWPMI                   1   /* SWPMI1 */
#  define STM32L4_NUSBOTGFS                1   /* USB OTG FS */
#  define STM32L4_NUSBFS                   0   /* No USB FS */
#if defined(CONFIG_STM32L4_STM32L496XX)
#  define STM32L4_NCAN                     2   /* CAN1-2 */
#else
#  define STM32L4_NCAN                     1   /* CAN1 */
#endif
#  define STM32L4_NSAI                     2   /* SAI1-2 */
#  define STM32L4_NSDMMC                   1   /* SDMMC interface */
#  define STM32L4_NDMA                     2   /* DMA1-2 */
#if defined(CONFIG_STM32L4_STM32L496XX)
#  define STM32L4_NPORTS                   9   /* 9 GPIO ports, GPIOA-I */
#else
#  define STM32L4_NPORTS                   8   /* 8 GPIO ports, GPIOA-H */
#endif
#  define STM32L4_NADC                     3   /* 12-bit ADC1-3, up to 24 channels */
#  define STM32L4_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32L4_NCRC                     1   /* CRC */
#  define STM32L4_NCOMP                    2   /* Comparators */
#  define STM32L4_NOPAMP                   2   /* Operational Amplifiers */
#endif /* CONFIG_STM32L4_STM32L4X6 */

#if defined(CONFIG_STM32L4_STM32L451XX) || defined(CONFIG_STM32L4_STM32L452XX) || \
    defined(CONFIG_STM32L4_STM32L462XX)
#  define STM32L4_NFSMC                    0   /* No FSMC memory controller */
#  define STM32L4_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32L4_NGTIM32                  1   /* 32-bit general timer TIM2 with DMA */
#  define STM32L4_NGTIM16                  3   /* 16-bit general timers TIM3, TIM15-16 with DMA */
#  define STM32L4_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
#  define STM32L4_NBTIM                    1   /* One basic timer, TIM6 */
#  define STM32L4_NLPTIM                   2   /* Two low-power timers, LPTIM1-2 */
#  define STM32L4_NRNG                     1   /* Random number generator (RNG) */
#  define STM32L4_NUART                    1   /* UART 4 */
#  define STM32L4_NUSART                   3   /* USART 1-3 */
#  define STM32L4_NLPUART                  1   /* LPUART 1 */
#  define STM32L4_QSPI                     1   /* QuadSPI1 */
#  define STM32L4_NSPI                     3   /* SPI1-3 */
#  define STM32L4_NI2C                     4   /* I2C1-4 */
#  define STM32L4_NSWPMI                   1   /* SWPMI1 */
#  define STM32L4_NUSBOTGFS                0   /* No USB OTG FS */
#if defined(CONFIG_STM32L4_STM32L451XX)
#  define STM32L4_NUSBFS                   0   /* No USB FS */
#else
#  define STM32L4_NUSBFS                   1   /* USB FS */
#endif
#  define STM32L4_NCAN                     1   /* CAN1 */
#  define STM32L4_NSAI                     1   /* SAI1 */
#if defined(CONFIG_STM32L4_HAVE_SDMMC1)
#  define STM32L4_NSDMMC                   1   /* SDMMC interface */
#else
#  define STM32L4_NSDMMC                   0   /* No SDMMC interface */
#endif
#  define STM32L4_NDMA                     2   /* DMA1-2 */
#  define STM32L4_NPORTS                   8   /* 8 GPIO ports, GPIOA-H */
#  define STM32L4_NADC                     1   /* 12-bit ADC1, 16 channels (10 in CE,CV) */
#  define STM32L4_NDAC                     1   /* 12-bit DAC1 */
#  define STM32L4_NCRC                     1   /* CRC */
#  define STM32L4_NCOMP                    2   /* Comparators */
#  define STM32L4_NOPAMP                   1   /* Operational Amplifiers */
#endif /* CONFIG_STM32L4_STM32L451XX */

#if defined(CONFIG_STM32L4_STM32L432XX)
#  define STM32L4_NFSMC                    0   /* No FSMC memory controller */
#  define STM32L4_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32L4_NGTIM32                  1   /* 32-bit general timer TIM2 with DMA */
#  define STM32L4_NGTIM16                  2   /* 16-bit general timers TIM15-16 with DMA */
#  define STM32L4_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
#  define STM32L4_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32L4_NLPTIM                   2   /* Two low-power timers, LPTIM1-2 */
#  define STM32L4_NRNG                     1   /* Random number generator (RNG) */
#  define STM32L4_NUART                    0   /* No UART */
#  define STM32L4_NUSART                   2   /* USART 1-2 */
#  define STM32L4_NLPUART                  1   /* LPUART 1 */
#  define STM32L4_QSPI                     1   /* QuadSPI1 */
#  define STM32L4_NSPI                     2   /* SPI1, SPI3 */
#  define STM32L4_NI2C                     2   /* I2C1, I2C3 */
#  define STM32L4_NSWPMI                   1   /* SWPMI1 */
#  define STM32L4_NUSBOTGFS                0   /* No USB OTG FS */
#  define STM32L4_NUSBFS                   1   /* USB FS */
#  define STM32L4_NCAN                     1   /* CAN1 */
#  define STM32L4_NSAI                     1   /* SAI1 */
#  define STM32L4_NSDMMC                   0   /* No SDMMC interface */
#  define STM32L4_NDMA                     2   /* DMA1-2 */
#  define STM32L4_NPORTS                   8   /* 8 GPIO ports, GPIOA-H */
#  define STM32L4_NADC                     1   /* 12-bit ADC1, 10 channels */
#  define STM32L4_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32L4_NCRC                     1   /* CRC */
#  define STM32L4_NCOMP                    2   /* Comparators */
#  define STM32L4_NOPAMP                   1   /* Operational Amplifiers */
#endif /* CONFIG_STM32L4_STM32L432XX */

#if defined(CONFIG_STM32L4_STM32L433XX)
#  define STM32L4_NFSMC                    0   /* No FSMC memory controller */
#  define STM32L4_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32L4_NGTIM32                  1   /* 32-bit general timer TIM2 with DMA */
#  define STM32L4_NGTIM16                  2   /* 16-bit general timers TIM15-16 with DMA */
#  define STM32L4_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
#  define STM32L4_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32L4_NLPTIM                   2   /* Two low-power timers, LPTIM1-2 */
#  define STM32L4_NRNG                     1   /* Random number generator (RNG) */
#  define STM32L4_NUART                    0   /* No UART */
#  define STM32L4_NUSART                   3   /* USART 1-3 */
#  define STM32L4_NLPUART                  1   /* LPUART 1 */
#  define STM32L4_QSPI                     1   /* QuadSPI1 */
#  define STM32L4_NSPI                     3   /* SPI1-SPI3 */
#  define STM32L4_NI2C                     3   /* I2C1-I2C3 */
#  define STM32L4_NSWPMI                   1   /* SWPMI1 */
#  define STM32L4_NUSBOTGFS                0   /* No USB OTG FS */
#  define STM32L4_NUSBFS                   1   /* USB FS */
#  define STM32L4_NCAN                     1   /* CAN1 */
#  define STM32L4_NSAI                     1   /* SAI1 */
#  define STM32L4_NSDMMC                   1   /* SDMMC interface */
#  define STM32L4_NDMA                     2   /* DMA1-2 */
#  define STM32L4_NPORTS                   8   /* 8 GPIO ports, GPIOA-H */
#  define STM32L4_NADC                     1   /* 12-bit ADC1, 10 channels */
#  define STM32L4_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32L4_NCRC                     1   /* CRC */
#  define STM32L4_NCOMP                    2   /* Comparators */
#  define STM32L4_NOPAMP                   1   /* Operational Amplifiers */
#endif /* CONFIG_STM32L4_STM32L433XX */

#if defined(CONFIG_STM32L4_STM32L412XX) || defined(CONFIG_STM32L4_STM32L422XX)
#  define STM32L4_NFSMC                    0   /* No FSMC memory controller */
#  define STM32L4_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32L4_NGTIM32                  1   /* 32-bit general timer TIM2 with DMA */
#  define STM32L4_NGTIM16                  2   /* 16-bit general timers TIM15-16 with DMA */
#  define STM32L4_NGTIMNDMA                0   /* No 16-bit general timers without DMA */
#  define STM32L4_NBTIM                    1   /* One basic timer, TIM6 */
#  define STM32L4_NLPTIM                   2   /* Two low-power timers, LPTIM1-2 */
#  define STM32L4_NRNG                     1   /* Random number generator (RNG) */
#  define STM32L4_NUART                    0   /* No UART */
#  define STM32L4_NUSART                   3   /* USART 1-3 */
#  define STM32L4_NLPUART                  1   /* LPUART 1 */
#  define STM32L4_QSPI                     1   /* QuadSPI1 */
#  define STM32L4_NSPI                     3   /* SPI1-SPI3 */
#  define STM32L4_NI2C                     3   /* I2C1-I2C3 */
#  define STM32L4_NSWPMI                   0   /* No SWPMI */
#  define STM32L4_NUSBOTGFS                0   /* No USB OTG FS */
#  define STM32L4_NUSBFS                   1   /* USB FS */
#  define STM32L4_NCAN                     0   /* No CAN */
#  define STM32L4_NSAI                     0   /* No SAI */
#  define STM32L4_NSDMMC                   0   /* No SDMMC interface */
#  define STM32L4_NDMA                     2   /* DMA1-2 */
#  define STM32L4_NPORTS                   8   /* 8 GPIO ports, GPIOA-H */
#  define STM32L4_NADC                     2   /* 12-bit ADC1-2, 10 channels */
#  define STM32L4_NDAC                     0   /* No DAC */
#  define STM32L4_NCRC                     1   /* CRC */
#  define STM32L4_NCOMP                    2   /* Comparators */
#  define STM32L4_NOPAMP                   1   /* Operational Amplifiers */
#endif /* CONFIG_STM32L4_STM32L412XX || CONFIG_STM32L4_STM32L422XX */

/* NVIC priority levels *****************************************************/

/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32L4_CHIP_H */
