/****************************************************************************
 * arch/arm/include/stm32l1/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32L1_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32L1_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported chip and provide alternate function
 * pin-mapping
 *
 * NOTE: Each GPIO pin may serve either for general purpose I/O or for a
 * special alternate function (such as USART, CAN, USB, SDIO, etc.).  That
 * particular pin-mapping will depend on the package and STM32 family.  If
 * you are incorporating a new STM32 chip into NuttX, you will need to add
 * the pin-mapping to a header file and to include that header file below.
 * The chip-specific pin-mapping is defined in the chip datasheet.
 */

/* STM32L EnergyLite Line ***************************************************/

/* STM32L151XX -- No LCD
 * STM32L152XX -- With LCD
 *
 * STM32L15XCX -- 48-pins
 * STM32L15XRX -- 64-pins
 * STM32L15XVX -- 100-pins
 * STM32L15XZX -- 144-pins
 *
 * STM32L15XX6 -- 32KB FLASH, 10KB SRAM, 4KB EEPROM
 * STM32L15XX8 -- 64KB FLASH, 10KB SRAM, 4KB EEPROM
 * STM32L15XXB -- 128KB FLASH, 16KB SRAM, 4KB EEPROM
 *
 * STM32L15XXC -- 256KB FLASH, 32KB SRAM, 8KB EEPROM (medium+ density)
 *
 * STM32L16XXD -- 384KB FLASH, 48KB SRAM, 12KB EEPROM (high density)
 * STM32L16XXE -- 512KB FLASH, 80KB SRAM, 16KB EEPROM (high density)
 */

#if defined(CONFIG_ARCH_CHIP_STM32L151C6) || defined(CONFIG_ARCH_CHIP_STM32L151C8) || \
    defined(CONFIG_ARCH_CHIP_STM32L151CB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    37  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 14-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                13  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151R6) || defined(CONFIG_ARCH_CHIP_STM32L151R8) || \
      defined(CONFIG_ARCH_CHIP_STM32L151RB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    51  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 20-channels */
#  define STM32_NDAC                     2   /* DAC , 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151V6) || defined(CONFIG_ARCH_CHIP_STM32L151V8) || \
      defined(CONFIG_ARCH_CHIP_STM32L151VB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 24-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152C6) || defined(CONFIG_ARCH_CHIP_STM32L152C8) || \
      defined(CONFIG_ARCH_CHIP_STM32L152CB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x18 */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    37  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 14-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                13  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152R6) || defined(CONFIG_ARCH_CHIP_STM32L152R8) || \
      defined(CONFIG_ARCH_CHIP_STM32L152RB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x32, 8x28 */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    51  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 20-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152V6) || defined(CONFIG_ARCH_CHIP_STM32L152V8) || \
      defined(CONFIG_ARCH_CHIP_STM32L152VB)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     1   /* DMA1, 7-channels */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 24-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                20  /* Capacitive sensing channels */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No Ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152CC)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x18 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    37  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 14-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                16  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152RC)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x32, 8x28 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    51  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 21-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                23  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L152VC)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                23  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151RE) || defined(CONFIG_ARCH_CHIP_STM32L152RE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    51  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                23  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151VE) || defined(CONFIG_ARCH_CHIP_STM32L152VE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                23  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151QE) || defined(CONFIG_ARCH_CHIP_STM32L152QE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    109 /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                33  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L151ZE) || defined(CONFIG_ARCH_CHIP_STM32L152ZE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    3   /* 16-bit general up/down timers TIM2-4 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    115 /* GPIOA-E,H */
#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                34  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L162ZD)
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-4 with DMA
                                              * 32-bit general timer TIM5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1, 7-channels, DMA2 (5 channels) */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-3, UART4-5 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40 */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    115 /* GPIOA-G,H */
#  define STM32_NADC                     1   /* ADC1, 40-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                34  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L162VE)
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    0   /* No advanced timers */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2-4 with DMA
                                              * 32-bit general timer TIM5 with DMA */
#  define STM32_NGTIMNDMA                3   /* 16-bit general timers TIM9-11 without DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 with DMA */
#  define STM32_NDMA                     2   /* DMA1, 12-channels */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2, overlapping with SPI2-3 */
#  define STM32_NUSART                   5   /* USART1-3, UART4-5 */
#  define STM32_NLPUART                  0   /* No LPUART */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NLCD                     1   /* LCD 4x44, 8x40*/
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS (only USB 2.0 device) */
#  define STM32_NGPIO                    83  /* GPIOA-G,H */

#  define STM32_NADC                     1   /* ADC1, 25-channels */
#  define STM32_NDAC                     2   /* DAC 1, 2 channels */
#  define STM32_NCMP                     2   /* (2) Comparators */
#  define STM32_NCAPSENSE                23  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F100 Value Line ****************************************************/

#else
#  error "Unsupported STM32 chip"
#endif

/* Peripheral IP versions ***************************************************/

/* Peripheral IP versions are invariant and should be decided here, not in
 * Kconfig.
 *
 * REVISIT: Currently only SPI IP version is handled here, with others being
 *          handled in Kconfig. Those others need to be gradually refactored
 *          and resolved here.
 */

#define STM32_HAVE_IP_SPI_V1

/* NVIC priority levels *****************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32L1_CHIP_H */
