/****************************************************************************
 * arch/arm/include/stm32wb/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32WB_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32WB_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define STM32WB_NFSMC               0   /* No FSMC */
#define STM32WB_NBTIM               0   /* No basic timers */
#define STM32WB_NATIM               1   /* One advanced timers TIM1 */
#define STM32WB_NGTIM32             1   /* 32-bit general timers TIM2 with DMA */
#define STM32WB_NLPTIM              2   /* Two low-power timers, LPTIM1-2 */
#define STM32WB_NGTIMNDMA           0   /* No general timers without DMA */

#if defined(CONFIG_STM32WB_STM32WB30) || defined(CONFIG_STM32WB_STM32WB50) \
    || defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_NGTIM16           2   /* 16-bit general timers TIM16-17 with DMA */
#else
#  define STM32WB_NGTIM16           0   /* No 16-bit general timers */
#endif

#if defined(CONFIG_STM32WB_STM32WB35) || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_NDMA              2   /* DMA1-2 with 7 channels each */
#  define STM32WB_NI2S              1   /* SAI1 (dual channel high quality audio) */
#  define STM32WB_NI2C              2   /* I2C1, I2C3 */
#  define STM32WB_NUSBOTG           1   /* USB 2.0 FS */
#  define STM32WB_NCMP              2   /* Two Comparators */
#  if defined(CONFIG_STM32WB_IO_CONFIG_R) || defined(CONFIG_STM32WB_IO_CONFIG_V)
#    define STM32WB_NSPI            3   /* SPI1-2, QSPI */
#  else
#    define STM32WB_NSPI            2   /* SPI1, QSPI */
#  endif
#else
#  define STM32WB_NDMA              1   /* DMA1 with 7 channels */
#  define STM32WB_NI2S              0   /* No SAI */
#  define STM32WB_NI2C              1   /* I2C1 */
#  define STM32WB_NUSBOTG           0   /* No USB */
#  define STM32WB_NCMP              0   /* No Comparators */
#  define STM32WB_NSPI              1   /* SPI1 */
#endif

#if defined(CONFIG_STM32WB_STM32WB15) || defined(CONFIG_STM32WB_STM32WB35) \
    || defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_NLPUART           1   /* LPUART1 */
#else
#  define STM32WB_NLPUART           0   /* No LPUART */
#endif

#if defined(CONFIG_STM32WB_IO_CONFIG_R) || defined(CONFIG_STM32WB_IO_CONFIG_V)
#  define STM32WB_NCAPSENSE         18  /* Capacitive sensing channels */
#else
#  define STM32WB_NCAPSENSE         0   /* No Capacitive sensing */
#endif

#if defined(CONFIG_STM32WB_STM32WB55)
#  define STM32WB_NLCD              1   /* One LCD controller with up to 8x40
                                         * terminals, depending on subfamily.
                                         * 55Cx: 4x13
                                         * 55Rx: 4x28
                                         * 55Vx: 4x44, 8x40 */
#else
#  define STM32WB_NLCD              0   /* No LCD */
#endif

#define STM32WB_NUSART              1   /* USART1 */
#define STM32WB_NCAN                0   /* No CAN */
#define STM32WB_NSDIO               0   /* No SDIO interface */
#define STM32WB_NADC                1   /* ADC1, up to 19-channels */
#define STM32WB_NDAC                0   /* No DAC */
#define STM32WB_NCRC                1   /* CRC */
#define STM32WB_NETHERNET           0   /* No ethernet */
#define STM32WB_NRNG                1   /* Random number generator (RNG) */
#define STM32WB_NDCMI               0   /* No digital camera interface (DCMI) */

#if defined(CONFIG_STM32WB_IO_CONFIG_C)
#  define STM32WB_NGPIO             30  /* GPIO[A,B,C,E,H] */
#elif defined(CONFIG_STM32WB_IO_CONFIG_C_48E)
#  define STM32WB_NGPIO             37  /* GPIO[A,B,C,E,H] */
#elif defined(CONFIG_STM32WB_IO_CONFIG_C_49)
#  define STM32WB_NGPIO             25  /* GPIO[A,B,C,H] */
#elif defined(CONFIG_STM32WB_IO_CONFIG_R)
#  define STM32WB_NGPIO             49  /* GPIO[A,B,C,D,E,H] */
#elif defined(CONFIG_STM32WB_IO_CONFIG_V)
#  define STM32WB_NGPIO             72  /* GPIO[A,B,C,D,E,H] */
#else
#  error "Unsupported STM32WB chip"
#endif

/* STM32WB1xCC have 48 Kib:
 *   1) 12 KiB of SRAM1 beginning at address 0x2000:0000 - 0x2000:3000
 *   2) 32 KiB of SRAM2a beginning at address 0x2003:0000 - 0x2003:8000
 *   3) 4 KiB of SRAM2b beginning at address 0x2003:8000 - 0x2003:9000
 *
 * STM32WB3xxx have 96 Kib:
 *
 *   1) 32 KiB of SRAM1 beginning at address 0x2000:0000 - 0x2000:8000
 *   2) 32 KiB of SRAM2a beginning at address 0x2003:0000 - 0x2003:8000
 *   3) 32 KiB of SRAM2b beginning at address 0x2003:8000 - 0x2004:0000
 *
 * STM32WB50CG and STM32WB55xC have 128 Kib:
 *
 *   1) 64 KiB of SRAM1 beginning at address 0x2000:0000 - 0x2001:0000
 *   2) 32 KiB of SRAM2a beginning at address 0x2003:0000 - 0x2003:8000
 *   3) 32 KiB of SRAM2b beginning at address 0x2003:8000 - 0x2004:0000
 *
 * STM32WB55x[E,Y,G] have 256 Kib:
 *
 *   1) 192 KiB of SRAM1 beginning at address 0x2000:0000 - 0x2001:8000
 *   2) 32 KiB of SRAM2a beginning at address 0x2003:0000 - 0x2003:8000
 *   3) 32 KiB of SRAM2b beginning at address 0x2003:8000 - 0x2004:0000
 */

#if defined(CONFIG_STM32WB_STM32WB10) || defined(CONFIG_STM32WB_STM32WB15)
#  define STM32WB_SRAM1_SIZE        (12*1024)
#  define STM32WB_SRAM2A_SIZE       (32*1024)
#  define STM32WB_SRAM2B_SIZE       (4*1024)
#elif defined(CONFIG_STM32WB_STM32WB30) || defined(CONFIG_STM32WB_STM32WB35)
#  define STM32WB_SRAM1_SIZE        (32*1024)
#  define STM32WB_SRAM2A_SIZE       (32*1024)
#  define STM32WB_SRAM2B_SIZE       (32*1024)
#elif (defined(CONFIG_STM32WB_STM32WB50) || defined(CONFIG_STM32WB_STM32WB55)) \
      && defined(CONFIG_STM32WB_IO_CONFIG_C)
#  define STM32WB_SRAM1_SIZE        (64*1024)
#  define STM32WB_SRAM2A_SIZE       (32*1024)
#  define STM32WB_SRAM2B_SIZE       (32*1024)
#elif defined(CONFIG_STM32WB_STM32WB55) && \
      (defined(CONFIG_STM32WB_IO_CONFIG_R) || defined(CONFIG_STM32WB_IO_CONFIG_V))
#  define STM32WB_SRAM1_SIZE        (192*1024)
#  define STM32WB_SRAM2A_SIZE       (32*1024)
#  define STM32WB_SRAM2B_SIZE       (32*1024)
#else
#  error "Unsupported STM32WB chip"
#endif

/* NVIC priority levels *****************************************************/

/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN      0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT  0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX      0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP     0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32WB_CHIP_H */
