/************************************************************************************
 * arch/arm/include/stm32f7/chip.h
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Bob Feretich <bob.feretich@rafresearch.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_STM32F7_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32F7_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* STM32F722xx, STM32F723xx,
 * STM32F745xx, STM32F746xx, STM32F756xx, STM32F765xx, STM32F767xx, STM32F768xx,
 * STM32F769xx, STM32F777xx and STM32F779xx  Differences between family members:
 *
 *   ----------- ---------------- ----- ---- ----- ---- ---- ---- ---- ---- ----- ----- ---- ------------ ------
 *                                       SPI   ADC LCD
 *   PART        PACKAGE          GPIOs  I2S  CHAN TFT  MIPI JPEG CAN  ETH  DFSDM CRYPTO FPU      RAM      L1
 *   ----------- ---------------- ----- ---- ----- ---- ---- ---- ---- ---- ----- ----- ---- ------------ ------
 *   STM32F722Rx LQFP64             50   3/3   16   No   No   No   1   No    No    No   SFPU (176+16+64)  8+8
 *   STM32F722Vx LQFP100            82   4/3   16   No   No   No   1   No    No    No   SFPU (176+16+64)  8+8
 *   STM32F722Zx LQFP144           114   5/3   24   No   No   No   1   No    No    No   SFPU (176+16+64)  8+8
 *   STM32F722Ix UFBGA176/LQFP176  140   5/3   24   No   No   No   1   No    No    No   SFPU (176+16+64)  8+8
 *
 *   STM32F723Vx WLCSP100           79   4/3   16   No   No   No   1   No    No    No   SFPU (176+16+64)  8+8
 *   STM32F723Zx UFBGA144/LQFP144  112   5/3   24   No   No   No   1   No    No    No   SFPU (176+16+64)  8+8
 *   STM32F723Ix UFBGA176/LQFP176  138   5/3   24   No   No   No   1   No    No    No   SFPU (176+16+64)  8+8
 *
 *   STM32F745Vx LQFP100            82   4/3   16   No   No   No   2   Yes   No    No   SFPU (240+16+64)  4+4
 *   STM32F745Zx WLCSP143/LQFP144  114   6/3   24   No   No   No   2   Yes   No    No   SFPU (240+16+64)  4+4
 *   STM32F745Ix UFBGA176/LQFP176  140   6/3   24   No   No   No   2   Yes   No    No   SFPU (240+16+64)  4+4
 *   STM32F745Bx LQFP208           168   6/3   24   No   No   No   2   Yes   No    No   SFPU (240+16+64)  4+4
 *   STM32F745Nx TFBGA216           68   6/3   24   No   No   No   2   Yes   No    No   SFPU (240+16+64)  4+4
 *
 *   STM32F746Vx LQFP100            82   4/3   16   Yes  No   No   2   Yes   No    No   SFPU (240+16+64)  4+4
 *   STM32F746Zx WLCSP143/LQFP144  114   6/3   24   Yes  No   No   2   Yes   No    No   SFPU (240+16+64)  4+4
 *   STM32F746Ix UFBGA176/LQFP176  140   6/3   24   Yes  No   No   2   Yes   No    No   SFPU (240+16+64)  4+4
 *   STM32F746Bx LQFP208           168   6/3   24   Yes  No   No   2   Yes   No    No   SFPU (240+16+64)  4+4
 *   STM32F746Nx TFBGA216          168   6/3   24   Yes  No   No   2   Yes   No    No   SFPU
 *
 *   STM32F756Vx LQFP100            82   4/3   16   Yes  No   No   2   Yes   No   Yes   SFPU (240+16+64)  4+4
 *   STM32F756Zx WLCSP143/LQFP144  114   6/3   24   Yes  No   No   2   Yes   No   Yes   SFPU (240+16+64)  4+4
 *   STM32F756Ix UFBGA176/LQFP176  140   6/3   24   Yes  No   No   2   Yes   No   Yes   SFPU (240+16+64)  4+4
 *   STM32F756Bx LQFP208           168   6/3   24   Yes  No   No   2   Yes   No   Yes   SFPU (240+16+64)  4+4
 *   STM32F756Nx TFBGA216          168   6/3   24   Yes  No   No   2   Yes   No   Yes   SFPU (240+16+64)  4+4
 *
 *   STM32F765Vx LQFP100            82   4/3   16   No   No   No   3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F765Zx WLCSP143/LQFP144  114   6/3   24   No   No   No   3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F765Ix UFBGA176/LQFP176  140   6/3   24   No   No   No   3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F765Bx LQFP208           168   6/3   24   No   No   No   3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F765Nx TFBGA216          168   6/3   24   No   No   No   3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *
 *   STM32F767Vx LQFP100            82   4/3   16   Yes  No   Yes  3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F767Zx WLCSP143/LQFP144  114   6/3   24   Yes  No   Yes  3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F767Ix UFBGA176/LQFP176  132   6/3   24   Yes  Yes  Yes  3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F767Bx LQFP208           168   6/3   24   Yes  Yes  Yes  3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F767Nx TFBGA216          159   6/3   24   Yes  Yes  Yes  3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *
 *   STM32F768Ax WLCSP180          129   6/3   24   Yes  Yes  Yes  3   No    Yes   No   DFPU (368+16+128) 16+16
 *
 *   STM32F769Vx LQFP100            82   4/3   16   Yes  No   Yes  3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F769Zx LQFP144           114   6/3   24   Yes  No   Yes  3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F769Ix UFBGA176/LQFP176  132   6/3   24   Yes  Yes  Yes  3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F769Bx LQFP208           168   6/3   24   Yes  Yes  Yes  3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *   STM32F769Nx TFBGA216          159   6/3   24   Yes  Yes  Yes  3   Yes   Yes   No   DFPU (368+16+128) 16+16
 *
 *   STM32F769Ax WLCSP180          129   6/3   24   Yes  Yes  Yes  3   No    Yes   No   DFPU (368+16+128) 16+16
 *
 *   STM32F777Vx LQFP100            82   4/3   16   Yes  No   Yes  3   Yes   Yes   Yes  DFPU (368+16+128) 16+16
 *   STM32F777Zx LQFP144           114   6/3   24   Yes  No   Yes  3   Yes   Yes   Yes  DFPU (368+16+128) 16+16
 *   STM32F777Ix UFBGA176/LQFP176  132   6/3   24   Yes  Yes  Yes  3   Yes   Yes   Yes  DFPU (368+16+128) 16+16
 *   STM32F777Bx LQFP208           159   6/3   24   Yes  Yes  Yes  3   Yes   Yes   Yes  DFPU (368+16+128) 16+16
 *   STM32F777Nx TFBGA216          159   6/3   24   Yes  Yes  Yes  3   Yes   Yes   Yes  DFPU (368+16+128) 16+16
 *
 *   STM32F778Ax WLCSP180          129   6/3   24   Yes  Yes  Yes  3   No    Yes   Yes  DFPU (368+16+128) 16+16
 *
 *   STM32F779Ix UFBGA176/LQFP176  132   6/3   24   Yes  Yes  Yes  3   Yes   Yes   Yes  DFPU (368+16+128) 16+16
 *   STM32F779Bx LQFP208           159   6/3   24   Yes  Yes  Yes  3   Yes   Yes   Yes  DFPU (368+16+128) 16+16
 *   STM32F779Nx TFBGA216          159   6/3   24   Yes  Yes  Yes  3   Yes   Yes   Yes  DFPU (368+16+128) 16+16

 *   STM32F779Ax WLCSP180          129   6/3   24   Yes  Yes  Yes  3   No    Yes   Yes  DFPU (368+16+128) 16+16
 *   ----------- ---------------- ----- ---- ----- ---- ---- ---- ---- ---- ----- ----- ---- ------------ ------
 *
 * Parts STM32F72xxC & STM32F73xxC have 256Kb of FLASH
 * Parts STM32F72xxE & STM32F73xxE have 512Kb of FLASH
 * Parts STM32F74xxE have 512Kb of FLASH
 * Parts STM32F74xxG have 1024Kb of FLASH
 * Parts STM32F74xxI have 2048Kb of FLASH
 *
 * The correct FLASH size will be set CONFIG_STM32F7_FLASH_CONFIG_x or overridden
 * with CONFIG_STM32F7_FLASH_OVERRIDE_x
 *
 */
#if defined(CONFIG_ARCH_CHIP_STM32F722RC) || \
  defined(CONFIG_ARCH_CHIP_STM32F722RE) || \
  defined(CONFIG_ARCH_CHIP_STM32F722VC) || \
  defined(CONFIG_ARCH_CHIP_STM32F722VE) || \
  defined(CONFIG_ARCH_CHIP_STM32F722ZC) || \
  defined(CONFIG_ARCH_CHIP_STM32F722ZE) || \
  defined(CONFIG_ARCH_CHIP_STM32F722IC) || \
  defined(CONFIG_ARCH_CHIP_STM32F722IE) || \
  defined(CONFIG_ARCH_CHIP_STM32F723RC) || \
  defined(CONFIG_ARCH_CHIP_STM32F723RE) || \
  defined(CONFIG_ARCH_CHIP_STM32F723VC) || \
  defined(CONFIG_ARCH_CHIP_STM32F723VE) || \
  defined(CONFIG_ARCH_CHIP_STM32F723ZC) || \
  defined(CONFIG_ARCH_CHIP_STM32F723ZE) || \
  defined(CONFIG_ARCH_CHIP_STM32F723IC) || \
  defined(CONFIG_ARCH_CHIP_STM32F723IE) || \
  defined(CONFIG_ARCH_CHIP_STM32F745VG) || \
  defined(CONFIG_ARCH_CHIP_STM32F745VE) || \
  defined(CONFIG_ARCH_CHIP_STM32F745IG) || \
  defined(CONFIG_ARCH_CHIP_STM32F745IE) || \
  defined(CONFIG_ARCH_CHIP_STM32F745ZE) || \
  defined(CONFIG_ARCH_CHIP_STM32F745ZG) || \
  defined(CONFIG_ARCH_CHIP_STM32F746BG) || \
  defined(CONFIG_ARCH_CHIP_STM32F746VG) || \
  defined(CONFIG_ARCH_CHIP_STM32F746VE) || \
  defined(CONFIG_ARCH_CHIP_STM32F746BE) || \
  defined(CONFIG_ARCH_CHIP_STM32F746ZG) || \
  defined(CONFIG_ARCH_CHIP_STM32F746IE) || \
  defined(CONFIG_ARCH_CHIP_STM32F746NG) || \
  defined(CONFIG_ARCH_CHIP_STM32F746NE) || \
  defined(CONFIG_ARCH_CHIP_STM32F746ZE) || \
  defined(CONFIG_ARCH_CHIP_STM32F746IG) || \
  defined(CONFIG_ARCH_CHIP_STM32F756NG) || \
  defined(CONFIG_ARCH_CHIP_STM32F756BG) || \
  defined(CONFIG_ARCH_CHIP_STM32F756IG) || \
  defined(CONFIG_ARCH_CHIP_STM32F756VG) || \
  defined(CONFIG_ARCH_CHIP_STM32F756ZG) || \
  defined(CONFIG_ARCH_CHIP_STM32F765NI) || \
  defined(CONFIG_ARCH_CHIP_STM32F765VI) || \
  defined(CONFIG_ARCH_CHIP_STM32F765VG) || \
  defined(CONFIG_ARCH_CHIP_STM32F765BI) || \
  defined(CONFIG_ARCH_CHIP_STM32F765NG) || \
  defined(CONFIG_ARCH_CHIP_STM32F765ZG) || \
  defined(CONFIG_ARCH_CHIP_STM32F765ZI) || \
  defined(CONFIG_ARCH_CHIP_STM32F765IG) || \
  defined(CONFIG_ARCH_CHIP_STM32F765BG) || \
  defined(CONFIG_ARCH_CHIP_STM32F765II) || \
  defined(CONFIG_ARCH_CHIP_STM32F767NG) || \
  defined(CONFIG_ARCH_CHIP_STM32F767IG) || \
  defined(CONFIG_ARCH_CHIP_STM32F767VG) || \
  defined(CONFIG_ARCH_CHIP_STM32F767ZG) || \
  defined(CONFIG_ARCH_CHIP_STM32F767NI) || \
  defined(CONFIG_ARCH_CHIP_STM32F767VI) || \
  defined(CONFIG_ARCH_CHIP_STM32F767BG) || \
  defined(CONFIG_ARCH_CHIP_STM32F767ZI) || \
  defined(CONFIG_ARCH_CHIP_STM32F767II) || \
  defined(CONFIG_ARCH_CHIP_STM32F769BI) || \
  defined(CONFIG_ARCH_CHIP_STM32F769II) || \
  defined(CONFIG_ARCH_CHIP_STM32F769BG) || \
  defined(CONFIG_ARCH_CHIP_STM32F769NI) || \
  defined(CONFIG_ARCH_CHIP_STM32F769AI) || \
  defined(CONFIG_ARCH_CHIP_STM32F769NG) || \
  defined(CONFIG_ARCH_CHIP_STM32F769IG) || \
  defined(CONFIG_ARCH_CHIP_STM32F777ZI) || \
  defined(CONFIG_ARCH_CHIP_STM32F777VI) || \
  defined(CONFIG_ARCH_CHIP_STM32F777NI) || \
  defined(CONFIG_ARCH_CHIP_STM32F777BI) || \
  defined(CONFIG_ARCH_CHIP_STM32F777II) || \
  defined(CONFIG_ARCH_CHIP_STM32F778AI) || \
  defined(CONFIG_ARCH_CHIP_STM32F779II) || \
  defined(CONFIG_ARCH_CHIP_STM32F779NI) || \
  defined(CONFIG_ARCH_CHIP_STM32F779BI) || \
  defined(CONFIG_ARCH_CHIP_STM32F779AI)
#else
#  error STM32 F7 chip not identified
#endif

/* Size SRAM */

#if defined(CONFIG_STM32F7_STM32F72XX) || defined(CONFIG_STM32F7_STM32F73XX)
#    define STM32F7_SRAM1_SIZE            (176*1024)  /* 176Kb SRAM1 on AHB bus Matrix */
#    define STM32F7_SRAM2_SIZE            (16*1024)   /* 16Kb SRAM2 on AHB bus Matrix */
#  if defined(CONFIG_ARMV7M_HAVE_DTCM)
#      define STM32F7_DTCM_SRAM_SIZE      (64*1024)   /* 64Kb DTCM SRAM on TCM interface */
#  else
#      define STM32F7_DTCM_SRAM_SIZE      (0)         /* No DTCM SRAM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV7M_HAVE_ITCM)
#      define STM32F7_ITCM_SRAM_SIZE      (16*1024)   /* 16Kb ITCM SRAM on TCM interface */
#  else
#      define STM32F7_ITCM_SRAM_SIZE      (0)         /* No ITCM SRAM on TCM interface */
#  endif
#elif defined(CONFIG_STM32F7_STM32F74XX) || defined(CONFIG_STM32F7_STM32F75XX)
#    define STM32F7_SRAM1_SIZE            (240*1024)  /* 240Kb SRAM1 on AHB bus Matrix */
#    define STM32F7_SRAM2_SIZE            (16*1024)   /* 16Kb SRAM2 on AHB bus Matrix */
#  if defined(CONFIG_ARMV7M_HAVE_DTCM)
#      define STM32F7_DTCM_SRAM_SIZE      (64*1024)   /* 64Kb DTCM SRAM on TCM interface */
#  else
#      define STM32F7_DTCM_SRAM_SIZE      (0)         /* No DTCM SRAM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV7M_HAVE_ITCM)
#      define STM32F7_ITCM_SRAM_SIZE      (16*1024)   /* 16Kb ITCM SRAM on TCM interface */
#  else
#      define STM32F7_ITCM_SRAM_SIZE      (0)         /* No ITCM SRAM on TCM interface */
#  endif
#elif defined(CONFIG_STM32F7_STM32F76XX) || defined(CONFIG_STM32F7_STM32F77XX)
#    define STM32F7_SRAM1_SIZE            (368*1024)  /* 368Kb SRAM1 on AHB bus Matrix */
#    define STM32F7_SRAM2_SIZE            (16*1024)   /* 16Kb SRAM2 on AHB bus Matrix */
#  if defined(CONFIG_ARMV7M_HAVE_DTCM)
#      define STM32F7_DTCM_SRAM_SIZE      (128*1024)  /* 128Kb DTCM SRAM on TCM interface */
#  else
#      define STM32F7_DTCM_SRAM_SIZE      (0)         /* No DTCM SRAM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV7M_HAVE_ITCM)
#      define STM32F7_ITCM_SRAM_SIZE      (16*1024)   /* 16Kb ITCM SRAM on TCM interface */
#  else
#      define STM32F7_ITCM_SRAM_SIZE      (0)         /* No ITCM SRAM on TCM interface */
#  endif
#else
#  error STM32 F7 chip Family not identified
#endif

/* Common to all Advanced (vs Foundation) Family members */

#if defined(CONFIG_STM32F7_STM32F72XX) || defined(CONFIG_STM32F7_STM32F73XX)
#      define STM32F7_NSPDIFRX                 0   /* Not supported */
#      define STM32F7_NGPIO                    9   /* 9 GPIO ports, GPIOA-I */
#      define STM32F7_NI2C                     3   /* I2C1-3 */
#else
#      define STM32F7_NSPDIFRX                 4   /* 4 SPDIFRX inputs */
#      define STM32F7_NGPIO                   11   /* 11 GPIO ports, GPIOA-K */
#      define STM32F7_NI2C                     4   /* I2C1-4 */
#endif

/* Common to all Family members */

#  define STM32F7_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32F7_NGTIM32                  2   /* 32-bit general timers TIM2 and 5 with DMA */
#  define STM32F7_NGTIM16                  2   /* 16-bit general timers TIM3 and 4 with DMA */
#  define STM32F7_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32F7_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32F7_NUART                    4   /* UART 4-5 and 7-8 */
#  define STM32F7_NUSART                   4   /* USART1-3 and 6 */
#  define STM32F7_NI2S                     3   /* I2S1-2 (multiplexed with SPI1-3) */
#  define STM32F7_NUSBOTGFS                1   /* USB OTG FS */
#  define STM32F7_NUSBOTGHS                1   /* USB OTG HS */
#  define STM32F7_NSAI                     2   /* SAI1-2 */
#  define STM32F7_NDMA                     2   /* DMA1-2 */
#  define STM32F7_NADC                     3   /* 12-bit ADC1-3, number of channels vary */
#  define STM32F7_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32F7_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32F7_NCRC                     1   /* CRC */

/* TBD FPU Configuration */

#if defined(CONFIG_ARCH_HAVE_FPU)
#else
#endif

#if defined(CONFIG_ARCH_HAVE_DPFPU)
#else
#endif

/* Diversification based on Family and package */

#if defined(CONFIG_STM32F7_HAVE_FMC)
#  define STM32F7_NFMC                     1   /* Have FMC memory controller */
#else
#  define STM32F7_NFMC                     0   /* No FMC memory controller */
#endif
#if defined(CONFIG_STM32F7_HAVE_ETHRNET)
#  define STM32F7_NETHERNET                1   /* 100/100 Ethernet MAC */
#else
#  define STM32F7_NETHERNET                0   /* No 100/100 Ethernet MAC */
#endif
#if defined(CONFIG_STM32F7_HAVE_RNG)
#  define STM32F7_NRNG                     1   /* Random number generator (RNG) */
#else
#  define STM32F7_NRNG                     0   /* No Random number generator (RNG) */
#endif

#if defined(CONFIG_STM32F7_HAVE_SPI5) && defined(CONFIG_STM32F7_HAVE_SPI6)
#  define STM32F7_NSPI                     6   /* SPI1-6 (Advanced Family Except V series) */
#elif defined(CONFIG_STM32F7_HAVE_SPI5)
#  define STM32F7_NSPI                     5   /* SPI1-5 (Foundation Family Except V & R series) */
#elif defined(CONFIG_STM32F7_HAVE_SPI4)
#  define STM32F7_NSPI                     4   /* SPI1-4 V series */
#else
#  define STM32F7_NSPI                     3   /* SPI1-3 R series */
#endif

#if defined(CONFIG_STM32F7_HAVE_SDMMC2)
#  define STM32F7_NSDMMC                   2   /* 2 SDMMC interfaces */
#else
#  define STM32F7_NSDMMC                   1   /* 1 SDMMC interface */
#endif
#if defined(CONFIG_STM32F7_HAVE_CAN3)
#  define STM32F7_NCAN                     3   /* CAN1-3 */
#elif defined(CONFIG_STM32F7_HAVE_CAN2)
#  define STM32F7_NCAN                     2   /* CAN1-2 */
#else
#  define STM32F7_NCAN                     1   /* CAN1 only */
#endif
#if defined(CONFIG_STM32F7_HAVE_DCMI)
#  define STM32F7_NDCMI                    1   /* Digital camera interface (DCMI) */
#else
#  define STM32F7_NDCMI                    0   /* No Digital camera interface (DCMI) */
#endif
#if defined(CONFIG_STM32F7_HAVE_DSIHOST)
#  define STM32F7_NDSIHOST                 1   /* Have MIPI DSI Host */
#else
#  define STM32F7_NDSIHOST                 0   /* No MIPI DSI Host */
#endif
#if defined (CONFIG_STM32F7_HAVE_LTDC)
#  define STM32F7_NLCDTFT                  1   /* One LCD-TFT */
#else
#  define STM32F7_NLCDTFT                  0   /* No LCD-TFT */
#endif
#if defined(CONFIG_STM32F7_HAVE_DMA2D)         /* bf20171107 Swapped defines they were reversed. */
#  define STM32F7_NDMA2D                   1   /* DChrom-ART Accelerator™ (DMA2D) */
#else
#  define STM32F7_NDMA2D                   0   /* No DChrom-ART Accelerator™ (DMA2D) */
#endif
#if defined(CONFIG_STM32F7_HAVE_JPEG)
#define STM32F7_NJPEG                      1   /* One JPEG Converter */
#else
#define STM32F7_NJPEG                      0   /* No JPEG Converter */
#endif
#if defined(CONFIG_STM32F7_HAVE_CRYP)
#define STM32F7_NCRYP                      1   /* One CRYP engine */
#else
#define STM32F7_NCRYP                      0   /* No  CRYP engine */
#endif
#if defined(CONFIG_STM32F7_HAVE_HASH)
#define STM32F7_NHASH                      1   /* One HASH engine */
#else
#define STM32F7_NHASH                      0   /* No HASH engine */
#endif
#if defined(CONFIG_STM32F7_HAVE_DFSDM)
#define STM32F7_NDFSDM                     4   /* One set of 4 Digital filters */
#else
#define STM32F7_NDFSDM                     0   /* No Digital filters */
#endif

/* NVIC priority levels *************************************************************/
/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

/* If CONFIG_ARMV7M_USEBASEPRI is selected, then interrupts will be disabled
 * by setting the BASEPRI register to NVIC_SYSH_DISABLE_PRIORITY so that most
 * interrupts will not have execution priority.  SVCall must have execution
 * priority in all cases.
 *
 * In the normal cases, interrupts are not nest-able and all interrupts run
 * at an execution priority between NVIC_SYSH_PRIORITY_MIN and
 * NVIC_SYSH_PRIORITY_MAX (with NVIC_SYSH_PRIORITY_MAX reserved for SVCall).
 *
 * If, in addition, CONFIG_ARCH_HIPRI_INTERRUPT is defined, then special
 * high priority interrupts are supported.  These are not "nested" in the
 * normal sense of the word.  These high priority interrupts can interrupt
 * normal processing but execute outside of OS (although they can "get back
 * into the game" via a PendSV interrupt).
 *
 * In the normal course of things, interrupts must occasionally be disabled
 * using the up_irq_save() inline function to prevent contention in use of
 * resources that may be shared between interrupt level and non-interrupt
 * level logic.  Now the question arises, if CONFIG_ARCH_HIPRI_INTERRUPT,
 * do we disable all interrupts (except SVCall), or do we only disable the
 * "normal" interrupts.  Since the high priority interrupts cannot interact
 * with the OS, you may want to permit the high priority interrupts even if
 * interrupts are disabled.  The setting CONFIG_ARCH_INT_DISABLEALL can be
 * used to select either behavior:
 *
 *   ----------------------------+--------------+----------------------------
 *   CONFIG_ARCH_HIPRI_INTERRUPT |      NO      |             YES
 *   ----------------------------+--------------+--------------+-------------
 *   CONFIG_ARCH_INT_DISABLEALL  |     N/A      |     YES      |      NO
 *   ----------------------------+--------------+--------------+-------------
 *                               |              |              |    SVCall
 *                               |    SVCall    |    SVCall    |    HIGH
 *   Disable here and below --------> MAXNORMAL ---> HIGH --------> MAXNORMAL
 *                               |              |    MAXNORMAL |
 *   ----------------------------+--------------+--------------+-------------
 */

#if defined(CONFIG_ARCH_HIPRI_INTERRUPT) && defined(CONFIG_ARCH_INT_DISABLEALL)
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + 2*NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_HIGH_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#else
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       NVIC_SYSH_PRIORITY_MAX
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_MAXNORMAL_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32F7_CHIP_H */
