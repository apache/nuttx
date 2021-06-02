/****************************************************************************
 * arch/arm/include/stm32h7/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32H7_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32H7_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* STM32H7x3xx  Differences between family members:
 *
 *   ----------- ---------------- ----- ----
 *                                       SPI
 *   PART        PACKAGE          GPIOs  I2S
 *   ----------- ---------------- ----- ----
 *   STM32H7x3Ax UFBGA169          132   6/3
 *   STM32H7x3Bx LQFP208           168   6/3
 *   STM32H7x3Ix LQFP176/UFBGA176  140   6/3
 *   STM32H7x3Vx LQFP100/TFBGA100   82   5/3
 *   STM32H7x3Xx TFBGA240          168   6/3
 *   STM32H7x3Zx LQFP144           114   6/3
 *   ----------- ---------------- ----- ----
 *
 * Parts STM32H7xxxG have 1024Kb of FLASH
 *
 * Parts STM32H7xxxI have 2048Kb of FLASH
 *
 * The correct FLASH size will be set CONFIG_STM32H7_FLASH_CONFIG_x or
 * overridden with CONFIG_STM32H7_FLASH_OVERRIDE_x
 */

#if defined (CONFIG_ARCH_CHIP_STM32H743AG) || \
    defined (CONFIG_ARCH_CHIP_STM32H743AI) || \
    defined (CONFIG_ARCH_CHIP_STM32H743BG) || \
    defined (CONFIG_ARCH_CHIP_STM32H743BI) || \
    defined (CONFIG_ARCH_CHIP_STM32H743IG) || \
    defined (CONFIG_ARCH_CHIP_STM32H743II) || \
    defined (CONFIG_ARCH_CHIP_STM32H743VG) || \
    defined (CONFIG_ARCH_CHIP_STM32H743VI) || \
    defined (CONFIG_ARCH_CHIP_STM32H743XG) || \
    defined (CONFIG_ARCH_CHIP_STM32H743XI) || \
    defined (CONFIG_ARCH_CHIP_STM32H743ZG) || \
    defined (CONFIG_ARCH_CHIP_STM32H743ZI) || \
    defined (CONFIG_ARCH_CHIP_STM32H753AI) || \
    defined (CONFIG_ARCH_CHIP_STM32H753BI) || \
    defined (CONFIG_ARCH_CHIP_STM32H753II) || \
    defined (CONFIG_ARCH_CHIP_STM32H753VI) || \
    defined (CONFIG_ARCH_CHIP_STM32H753XI) || \
    defined (CONFIG_ARCH_CHIP_STM32H753ZI)
#elif defined(CONFIG_ARCH_CHIP_STM32H747XI)
#else
#  error STM32 H7 chip not identified
#endif

/* Size SRAM */

#if defined(CONFIG_STM32H7_STM32H7X3XX)
/* Memory */

#    define STM32H7_SRAM_SIZE             (512*1024)  /* 512Kb SRAM on AXI bus Matrix (D1) */
#    define STM32H7_SRAM1_SIZE            (128*1024)  /* 128Kb SRAM1 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM2_SIZE            (128*1024)  /* 128Kb SRAM2 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM3_SIZE            (32*1024)   /*  32Kb SRAM3 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM123_SIZE          (288*1024)  /* 128Kb SRAM123 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM4_SIZE            (64*1024)   /*  64Kb SRAM2 on AHB bus Matrix (D3) */
#  if defined(CONFIG_ARMV7M_HAVE_DTCM)
#      define STM32H7_DTCM_SRAM_SIZE      (128*1024)  /* 128Kb DTCM SRAM on TCM interface */
#  else
#      define STM32H7_DTCM_SRAM_SIZE      (0)         /* No DTCM SRAM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV7M_HAVE_ITCM)
#      define STM32H7_ITCM_SRAM_SIZE      (64*1024)   /*  64b ITCM SRAM on TCM interface */
#  else
#      define STM32H7_ITCM_SRAM_SIZE      (0)         /* No ITCM SRAM on TCM interface */
#  endif

/* Peripherals */

#  if defined(CONFIG_STM32H7_IO_CONFIG_A)
#      define STM32H7_NGPIO               (10)        /* GPIOA-GPIOJ */
#  elif defined(CONFIG_STM32H7_IO_CONFIG_B)
#      define STM32H7_NGPIO               (11)        /* GPIOA-GPIOK */
#  elif defined(CONFIG_STM32H7_IO_CONFIG_I)
#      define STM32H7_NGPIO               (9)         /* GPIOA-GPIOI */
#  elif defined(CONFIG_STM32H7_IO_CONFIG_V)
#      define STM32H7_NGPIO               (8)         /* GPIOA-GPIOH, missing GPIOF-GPIOG */
#  elif defined(CONFIG_STM32H7_IO_CONFIG_X)
#      define STM32H7_NGPIO               (11)        /* GPIOA-GPIOK */
#  elif defined(CONFIG_STM32H7_IO_CONFIG_Z)
#      define STM32H7_NGPIO               (8)         /* GPIOA-GPIOH */
#  else
#      error CONFIG_STM32H7_IO_CONFIG_x Not Set
#  endif

#  define STM32H7_NDMA                    (4)         /* (4) DMA1, DMA2, BDMA and MDMA */
#  define STM32H7_NADC                    (3)         /* (3) ADC1-3*/
#  define STM32H7_NDAC                    (2)         /* (2) DAC1-2*/
#  define STM32H7_NCMP                    (2)         /* (2) ultra-low power comparators */
#  define STM32H7_NPGA                    (2)         /* (2) Operational amplifiers: OPAMP */
#  define STM32H7_NDFSDM                  (1)         /* (1) digital filters for sigma delta modulator */
#  define STM32H7_NUSART                  (4)         /* (4) USART1-3, 6 */
#  define STM32H7_NSPI                    (6)         /* (6) SPI1-6 */
#  define STM32H7_NI2S                    (3)         /* (3) I2S1-3 */
#  define STM32H7_NUART                   (4)         /* (4) UART4-5, 7-8 */
#  define STM32H7_NI2C                    (4)         /* (4) I2C1-4 */
#  define STM32H7_NSAI                    (4)         /* (4) SAI1-4*/
#  define STM32H7_NCAN                    (2)         /* (2) CAN1-2 */
#  define STM32H7_NSDIO                   (2)         /* (2) SDIO */
#elif defined(CONFIG_STM32H7_STM32H7X7XX)
/* Memory */

#    define STM32H7_SRAM_SIZE             (512*1024)  /* 512Kb SRAM on AXI bus Matrix (D1) */
#    define STM32H7_SRAM1_SIZE            (128*1024)  /* 128Kb SRAM1 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM2_SIZE            (128*1024)  /* 128Kb SRAM2 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM3_SIZE            (32*1024)   /*  32Kb SRAM3 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM123_SIZE          (288*1024)  /* 128Kb SRAM123 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM4_SIZE            (64*1024)   /*  64Kb SRAM2 on AHB bus Matrix (D3) */
#  if defined(CONFIG_ARMV7M_HAVE_DTCM)
#      define STM32H7_DTCM_SRAM_SIZE      (128*1024)  /* 128Kb DTCM SRAM on TCM interface */
#  else
#      define STM32H7_DTCM_SRAM_SIZE      (0)         /* No DTCM SRAM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV7M_HAVE_ITCM)
#      define STM32H7_ITCM_SRAM_SIZE      (64*1024)   /*  64b ITCM SRAM on TCM interface */
#  else
#      define STM32H7_ITCM_SRAM_SIZE      (0)         /* No ITCM SRAM on TCM interface */
#  endif

/* Peripherals */

#  define STM32H7_NGPIO                   (11)        /* GPIOA-GPIOK */
#  define STM32H7_NDMA                    (4)         /* (4) DMA1, DMA2, BDMA and MDMA */
#  define STM32H7_NADC                    (3)         /* (3) ADC1-3*/
#  define STM32H7_NDAC                    (2)         /* (2) DAC1-2*/
#  define STM32H7_NCMP                    (2)         /* (2) ultra-low power comparators */
#  define STM32H7_NPGA                    (2)         /* (2) Operational amplifiers: OPAMP */
#  define STM32H7_NDFSDM                  (1)         /* (1) digital filters for sigma delta modulator */
#  define STM32H7_NUSART                  (4)         /* (4) USART1-3, 6 */
#  define STM32H7_NSPI                    (6)         /* (6) SPI1-6 */
#  define STM32H7_NI2S                    (3)         /* (3) I2S1-3 */
#  define STM32H7_NUART                   (4)         /* (4) UART4-5, 7-8 */
#  define STM32H7_NI2C                    (4)         /* (4) I2C1-4 */
#  define STM32H7_NSAI                    (4)         /* (4) SAI1-4*/
#  define STM32H7_NCAN                    (2)         /* (2) CAN1-2 */
#  define STM32H7_NSDIO                   (2)         /* (2) SDIO */
#else
#  error STM32 H7 chip Family not identified
#endif

/* TBD FPU Configuration */

#if defined(CONFIG_ARCH_HAVE_FPU)
#else
#endif

#if defined(CONFIG_ARCH_HAVE_DPFPU)
#else
#endif

/* Diversification based on Family and package */

#if defined(CONFIG_STM32H7_HAVE_ETHERNET)
#  define STM32H7_NETHERNET                1   /* 100/100 Ethernet MAC */
#else
#  define STM32H7_NETHERNET                0   /* No 100/100 Ethernet MAC */
#endif

#if defined(CONFIG_STM32H7_HAVE_FMC)
#  define STM32H7_NFMC                     1   /* Have FMC memory controller */
#else
#  define STM32H7_NFMC                     0   /* No FMC memory controller */
#endif

/* NVIC priority levels *****************************************************/

/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32H7_CHIP_H */
