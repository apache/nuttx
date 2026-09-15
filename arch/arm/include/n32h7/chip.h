/****************************************************************************
 * arch/arm/include/n32h7/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_N32H7_CHIP_H
#define __ARCH_ARM_INCLUDE_N32H7_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* N32H7x3xx  Differences between family members:
 *
 *   ----------- ---------------- ----- ----
 *                                       SPI
 *   PART        PACKAGE          GPIOs  I2S
 *   ----------- ---------------- ----- ----
 *   N32H762Ix    LQFP176          138   7/4
 *   ----------- ---------------- ----- ----
 *
 * Parts N32H7xxxI have 2048Kb of FLASH
 *
 * The correct FLASH size will be set CONFIG_N32H7_FLASH_CONFIG_x or
 * overridden with CONFIG_N32H7_FLASH_OVERRIDE_x
 */

#if defined (CONFIG_ARCH_CHIP_N32H762II)

/* Memory */
#    define N32H7_AXI_RAM_SIZE          (128*1024)  /* Placeholder, actual value to be determined */
#    define N32H7_AHB_RAM_SIZE          (352*1024)  /* Placeholder, actual value to be determined */
#  if defined(CONFIG_ARMV7M_HAVE_DTCM)
#      define N32H7_DTCM_SRAM_SIZE     (1024*1024)  /* Assume same as N32H7, adjust as needed */
#  else
#      define N32H7_DTCM_SRAM_SIZE      (0)         /* No DTCM SRAM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV7M_HAVE_ITCM)
#      define N32H7_ITCM_SRAM_SIZE      (0)         /* Assume same as N32H7, adjust as needed */
#  else
#      define N32H7_ITCM_SRAM_SIZE      (0)         /* No ITCM SRAM on TCM interface */
#  endif

/* Peripherals */
#  if defined(CONFIG_N32H7_IO_CONFIG_V)
#      define N32H7_NGPIO               (5)        /* Placeholder, actual value to be determined */
#  elif defined(CONFIG_N32H7_IO_CONFIG_Z)
#      define N32H7_NGPIO               (7)       /* Placeholder, actual value to be determined */
#  elif defined(CONFIG_N32H7_IO_CONFIG_I)
#      define N32H7_NGPIO               (9)        /* Placeholder, actual value to be determined */
#  elif defined(CONFIG_N32H7_IO_CONFIG_X)
#      define N32H7_NGPIO               (11)       /* Placeholder, actual value to be determined */
#  else
#      error CONFIG_N32H7_IO_CONFIG_x Not Set
#  endif

#  define N32H7_NDMA                    (3)         /* Placeholder, actual value to be determined */
#  define N32H7_NADC                    (3)         /* Placeholder, actual value to be determined */
#  define N32H7_NDAC                    (2)         /* Placeholder, actual value to be determined */
#  define N32H7_NCMP                    (4)         /* Placeholder, actual value to be determined */
#  define N32H7_NFMAC                   (1)         /* Placeholder, actual value to be determined */
#  define N32H7_NUSART                  (7)         /* Placeholder, actual value to be determined */
#  define N32H7_NSPI                    (7)         /* Placeholder, actual value to be determined */
#  define N32H7_NI2S                    (4)         /* Placeholder, actual value to be determined */
#  define N32H7_NUART                   (7)         /* Placeholder, actual value to be determined */
#  define N32H7_NI2C                    (10)        /* Placeholder, actual value to be determined */
#  define N32H7_NCAN                    (4)         /* Placeholder, actual value to be determined */
#  define N32H7_NSDMMC                  (2)         /* Placeholder, actual value to be determined */
#else
#  error N32 H7 chip not identified
#endif

/**
 * @brief Compute TCM configuration value from ITCM/DTCM/AXI sizes (KB).
 *        All sizes must be multiples of 128 and sum to 1024 KB.
 *        Result matches N32H7 TRM encoding 0x00..0x2C.
 */
#define N32H7_TCM_CFG(itcm_kb, dtcm_kb, axi_kb) \
    ( ( ( (axi_kb) / 128 ) * (19 - (axi_kb) / 128 ) / 2 ) + \
      ( (8 - (axi_kb) / 128) - (itcm_kb) / 128 ) )

/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_N32H7_CHIP_H */
