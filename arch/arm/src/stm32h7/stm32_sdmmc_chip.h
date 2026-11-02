/****************************************************************************
 * arch/arm/src/stm32h7/stm32_sdmmc_chip.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_SDMMC_CHIP_H
#define __ARCH_ARM_SRC_STM32H7_STM32_SDMMC_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "chip.h"
#include "hardware/stm32_rcc.h"
#include "stm32_dtcm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory that the SDMMC IDMA can access.  Every SDMMC instance reaches the
 * AXI SRAM.  On the STM32H74x/H75x, SDMMC2 also reaches the D2 SRAM1/2/3
 * and the D3 SRAM4 (ST AN5200).  The Cortex-M4 uses a private alias for
 * SRAM123 that must not be given to the IDMA.
 */

#define STM32_SDMMC_IDMA_RAM_START  STM32_AXISRAM_BASE
#define STM32_SDMMC_IDMA_RAM_END    (STM32_AXISRAM_BASE + STM32_SRAM_SIZE)

#if defined(CONFIG_ARCH_CHIP_STM32H7_CORTEXM7) && \
    !defined(CONFIG_STM32_STM32H72XXX_OR_STM32H73XXX) && \
    (defined(CONFIG_STM32_STM32H7X0XX) || \
     defined(CONFIG_STM32_STM32H7X3XX) || \
     defined(CONFIG_STM32_STM32H7X5XX))
#  define STM32_SDMMC2_IDMA_SRAM123_START STM32_SRAM123_BASE
#  define STM32_SDMMC2_IDMA_SRAM123_END   (STM32_SRAM123_BASE + \
                                           STM32_SRAM123_SIZE)
#  define STM32_SDMMC2_IDMA_SRAM4_START   STM32_SRAM4_BASE
#  define STM32_SDMMC2_IDMA_SRAM4_END     (STM32_SRAM4_BASE + \
                                           STM32_SRAM4_SIZE)
#endif

/* The SDMMC kernel clock frequency.  board.h may provide STM32_SDMMC_CLK or
 * select the kernel clock with STM32_RCC_D1CCIPR_SDMMCSEL; pll1_q_ck is the
 * default after reset.
 */

#if !defined(STM32_SDMMC_CLK)
#  if !defined(STM32_RCC_D1CCIPR_SDMMCSEL)
#    define STM32_RCC_D1CCIPR_SDMMCSEL  RCC_D1CCIPR_SDMMC_PLL1
#  endif
#  if STM32_RCC_D1CCIPR_SDMMCSEL  == RCC_D1CCIPR_SDMMC_PLL1
#    define STM32_SDMMC_CLK  STM32_PLL1Q_FREQUENCY
#  else
#    define STM32_SDMMC_CLK  STM32_PLL2R_FREQUENCY
#  endif
#endif

#endif /* __ARCH_ARM_SRC_STM32H7_STM32_SDMMC_CHIP_H */
