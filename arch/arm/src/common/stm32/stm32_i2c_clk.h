/****************************************************************************
 * arch/arm/src/common/stm32/stm32_i2c_clk.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_I2C_CLK_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_I2C_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map each I2C instance to its RCC enable and reset register and bit.  The
 * bus differs between families: I2C1/2 sit on APB1 everywhere, I2C3 on
 * APB1 (H7, N6) or APB3 (H5), I2C4 on APB4 (H7, N6) or APB3 (H5).
 */

#if defined(RCC_APB1LENR_I2C1EN)
#  define STM32_RCC_I2C1_EN_REG    STM32_RCC_APB1LENR
#  define STM32_RCC_I2C1_EN        RCC_APB1LENR_I2C1EN
#  define STM32_RCC_I2C1_RST_REG   STM32_RCC_APB1LRSTR
#  define STM32_RCC_I2C1_RST       RCC_APB1LRSTR_I2C1RST
#elif defined(RCC_APB1ENR1_I2C1EN)
#  define STM32_RCC_I2C1_EN_REG    STM32_RCC_APB1ENR1
#  define STM32_RCC_I2C1_EN        RCC_APB1ENR1_I2C1EN
#  define STM32_RCC_I2C1_RST_REG   STM32_RCC_APB1RSTR1
#  define STM32_RCC_I2C1_RST       RCC_APB1RSTR1_I2C1RST
#elif defined(RCC_APB1ENR_I2C1EN)
#  define STM32_RCC_I2C1_EN_REG    STM32_RCC_APB1ENR
#  define STM32_RCC_I2C1_EN        RCC_APB1ENR_I2C1EN
#  define STM32_RCC_I2C1_RST_REG   STM32_RCC_APB1RSTR
#  define STM32_RCC_I2C1_RST       RCC_APB1RSTR_I2C1RST
#endif

#if defined(RCC_APB1LENR_I2C2EN)
#  define STM32_RCC_I2C2_EN_REG    STM32_RCC_APB1LENR
#  define STM32_RCC_I2C2_EN        RCC_APB1LENR_I2C2EN
#  define STM32_RCC_I2C2_RST_REG   STM32_RCC_APB1LRSTR
#  define STM32_RCC_I2C2_RST       RCC_APB1LRSTR_I2C2RST
#elif defined(RCC_APB1ENR1_I2C2EN)
#  define STM32_RCC_I2C2_EN_REG    STM32_RCC_APB1ENR1
#  define STM32_RCC_I2C2_EN        RCC_APB1ENR1_I2C2EN
#  define STM32_RCC_I2C2_RST_REG   STM32_RCC_APB1RSTR1
#  define STM32_RCC_I2C2_RST       RCC_APB1RSTR1_I2C2RST
#elif defined(RCC_APB1ENR_I2C2EN)
#  define STM32_RCC_I2C2_EN_REG    STM32_RCC_APB1ENR
#  define STM32_RCC_I2C2_EN        RCC_APB1ENR_I2C2EN
#  define STM32_RCC_I2C2_RST_REG   STM32_RCC_APB1RSTR
#  define STM32_RCC_I2C2_RST       RCC_APB1RSTR_I2C2RST
#endif

#if defined(RCC_APB1LENR_I2C3EN)
#  define STM32_RCC_I2C3_EN_REG    STM32_RCC_APB1LENR
#  define STM32_RCC_I2C3_EN        RCC_APB1LENR_I2C3EN
#  define STM32_RCC_I2C3_RST_REG   STM32_RCC_APB1LRSTR
#  define STM32_RCC_I2C3_RST       RCC_APB1LRSTR_I2C3RST
#elif defined(RCC_APB3ENR_I2C3EN)
#  define STM32_RCC_I2C3_EN_REG    STM32_RCC_APB3ENR
#  define STM32_RCC_I2C3_EN        RCC_APB3ENR_I2C3EN
#  define STM32_RCC_I2C3_RST_REG   STM32_RCC_APB3RSTR
#  define STM32_RCC_I2C3_RST       RCC_APB3RSTR_I2C3RST
#elif defined(RCC_APB1ENR1_I2C3EN)
#  define STM32_RCC_I2C3_EN_REG    STM32_RCC_APB1ENR1
#  define STM32_RCC_I2C3_EN        RCC_APB1ENR1_I2C3EN
#  define STM32_RCC_I2C3_RST_REG   STM32_RCC_APB1RSTR1
#  define STM32_RCC_I2C3_RST       RCC_APB1RSTR1_I2C3RST
#elif defined(RCC_APB1ENR_I2C3EN)
#  define STM32_RCC_I2C3_EN_REG    STM32_RCC_APB1ENR
#  define STM32_RCC_I2C3_EN        RCC_APB1ENR_I2C3EN
#  define STM32_RCC_I2C3_RST_REG   STM32_RCC_APB1RSTR
#  define STM32_RCC_I2C3_RST       RCC_APB1RSTR_I2C3RST
#endif

#if defined(RCC_APB4LENR_I2C4EN)
#  define STM32_RCC_I2C4_EN_REG    STM32_RCC_APB4LENR
#  define STM32_RCC_I2C4_EN        RCC_APB4LENR_I2C4EN
#  define STM32_RCC_I2C4_RST_REG   STM32_RCC_APB4LRSTR
#  define STM32_RCC_I2C4_RST       RCC_APB4LRSTR_I2C4RST
#elif defined(RCC_APB3ENR_I2C4EN)
#  define STM32_RCC_I2C4_EN_REG    STM32_RCC_APB3ENR
#  define STM32_RCC_I2C4_EN        RCC_APB3ENR_I2C4EN
#  define STM32_RCC_I2C4_RST_REG   STM32_RCC_APB3RSTR
#  define STM32_RCC_I2C4_RST       RCC_APB3RSTR_I2C4RST
#elif defined(RCC_APB4ENR_I2C4EN)
#  define STM32_RCC_I2C4_EN_REG    STM32_RCC_APB4ENR
#  define STM32_RCC_I2C4_EN        RCC_APB4ENR_I2C4EN
#  define STM32_RCC_I2C4_RST_REG   STM32_RCC_APB4RSTR
#  define STM32_RCC_I2C4_RST       RCC_APB4RSTR_I2C4RST
#elif defined(RCC_APB1ENR1_I2C4EN)
#  define STM32_RCC_I2C4_EN_REG    STM32_RCC_APB1ENR1
#  define STM32_RCC_I2C4_EN        RCC_APB1ENR1_I2C4EN
#  define STM32_RCC_I2C4_RST_REG   STM32_RCC_APB1RSTR1
#  define STM32_RCC_I2C4_RST       RCC_APB1RSTR1_I2C4RST
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_I2C_CLK_H */
