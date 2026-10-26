/****************************************************************************
 * arch/arm/src/common/stm32/stm32_lptim_clk.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_LPTIM_CLK_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_LPTIM_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RCC clock-enable and reset aliases for the common LPTIM driver.
 *
 * The bus of each LPTIM instance differs between families and is detected
 * from the RCC bit definitions.  APB3 is checked first, the STM32U5 RCC
 * header also defines the STM32L4 APB1 LPTIM bits.  The LPTIM kernel clock
 * is provided by each board as STM32_LPTIMn_CLKIN.
 */

#if defined(RCC_APB3ENR_LPTIM1EN)
#  define STM32_RCC_LPTIM1_EN_REG   STM32_RCC_APB3ENR
#  define STM32_RCC_LPTIM1_EN       RCC_APB3ENR_LPTIM1EN
#  define STM32_RCC_LPTIM1_RST_REG  STM32_RCC_APB3RSTR
#  define STM32_RCC_LPTIM1_RST      RCC_APB3RSTR_LPTIM1RST
#elif defined(RCC_APB1ENR1_LPTIM1EN)
#  define STM32_RCC_LPTIM1_EN_REG   STM32_RCC_APB1ENR1
#  define STM32_RCC_LPTIM1_EN       RCC_APB1ENR1_LPTIM1EN
#  define STM32_RCC_LPTIM1_RST_REG  STM32_RCC_APB1RSTR1
#  define STM32_RCC_LPTIM1_RST      RCC_APB1RSTR1_LPTIM1RST
#elif defined(RCC_APB1LENR_LPTIM1EN)
#  define STM32_RCC_LPTIM1_EN_REG   STM32_RCC_APB1LENR
#  define STM32_RCC_LPTIM1_EN       RCC_APB1LENR_LPTIM1EN
#  define STM32_RCC_LPTIM1_RST_REG  STM32_RCC_APB1LRSTR
#  define STM32_RCC_LPTIM1_RST      RCC_APB1LRSTR_LPTIM1RST
#elif defined(RCC_APB1ENR_LPTIM1EN)
#  define STM32_RCC_LPTIM1_EN_REG   STM32_RCC_APB1ENR
#  define STM32_RCC_LPTIM1_EN       RCC_APB1ENR_LPTIM1EN
#  define STM32_RCC_LPTIM1_RST_REG  STM32_RCC_APB1RSTR
#  define STM32_RCC_LPTIM1_RST      RCC_APB1RSTR_LPTIM1RST
#endif

#if defined(RCC_APB1HENR_LPTIM2EN)
#  define STM32_RCC_LPTIM2_EN_REG   STM32_RCC_APB1HENR
#  define STM32_RCC_LPTIM2_EN       RCC_APB1HENR_LPTIM2EN
#  define STM32_RCC_LPTIM2_RST_REG  STM32_RCC_APB1HRSTR
#  define STM32_RCC_LPTIM2_RST      RCC_APB1HRSTR_LPTIM2RST
#elif defined(RCC_APB4ENR_LPTIM2EN)
#  define STM32_RCC_LPTIM2_EN_REG   STM32_RCC_APB4ENR
#  define STM32_RCC_LPTIM2_EN       RCC_APB4ENR_LPTIM2EN
#  define STM32_RCC_LPTIM2_RST_REG  STM32_RCC_APB4RSTR
#  define STM32_RCC_LPTIM2_RST      RCC_APB4RSTR_LPTIM2RST
#elif defined(RCC_APB4LENR_LPTIM2EN)
#  define STM32_RCC_LPTIM2_EN_REG   STM32_RCC_APB4LENR
#  define STM32_RCC_LPTIM2_EN       RCC_APB4LENR_LPTIM2EN
#  define STM32_RCC_LPTIM2_RST_REG  STM32_RCC_APB4LRSTR
#  define STM32_RCC_LPTIM2_RST      RCC_APB4LRSTR_LPTIM2RST
#elif defined(RCC_APB1ENR2_LPTIM2EN)
#  define STM32_RCC_LPTIM2_EN_REG   STM32_RCC_APB1ENR2
#  define STM32_RCC_LPTIM2_EN       RCC_APB1ENR2_LPTIM2EN
#  define STM32_RCC_LPTIM2_RST_REG  STM32_RCC_APB1RSTR2
#  define STM32_RCC_LPTIM2_RST      RCC_APB1RSTR2_LPTIM2RST
#elif defined(RCC_APB1ENR_LPTIM2EN)
#  define STM32_RCC_LPTIM2_EN_REG   STM32_RCC_APB1ENR
#  define STM32_RCC_LPTIM2_EN       RCC_APB1ENR_LPTIM2EN
#  define STM32_RCC_LPTIM2_RST_REG  STM32_RCC_APB1RSTR
#  define STM32_RCC_LPTIM2_RST      RCC_APB1RSTR_LPTIM2RST
#endif

#if defined(RCC_APB3ENR_LPTIM3EN)
#  define STM32_RCC_LPTIM3_EN_REG   STM32_RCC_APB3ENR
#  define STM32_RCC_LPTIM3_EN       RCC_APB3ENR_LPTIM3EN
#  define STM32_RCC_LPTIM3_RST_REG  STM32_RCC_APB3RSTR
#  define STM32_RCC_LPTIM3_RST      RCC_APB3RSTR_LPTIM3RST
#elif defined(RCC_APB4ENR_LPTIM3EN)
#  define STM32_RCC_LPTIM3_EN_REG   STM32_RCC_APB4ENR
#  define STM32_RCC_LPTIM3_EN       RCC_APB4ENR_LPTIM3EN
#  define STM32_RCC_LPTIM3_RST_REG  STM32_RCC_APB4RSTR
#  define STM32_RCC_LPTIM3_RST      RCC_APB4RSTR_LPTIM3RST
#elif defined(RCC_APB4LENR_LPTIM3EN)
#  define STM32_RCC_LPTIM3_EN_REG   STM32_RCC_APB4LENR
#  define STM32_RCC_LPTIM3_EN       RCC_APB4LENR_LPTIM3EN
#  define STM32_RCC_LPTIM3_RST_REG  STM32_RCC_APB4LRSTR
#  define STM32_RCC_LPTIM3_RST      RCC_APB4LRSTR_LPTIM3RST
#elif defined(RCC_APB1ENR2_LPTIM3EN)
#  define STM32_RCC_LPTIM3_EN_REG   STM32_RCC_APB1ENR2
#  define STM32_RCC_LPTIM3_EN       RCC_APB1ENR2_LPTIM3EN
#  define STM32_RCC_LPTIM3_RST_REG  STM32_RCC_APB1RSTR2
#  define STM32_RCC_LPTIM3_RST      RCC_APB1RSTR2_LPTIM3RST
#elif defined(RCC_APB1ENR_LPTIM3EN)
#  define STM32_RCC_LPTIM3_EN_REG   STM32_RCC_APB1ENR
#  define STM32_RCC_LPTIM3_EN       RCC_APB1ENR_LPTIM3EN
#  define STM32_RCC_LPTIM3_RST_REG  STM32_RCC_APB1RSTR
#  define STM32_RCC_LPTIM3_RST      RCC_APB1RSTR_LPTIM3RST
#endif

#if defined(RCC_APB3ENR_LPTIM4EN)
#  define STM32_RCC_LPTIM4_EN_REG   STM32_RCC_APB3ENR
#  define STM32_RCC_LPTIM4_EN       RCC_APB3ENR_LPTIM4EN
#  define STM32_RCC_LPTIM4_RST_REG  STM32_RCC_APB3RSTR
#  define STM32_RCC_LPTIM4_RST      RCC_APB3RSTR_LPTIM4RST
#elif defined(RCC_APB4ENR_LPTIM4EN)
#  define STM32_RCC_LPTIM4_EN_REG   STM32_RCC_APB4ENR
#  define STM32_RCC_LPTIM4_EN       RCC_APB4ENR_LPTIM4EN
#  define STM32_RCC_LPTIM4_RST_REG  STM32_RCC_APB4RSTR
#  define STM32_RCC_LPTIM4_RST      RCC_APB4RSTR_LPTIM4RST
#elif defined(RCC_APB4LENR_LPTIM4EN)
#  define STM32_RCC_LPTIM4_EN_REG   STM32_RCC_APB4LENR
#  define STM32_RCC_LPTIM4_EN       RCC_APB4LENR_LPTIM4EN
#  define STM32_RCC_LPTIM4_RST_REG  STM32_RCC_APB4LRSTR
#  define STM32_RCC_LPTIM4_RST      RCC_APB4LRSTR_LPTIM4RST
#endif

#if defined(RCC_APB3ENR_LPTIM5EN)
#  define STM32_RCC_LPTIM5_EN_REG   STM32_RCC_APB3ENR
#  define STM32_RCC_LPTIM5_EN       RCC_APB3ENR_LPTIM5EN
#  define STM32_RCC_LPTIM5_RST_REG  STM32_RCC_APB3RSTR
#  define STM32_RCC_LPTIM5_RST      RCC_APB3RSTR_LPTIM5RST
#elif defined(RCC_APB4ENR_LPTIM5EN)
#  define STM32_RCC_LPTIM5_EN_REG   STM32_RCC_APB4ENR
#  define STM32_RCC_LPTIM5_EN       RCC_APB4ENR_LPTIM5EN
#  define STM32_RCC_LPTIM5_RST_REG  STM32_RCC_APB4RSTR
#  define STM32_RCC_LPTIM5_RST      RCC_APB4RSTR_LPTIM5RST
#elif defined(RCC_APB4LENR_LPTIM5EN)
#  define STM32_RCC_LPTIM5_EN_REG   STM32_RCC_APB4LENR
#  define STM32_RCC_LPTIM5_EN       RCC_APB4LENR_LPTIM5EN
#  define STM32_RCC_LPTIM5_RST_REG  STM32_RCC_APB4LRSTR
#  define STM32_RCC_LPTIM5_RST      RCC_APB4LRSTR_LPTIM5RST
#endif

#if defined(RCC_APB3ENR_LPTIM6EN)
#  define STM32_RCC_LPTIM6_EN_REG   STM32_RCC_APB3ENR
#  define STM32_RCC_LPTIM6_EN       RCC_APB3ENR_LPTIM6EN
#  define STM32_RCC_LPTIM6_RST_REG  STM32_RCC_APB3RSTR
#  define STM32_RCC_LPTIM6_RST      RCC_APB3RSTR_LPTIM6RST
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_LPTIM_CLK_H */
