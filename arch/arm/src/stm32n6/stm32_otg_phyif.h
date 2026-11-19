/****************************************************************************
 * arch/arm/src/stm32n6/stm32_otg_phyif.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_OTG_PHYIF_H
#define __ARCH_ARM_SRC_STM32N6_STM32_OTG_PHYIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "hardware/stm32n6xxx_memorymap.h"
#include "hardware/stm32n6xxx_rcc.h"
#include "hardware/stm32n6xxx_usbphyc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The common OTG driver drives USB1 or USB2 OTG HS as its OTG HS instance.
 * Each core is wired to its own embedded high speed PHY on the UTMI+
 * interface and has no alternate function pins.
 */

#ifdef CONFIG_STM32N6_OTGHS_USB2
#  define STM32_OTGHS_BASE         STM32_USB2_OTG_HS_BASE
#  define STM32_IRQ_OTGHS          STM32_IRQ_USB2_OTG_HS
#  define STM32_OTGHS_PHYC_CR      STM32_USB2_HS_PHYC_CR
#  define STM32_OTGHS_RST          RCC_AHB5RSTR_OTG2RST
#  define STM32_OTGHS_PHYRST       RCC_AHB5RSTR_OTGPHY2RST
#  define STM32_OTGHS_PHYCTLRST    RCC_AHB5RSTR_OTG2PHYCTLRST
#  define STM32_OTGHS_PHY_FREQUENCY STM32_OTGPHY2_FREQUENCY
#else
#  define STM32_OTGHS_BASE         STM32_USB1_OTG_HS_BASE
#  define STM32_IRQ_OTGHS          STM32_IRQ_USB1_OTG_HS
#  define STM32_OTGHS_PHYC_CR      STM32_USB1_HS_PHYC_CR
#  define STM32_OTGHS_RST          RCC_AHB5RSTR_OTG1RST
#  define STM32_OTGHS_PHYRST       RCC_AHB5RSTR_OTGPHY1RST
#  define STM32_OTGHS_PHYCTLRST    RCC_AHB5RSTR_OTG1PHYCTLRST
#  define STM32_OTGHS_PHY_FREQUENCY STM32_OTGPHY1_FREQUENCY
#endif

#define STM32_OTG_EMBEDDED_HSPHY   1

/* The PHY reference clock comes from board.h (RCC CCIPR6 OTGPHYxSEL) */

#ifndef STM32_OTGHS_PHY_FREQUENCY
#  error "board.h must provide STM32_OTGPHY1/2_FREQUENCY"
#elif STM32_OTGHS_PHY_FREQUENCY == 19200000
#  define STM32_OTGHS_PHY_FSEL     USBPHYC_CR_FSEL_19_2MHZ
#elif STM32_OTGHS_PHY_FREQUENCY == 20000000
#  define STM32_OTGHS_PHY_FSEL     USBPHYC_CR_FSEL_20MHZ
#elif STM32_OTGHS_PHY_FREQUENCY == 24000000
#  define STM32_OTGHS_PHY_FSEL     USBPHYC_CR_FSEL_24MHZ
#else
#  error "Unsupported OTG PHY reference clock (19.2, 20 or 24 MHz)"
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_otg_phyinit
 *
 * Description:
 *   Power up and configure the embedded high speed PHY of the selected
 *   USB OTG HS instance.
 *
 * Returned Value:
 *   OK on success, -ETIMEDOUT if VDD33USB does not become ready.
 *
 ****************************************************************************/

int stm32_otg_phyinit(void);

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_OTG_PHYIF_H */
