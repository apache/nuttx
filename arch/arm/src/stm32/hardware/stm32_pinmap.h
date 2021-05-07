/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_pinmap.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_PINMAP_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* STM32L EnergyLite Line ***************************************************/

#if defined(CONFIG_STM32_ENERGYLITE)

/* STM32L15xx family */

#  if defined(CONFIG_STM32_STM32L15XX)
#    include "hardware/stm32l15xxx_pinmap.h"
#  else
#    error "Unsupported EnergyLite chip"
#  endif

/* STM32 F1 Family **********************************************************/

#elif defined(CONFIG_STM32_STM32F10XX)

/* STM32F100 Value Line */

#  if defined(CONFIG_STM32_VALUELINE)
#    include "hardware/stm32f100_pinmap.h"

/* STM32 F102 USB Access Medium Density Family */
#  elif defined(CONFIG_ARCH_CHIP_STM32F102CB)
#    include "hardware/stm32f102_pinmap.h"

/* STM32 F103 Low / Medium Density Family */
#  elif defined(CONFIG_ARCH_CHIP_STM32F103C4) || \
        defined(CONFIG_ARCH_CHIP_STM32F103C8) || \
        defined(CONFIG_ARCH_CHIP_STM32F103CB)
#    include "hardware/stm32f103c_pinmap.h"

/* STM32 F103 High Density Family */

/* STM32F103RC, STM32F103RD, and STM32F103RE are all provided in 64 pin
 * packages and differ only in the available FLASH and SRAM.
 */

#  elif defined(CONFIG_ARCH_CHIP_STM32F103RB) || \
        defined(CONFIG_ARCH_CHIP_STM32F103RC) || \
        defined(CONFIG_ARCH_CHIP_STM32F103RD) || \
        defined(CONFIG_ARCH_CHIP_STM32F103RE) || \
        defined(CONFIG_ARCH_CHIP_STM32F103RG)
#    include "hardware/stm32f103r_pinmap.h"

/* STM32F103VC, STM32F103VD, and STM32F103VE are all provided in 100 pin
 * packages and differ only in the available FLASH and SRAM.
 */

#  elif defined(CONFIG_ARCH_CHIP_STM32F103VC) || defined(CONFIG_ARCH_CHIP_STM32F103VE)
#    include "hardware/stm32f103v_pinmap.h"

/* STM32F103ZC, STM32F103ZD, and STM32F103ZE are all provided in 144 pin
 * packages and differ only in the available FLASH and SRAM.
 */
#  elif defined(CONFIG_ARCH_CHIP_STM32F103ZE)
#    include "hardware/stm32f103z_pinmap.h"

/* STM32 F105/F107 Connectivity Line */

#  elif defined(CONFIG_ARCH_CHIP_STM32F105VB)
#    include "hardware/stm32f105v_pinmap.h"

#  elif defined(CONFIG_ARCH_CHIP_STM32F105RB)
#    include "hardware/stm32f105r_pinmap.h"

#  elif defined(CONFIG_ARCH_CHIP_STM32F107VC)
#    include "hardware/stm32f107v_pinmap.h"
#  else
#    error "Unsupported STM32F10XXX chip"
#  endif

/* STM32 F2 Family **********************************************************/

#elif defined(CONFIG_STM32_STM32F20XX)
#  include "hardware/stm32f20xxx_pinmap.h"

/* STM32 F3 Family **********************************************************/

#elif defined(CONFIG_STM32_STM32F30XX)
#  include "hardware/stm32f30xxx_pinmap.h"
#elif defined(CONFIG_STM32_STM32F33XX)
#  include "hardware/stm32f33xxx_pinmap.h"
#elif defined(CONFIG_STM32_STM32F37XX)
#  include "hardware/stm32f37xxx_pinmap.h"

/* STM32 F412 Family ********************************************************/

#elif defined(CONFIG_STM32_STM32F412)
#  include "hardware/stm32f412xx_pinmap.h"

/* STM32 F4 Family **********************************************************/

#elif defined(CONFIG_STM32_STM32F4XXX)
#  include "hardware/stm32f40xxx_pinmap.h"

/* STM32 G4 Family **********************************************************/

#elif defined(CONFIG_STM32_STM32G4XXX)
#  include "hardware/stm32g4xxxx_pinmap.h"

#else
#  error "No pinmap file for this STM32 chip"
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_PINMAP_H */
