/****************************************************************************
 * arch/arm/src/stm32f3/hardware/stm32_pinmap.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F3_HARDWARE_STM32_PINMAP_H
#define __ARCH_ARM_SRC_STM32F3_HARDWARE_STM32_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32_STM32F30XX)
#  include "hardware/stm32f30xxx_pinmap.h"
#elif defined(CONFIG_STM32_STM32F33XX)
#  include "hardware/stm32f33xxx_pinmap.h"
#elif defined(CONFIG_STM32_STM32F37XX)
#  include "hardware/stm32f37xxx_pinmap.h"
#else
#  error "Unsupported STM32F3 pin map"
#endif

#endif /* __ARCH_ARM_SRC_STM32F3_HARDWARE_STM32_PINMAP_H */
