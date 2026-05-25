/****************************************************************************
 * arch/arm/src/stm32f0/hardware/stm32_pinmap.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0_HARDWARE_STM32_PINMAP_H
#define __ARCH_ARM_SRC_STM32F0_HARDWARE_STM32_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32_STM32F03X)
#  include "hardware/stm32f03x_pinmap.h"
#elif defined(CONFIG_STM32_STM32F05X)
#  include "hardware/stm32f05x_pinmap.h"
#elif defined(CONFIG_STM32_STM32F07X)
#  include "hardware/stm32f07x_pinmap.h"
#elif defined(CONFIG_STM32_STM32F09X)
#  include "hardware/stm32f09x_pinmap.h"
#else
#  error "Unsupported STM32F0 pin map"
#endif

#endif /* __ARCH_ARM_SRC_STM32F0_HARDWARE_STM32_PINMAP_H */
