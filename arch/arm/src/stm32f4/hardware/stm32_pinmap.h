/****************************************************************************
 * arch/arm/src/stm32f4/hardware/stm32_pinmap.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F4_HARDWARE_STM32_PINMAP_H
#define __ARCH_ARM_SRC_STM32F4_HARDWARE_STM32_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32_STM32F412)
#  include "hardware/stm32f412xx_pinmap.h"
#elif defined(CONFIG_STM32_STM32F4XXX)
#  include "hardware/stm32f40xxx_pinmap.h"
#else
#  error "Unsupported STM32F4 pin map"
#endif

#endif /* __ARCH_ARM_SRC_STM32F4_HARDWARE_STM32_PINMAP_H */
