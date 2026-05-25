/****************************************************************************
 * arch/arm/src/stm32f0/hardware/stm32_memorymap.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0_HARDWARE_STM32_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32F0_HARDWARE_STM32_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32_STM32F03X)
#  include "hardware/stm32f03x_memorymap.h"
#elif defined(CONFIG_STM32_STM32F05X) || defined(CONFIG_STM32_STM32F07X) || \
      defined(CONFIG_STM32_STM32F09X)
#  include "hardware/stm32f05xf07xf09x_memorymap.h"
#else
#  error "Unsupported STM32F0 memory map"
#endif

#endif /* __ARCH_ARM_SRC_STM32F0_HARDWARE_STM32_MEMORYMAP_H */
