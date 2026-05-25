/****************************************************************************
 * arch/arm/src/stm32f1/hardware/stm32_pinmap.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F1_HARDWARE_STM32_PINMAP_H
#define __ARCH_ARM_SRC_STM32F1_HARDWARE_STM32_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32_VALUELINE)
#  include "hardware/stm32f100_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_STM32F102CB)
#  include "hardware/stm32f102_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_STM32F103C4) || \
      defined(CONFIG_ARCH_CHIP_STM32F103C8) || \
      defined(CONFIG_ARCH_CHIP_STM32F103CB)
#  include "hardware/stm32f103c_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_STM32F103RB) || \
      defined(CONFIG_ARCH_CHIP_STM32F103RC) || \
      defined(CONFIG_ARCH_CHIP_STM32F103RD) || \
      defined(CONFIG_ARCH_CHIP_STM32F103RE) || \
      defined(CONFIG_ARCH_CHIP_STM32F103RG)
#  include "hardware/stm32f103r_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_STM32F103VC) || \
      defined(CONFIG_ARCH_CHIP_STM32F103VE)
#  include "hardware/stm32f103v_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_STM32F103ZE)
#  include "hardware/stm32f103z_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_STM32F105VB)
#  include "hardware/stm32f105v_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_STM32F105RB)
#  include "hardware/stm32f105r_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_STM32F107VC)
#  include "hardware/stm32f107v_pinmap.h"
#else
#  error "Unsupported STM32F1 pin map"
#endif

#endif /* __ARCH_ARM_SRC_STM32F1_HARDWARE_STM32_PINMAP_H */
