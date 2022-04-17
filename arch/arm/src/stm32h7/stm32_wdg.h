/****************************************************************************
 * arch/arm/src/stm32h7/stm32_wdg.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_WDG_H
#define __ARCH_ARM_SRC_STM32H7_STM32_WDG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_wdg.h"

#ifdef CONFIG_WATCHDOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_iwdginitialize
 *
 * Description:
 *   Initialize the IWDG watchdog time.  The watchdog timer is initialized
 *   and registers as 'devpath'.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *   lsifreq - The calibrated LSI clock frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_IWDG
void stm32_iwdginitialize(const char *devpath, uint32_t lsifreq);
#endif

/****************************************************************************
 * Name: stm32_wwdginitialize
 *
 * Description:
 *   Initialize the WWDG watchdog time.  The watchdog timer is initialized
 *   and registers as 'devpath'.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_WWDG
void stm32_wwdginitialize(const char *devpath);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_WATCHDOG */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_WDG_H */
