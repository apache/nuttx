/****************************************************************************
 * arch/arm/src/stm32f7/stm32_pm.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_PM_H
#define __ARCH_ARM_SRC_STM32F7_STM32_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_pmstop
 *
 * Description:
 *   Enter STOP mode.
 *
 * Input Parameters:
 *   lpds - true: To further reduce power consumption in Stop mode, put the
 *          internal voltage regulator in low-power under-drive mode using
 *          the LPDS and LPUDS bits of the Power control register (PWR_CR1).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pmstop(bool lpds);

/****************************************************************************
 * Name: stm32_pmstandby
 *
 * Description:
 *   Enter STANDBY mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pmstandby(void);

/****************************************************************************
 * Name: stm32_pmsleep
 *
 * Description:
 *   Enter SLEEP mode.
 *
 * Input Parameters:
 *   sleeponexit - true:  SLEEPONEXIT bit is set when the WFI instruction is
 *                        executed, the MCU enters Sleep mode as soon as it
 *                        exits the lowest priority ISR.
 *               - false: SLEEPONEXIT bit is cleared, the MCU enters Sleep
 *                        mode as soon as WFI or WFE instruction is
 *                        executed.
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pmsleep(bool sleeponexit);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32F7_STM32_PM_H */
