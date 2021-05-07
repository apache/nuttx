/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pm.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_PM_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_PM_H

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
 * Name: stm32l4_pmstop
 *
 * Description:
 *   Enter STOP mode.
 *
 * Input Parameters:
 *   lpds - true: To further reduce power consumption in Stop mode, put the
 *          internal voltage regulator in low-power mode using the LPDS bit
 *          of the Power control register (PWR_CR).
 *
 * Returned Value:
 *   Zero means that the STOP was successfully entered and the system has
 *   been re-awakened.  The internal voltage regulator is back to its
 *   original state.  Otherwise, STOP mode did not occur and a negated
 *   errno value is returned to indicate the cause of the failure.
 *
 ****************************************************************************/

int stm32l4_pmstop(bool lpds);

/****************************************************************************
 * Name: stm32l4_pmstop2
 *
 * Description:
 *   Enter STOP2 mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero means that the STOP2 was successfully entered and the system has
 *   been re-awakened.  Otherwise, STOP2 mode did not occur and a negated
 *   errno value is returned to indicate the cause of the failure.
 *
 ****************************************************************************/

int stm32l4_pmstop2(void);

/****************************************************************************
 * Name: stm32l4_pmstandby
 *
 * Description:
 *   Enter STANDBY mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, this function will not return (STANDBY mode can only be
 *   terminated with a reset event).  Otherwise, STANDBY mode did not occur
 *   and a negated errno value is returned to indicate the cause of the
 *   failure.
 *
 ****************************************************************************/

int stm32l4_pmstandby(void);

/****************************************************************************
 * Name: stm32l4_pmsleep
 *
 * Description:
 *   Enter SLEEP mode.
 *
 * Input Parameters:
 *   sleeponexit - true:  SLEEPONEXIT bit is set when the WFI instruction is
 *                        executed, the MCU enters Sleep mode as soon as it
 *                        exits the lowest priority ISR.
 *               - false: SLEEPONEXIT bit is cleared, the MCU enters Sleep
 *                        mode as soon as WFI or WFE instruction is executed.
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32l4_pmsleep(bool sleeponexit);

/****************************************************************************
 * Name: stm32l4_pmlpr
 *
 * Description:
 *   Enter Low-Power Run (LPR) mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero means that LPR was successfully entered. Otherwise, LPR mode was
 *   not entered and a negated errno value is returned to indicate the cause
 *   of the failure.
 *
 ****************************************************************************/

int stm32l4_pmlpr(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_PM_H */
