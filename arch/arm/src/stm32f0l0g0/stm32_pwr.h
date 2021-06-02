/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_STM32_PWR_H
#define __ARCH_ARM_SRC_STM32F0L0G0_STM32_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "hardware/stm32_pwr.h"

#ifdef CONFIG_STM32F0L0G0_PWR

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
 * Public Types
 ****************************************************************************/

/* Identify MCU-specific wakeup pin.  Different STM32 parts support differing
 * numbers of wakeup pins.
 */

enum stm32_pwr_wupin_e
{
  PWR_WUPIN_1 = 0,  /* Wake-up pin 1 (all parts) */
  PWR_WUPIN_2,      /* Wake-up pin 2 */
  PWR_WUPIN_3       /* Wake-up pin 3 */
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwr_initbkp
 *
 * Description:
 *   Insures the referenced count access to the backup domain (RTC registers,
 *   RTC backup data registers and backup SRAM is consistent with the HW
 *   state without relying on a variable.
 *
 *   NOTE: This function should only be called by SoC Start up code.
 *
 * Input Parameters:
 *   writable - set the initial state of the enable and the
 *              bkp_writable_counter
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pwr_initbkp(bool writable);

/****************************************************************************
 * Name: stm32_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data
 *   registers and backup SRAM).
 *
 *   NOTE:
 *   Reference counting is used in order to supported nested calls to this
 *   function.  As a consequence, every call to stm32_pwr_enablebkp(true)
 *   must be followed by a matching call to stm32_pwr_enablebkp(false).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pwr_enablebkp(bool writable);

/****************************************************************************
 * Name: stm32_pwr_enablewkup
 *
 * Description:
 *   Enables the WKUP pin.
 *
 * Input Parameters:
 *   wupin - Selects the WKUP pin to enable/disable
 *   wupon - state to set it to
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.  The only cause of failure is if the selected MCU does not
 *   support the requested wakeup pin.
 *
 ****************************************************************************/

int stm32_pwr_enablewkup(enum stm32_pwr_wupin_e wupin, bool wupon);

/****************************************************************************
 * Name: stm32_pwr_getsbf
 *
 * Description:
 *   Return the standby flag.
 *
 ****************************************************************************/

bool stm32_pwr_getsbf(void);

/****************************************************************************
 * Name: stm32_pwr_getwuf
 *
 * Description:
 *   Return the wakeup flag.
 *
 ****************************************************************************/

bool stm32_pwr_getwuf(void);

/****************************************************************************
 * Name: stm32_pwr_setvos
 *
 * Description:
 *   Set voltage scaling for EnergyLite devices.
 *
 * Input Parameters:
 *   vos - Properly aligned voltage scaling select bits for the PWR_CR
 *         register.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   At present, this function is called only from initialization logic.  If
 *   used for any other purpose that protection to assure that its operation
 *   is atomic will be required.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32F0L0G0_ENERGYLITE) || defined(CONFIG_STM32F0L0G0_STM32G0)
void stm32_pwr_setvos(uint16_t vos);
#endif /* CONFIG_STM32F0L0G0_ENERGYLITE || CONFIG_STM32F0L0G0_STM32G0 */

/****************************************************************************
 * Name: stm32_pwr_setpvd
 *
 * Description:
 *   Sets power voltage detector for EnergyLite devices.
 *
 * Input Parameters:
 *   pls - PVD level
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   At present, this function is called only from initialization logic.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32F0L0G0_ENERGYLITE)
void stm32_pwr_setpvd(uint16_t pls);

/****************************************************************************
 * Name: stm32_pwr_enablepvd
 *
 * Description:
 *   Enable the Programmable Voltage Detector
 *
 ****************************************************************************/

void stm32_pwr_enablepvd(void);

/****************************************************************************
 * Name: stm32_pwr_disablepvd
 *
 * Description:
 *   Disable the Programmable Voltage Detector
 *
 ****************************************************************************/

void stm32_pwr_disablepvd(void);

#endif /* CONFIG_STM32F0L0G0_ENERGYLITE */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32F0L0G0_PWR */
#endif /* __ARCH_ARM_SRC_STM32F0L0G0_STM32_PWR_H */
