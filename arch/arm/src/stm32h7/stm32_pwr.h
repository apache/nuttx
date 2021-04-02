/****************************************************************************
 * arch/arm/src/stm32h7/stm32_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_PWR_H
#define __ARCH_ARM_SRC_STM32H7_STM32_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "hardware/stm32_pwr.h"

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
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void stm32_pwr_enablebkp(bool writable);

/****************************************************************************
 * Name: stm32_pwr_enablebreg
 *
 * Description:
 *   Enables the Backup regulator, the Backup regulator (used to maintain
 *   backup SRAM content in Standby and VBAT modes) is enabled. If BRE is
 *   reset, the backup regulator is switched off. The backup SRAM can still
 *   be used but its content will be lost in the Standby and VBAT modes. Once
 *   set, the application must wait that the Backup Regulator Ready flag
 *   (BRR) is set to indicate that the data written into the RAM will be
 *   maintained in the Standby and VBAT modes.
 *
 *   This function needs to be called after stm32_pwr_enablebkp(true) has
 *   been called.
 *
 * Input Parameters:
 *   region - state to set it to
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pwr_enablebreg(bool region);

/****************************************************************************
 * Name: stm32_pwr_configurewkup
 *
 * Description:
 *   Configures the external wakeup (WKUP) signals for wakeup from standby
 *   mode.
 *   Sets rising/falling edge sensitivity and pull state.
 *
 *
 * Input Parameters:
 *   pin    - WKUP pin number (0-5) to work on
 *   en     - Enables the specified WKUP pin if true
 *   rising - If true, wakeup is triggered on rising edge, otherwise,
 *            it is triggered on the falling edge.
 *   pull   - Specifies the WKUP pin pull resistor configuration
 *            (GPIO_FLOAT, GPIO_PULLUP, or GPIO_PULLDOWN)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pwr_configurewkup(uint32_t pin, bool en,
                             bool rising, uint32_t pull);

/****************************************************************************
 * Name: stm32_pwr_setvbatcharge
 *
 * Description:
 *   Configures the internal charge resistor to charge a battery attached
 *   to the VBAT pin.
 *
 *
 * Input Parameters:
 *   enable    - Enables the charge resistor if true, disables it if false
 *   resistor  - Sets charge resistor to 1.5 KOhm if true,
 *               sets it to 5 KOhm if false.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_pwr_setvbatcharge(bool enable, bool resistor);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_PWR_H */
