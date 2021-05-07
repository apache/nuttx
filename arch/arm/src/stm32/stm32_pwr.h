/****************************************************************************
 * arch/arm/src/stm32/stm32_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_PWR_H
#define __ARCH_ARM_SRC_STM32_STM32_PWR_H

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
 * Public Types
 ****************************************************************************/

/* Identify MCU-specific wakeup pin.
 * Different STM32 parts support differing numbers of wakeup pins.
 */

enum stm32_pwr_wupin_e
{
  PWR_WUPIN_1 = 0,  /* Wake-up pin 1 (all parts) */
  PWR_WUPIN_2,      /* Wake-up pin 2 */
  PWR_WUPIN_3       /* Wake-up pin 3 */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwr_enablesdadc
 *
 * Description:
 *   Enables SDADC power
 *
 * Input Parameters:
 *   sdadc - SDADC number 1-3
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_STM32F37XX)
void stm32_pwr_enablesdadc(uint8_t sdadc);
#endif

/****************************************************************************
 * Name: stm32_pwr_initbkp
 *
 * Description:
 *   Insures the referenced count access to the backup domain
 *   (RTC registers, RTC backup data registers and backup SRAM is consistent
 *   with the HW state without relying on a variable.
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
 *   Enables access to the backup domain
 *  (RTC registers, RTC backup data registers and backup SRAM).
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
 * Name: stm32_pwr_enablebreg
 *
 * Description:
 *   Enables the Backup regulator, the Backup regulator (used to maintain
 *   backup SRAM content in Standby and VBAT modes) is enabled. If BRE is
 *   reset, the backup regulator is switched off. The backup SRAM can still
 *   be used but its content will be lost in the Standby and VBAT modes.
 *   Once set, the application must wait that the Backup Regulator Ready flag
 *   (BRR) is set to indicate that the data written into the RAM will be
 *   maintained in the Standby and VBAT modes.
 *
 * Input Parameters:
 *   region - state to set it to
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
void stm32_pwr_enablebreg(bool region);
#else
#  define stm32_pwr_enablebreg(region)
#endif

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
 *   At present, this function is called only from initialization logic.
 *   If used for any other purpose that protection to assure that its
 *   operation is atomic will be required.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_ENERGYLITE
void stm32_pwr_setvos(uint16_t vos);

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

#endif /* CONFIG_STM32_ENERGYLITE */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_PWR_H */
