/****************************************************************************
 * arch/arm/src/at32/at32_pwr.h
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

#ifndef __ARCH_ARM_SRC_AT32_AT32_PWR_H
#define __ARCH_ARM_SRC_AT32_AT32_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#include "chip.h"
#include "hardware/at32_pwr.h"

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
 * Different AT32 parts support differing numbers of wakeup pins.
 */

enum at32_pwr_wupin_e
{
  PWC_WUPIN_1 = 0,  /* Wake-up pin 1 (all parts) */
  PWC_WUPIN_2,      /* Wake-up pin 2 */
  PWC_WUPIN_3       /* Wake-up pin 3 */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: at32_pwr_initbkp
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

void at32_pwr_initbkp(bool writable);

/****************************************************************************
 * Name: at32_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain
 *  (RTC registers, RTC backup data registers and backup SRAM).
 *
 *   NOTE:
 *   Reference counting is used in order to supported nested calls to this
 *   function.  As a consequence, every call to at32_pwr_enablebkp(true)
 *   must be followed by a matching call to at32_pwr_enablebkp(false).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void at32_pwr_enablebkp(bool writable);

/****************************************************************************
 * Name: at32_pwr_enablewkup
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

int at32_pwr_enablewkup(enum at32_pwr_wupin_e wupin, bool wupon);

/****************************************************************************
 * Name: at32_pwr_getsbf
 *
 * Description:
 *   Return the standby flag.
 *
 ****************************************************************************/

bool at32_pwr_getsbf(void);

/****************************************************************************
 * Name: at32_pwr_getwuf
 *
 * Description:
 *   Return the wakeup flag.
 *
 ****************************************************************************/

bool at32_pwr_getwuf(void);

/****************************************************************************
 * Name: at32_pwr_setpvd
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

void at32_pwr_setpvd(uint16_t pls);

/****************************************************************************
 * Name: at32_pwr_enablepvd
 *
 * Description:
 *   Enable the Programmable Voltage Detector
 *
 ****************************************************************************/

void at32_pwr_enablepvd(void);

/****************************************************************************
 * Name: at32_pwr_disablepvd
 *
 * Description:
 *   Disable the Programmable Voltage Detector
 *
 ****************************************************************************/

void at32_pwr_disablepvd(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_AT32_AT32_PWR_H */
