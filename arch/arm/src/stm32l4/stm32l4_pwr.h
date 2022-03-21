/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_PWR_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "hardware/stm32l4_pwr.h"

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
 * Name: enableclk
 *
 * Description:
 *   Enable/disable the clock to the power control peripheral.  Enabling
 *   must be done after the APB1 clock is validly configured, and prior to
 *   using any functionality controlled by the PWR block (i.e. much of
 *   anything else provided by this module).
 *
 * Input Parameters:
 *   enable - True: enable the clock to the Power control (PWR) block.
 *
 * Returned Value:
 *   True:  the PWR block was previously enabled.
 *
 ****************************************************************************/

bool stm32l4_pwr_enableclk(bool enable);

/****************************************************************************
 * Name: stm32l4_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data
 *   registers and backup SRAM).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   True: The backup domain was previously writable.
 *
 ****************************************************************************/

bool stm32l4_pwr_enablebkp(bool writable);

/****************************************************************************
 * Name: stm32l4_pwr_enableusv
 *
 * Description:
 *   Enables or disables the USB Supply Valid monitoring.  Setting this bit
 *   is mandatory to use the USB OTG FS peripheral.
 *
 * Input Parameters:
 *   set - True: Vddusb is valid; False: Vddusb is not present. Logical and
 *         electrical isolation is applied to ignore this supply.
 *
 * Returned Value:
 *   True: The bit was previously set.
 *
 ****************************************************************************/

bool stm32l4_pwr_enableusv(bool set);

/****************************************************************************
 * Name: stm32l4_pwr_enable_pvme2
 *
 * Description:
 *   Enables or disables the peripheral voltage monitoring for Vddio2.
 *
 * Input Parameters:
 *   set - True: Vddio2 monitoring enable; False: Vddio2 monitoring disable.
 *
 * Returned Value:
 *   True: The bit was previously set.
 *
 ****************************************************************************/

#if !defined(CONFIG_STM32L4_STM32L4X3)
bool stm32l4_pwr_enable_pvme2(bool set);
#endif

/****************************************************************************
 * Name: stm32l4_pwr_get_pvmo2
 *
 * Description:
 *   Get value of peripheral voltage monitor output 2 (Vddio2).
 *
 * Returned Value:
 *   True: Vddio2 voltage is below PVM2 threshold.
 *   False: Vddio2 voltage is above PVM2 threshold.
 *
 ****************************************************************************/

#if !defined(CONFIG_STM32L4_STM32L4X3)
bool stm32l4_pwr_get_pvmo2(void);
#endif

/****************************************************************************
 * Name: stm32l4_pwr_vddio2_valid
 *
 * Description:
 *   Report that the Vddio2 independent I/Os supply voltage is valid or not.
 *   Setting this bit is mandatory to use the PG2 - PG15 I/Os.
 *
 * Input Parameters:
 *   set - True: Vddio2 is valid; False: Vddio2 is not present.  Logical and
 *         electrical isolation is applied to ignore this supply.
 *
 * Returned Value:
 *   True: The bit was previously set.
 ****************************************************************************/

#if !defined(CONFIG_STM32L4_STM32L4X3)
bool stm32l4_pwr_vddio2_valid(bool set);
#endif

/****************************************************************************
 * Name: stm32_pwr_setvos
 *
 * Description:
 *   Set voltage scaling for Vcore
 *
 * Input Parameters:
 *   vos - Either 1 or 2, to set to Range 1 or 2, respectively
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

void stm32_pwr_setvos(int vos);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_PWR_H */
