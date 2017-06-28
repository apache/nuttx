/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pwr.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: dev@ziggurat29.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_PWR_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_PWR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "chip/stm32l4_pwr.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: enableclk
 *
 * Description:
 *   Enable/disable the clock to the power control peripheral.  Enabling must be done
 *   after the APB1 clock is validly configured, and prior to using any functionality
 *   controlled by the PWR block (i.e. much of anything else provided by this module).
 *
 * Input Parameters:
 *   enable - True: enable the clock to the Power control (PWR) block.
 *
 * Returned Value:
 *   True:  the PWR block was previously enabled.
 *
 ************************************************************************************/

bool stm32l4_pwr_enableclk(bool enable);

/************************************************************************************
 * Name: stm32l4_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data registers
 *   and backup SRAM).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   True: The backup domain was previously writable.
 *
 ************************************************************************************/

bool stm32l4_pwr_enablebkp(bool writable);

/************************************************************************************
 * Name: stm32l4_pwr_enableusv
 *
 * Description:
 *   Enables or disables the USB Supply Valid monitoring.  Setting this bit is
 *   mandatory to use the USB OTG FS peripheral.
 *
 * Input Parameters:
 *   set - True: Vddusb is valid; False: Vddusb is not present. Logical and electrical
 *         isolation is applied to ignore this supply.
 *
 * Returned Value:
 *   True: The bit was previously set.
 *
 ************************************************************************************/

bool stm32l4_pwr_enableusv(bool set);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_PWR_H */
