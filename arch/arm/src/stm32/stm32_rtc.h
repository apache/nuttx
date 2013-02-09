/************************************************************************************
 * arch/arm/src/stm32/stm32_rtc.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu> (Original for the F1)
 *           Gregory Nutt <gnutt@nuttx.org> (On-going support and development)
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_RTC_H
#define __ARCH_ARM_SRC_STM32_STM32_RTC_H

#include <nuttx/config.h>

#include "chip.h"

/* The STM32 F1 has a simple battery-backed counter for its RTC and has a separate
 * block for the BKP registers.
 */

#if defined(CONFIG_STM32_STM32F10XX)
#  include "chip/stm32_rtc.h"
#  include "chip/stm32_bkp.h"

/* The other families use a more traditional Realtime Clock/Calendar (RTCC) with
 * broken-out data/time in BCD format.  The backup registers are integrated into
 * the RTCC in these families.
 */

#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F30XX) ||\
      defined(CONFIG_STM32_STM32F40XX)
#  include "chip/stm32_rtcc.h"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define STM32_RTC_PRESCALER_SECOND      32767   /* Default prescaler to get a second base */
#define STM32_RTC_PRESCALER_MIN         1       /* Maximum speed of 16384 Hz */

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Set alarm output pin */

EXTERN void stm32_rtc_settalarmpin(bool activate);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_RTC_H */
