/****************************************************************************
 * arch/arm/src/stm32/stm32_rtc.c
 *
 *   Copyright (C) 2011, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* This file is only a thin shell that includes the correct RTC
 * implementation for the selected STM32 family.  The correct file cannot be
 * selected by the make system because it needs the intelligence that only
 * exists in chip.h that can associate an STM32 part number with an STM32
 * family.
 */

/* The STM32 F1 has a simple battery-backed counter for its RTC and has a
 * separate block for the BKP registers.
 */

#if defined(CONFIG_STM32_STM32F10XX)
#  include "stm32_rtcounter.c"

/* The other families use a more traditional Realtime Clock/Calendar (RTCC)
 * with broken-out data/time in BCD format.  The backup registers are
 * integrated into the RTCC in these families.
 */

#elif defined(CONFIG_STM32_STM32F20XX) || \
      defined(CONFIG_STM32_STM32F30XX)
#  include "stm32_rtcc.c"
#elif defined(CONFIG_STM32_STM32L15XX)
#  include "stm32l15xxx_rtcc.c"
#elif defined(CONFIG_STM32_STM32F4XXX)
#  include "stm32f40xxx_rtcc.c"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
