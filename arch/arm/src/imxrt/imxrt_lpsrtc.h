/****************************************************************************
 * arch/arm/src/imxrt/imxrt_lpsrtc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_LPSRTC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_LPSRTC_H

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_IMXRT_SNVS_LPSRTC

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#  ifdef CONFIG_RTC_DATETIME
#    error CONFIG_RTC_DATETIME should not be selected with this driver
#  endif

#  ifdef CONFIG_RTC_PERIODIC
#    error CONFIG_RTC_PERIODIC should not be selected with this driver
#  endif

/* REVISIT: This is probably supportable.  The 47 bit timer does have
 * accuracy greater than 1 second.
 */

#  ifdef CONFIG_RTC_HIRES
#    error CONFIG_RTC_PERIODIC should not be selected with this driver
#  endif

/****************************************************************************
 * Public Function Prototypes
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

/************************************************************************************
 * Name: imxrt_lpsrtc_initialize
 *
 * Description:
 *   Initialize the LPSRTC per the selected configuration.  This function is called
 *   via up_rtc_initialize (see imxrt_hprtc.c).
 *
 *   NOTE that the LPSRTC is always configured synchronized with the HPRTC.  This
 *   means that the time is set via the LPSRTC but read via the HPRTC.  Also, only
 *   the alarms from the HPRTC are used.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int imxrt_lpsrtc_initialize(void);

/****************************************************************************
 * Name: imxrt_lpsrtc_havesettime
 *
 * Description:
 *   Check if the LPSRTC time has been set
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

bool imxrt_lpsrtc_havesettime(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_IMXRT_SNVS_LPSRTC */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_LPSRTC_H */
