/****************************************************************************
 * arch/arm/src/imxrt/imxrt_lpsrtc.h
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

/****************************************************************************
 * Name: imxrt_lpsrtc_initialize
 *
 * Description:
 *   Initialize the LPSRTC per the selected configuration.
 *   This function is called via up_rtc_initialize (see imxrt_hprtc.c).
 *
 *   NOTE that the LPSRTC is always configured synchronized with the HPRTC.
 *   This means that the time is set via the LPSRTC but read via the HPRTC.
 *   Also, only the alarms from the HPRTC are used.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

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
