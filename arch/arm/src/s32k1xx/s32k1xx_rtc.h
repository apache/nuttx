/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_rtc.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32JK1XX_RTC_H
#define __ARCH_ARM_SRC_S32K1XX_S32JK1XX_RTC_H

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_S32K1XX_RTC

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#  ifdef CONFIG_RTC_DATETIME
#    error CONFIG_RTC_DATETIME should not be selected with this driver
#  endif

#  ifdef CONFIG_RTC_PERIODIC
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
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the rtc per the selected configuration.
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void);

/****************************************************************************
 * Name: s32k1xx_rtc_havesettime
 *
 * Description:
 *   Check if the rtc time has been set
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

bool s32k1xx_rtc_havesettime(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_S32K1XX_RTC */
#endif /* __ARCH_ARM_SRC_S32K1XX_S32JK1XX_RTC_H */
