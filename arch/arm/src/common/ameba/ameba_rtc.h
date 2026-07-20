/****************************************************************************
 * arch/arm/src/common/ameba/ameba_rtc.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_RTC_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct rtc_lowerhalf_s;

/****************************************************************************
 * Name: ameba_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the Ameba RTC lower-half driver.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "ameba_rtc.h"
 *
 *     struct rtc_lowerhalf_s *lower = ameba_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 *   The first call also gates the RTC clock and initialises the fwlib RTC
 *   (24-hour format).  The returned interface is a singleton; the board
 *   normally hands it straight to rtc_initialize() to create /dev/rtc0.
 *
 * Returned Value:
 *   On success a non-NULL RTC lower-half interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *ameba_rtc_lowerhalf(void);

/****************************************************************************
 * Name: ameba_rtc_initialize
 *
 * Description:
 *   Convenience wrapper that instantiates the Ameba RTC lower half and binds
 *   it to the NuttX RTC character driver at "/dev/rtc0" (minor 0).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ameba_rtc_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_RTC_H */
