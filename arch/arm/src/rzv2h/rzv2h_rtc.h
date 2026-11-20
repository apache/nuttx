/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_rtc.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_RTC_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: rzv2h_rtc_initialize
 *
 * Description:
 *   Initialize the RTC lower-half driver.  Must be called from
 *   rzv2h_bringup() before rtc_initialize().
 *
 ****************************************************************************/

int rzv2h_rtc_initialize(void);

/****************************************************************************
 * Name: rzv2h_rtc_lowerhalf
 *
 * Description:
 *   Return a pointer to the RTC lower-half driver instance.  The instance
 *   is a file-scope static; this function is the only way to obtain it.
 *
 *   Called from rzv2h_bringup() to pass to rtc_initialize() and
 *   up_rtc_set_lowerhalf().
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *rzv2h_rtc_lowerhalf(void);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_RTC_H */
