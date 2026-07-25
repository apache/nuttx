/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_timer.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_TIMER_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
 * Name: rp23xx_timer_initialize
 *
 * Description:
 *   Bind one of the two RP2350 system timer blocks (TIMER0 or TIMER1) to a
 *   /dev/timerN device.  Each block is a free-running 64-bit microsecond
 *   counter; this driver uses its ALARM0 to implement the NuttX timer
 *   lower-half (single-shot and periodic timeouts, 1 us resolution, up to
 *   2^32 - 1 us ~= 71.5 minutes per interval).
 *
 * Input Parameters:
 *   devpath  - The full path to the timer device, e.g. "/dev/timer0".
 *   instance - The timer block to use: 0 for TIMER0, 1 for TIMER1.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rp23xx_timer_initialize(FAR const char *devpath, int instance);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_TIMER_H */
