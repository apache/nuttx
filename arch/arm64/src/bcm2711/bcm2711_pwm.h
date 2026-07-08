/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_pwm.h
 *
 * Contributed by: Matteo Golin <linguini@apache.org>
 *
 * SPDX-License-Identifer: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership. The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
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

#ifndef __ARCH_ARM64_SRC_BCM2711_BCM2711_PWM_H
#define __ARCH_ARM64_SRC_BCM2711_BCM2711_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/timers/pwm.h>

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
 * Name: bcm2711_pwminitialize
 *
 * Description:
 *   Initialize one PWM interface for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - Which PWM interface to initialize (0 or 1)
 *
 * Returned Value:
 *   On success, a pointer to the lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *bcm2711_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_BCM2711_BCM2711_PWM_H */
