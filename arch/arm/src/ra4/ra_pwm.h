/****************************************************************************
 * arch/arm/src/ra4/ra_pwm.h
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

#ifndef __ARCH_ARM_SRC_RA4_RA_PWM_H
#define __ARCH_ARM_SRC_RA4_RA_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/ra_gpt.h"
#include "nuttx/timers/pwm.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* This structure represents the state of one PWM timer */

struct ra_pwm_lowerhalf_s
{
  const struct pwm_ops_s   * ops;        /* PWM operations */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra_pwm_initialize
 *
 * Description:
 *   Initialize the selected PWM port. And return a unique instance of struct
 *   struct ra_pwm_lowerhalf_s.  This function may be called to obtain
 *   multiple instances of the interface, each of which may be set up with a
 *   different frequency and address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple PWM interfaces)
 *   GPIO pin number for pin A
 *   GPIO pin number for pin B (CONFIG_PWM_NCHANNELS == 2)
 *
 * Returned Value:
 *   Valid PWM device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#if defined(CONFIG_PWM_NCHANNELS) && (CONFIG_PWM_NCHANNELS == 2)
struct ra_pwm_lowerhalf_s *ra_pwm_initialize(int      port,
                                                     int      pin_a,
                                                     int      pin_b,
                                                     uint32_t flags);
#else
struct ra_pwm_lowerhalf_s *ra_pwm_initialize(int      port,
                                                     int      pin,
                                                     uint32_t flags);
#endif

/****************************************************************************
 * Name: ra_pwmdev_uninitialize
 *
 * Description:
 *   De-initialize the selected pwm port, and power down the device.
 *
 * Input Parameter:
 *   Device structure as returned by the ra_pwmdev_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int ra_pwm_uninitialize(struct pwm_lowerhalf_s *dev);

/****************************************************************************
 * Name: ra_pwminitialize
 *
 * Description:
 *   Initialize one PWM channel for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   channel - A number identifying the PWM channel use.
 *
 * Returned Value:
 *   On success, a pointer to the RA PWM lower half driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *ra_pwminitialize(int channel);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA4_RA_PWM_H */
