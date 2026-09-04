/****************************************************************************
 * include/nuttx/timers/sifive_pwm.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_SIFIVE_PWM_H
#define __INCLUDE_NUTTX_TIMERS_SIFIVE_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/timers/pwm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Compare channels implemented by the IP.  PWMCMP0..PWMCMP3 each drive one
 * output, so the lower-half accepts channel numbers 1..4 (the PWM upper half
 * numbers channels from one).
 */

#define SIFIVE_PWM_NCHANNELS  4

/* Reference clock used when the device tree carries no "clock-frequency".
 * Matches the default of QEMU's sifive-pwm model (hw/timer/sifive_pwm.c).
 */

#define SIFIVE_PWM_DEFAULT_FREQ  500000000

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: sifive_pwm_initialize
 *
 * Description:
 *   Initialise a SiFive PWM lower-half bound to a memory-mapped register
 *   window (as modelled by QEMU's "sifive,pwm0" device on the virt
 *   platform).  The returned device is passed to pwm_register().
 *
 *   The counter runs in its natural 16-bit wrap mode, so the period is
 *   fixed at 65536 scaled ticks and all four compare registers are
 *   available as duty outputs.  A requested frequency is approximated by
 *   the PWMCFG.SCALE prescaler, which quantises it to powers of two - the
 *   same behaviour as the mainline Linux pwm-sifive driver.
 *
 * Input Parameters:
 *   Any number of controllers may be initialised: each call returns its own
 *   state block, so callers must not assume a single instance.
 *
 * Input Parameters:
 *   base    - Physical (kernel-visible) base address of the PWM register
 *             window, taken from the device tree "reg" property.
 *   freq_hz - Reference clock feeding the counter, in Hz.  Pass
 *             SIFIVE_PWM_DEFAULT_FREQ when the device tree does not say.
 *
 * Returned Value:
 *   Valid PWM lower-half structure reference on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *sifive_pwm_initialize(uintptr_t base,
                                                  uint32_t freq_hz);

/****************************************************************************
 * Name: sifive_pwm_uninitialize
 *
 * Description:
 *   Release a lower-half returned by sifive_pwm_initialize().  Only needed
 *   on an error path before the device reaches pwm_register(): once
 *   registered, the lower-half must persist for the lifetime of the driver.
 *
 * Input Parameters:
 *   dev - Lower-half returned by sifive_pwm_initialize(), or NULL.
 *
 ****************************************************************************/

void sifive_pwm_uninitialize(FAR struct pwm_lowerhalf_s *dev);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIMERS_SIFIVE_PWM_H */
