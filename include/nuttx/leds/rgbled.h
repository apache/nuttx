/****************************************************************************
 * include/nuttx/leds/rgbled.h
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

#ifndef __INCLUDE_NUTTX_LEDS_RGBLED_H
#define __INCLUDE_NUTTX_LEDS_RGBLED_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <fixedmath.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_RGBLED

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

/****************************************************************************
 * Name: rgbled_register
 *
 * Description:
 *   This function binds three instances of a "lower half" PWM driver with
 *   the "upper half" RGB LED device and registers that device so that can
 *   be used by application code.
 *
 *
 * Input Parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name all PWM drivers
 *     as "/dev/rgdbled0", "/dev/rgbled1", etc.  where the driver path
 *     differs only in the "minor" number at the end of the device name.
 *   ledr, ledg, and ledb - A pointer to an instance of lower half PWM
 *     drivers for the red, green, and blue LEDs, respectively.  These
 *     instances will be bound to the RGB LED driver and must persists as
 *     long as that driver persists.
 *   chanr, chang, chanb -Red/Green/Blue PWM channels (only if
 *     CONFIG_PWM_MULTICHAN is defined)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rgbled_register(FAR const char *path, FAR struct pwm_lowerhalf_s *ledr,
                                          FAR struct pwm_lowerhalf_s *ledg,
                                          FAR struct pwm_lowerhalf_s *ledb
#ifdef CONFIG_PWM_MULTICHAN
                                        , int chanr, int chang, int chanb
#endif
                                          );

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RGBLED */
#endif /* __INCLUDE_NUTTX_LEDS_RGBLED_H */
