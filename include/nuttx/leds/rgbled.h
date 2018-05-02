/****************************************************************************
 * include/nuttx/rgbled.h
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __INCLUDE_NUTTX_LEDS_RGBLED_H
#define __INCLUDE_NUTTX_LEDS_RGBLED_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <fixedmath.h>

#include <nuttx/drivers/pwm.h>
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
