/****************************************************************************
 * include/nuttx/audio/tone.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_TONE_H
#define __INCLUDE_NUTTX_AUDIO_TONE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <fixedmath.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_AUDIO_TONE

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
 * Name: tone_register
 *
 * Description:
 *   This function binds an instance of a "lower half" PWM driver with
 *   the "upper half" Audio Tone device and registers that device so that can
 *   be used by application code.
 *
 *
 * Input Parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name all PWM drivers
 *     as "/dev/tone0", "/dev/tone1", etc.  where the driver path
 *     differs only in the "minor" number at the end of the device name.
 *   channel - The the PWM peripheral supports multiple output channels, then
 *     this value must be provided to indicate the output channel that drives
 *     the tone.
 *   tone - A pointer to an instance of lower half PWM driver tone. This
 *     instance will be bound to the Audio Tone driver and must persists as
 *     long as that driver persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tone_register(FAR const char *path, FAR struct pwm_lowerhalf_s *tone,
#ifdef CONFIG_PWM_MULTICHAN
                  int channel,
#endif
                  FAR struct oneshot_lowerhalf_s *oneshot);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_TONE */
#endif /* __INCLUDE_NUTTX_AUDIO_TONE_H */
