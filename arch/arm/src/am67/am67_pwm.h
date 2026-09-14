/****************************************************************************
 * arch/arm/src/am67/am67_pwm.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_PWM_H
#define __ARCH_ARM_SRC_AM67_AM67_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>

#if defined(CONFIG_AM67_EPWM0) || defined(CONFIG_AM67_EPWM1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am67_epwm_init
 *
 * Description:
 *   Boot-time EPWM preparation: unlock CTRL_MMR partition 1 (kick lock)
 *   so that the clock-gate and pad writes issued later by the PWM
 *   lower-half setup() can land.  Everything else (clock enable, PID
 *   check, pinmux, waveform) is done by the lower-half ops, called by
 *   the upper half on open/ioctl.  Must run before pwm_register().
 *
 * Assumptions:
 *   The EPWM0 power domain must be on (it is managed by the DMSC/TISCI
 *   firmware, not by this driver).  On the current board it has been
 *   observed to stay on without any intervention (mechanism unconfirmed;
 *   possibly shared with the EPWM2 cooling fan or an unreaped boot
 *   default) - do NOT rely on this.  The only explicit guarantee today
 *   is forcing the device active from Linux before the R5F starts:
 *
 *     echo on > /sys/devices/platform/bus@f0000/23000000.pwm/power/control
 *
 *   ("on" disables Linux runtime power management for that device, which
 *   keeps the domain powered until reboot.)  If the domain is off, EPWM
 *   registers read as zeros - no bus fault - so a PID mismatch of
 *   0x00000000 means "not powered", not "wrong address".  Sub-word
 *   (8-bit) accesses also read as zeros; use 16/32-bit accesses only.
 *
 * WARNING: This power domain is currently held up by an unidentified
 *   party on the Linux side.  A Linux reboot or deliberate PM action
 *   (pwmchip unexport, driver unbind, 'echo auto') WILL release it, and
 *   if the holder is the EPWM2 fan's thermal policy, it could drop
 *   spontaneously (e.g. fan off when cool) - unverified but not
 *   excluded.  Either way the R5F gets no notification; outputs die
 *   silently.  Do not drive safety-critical loads (motors, ESCs) until
 *   a NuttX-side TISCI client owns this domain.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value from the first failing
 *   step otherwise.
 *
 ****************************************************************************/

int am67_epwm_init(void);

/****************************************************************************
 * Name: am67_epwminitialize
 *
 * Description:
 *   Return the EPWM lower-half instance for the given PWM number so the
 *   board bringup can bind it to the upper half with pwm_register().
 *   No hardware is touched here; the upper half drives the hardware
 *   through the ops (setup on first open, start/stop via ioctl).
 *
 * Input Parameters:
 *   pwm - PWM instance number: 0 (EPWM0) or 1 (EPWM1).  EPWM2 is the
 *   board cooling fan and is deliberately not supported.
 *
 * Returned Value:
 *   Pointer to the lower-half driver on success; NULL on an unsupported
 *   or unconfigured instance number.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *am67_epwminitialize(int pwm);

#endif /* CONFIG_AM67_EPWM0 || CONFIG_AM67_EPWM1 */
#endif /* __ARCH_ARM_SRC_AM67_AM67_PWM_H */
