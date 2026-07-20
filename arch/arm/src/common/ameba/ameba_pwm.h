/****************************************************************************
 * arch/arm/src/common/ameba/ameba_pwm.h
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_PWM_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Ameba PWM timer drives several compare channels off ONE shared time
 * base, so a single /dev/pwmN device exposes every wired channel: one
 * frequency applies to all, while each channel carries its own duty cycle
 * (this is the CONFIG_PWM_NCHANNELS multi-channel form of the NuttX PWM
 * upper half).  The board supplies, per hardware channel, the output pad it
 * is routed to (encoded with the AMEBA_PA()/AMEBA_PB() PinName codes used by
 * the GPIO driver); channels the board does not use carry AMEBA_PWM_PIN_NC.
 */

#define AMEBA_PWM0            0

/* Sentinel pad code for a hardware channel the board leaves unrouted. */

#define AMEBA_PWM_PIN_NC      0xff

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
 * Name: ameba_pwm_register
 *
 * Description:
 *   Configure the Ameba PWM timer and register it with the NuttX PWM
 *   character driver at the given path (typically "/dev/pwm0").  All wired
 *   channels share the timer's single time base (one frequency) and each
 *   carries its own duty cycle.
 *
 * Input Parameters:
 *   path  - The device path to register, e.g. "/dev/pwm0".
 *   pins  - Array giving, for each hardware channel, the output pad encoded
 *           with AMEBA_PA()/AMEBA_PB(), or AMEBA_PWM_PIN_NC for a channel
 *           the board does not use.
 *   npins - Number of entries in pins (clamped to the timer's channel
 *           count).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ameba_pwm_register(const char *path, const uint8_t *pins,
                       unsigned int npins);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_PWM_H */
