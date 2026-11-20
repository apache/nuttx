/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_pwm.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_PWM_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>

#ifdef CONFIG_RZV2H_GPT_PWM

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_gpt_pwminitialize
 *
 * Description:
 *   Return the lower-half PWM driver instance for one GPT channel.
 *   Hardware initialization is deferred until the first open()
 *   of the associated /dev/pwmN device.
 *
 * Input Parameters:
 *   channel - GPT channel number (0-15)
 *
 * Returned Value:
 *   On success, a pointer to the lower-half PWM driver is returned.
 *   NULL if the channel is not enabled in Kconfig or is out of range.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *rzv2h_gpt_pwminitialize(uint8_t channel);

/****************************************************************************
 * Name: rzv2h_pwm_setup
 *
 * Description:
 *   Initialize PWM and register PWM devices for all enabled GPT channels.
 *   Each enabled channel is registered as /dev/pwmN.  Hardware
 *   initialization is deferred until the first open() via the
 *   upper-half PWM driver.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure:
 *   - -ENODEV if rzv2h_gpt_pwminitialize() returns NULL.
 *   - Negated errno from pwm_register() on failure.
 *
 ****************************************************************************/

int rzv2h_pwm_setup(void);

#endif /* CONFIG_RZV2H_GPT_PWM */
#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_PWM_H */
