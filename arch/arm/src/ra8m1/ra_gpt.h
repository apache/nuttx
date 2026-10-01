/****************************************************************************
 * arch/arm/src/ra8m1/ra_gpt.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_RA_GPT_H
#define __ARCH_ARM_SRC_RA8M1_RA_GPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPT channels: GPT32 = channels 0-7 (32-bit), GPT16 = channels 8-13
 * (16-bit).
 */

#define RA_GPT32_FIRST   0
#define RA_GPT32_LAST    7
#define RA_GPT16_FIRST   8
#define RA_GPT16_LAST    13

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_RA_GPT_TIMER

/****************************************************************************
 * Name: ra_gpt_timer_initialize
 *
 * Description:
 *   Bind a GPT channel to the upper-half timer driver and register it as
 *   a character device (e.g. "/dev/timer0").
 *
 * Input Parameters:
 *   devpath - The device path to register.
 *   channel - GPT channel number, 0-7 (GPT32) or 8-13 (GPT16).  The
 *             channel must be enabled with CONFIG_RA_GPTn_GPT.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_gpt_timer_initialize(const char *devpath, int channel);

#endif /* CONFIG_RA_GPT_TIMER */

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_RA8M1_RA_GPT_H */
