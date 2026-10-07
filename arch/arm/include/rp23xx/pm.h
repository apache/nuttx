/****************************************************************************
 * arch/arm/include/rp23xx/pm.h
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

#ifndef __ARCH_ARM_INCLUDE_RP23XX_PM_H
#define __ARCH_ARM_INCLUDE_RP23XX_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sys/boardctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* boardctl() command: suspend to RAM (POWMAN P1.0).  The argument is a
 * pointer to struct rp23xx_suspend_s.  Needs CONFIG_RP23XX_PM_SUSPEND and
 * CONFIG_BOARDCTL_IOCTL.  An armed RTC alarm also ends the suspend.
 */

#define BOARDIOC_RP23XX_SUSPEND  (BOARDIOC_USER + 0x0001)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rp23xx_suspend_s
{
  uint32_t wake_ms;      /* In: timed wake in milliseconds, 0 for none */
  uint32_t wake_source;  /* Out: POWMAN LAST_SWCORE_PWRUP after the wake */
};

#endif /* __ARCH_ARM_INCLUDE_RP23XX_PM_H */
