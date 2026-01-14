/***************************************************************************
 * arch/arm/src/common/arm_poweroff.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <debug.h>

#include "arch/syscall.h"

#ifdef CONFIG_ARM_SEMIHOSTING_POWEROFF

/***************************************************************************
 * Preprocessor Definitions
 ***************************************************************************/

/* per https://github.com/ARM-software/abi-aa semihosting.pdf 2024Q3 */

#define ARMSMH_OPNUM_SYSEXIT    0x18
#define ARMSMH_PARAM_APPEXIT    0x20026

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: up_systempoweroff
 *
 * Description:
 *   Semihosting EXIT based poweroff logic.
 *
 ***************************************************************************/

void up_systempoweroff(void)
{
  int ret;

  /* Set up for the system poweroff */

  ret = smh_call(ARMSMH_OPNUM_SYSEXIT, ARMSMH_PARAM_APPEXIT);
  if (ret)
    {
      sinfo("Failed to power off CPU, error code: %d\n", ret);
    }

  /* Wait for power off */

  for (; ; );
}
#endif /* CONFIG_ARM_SEMIHOSTING_POWEROFF */