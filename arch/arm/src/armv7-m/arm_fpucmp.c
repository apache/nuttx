/****************************************************************************
 * arch/arm/src/armv7-m/arm_fpucmp.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <nuttx/irq.h>

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fpucmp
 *
 * Description:
 *   compare FPU areas from thread context
 *
 ****************************************************************************/

bool up_fpucmp(const void *saveregs1, const void *saveregs2)
{
  const uint32_t *regs1 = saveregs1;
  const uint32_t *regs2 = saveregs2;

  /* compare of hardware fp registers should skip REG_FP_RESERVED */

  return memcmp(&regs1[REG_S0], &regs2[REG_S0],
                4 * (HW_FPU_REGS - 1)) == 0 &&
         memcmp(&regs1[REG_S16], &regs2[REG_S16], 4 * SW_FPU_REGS) == 0;
}
#endif /* CONFIG_ARCH_FPU */
