/****************************************************************************
 * arch/xtensa/src/common/xtensa_fpucmp.c
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

#include "xtensa.h"

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
 *   Compare FPU areas from thread context.
 *
 * Input Parameters:
 *   saveregs1 - Pointer to the saved FPU registers.
 *   saveregs2 - Pointer to the saved FPU registers.
 *
 * Returned Value:
 *   True if FPU areas compare equal, False otherwise.
 *
 ****************************************************************************/

bool up_fpucmp(const void *saveregs1, const void *saveregs2)
{
  const uint32_t *regs1 = saveregs1;
  const uint32_t *regs2 = saveregs2;

  return memcmp(&regs1[COMMON_CTX_REGS], &regs2[COMMON_CTX_REGS],
                XTENSA_CP_SA_SIZE) == 0;
}
#endif /* CONFIG_ARCH_FPU */
