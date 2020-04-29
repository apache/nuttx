/****************************************************************************
 * arch/z16/src/common/z16_registerdump.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "z16_internal.h"

#ifdef CONFIG_ARCH_STACKDUMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

static chipreg_t s_last_regs[XCPTCONTEXT_REGS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z16_registerdump
 ****************************************************************************/

static void z16_registerdump(void)
{
#ifdef CONFIG_DEBUG_INFO
  FAR uint32_t *regs32 = (FAR uint32_t *)g_current_regs;

  if (regs32 == NULL)
    {
      z16_saveusercontext(s_last_regs);
      regs32 = (FAR uint32_t *)s_last_regs;
    }

  _alert("R0 :%08x R1 :%08x R2 :%08x R3 :%08x "
        "R4 :%08x R5 :%08x R6 :%08x R7 :%08x\n"
        regs32[REG_R0 / 2],  regs32[REG_R1 / 2],  regs32[REG_R2 / 2],
        regs32[REG_R3 / 2],  regs32[REG_R4 / 2],  regs32[REG_R5 / 2],
        regs32[REG_R6 / 2],  regs32[REG_R7 / 2]);
  _alert("R8 :%08x R9 :%08x R10:%08x R11:%08x R12:%08x R13:%08x\n"
        regs32[REG_R8 / 2],  regs32[REG_R9 / 2],  regs32[REG_R10 / 2],
        regs3[REG_R11 / 2],  regs32[REG_R12 / 2], regs32[REG_R13 / 2]);
  _alert("FP :%08x SP :%08x FLG:%04x\n"
        regs32[REG_R14 / 2], regs32[REG_R15 / 2], regs32[REG_FLAGS]);
#endif
}

#endif /* CONFIG_ARCH_STACKDUMP */
