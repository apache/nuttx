/****************************************************************************
 * libs/libc/machine/tricore/arch_setjmp.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>

#include <arch/setjmp.h>
#include <nuttx/arch.h>

#include <IfxCpu_reg.h>
#include <IfxCpu_Intrinsics.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int setjmp(jmp_buf env)
{
  uintptr_t *regs;
  uintptr_t pcxi;

  env->regs[JB_UA11] = (uintptr_t)__getA11();

  pcxi = __mfcr(CPU_PCXI);
  regs = tricore_csa2addr(pcxi);
  memcpy(env, regs, TC_CONTEXT_SIZE);
  return 0;
}

void longjmp(jmp_buf env, int val)
{
  void *func = (void *)env->regs[JB_UA11];
  uintptr_t *regs;
  uintptr_t pcxi;

  pcxi = __mfcr(CPU_PCXI);
  regs = tricore_csa2addr(pcxi);
  memcpy(regs, env, TC_CONTEXT_SIZE);

  if (val == 0)
    {
      val = 1;
    }

  __moveToDataParamRet(val);
  __jumpToFunctionWithLink(func);
}
