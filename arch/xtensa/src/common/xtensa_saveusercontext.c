/****************************************************************************
 * arch/xtensa/src/common/xtensa_saveusercontext.c
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

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/macro.h>

#include <string.h>

#include <arch/irq.h>
#include <arch/syscall.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_saveusercontext
 *
 * Description:
 *   Save the current thread context.  Full prototype is:
 *
 *   int  up_saveusercontext(void *saveregs);
 *
 * Returned Value:
 *   0: Normal return
 *   1: Context switch return
 *
 ****************************************************************************/

int up_saveusercontext(void *saveregs)
{
  if (up_interrupt_context())
    {
      __asm__ __volatile__
        (
          "   s32i a0, %0, (4 * " STRINGIFY(REG_A0) ")\n"
          "   s32i a1, %0, (4 * " STRINGIFY(REG_A1) ")\n"
          "   s32i a2, %0, (4 * " STRINGIFY(REG_A2) ")\n"
          "   s32i a3, %0, (4 * " STRINGIFY(REG_A3) ")\n"
          "   movi a2, 1f\n"
          "1: s32i a2, %0, (4 * " STRINGIFY(REG_PC) ")\n"
          "   rsr  a2, PS\n"
          "   s32i a2, %0, (4 * " STRINGIFY(REG_PS) ")\n"
          "   s32i a4, %0, (4 * " STRINGIFY(REG_A4) ")\n"
          "   s32i a5, %0, (4 * " STRINGIFY(REG_A5) ")\n"
          "   s32i a6, %0, (4 * " STRINGIFY(REG_A6) ")\n"
          "   s32i a7, %0, (4 * " STRINGIFY(REG_A7) ")\n"
          "   s32i a8, %0, (4 * " STRINGIFY(REG_A8) ")\n"
          "   s32i a9, %0, (4 * " STRINGIFY(REG_A9) ")\n"
          "   s32i a10, %0, (4 * " STRINGIFY(REG_A10) ")\n"
          "   s32i a11, %0, (4 * " STRINGIFY(REG_A11) ")\n"
#ifndef __XTENSA_CALL0_ABI__
          "   s32i a12, %0, (4 * " STRINGIFY(REG_A12) ")\n"
          "   s32i a13, %0, (4 * " STRINGIFY(REG_A13) ")\n"
          "   s32i a14, %0, (4 * " STRINGIFY(REG_A14) ")\n"
          "   s32i a15, %0, (4 * " STRINGIFY(REG_A15) ")\n"
#endif
          "   rur  a2, THREADPTR\n"
          "   s32i a2, %0, (4 * " STRINGIFY(THREADPTR) ")\n"
          "   rsr  a2, SAR\n"
          "   s32i a2, %0, (4 * " STRINGIFY(REG_SAR) ")\n"
#if XCHAL_HAVE_S32C1I != 0
          "   rsr  a2, SCOMPARE1\n"
          "   s32i a2, %0, (4 * " STRINGIFY(REG_SCOMPARE1) ")\n"
#endif
#if XCHAL_HAVE_LOOPS != 0
          "   rsr  a2, LBEG\n"
          "   s32i a2, %0, (4 * " STRINGIFY(REG_LBEG) ")\n"
          "   rsr  a2, LEND\n"
          "   s32i a2, %0, (4 * " STRINGIFY(REG_LEND) ")\n"
          "   rsr  a2, LCOUNT\n"
          "   s32i a2, %0, (4 * " STRINGIFY(REG_LCOUNT) ")\n"
#endif
          :
          : "a" (saveregs)
        );
      return 0;
    }

  return sys_call1(SYS_save_context, (uintptr_t)saveregs);
}
