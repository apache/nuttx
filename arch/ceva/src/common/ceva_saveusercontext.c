/****************************************************************************
 * arch/ceva/src/common/ceva_saveusercontext.c
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

#include <arch/syscall.h>

#include "ceva_internal.h"

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
 * Return:
 *   0: Normal return
 *   1: Context switch return
 *
 ****************************************************************************/

int up_saveusercontext(void *saveregs)
{
  int ret;

  /* Let sys_call1() do all of the work */

  ret = sys_call1(SYS_save_context, (uintptr_t)saveregs);
  if (ret == 0)
    {
      /* There are two return conditions.  On the first return, A0 (the
       * return value will be zero.  On the second return we need to
       * force A0 to be 1.
       */

      saveregs[REG_A0] = 1;
    }

  return ret;
}
