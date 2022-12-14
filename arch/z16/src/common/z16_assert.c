/****************************************************************************
 * arch/z16/src/common/z16_assert.c
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
#include <nuttx/arch.h>

#include "z16_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static chipreg_t s_last_regs[XCPTCONTEXT_REGS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(void)
{
  FAR volatile uint32_t *regs32 = (FAR volatile uint32_t *)g_current_regs;

  board_autoled_on(LED_ASSERTION);

  if (regs32 == NULL)
    {
      up_saveusercontext(s_last_regs);
      regs32 = (FAR volatile uint32_t *)s_last_regs;
    }

  z16_registerdump(regs32);
}
