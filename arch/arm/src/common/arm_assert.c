/****************************************************************************
 * arch/arm/src/common/arm_assert.c
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

#include <stdio.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/board/board.h>

#include "sched/sched.h"
#include "arm_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t s_last_regs[XCPTCONTEXT_SIZE];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(void)
{
  struct tcb_s *rtcb = running_task();

  board_autoled_on(LED_ASSERTION);

  /* Update the xcp context */

  if (CURRENT_REGS)
    {
      rtcb->xcp.regs = (uint32_t *)CURRENT_REGS;
    }
  else
    {
      up_saveusercontext(s_last_regs);
      rtcb->xcp.regs = (uint32_t *)s_last_regs;
    }

  /* Dump the interrupt registers */

  arm_registerdump(rtcb->xcp.regs);
}
