/****************************************************************************
 * arch/arm/src/phy62xx/phy6222_patch.c
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

#include "types.h"
#include "rom_sym_def.h"
#include <arch/irq.h>
#include "mcu_phy_bumbee.h"

extern int clear_timer_int(AP_TIM_TypeDef* TIMx);
extern void clear_timer(AP_TIM_TypeDef* TIMx);
extern void LL_evt_schedule(void);

#ifndef CONFIG_PHY6222_SDK
void TIM1_IRQHandler1(void)
{
  /*  HAL_ENTER_CRITICAL_SECTION() */

  if (AP_TIM1->status & 0x1)
    {
      clear_timer_int(AP_TIM1);
      clear_timer(AP_TIM1);
      LL_evt_schedule();
    }

  /* HAL_EXIT_CRITICAL_SECTION(); */
}
#endif
