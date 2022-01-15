/****************************************************************************
 * arch/arm/src/phy62xx/timer.h
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

#ifndef __ARCH_ARM_SRC_PHY62XX_TIMER_H
#define __ARCH_ARM_SRC_PHY62XX_TIMER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "types.h"
#include "bus_dev.h"

#define FREE_TIMER_NUMBER 2

typedef enum
{
    AP_TIMER_ID_5 = 5,
    AP_TIMER_ID_6 = 6,
} User_Timer_e;

enum
{
    HAL_EVT_TIMER_5 = AP_TIMER_ID_5,
    HAL_EVT_TIMER_6 = AP_TIMER_ID_6,
    HAL_EVT_WAKEUP = 0x10,
    HAL_EVT_SLEEP
};

typedef void(*ap_tm_hdl_t)(uint8_t evt);

int hal_timer_init(ap_tm_hdl_t callback);

int hal_timer_deinit(void);

int hal_timer_set(User_Timer_e timeId, uint32_t us);

int hal_timer_mask_int(User_Timer_e timeId, bool en);

int hal_timer_stop(User_Timer_e timeId);

void __attribute__((used)) hal_TIMER5_IRQHandler(void);
void __attribute__((used)) hal_TIMER6_IRQHandler(void);

extern void set_timer(AP_TIM_TypeDef *TIMx, int time);

extern uint32_t read_current_fine_time(void);

extern uint32  read_LL_remainder_time(void);

#ifndef BASE_TIME_UINTS
#define BASE_TIME_UNITS   (0x3fffff)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_PHY62XX_TIMER_H */
