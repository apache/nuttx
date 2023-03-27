/****************************************************************************
 * arch/risc-v/src/litex/hardware/litex_timer.h
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

#ifndef __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_TIMER_H
#define __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/litex_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LITEX_TIMER register offsets *********************************************/

#define LITEX_TIMER_LOAD_OFFSET            0x0000
#define LITEX_TIMER_RELOAD_OFFSET          0x0004
#define LITEX_TIMER_EN_OFFSET              0x0008
#define LITEX_TIMER_UPDATE_VALUE_OFFSET    0x000C
#define LITEX_TIMER_VALUE_OFFSET           0x0010
#define LITEX_TIMER_EV_STATUS_OFFSET       0x0014
#define LITEX_TIMER_EV_PENDING_OFFSET      0x0018
#define LITEX_TIMER_EV_ENABLE_OFFSET       0x001C

/* LITEX_TIMER register addresses *******************************************/

#define LITEX_TIMER0_LOAD           (LITEX_TIMER0_BASE+LITEX_TIMER_LOAD_OFFSET)
#define LITEX_TIMER0_RELOAD         (LITEX_TIMER0_BASE+LITEX_TIMER_RELOAD_OFFSET)
#define LITEX_TIMER0_EN             (LITEX_TIMER0_BASE+LITEX_TIMER_EN_OFFSET)
#define LITEX_TIMER0_UPDATE_VALUE   (LITEX_TIMER0_BASE+LITEX_TIMER_UPDATE_VALUE_OFFSET)
#define LITEX_TIMER0_VALUE          (LITEX_TIMER0_BASE+LITEX_TIMER_VALUE_OFFSET)
#define LITEX_TIMER0_EV_STATUS      (LITEX_TIMER0_BASE+LITEX_TIMER_EV_STATUS_OFFSET)
#define LITEX_TIMER0_EV_PENDING     (LITEX_TIMER0_BASE+LITEX_TIMER_EV_PENDING_OFFSET)
#define LITEX_TIMER0_EV_ENABLE      (LITEX_TIMER0_BASE+LITEX_TIMER_EV_ENABLE_OFFSET)

/* LITEX_TIMER register bit definitions *************************************/

#endif /* __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_TIMER_H */
