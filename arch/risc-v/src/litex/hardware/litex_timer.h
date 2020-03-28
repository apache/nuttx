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

#ifndef ARCH_RISCV_SRC_LITEX_CHIP_LITEX_TIMER_H
#define ARCH_RISCV_SRC_LITEX_CHIP_LITEX_TIMER_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_LOAD_OFFSET         0x00
#define TIMER_RELOAD_OFFSET       0x04
#define TIMER_EN_OFFSET           0x08
#define TIMER_UPDATE_VALUE_OFFSET 0x0c
#define TIMER_VALUE_OFFSET        0x10
#define TIMER_EV_STATUS_OFFSET    0x14
#define TIMER_EV_PENDING_OFFSET   0x18
#define TIMER_EV_ENABLE_OFFSET    0x1c

#define LITEX_TIMER0_LOAD         (LITEX_TIMER0_BASE + TIMER_LOAD_OFFSET)
#define LITEX_TIMER0_RELOAD       (LITEX_TIMER0_BASE + TIMER_RELOAD_OFFSET)
#define LITEX_TIMER0_EN           (LITEX_TIMER0_BASE + TIMER_EN_OFFSET)
#define LITEX_TIMER0_UPDATE_VALUE (LITEX_TIMER0_BASE + TIMER_UPDATE_VALUE_OFFSET)
#define LITEX_TIMER0_VALUE        (LITEX_TIMER0_BASE + TIMER_VALUE_OFFSET)
#define LITEX_TIMER0_EV_STATUS    (LITEX_TIMER0_BASE + TIMER_EV_STATUS_OFFSET)
#define LITEX_TIMER0_EV_PENDING   (LITEX_TIMER0_BASE + TIMER_EV_PENDING_OFFSET)
#define LITEX_TIMER0_EV_ENABLE    (LITEX_TIMER0_BASE + TIMER_EV_ENABLE_OFFSET)

#endif /* _ARCH_RISCV_SRC_LITEX_CHIP_LITEX_TICKTIMER_H */
