/****************************************************************************
 * arch/arm/src/pp502x/hardware/pp502x_timer.h
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

#ifndef __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_TIMER_H
#define __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/pp502x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Both timers and USEC_TIMER count at 1 MHz */

#define PP502X_TIMER1_CFG_OFFSET    0x0000  /* Timer 1 configuration */
#define PP502X_TIMER1_VAL_OFFSET    0x0004  /* Timer 1 value, read acks */
#define PP502X_TIMER2_CFG_OFFSET    0x0008  /* Timer 2 configuration */
#define PP502X_TIMER2_VAL_OFFSET    0x000c  /* Timer 2 value, read acks */
#define PP502X_USEC_TIMER_OFFSET    0x0010  /* Free-running microseconds */

/* Register Addresses *******************************************************/

#define PP502X_TIMER1_CFG   (PP502X_TIMER_BASE + PP502X_TIMER1_CFG_OFFSET)
#define PP502X_TIMER1_VAL   (PP502X_TIMER_BASE + PP502X_TIMER1_VAL_OFFSET)
#define PP502X_TIMER2_CFG   (PP502X_TIMER_BASE + PP502X_TIMER2_CFG_OFFSET)
#define PP502X_TIMER2_VAL   (PP502X_TIMER_BASE + PP502X_TIMER2_VAL_OFFSET)
#define PP502X_USEC_TIMER   (PP502X_TIMER_BASE + PP502X_USEC_TIMER_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* TIMERn_CFG */

#define PP502X_TIMER_CFG_ENABLE         (1 << 31)  /* Count and interrupt */
#define PP502X_TIMER_CFG_REPEAT         (1 << 30)  /* Reload at zero */
#define PP502X_TIMER_CFG_PERIOD_MASK    0x1fffffff /* Period less one, in us */

#endif /* __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_TIMER_H */
