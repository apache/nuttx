/****************************************************************************
 * drivers/timers/sp804.h
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

#ifndef __DRIVERS_TIMERS_SP804_H
#define __DRIVERS_TIMERS_SP804_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SP804 per-timer register offsets */

#define SP804_LOAD          0x00  /* Load register */
#define SP804_VALUE         0x04  /* Current value (RO) */
#define SP804_CONTROL       0x08  /* Control register */
#define SP804_INTCLR        0x0c  /* Interrupt clear (WO) */

/* Timer 2 sits at base + 0x20 with the same layout */

#define SP804_TIMER2_OFF    0x20

/* Control register bits */

#define SP804_CTRL_ONESHOT  (1 << 0)  /* One-shot mode */
#define SP804_CTRL_32BIT    (1 << 1)  /* 32-bit counter */
#define SP804_CTRL_IE       (1 << 5)  /* Interrupt enable */
#define SP804_CTRL_PERIODIC (1 << 6)  /* Periodic (auto-reload) mode */
#define SP804_CTRL_ENABLE   (1 << 7)  /* Timer enable */

/* Helper macros */

#define sp804_getreg(b, o) \
  (*(FAR volatile uint32_t *)((b) + (o)))
#define sp804_putreg(v, b, o) \
  (*((FAR volatile uint32_t *)((b) + (o))) = (v))

#endif /* __DRIVERS_TIMERS_SP804_H */
