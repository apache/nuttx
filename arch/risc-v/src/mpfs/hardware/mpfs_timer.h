/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_timer.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_TIMER_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define MPFS_MSTIMER_TIM1VALUE_OFFSET       0x000 /* Current value of timer 1 */
#define MPFS_MSTIMER_TIM1LOADVAL_OFFSET     0x004 /* Load value for timer 1 */
#define MPFS_MSTIMER_TIM1BGLOADVAL_OFFSET   0x008 /* Background load value for timer 1 */
#define MPFS_MSTIMER_TIM1CONTROL_OFFSET     0x00C /* Timer 1 Control Register */
#define MPFS_MSTIMER_TIM1RIS_OFFSET         0x010 /* Raw interrupt status bit for timer 1 */
#define MPFS_MSTIMER_TIM1MIS_OFFSET         0x014 /* Masked interrupt status bit for timer 1 */
#define MPFS_MSTIMER_TIM2VALUE_OFFSET       0x018 /* Current value of timer 2 */
#define MPFS_MSTIMER_TIM2LOADVAL_OFFSET     0x01C /* Load value for timer 2 */
#define MPFS_MSTIMER_TIM2BGLOADVAL_OFFSET   0x020 /* Background load value for timer 2 */
#define MPFS_MSTIMER_TIM2CONTROL_OFFSET     0x024 /* Timer 2 Control Register */
#define MPFS_MSTIMER_TIM2RIS_OFFSET         0x028 /* Raw interrupt status bit for timer 2 */
#define MPFS_MSTIMER_TIM2MIS_OFFSET         0x02C /* Masked interrupt status bit for timer 2 */
#define MPFS_MSTIMER_TIM64VALUEU_OFFSET     0x030 /* Current value of the upper 32 bit word of the 64 bit timer */
#define MPFS_MSTIMER_TIM64VALUEL_OFFSET     0x034 /* Current value of the lower 32 bit word of the 64 bit timer */
#define MPFS_MSTIMER_TIM64LOADVALU_OFFSET   0x038 /* Load value for upper 32 bits of 64 bit timer */
#define MPFS_MSTIMER_TIM64LOADVALL_OFFSET   0x03C /* Load value for lower 32 bits of 64 bit timer */
#define MPFS_MSTIMER_TIM64BGLOADVALU_OFFSET 0x040 /* Background load value for upper 32bits of 64 bit mode timer */
#define MPFS_MSTIMER_TIM64BGLOADVALL_OFFSET 0x044 /* Background load value for lower 32bits of 64 bit mode timer */
#define MPFS_MSTIMER_TIM64CONTROL_OFFSET    0x048 /* 64 bit Timer Control Register */
#define MPFS_MSTIMER_TIM64RIS_OFFSET        0x04C /* Raw interrupt status bit for 64 bit mode timer */
#define MPFS_MSTIMER_TIM64MIS_OFFSET        0x050 /* Masked interrupt status bit for 64 bit mode timer */
#define MPFS_MSTIMER_TIM64MODE_OFFSET       0x054 /* Register to enable/disable 64 bit mode timer */

/* Timer Control Registers */
#define MPFS_MSTIMER_ENABLE_MASK            (1 << 0) /* Bit 0:  Enable bit for timer */
#define MPFS_MSTIMER_MODE_MASK              (1 << 1) /* Bit 1:  Operating mode for timer  */
#define MPFS_MSTIMER_INTEN_MASK             (1 << 2) /* Bit 2:  Timer interrupt enable bit */

/* Raw interrupt status bit for timers */
#define MPFS_MSTIMER_RIS_MASK               (1 << 0) /* Bit 0:  Raw interrupt status bit for timer */

/* Timer 64bit mode register */
#define MPFS_MSTIMER_64BITMODE_MASK         (1 << 0) /* Bit 0:  Enable/disable 64 bit mode timer */

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_TIMER_H */
