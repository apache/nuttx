/****************************************************************************
 * arch/risc-v/src/ab32vg1/hardware/ab32vg1_timer.h
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

#ifndef __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_TIMER_H
#define __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/ab32vg1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer 0/1/2 (register manual, chapter 2) */

#define AB32VG1_TMR0CON         AB32VG1_SFR(0, 0x14)
#define AB32VG1_TMR0CPND        AB32VG1_SFR(0, 0x15)
#define AB32VG1_TMR0CNT         AB32VG1_SFR(0, 0x16)
#define AB32VG1_TMR0PR          AB32VG1_SFR(0, 0x17)

#define AB32VG1_TMR1CON         AB32VG1_SFR(0, 0x35)
#define AB32VG1_TMR1CPND        AB32VG1_SFR(0, 0x36)
#define AB32VG1_TMR1CNT         AB32VG1_SFR(0, 0x37)
#define AB32VG1_TMR1PR          AB32VG1_SFR(0, 0x38)

#define AB32VG1_TMR2CON         AB32VG1_SFR(0, 0x3a)
#define AB32VG1_TMR2CPND        AB32VG1_SFR(0, 0x3b)
#define AB32VG1_TMR2CNT         AB32VG1_SFR(0, 0x3c)
#define AB32VG1_TMR2PR          AB32VG1_SFR(0, 0x3d)

/* TMRxCON */

#define TMR_CON_TMREN           (1 << 0)          /* Timer enable */
#define TMR_CON_INCSEL_SHIFT    2                 /* Count source */
#define TMR_CON_INCSEL_SYSCLK   (0 << TMR_CON_INCSEL_SHIFT)
#define TMR_CON_INCSRC          (1 << 6)          /* 1: external pin */
#define TMR_CON_TIE             (1 << 7)          /* Overflow int enable */
#define TMR_CON_TPND            (1 << 9)          /* Overflow pending */

/* TMRxCPND */

#define TMR_CPND_TPCLR          (1 << 9)

#endif /* __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_TIMER_H */
