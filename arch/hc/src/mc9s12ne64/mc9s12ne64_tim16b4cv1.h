/************************************************************************************
 * arch/hc/src/mc9s12ne64/mc9s12ne64_tim16b4cv1.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_TIM_H
#define __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_TIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_TIM_TIOS_OFFSET         0x0000 /* Timer Input Capture/Output Compare Select */
#define HCS12_TIM_CFORC_OFFSET        0x0001 /* Timer Compare Force Register */
#define HCS12_TIM_OC7M_OFFSET         0x0002 /* Output Compare 7 Mask Register */
#define HCS12_TIM_OC7D_OFFSET         0x0003 /* Output Compare 7 Data Register */
#define HCS12_TIM_TCNTHI2_OFFSET      0x0004 /* Timer Count Register */
#define HCS12_TIM_TCNTLO2_OFFSET      0x0005 /* Timer Count Register */
#define HCS12_TIM_TSCR1_OFFSET        0x0006 /* Timer System Control Register 1 */
#define HCS12_TIM_TTOV_OFFSET         0x0007 /* Timer Toggle Overflow Register */
#define HCS12_TIM_TCTL1_OFFSET        0x0008 /* Timer Control Register1 */
#define HCS12_TIM_TCTL3_OFFSET        0x000a /* Timer Control Register3 */
#define HCS12_TIM_TIE_OFFSET          0x000c /* Timer Interrupt Enable Register */
#define HCS12_TIM_TSCR2_OFFSET        0x000d /* Timer System Control Register 2 */
#define HCS12_TIM_TFLG1_OFFSET        0x000e /* Main Timer Interrupt Flag 1 */
#define HCS12_TIM_TFLG2_OFFSET        0x000f /* Main Timer Interrupt Flag 2 */
#define HCS12_TIM_TC4HI_OFFSET        0x0018 /* Timer Input Capture/Output Compare Register 4 */
#define HCS12_TIM_TC4LO_OFFSET        0x0019 /* Timer Input Capture/Output Compare Register 4 */
#define HCS12_TIM_TC5HI_OFFSET        0x001a /* Timer Input Capture/Output Compare Register 5 */
#define HCS12_TIM_TC5LO_OFFSET        0x001b /* Timer Input Capture/Output Compare Register 5 */
#define HCS12_TIM_TC6HI_OFFSET        0x001c /* Timer Input Capture/Output Compare Register 6 */
#define HCS12_TIM_TC6LO_OFFSET        0x001d /* Timer Input Capture/Output Compare Register 6 */
#define HCS12_TIM_TC7HI_OFFSET        0x001e /* Timer Input Capture/Output Compare Register 7 */
#define HCS12_TIM_TC7LO_OFFSET        0x001f /* Timer Input Capture/Output Compare Register 7 */
#define HCS12_TIM_PACTL_OFFSET        0x0020 /* 16-Bit Pulse Accumulator Control Register */
#define HCS12_TIM_PAFLG_OFFSET        0x0021 /* Pulse Accumulator Flag Register */
#define HCS12_TIM_PACNTHI_OFFSET      0x0022 /* Pulse Accumulator Count Register */
#define HCS12_TIM_PACNTLO_OFFSET      0x0023 /* Pulse Accumulator Count Register */
#define HCS12_TIM_TIMTST2_OFFSET      0x002d /* Timer Test Register */

/* Register Addresses ***************************************************************/

#define HCS12_TIM_TIOS                (HCS12_TIM_BASE+HCS12_TIM_TIOS_OFFSET)
#define HCS12_TIM_CFORC               (HCS12_TIM_BASE+HCS12_TIM_CFORC_OFFSET)
#define HCS12_TIM_OC7M                (HCS12_TIM_BASE+HCS12_TIM_OC7M_OFFSET)
#define HCS12_TIM_OC7D                (HCS12_TIM_BASE+HCS12_TIM_OC7D_OFFSET)
#define HCS12_TIM_TCNTHI2             (HCS12_TIM_BASE+HCS12_TIM_TCNTHI2_OFFSET)
#define HCS12_TIM_TCNTLO2             (HCS12_TIM_BASE+HCS12_TIM_TCNTLO2_OFFSET)
#define HCS12_TIM_TSCR1               (HCS12_TIM_BASE+HCS12_TIM_TSCR1_OFFSET)
#define HCS12_TIM_TTOV                (HCS12_TIM_BASE+HCS12_TIM_TTOV_OFFSET)
#define HCS12_TIM_TCTL1               (HCS12_TIM_BASE+HCS12_TIM_TCTL1_OFFSET)
#define HCS12_TIM_TCTL3               (HCS12_TIM_BASE+HCS12_TIM_TCTL3_OFFSET)
#define HCS12_TIM_TIE                 (HCS12_TIM_BASE+HCS12_TIM_TIE_OFFSET)
#define HCS12_TIM_TSCR2               (HCS12_TIM_BASE+HCS12_TIM_TSCR2_OFFSET)
#define HCS12_TIM_TFLG1               (HCS12_TIM_BASE+HCS12_TIM_TFLG1_OFFSET)
#define HCS12_TIM_TFLG2               (HCS12_TIM_BASE+HCS12_TIM_TFLG2_OFFSET)
#define HCS12_TIM_TC4HI               (HCS12_TIM_BASE+HCS12_TIM_TC4HI_OFFSET)
#define HCS12_TIM_TC4LO               (HCS12_TIM_BASE+HCS12_TIM_TC4LO_OFFSET)
#define HCS12_TIM_TC5HI               (HCS12_TIM_BASE+HCS12_TIM_TC5HI_OFFSET)
#define HCS12_TIM_TC5LO               (HCS12_TIM_BASE+HCS12_TIM_TC6HI_OFFSET)
#define HCS12_TIM_TC6HI               (HCS12_TIM_BASE+HCS12_TIM_TC6LO_OFFSET)
#define HCS12_TIM_TC6LO               (HCS12_TIM_BASE+HCS12_TIM_TC7HI_OFFSET)
#define HCS12_TIM_TC7HI               (HCS12_TIM_BASE+HCS12_TIM_TC7LO_OFFSET)
#define HCS12_TIM_TC7LO               (HCS12_TIM_BASE+HCS12_TIM_PACTL_OFFSET)
#define HCS12_TIM_PACTL               (HCS12_TIM_BASE+HCS12_TIM_PAFLG_OFFSET)
#define HCS12_TIM_PAFLG               (HCS12_TIM_BASE+HCS12_TIM_PACNTHI_OFFSET)
#define HCS12_TIM_PACNTHI             (HCS12_TIM_BASE+HCS12_TIM_PACNTLO_OFFSET)
#define HCS12_TIM_PACNTLO             (HCS12_TIM_BASE+HCS12_TIM_TIMTST2_OFFSET)
#define HCS12_TIM_TIMTST2             (HCS12_TIM_BASE+HCS12_TIM_TIMTST2_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Timer Input Capture/Output Compare Select */
#define TIM_TIOS_
/* Timer Compare Force Register */
#define TIM_CFORC_
/* Output Compare 7 Mask Register */
#define TIM_OC7M_
/* Output Compare 7 Data Register */
#define TIM_OC7D_
/* Timer Count Register */
#define TIM_TCNTHI2_
/* Timer Count Register */
#define TIM_TCNTLO2_
/* Timer System Control Register 1 */
#define TIM_TSCR1_
/* Timer Toggle Overflow Register */
#define TIM_TTOV_
/* Timer Control Register1 */
#define TIM_TCTL1_
/* Timer Control Register3 */
#define TIM_TCTL3_
/* Timer Interrupt Enable Register */
#define TIM_TIE_
/* Timer System Control Register 2 */
#define TIM_TSCR2_
/* Main Timer Interrupt Flag 1 */
#define TIM_TFLG1_
/* Main Timer Interrupt Flag 2 */
#define TIM_TFLG2_
/* Timer Input Capture/Output Compare Register 4 */
#define TIM_TC4HI_
/* Timer Input Capture/Output Compare Register 4 */
#define TIM_TC4LO_
/* Timer Input Capture/Output Compare Register 5 */
#define TIM_TC5HI_
/* Timer Input Capture/Output Compare Register 5 */
#define TIM_TC5LO_
/* Timer Input Capture/Output Compare Register 6 */
#define TIM_TC6HI_
/* Timer Input Capture/Output Compare Register 6 */
#define TIM_TC6LO_
/* Timer Input Capture/Output Compare Register 7 */
#define TIM_TC7HI_
/* Timer Input Capture/Output Compare Register 7 */
#define TIM_TC7LO_
/* 16-Bit Pulse Accumulator Control Register */
#define TIM_PACTL_
/* Pulse Accumulator Flag Register */
#define TIM_PAFLG_
/* Pulse Accumulator Count Register */
#define TIM_PACNTHI_
/* Pulse Accumulator Count Register */
#define TIM_PACNTLO_
/* Timer Test Register */
#define TIM_TIMTST2_

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_TIM_H */
