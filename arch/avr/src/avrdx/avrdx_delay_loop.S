/****************************************************************************
 * arch/avr/src/avrdx/avrdx_delay_loop.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * External Symbols
 ****************************************************************************/

	.file	"avrdx_delay_loop.S"

/****************************************************************************
 * Name: avrdx_delay_loop
 *
 * Description:
 *   Spins in a loop to cause a delay. On chips with AVRxt cores, it takes:
 *   (3 + count * 6 + call) cycles where count is the parameter and call
 *   is the number of cycles needed for instruction that jumps into this
 *   function (2 cycles for RCALL or 3 cycles for CALL, depends on what
 *   the compiler/linker does.)
 *
 * Input Parameters:
 *   Loop count - uint32_t (registers r22:r25)
 *
 * Return value: none
 *
 * Assumptions/Limitations:
 *   - only called from avrdx_udelay
 *   - count is not zero
 *
 ****************************************************************************/

	.global avrdx_delay_loop

avrdx_delay_loop:

	; Because:
	; - all of the registers that contain loop count are call-clobbered.
	; - avrdx_udelay does not call this function with count set to zero
	; we can start counting right away

	; Note - r22 cannot be used with SBIW. SBIW takes two clock cycles
	; though so we can achieve the same timing with with SUBI/SBC

1:
	subi r22, 1    ; subtract immediate, 1 cycle
	sbc r23, r1    ; subtract zero register from r26 with carry
	               ; 1 cycle
	sbc r24, r1    ; 1 cycle
	sbc r25, r1    ; 1 cycle
	brne 1b        ; 2 cycles if the condition is true (ie. if branching)
	               ; 1 cycle otherwise

	; 1 pass through the loop takes 6 cycles total (except for last pass
	; which takes 5 cycles)

2:
	ret            ; 4 cycles
