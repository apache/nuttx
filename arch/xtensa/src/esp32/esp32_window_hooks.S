/****************************************************************************
 * arch/xtensa/src/esp32/esp32_window_hooks.S
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

	.file	"esp32_window_hooks.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip_macros.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_XTENSA_HAVE_WINDOW_EXCEPTION_HOOKS
	/* PID Controller is configured to switch to PID 0 when the CPU fetches
	 * instruction from the following window exception vectors:
	 * - Window Overflow 4: mapped as Level 2 vector entry address
	 * - Window Underflow 4: mapped as Level 4 vector entry address
	 * - Window Overflow 8: mapped as Level 5 vector entry address
	 * - Window Underflow 8: mapped as Level 6 vector entry address
	 *
	 * See arch/xtensa/src/esp32/esp32_userspace.c
	 */

/****************************************************************************
 * Name: _overflow4_exit_hook
 ****************************************************************************/

	.section	HANDLER_SECTION, "ax"
	.global 	_overflow4_exit_hook
	.type   	_overflow4_exit_hook, @function
	.align  	4

_overflow4_exit_hook:
	wsr 	a0, misc0
	wsr 	a1, misc1

	/* Restore PID to the same value prior to the exception */

	get_prev_pid 2 a0
	set_next_pid a0 a1

	rsr 	a0, misc0
	rsr 	a1, misc1
	rfwo				/* rotates back to call[i] position */

/****************************************************************************
 * Name: _underflow4_exit_hook
 ****************************************************************************/

	.section	HANDLER_SECTION, "ax"
	.global 	_underflow4_exit_hook
	.type   	_underflow4_exit_hook, @function
	.align  	4

_underflow4_exit_hook:
	wsr 	a0, misc0
	wsr 	a1, misc1

	/* Restore PID to the same value prior to the exception */

	get_prev_pid 4 a0
	set_next_pid a0 a1

	rsr 	a0, misc0
	rsr 	a1, misc1
	rfwu

/****************************************************************************
 * Name: _overflow8_exit_hook
 ****************************************************************************/

	.section	HANDLER_SECTION, "ax"
	.global 	_overflow8_exit_hook
	.type   	_overflow8_exit_hook, @function
	.align  	4

_overflow8_exit_hook:
	wsr 	a0, misc0
	wsr 	a1, misc1

	/* Restore PID to the same value prior to the exception */

	get_prev_pid 5 a0
	set_next_pid a0 a1

	rsr 	a0, misc0
	rsr 	a1, misc1
	rfwo				/* rotates back to call[i] position */

/****************************************************************************
 * Name: _underflow8_exit_hook
 ****************************************************************************/

	.section	HANDLER_SECTION, "ax"
	.global 	_underflow8_exit_hook
	.type   	_underflow8_exit_hook, @function
	.align  	4

_underflow8_exit_hook:
	wsr 	a0, misc0
	wsr 	a1, misc1

	/* Restore PID to the same value prior to the exception */

	get_prev_pid 6 a0
	set_next_pid a0 a1

	rsr 	a0, misc0
	rsr 	a1, misc1
	rfwu

#endif /* CONFIG_XTENSA_HAVE_WINDOW_EXCEPTION_HOOKS */
