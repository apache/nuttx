/****************************************************************************
 * arch/arm/src/armv8-m/arm_dispatch_syscall.S
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

#include <syscall.h>

#include "arm_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl		arm_dispatch_syscall

	.syntax		unified
	.thumb
	.file		"arm_dispatch_syscall.S"

/****************************************************************************
 * .text
 ****************************************************************************/

/****************************************************************************
 *   Call the stub function corresponding to the system call.  NOTE the non-
 *   standard parameter passing:
 *
 *     R0 = SYS_ call number
 *     R1 = parm0
 *     R2 = parm1
 *     R3 = parm2
 *     R4 = parm3
 *     R5 = parm4
 *     R6 = parm5
 *
 *   The values of R4-R5 may be preserved in the proxy called by the user
 *   code if they are used (but otherwise will not be).
 *
 *   Register usage:
 *
 *     R0 - Need not be preserved.
 *     R1-R3 - Need to be preserved until the stub is called.  The values of
 *       R0 and R1 returned by the stub must be preserved.
 *     R4-R11 must be preserved to support the expectations of the user-space
 *       callee.  R4-R6 may have been preserved by the proxy, but don't know
 *       for sure.
 *     R12 - Need not be preserved
 *     R13 - (stack pointer)
 *     R14 - Need not be preserved
 *     R15 - (PC)
 *
 ****************************************************************************/

	.text
	.thumb_func
	.type	arm_dispatch_syscall, function
arm_dispatch_syscall:
	.cfi_sections	.debug_frame
	.cfi_startproc
	sub	sp, sp, #32
	.cfi_def_cfa sp, 0
	str	r4, [sp, #0]
	.cfi_offset r4, 0
	str	r5, [sp, #4]
	.cfi_offset r5, 4
	str	r6, [sp, #8]
	.cfi_offset r6, 8
	str	lr, [sp, #12]
	.cfi_offset lr, 12
	str	ip, [sp, #16]
	.cfi_offset pc, 16
	ldr	ip, =g_stublookup
	ldr	ip, [ip, r0, lsl #2]
	blx	ip
	ldr	lr, [sp, #12]
	add	sp, sp, #32
	mov	r2, r0
	mov	r0, SYS_syscall_return
	svc	SYS_syscall
	.cfi_endproc
	.size	arm_dispatch_syscall, .-arm_dispatch_syscall
	.end
