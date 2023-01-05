############################################################################
# libs/libc/machine/risc-v/gnu/arch_setjmp.S
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

#include "asm.h"

.section .text
.globl  setjmp
.type   setjmp, @function

setjmp:
	REG_S ra,   0*SZREG(a0)
	REG_S s0,   1*SZREG(a0)
	REG_S s1,   2*SZREG(a0)
	REG_S s2,   3*SZREG(a0)
	REG_S s3,   4*SZREG(a0)
	REG_S s4,   5*SZREG(a0)
	REG_S s5,   6*SZREG(a0)
	REG_S s6,   7*SZREG(a0)
	REG_S s7,   8*SZREG(a0)
	REG_S s8,   9*SZREG(a0)
	REG_S s9,  10*SZREG(a0)
	REG_S s10, 11*SZREG(a0)
	REG_S s11, 12*SZREG(a0)
	REG_S sp,  13*SZREG(a0)

	addi  a0, a0, 14 * SZREG

#ifdef CONFIG_ARCH_FPU
	FREG_S  fs0,   0*SZFREG(a0)
	FREG_S  fs1,   1*SZFREG(a0)
	FREG_S  fs2,   2*SZFREG(a0)
	FREG_S  fs3,   3*SZFREG(a0)
	FREG_S  fs4,   4*SZFREG(a0)
	FREG_S  fs5,   5*SZFREG(a0)
	FREG_S  fs6,   6*SZFREG(a0)
	FREG_S  fs7,   7*SZFREG(a0)
	FREG_S  fs8,   8*SZFREG(a0)
	FREG_S  fs9,   9*SZFREG(a0)
	FREG_S  fs10, 10*SZFREG(a0)
	FREG_S  fs11, 11*SZFREG(a0)
#endif

	li    a0, 0
	ret
	.size	setjmp, .-setjmp

/* volatile void longjmp (jmp_buf, int);  */

.section .text
.globl  longjmp
.type   longjmp, @function
longjmp:
	REG_L ra,   0*SZREG(a0)
	REG_L s0,   1*SZREG(a0)
	REG_L s1,   2*SZREG(a0)
	REG_L s2,   3*SZREG(a0)
	REG_L s3,   4*SZREG(a0)
	REG_L s4,   5*SZREG(a0)
	REG_L s5,   6*SZREG(a0)
	REG_L s6,   7*SZREG(a0)
	REG_L s7,   8*SZREG(a0)
	REG_L s8,   9*SZREG(a0)
	REG_L s9,  10*SZREG(a0)
	REG_L s10, 11*SZREG(a0)
	REG_L s11, 12*SZREG(a0)
	REG_L sp,  13*SZREG(a0)

	addi  a0, a0, 14 * SZREG

#ifdef CONFIG_ARCH_FPU
	FREG_L  fs0,   0*SZFREG(a0)
	FREG_L  fs1,   1*SZFREG(a0)
	FREG_L  fs2,   2*SZFREG(a0)
	FREG_L  fs3,   3*SZFREG(a0)
	FREG_L  fs4,   4*SZFREG(a0)
	FREG_L  fs5,   5*SZFREG(a0)
	FREG_L  fs6,   6*SZFREG(a0)
	FREG_L  fs7,   7*SZFREG(a0)
	FREG_L  fs8,   8*SZFREG(a0)
	FREG_L  fs9,   9*SZFREG(a0)
	FREG_L  fs10, 10*SZFREG(a0)
	FREG_L  fs11, 11*SZFREG(a0)
#endif

	seqz a0, a1
	add  a0, a0, a1   # a0 = (a1 == 0) ? 1 : a1
	ret
	.size	longjmp, .-longjmp
