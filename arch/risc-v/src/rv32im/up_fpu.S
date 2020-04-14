/************************************************************************************
 * arch/risc-v/src/rv32im/up_fpu.S
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/rv32im/irq.h>

#ifdef CONFIG_ARCH_FPU

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	.globl		up_fpuconfig
	.globl		up_savefpu
	.globl		up_restorefpu

	.file		"up_fpu.S"

#define FS_MASK		0x6000
#define FS_OFF		0x0000
#define FS_INITIAL	0x2000
#define FS_CLEAN	0x4000
#define FS_DIRTY	0x6000

#if defined(CONFIG_ARCH_DPFPU)
#  define FLOAD		fld
#  define FSTORE	fsd
#elif defined(CONFIG_ARCH_QPFPU)
#  define FLOAD		flq
#  define FSTORE	fsq
#else
#  define FLOAD		flw
#  define FSTORE	fsw
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_fpuconfig
 *
 * Description:
 *   init fpu
 *
 * C Function Prototype:
 *   void up_fpuconfig(void);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   This function does not return anything explicitly.
 *
 ************************************************************************************/

	.type		up_fpuconfig, function

up_fpuconfig:
	li		a0, FS_INITIAL
	csrs		mstatus, a0
	csrwi		fcsr, 0
	ret

/************************************************************************************
 * Name: up_savefpu
 *
 * Description:
 *   Given the pointer to a register save area (in A0), save the state of the
 *   floating point registers.
 *
 * C Function Prototype:
 *   void up_savefpu(uint32_t *regs);
 *
 * Input Parameters:
 *   regs - A pointer to the register save area in which to save the floating point
 *     registers
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

	.type		up_savefpu, function

up_savefpu:
	lw		t0, REG_INT_CTX(a0)
	li		t1, FS_MASK
	and		t2, t0, t1
	li		t1, FS_DIRTY
	bne		t2, t1, 1f
	li		t1, ~FS_MASK
	and		t0, t0, t1
	li		t1, FS_CLEAN
	or		t0, t0, t1
	sw		t0, REG_INT_CTX(a0)

	/* Store all floating point registers */

	FSTORE		f0,  REG_F0(a0)
	FSTORE		f1,  REG_F1(a0)
	FSTORE		f2,  REG_F2(a0)
	FSTORE		f3,  REG_F3(a0)
	FSTORE		f4,  REG_F4(a0)
	FSTORE		f5,  REG_F5(a0)
	FSTORE		f6,  REG_F6(a0)
	FSTORE		f7,  REG_F7(a0)
	FSTORE		f8,  REG_F8(a0)
	FSTORE		f9,  REG_F9(a0)
	FSTORE		f10, REG_F10(a0)
	FSTORE		f11, REG_F11(a0)
	FSTORE		f12, REG_F12(a0)
	FSTORE		f13, REG_F13(a0)
	FSTORE		f14, REG_F14(a0)
	FSTORE		f15, REG_F15(a0)
	FSTORE		f16, REG_F16(a0)
	FSTORE		f17, REG_F17(a0)
	FSTORE		f18, REG_F18(a0)
	FSTORE		f19, REG_F19(a0)
	FSTORE		f20, REG_F20(a0)
	FSTORE		f21, REG_F21(a0)
	FSTORE		f22, REG_F22(a0)
	FSTORE		f23, REG_F23(a0)
	FSTORE		f24, REG_F24(a0)
	FSTORE		f25, REG_F25(a0)
	FSTORE		f26, REG_F26(a0)
	FSTORE		f27, REG_F27(a0)
	FSTORE		f28, REG_F28(a0)
	FSTORE		f29, REG_F29(a0)
	FSTORE		f30, REG_F30(a0)
	FSTORE		f31, REG_F31(a0)

	frcsr		t0
	sw		t0, REG_FCSR(a0)

1:
	ret

/************************************************************************************
 * Name: up_restorefpu
 *
 * Description:
 *   Given the pointer to a register save area (in A0), restore the state of the
 *   floating point registers.
 *
 * C Function Prototype:
 *   void up_restorefpu(const uint32_t *regs);
 *
 * Input Parameters:
 *   regs - A pointer to the register save area containing the floating point
 *     registers.
 *
 * Returned Value:
 *   This function does not return anything explicitly.  However, it is called from
 *   interrupt level assembly logic that assumes that r0 is preserved.
 *
 ************************************************************************************/

	.type		up_restorefpu, function

up_restorefpu:
	lw		t0, REG_INT_CTX(a0)
	li		t1, FS_MASK
	and		t2, t0, t1
	li		t1, FS_INITIAL
	ble		t2, t1, 1f

	/* Load all floating point registers */

	FLOAD		f0, REG_F0(a0)
	FLOAD		f1, REG_F1(a0)
	FLOAD		f2, REG_F2(a0)
	FLOAD		f3, REG_F3(a0)
	FLOAD		f4, REG_F4(a0)
	FLOAD		f5, REG_F5(a0)
	FLOAD		f6, REG_F6(a0)
	FLOAD		f7, REG_F7(a0)
	FLOAD		f8, REG_F8(a0)
	FLOAD		f9, REG_F9(a0)
	FLOAD		f10, REG_F10(a0)
	FLOAD		f11, REG_F11(a0)
	FLOAD		f12, REG_F12(a0)
	FLOAD		f13, REG_F13(a0)
	FLOAD		f14, REG_F14(a0)
	FLOAD		f15, REG_F15(a0)
	FLOAD		f16, REG_F16(a0)
	FLOAD		f17, REG_F17(a0)
	FLOAD		f18, REG_F18(a0)
	FLOAD		f19, REG_F19(a0)
	FLOAD		f20, REG_F20(a0)
	FLOAD		f21, REG_F21(a0)
	FLOAD		f22, REG_F22(a0)
	FLOAD		f23, REG_F23(a0)
	FLOAD		f24, REG_F24(a0)
	FLOAD		f25, REG_F25(a0)
	FLOAD		f26, REG_F26(a0)
	FLOAD		f27, REG_F27(a0)
	FLOAD		f28, REG_F28(a0)
	FLOAD		f29, REG_F29(a0)
	FLOAD		f30, REG_F30(a0)
	FLOAD		f31, REG_F31(a0)

	/* Store the floating point control and status register */

	lw		t0, REG_FCSR(a0)
	fscsr		t0

1:
	ret

#endif /* CONFIG_ARCH_FPU */
