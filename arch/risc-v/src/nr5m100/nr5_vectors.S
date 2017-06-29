/************************************************************************************
 * arch/risc-v/src/nr5m100/nr5_vectors.S
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "nr5_csr.h"

	.section .text.vec
	.global	__reset_vec
	.global	__trap_vec


/************************************************************************************
 * ADDR: 00h - RESET
 ************************************************************************************/

__reset_vec:
	csrw	NR5_EPIC_PRIMASK, 7
	j		__start

/************************************************************************************
 * ADDR: 08h - PRI0 (Illegal Instruction, Bus Error)
 ************************************************************************************/

__trap_vec:
__pri0_vec:
	j		irq_handler
	nop

/************************************************************************************
 * ADDR: 10h - PRI1 Interrupt Handler
 ************************************************************************************/

__pri1_vec:
	j		irq_handler
	nop

/************************************************************************************
 * ADDR: 18h - PRI2 Interrupt Handler
 ************************************************************************************/

__pri2_vec:
	j		irq_handler
	nop

/************************************************************************************
 * ADDR: 20h - PRI3 Interrupt Handler
 ************************************************************************************/

__pri3_vec:
	j		irq_handler
	nop

/************************************************************************************
 * ADDR: 28h - PRI4 Interrupt Handler
 ************************************************************************************/

__pri4_vec:
	j		irq_handler
	nop

/****************************************************************************
 * Modeline to set vim formatting options for ASM file.  For this to work,
 * you must enable moeline processing in your ~/.vimrc file with:
 *
 *    ~/.vimrc:
 *      set modeline
 *
 * vim: noet:ts=4:sw=4
 ****************************************************************************/
