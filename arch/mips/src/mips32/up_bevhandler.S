/****************************************************************************
 * arch/mips/src/mips32/up_bevexcptn.S
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/mips32/registers.h>

#include "chip/excptmacros.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Global Symbols
 ****************************************************************************/

	.file	"up_bevexcptn.S"
	.global	up_dobev

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: _bev_exception
 *
 * Description:
 *   Boot Exception Vector Handler.  Jumps to bev_handler
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

	.section .bev_excpt,"ax",@progbits
	.set	noreorder
	.ent	_bev_exception
_bev_exception:
	la		k0, _bev_handler
	jr		k0
	nop
	.end _bev_exception
			
/****************************************************************************
 * Name: _bev_handler
 *
 * Description:
 *   BEV exception handler
 *
 ****************************************************************************/

	.section .bev_handler, "ax", @progbits
	.set	noreorder
	.ent	_bev_handler
_bev_handler:
	EXCPT_PROLOGUE t0			/* Save registers on stack, enable nested interrupts */
	move	a0, sp				/* Pass register save structure as the parameter 1 */
	USE_INTSTACK t0, t1, t2		/* Switch to the interrupt stack */
	la		t0, up_dobev		/* Call up_dobev(regs) */
	jalr	t0, ra
	di							/* Disable interrupts */
	RESTORE_STACK t0, t1		/* Undo the operations of USE_STACK */
	EXCPT_EPILOGUE v0			/* Return to the context returned by up_dobev() */
	.end	_bev_handler
