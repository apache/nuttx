/****************************************************************************
 * arch/ceva/src/xc5/up_head.S
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
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK == 0
#  undef CONFIG_ARCH_INTERRUPTSTACK
#  define CONFIG_ARCH_INTERRUPTSTACK CONFIG_IDLETHREAD_STACKSIZE
#endif

.MACRO  IRQ_HANDLER irq retx
	push {dw}	retreg
	push {16dw,l}	r4, a8
	nop
	mov		#irq, a0
	call {t} INCODE	exception_common
	pop {16dw,l}	r4, a8
	pop {dw}	retreg
	retx
.ENDM

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file		"up_head.S"
	.extern		_g_idle_topstack
	.extern		_g_idle_basestack
	.extern		_up_doirq
	.extern		_up_start
	.extern		_up_relocate

/****************************************************************************
 * Interrupt Functions
 ****************************************************************************/
	.CSECT		inttbl
	br {t}		reset_handler

	.ORG		0x20
	IRQ_HANDLER	IRQ_TRAPE, retb

	.ORG		0x40
	IRQ_HANDLER	IRQ_TRAP, reti

	.ORG		0x60
	IRQ_HANDLER	IRQ_NMI, retn

	.ORG		0x80
	IRQ_HANDLER	IRQ_INT0, reti

	.ORG		0xc0
	IRQ_HANDLER	IRQ_INT1, reti

	.ORG		0x100
	IRQ_HANDLER	IRQ_INT2, reti

	.ORG		0x140
	IRQ_HANDLER	IRQ_INT3, reti

	.ORG		0x180
	IRQ_HANDLER	IRQ_INT4, reti

	.ORG		0x200
	IRQ_HANDLER	IRQ_TRAP0, reti

	.ORG		0x240
	IRQ_HANDLER	IRQ_TRAP1, reti

	.ORG		0x280
	IRQ_HANDLER	IRQ_TRAP2, reti

	.ORG		0x2c0
	IRQ_HANDLER	IRQ_TRAP3, reti

	.ORG		0x300
	.GLOBAL		_up_vintc_handler
_up_vintc_handler:
	IRQ_HANDLER	IRQ_VINT, reti

	.CSECT		resetsec
reset_handler:
	mov		#0x0, mod0
	mov		#0x0, mod1
	mov		#0x0, mod2
	mov		modg, a0
	and		a0, #0x300, a0
	or		a0, #0x0000001b, a0
	mov		a0, modg
	mov		#0xf0, modpb
	ld {dw}		(#_g_idle_topstack), a0
	nop
	nop
	nop
	nop
	mov		a0, sp
	nop
	nop
	push {dw}       retreg
	callr {t}	_up_relocate
	pop {dw}        retreg
	brr {t}		_up_start

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Common exception handling logic, need sync with:
 * arch/ceva/include/xc5/reg.h
 */
	.func_start	3 exception_common

exception_common:
	/* Note: a0 contain exception number
	 * Complete the context save
	 */

	push {16dw,h}	auxreg1
	push {16dw,h}	r4, a8
	push {dw}	mod3
	push {4dw}	modx
	bkst
	push {16dw,h}	auxreg0
	push {16dw,h}	auxreg2
	push {16dw,l}	auxreg1
	push {16dw,l}	auxreg0
	push {16dw,l}	auxreg2
	nop

/*#ifndef CONFIG_ARCH_XC5_NO_VPU
	vpush{8dw}	via0
	vpush{8dw}	vib0
	vpush{8dw}	vic0
	vpush{8dw}	vid0
	vpush{8dw}	vie0
	vpush{8dw}	vif0
	vpush{8dw}	vig0
	vpush{8dw}	vih0
	vpush{8dw}	voa0e
	vpush{8dw}	voa0
	vpush{8dw}	vob0e
	vpush{8dw}	vob0
	vpush{8dw}	vc0
	vpush{8dw}	vpr0
	vpush{dw} 	modv0
	vpush{dw} 	modv1
#endif*/

	mov		lci0, g4
	mov		lci1, g5 || bkst
	mov		lci2, g6
	mov		lci3, g7
	mov		lcstep0, modu2
	mov		lcstep1, modu3
	push {8dw}	auxreg1
	nop
	bkst
	push {dw}	bknest0
	push {dw}	bknest1
	bkst
	push {dw}	bknest0
	push {dw}	bknest1
	nop
	nop
	nop

	subsps		#1
	nop
	mov		sp, r0
	nop
	st {dw}		sp, (r0)

	/* Prepare the C language environment */
	mov		#0x0, mod0
	mov		#0x0, mod1
	mov		#0x0, mod2
	mov		modg, a1
	and		a1, #0x300, a1
	or		a1, #0x0000001b, a1
	mov		a1, modg

	/* There are two arguments to up_doirq:
	 *
	 *   a0 = The IRQ number
	 *   r0 = The top of the stack points to the saved state
	 */

	/* Switch to the dedicated stack */

	mov		#_g_intstackbase, r1
	nop
	mov		r1, sp
	nop
	nop

	push {dw}	retreg
	mov		#_up_doirq, r1
	nop
	nop
	callar		r1
	pop {dw}	retreg

	/* On return from up_doirq, r0 will hold a pointer to register context
	 * array to use for the interrupt return.
	 */

	mov		r0, sp
	addsps		#1
	nop
	nop

	/* Unwind the same stack frame that we created at entry */

	pop {dw}	bknest1
	pop {dw}	bknest0
	nop
	nop
	bkrest
	pop {dw}	bknest1
	pop {dw}	bknest0
	nop
	nop
	bkrest
	pop {8dw}	auxreg1
	nop
	nop
	nop
	bkrest
	mov		modu3, lcstep1
	mov		modu2, lcstep0
	mov		g7, lci3
	mov		g6, lci2
	mov		g5, lci1
	mov		g4, lci0

/*#ifndef CONFIG_ARCH_XC5_NO_VPU
	vpop{dw}	modv1
	vpop{dw}	modv0
	vpop{8dw}	vpr0
	vpop{8dw}	vc0
	vpop{8dw}	vob0
	vpop{8dw}	vob0e
	vpop{8dw}	voa0
	vpop{8dw}	voa0e
	vpop{8dw}	vih0
	vpop{8dw}	vig0
	vpop{8dw}	vif0
	vpop{8dw}	vie0
	vpop{8dw}	vid0
	vpop{8dw}	vic0
	vpop{8dw}	vib0
	vpop{8dw}	via0
#endif*/

	pop {16dw,l}	auxreg2
	pop {16dw,l}	auxreg0
	pop {16dw,l}	auxreg1
	nop
	nop
	pop {16dw,h}	auxreg2
	pop {16dw,h}	auxreg0
	pop {4dw}	modx
	pop {dw}	mod3
	pop {16dw,h}	r4, a8
	bkrest
	pop {16dw,h}	auxreg1

	ret

	.func_end	3 exception_common

	.bss
	.public		_g_intstackalloc
	.public		_g_intstackbase
_g_intstackalloc:
	DD		CONFIG_ARCH_INTERRUPTSTACK/4 dup ?
_g_intstackbase:
