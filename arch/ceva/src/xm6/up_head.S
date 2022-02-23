/****************************************************************************
 * arch/ceva/src/xm6/up_head.S
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

.IF CONFIG_ARCH_XM6_BUG001
	.EQU		prx pr14
.ELSE
	.EQU		prx pr15
.ENDIF

.MACRO	IRQ_HANDLER	irq retx
	push		{auxreg10}
	push		{auxreg2}
	mov		#irq, r0.ui
	callr		#exception_common, ?prx.b
	pop		{auxreg10}
	nop		#0x04
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

/****************************************************************************
 * Interrupt Functions
 ****************************************************************************/

	.CSECT		inttbl
.IF CONFIG_ARCH_XM6_BUG001
	cmp {eq}	r0.ui, r0.ui, prx.b2, pr15.b2
.ENDIF
	br		#reset_handler, #0x00, #0x00, ?prx.b

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

	.ORG		0x1c0
	.GLOBAL		_up_vintc_handler
_up_vintc_handler:
	IRQ_HANDLER	IRQ_VINT, reti

	.ORG		0x200
	IRQ_HANDLER	IRQ_TRAP0, reti

	.ORG		0x240
	IRQ_HANDLER	IRQ_TRAP1, reti

	.ORG		0x280
	IRQ_HANDLER	IRQ_TRAP2, reti

	.ORG		0x2c0
	IRQ_HANDLER	IRQ_TRAP3, reti

reset_handler:
	/* Tricky for compiler bug: when codes are placed onto ddr memory, label
	* address will be place at the back of its first code line, so when branch
	* to this label, its first code line will be skipped.
	*/
	nop
	nop
	nop
	nop

	/* Initialize the C language environment */
	ld		(#_g_idle_topstack).ui, r0.ui
	mov		r0.ui, sp.ui

	br		#_up_start, #0x00, #0x00, ?prx.b

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Common exception handling logic, need sync with:
 * arch/ceva/include/xm6/reg.h
 */

	.func_start	3 exception_common

exception_common:
	/* Tricky for compiler bug: when codes are placed onto ddr memory, label address
	* will be placed at the back of its first code line, so when branch to
	* to this label, its first code line will be skipped.
	*/
	nop
	nop
	nop
	nop

	/* Note: r0 contain exception number
	 * Complete the context save
	 */

	push		{auxreg3}
	push		{auxreg4}
	push		{auxreg5}
	push		{auxreg6}
	push		{auxreg7}
	push		{auxreg8}
	push		{auxreg9}

	push		moda.ui
	push		modc.ui
	push		modd.ui
	push		mode.ui
	push		modg.ui
	push		modi.ui

	modr		(sp.ui).ui +#-4
	mov		sp.ui, r1.ui
	st		r1.ui, (r1.ui).ui

	/* Prepare the C language environment */

	lbf {sv}	#0x00

	/* There are two arguments to up_doirq:
	 *
	 *   r0 = The IRQ number
	 *   r1 = The top of the stack points to the saved state
	 */

	/* Switch to the dedicated stack */

	mov		#_g_intstackbase, r2.ui
	mov		r2.ui, sp.ui

	push		retreg.ui
	callr		#_up_doirq, ?prx.b
	pop		retreg.ui

	/* On return from up_doirq, r0 will hold a pointer to register context
	 * array to use for the interrupt return.
	 */

	modr		(r0.ui).ui +#4

	/* Restore the stack pointer */

	mov		r0.ui, sp.ui

	/* Unwind the same stack frame that we created at entry */

	pop		modi.ui
	pop		modg.ui
	pop		mode.ui
	pop		modd.ui
	pop		modc.ui
	pop		moda.ui

	pop		{auxreg9}
	pop		{auxreg8}
	pop		{auxreg7}
	pop		{auxreg6}
	pop		{auxreg5}
	pop		{auxreg4}
	pop		{auxreg3}
	pop		{auxreg2}

	ret		?prx.b

	.func_end	3 exception_common

	.bss
	.public		_g_intstackalloc
	.public		_g_intstackbase
_g_intstackalloc:
	DD		CONFIG_ARCH_INTERRUPTSTACK/4 dup ?
_g_intstackbase:
