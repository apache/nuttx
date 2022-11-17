/****************************************************************************
 * arch/avr32/src/avr32/avr_nommuhead.S
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
#include <arch/avr32/avr32.h>

#include "avr_internal.h"

/****************************************************************************
 * External Symbols
 ****************************************************************************/

	.file		"up_nommuhead.S"
	.global		_sbss			/* Start of .bss.  Defined by ld.script */
	.global		_ebss			/* End of .bss.  Defined by ld.script */
#ifdef CONFIG_BOOT_RUNFROMFLASH
	.global		_sdata			/* Start of .data section in RAM */
	.global		_edata			/* End of .data section in RAM */
	.global		_eronly			/* Start of .data section in FLASH */
#endif
	.global		avr_lowinit		/* Perform low level initialization */
	.global		nx_start		/* NuttX entry point */
	.global		vectortab		/* Vector base address */

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * OS Entry Point
 ****************************************************************************/

/* The following entry point must be linked to execute at 0x80000000 if it
 * is to execute upon reset.
 */
	.text
	.global 	__start
	.type		__start, @function
__start:

	/* Set the IDLE thread stack pointer.  This stack will be used
	 * through NuttX initialization and will, eventually, be inherited
	 * by the IDLE thread when threading is enabled.
	 */

	lddpc		sp, .Lstackbase

	/* Set up the vector base address so interrupts can be enabled. */

	lda.w		r0, vectortab
	mtsr		AVR32_EVBA, r0

	/* Enable exception processing */

	csrf		AVR32_SR_EM_SHIFT

	/* Clear system BSS section */

	lda.w		r0, _sbss		/* R =Start of .bss */
	lda.w		r1, _ebss		/* r1=End of .bss */
	mov			r2, 0			/* Value to write to .bss */
	rjmp		2f				/* Start at the bottom of the loop */
1:
	st.d		r0++, r2		/* Zero .bss */
2:
	cp			r0, r1			/* Finished? */
	brlo		1b				/* No... keep looping */

	/* Copy system .data sections to new home in RAM. */

#ifdef CONFIG_BOOT_RUNFROMFLASH

	lda.w		r0, _sdata		/* r0=Start of .data section in RAM */
	lda.w		r1, _edata		/* r1=End of .data section in RAM */
	lda.w 		r2, _eronly		/* r2=Start of .data in FLASH */
	rjmp		4f				/* Start at the bottom of the loop */
3:
	ld.d		r4, r2++		/* Fetch the next data value */
	st.d		r0++, r4		/* Write it to the .data section */
4:
	cp			r0, r1			/* Finished? */
	brlo		3b				/* No... keep looping */

#endif

	/* Clear the frame pointer and link register since this is the outermost
	 * frame.
	 */

	mov			r7, 0
	mov			lr, 0

	/* Perform low-level initialization */

	mcall		.Lavr_lowinit

	/* Then jump to OS entry (will not return) */

	lda.w		pc, nx_start

.Lstackbase:
	.word	_ebss+CONFIG_IDLETHREAD_STACKSIZE
.Lavr_lowinit:
	.word	avr_lowinit
	.size	__start, .-__start

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to other
	 * uses of _ebss in this file
	 */

	.data
	.align	4
	.globl	g_idle_topstack
	.type	g_idle_topstack, object
g_idle_topstack:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_idle_topstack, .-g_idle_topstack
	.end
