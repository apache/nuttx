/****************************************************************************
 *  arch/x86/src/qemu/qemu_head.S
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

	.file	"qemu_head.S"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Memory Map: _sbss is the start of the BSS region (see ld.script) _ebss is
 * the end of the BSS region (see ld.script). The idle task stack starts at
 * the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread
 * is the thread that the system boots on and, eventually, becomes the idle,
 * do nothing task that runs only when there is nothing else to run.  The
 * heap continues from there until the end of memory.  See g_idle_topstack below.
 */

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Trace macros, use like trace 'i' to print char to serial port. */

	.macro	trace, ch
#ifdef CONFIG_DEBUG_FEATURES
	mov		$0x3f8, %dx
	mov		$\ch, %al
	out		%al, %dx
#endif
	.endm

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.global	__start						/* Making entry point visible to linker */
	.global	nx_start					/* nx_start is defined elsewhere */
	.global	up_lowsetup					/* up_lowsetup is defined elsewhere */
	.global	g_idle_topstack				/* The start of the heap */

/****************************************************************************
 * .text
 ****************************************************************************/
/****************************************************************************
 * Multiboot Header
 ****************************************************************************/

	/* Setting up the Multiboot header - see GRUB docs for details */

	.set	ALIGN, 1<<0					/* Align loaded modules on page boundaries */
	.set	MEMINFO, 1<<1				/* Provide memory map */
	.set	FLAGS, ALIGN | MEMINFO		/* This is the Multiboot 'flag' field */
	.set	MAGIC, 0x1badb002			/* 'magic number' lets bootloader find the header */
	.set	CHECKSUM, -(MAGIC + FLAGS)	/* Checksum required */

	.text
	.align	4
	.long	MAGIC
	.long	FLAGS
	.long	CHECKSUM

/****************************************************************************
 * Name: Start
 ****************************************************************************/

	.type	__start, @function
__start:
	/* Set up the stack */

	mov		$(_ebss + CONFIG_IDLETHREAD_STACKSIZE), %esp

	/* Multiboot setup */

	push	%eax						/* Multiboot magic number */
	push	%ebx						/* Multiboot data structure */

	/* Initialize and start NuttX */

	call	up_lowsetup					/* Low-level, pre-OS initialization */
	call	nx_start					/* Start NuttX */

	/* NuttX will not return */

	cli
hang:
	hlt									/* Halt machine should NuttX return */
	jmp	hang
	.size	__start, . - __start

/****************************************************************************
 * .rodata
 ****************************************************************************/

	.section	.rodata, "a"

/* HEAP BASE: _sbss is the start of the BSS region (see ld.script) _ebss is
 * the end of the BSS region (see ld.script). The heap continues from there
 * until the end of memory.
 */

	.type	g_idle_topstack, @object
g_idle_topstack:
	.long	_ebss + CONFIG_IDLETHREAD_STACKSIZE
	.size	g_idle_topstack, . - g_idle_topstack
	.end
