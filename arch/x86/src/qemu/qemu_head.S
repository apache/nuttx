/****************************************************************************
 *  arch/x86/src/qemu/qemu_head.S
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

/****************************************************************************
 * .text
 ****************************************************************************/

#ifdef CONFIG_X86_NASM
global		__start						/* Making entry point visible to linker */
extern		os_start					/* os_start is defined elsewhere */
extern		up_lowsetup					/* up_lowsetup is defined elsewhere */
 
/* Setting up the Multiboot header - see GRUB docs for details */

MODULEALIGN	equ	1<<0					/* Align loaded modules on page boundaries */
MEMINFO		equ	1<<1					/* Provide memory map */
FLAGS		equ	MODULEALIGN | MEMINFO	/* This is the Multiboot 'flag' field */
MAGIC		equ	0x1BADB002				/* 'magic number' lets bootloader find the header */
CHECKSUM	equ -(MAGIC + FLAGS)		/* Checksum required */
 
section .text
align 4
MultiBootHeader:
	dd		MAGIC
	dd		FLAGS
	dd		CHECKSUM
 
/* Reserve initial kernel stack space */

STACKSIZE	equ	0x4000					/* That's 16k */
 
__start:
	mov		esp, stack+STACKSIZE		/* Set up the stack */
	push	eax							/* Pass Multiboot magic number */
	push	ebx							/* Pass Multiboot info structure */
 
	call	up_lowsetup					/* Low-level, pre-OS initialization */
	call	os_start					/* Start NuttX */
 
	cli
hang:
	hlt									/* Halt machine should NuttX return */
	jmp		hang
 
section .bss
align 4
stack:
	resb STACKSIZE						/* Reserve 16k stack on a doubleword boundary */

#else /* !CONFIG_X86_NASM (GAS) */

	.global	__start						/* Making entry point visible to linker */
	.global	_os_start					/* os_start is defined elsewhere */
	.global	_up_lowsetup				/* up_lowsetup is defined elsewhere */

/* Setting up the Multiboot header - see GRUB docs for details */

	.set	ALIGN, 1<<0					/* Align loaded modules on page boundaries */
	.set	MEMINFO, 1<<1				/* Provide memory map */
	.set	FLAGS, ALIGN | MEMINFO		/* This is the Multiboot 'flag' field */
	.set	MAGIC, 0x1BADB002			/* 'magic number' lets bootloader find the header */
	.set	CHECKSUM, -(MAGIC + FLAGS)	/* Checksum required */

	.align	4
	.long	MAGIC
	.long	FLAGS
	.long	CHECKSUM

/* Reserve initial kernel stack space */

	.set	STACKSIZE, 0x4000			/* That is, 16k */
	.comm	stack, STACKSIZE, 32		/* Reserve 16k stack on a quadword boundary */

__start:
	mov		$(stack + STACKSIZE), %esp	/* Set up the stack */
	push	%eax						/* Multiboot magic number */
	push	%ebx						/* Multiboot data structure */

	call	_up_lowsetup				/* Low-level, pre-OS initialization */
	call	_os_start					/* Start NuttX */

	cli
hang:
	hlt									/* Halt machine should NuttX return */
	jmp	hang
	.end
#endif /* CONFIG_X86_NASM */