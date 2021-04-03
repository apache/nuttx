/****************************************************************************
 * arch/x86/src/i486/i486_utils.S
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

/*   Based on Bran's kernel development tutorials. Rewritten for JamesM's
 *   kernel development tutorials.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

	.file	"i486_utils.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KSEG 0x10

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	gdt_flush
	.globl	idt_flush

/****************************************************************************
 * .text
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: gdt_flush
 ****************************************************************************/

	.type	gdt_flush, @function
gdt_flush:
	movl	%eax, 4(%esp)	/* Get the pointer to the GDT, passed as a parameter */
	lgdt	(%eax)			/* Load the new GDT pointer */

	mov		$KSEG, %ax 		/* KSEG is the offset in the GDT to our data segment */
	mov		%ax, %ds 		/* Load all data segment selectors */
	mov		%ax, %es
	mov		%ax, %fs
	mov		%ax, %gs
	mov		%ax, %ss
	jmp		$0x08, $.Lgflush	/* 0x08 is the offset to our code segment: Far jump! */
.Lgflush:
	ret
	.size	gdt_flush, . - gdt_flush

/****************************************************************************
 * Name: idt_flush
 ****************************************************************************/

	.type	idt_flush, @function
idt_flush:
	movl	%eax, 4(%esp)	/* Get the pointer to the IDT, passed as a parameter */
	lidt	(%eax)			/* Load the IDT pointer */
	ret
	.size	idt_flush, . - idt_flush
	.end
