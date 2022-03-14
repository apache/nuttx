/********************************************************************
 * arch/arm/src/dm320/dm320_restart.S
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
 ********************************************************************/

/********************************************************************
 * Included Files
 ********************************************************************/

#include <nuttx/config.h>

#include "arm.h"

/********************************************************************
 * Pre-processor Definitions
 ********************************************************************/

/********************************************************************
 * Assembly Macros
 ********************************************************************/

/* Since the page table is closely related to the NuttX base
 * address, we can convert the page table base address to the
 * base address of the section containing both.
 */

	.macro	mksection, section, pgtable
	bic	\section, \pgtable, #0x000ff000
	.endm

/**************************************************************************
 * Name: up_restart
 **************************************************************************/

	.text
	.globl	up_restart
	.type	up_restart, %function
up_restart:
	/* Make sure that we are in SYS mode with all IRQs disabled */

	mov	r0, #(PSR_MODE_SYS | PSR_I_BIT | PSR_F_BIT)
	msr	cpsr_c, r0

	/* Create identity mapping for first MB section to support
	 * this re-start logic executing out of the physical address
	 * space.
	 */

	mksection r0, r4		/* r0=phys. base section */
	ldr	r1, .LCmmuflags		/* FLGS=MMU_MEMFLAGS */
	add	r3, r1, r0		/* r3=flags + base */
	str	r3, [r4, r0, lsr #18]	/* identity mapping */

	/* Jump into the physical address space */

	ldr	pc, .LCphysrestart
	nop
	nop

	/* We are now executing at our physical address, with the
	 * MMU disabled.
	 */

up_phyrestart:

	mov	r0, #0
	mcr	p15, 0, r0, c7, c7		/* Invalidate I,D caches */
	mcr	p15, 0, r0, c7, c10, 4		/* Drain write buffer */
	mcr	p15, 0, r0, c8, c7		/* Invalidate I,D TLBs */

	/* Clear bits in control register (see start.h): Disable,
	 * MMU, Data cache, alignment traps, write buffer, Instruction
	 * cache, exceptions at 0xffff0000, round robin)
	 */

	mrc	p15, 0, r0, c1, c0		/* Get control register */
	bic	r0, r0, #(CR_M|CR_C|CR_A|CR_W)
	bic	r0, r0, #(CR_S|CR_I|CR_V|CR_RR)
	mcr	p15, 0, r0, c1, c0, 0		/* Write control reg */

	/* We know that the bootloader entry point is at the
	 * beginning of flash.
	 */
#if 1
	ldr	pc, .LCbtldrentry		/* Restart bootloader */
#else
	b	__start				/* Restart NuttX */
#endif

	.type	.LCphysrestart, %object
.LCphysrestart:
	.long	(up_phyrestart - CONFIG_RAM_VSTART - CONFIG_RAM_START)
.LCbtldrentry:
	.long	DM320_EXT_MEM_PADDR

/**************************************************************************
 * PC_Relative Data
 **************************************************************************/

	.type	.LCmmuflags, %object
.LCmmuflags:
	.long	MMU_MEMFLAGS
	.size	up_restart, .-up_restart

	.end
