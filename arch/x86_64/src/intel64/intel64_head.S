/****************************************************************************
 *  arch/x86/src/intel64/intel64_head.S
 *
 *   Copyright (C) 2011 Gregory Nutt,
 *                 2020 Chung-Fan Yang.
 *   All rights reserved.
 *
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Chung-Fan Yang <sonic.tp.tp@gmail.com>
 *
 *   Part of the code is created referencing the Jailhouse inmate library
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

	.file	"intel64_head.S"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Memory Map: _sbss is the start of the BSS region (see ld.script) _ebss is
 * the end of the BSS regsion (see ld.script). The idle task stack starts at
 * the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread
 * is the thread that the system boots on and, eventually, becomes the idle,
 * do nothing task that runs only when there is nothing else to run.  The
 * heap continues from there until the end of memory.  See g_idle_topstack below.
 */

#define STACKBASE	((_ebss + 0x1f) & 0xffffffe0)
#define IDLE_STACK	(STACKBASE+CONFIG_IDLETHREAD_STACKSIZE)
#define HEAP_BASE	(STACKBASE+CONFIG_IDLETHREAD_STACKSIZE)

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
	.globl  __start
	.globl  __pmode_entry
	.global	__nxstart						/* Making entry point visible to linker */
	.global	os_start					/* os_start is defined elsewhere */
	.global	up_lowsetup					/* up_lowsetup is defined elsewhere */
	.global	g_idle_topstack				/* The start of the heap */
    .global pdpt
    .global pd
    .global full_map_pd1
    .global pt

/****************************************************************************
 * .text
 ****************************************************************************/

/****************************************************************************
 * Name: Start
 ****************************************************************************/

#define X86_CR0_PE		0x00000001
#define X86_CR0_WP		0x00010000
#define X86_CR0_PG		0x80000000

#define X86_CR4_PAE      0x00000020
#define X86_CR4_PGE      0x00000080
#define X86_CR4_OSXFSR   0x00000200
#define X86_CR4_XMMEXCPT 0x00000400
#define X86_CR4_FGSBASE  0x00010000
#define X86_CR4_PCIDE    0x00020000

#define MSR_EFER		0xc0000080
#define EFER_LME		0x00000100

#define MSR_MTRR_DEF_TYPE	0x000002ff
#define MTRR_ENABLE		0x00000800

    .set    ALIGN, 1<<0/* Align loaded modules on page boundaries */
    .set    MEMINFO, 1<<1/* Provide memory map */
    .set    FLAGS, ALIGN | MEMINFO/* This is the Multiboot 'flag' field */
    .set    MAGIC, 0x1badb002/* 'magic number' lets bootloader find the header */
    .set    CHECKSUM, -(MAGIC + FLAGS)/* Checksum required */

	.code16
	.section ".loader", "ax"

#ifndef CONFIG_ARCH_EXCLUDE_MULTIBOOT
	.align	4
	.long	MAGIC
	.long	FLAGS
	.long	CHECKSUM
#endif

    .type   __start, @function
__start:
    // Load a GDT for protected mode
    movl $loader_gdt_ptr, %ebx
	lgdtl (%ebx)

    // enable protected mode in CR0
	mov %cr0,%eax
	or $X86_CR0_PE,%al
	mov %eax,%cr0

    // Long jump into protected mode
    // Hardcode the address
	ljmpl $0x8,$0x3c000000

    // Loader GDT and GDTR
	.align(16)
	.global loader_gdt
loader_gdt:
	.quad	0
	.quad	0x00cf9a000000ffff
	.quad	0x00cf92000000ffff

loader_gdt_ptr:
	.short	loader_gdt_ptr - loader_gdt - 1
	.long	loader_gdt

    .size	__start, . - __start

	.code32

    .type   __pmode_entry, @function
__pmode_entry:
start32:
    mov $0x10, %ax
    mov %ax, %ss
    mov %ax, %ds

    // initialize rest of the page directory
    // Popluate the whole lower 1GB on 1:1 mapping

    mov $0x00000103, %eax

    lea pd, %edi
    lea pt, %esi
    mov $512, %ecx
pd_loop: 

    mov %esi, %edx
    or $0x00000003, %edx
    mov %edx, 0(%edi)
    add $8, %edi

    mov $512, %ebx
pt_loop:
    mov %eax, 0(%esi)
    add $0x00001000, %eax
    add $8, %esi

    dec %ebx
    jnz pt_loop

    dec %ecx
    jnz pd_loop

    // Popluate the upper 3GB as non-present
    mov $0x600, %ecx // 512 * 3
epd_loop: 

    mov %esi, %edx
    or $0x00000003, %edx
    mov %edx, 0(%edi)
    add $8, %edi

    mov $1024, %ebx
ept_loop:
    movl $0x0, 0(%esi)
    add $4, %esi

    dec %ebx
    jnz ept_loop

    dec %ecx
    jnz epd_loop

    // Enable PAE
	mov %cr4,%eax
	or $(X86_CR4_PAE | X86_CR4_PGE),%eax
	mov %eax,%cr4

    // Load the level 4 page table
    // Level 1, 2 and 3 are set at build time in assembelly
    // 4KiB page table is used
    // Kernel mapped to 1GB HiMem
	mov $pml4,%eax
	mov %eax,%cr3

	movl $MSR_MTRR_DEF_TYPE,%ecx
	rdmsr
	or $MTRR_ENABLE,%eax
	wrmsr

	movl $MSR_EFER,%ecx
	rdmsr
	or $EFER_LME,%eax
	wrmsr

    // Enable paging related bits in CR0
	mov $(X86_CR0_PG | X86_CR0_WP | X86_CR0_PE),%eax
	mov %eax,%cr0

    // Enable PCID and FGSBASE
	mov %cr4,%eax
	or $X86_CR4_PCIDE,%eax
	or $X86_CR4_FGSBASE,%eax
	mov %eax,%cr4

    // Load a GDT with 64bits mode set
    lgdt gdt64_ptr

    // Long jump into 64 bit mode, updating cs to new GDT
	ljmpl $0x8,$start64


    // IST for 64 bit long mode
    // will be filled in up_irq
	.align(16)
	.global ist64
ist64:
	.long	0
	.quad	0xdeadbeefdeadbee0
	.quad	0xdeadbeefdeadbee1
	.quad	0xdeadbeefdeadbee2
	.quad	0
	.quad	0
	.quad	0
	.quad	0
	.quad	0
	.quad	0
	.quad	0
	.quad	0
	.quad	0
	.word	0

    // GDT for 64 bit long mode
	.align(16)
	.global gdt64
	.global gdt64_ist
gdt64:
	.quad	0
    .quad   0x002f98000000ffff
    .quad   0x000f92000000ffff
    .quad   0x000f98000000ffff
    .quad   0x000f92000000ffff
    .quad   0x002f98000000ffff
gdt64_ist:
    .quad   0x0 // TSS segment low
    .quad   0x0 // TSS segment high

gdt64_ptr:
	.short	gdt64_ptr - gdt64 - 1
	.long	gdt64

mxcsr_mem:
    .long   0x00001f80

    .code64
start64:

    // Set Segement Registers for proper iret, etc. operation
    mov $0x10, %ax
    mov %ax, %ss
    mov %ax, %ds
    mov %ax, %es
    mov %ax, %fs
    mov %ax, %gs

    // Enable SSE
    mov %cr0, %rax
    and $0xFFFB, %ax
    or $0x2, %rax 
    mov %rax, %cr0
    mov %cr4, %rax
    mov $(X86_CR4_OSXFSR | X86_CR4_XMMEXCPT), %rbx
    or %rbx, %rax 
    mov %rax, %cr4
    // Mask all SSE precision exception
    // Setup MXCSR
    ldmxcsr mxcsr_mem

    //clear out bss section
    mov $_sbss, %rbx
    mov $_ebss, %rdx
clear_bss:
    movb $0, (%rbx)
    inc %rbx
    cmp %rbx, %rdx
    jne clear_bss

    // Properly setup RSP to idle stack
    mov $idle_stack, %rbx
    add $CONFIG_IDLETHREAD_STACKSIZE, %rbx
    mov %rbx, %rsp

    // Finally, we can start the OS
    jmp __nxstart
	.size	__pmode_entry, . - __pmode_entry

    .type   __nxstart, @function

__nxstart:
	/* Initialize and start NuttX */
	call	up_lowsetup					/* Low-level, pre-OS initialization */

	call	nx_start					/* Start NuttX */

	/* NuttX will not return */
    /* We should never end up here */
    /* If we really do, then we are doomed, halting the processor for ever */

	cli
hang:
	hlt									/* Halt machine should NuttX return */
	jmp	hang
	.size	__nxstart, . - __nxstart

	.pushsection ".data"
	.align(4096)
pml4:
	.quad	pdpt + 0x003

	.align(4096)
pdpt:
	.quad	pd + 0x003
	.quad	pd_2 + 0x003
	.quad	pd_3 + 0x003
	.quad	pd_4 + 0x003

	.align(4096)
pd:
	.fill 64, 8, 0x83
	.fill 64, 8, 0x80
	.fill 128, 8, 0x80
	.fill 256, 8, 0x80

	.align(4096)
pd_2:
	.fill 512, 8, 0x80

pd_3:
	.fill 512, 8, 0x80

	.align(4096)
pd_4:
	.fill 512, 8, 0x80

	.align(4096)
pt:
	.fill 0x40000, 8, 0x00
	.fill 0x40000, 8, 0x00
	.fill 0x40000, 8, 0x00
	.fill 0x40000, 8, 0x00

	.align(4096)
full_map_pd1:
	.fill 512, 8, 0x80

/****************************************************************************
 * .bss
 ****************************************************************************/

/* The stack for the IDLE task thread is declared in .bss.  NuttX boots and
 * initializes on the IDLE thread, then at the completion of OS startup, this
 * thread becomes the thread that executes when there is nothing else to
 * do in the system (see up_idle()).
 */

	.type	idle_stack, @object
	.comm   idle_stack, CONFIG_IDLETHREAD_STACKSIZE, 32
	.size	idle_stack, CONFIG_IDLETHREAD_STACKSIZE

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
	.quad	_ebss
	.size	g_idle_topstack, . - g_idle_topstack
	.end
