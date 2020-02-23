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
#include <arch/arch.h>
#include <arch/multiboot2.h>

    .file    "intel64_head.S"

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

#define STACKBASE    ((_ebss + 0x1f) & 0xffffffe0)
#define IDLE_STACK    (STACKBASE+CONFIG_IDLETHREAD_STACKSIZE)
#define HEAP_BASE    (STACKBASE+CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Trace macros, use like trace 'i' to print char to serial port. */

    .macro    trace, ch
#ifdef CONFIG_DEBUG_FEATURES
    mov        $0x3f8, %dx
    mov        $\ch, %al
    out        %al, %dx
#endif
    .endm

/****************************************************************************
 * Public Symbols
 ****************************************************************************/
    .global     __pmode_entry                   /* The 32bit protected mode entry */
    .global     __nxstart
    .global     nx_start                        /* nx_start is defined elsewhere */
    .global     up_lowsetup                     /* up_lowsetup is defined elsewhere */
    .global     g_idle_topstack                 /* The end of the idle stack, the start of the heap */

    /* These are the page tables */
    .global     pdpt
    .global     pd
    .global     kernel_pd
    .global     full_map_pd1
    .global     pt

/****************************************************************************
 * The multiboot2 header
 ****************************************************************************/

    .set    HEADER_LENGTH, header_end - header_start
    .set    CHECKSUM, -(MULTIBOOT2_HEADER_MAGIC + MULTIBOOT_ARCHITECTURE_I386 + HEADER_LENGTH)

#ifndef CONFIG_ARCH_EXCLUDE_MULTIBOOT
    .section ".multiboot", "a"
    .align    8

header_start:
    .long MULTIBOOT2_HEADER_MAGIC
    .long MULTIBOOT_ARCHITECTURE_I386
    .long HEADER_LENGTH
    .long CHECKSUM

    // multiboot tags go here

    .short MULTIBOOT_HEADER_TAG_END
    .short 0    // flags, none set
    .long 8     // size, including itself (short + short + long)
header_end:

#endif

/****************************************************************************
 * .text
 ****************************************************************************/

    .code32
    .section ".loader.text", "ax"

/****************************************************************************
 * Name: __pmode_entry
 *
 * Description:
 *   Entry point for 32-bit protected mode
 *   Function to transit protected mode to 64-bit long mode
 *
 ****************************************************************************/

    .type   __pmode_entry, @function
__pmode_entry:
start32:

    // initialize rest of the page directory
    lea     pd,     %edi
    lea     pt,     %esi

    // Popluate the lower 4GB as non-present
    // for ecx = 0...512 * 4 : Loop and setup the page directories
    mov     $0x800, %ecx // 512 * 3
epd_loop: 
    mov     %esi,   %edx
    or      $(X86_PAGE_WR | X86_PAGE_PRESENT), %edx
    mov     %edx, 0(%edi)
    add     $(X86_PAGE_ENTRY_SIZE), %edi

    // for ebx = 0...1024: Loop and clear the page table of each page directory
    mov     $1024,  %ebx
ept_loop:
    movl    $0x0, 0(%esi)
    add     $4,     %esi

    // end for ebx
    dec     %ebx
    jnz     ept_loop

    // end for ecx
    dec     %ecx
    jnz     epd_loop

    // Temporary populate the lower 128MB on 1:1 mapping
    lea     pd,     %edi
    mov     $(X86_PAGE_GLOBAL | X86_PAGE_WR | X86_PAGE_PRESENT | X86_PAGE_HUGE), %eax

    // for ecx = 0...64 : Loop and setup 64x 2MB page directories
    mov     $64,   %ecx
pd_loop: 
    mov     %eax, 0(%edi)
    add     $(HUGE_PAGE_SIZE), %eax
    add     $(X86_PAGE_ENTRY_SIZE), %edi

    // end for ecx
    dec     %ecx
    jnz     pd_loop

    // Populate the 128MB after 4GB boundary with Global mapping to 1MB~ for kernel code
    lea     kernel_pd,     %edi
    mov     $(X86_PAGE_GLOBAL | X86_PAGE_WR | X86_PAGE_PRESENT | X86_PAGE_HUGE), %eax

    // for ecx = 0...64 : Loop and setup 128MB, 64x 2MB page directories
    mov     $64,   %ecx
kpd_loop: 
    mov     %eax, 0(%edi)
    add     $(HUGE_PAGE_SIZE), %eax
    add     $(X86_PAGE_ENTRY_SIZE), %edi

    // end for ecx
    dec     %ecx
    jnz     kpd_loop

    // Enable PAE
    mov     %cr4,   %eax
    or      $(X86_CR4_PAE | X86_CR4_PGE),   %eax
    mov     %eax,   %cr4

    // Load the 4 level page table
    // Level 1 and 2 were preset at build time in assembly for this loading 
    // process
    // 4KiB page table is used
    // Kernel mapped to 1GB HiMem
    lea     pml4,   %eax
    mov     %eax,   %cr3

    movl    $MSR_MTRR_DEF_TYPE, %ecx
    rdmsr
    or      $MTRR_ENABLE,   %eax
    wrmsr

    movl    $MSR_EFER,  %ecx
    rdmsr
    or      $EFER_LME,  %eax
    wrmsr

    // Enable paging related bits in CR0
    mov     $(X86_CR0_PG | X86_CR0_WP | X86_CR0_PE),    %eax
    mov     %eax,   %cr0

    // Enable PCID and FGSBASE
    mov     %cr4,   %eax
    or      $X86_CR4_PCIDE,     %eax
    or      $X86_CR4_FGSBASE,   %eax
    mov     %eax,   %cr4


    // Load a GDT with 64bits mode set
    lgdt    gdt64_ptr

    // Long jump into 64 bit mode, updating cs to new GDT
    ljmpl   $(X86_GDT_CODE_SEL),   $start64

    .code64
start64:

    // Set Segement Registers for proper iret, etc. operation
    mov     $(X86_GDT_DATA_SEL),  %ax
    mov     %ax,    %ss
    mov     %ax,    %ds
    mov     %ax,    %es
    mov     %ax,    %fs
    mov     %ax,    %gs

    // Enable SSE
    mov     %cr0,   %rax
    mov     $(X86_CR0_EM), %rbx
    not     %rbx
    and     %rbx,   %rax
    or      $(X86_CR0_MP), %rax 
    mov     %rax,   %cr0

    // Enable Saving XMM context
    mov     %cr4,   %rax
    or      $(X86_CR4_OSXFSR | X86_CR4_XMMEXCPT),   %rax 
    mov     %rax,   %cr4

    // Setup MXCSR, masking all SSE precision exception
    ldmxcsr     mxcsr_mem

    //clear out bss section
    movabs  $_sbss, %rbx
    movabs  $_ebss, %rdx
clear_bss:
    movb    $0,    (%rbx)
    inc     %rbx
    cmp     %rbx,   %rdx
    jne     clear_bss

    // Properly setup RSP to idle stack
    movabs  $idle_stack,    %rbx
    add     $CONFIG_IDLETHREAD_STACKSIZE,   %rbx
    mov     %rbx,   %rsp

    // Finally, we can start the OS
    jmp     __nxstart
    .size   __pmode_entry, . - __pmode_entry

/****************************************************************************
 * Name: __nxstart
 *
 * Description:
 *   Do low-level initialization and call nx_start
 *
 ****************************************************************************/

    .section .text, "a"
    .type   __nxstart, @function

__nxstart:
    /* We are now in high memory, revoke the lower 128MB memory mapping */
    lea     pd,     %edi
    mov     $0,     %eax

    // for ecx = 0...64 : Loop and setup 64x 2MB page directories
    mov     $64,   %ecx
npd_loop: 
    mov     %eax, 0(%edi)
    add     $(HUGE_PAGE_SIZE), %eax
    add     $(X86_PAGE_ENTRY_SIZE), %edi

    // end for ecx
    dec     %ecx
    jnz     npd_loop

    /* Initialize and start NuttX */
    movabs  $up_lowsetup,   %rbx
    call    *%rbx                          /* Low-level, pre-OS initialization */

    movabs  $nx_start,   %rbx
    call    *%rbx                           /* Start NuttX */

    /* NuttX will not return */
    /* We should never end up here */
    /* If we really do, then we are doomed, halting the processor for ever */

    cli
hang:
    hlt                                    /* Halt machine should NuttX return */
    jmp    hang
    .size    __nxstart, . - __nxstart

/****************************************************************************
 * .data
 ****************************************************************************/

    .section ".loader.data", "ax"

    // IST for 64 bit long mode
    // will be filled in up_irq
    .align(16)
    .global ist64
ist64:
    .long    0
    .quad    0xdeadbeefdeadbee0
    .quad    0xdeadbeefdeadbee1
    .quad    0xdeadbeefdeadbee2
    .quad    0
    .quad    0
    .quad    0
    .quad    0
    .quad    0
    .quad    0
    .quad    0
    .quad    0
    .quad    0
    .word    0

    // GDT for 64 bit long mode
    .align(16)
    .global gdt64
    .global gdt64_ist
gdt64:
    .quad   0
    .quad   X86_GDT_CODE64_ENTRY
    .quad   X86_GDT_DATA_ENTRY
    .quad   X86_GDT_CODE32_ENTRY
    .quad   X86_GDT_DATA_ENTRY
    .quad   X86_GDT_CODE64_ENTRY
gdt64_ist:
    .quad   0x0 // TSS segment low
    .quad   0x0 // TSS segment high

gdt64_ptr:
    .short  gdt64_ptr - gdt64 - 1
    .long   gdt64

mxcsr_mem:
    .long   0x00001f80

    .align(PAGE_SIZE)
pml4:
    .quad    pdpt + X86_PAGE_PRESENT + X86_PAGE_WR

    .align(PAGE_SIZE)
pdpt:
    .quad    pd   + X86_PAGE_PRESENT + X86_PAGE_WR
    .quad    pd_2 + X86_PAGE_PRESENT + X86_PAGE_WR
    .quad    pd_3 + X86_PAGE_PRESENT + X86_PAGE_WR
    .quad    pd_4 + X86_PAGE_PRESENT + X86_PAGE_WR
    .quad    kernel_pd + X86_PAGE_PRESENT + X86_PAGE_WR

    .align(PAGE_SIZE)
pd:
    .fill X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0

    .align(PAGE_SIZE)
pd_2:
    .fill X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0

    .align(PAGE_SIZE)
pd_3:
    .fill X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0

    .align(PAGE_SIZE)
pd_4:
    .fill X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0

    .align(PAGE_SIZE)
kernel_pd:
    .fill X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0

    .align(PAGE_SIZE)
pt:
    .fill X86_NUM_PAGE_ENTRY * X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0
    .fill X86_NUM_PAGE_ENTRY * X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0
    .fill X86_NUM_PAGE_ENTRY * X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0
    .fill X86_NUM_PAGE_ENTRY * X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0

/****************************************************************************
 * .bss
 ****************************************************************************/

/* The stack for the IDLE task thread is declared in .bss.  NuttX boots and
 * initializes on the IDLE thread, then at the completion of OS startup, this
 * thread becomes the thread that executes when there is nothing else to
 * do in the system (see up_idle()).
 */

    .section    .bss, "a"

    .type    idle_stack, @object
    .comm    idle_stack, CONFIG_IDLETHREAD_STACKSIZE, 32
    .size    idle_stack, CONFIG_IDLETHREAD_STACKSIZE

/****************************************************************************
 * .rodata
 ****************************************************************************/

    .section    .rodata, "a"

/* HEAP BASE: _sbss is the start of the BSS region (see ld.script) _ebss is
 * the end of the BSS region (see liker script). The heap continues from there
 * until the end of memory.
 */

    .type    g_idle_topstack, @object
g_idle_topstack:
    .quad    _ebss
    .size    g_idle_topstack, . - g_idle_topstack
    .end
