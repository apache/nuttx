/****************************************************************************
 *  arch/x86/src/intel64/intel64_head.S
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
#include <arch/arch.h>
#include <arch/multiboot2.h>

	.file "intel64_head.S"

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

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.global    __pmode_entry                   /* The 32bit protected mode entry */
	.global    __enable_sse_avx
	.global    __enable_pcid
	.global    __revoke_low_memory
	.global    __nxstart                       /* __nxstart is defined elsewhere */
	.global    nx_start                        /* nx_start is defined elsewhere */
	.global    g_idle_topstack                 /* The end of the idle stack, the start of the heap */
	.global    g_mb_info_struct
	.global    g_mb_magic

	/* These are the page tables */
	.global    g_pdpt_low
	.global    g_pd_low
	.global    g_pt_low

	/* These are the GDT */
	.global    g_gdt64_low
	.global    g_gdt64_ist_low
	.global    g_gdt64_low_end

	.global    g_ist64_low

/****************************************************************************
 * The multiboot2 header
 ****************************************************************************/

	.set    HEADER_LENGTH, header_end - header_start
	.set    CHECKSUM, -(MULTIBOOT2_HEADER_MAGIC + MULTIBOOT_ARCHITECTURE_I386 + HEADER_LENGTH)

	.section ".multiboot", "a"
	.align  8

header_start:
#ifdef CONFIG_ARCH_MULTIBOOT2
	.long   MULTIBOOT2_HEADER_MAGIC
	.long   MULTIBOOT_ARCHITECTURE_I386
	.long   HEADER_LENGTH
	.long   CHECKSUM

	/* multiboot tags go here */

	.short  MULTIBOOT_HEADER_TAG_INFORMATION_REQUEST
	.short  0    /* flags, none set */
	.long   24   /* size, including itself (short + short + long) */
	.long   MULTIBOOT_TAG_TYPE_EFI64
	.long   MULTIBOOT_TAG_TYPE_FRAMEBUFFER
	.long   MULTIBOOT_TAG_TYPE_ACPI_OLD
	.long   MULTIBOOT_TAG_TYPE_ACPI_NEW

	.short  MULTIBOOT_HEADER_TAG_END
	.short  0    /* flags, none set */
	.long   8    /* size, including itself (short + short + long) */
#endif
header_end:

	.code16
	.section ".realmode", "ax"

/****************************************************************************
 * Name: __reset_entry
 *
 * Description:
 *   Real mode entry point.
 *
 ****************************************************************************/

	.type   __reset_entry, @function

__reset_entry:
	/* Load a GDT for protected mode */
	movl    $loader_gdt_ptr, %ebx
	lgdtl   (%ebx)

	/* enable protected mode in CR0 */
	mov     %cr0, %eax
	or      $X86_CR0_PE, %al
	mov     %eax, %cr0

	/* Long jump into protected mode. Hardcode the address. */
	ljmpl   $0x8, $0x100000

	/* Loader GDT and GDTR */
	.align(16)
	.global loader_gdt
loader_gdt:
	.quad   0
	.quad   0x00cf9a000000ffff
	.quad   0x00cf92000000ffff
loader_gdt_ptr:
	.short	loader_gdt_ptr - loader_gdt - 1
	.long   loader_gdt

	.size   __reset_entry, . - __reset_entry

/****************************************************************************
 * .text
 ****************************************************************************/

	.code32
	.section ".loader.text", "ax"

start32_0:
	mov     $0x10, %ax
	mov     %ax, %ss
	mov     %ax, %ds

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
#ifdef CONFIG_ARCH_MULTIBOOT2
	movl    %ebx, g_mb_info_struct
	movl    %eax, g_mb_magic
#endif

	/* initialize rest of the page directory */
	lea     g_pd_low, %edi
	lea     g_pt_low, %esi

	/* Popluate the lower 4GB as non-present
	 * for ecx = 0...512 * 4 : Loop and setup the page directories
	 */
	mov     $0x800, %ecx /* 512 * 4 */
epd_loop:
	mov     %esi,   %edx
	or      $(X86_PAGE_WR | X86_PAGE_PRESENT), %edx
	mov     %edx, 0(%edi)
	add     $(X86_PAGE_ENTRY_SIZE), %edi

	/* for ebx = 0...1024: Loop and clear the page table of each page directory */
	mov     $1024,  %ebx
ept_loop:
	movl    $0x0, 0(%esi)
	add     $4,     %esi

	/* end for ebx */
	dec     %ebx
	jnz     ept_loop

	/* end for ecx */
	dec     %ecx
	jnz     epd_loop

	/* Temporary populate the lower 128MB on 1:1 mapping */
	lea     g_pd_low, %edi
	mov     $(X86_PAGE_GLOBAL | X86_PAGE_WR | X86_PAGE_PRESENT | X86_PAGE_HUGE), %eax

	/* for ecx = 0...64 : Loop and setup 64x 2MB page directories */
	mov     $64,    %ecx
pd_loop:
	mov     %eax, 0(%edi)
	add     $(HUGE_PAGE_SIZE), %eax
	add     $(X86_PAGE_ENTRY_SIZE), %edi

	/* end for ecx */
	dec     %ecx
	jnz     pd_loop

	/* Populate the 1GB after 4GB boundary with Global mapping to kernel code.
	 * This creates maps the lower 1GB to 4GB~5GB
	 */
	lea     g_pdpt_low, %edi
	mov     $(X86_PAGE_GLOBAL | X86_PAGE_WR | X86_PAGE_PRESENT | X86_PAGE_HUGE), %eax

	mov     $0x4,   %ecx
	mov     %eax,   0(%edi, %ecx, X86_PAGE_ENTRY_SIZE)

	/* Enable PAE */
	mov     %cr4,   %eax
	or      $(X86_CR4_PAE | X86_CR4_PGE),   %eax
	mov     %eax,   %cr4

	/* Load the 4 level page table.
	 * Level 1 and 2 were preset at build time in assembly for this loading process.
	 * 4KiB page table is used.
	 * Kernel mapped to 1GB HiMem
	 */
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

	/* Enable paging related bits in CR0 */
	mov     $(X86_CR0_PG | X86_CR0_WP | X86_CR0_PE),    %eax
	mov     %eax,   %cr0

	/* Enable FGSBASE */
	mov     %cr4,   %eax
	or      $X86_CR4_FGSBASE,   %eax
	mov     %eax,   %cr4

	/* Load a GDT with 64bits mode set */
	lgdt    gdt64_ptr

	/* Long jump into 64 bit mode, updating cs to new GDT */
	ljmpl   $(X86_GDT_CODE_SEL),   $start64

	.code64
start64:
    /* Set Segement Registers for proper iret, etc. operation */
    mov     $(X86_GDT_DATA_SEL),  %ax
    mov     %ax,    %ss
    mov     %ax,    %ds
    mov     %ax,    %es
    mov     %ax,    %fs
    mov     %ax,    %gs

    /* Properly setup RSP to idle stack */
    movabs  $g_idle_topstack,    %rbx
    mov     (%rbx),   %rsp

    /* We use jmp instruction below which doesn't push 1 byte on stack, so we
     * have to push a dummy value here, otherwise SSE instructions calledd
     * during initialization will fail.
     */
    pushq   $0

    /* Finally, we can start the OS */
    movabs  $__nxstart,   %rbx
    jmp     *%rbx
    .size   __pmode_entry, . - __pmode_entry

/****************************************************************************
 * Name: __revoke_low_memory
 *
 * Description:
 *   Revoke the lower 128MB memory mapping
 *
 ****************************************************************************/

	.section .text, "ax"
	.type   __revoke_low_memory, @function

__revoke_low_memory:
	/* Revoke the lower 128MB memory mapping */
	lea     g_pd_low, %edi
	lea     g_pt_low, %esi

	/* for ecx = 0...64 : Loop and setup 64x 2MB page directories */
	mov     $64,   %ecx
npd_loop:
	mov     %esi,   %edx
	or      $(X86_PAGE_WR | X86_PAGE_PRESENT), %edx
	mov     %edx, 0(%edi)
	add     $(PAGE_SIZE), %esi
	add     $(X86_PAGE_ENTRY_SIZE), %edi

	/* end for ecx */
	dec     %ecx
	jnz     npd_loop

	ret

	.size   __revoke_low_memory, . - __revoke_low_memory

/****************************************************************************
 * Name: __enable_sse_avx
 *
 * Description:
 *   Do low-level initialization SSE related processor setting
 *
 ****************************************************************************/

	.type   __enable_sse_avx, @function

__enable_sse_avx:
	/* Enable SSE */
	mov     %cr0,   %rax
	mov     $(X86_CR0_EM), %rbx
	not     %rbx
	and     %rbx,   %rax
	or      $(X86_CR0_MP), %rax
	mov     %rax,   %cr0

	/* Enable Saving XMM context */
	mov     %cr4,   %rax
	or      $(X86_CR4_OSXFSR | X86_CR4_XMMEXCPT),   %rax
	mov     %rax,   %cr4

	/* Setup MXCSR, masking all SSE precision exception */
	ldmxcsr mxcsr_mem

	ret

	.size   __enable_sse_avx, . - __enable_sse_avx

/****************************************************************************
 * Name: __enable_pcid
 *
 * Description:
 *   Enable PCID support
 *
 ****************************************************************************/

	.type   __enable_pcid, @function

__enable_pcid:
	/* Enable PCID and FGSBASE */
	mov     %cr4,   %rax
	or      $X86_CR4_PCIDE,     %rax
	mov     %rax,   %cr4

	ret

	.size   __enable_pcid, . - __enable_pcid

/****************************************************************************
 * .data
 ****************************************************************************/

	.section ".loader.data", "ax"

	/* IST for 64 bit long mode will be filled in up_irq */
	.align(16)
g_ist64_low:
	.long   0
	.quad   0xdeadbeefdeadbee0
	.quad   0xdeadbeefdeadbee1
	.quad   0xdeadbeefdeadbee2
	.quad   0
	.quad   0
	.quad   0
	.quad   0
	.quad   0
	.quad   0
	.quad   0
	.quad   0
	.quad   0
	.word   0

	/* GDT for 64 bit long mode */
	.align(16)
g_gdt64_low:
	.quad   0
	.quad   X86_GDT_CODE64_ENTRY
	.quad   X86_GDT_DATA_ENTRY
	.quad   X86_GDT_CODE32_ENTRY
	.quad   X86_GDT_DATA_ENTRY
	.quad   X86_GDT_CODE64_ENTRY
g_gdt64_ist_low:
	.quad   0x0 /* TSS segment low */
	.quad   0x0 /* TSS segment high */
g_gdt64_low_end:

gdt64_ptr:
	.short  g_gdt64_low_end - g_gdt64_low - 1
	.long   g_gdt64_low

mxcsr_mem:
	.long   0x00001f80

	.align(PAGE_SIZE)
pml4:
	.quad   g_pdpt_low + X86_PAGE_PRESENT + X86_PAGE_WR

	.align(PAGE_SIZE)
g_pdpt_low:
	.quad   g_pd_low + X86_PAGE_PRESENT + X86_PAGE_WR
	.quad   pd_2_low + X86_PAGE_PRESENT + X86_PAGE_WR
	.quad   pd_3_low + X86_PAGE_PRESENT + X86_PAGE_WR
	.quad   pd_4_low + X86_PAGE_PRESENT + X86_PAGE_WR

	.fill   X86_NUM_PAGE_ENTRY - 4, X86_PAGE_ENTRY_SIZE, 0

	.align(PAGE_SIZE)
g_pd_low:
	.fill   X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0

	.align(PAGE_SIZE)
pd_2_low:
	.fill   X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0

	.align(PAGE_SIZE)
pd_3_low:
	.fill   X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0

	.align(PAGE_SIZE)
pd_4_low:
	.fill   X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0

	.align(PAGE_SIZE)
g_pt_low:
	.fill   X86_NUM_PAGE_ENTRY * X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0
	.fill   X86_NUM_PAGE_ENTRY * X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0
	.fill   X86_NUM_PAGE_ENTRY * X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0
	.fill   X86_NUM_PAGE_ENTRY * X86_NUM_PAGE_ENTRY, X86_PAGE_ENTRY_SIZE, 0
