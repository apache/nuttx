/****************************************************************************
 *  arch/x86/src/intel64/intel64_head.S
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <arch/irq.h>
#include <arch/multiboot2.h>
#include <sys/types.h>

	.file "intel64_head.S"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Get XCR0 register value */

#if defined(CONFIG_ARCH_X86_64_AVX512)
#  define X86_XCR0_VAL (X86_XCR0_X87 | X86_XCR0_SSE | X86_XCR0_AVX | \
                        X86_XCR0_OPMASK | X86_XCR0_HI256 | X86_XCR0_HI16)
#elif defined(CONFIG_ARCH_X86_64_AVX)
#  define X86_XCR0_VAL (X86_XCR0_X87 | X86_XCR0_SSE | X86_XCR0_AVX)
#else
#  define X86_XCR0_VAL (X86_XCR0_X87 | X86_XCR0_SSE)
#endif

/* Get CR4 register value */

#ifdef CONFIG_ARCH_X86_64_HAVE_XSAVE
#  define X86_CR4_FPU_VAL (X86_CR4_OSXSAVE | X86_CR4_OSXFSR | X86_CR4_XMMEXCPT)
#else
#  define X86_CR4_FPU_VAL (X86_CR4_OSXFSR | X86_CR4_XMMEXCPT)
#endif

/* XSAVE header data offset */

#define X86_XSAVE_XSTATEBV_OFFSET  512
#define X86_XSAVE_XCOMPBC_OFFSET   520
#define X86_XSAVE_RESERVED0_OFFSET 528

/* Memory Map: _sbss is the start of the BSS region (see ld.script) _ebss is
 * the end of the BSS region (see ld.script). The idle task stack starts at
 * the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread
 * is the thread that the system boots on and, eventually, becomes the idle,
 * do nothing task that runs only when there is nothing else to run.  The
 * heap continues from there until the end of memory.  See g_idle_topstack below.
 */

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.global    __pmode_entry                   /* The 32bit protected mode entry */
#ifdef CONFIG_SMP
	.global    __ap_entry                      /* The 32bit real mode entry for AP */
#endif
	.global    __enable_sse_avx
	.global    __enable_pcid
	.global    __revoke_low_memory
#ifdef CONFIG_LIB_SYSCALL
	.global    x86_64_syscall_entry
	.global    x86_64_syscall
#endif
	.global    __nxstart                       /* __nxstart is defined elsewhere */
	.global    nx_start                        /* nx_start is defined elsewhere */
	.global    x86_64_ap_boot                  /* x86_64_ap_boot is defined elsewhere */
	.global    g_idle_topstack                 /* The end of the idle stack, the start of the heap */
	.global    g_mb_info_struct
	.global    g_mb_magic
	.global    g_cpu_count

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

#ifdef CONFIG_MULTBOOT2_FB
	.short  MULTIBOOT_HEADER_TAG_FRAMEBUFFER
	.short  0                           /* flags, none set */
	.long   20                          /* size, including itself (short + short + long) */
	.long   CONFIG_MULTBOOT2_FB_WIDTH   /* width */
	.long   CONFIG_MULTBOOT2_FB_HEIGHT  /* height */
	.long   CONFIG_MULTBOOT2_FB_DEEPTH  /* depth */
	.long   0
#endif

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

	/* Enable protected mode in CR0 */
	mov     %cr0, %eax
	or      $X86_CR0_PE, %al
	mov     %eax, %cr0

	/* Long jump into protected mode. Hardcode the address. */
	ljmpl   $0x8, $start32_0

	/* Loader GDT and GDTR */
	.align(16)
	.global loader_gdt
loader_gdt:
	.quad   0
	.quad   0x00cf9a000000ffff
	.quad   0x00cf92000000ffff
loader_gdt_ptr:
	.short  loader_gdt_ptr - loader_gdt - 1
	.long   loader_gdt

	.size   __reset_entry, . - __reset_entry

#ifdef CONFIG_SMP
/****************************************************************************
 * Name: __ap_entry
 *
 * Description:
 *   Entry point for AP CPU 32-bit real mode which must be aligned to 4096
 *   bytes. This is simply copy of __reset_entry.
 *
 ****************************************************************************/

	.type   __ap_entry, @function
	.align(4096)
__ap_entry:
	/* Load a GDT for protected mode */
	movl    $loader_gdt_ptr, %ebx
	lgdtl   (%ebx)

	/* enable protected mode in CR0 */
	mov     %cr0, %eax
	or      $X86_CR0_PE, %al
	mov     %eax, %cr0

	/* Long jump into protected mode. Hardcode the address. */
	ljmpl   $0x8, $start32_0
#endif

/****************************************************************************
 * .text
 ****************************************************************************/

	.code32
#ifdef CONFIG_ARCH_MULTIBOOT1
	.section ".multiboot1", "a"
.set MB_FLAG_ALIGNED,   1       /* All boot modules must be loaded aligned */
.set MB_FLAG_MEMINFO,   2       /* Boot with memory maps passing  */
.set MB_FLAG_VIDEO,     4       /* Enable video mode  */

.set MB_FLAGS,          MB_FLAG_ALIGNED | MB_FLAG_MEMINFO
.set MB_MAGIC,          0x1BADB002
.set MB_CHECKSUM,       -(MB_MAGIC + MB_FLAGS)

multiboot1_header:
	.align 4
	.long               MB_MAGIC
	.long               MB_FLAGS
	.long               MB_CHECKSUM

	.long               0       /* header_addr  */
	.long               0       /* load_addr */
	.long               0       /* load_end_addr */
	.long               0       /* bss_end_addr */
	.long               0       /* entry_addr */

	.long               0       /* Video mode type */
	.long               1024    /* Video width */
	.long               768     /* Video height */
	.long               32      /* Video depth */
#endif

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

	/* Jump to 64 bit initialiation if this is AP core */
	mov     bsp_done, %eax
	cmp     $0, %eax
	jne     start64_init

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

start64_init:
	/* Populate the 1GB after 4GB boundary with Global mapping to kernel code.
	 * This creates maps the lower 1GB to 4GB~5GB
	 */
	lea     g_pdpt_low, %edi
	mov     $(X86_PDPT_KERNEL_MAP), %eax

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
#ifdef CONFIG_LIB_SYSCALL
	or      $(EFER_LME | EFER_SCE),  %eax
#else
	or      $EFER_LME,  %eax
#endif
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

	/* Start BSP or AP */
	mov     bsp_done, %eax
	cmp     $0, %eax
	jne     ap_start

	/* Properly setup RSP to idle stack */
	movabs  $g_idle_topstack,    %rbx
	mov     (%rbx),   %rsp

	/* Allocate space XCPTCONTEXT */
	leaq    -XCPTCONTEXT_SIZE(%rsp), %rsp

	/* Set bsp_done flag */
	movl     $1, bsp_done

	/* We use jmp instruction below which doesn't push 1 byte on stack, so we
	 * have to push a dummy value here, otherwise SSE instructions called
	 * during initialization will fail.
	 */
	pushq   $0

	/* Finally, we can start the OS */
	movabs  $__nxstart,   %rbx
	jmp     *%rbx

ap_start:
#ifdef CONFIG_SMP
	/* Get CPU ID */
	movl    g_cpu_count, %edi

	/* Setup AP stack */
	movabs  $g_idle_topstack,    %rbx
	movl    %edi, %eax
	imul    $8, %eax, %eax
	add     %rax, %rbx
	mov     (%rbx),   %rsp

	/* Move initial RSP below IDLE TCB regs */
	sub     $XCPTCONTEXT_SIZE, %rsp
	and     $(~XCPTCONTEXT_SIZE), %rsp

	/* Jump to ap_start routine */
	movabs  $x86_64_ap_boot,   %rbx

	/* We need to simulate the behavior of the call instruction, which by
	 * default pushes 8 bytes of the RIP. Otherwise, the function's stack
	 * won't be 16-byte aligned.
	 */

	pushq   $0
	jmp     *%rbx
#endif

	.size   __pmode_entry, . - __pmode_entry

g_cpu_count:
	.long 1

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
	movq    $X86_64_LOAD_OFFSET,%rcx
	lea     g_pd_low, %edi
	add     %rcx, %rdi
	lea     g_pt_low, %esi
	add     %rcx, %rsi

	/* for ecx = 0...64 : Loop and setup 64x 2MB page directories */
	mov     $64,   %ecx
npd_loop:
	mov     %rsi,   %rdx
	or      $(X86_PAGE_WR | X86_PAGE_PRESENT), %rdx
	mov     %rdx, 0(%rdi)
	add     $(PAGE_SIZE), %rsi
	add     $(X86_PAGE_ENTRY_SIZE), %rdi

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
	or      $X86_CR4_FPU_VAL,   %rax
	mov     %rax,   %cr4

#ifdef CONFIG_ARCH_X86_64_HAVE_XSAVE
	/* Configure XSAVE/XRSTOR for user state*/
	mov     $X86_XCR0_VAL, %eax

	xor     %rdx,  %rdx
	xor     %rcx,  %rcx
	xsetbv

#else
	/* Setup MXCSR, masking all SSE precision exception */
	ldmxcsr mxcsr_mem
#endif

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

#ifdef CONFIG_LIB_SYSCALL
  /****************************************************************************
  * Name: x86_64_syscall_entry
  *
  * Description:
  *   Landing point for syscall instruction.
  *
  *   At this point RFLAGS are masked with MSR_FMASK
  *     - IF=0 (interrupts are disabled)
  *     - DF=0
  *
  ****************************************************************************/

	.type   x86_64_syscall_entry, @function
x86_64_syscall_entry:
	/* Store current RSP on CPU private data first */
	movq   %rsp, %gs:X86_64_CPUPRIV_USTACK_OFFSET

#ifdef CONFIG_ARCH_KERNEL_STACK
	/* If current RSP is greater than kernel stack, we have to switch stack.
	 * Otherwise we are in nested syscall and we can't modify stack pointer
	 */
	cmp    %gs:X86_64_CPUPRIV_UVBASE_OFFSET, %rsp
	jb     no_kstack_switch

	/* Change to kernel stack */
	movq   %gs:X86_64_CPUPRIV_KTOPSTK_OFFSET, %rsp
no_kstack_switch:
#endif

	/* Store some registers on stack.
	 * We need some free registers here to handle stored registers alignment
	 * and kernel stack for nested syscalls but at this point we don't have any
	 *
	 * RDI is needed only for CONFIG_ARCH_KERNEL_STACK=y but to simplify
	 * the logic here - we always release it.
	 */
	pushq   %rcx
	pushq   %rdi

	/* Get original kernel stack for this call */

	movq   %rsp, %rdi
	add    $16, %rdi

	/* Get aligned registers area */
	movq   %rsp, %rcx
	sub    $8, %rcx
	sub    $((XCPTCONTEXT_REGS + 8) * 8), %rcx
	add    $(0x3f), %rcx
	and    $(0xffffffffffffffc0), %rcx

	/* Syscall arguments */
	movq    %rax, (8*REG_RAX)(%rcx)
	movq    %rsi, (8*REG_RSI)(%rcx)
	movq    %rdx, (8*REG_RDX)(%rcx)
	movq    %r10, (8*REG_R10)(%rcx)
	movq    %r8,  (8*REG_R8)(%rcx)
	movq    %r9,  (8*REG_R9)(%rcx)

	/* Callee registers */
	movq    %rbx, (8*REG_RBX)(%rcx)
	movq    %r11, (8*REG_R11)(%rcx)
	movq    %r12, (8*REG_R12)(%rcx)
	movq    %r13, (8*REG_R13)(%rcx)
	movq    %r14, (8*REG_R14)(%rcx)
	movq    %r15, (8*REG_R15)(%rcx)
	movq    %rbp, (8*REG_RBP)(%rcx)

#ifndef CONFIG_ARCH_X86_64_HAVE_XSAVE
	/* Save xmm registers */
	fxsaveq (%rcx)
#else
#  ifdef CONFIG_ARCH_CHIP_INTEL64_QEMU
	/* BUGFIX for QEMU: make sure that xsave header is zeroed!
	 * QEMU desn't clear these fields during xsave, so if the memory region
	 * for xsave state was not cleared before use, there may be junk data there,
	 * that cause xrstor to crash later.
	 */
	movq    $0, (X86_XSAVE_XSTATEBV_OFFSET)(%rcx)
	movq    $0, (X86_XSAVE_XCOMPBC_OFFSET)(%rcx)
	movq    $0, (X86_XSAVE_RESERVED0_OFFSET)(%rcx)
#  endif

	movl    $XSAVE_STATE_COMPONENTS, %eax
	xor     %edx, %edx
	xsave   (%rcx)
#endif

	/* Save RDI and RCX which are on stack now */
	popq    (8*REG_RDI)(%rcx)
	popq    (8*REG_RCX)(%rcx)

	/* Store user stack pointer. We can't movq directly here.
	 * NOTE: for nested syscalls this value points to kernel stack.
	 */
	pushq   %gs:X86_64_CPUPRIV_USTACK_OFFSET
	popq    (8*REG_RSP)(%rcx)

	/* Move stack pointer after registers area */
	movq    %rcx, %rsp

#ifdef CONFIG_ARCH_KERNEL_STACK
	/* Update stored kernel stack */
	movq    %rsp, %gs:X86_64_CPUPRIV_KTOPSTK_OFFSET

	/* Store original kernel stack on stack which is now in RDI */
	pushq   %rdi
#endif

	/* Return value from syscall stored in rax */
	movq    %rcx, %rdi
	call    x86_64_syscall
	movq    %rax, %rdi

#ifndef CONFIG_ARCH_X86_64_HAVE_XSAVE
	/* Restore xmm registers */
	fxrstorq (%rdi)
#else
	movl    $XSAVE_STATE_COMPONENTS, %eax
	xor     %edx, %edx
	xrstor  (%rdi)
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
	/* Restore original kernel stack */
	popq    %gs:X86_64_CPUPRIV_KTOPSTK_OFFSET
#endif

	/* Restore original user RSP */
	movq    (8*REG_RSP)(%rdi), %rsp

	/* Restore CPU registers - regs are on RDI now */
	movq    (8*REG_RBX)(%rdi), %rbx
	movq    (8*REG_R11)(%rdi), %r11
	movq    (8*REG_R12)(%rdi), %r12
	movq    (8*REG_R13)(%rdi), %r13
	movq    (8*REG_R14)(%rdi), %r14
	movq    (8*REG_R15)(%rdi), %r15
	movq    (8*REG_RBP)(%rdi), %rbp

	movq    (8*REG_RAX)(%rdi), %rax
	movq    (8*REG_RSI)(%rdi), %rsi
	movq    (8*REG_RDX)(%rdi), %rdx
	movq    (8*REG_R10)(%rdi), %r10
	movq    (8*REG_RCX)(%rdi), %rcx

	movq    (8*REG_R11)(%rdi), %r11
	movq    (8*REG_RDI)(%rdi), %rdi

#  ifdef CONFIG_BUILD_KERNEL
	/* Do not return to RING3 if this is nested syscall */
	cmp %gs:X86_64_CPUPRIV_UVBASE_OFFSET, %rcx
	jb syscall_no_ring3

	/* Return to user code pointed in RCX */
	sysretq

syscall_no_ring3:
#  endif
	/* Return to address pointed in RCX - must be on stack */
	pushq   %rcx
	ret

	.size   x86_64_syscall_entry, . - x86_64_syscall_entry
#endif

/****************************************************************************
 * .data
 ****************************************************************************/

	.section ".loader.data", "ax"

bsp_done:
	.long   0

	/* TSS (IST) for 64 bit long mode will be filled in up_irq. */
	.align(16)
g_ist64_low:
	.fill   X86_TSS_SIZE * CONFIG_SMP_NCPUS, 1, 0

	/* GDT for 64 bit long mode */
	.align(16)
g_gdt64_low:
	/* Must be in sequence required by SYSCALL and SYSRET.
	 * For details look at the comment in intel64_cpu.c about MSR_STAT write.
	 */
	.quad   0                      /* 0x00: null descriptor */
	.quad   X86_GDT_CODE64_ENTRY   /* 0x08: kernel code 64 */
	.quad   X86_GDT_DATA_ENTRY     /* 0x10: kernel data */
	.quad   X86_GDT_CODE32_ENTRY   /* 0x18: kernel code 32 */
	.quad   X86_GDT_DATA_ENTRY     /* 0x20: kernel data */
	.quad   X86_GDT_CODE64_ENTRY   /* 0x28: kernel code 64 */
	.quad   X86_GDT_DATAUSER_ENTRY /* 0x30: user data */
	.quad   X86_GDT_CODEUSER_ENTRY /* 0x38: user code */

g_gdt64_ist_low:
	/* TSS segment low + segment high per CPU */
	.fill   CONFIG_SMP_NCPUS * 16, 1, 0

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

#ifdef CONFIG_ARCH_PVHBOOT
	/* PVH Header with pvh_start_addr = start32 */
	.pushsection .note.nuttx, "a", @note
	.align 4
	.long 2f - 1f
	.long 4f - 3f
	.long 18
	1:.asciz "nuttx"
	2:.align 4
	3:.long start32
	4:.align 4
	.popsection
#endif
