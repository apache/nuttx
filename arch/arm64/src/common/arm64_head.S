/****************************************************************************
 * arch/arm64/src/common/arm64_head.S
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
 *    DESCRIPTION
 *        Bring-up code for ARMv8.
 *        Based on head.S(arm64 porting) at Xen Hypervisor Project
 *        Based on reset.S(aarch64 porting) at Zephyr RTOS Project
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm64_arch.h"
#include "arm64_macro.inc"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "arm64_head.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* macro define from xen head, for efi head define */
#define PAGE_SHIFT              12
#define __HEAD_FLAG_PAGE_SIZE   ((PAGE_SHIFT - 10) / 2)

#define __HEAD_FLAG_PHYS_BASE   1


#define __HEAD_FLAGS            ((__HEAD_FLAG_PAGE_SIZE << 1) | \
                                 (__HEAD_FLAG_PHYS_BASE << 3))

#ifdef CONFIG_ARCH_EARLY_PRINT

#define RODATA_STR(label, msg)                  \
.pushsection .rodata.str, "aMS", %progbits, 1 ; \
label:  .asciz msg;                             \
.popsection

/* Macro to print a string to the UART, if there is one.
 * Clobbers x0 - x3, x30 is lr for return
 */
#define PRINT(sym, _s)            \
    mov   x3, x30 ;                \
    ldr   x1, =boot_string_##sym ; \
    bl    boot_stage_puts;         \
    mov   x30, x3 ;                \
    RODATA_STR(boot_string_##sym, _s)
#else
#define PRINT(sym, s)
#endif /* CONFIG_ARCH_EARLY_PRINT */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

    /* Kernel startup entry point.
     * ---------------------------
     *
     * The requirements are:
     *   MMU = off, D-cache = off, I-cache = on or off,
     *   x0 = physical address to the FDT blob.
     *       it will be used when NuttX support device tree in the future
     *
     * This must be the very first address in the loaded image.
     * It should be loaded at any 4K-aligned address.
     */
    .globl __start;
__start:

    /* DO NOT MODIFY. Image header expected by Linux boot-loaders.
     *
     * This add instruction has no meaningful effect except that
     * its opcode forms the magic "MZ" signature of a PE/COFF file
     * that is required for UEFI applications.
     *
     * Some bootloader (such imx8 uboot) checking the magic "MZ" to see
     * if the image is a valid Linux image. but modifying the bootLoader is
     * unnecessary unless we need to do a customize secure boot.
     * so just put the ''MZ" in the header to make bootloader happiness
     */

    add     x13, x18, #0x16      /* the magic "MZ" signature */
    b       real_start           /* branch to kernel start */
    .quad   0x480000              /* Image load offset from start of RAM */
    .quad   _e_initstack - __start         /* Effective size of kernel image, little-endian */
    .quad   __HEAD_FLAGS         /* Informative flags, little-endian */
    .quad   0                    /* reserved */
    .quad   0                    /* reserved */
    .quad   0                    /* reserved */
    .ascii  "ARM\x64"            /* Magic number, "ARM\x64" */
    .long   0                    /* reserved */

real_start:
    /* Disable all exceptions and interrupts */

    msr    DAIFSet, 0xf
#ifdef CONFIG_SMP

    ldr    x0, =cpu_boot_params
    get_cpu_id x1

    /* The global variable cpu_boot_params is not safety to
     * access in some case. eg. Some debugger will reboot
     * the NuttX but not reload the whole image, so it will
     * be not predicable for the initial value of the global
     * value in that case
     *
     * get_cpu_id is safety because the CPU identification is
     * not change in any case, so the code will judge in a
     * very simple method:
     *  -- Primary core will go on until it want to boot the
     *     other core. For NuttX OS usage case, we can consider
     *     the CPU id (the affinity of mpidr_el1) of primary
     *     core is always 0.
     *  -- The other cores will waiting until the primary
     *     core write mpid and notify it to go on
     */

    cmp    x1, #0
    beq    primary_core

    /* loop until our turn comes */

1:  dmb    ld
    wfe
    ldr    x2,  [x0, #BOOT_PARAM_MPID]
    cmp    x1, x2
    bne    1b

    /* we can now load our stack pointer value and move on */

    ldr    x24, [x0, #BOOT_PARAM_SP]
    add    x24, x24, #(CONFIG_IDLETHREAD_STACKSIZE)
    ldr    x25, =arm64_boot_secondary_c_routine
    bl     __reset_prep_c

    PRINT(second_boot, "- Ready to Boot Second CPU\r\n")

    b      cpu_boot

primary_core:
    /* set primary core id */

    str    x1,  [x0, #BOOT_PARAM_MPID]

    ldr    x24, [x0, #BOOT_PARAM_SP]
    add    x24, x24, #(CONFIG_IDLETHREAD_STACKSIZE)
#else
    /* In some case, we need to boot one core in a SMP system,
     * To avoid the primary core disturbed by the other cores,
     * we need keep the other cores into WFE loop
     */
    get_cpu_id x1
    cmp    x1, #0
    bne    fail

    /* load stack and entry point */

    ldr    x24, =(g_idle_stack + CONFIG_IDLETHREAD_STACKSIZE)
#endif /* CONFIG_SMP */

    ldr    x25, =arm64_boot_primary_c_routine

    /* Prepare for calling C code */

    bl     __reset_prep_c

#ifdef CONFIG_ARCH_EARLY_PRINT
    /* Initialize the UART for early print.
     * Should only be called on the boot CPU
     */

    bl    up_earlyserialinit
#endif

    PRINT(primary_boot, "- Ready to Boot Primary CPU\r\n")

cpu_boot:

    /* Platform hook for highest EL */

    bl  arm64_el_init

switch_el:
    switch_el x0, 3f, 2f, 1f
3:
#ifdef CONFIG_ARCH_HAVE_EL3
    PRINT(switch_el3, "- Boot from EL3\r\n")

    /* EL3 init */

    bl    arm64_boot_el3_init

    /* Get next EL */

    adr   x0, switch_el
    bl    arm64_boot_el3_get_next_el
    eret
#endif
2:
    PRINT(switch_el2, "- Boot from EL2\r\n")

    /* EL2 init */

    bl    arm64_boot_el2_init

    /* Move to EL1 with all exceptions masked */

    mov_imm    x0, (SPSR_DAIF_MASK | SPSR_MODE_EL1T)
    msr   spsr_el2, x0

    adr   x0, 1f
    msr   elr_el2, x0
    eret

1:
    PRINT(switch_el1, "- Boot from EL1\r\n")

    /* EL1 init */

    bl    arm64_boot_el1_init

    /* set SP_ELx and Enable SError interrupts */

    msr   SPSel, #1
    msr   DAIFClr, #(DAIFCLR_ABT_BIT)
    isb

jump_to_c_entry:
    PRINT(jump_to_c_entry, "- Boot to C runtime for OS Initialize\r\n")
    ret x25

/* Fail-stop */

fail:
    /* Boot failed */

1:  wfe
    b     1b

/* Set the minimum necessary to safely call C code */

__reset_prep_c:

    /* return address: x23 */

    mov   x23, lr

    switch_el x0, 3f, 2f, 1f
3:
#ifdef CONFIG_ARCH_HAVE_EL3
    /* Reinitialize SCTLR from scratch in EL3 */

    ldr   w0, =(SCTLR_EL3_RES1 | SCTLR_I_BIT | SCTLR_SA_BIT)
    msr   sctlr_el3, x0
#endif

    /* Set SP_EL1 */

    msr   sp_el1, x24
    b     out
2:
    /* Disable alignment fault checking */

    mrs   x0, sctlr_el2
    bic   x0, x0, SCTLR_A_BIT
    msr   sctlr_el2, x0

    /* Set SP_EL1 */

    msr   sp_el1, x24
    b     out
1:
    /* Disable alignment fault checking */

    mrs   x0, sctlr_el1
    bic   x0, x0, SCTLR_A_BIT
    msr   sctlr_el1, x0

    /* Set SP_EL1. We cannot use sp_el1 at EL1 */

    msr   SPSel, #1
    mov   sp, x24
out:
    isb

    /* Select SP_EL0 and Initialize stack */

    msr   SPSel, #0
    mov   sp, x24

    ret   x23

#ifdef CONFIG_ARCH_EARLY_PRINT

/* Print early debug messages.
 * x0: Nul-terminated string to print.
 * Clobbers x0-x1
 */

boot_stage_puts:
    ldrb  w0, [x1], #1       /* Load next char */
    cmp   w0, 0
    beq   1f                 /* Exit on nul */
    stp   xzr, x30, [sp, #-16]!
    bl    arm64_lowputc
    ldp   xzr, x30, [sp], #16
    b     boot_stage_puts
1:
    ret

.type boot_low_puts, %function;

#endif /* !CONFIG_ARCH_EARLY_PRINT */
