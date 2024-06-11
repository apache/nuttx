/*********************************************************************************
 * libs/libc/machine/x86_64/gnu/sysdep.h
 *
 * Copyright (c) 2014, Intel Corporation
 * All rights reserved.
 * The GNU C Library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * The GNU C Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 *
 *********************************************************************************/

#ifndef __LIBS_LIBC_MACHINE_X86_64_GNU_SYSDEP_H
#define __LIBS_LIBC_MACHINE_X86_64_GNU_SYSDEP_H

#ifndef C_LABEL
/* Define a macro we can use to construct the asm name for a C symbol.  */
#  define C_LABEL(name) name##:
#endif

/* Makros to generate eh_frame unwind information.  */
#ifdef __ASSEMBLER__
#  define cfi_startproc   .cfi_startproc
#  define cfi_endproc   .cfi_endproc
#  define cfi_def_cfa(reg, off)  .cfi_def_cfa reg, off
#  define cfi_def_cfa_register(reg) .cfi_def_cfa_register reg
#  define cfi_def_cfa_offset(off) .cfi_def_cfa_offset off
#  define cfi_adjust_cfa_offset(off) .cfi_adjust_cfa_offset off
#  define cfi_offset(reg, off)  .cfi_offset reg, off
#  define cfi_rel_offset(reg, off) .cfi_rel_offset reg, off
#  define cfi_register(r1, r2)  .cfi_register r1, r2
#  define cfi_return_column(reg) .cfi_return_column reg
#  define cfi_restore(reg)  .cfi_restore reg
#  define cfi_same_value(reg)  .cfi_same_value reg
#  define cfi_undefined(reg)  .cfi_undefined reg
#  define cfi_remember_state  .cfi_remember_state
#  define cfi_restore_state  .cfi_restore_state
#  define cfi_window_save  .cfi_window_save
#  define cfi_personality(enc, exp) .cfi_personality enc, exp
#  define cfi_lsda(enc, exp)  .cfi_lsda enc, exp

/* Used by some assembly code.  */
#  define C_SYMBOL_NAME(name) name

/* Syntactic details of assembler.  */

#ifdef _CET_ENDBR
#  define _CET_NOTRACK notrack
#else
#  define _CET_ENDBR
#  define _CET_NOTRACK
#endif

/* ELF uses byte-counts for .align, most others use log2 of count of bytes.  */
#  define ALIGNARG(log2) 1<<log2
#  define ASM_SIZE_DIRECTIVE(name) .size name,.-name;

/* Define an entry point visible from C.  */
#  define ENTRY_P2ALIGN(name, alignment)           \
  .globl C_SYMBOL_NAME(name);            \
  .type C_SYMBOL_NAME(name),@function;           \
  .align ALIGNARG(alignment);            \
  C_LABEL(name)              \
  cfi_startproc;             \
  _CET_ENDBR;              \
  CALL_MCOUNT

/* Common entry 16 byte aligns.  */
#  define ENTRY(name) ENTRY_P2ALIGN (name, 4)

#undef END
#  define END(name)             \
  cfi_endproc;              \
  ASM_SIZE_DIRECTIVE(name)

#  define ENTRY_CHK(name) ENTRY (name)
#  define END_CHK(name) END (name)

/* Local label name for asm code. */
#ifndef L
/* ELF-like local names start with `.L'.  */
#  define LOCAL_LABEL(name) .L##name
#  define L(name) LOCAL_LABEL(name)
#endif

/* Syntactic details of assembler.  */

/* This macro is for setting proper CFI with DW_CFA_expression describing
 * the register as saved relative to %rsp instead of relative to the CFA.
 * Expression is DW_OP_drop, DW_OP_breg7 (%rsp is register 7), sleb128 offset
 * from %rsp.
 * */

#  define cfi_offset_rel_rsp(regn, off) .cfi_escape 0x10, regn, 0x4, 0x13, \
     0x77, off & 0x7F | 0x80, off >> 7

/* If compiled for profiling, call `mcount' at the start of each function.  */
#ifdef PROF
/* The mcount code relies on a normal frame pointer being on the stack
 * to locate our caller, so push one just for its benefit.
 */

#  define CALL_MCOUNT                                                          \
  pushq %rbp;                                                                \
  cfi_adjust_cfa_offset(8);                                                  \
  movq %rsp, %rbp;                                                           \
  cfi_def_cfa_register(%rbp);                                                \
  call JUMPTARGET(mcount);                                                   \
  popq %rbp;                                                                 \
  cfi_def_cfa(rsp,8);
#else
#  define CALL_MCOUNT  /* Do nothing.  */
#endif

/* Registers to hold long and pointer.  */
#  define RAX_LP rax
#  define RBP_LP rbp
#  define RBX_LP rbx
#  define RCX_LP rcx
#  define RDI_LP rdi
#  define RDX_LP rdx
#  define RSI_LP rsi
#  define RSP_LP rsp
#  define R8_LP r8
#  define R9_LP r9
#  define R10_LP r10
#  define R11_LP r11
#  define R12_LP r12
#  define R13_LP r13
#  define R14_LP r14
#  define R15_LP r15

#else /* __ASSEMBLER__ */

/* Instruction to operate on long and pointer.  */
#  define LP_OP(insn) #insn "q"

/* Assembler address directive. */
#  define ASM_ADDR ".quad"

#  define CFI_STRINGIFY(Name) CFI_STRINGIFY2 (Name)
#  define CFI_STRINGIFY2(Name) #Name
#  define CFI_STARTPROC ".cfi_startproc"
#  define CFI_ENDPROC ".cfi_endproc"
#  define CFI_DEF_CFA(reg, off) \
   ".cfi_def_cfa " CFI_STRINGIFY(reg) "," CFI_STRINGIFY(off)
#  define CFI_DEF_CFA_REGISTER(reg) \
   ".cfi_def_cfa_register " CFI_STRINGIFY(reg)
#  define CFI_DEF_CFA_OFFSET(off) \
   ".cfi_def_cfa_offset " CFI_STRINGIFY(off)
#  define CFI_ADJUST_CFA_OFFSET(off) \
   ".cfi_adjust_cfa_offset " CFI_STRINGIFY(off)
#  define CFI_OFFSET(reg, off) \
   ".cfi_offset " CFI_STRINGIFY(reg) "," CFI_STRINGIFY(off)
#  define CFI_REL_OFFSET(reg, off) \
   ".cfi_rel_offset " CFI_STRINGIFY(reg) "," CFI_STRINGIFY(off)
#  define CFI_REGISTER(r1, r2) \
   ".cfi_register " CFI_STRINGIFY(r1) "," CFI_STRINGIFY(r2)
#  define CFI_RETURN_COLUMN(reg) \
   ".cfi_return_column " CFI_STRINGIFY(reg)
#  define CFI_RESTORE(reg) \
   ".cfi_restore " CFI_STRINGIFY(reg)
#  define CFI_UNDEFINED(reg) \
   ".cfi_undefined " CFI_STRINGIFY(reg)
#  define CFI_REMEMBER_STATE \
   ".cfi_remember_state"
#  define CFI_RESTORE_STATE \
   ".cfi_restore_state"
#  define CFI_WINDOW_SAVE \
   ".cfi_window_save"
#  define CFI_PERSONALITY(enc, exp) \
   ".cfi_personality " CFI_STRINGIFY(enc) "," CFI_STRINGIFY(exp)
#  define CFI_LSDA(enc, exp) \
   ".cfi_lsda " CFI_STRINGIFY(enc) "," CFI_STRINGIFY(exp)

#endif /* __ASSEMBLER__ */

#endif /* __LIBS_LIBC_MACHINE_X86_64_GNU_SYSDEP_H */
