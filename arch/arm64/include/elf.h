/****************************************************************************
 * arch/arm64/include/elf.h
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

/* Reference: "ELF for the ARM 64-bit Architecture," ARM IHI 0056B, current
 * through AArch64 ABI release 1.0, May 22, 2013, ARM Limited.
 */

#ifndef __ARCH_ARM64_INCLUDE_ELF_H
#define __ARCH_ARM64_INCLUDE_ELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* 4.2.1 ELF Identification.  Should have:
 *
 * e_machine         = EM_AARCH64
 * e_ident[EI_CLASS] = ELFCLASS64
 * e_ident[EI_DATA]  = ELFDATA2LSB (little endian) or
 *                     ELFDATA2MSB (big endian)
 */

#define EM_ARCH  EM_AARCH64

/* e_flags: there are no processor-specific flags so this field
 * shall contain zero.
 */

/* Table 4-2, Processor specific section types */

#define SHT_AARCH64_ATTRIBUTES  0x70000003 /* Object file compatibility attributes */

/* 4.6.3 Relocation codes
 *
 * S (when used on its own) is the address of the symbol.
 * A is the addend for the relocation.
 * P is the address of the place being relocated (derived from r_offset).
 * X is the result of a relocation operation, before any masking or
 *   bit-selection operation is applied
 * Page(expr) is the page address of the expression expr, defined as
 *   (expr & ~0xFFF). (This applies even if the machine page size supported
 *   by the platform has a different value.)
 * GOT is the address of the Global Offset Table, the table of code and data
 *   addresses to be resolved at dynamic link time. The GOT and each entry in
 *   it must be 64-bit aligned.
 * GDAT(S+A) represents a 64-bit entry in the GOT for address S+A. The entry
 *   will be relocated at run time with relocation R_AARCH64_GLOB_DAT(S+A).
 * G(expr) is the address of the GOT entry for the expression expr.
 * Delta(S) if S is a normal symbol, resolves to the difference between the
 *   static link address of S and the execution address of S. If S is the
 *   null symbol (ELF symbol index 0), resolves to the difference between
 *   the static link address of P and the execution address of P.
 * Indirect(expr) represents the result of calling expr as a function. The
 *   result is the return value from the function that is returned in r0.
 *   The arguments passed to the function are defined by the platform ABI.
 * [msb:lsb] is a bit-mask operation representing the selection of bits in
 *   a value. The bits selected range from lsb up to msb inclusive. For
 *   example, ‘bits [3:0]’ represents the bits under the mask 0x0000000F.
 *   When range checking is applied to a value, it is applied before the
 *   masking operation is performed.
 */

#define R_AARCH64_NONE                         0    /* Miscellaneous No relocation */
#define R_AARCH64_ABS64                        257  /* Direct 64 bit. */
#define R_AARCH64_ABS32                        258  /* Direct 32 bit.  */
#define R_AARCH64_ABS16                        259  /* Direct 16-bit.  */
#define R_AARCH64_PREL64                       260  /* PC-relative 64-bit. */
#define R_AARCH64_PREL32                       261  /* PC-relative 32-bit. */
#define R_AARCH64_PREL16                       262  /* PC-relative 16-bit. */
#define R_AARCH64_MOVW_UABS_G0                 263  /* Dir. MOVZ imm. from bits 15:0.  */
#define R_AARCH64_MOVW_UABS_G0_NC              264  /* Likewise for MOVK; no check.  */
#define R_AARCH64_MOVW_UABS_G1                 265  /* Dir. MOVZ imm. from bits 31:16.  */
#define R_AARCH64_MOVW_UABS_G1_NC              266  /* Likewise for MOVK; no check.  */
#define R_AARCH64_MOVW_UABS_G2                 267  /* Dir. MOVZ imm. from bits 47:32.  */
#define R_AARCH64_MOVW_UABS_G2_NC              268  /* Likewise for MOVK; no check.  */
#define R_AARCH64_MOVW_UABS_G3                 269  /* Dir. MOV{K,Z} imm. from 63:48.  */
#define R_AARCH64_MOVW_SABS_G0                 270  /* Dir. MOV{N,Z} imm. from 15:0.  */
#define R_AARCH64_MOVW_SABS_G1                 271  /* Dir. MOV{N,Z} imm. from 31:16.  */
#define R_AARCH64_MOVW_SABS_G2                 272  /* Dir. MOV{N,Z} imm. from 47:32.  */
#define R_AARCH64_LD_PREL_LO19                 273  /* PC-rel. LD imm. from bits 20:2.  */
#define R_AARCH64_ADR_PREL_LO21                274  /* PC-rel. ADR imm. from bits 20:0.  */
#define R_AARCH64_ADR_PREL_PG_HI21             275  /* Page-rel. ADRP imm. from 32:12.  */
#define R_AARCH64_ADR_PREL_PG_HI21_NC          276  /* Likewise; no overflow check.  */
#define R_AARCH64_ADD_ABS_LO12_NC              277  /* Dir. ADD imm. from bits 11:0.  */
#define R_AARCH64_LDST8_ABS_LO12_NC            278  /* Likewise for LD/ST; no check. */
#define R_AARCH64_TSTBR14                      279  /* PC-rel. TBZ/TBNZ imm. from 15:2.  */
#define R_AARCH64_CONDBR19                     280  /* PC-rel. cond. br. imm. from 20:2. */
#define R_AARCH64_JUMP26                       282  /* PC-rel. B imm. from bits 27:2.  */
#define R_AARCH64_CALL26                       283  /* Likewise for CALL.  */
#define R_AARCH64_LDST16_ABS_LO12_NC           284  /* Dir. ADD imm. from bits 11:1.  */
#define R_AARCH64_LDST32_ABS_LO12_NC           285  /* Likewise for bits 11:2.  */
#define R_AARCH64_LDST64_ABS_LO12_NC           286  /* Likewise for bits 11:3.  */
#define R_AARCH64_MOVW_PREL_G0                 287  /* PC-rel. MOV{N,Z} imm. from 15:0.  */
#define R_AARCH64_MOVW_PREL_G0_NC              288  /* Likewise for MOVK; no check.  */
#define R_AARCH64_MOVW_PREL_G1                 289  /* PC-rel. MOV{N,Z} imm. from 31:16. */
#define R_AARCH64_MOVW_PREL_G1_NC              290  /* Likewise for MOVK; no check.  */
#define R_AARCH64_MOVW_PREL_G2                 291  /* PC-rel. MOV{N,Z} imm. from 47:32. */
#define R_AARCH64_MOVW_PREL_G2_NC              292  /* Likewise for MOVK; no check.  */
#define R_AARCH64_MOVW_PREL_G3                 293  /* PC-rel. MOV{N,Z} imm. from 63:48. */
#define R_AARCH64_LDST128_ABS_LO12_NC          299  /* Dir. ADD imm. from bits 11:4.  */
#define R_AARCH64_MOVW_GOTOFF_G0               300  /* GOT-rel. off. MOV{N,Z} imm. 15:0. */
#define R_AARCH64_MOVW_GOTOFF_G0_NC            301  /* Likewise for MOVK; no check.  */
#define R_AARCH64_MOVW_GOTOFF_G1               302  /* GOT-rel. o. MOV{N,Z} imm. 31:16.  */
#define R_AARCH64_MOVW_GOTOFF_G1_NC            303  /* Likewise for MOVK; no check.  */
#define R_AARCH64_MOVW_GOTOFF_G2               304  /* GOT-rel. o. MOV{N,Z} imm. 47:32.  */
#define R_AARCH64_MOVW_GOTOFF_G2_NC            305  /* Likewise for MOVK; no check.  */
#define R_AARCH64_MOVW_GOTOFF_G3               306  /* GOT-rel. o. MOV{N,Z} imm. 63:48.  */
#define R_AARCH64_GOTREL64                     307  /* GOT-relative 64-bit.  */
#define R_AARCH64_GOTREL32                     308  /* GOT-relative 32-bit.  */
#define R_AARCH64_GOT_LD_PREL19                309  /* PC-rel. GOT off. load imm. 20:2.  */
#define R_AARCH64_LD64_GOTOFF_LO15             310  /* GOT-rel. off. LD/ST imm. 14:3.  */
#define R_AARCH64_ADR_GOT_PAGE                 311  /* P-page-rel. GOT off. ADRP 32:12.  */
#define R_AARCH64_LD64_GOT_LO12_NC             312  /* Dir. GOT off. LD/ST imm. 11:3.  */
#define R_AARCH64_LD64_GOTPAGE_LO15            313  /* GOT-page-rel. GOT off. LD/ST 14:3 */
#define R_AARCH64_TLSGD_ADR_PREL21             512  /* PC-relative ADR imm. 20:0.  */
#define R_AARCH64_TLSGD_ADR_PAGE21             513  /* page-rel. ADRP imm. 32:12.  */
#define R_AARCH64_TLSGD_ADD_LO12_NC            514  /* direct ADD imm. from 11:0.  */
#define R_AARCH64_TLSGD_MOVW_G1                515  /* GOT-rel. MOV{N,Z} 31:16.  */
#define R_AARCH64_TLSGD_MOVW_G0_NC             516  /* GOT-rel. MOVK imm. 15:0.  */
#define R_AARCH64_TLSLD_ADR_PREL21             517  /* Like 512; local dynamic model.  */
#define R_AARCH64_TLSLD_ADR_PAGE21             518  /* Like 513; local dynamic model.  */
#define R_AARCH64_TLSLD_ADD_LO12_NC            519  /* Like 514; local dynamic model.  */
#define R_AARCH64_TLSLD_MOVW_G1                520  /* Like 515; local dynamic model.  */
#define R_AARCH64_TLSLD_MOVW_G0_NC             521  /* Like 516; local dynamic model.  */
#define R_AARCH64_TLSLD_LD_PREL19              522  /* TLS PC-rel. load imm. 20:2.  */
#define R_AARCH64_TLSLD_MOVW_DTPREL_G2         523  /* TLS DTP-rel. MOV{N,Z} 47:32.  */
#define R_AARCH64_TLSLD_MOVW_DTPREL_G1         524  /* TLS DTP-rel. MOV{N,Z} 31:16.  */
#define R_AARCH64_TLSLD_MOVW_DTPREL_G1_NC      525  /* Likewise; MOVK; no check.  */
#define R_AARCH64_TLSLD_MOVW_DTPREL_G0         526  /* TLS DTP-rel. MOV{N,Z} 15:0.  */
#define R_AARCH64_TLSLD_MOVW_DTPREL_G0_NC      527  /* Likewise; MOVK; no check.  */
#define R_AARCH64_TLSLD_ADD_DTPREL_HI12        528  /* DTP-rel. ADD imm. from 23:12. */
#define R_AARCH64_TLSLD_ADD_DTPREL_LO12        529  /* DTP-rel. ADD imm. from 11:0.  */
#define R_AARCH64_TLSLD_ADD_DTPREL_LO12_NC     530  /* Likewise; no ovfl. check.  */
#define R_AARCH64_TLSLD_LDST8_DTPREL_LO12      531  /* DTP-rel. LD/ST imm. 11:0.  */
#define R_AARCH64_TLSLD_LDST8_DTPREL_LO12_NC   532  /* Likewise; no check.  */
#define R_AARCH64_TLSLD_LDST16_DTPREL_LO12     533  /* DTP-rel. LD/ST imm. 11:1.  */
#define R_AARCH64_TLSLD_LDST16_DTPREL_LO12_NC  534  /* Likewise; no check.  */
#define R_AARCH64_TLSLD_LDST32_DTPREL_LO12     535  /* DTP-rel. LD/ST imm. 11:2.  */
#define R_AARCH64_TLSLD_LDST32_DTPREL_LO12_NC  536  /* Likewise; no check.  */
#define R_AARCH64_TLSLD_LDST64_DTPREL_LO12     537  /* DTP-rel. LD/ST imm. 11:3.  */
#define R_AARCH64_TLSLD_LDST64_DTPREL_LO12_NC  538  /* Likewise; no check.  */
#define R_AARCH64_TLSIE_MOVW_GOTTPREL_G1       539  /* GOT-rel. MOV{N,Z} 31:16.  */
#define R_AARCH64_TLSIE_MOVW_GOTTPREL_G0_NC    540  /* GOT-rel. MOVK 15:0.  */
#define R_AARCH64_TLSIE_ADR_GOTTPREL_PAGE21    541  /* Page-rel. ADRP 32:12.  */
#define R_AARCH64_TLSIE_LD64_GOTTPREL_LO12_NC  542  /* Direct LD off. 11:3.  */
#define R_AARCH64_TLSIE_LD_GOTTPREL_PREL19     543  /* PC-rel. load imm. 20:2.  */
#define R_AARCH64_TLSLE_MOVW_TPREL_G2          544  /* TLS TP-rel. MOV{N,Z} 47:32.  */
#define R_AARCH64_TLSLE_MOVW_TPREL_G1          545  /* TLS TP-rel. MOV{N,Z} 31:16.  */
#define R_AARCH64_TLSLE_MOVW_TPREL_G1_NC       546  /* Likewise; MOVK; no check.  */
#define R_AARCH64_TLSLE_MOVW_TPREL_G0          547  /* TLS TP-rel. MOV{N,Z} 15:0.  */
#define R_AARCH64_TLSLE_MOVW_TPREL_G0_NC       548  /* Likewise; MOVK; no check.  */
#define R_AARCH64_TLSLE_ADD_TPREL_HI12         549  /* TP-rel. ADD imm. 23:12.  */
#define R_AARCH64_TLSLE_ADD_TPREL_LO12         550  /* TP-rel. ADD imm. 11:0.  */
#define R_AARCH64_TLSLE_ADD_TPREL_LO12_NC      551  /* Likewise; no ovfl. check.  */
#define R_AARCH64_TLSLE_LDST8_TPREL_LO12       552  /* TP-rel. LD/ST off. 11:0.  */
#define R_AARCH64_TLSLE_LDST8_TPREL_LO12_NC    553  /* Likewise; no ovfl. check. */
#define R_AARCH64_TLSLE_LDST16_TPREL_LO12      554  /* TP-rel. LD/ST off. 11:1.  */
#define R_AARCH64_TLSLE_LDST16_TPREL_LO12_NC   555  /* Likewise; no check.  */
#define R_AARCH64_TLSLE_LDST32_TPREL_LO12      556  /* TP-rel. LD/ST off. 11:2.  */
#define R_AARCH64_TLSLE_LDST32_TPREL_LO12_NC   557  /* Likewise; no check.  */
#define R_AARCH64_TLSLE_LDST64_TPREL_LO12      558  /* TP-rel. LD/ST off. 11:3.  */
#define R_AARCH64_TLSLE_LDST64_TPREL_LO12_NC   559  /* Likewise; no check.  */
#define R_AARCH64_TLSDESC_LD_PREL19            560  /* PC-rel. load immediate 20:2.  */
#define R_AARCH64_TLSDESC_ADR_PREL21           561  /* PC-rel. ADR immediate 20:0.  */
#define R_AARCH64_TLSDESC_ADR_PAGE21           562  /* Page-rel. ADRP imm. 32:12.  */
#define R_AARCH64_TLSDESC_LD64_LO12            563  /* Direct LD off. from 11:3.  */
#define R_AARCH64_TLSDESC_ADD_LO12             564  /* Direct ADD imm. from 11:0.  */
#define R_AARCH64_TLSDESC_OFF_G1               565  /* GOT-rel. MOV{N,Z} imm. 31:16.  */
#define R_AARCH64_TLSDESC_OFF_G0_NC            566  /* GOT-rel. MOVK imm. 15:0; no ck.  */
#define R_AARCH64_TLSDESC_LDR                  567  /* Relax LDR.  */
#define R_AARCH64_TLSDESC_ADD                  568  /* Relax ADD.  */
#define R_AARCH64_TLSDESC_CALL                 569  /* Relax BLR.  */
#define R_AARCH64_TLSLE_LDST128_TPREL_LO12     570  /* TP-rel. LD/ST off. 11:4.  */
#define R_AARCH64_TLSLE_LDST128_TPREL_LO12_NC  571  /* Likewise; no check.  */
#define R_AARCH64_TLSLD_LDST128_DTPREL_LO12    572  /* DTP-rel. LD/ST imm. 11:4. */
#define R_AARCH64_TLSLD_LDST128_DTPREL_LO12_NC 573  /* Likewise; no check.  */
#define R_AARCH64_COPY                         1024 /* Copy symbol at runtime.  */
#define R_AARCH64_GLOB_DAT                     1025 /* Create GOT entry.  */
#define R_AARCH64_JUMP_SLOT                    1026 /* Create PLT entry.  */
#define R_AARCH64_RELATIVE                     1027 /* Adjust by program base.  */
#define R_AARCH64_TLS_DTPMOD                   1028 /* Module number, 64 bit.  */
#define R_AARCH64_TLS_DTPREL                   1029 /* Module-relative offset, 64 bit.  */
#define R_AARCH64_TLS_TPREL                    1030 /* TP-relative offset, 64 bit.  */
#define R_AARCH64_TLSDESC                      1031 /* TLS Descriptor.  */
#define R_AARCH64_IRELATIVE                    1032 /* STT_GNU_IFUNC relocation.  */

/* 5.1 Program Header */

#define PT_AARCH64_ARCHEXT  0x70000000  /* Reserved for architecture compatibility information */
#define PT_AARCH64_UNWIND   0x70000001  /* Reserved for exception unwinding tables */

#define EF_FLAG             0

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef unsigned long elf_greg_t;
struct user_pt_regs
{
  uint64_t regs[31];
  uint64_t sp;
  uint64_t pc;
  uint64_t pstate;
};

#define ELF_NGREG (sizeof(struct user_pt_regs) / sizeof(elf_greg_t))
typedef elf_greg_t elf_gregset_t[ELF_NGREG];

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __ARCH_ARM64_INCLUDE_ELF_H */
