/****************************************************************************
 * arch/sparc/include/elf.h
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

#ifndef __ARCH_SPARC_INCLUDE_ELF_H
#define __ARCH_SPARC_INCLUDE_ELF_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Relocation codes */

#define R_SPARC_NONE		0
#define R_SPARC_8		1
#define R_SPARC_16		2
#define R_SPARC_32		3
#define R_SPARC_DISP8		4
#define R_SPARC_DISP16		5
#define R_SPARC_DISP32		6
#define R_SPARC_WDISP30		7
#define R_SPARC_WDISP22		8
#define R_SPARC_HI22		9
#define R_SPARC_22		10
#define R_SPARC_13		11
#define R_SPARC_LO10		12
#define R_SPARC_GOT10		13
#define R_SPARC_GOT13		14
#define R_SPARC_GOT22		15
#define R_SPARC_PC10		16
#define R_SPARC_PC22		17
#define R_SPARC_WPLT30		18
#define R_SPARC_COPY		19
#define R_SPARC_GLOB_DAT	20
#define R_SPARC_JMP_SLOT	21
#define R_SPARC_RELATIVE	22
#define R_SPARC_UA32		23
#define R_SPARC_PLT32		24
#define R_SPARC_HIPLT22		25
#define R_SPARC_LOPLT10		26
#define R_SPARC_PCPLT32		27
#define R_SPARC_PCPLT22		28
#define R_SPARC_PCPLT10		29
#define R_SPARC_10		30
#define R_SPARC_11		31
#define R_SPARC_64		32
#define R_SPARC_OLO10		33
#define R_SPARC_HH22		34
#define R_SPARC_HM10		35
#define R_SPARC_LM22		36
#define R_SPARC_PC_HH22		37
#define R_SPARC_PC_HM10		38
#define R_SPARC_PC_LM22		39
#define R_SPARC_WDISP16		40
#define R_SPARC_WDISP19		41
#define R_SPARC_GLOB_JMP	42
#define R_SPARC_7		43
#define R_SPARC_5		44
#define R_SPARC_6		45

#define OPCODE_NOP          0x01000000                          /*  nop                                 */
#define OPCODE_CALL         0x40000000                          /*  call ?; add PC-rel word address     */
#define OPCODE_SETHI_G1     0x03000000                          /*  sethi ?, %g1; add value>>10         */
#define OPCODE_JMP_G1       0x81c06000                          /*  jmp %g1+?; add lo 10 bits of value  */
#define OPCODE_SAVE_SP      0x9de3bfa8                          /*  save %sp, -(16+6)*4, %sp            */
#define OPCODE_BA           0x30800000                          /*  b,a ?; add PC-rel word address      */

#endif /* __ARCH_SPARC_INCLUDE_ELF_H */
