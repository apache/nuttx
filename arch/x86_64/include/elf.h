/****************************************************************************
 * arch/x86_64/include/elf.h
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

#ifndef __ARCH_X86_64_INCLUDE_ELF_H
#define __ARCH_X86_64_INCLUDE_ELF_H

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define R_X86_64_NONE           0
#define R_X86_64_64             1
#define R_X86_64_PC32           2
#define R_X86_64_PLT32          4
#define R_X86_64_32             10
#define R_X86_64_32S            11
#define R_X86_64_PC64           24
#define R_X86_64_REX_GOTPCRELX  42

/* 4.3.1 ELF Identification.  Should have:
 *
 * e_machine         = EM_X86_64
 */

#define EM_ARCH      EM_X86_64

#define EF_FLAG      0

/* Segment register layout in coredumps. */

struct user_regs_struct
{
  uint64_t  r15;
  uint64_t  r14;
  uint64_t  r13;
  uint64_t  r12;
  uint64_t  bp;
  uint64_t  bx;
  uint64_t  r11;
  uint64_t  r10;
  uint64_t  r9;
  uint64_t  r8;
  uint64_t  ax;
  uint64_t  cx;
  uint64_t  dx;
  uint64_t  si;
  uint64_t  di;
  uint64_t  orig_ax;
  uint64_t  ip;
  uint64_t  cs;
  uint64_t  flags;
  uint64_t  sp;
  uint64_t  ss;
  uint64_t  fs_base;
  uint64_t  gs_base;
  uint64_t  ds;
  uint64_t  es;
  uint64_t  fs;
  uint64_t  gs;
};

typedef uint64_t elf_greg_t;

#define ELF_NGREG (sizeof(struct user_regs_struct) / sizeof(elf_greg_t))
typedef elf_greg_t elf_gregset_t[ELF_NGREG];

#endif /* __ARCH_X86_64_INCLUDE_ELF_H */
