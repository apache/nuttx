/****************************************************************************
 * arch/xtensa/include/elf.h
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

#ifndef __ARCH_XTENSA_INCLUDE_ELF_H
#define __ARCH_XTENSA_INCLUDE_ELF_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Relocation codes */

#define R_XTENSA_NONE       0
#define R_XTENSA_32         1
#define R_XTENSA_ASM_EXPAND 11
#define R_XTENSA_SLOT0_OP   20

/* elf header of xtensa core dump */

#define EM_ARCH   EM_XTENSA
#define EF_FLAG   0

/* register set to dump status */

typedef uint32_t xtensa_elf_greg_t;
typedef struct
{
    xtensa_elf_greg_t pc;
    xtensa_elf_greg_t ps;
    xtensa_elf_greg_t lbeg;
    xtensa_elf_greg_t lend;
    xtensa_elf_greg_t lcount;
    xtensa_elf_greg_t sar;
    xtensa_elf_greg_t windowstart;
    xtensa_elf_greg_t windowbase;
    xtensa_elf_greg_t threadptr;
    xtensa_elf_greg_t reserved[7 + 48];
    xtensa_elf_greg_t ar[64];
}
__attribute__((packed)) xtensa_gregset_t;

#define XTENSA_ELF_NGREG (sizeof(xtensa_gregset_t) / sizeof(xtensa_elf_greg_t))

typedef unsigned long elf_gregset_t[XTENSA_ELF_NGREG];

#endif /* __ARCH_XTENSA_INCLUDE_ELF_H */
