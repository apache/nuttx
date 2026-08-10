/****************************************************************************
 * arch/mips/include/elf.h
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

#ifndef __ARCH_MIPS_INCLUDE_ELF_H
#define __ARCH_MIPS_INCLUDE_ELF_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define R_MIPS_NONE          0
#define R_MIPS_32            2
#define R_MIPS_26            4
#define R_MIPS_HI16          5
#define R_MIPS_LO16          6

#define MIPS_HI16_COUNT      10
#define ARCH_ELFDATA         1

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct arch_elfdata_s
{
  struct hi_rel_s
  {
    Elf_Addr ahi;
    Elf32_Addr *p;
  }
  hi[MIPS_HI16_COUNT];
  int pos;

  Elf_Addr ahi;         /* The newest ahi */
};

typedef struct arch_elfdata_s arch_elfdata_t;

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_MIPS_INCLUDE_ELF_H */
