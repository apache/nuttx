/****************************************************************************
 * arch/sim/include/elf.h
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

#ifndef __ARCH_SIM_INCLUDE_ELF_H
#define __ARCH_SIM_INCLUDE_ELF_H

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* 4.3.1 ELF Identification.  Should have:
 *
 * e_machine         = EM_386
 * e_ident[EI_CLASS] = ELFCLASS32
 * e_ident[EI_DATA]  = ELFDATA2LSB (little endian) or
 *                     ELFDATA2MSB (big endian)
 */

#ifdef CONFIG_SIM_M32
#  define EM_ARCH                EM_386
#  define ELF_CLASS              ELFCLASS32
#else
#  define EM_ARCH                EM_X86_64
#  define ELF_CLASS              ELFCLASS32
#endif

#ifdef CONFIG_ENDIAN_BIG
#  define ELF_DATA               ELFDATA2MSB
#else
#  define ELF_DATA               ELFDATA2LSB
#endif

#define EF_FLAG                  0

#endif /* __ARCH_SIM_INCLUDE_ELF_H */
