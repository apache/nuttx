/****************************************************************************
 * arch/xtensa/include/elf.h
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

#endif /* __ARCH_XTENSA_INCLUDE_ELF_H */
