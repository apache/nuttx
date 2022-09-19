/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_linker.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_RV32M1_LINKER_H
#define __ARCH_RISCV_SRC_RV32M1_RV32M1_LINKER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RV32M1_ITCM
#  define SECTION_ITCM     ".itcm"    /* System ITCM */
#  define SECTION_UITCM    ".uitcm"   /* User ITCM */
#endif

#ifndef __ASSEMBLY__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LOCATE_ITCM is a recommendation rather than a forced command
 * to place codes in Section ITCM. It is effective when RV32M1
 * has ITCM.
 */

#ifdef CONFIG_RV32M1_ITCM
#  define LOCATE_ITCM   locate_code(SECTION_ITCM)    /* System ITCM */
#  define LOCATE_UITCM  locate_code(SECTION_UITCM)   /* User ITCM */
#else
#  define LOCATE_ITCM  
#  define LOCATE_UITCM
#endif

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_RV32M1_ITCM
  EXTERN uint8_t _slitcm[];        /* Start of ITCM LMA */
  EXTERN uint8_t _svitcm[];        /* Start of ITCM VMA */
  EXTERN uint8_t _evitcm[];        /* End+1 of ITCM VMA */
  EXTERN uint8_t _suvitcm[];       /* Start of User ITCM VMA */
  EXTERN uint8_t _euvitcm[];       /* End+1 of User ITCM VMA */
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_RV32M1_RV32M1_LINKER_H */
