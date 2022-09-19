/****************************************************************************
 * arch/risc-v/src/common/riscv_common_memorymap.h
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

#ifndef __ARCH_RISC_V_SRC_COMMON_RISCV_COMMON_MEMORYMAP_H
#define __ARCH_RISC_V_SRC_COMMON_RISCV_COMMON_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _START_TEXT  _stext
#define _END_TEXT    _etext
#define _START_BSS   _sbss
#define _END_BSS     _ebss
#define _DATA_INIT   _eronly
#define _START_DATA  _sdata
#define _END_DATA    _edata
#define _START_TDATA _stdata
#define _END_TDATA   _etdata
#define _START_TBSS  _stbss
#define _END_TBSS    _etbss

/****************************************************************************
 * Public Types
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifndef __ASSEMBLY__
EXTERN uintptr_t g_idle_topstack;

/* Address of per-cpu idle stack base */

EXTERN const uint8_t * const g_cpu_basestack[CONFIG_SMP_NCPUS];

/* Address of the saved user stack pointer */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
EXTERN uint8_t g_intstackalloc[]; /* Allocated stack base */
EXTERN uint8_t g_intstacktop[];   /* Initial top of interrupt stack */
#endif

/* These symbols are setup by the linker script. */

EXTERN uint8_t _stext[];           /* Start of .text */
EXTERN uint8_t _etext[];           /* End_1 of .text + .rodata */
EXTERN const uint8_t _eronly[];    /* End+1 of read only section (.text + .rodata) */
EXTERN uint8_t _sdata[];           /* Start of .data */
EXTERN uint8_t _edata[];           /* End+1 of .data */
EXTERN uint8_t _sbss[];            /* Start of .bss */
EXTERN uint8_t _ebss[];            /* End+1 of .bss */
EXTERN uint8_t _stdata[];          /* Start of .tdata */
EXTERN uint8_t _etdata[];          /* End+1 of .tdata */
EXTERN uint8_t _stbss[];           /* Start of .tbss */
EXTERN uint8_t _etbss[];           /* End+1 of .tbss */

#endif /* __ASSEMBLY__ */

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ARCH_RISC_V_SRC_COMMON_RISCV_COMMON_MEMORYMAP_H */
